"""Ti60 JTAG 控制台：自定义 User TAP 邮箱，只传控制，不传视频。"""

import argparse
import queue
import threading
import time
import tkinter as tk
import webbrowser
from dataclasses import dataclass
from tkinter import messagebox, ttk

from jtag_link import DEFAULT_URL, JtagLink


STATUS_NAMES = {0: '成功', 1: '协议版本错误', 2: '包类型错误', 3: '长度错误',
                4: '未知命令', 5: '超出范围', 6: '忙或尚无完整帧'}
MODE_NAMES = ['DVP 四宫格', 'MIPI Canny', 'MIPI 彩色', 'Canny 图形标注', 'MIPI 背景帧差', '原图 / Canny 同帧对照', 'MIPI 彩色叠加 Canny 黄色边缘']
# 只改变菜单顺序；协议模式编号仍由 MODE_NAMES 的下标确定。
MODE_DISPLAY_ORDER = (3, 1, 6, 5, 2, 4, 0)


def crc8(data):
    """与 FPGA 一致：多项式 0x07，初值 0，不反射，不做末尾异或。"""
    value = 0
    for octet in data:
        value ^= octet
        for _ in range(8):
            value = ((value << 1) ^ (7 if value & 0x80 else 0)) & 255
    return value


def encode(sequence, command, payload=b''):
    if len(payload) > 16:
        raise ValueError('载荷不能超过 16 字节')
    body = bytes([1, sequence, 1, command, len(payload)]) + payload
    return b'\xa5\x5a' + body + bytes([crc8(body)])


class Decoder:
    """增量解帧；支持拆包、粘包、噪声和 CRC 错误后重新同步。"""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(b'\xa5\x5a')
            if start < 0:
                self.buffer[:] = b'\xa5' if self.buffer.endswith(b'\xa5') else b''
                break
            del self.buffer[:start]
            if len(self.buffer) < 7:
                break
            length = self.buffer[6]
            if length > 16:
                del self.buffer[0]
                continue
            total = length + 8
            if len(self.buffer) < total:
                break
            frame = bytes(self.buffer[:total])
            if frame[2] != 1 or frame[4] != 2 or crc8(frame[2:-1]) != frame[-1]:
                del self.buffer[0]
                continue
            frames.append((frame[3], frame[5], frame[7:-1]))
            del self.buffer[:total]
        return frames


@dataclass(frozen=True)
class Status:
    threshold: int
    display: int
    history: int
    count: int
    age_a: int
    age_b: int
    faults: int

    @classmethod
    def parse(cls, payload):
        if len(payload) != 9:
            raise ValueError('固件不支持历史状态；请确认使用配套候选固件')
        return cls(payload[1] | payload[2] << 8, *payload[3:])


@dataclass(frozen=True)
class BackgroundStatus:
    threshold: int
    flags: int
    count: int
    epoch: int
    frames: int
    cycles: int

    @classmethod
    def parse(cls, payload):
        if len(payload) != 16:
            raise ValueError('需要本次背景帧差配套固件')
        return cls(*payload[1:4], *(int.from_bytes(payload[index:index + 4], 'little')
                                    for index in (4, 8, 12)))


class VideoConsole:
    def __init__(self, root):
        self.root = root
        self.root.title('Ti60 · 双摄控制台 · 彩色叠加版 20261103')
        self.root.geometry('900x860')
        self.link = None
        self.status = None
        self.background_status = None
        self.background_sample = None
        self.background_wait_epoch = None
        self.background_deadline = None
        self.background_notice = ''
        self.background_request_active = False
        self.busy = False
        self.playing = False
        self.play_timer = None
        self.closed = False
        self.events = queue.Queue()
        self.port_name = tk.StringVar(value=DEFAULT_URL)
        self.mode = tk.StringVar(value=MODE_NAMES[3])
        self.canny_low = tk.IntVar(value=10)
        self.canny_high = tk.IntVar(value=20)
        self.canny_status = None
        self.canny_summary = tk.StringVar(value='默认低10 / 高20；设置成对提交，从后续完整帧生效。')
        self.threshold = tk.IntVar(value=64)
        self.background_threshold = tk.IntVar(value=12)
        self.background_summary = tk.StringVar(value='先连接，再点击“更新背景”；将当前固定场景作为新背景。')
        self.age_a = tk.IntVar(value=0)
        self.age_b = tk.IntVar(value=0)
        self.rate = tk.StringVar(value='5')
        self.summary = tk.StringVar(value='未连接；JTAG仅控制，HDMI输出视频。需要配套JTAG固件。')
        self._build()
        self.root.after(50, self._events)
        self.root.after(1000, self._poll)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

    def _build(self):
        outer = ttk.Frame(self.root, padding=18)
        outer.pack(fill='both', expand=True)
        ttk.Label(outer, text='Ti60  /  双摄、历史画面与背景更新', font=('Microsoft YaHei UI', 18)).pack(anchor='w')
        connection = ttk.Frame(outer)
        connection.pack(fill='x', pady=15)
        self.port_box = ttk.Combobox(connection, textvariable=self.port_name, width=36,
                                     values=[DEFAULT_URL])
        self.port_box.pack(side='left')
        ttk.Button(connection, text='连接', command=self.connect).pack(side='left')
        ttk.Button(connection, text='断开 / 释放烧录器', command=self.disconnect).pack(side='left', padx=8)
        ttk.Label(connection, text='JTAG · 1 MHz').pack(side='left')
        view = ttk.LabelFrame(outer, text='实时画面与边缘处理', padding=12)
        view.pack(fill='x')
        self.mode_box = ttk.Combobox(view, values=[MODE_NAMES[index] for index in MODE_DISPLAY_ORDER],
                                     textvariable=self.mode, state='readonly', width=34)
        self.mode_box.grid(row=0, column=0)
        ttk.Button(view, text='切换画面 / 返回实时', command=self.change_mode).grid(row=0, column=1, padx=8)
        self.overlay_button = ttk.Button(view, text='彩色叠加黄色边缘', command=self.show_mipi_overlay)
        self.overlay_button.grid(row=0, column=2, padx=(4, 0))
        ttk.Spinbox(view, from_=0, to=1020, textvariable=self.threshold, width=8).grid(row=1, column=0, pady=10)
        ttk.Button(view, text='设置 Sobel 阈值', command=self.set_threshold).grid(row=1, column=1)
        canny = ttk.LabelFrame(outer, text='Canny 高低阈值', padding=8)
        canny.pack(fill='x', pady=(8, 0))
        ttk.Label(canny, text='低').grid(row=0, column=0)
        ttk.Spinbox(canny, from_=0, to=2047, textvariable=self.canny_low, width=7).grid(row=0, column=1)
        ttk.Label(canny, text='高').grid(row=0, column=2)
        ttk.Spinbox(canny, from_=0, to=2047, textvariable=self.canny_high, width=7).grid(row=0, column=3)
        ttk.Button(canny, text='设置 Canny 阈值', command=self.set_canny_thresholds).grid(row=0, column=4, padx=6)
        ttk.Button(canny, text='恢复 10 / 20', command=lambda: self.set_canny_thresholds((10, 20))).grid(row=0, column=5)
        ttk.Button(canny, text='读取当前设置', command=self.get_canny_thresholds).grid(row=0, column=6, padx=6)
        ttk.Label(canny, textvariable=self.canny_summary).grid(row=1, column=0, columnspan=7, sticky='w')
        background = ttk.LabelFrame(outer, text='背景帧差 · 固定机位', padding=12)
        background.pack(fill='x', pady=(12, 0))
        self.background_button = ttk.Button(background, text='更新背景（当前画面）', command=self.relearn_background)
        self.background_button.grid(row=0, column=0, padx=4)
        ttk.Spinbox(background, from_=0, to=255, textvariable=self.background_threshold, width=8).grid(row=0, column=1, padx=8)
        ttk.Button(background, text='设置差分阈值', command=self.set_background_threshold).grid(row=0, column=2)
        ttk.Label(background, textvariable=self.background_summary, wraplength=750).grid(row=1, column=0, columnspan=3, sticky='w', pady=(8, 0))
        ttk.Label(background, text='自动切到背景帧差并返回实时；保持相机不动，移走不想纳入背景的目标。').grid(row=2, column=0, columnspan=3, sticky='w', pady=(6, 0))
        ttk.Button(background, text='打开 HDMI 实时预览', command=lambda: webbrowser.open('http://127.0.0.1:8766/')).grid(row=3, column=0, sticky='w', pady=(6, 0))
        ttk.Label(background, text='差分阈值 0–255；默认12，兼顾暗物体与碎片过滤。').grid(row=3, column=1, columnspan=2, sticky='w')
        history = ttk.LabelFrame(outer, text='冻结与历史帧（年龄 0 = 冻结时最新完整帧）', padding=12)
        history.pack(fill='x', pady=14)
        ttk.Button(history, text='冻结最新', command=lambda: self.history(1, 0, 0)).grid(row=0, column=0, padx=3)
        ttk.Button(history, text='返回实时', command=lambda: self.history(0, 0, 0)).grid(row=0, column=1, padx=3)
        ttk.Button(history, text='上一帧 · 更早', command=lambda: self.step(1)).grid(row=0, column=2, padx=3)
        ttk.Button(history, text='下一帧 · 更新', command=lambda: self.step(-1)).grid(row=0, column=3, padx=3)
        ttk.Label(history, text='A 帧年龄').grid(row=1, column=0, pady=12)
        self.age_a_box = ttk.Spinbox(history, from_=0, to=0, textvariable=self.age_a, width=8)
        self.age_a_box.grid(row=1, column=1)
        ttk.Label(history, text='B 帧年龄').grid(row=1, column=2)
        self.age_b_box = ttk.Spinbox(history, from_=0, to=0, textvariable=self.age_b, width=8)
        self.age_b_box.grid(row=1, column=3)
        ttk.Button(history, text='显示 A 全帧', command=self.show_a).grid(row=2, column=0)
        ttk.Button(history, text='A 上 / B 下对比', command=self.compare).grid(row=2, column=1)
        ttk.Button(history, text='从最早帧回放', command=self.play).grid(row=2, column=2)
        ttk.Button(history, text='暂停', command=self.pause).grid(row=2, column=3)
        ttk.Combobox(history, values=['1', '2', '5', '10'], textvariable=self.rate, state='readonly', width=6).grid(row=3, column=0, pady=10)
        ttk.Label(history, text='帧/秒；到最新帧自动暂停。对比保留坐标，不缩放两幅图。').grid(row=3, column=1, columnspan=3, sticky='w')
        ttk.Label(outer, textvariable=self.summary, wraplength=790).pack(anchor='w', pady=8)
        ttk.Label(outer, text='只连接原JTAG烧录器，不连接串口模块。需要WinUSB/libusbK兼容驱动。\n连接前关闭Efinity Programmer；重新烧录前先断开本控制台。ACK只表示命令接收。', foreground='#5e6270').pack(anchor='w')

    def _submit(self, work, callback=None):
        if self.busy:
            return False
        self.busy = True
        def run():
            try:
                self.events.put((True, work(), callback))
            except Exception as error:
                self.events.put((False, str(error), None))
        threading.Thread(target=run, daemon=True).start()
        return True

    def connect(self):
        if self.busy:
            return
        self.pause()
        self.status = None
        self.canny_status = None
        self._reset_background('连接后可更新背景。')
        port = self.port_name.get().strip()
        def work():
            if self.link:
                self.link.close()
            self.link = None
            candidate = JtagLink(port)
            try:
                status = Status.parse(candidate.command(0x32))
                canny = candidate.command(0x13)
            except Exception:
                candidate.close()
                raise
            self.link = candidate
            return status, canny
        def done(result):
            self._status(result[0])
            self._canny_result(result[1], True)
        self._submit(work, done)

    def disconnect(self):
        if self.busy:
            self.summary.set('等待当前命令结束后再断开。')
            return
        self.pause()
        if self.link:
            self.link.close()
        self.link = None
        self.status = None
        self._reset_background('已断开；重新连接后可更新背景。')
        self.summary.set('已断开，烧录器已释放。')

    def _reset_background(self, message):
        self.background_status = None
        self.background_sample = None
        self.background_wait_epoch = None
        self.background_deadline = None
        self.background_notice = ''
        self.background_request_active = False
        self.background_button.configure(state='normal', text='更新背景（当前画面）')
        self.background_summary.set(message)

    def _command(self, command, payload=b'', callback=None):
        if not self.link:
            self.summary.set('请先连接JTAG烧录器。')
            return False
        def work():
            self.link.command(command, payload)
            # ACK 表示接收成功；让显示帧边界和原模式邮箱有时间生效。
            time.sleep(0.06 if command != 0x30 else 0.25)
            return Status.parse(self.link.command(0x32))
        def done(status):
            self._status(status)
            if callback:
                callback()
        return self._submit(work, done)

    def _status(self, status):
        previous = self.status
        self.status = status
        # 周期查询只同步变化的设备值，不覆盖用户尚未提交的编辑。
        if previous is None or status.threshold != previous.threshold:
            self.threshold.set(status.threshold)
        if (previous is None or status.display != previous.display) and status.display < len(MODE_NAMES):
            self.mode.set(MODE_NAMES[status.display])
        if previous is None or status.age_a != previous.age_a:
            self.age_a.set(status.age_a)
        if previous is None or status.age_b != previous.age_b:
            self.age_b.set(status.age_b)
        maximum = max(0, status.count - 1)
        self.age_a_box.configure(to=maximum)
        self.age_b_box.configure(to=maximum)
        state = ['实时', '冻结 / 单帧', '上下对比'][status.history] if status.history < 3 else '未知'
        self.summary.set(f'{state} | 完整历史 {status.count} 帧 | A={status.age_a} B={status.age_b} | 故障位 0x{status.faults:02X}'
                         + (' | 正在回放' if self.playing else ''))

    def history(self, control, age_a, age_b, callback=None, keep_playing=False):
        if not keep_playing:
            self.pause()
        if control and (not self.status or not self.status.count):
            self.summary.set('尚无完整帧；请先在实时模式等待图像。')
            return False
        maximum = max(0, self.status.count - 1) if self.status else 0
        if not (0 <= age_a <= maximum and 0 <= age_b <= maximum):
            self.summary.set(f'年龄范围应为 0–{maximum}。')
            return False
        return self._command(0x31, bytes([control, age_a, age_b]), callback)

    def show_a(self):
        try:
            self.history(1, self.age_a.get(), 0)
        except tk.TclError:
            self.summary.set('请输入整数帧年龄。')

    def compare(self):
        try:
            self.history(2, self.age_a.get(), self.age_b.get())
        except tk.TclError:
            self.summary.set('请输入整数帧年龄。')

    def step(self, direction):
        if self.status:
            age = min(max(0, self.status.age_a + direction), max(0, self.status.count - 1))
            self.history(1, age, 0)

    def change_mode(self):
        self.pause()
        self._command(0x30, bytes([MODE_NAMES.index(self.mode.get())]))

    def show_mipi_overlay(self):
        self.mode.set(MODE_NAMES[6])
        self.change_mode()

    def _canny_result(self, payload, force=False):
        if len(payload) != 5:
            raise ValueError('固件不支持成对Canny阈值，请使用本轮配套固件。')
        if payload[0] != 0:
            raise ValueError(f'Canny阈值命令失败，设备状态码 {payload[0]}。')
        current = (int.from_bytes(payload[1:3], 'little'), int.from_bytes(payload[3:5], 'little'))
        if force or current != self.canny_status:
            self.canny_low.set(current[0])
            self.canny_high.set(current[1])
        self.canny_status = current
        self.canny_summary.set(f'设备设置：低 {current[0]} / 高 {current[1]}；后续完整帧使用，冻结画面不重算。')

    def get_canny_thresholds(self):
        if self.link:
            self._submit(lambda: self.link.command(0x13), lambda p: self._canny_result(p, True))
        else:
            self.summary.set('请先连接JTAG烧录器。')

    def set_canny_thresholds(self, values=None):
        try:
            low, high = values if values is not None else (self.canny_low.get(), self.canny_high.get())
            if not 0 <= low <= high <= 2047:
                raise ValueError('需要 0 ≤ Canny低阈值 ≤ 高阈值 ≤ 2047。')
            if not self.link:
                self.summary.set('请先连接JTAG烧录器。')
                return False
            payload = low.to_bytes(2, 'little') + high.to_bytes(2, 'little')
            return self._submit(lambda: self.link.command(0x14, payload), lambda p: self._canny_result(p, True))
        except (ValueError, tk.TclError) as error:
            self.summary.set(str(error))
            return False

    def set_threshold(self):
        try:
            threshold = self.threshold.get()
            if not 0 <= threshold <= 1020:
                raise ValueError('阈值范围 0–1020')
            self._command(0x11, threshold.to_bytes(2, 'little'))
        except (ValueError, tk.TclError) as error:
            self.summary.set(str(error))

    def _background(self, status):
        previous = self.background_status
        self.background_status = status
        if previous is None or previous.threshold != status.threshold:
            self.background_threshold.set(status.threshold)
        now = time.monotonic()
        fps_text = ''
        if self.background_sample is not None:
            previous_time, previous_frames = self.background_sample
            elapsed = now - previous_time
            if elapsed > 0:
                fps = ((status.frames - previous_frames) & 0xffffffff) / elapsed
                fps_text = f' | 完成帧率 {fps:.1f} fps'
        self.background_sample = now, status.frames
        state = self.background_notice or ('等待完整新背景' if status.flags & 2 else ('背景已就绪' if status.flags & 1 else '尚未建立背景'))
        if self.background_wait_epoch is not None:
            if status.epoch != self.background_wait_epoch and status.flags & 1 and not status.flags & 2:
                self.background_wait_epoch = None
                self.background_deadline = None
                self.background_request_active = False
                self.background_button.configure(state='normal', text='更新背景（当前画面）')
                self.background_notice = '更新背景完成（重新采集完成）'
                state = self.background_notice
            elif self.background_deadline is not None and now >= self.background_deadline:
                self.background_wait_epoch = None
                self.background_deadline = None
                self.background_request_active = False
                self.background_button.configure(state='normal', text='更新背景（当前画面）')
                self.background_notice = '更新背景超时，尚未确认生效；请检查摄像头和状态后重试'
                state = self.background_notice
            else:
                state = '更新背景中，等待完整新帧生效'
        overflow = ' | 区域容量溢出' if status.flags & 4 else ''
        self.background_summary.set(f'{state} | 背景代数 {status.epoch} | 目标 {status.count}/8'
                                    f'{fps_text}{overflow}')

    def relearn_background(self):
        if not self.link:
            self.summary.set('请先连接JTAG烧录器。')
            return
        if self.busy or self.background_request_active:
            self.summary.set('已有命令或背景更新正在执行，请等待完成。')
            return
        self.pause()
        self.background_request_active = True
        self.background_notice = ''
        self.background_button.configure(state='disabled', text='正在更新背景…')
        self.background_summary.set('正在切换到实时背景帧差，随后采集当前场景…')
        def work():
            # 确认实时背景模式已经生效，不能仅靠固定延时或ACK推断。
            self.link.command(0x30, b'\x04')
            mode_deadline = time.monotonic() + 3
            while True:
                video = Status.parse(self.link.command(0x32))
                if video.display == 4 and video.history == 0:
                    break
                if time.monotonic() >= mode_deadline:
                    raise TimeoutError('未能切换到实时背景帧差，未提交更新请求')
                time.sleep(0.05)
            before = BackgroundStatus.parse(self.link.command(0x40))
            if before.flags & 2:
                raise RuntimeError('FPGA已有背景采集正在进行，请稍后再试')
            self.link.command(0x41)
            return before.epoch, video, BackgroundStatus.parse(self.link.command(0x40))
        def done(result):
            self.background_wait_epoch, video, status = result
            self.background_deadline = time.monotonic() + 8
            self._status(video)
            self.mode.set(MODE_NAMES[4])
            self._background(status)
        self._submit(work, done)

    def set_background_threshold(self):
        if not self.link:
            self.summary.set('请先连接JTAG烧录器。')
            return
        try:
            threshold = self.background_threshold.get()
            if not 0 <= threshold <= 255:
                raise ValueError('差分阈值范围 0–255')
        except (ValueError, tk.TclError) as error:
            self.summary.set(str(error))
            return
        self._submit(lambda: BackgroundStatus.parse(self.link.command(0x42, bytes([threshold]))), self._background)

    def pause(self):
        self.playing = False
        if self.play_timer is not None:
            self.root.after_cancel(self.play_timer)
            self.play_timer = None

    def play(self):
        if self.busy or not self.status or not self.status.count:
            return
        self.pause()
        self.playing = True
        self.history(1, self.status.count - 1, 0, self._schedule, keep_playing=True)

    def _schedule(self):
        if self.playing:
            self.play_timer = self.root.after(max(1, round(1000 / int(self.rate.get())) - 60), self._next)

    def _next(self):
        self.play_timer = None
        if not self.playing or not self.status:
            return
        if self.status.age_a == 0:
            self.pause()
            self._status(self.status)
            return
        if not self.history(1, self.status.age_a - 1, 0, self._schedule, keep_playing=True):
            self._schedule()

    def _poll(self):
        if self.link and not self.busy and not self.playing:
            def work():
                return (Status.parse(self.link.command(0x32)),
                        BackgroundStatus.parse(self.link.command(0x40)),
                        self.link.command(0x13))
            def done(result):
                self._status(result[0])
                self._background(result[1])
                self._canny_result(result[2])
            self._submit(work, done)
        if not self.closed:
            self.root.after(250 if self.background_request_active else 1000, self._poll)

    def _events(self):
        while not self.events.empty():
            success, value, callback = self.events.get_nowait()
            self.busy = False
            if success and callback:
                callback(value)
            elif not success:
                self.pause()
                self.summary.set(f'失败：{value}')
                if self.background_request_active:
                    self.background_wait_epoch = None
                    self.background_deadline = None
                    self.background_request_active = False
                    self.background_button.configure(state='normal', text='更新背景（当前画面）')
                    self.background_notice = f'更新背景未确认完成：{value}'
                    self.background_summary.set(self.background_notice)
        if not self.closed:
            self.root.after(50, self._events)

    def close(self):
        # USB传输尚在工作线程时，不并发关闭底层句柄。
        if self.busy:
            self.pause()
            self.root.after(100, self.close)
            return
        self.closed = True
        self.pause()
        if self.link:
            self.link.close()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--smoke-test', action='store_true', help='创建界面并检查布局，不打开USB')
    args = parser.parse_args()
    root = tk.Tk()
    app = VideoConsole(root)
    if args.smoke_test:
        root.withdraw()
        root.update_idletasks()
        assert root.winfo_reqwidth() > 0
        app.close()
        print('PASS Tk GUI smoke test; no USB or serial port opened')
    else:
        root.mainloop()


if __name__ == '__main__':
    main()
