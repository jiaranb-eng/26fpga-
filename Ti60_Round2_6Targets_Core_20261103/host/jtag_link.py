"""自定义 JTAG 整包传输；复用 PyFtdi，不依赖 VIO 或官方调试界面。"""

import time
from types import SimpleNamespace

SCAN_BYTES = 32
SIGNATURE = b'JTG1'
DEFAULT_URL = 'ftdi://0x0403:0x6014:0:ff/1'
TI60_IDCODE = 0x10660A79


def pack_scan(frame):
    """保留 UART 协议 CRC/序号，扫描层只增加边界和固件签名。"""
    if not 8 <= len(frame) <= 24:
        raise ValueError('JTAG 命令帧必须为 8–24 字节')
    return b'TJ\x01' + bytes([len(frame)]) + frame.ljust(24, b'\0') + SIGNATURE


def unpack_scan(data):
    """拒绝未下载配套固件、错误链路或响应长度损坏。"""
    if len(data) != SCAN_BYTES or data[:2] != b'RJ' or data[28:] != SIGNATURE:
        raise ValueError('未检测到 JTAG 控制固件签名；请下载配套 bitstream')
    flags, length = data[2:4]
    if flags & 0xF0 or length > 24:
        raise ValueError('JTAG 返回格式错误')
    frame = data[28-length:28] if length else b''
    return flags, frame


class FtdiScan:
    """只支持单颗 Ti60、FT232H、1 MHz；构造器才会打开 USB。"""

    def __init__(self, url):
        # 使用随 Efinity 安装的开源库，不复制或修改厂商代码。
        from pyftdi.bits import BitSequence
        from pyftdi.jtag import JtagEngine
        if not url.startswith('ftdi://0x0403:0x6014:') or not url.endswith('/1'):
            raise ValueError('目前仅验证 FT232H 接口1；不自动尝试其他USB设备')
        self.bits = BitSequence
        self.engine = JtagEngine(trst=False, frequency=1_000_000)
        try:
            # 适配 Efinity 随附 PyFtdi 的 init 参数；其他 GPIO 保持输入。
            self.engine.configure(url, init=SimpleNamespace(val='', dir=''))
            self.engine.reset()
            self.engine.go_idle()
            self._instruction(0x03)
            self.engine.change_state('shift_dr')
            identity = int(self.engine.shift_and_update_register(BitSequence(0, length=64)))
            self.engine.go_idle()
            self.engine.sync()
            if identity != TI60_IDCODE:
                raise ValueError(f'要求单颗 Ti60，实际 IDCODE/扫描链=0x{identity:016X}')
            self._instruction(0x08)
            # 提供复位释放和同步时钟，不提交命令。
            self.exchange(bytes(SCAN_BYTES))
        except Exception:
            self.engine.close()
            raise

    def _instruction(self, code):
        self.engine.change_state('shift_ir')
        self.engine.shift_and_update_register(self.bits(code, length=5))
        self.engine.go_idle()
        self.engine.sync()

    def exchange(self, data):
        if len(data) != SCAN_BYTES:
            raise ValueError('每次扫描必须恰好256位')
        self.engine.change_state('shift_dr')
        incoming = self.engine.shift_and_update_register(
            self.bits(int.from_bytes(data, 'little'), length=256))
        self.engine.go_idle()
        self.engine.sync()  # 使Update-DR真正执行，不能停在尚未提交的边沿
        return int(incoming).to_bytes(SCAN_BYTES, 'little')

    def close(self):
        # 不触发配置复位，不写Flash；关闭后烧录器可被Efinity接管。
        self.engine.close()


class JtagLink:
    """停止等待；不重试可能已生效的写命令，异常后必须重新连接。"""

    def __init__(self, url=DEFAULT_URL, transport=None, timeout=1.0):
        self.transport = transport if transport is not None else FtdiScan(url)
        self.timeout = timeout
        self.sequence = 0
        self.closed = False
        self.failed = False
        try:
            flags, _ = unpack_scan(self.transport.exchange(bytes(SCAN_BYTES)))
            if not flags & 8 or flags & 1:
                raise RuntimeError('JTAG 邮箱未就绪或上次命令仍忙，请稍后重新连接')
        except Exception:
            self.close()
            raise

    def command(self, command, payload=b''):
        # 延迟导入避免 GUI 导入时访问USB或形成顶层循环依赖。
        from video_console import Decoder, STATUS_NAMES, encode
        if self.closed or self.failed:
            raise RuntimeError('JTAG连接已失效，请重新连接；未自动重发命令')
        self.sequence = (self.sequence + 1) & 255
        request = encode(self.sequence, command, payload)
        try:
            flags, _ = unpack_scan(self.transport.exchange(bytes(SCAN_BYTES)))
            if flags & 1 or not flags & 8:
                raise RuntimeError('JTAG邮箱忙或复位中；未发送命令')
            # 这次扫描返回的是提交前状态，必须另行轮询完成。
            flags, _ = unpack_scan(self.transport.exchange(pack_scan(request)))
            if flags & 1 or not flags & 8:
                raise RuntimeError('提交时邮箱未就绪，执行状态不确定')
            deadline = time.monotonic() + self.timeout
            while time.monotonic() < deadline:
                flags, frame = unpack_scan(self.transport.exchange(bytes(SCAN_BYTES)))
                if not flags & 8:
                    raise RuntimeError('FPGA控制端复位，请重新连接')
                if flags & 1:
                    time.sleep(0.002)
                    continue
                if flags & 4:
                    raise TimeoutError('FPGA协议响应超时；命令未自动重发')
                if flags & 2:
                    decoded = Decoder().feed(frame)
                    if len(decoded) != 1:
                        raise ValueError('JTAG响应CRC或包格式错误；执行状态不确定')
                    sequence, received_command, response = decoded[0]
                    if (sequence, received_command) == (self.sequence, command):
                        if not response:
                            raise ValueError('空响应')
                        if response[0]:
                            # 硬件明确拒绝，连接仍可继续使用。
                            return self._rejected(response, STATUS_NAMES)
                        return response
                time.sleep(0.002)
            raise TimeoutError('JTAG等待超时；执行状态不确定，请重连后查询，勿盲目重复操作')
        except CommandRejected:
            raise
        except Exception:
            self.failed = True
            raise

    @staticmethod
    def _rejected(response, names):
        raise CommandRejected(names.get(response[0], f'错误 {response[0]}'))

    def close(self):
        if not self.closed:
            self.closed = True
            self.transport.close()


class CommandRejected(ValueError):
    """协议返回的明确错误，区别于失联后的未知执行状态。"""
