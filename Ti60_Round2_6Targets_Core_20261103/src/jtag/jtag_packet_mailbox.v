`timescale 1ns / 1ps

// JTAG整包邮箱：停等交握保持跨域总线，视频时钟不等待电脑。
// 官方User TAP提供扫描信号，本模块不实现或替代JTAG TAP。
module jtag_packet_mailbox #(
    parameter integer TIMEOUT_CYCLES = 2097152 // 超过原协议残包恢复时间
) (
    input wire clk_i,                         // 系统控制时钟
    input wire rst_n_i,                       // 两个交握域共同复位
    input wire tck_i,                         // User TAP测试时钟
    input wire capture_i,                     // 捕获返回邮箱
    input wire shift_i,                       // 移入移出一位
    input wire update_i,                      // 完成扫描后尝试提交
    input wire tap_reset_i,                   // 仅清除残留扫描长度
    input wire select_i,                      // 用户指令选中
    input wire tdi_i,                         // 扫描链串行输入
    output wire tdo_o,                        // 扫描链最低位输出
    output wire [7:0] rx_data_o,               // 原协议接收字节
    output wire rx_valid_o,                   // 连续有效接收节拍
    input wire [7:0] tx_data_i,                // 原协议响应字节
    input wire tx_valid_i,                    // 原协议响应有效
    output wire tx_ready_o                    // 邮箱响应收集许可
);

reg [255:0] reg_scan = 0;                     // TCK扫描快照，不直接执行控制
reg [8:0] cnt_bits = 0;                       // 饱和于257，拒绝过长扫描
reg [199:0] reg_request = 0;                  // 长度和192位包，忙期间保持
reg flag_request = 0;                        // TCK请求翻转位
reg flag_accepted = 0;                       // 区分初始空闲与命令完成
(* async_reg = "true" *) reg [1:0] reg_done_sync = 0; // 系统完成位进入TCK域
reg [1:0] reg_tck_release = 0;                // TCK域异步置位、同步解除复位
wire flag_busy;
wire flag_submit;
wire flag_tck_ready;
wire [7:0] scan_flags;

(* async_reg = "true" *) reg [1:0] reg_request_sync = 0; // 请求进入系统域
reg flag_seen = 0;                           // 已取走的请求翻转值
reg flag_done = 0;                           // 响应保持后发布的完成翻转值
reg flag_active = 0;                         // 系统端等待响应或超时
reg flag_error = 0;                          // 本次请求无协议响应
reg [191:0] reg_receive = 0;                  // 系统端逐字节移出请求
reg [4:0] cnt_receive = 0;                    // 尚未送出的字节数
reg [191:0] reg_response = 0;                 // 响应字节从高位移入
reg [4:0] cnt_response = 0;                   // 响应长度，最大24
reg flag_tx_previous = 0;                    // 检测原协议连续响应结束
reg [21:0] cnt_timeout = 0;                   // 系统端超时，不依赖TCK继续运行

assign flag_tck_ready = reg_tck_release[1];
assign flag_busy = flag_request != reg_done_sync[1];
assign flag_submit = select_i && update_i && flag_tck_ready && !flag_busy &&
                     cnt_bits == 9'd256 && reg_scan[23:0] == 24'h014a54 &&
                     reg_scan[31:24] >= 8'd8 && reg_scan[31:24] <= 8'd24 &&
                     reg_scan[255:224] == 32'h3147544a;
assign scan_flags = {4'd0, flag_tck_ready, flag_error && !flag_busy,
                     flag_accepted && !flag_busy, flag_busy};
assign tdo_o = reg_scan[0];
assign rx_data_o = reg_receive[7:0];
assign rx_valid_o = cnt_receive != 0;
assign tx_ready_o = flag_active && cnt_response < 5'd24;

// 系统复位不要求外部TCK运行；解除后至少等待两个TCK边沿。
always @(posedge tck_i or negedge rst_n_i) begin
    if (!rst_n_i) reg_tck_release <= 0;
    else reg_tck_release <= {reg_tck_release[0], 1'b1};
end

// 完成总线已稳定后，再允许扫描域看到空闲。
always @(posedge tck_i or negedge rst_n_i) begin
    if (!rst_n_i) reg_done_sync <= 0;
    else reg_done_sync <= {reg_done_sync[0], flag_done};
end

// Capture只读快照；Update只有完整且空闲时才写入命令邮箱。
always @(posedge tck_i or negedge rst_n_i) begin
    if (!rst_n_i) begin
        reg_scan <= 0;
        cnt_bits <= 0;
        reg_request <= 0;
        flag_request <= 0;
        flag_accepted <= 0;
    end else begin
        if (tap_reset_i) cnt_bits <= 0;
        else if (select_i && capture_i) begin
            if (flag_busy)
                reg_scan <= {32'h3147544a, 192'd0, 8'd0, scan_flags, 16'h4a52};
            else
                reg_scan <= {32'h3147544a, reg_response, 3'd0, cnt_response,
                             scan_flags, 16'h4a52};
            cnt_bits <= 0;
        end else if (select_i && shift_i) begin
            reg_scan <= {tdi_i, reg_scan[255:1]};
            if (cnt_bits < 9'd257) cnt_bits <= cnt_bits + 1'b1;
        end
        if (flag_submit) begin
            reg_request <= reg_scan[223:24];
            flag_request <= ~flag_request;
            flag_accepted <= 1'b1;
        end
    end
end

// 请求总线在翻转前锁存，源端保持至响应完成，双级同步后取样。
always @(posedge clk_i or negedge rst_n_i) begin
    if (!rst_n_i) reg_request_sync <= 0;
    else reg_request_sync <= {reg_request_sync[0], flag_request};
end

// 收发复用既有协议；没有自造DDR、串口或JTAG器件行为模型。
always @(posedge clk_i or negedge rst_n_i) begin
    if (!rst_n_i) begin
        flag_seen <= 0;
        flag_done <= 0;
        flag_active <= 0;
        flag_error <= 0;
        reg_receive <= 0;
        cnt_receive <= 0;
        reg_response <= 0;
        cnt_response <= 0;
        flag_tx_previous <= 0;
        cnt_timeout <= 0;
    end else begin
        flag_tx_previous <= tx_valid_i && tx_ready_o;
        if (reg_request_sync[1] != flag_seen && !flag_active) begin
            flag_seen <= reg_request_sync[1];
            flag_active <= 1'b1;
            flag_error <= 1'b0;
            reg_receive <= reg_request[199:8];
            cnt_receive <= reg_request[4:0];
            reg_response <= 0;
            cnt_response <= 0;
            cnt_timeout <= 0;
        end else if (flag_active) begin
            if (cnt_receive != 0) begin
                reg_receive <= {8'd0, reg_receive[191:8]};
                cnt_receive <= cnt_receive - 1'b1;
            end
            if (tx_valid_i && tx_ready_o) begin
                reg_response <= {tx_data_i, reg_response[191:8]};
                cnt_response <= cnt_response + 1'b1;
            end
            if (flag_tx_previous && !tx_valid_i) begin
                flag_done <= flag_seen;
                flag_active <= 1'b0;
            end else if (cnt_timeout == TIMEOUT_CYCLES - 1) begin
                flag_done <= flag_seen;
                flag_active <= 1'b0;
                flag_error <= 1'b1;
                cnt_receive <= 0;
            end else begin
                cnt_timeout <= cnt_timeout + 1'b1;
            end
        end
    end
end

endmodule
