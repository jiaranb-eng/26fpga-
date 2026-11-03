`timescale 1ns / 1ps

// JTAG控制包装器：保留既有命令含义和帧边界，仅替换物理传输层。
module jtag_video_control #(
    parameter [9:0] DEFAULT_THRESHOLD = 10'd256,
    parameter [9:0] MAX_THRESHOLD = 10'd1020,
    parameter integer MAILBOX_TIMEOUT_CYCLES = 2097152,
    parameter integer FRAME_TIMEOUT_CYCLES = 960000
) (
    input wire clk,
    input wire rst_n,
    input wire jtag_tck_i,
    input wire jtag_capture_i,
    input wire jtag_shift_i,
    input wire jtag_update_i,
    input wire jtag_reset_i,
    input wire jtag_select_i,
    input wire jtag_tdi_i,
    output wire jtag_tdo_o,
    output wire [9:0] threshold_o,
    output wire threshold_valid_o,
    input wire threshold_ready_i,
    output wire [10:0] canny_low_o,canny_high_o,
    output wire canny_valid_o,input wire canny_ready_i,
    input wire [2:0] display_mode_i,
    input wire [7:0] history_count_i,
    input wire [7:0] video_fault_i,
    input wire display_changed_i,
    output wire [2:0] display_request_o,
    output wire display_valid_o,
    output wire [1:0] history_control_o,
    output wire [7:0] history_age_a_o,
    output wire [7:0] history_age_b_o,
    output wire [7:0] background_threshold_o,
    output wire background_relearn_o,
    input wire background_valid_i, input wire background_pending_i,
    input wire [31:0] background_epoch_i, input wire [31:0] background_frames_i,
    input wire [31:0] background_cycles_i, input wire [3:0] background_count_i,
    input wire background_overflow_i,
    output wire background_mask_capture_o,
    output wire [13:0] background_mask_address_o,
    input wire [7:0] background_mask_data_i,
    input wire background_mask_ready_i, input wire background_mask_busy_i,
    input wire [7:0] background_mask_epoch_i,

    output wire osd_wr_en_o,
    output wire [5:0] osd_wr_addr_o,
    output wire [7:0] osd_wr_data_o
);

wire [7:0] rx_data;
wire rx_valid;
wire [7:0] tx_data;
wire tx_valid;
wire tx_ready;

// 物理扫描域与系统域之间仅传输整包邮箱。
jtag_packet_mailbox #(
    .TIMEOUT_CYCLES(MAILBOX_TIMEOUT_CYCLES)
) u_jtag_packet_mailbox (
    .clk_i(clk), .rst_n_i(rst_n), .tck_i(jtag_tck_i),
    .capture_i(jtag_capture_i), .shift_i(jtag_shift_i),
    .update_i(jtag_update_i), .tap_reset_i(jtag_reset_i),
    .select_i(jtag_select_i), .tdi_i(jtag_tdi_i), .tdo_o(jtag_tdo_o),
    .rx_data_o(rx_data), .rx_valid_o(rx_valid),
    .tx_data_i(tx_data), .tx_valid_i(tx_valid), .tx_ready_o(tx_ready)
);

// 原命令解码器未经修改，阈值/模式/历史控制语义一致。
fpga_control_protocol #(
    .DEFAULT_THRESHOLD(DEFAULT_THRESHOLD), .MAX_THRESHOLD(MAX_THRESHOLD),
    .FRAME_TIMEOUT_CYCLES(FRAME_TIMEOUT_CYCLES)
) u_fpga_control_protocol (
    .clk(clk), .rst_n(rst_n),
    .rx_data_i(rx_data), .rx_valid_i(rx_valid),
    .tx_data_o(tx_data), .tx_valid_o(tx_valid), .tx_ready_i(tx_ready),
    .threshold_o(threshold_o), .threshold_valid_o(threshold_valid_o),
    .threshold_ready_i(threshold_ready_i),
    .canny_low_o(canny_low_o),.canny_high_o(canny_high_o),.canny_valid_o(canny_valid_o),.canny_ready_i(canny_ready_i),
    .display_mode_i(display_mode_i), .history_count_i(history_count_i),
    .video_fault_i(video_fault_i), .display_changed_i(display_changed_i),
    .display_request_o(display_request_o), .display_valid_o(display_valid_o),
    .history_control_o(history_control_o), .history_age_a_o(history_age_a_o),
    .history_age_b_o(history_age_b_o),
    .background_mask_capture_o(background_mask_capture_o),
    .background_mask_address_o(background_mask_address_o),.background_mask_data_i(background_mask_data_i),
    .background_mask_ready_i(background_mask_ready_i),.background_mask_busy_i(background_mask_busy_i),
    .background_mask_epoch_i(background_mask_epoch_i),
    .background_threshold_o(background_threshold_o),.background_relearn_o(background_relearn_o),
    .background_valid_i(background_valid_i),.background_pending_i(background_pending_i),
    .background_epoch_i(background_epoch_i),.background_frames_i(background_frames_i),
    .background_cycles_i(background_cycles_i),.background_count_i(background_count_i),.background_overflow_i(background_overflow_i),
    .osd_wr_en_o(osd_wr_en_o), .osd_wr_addr_o(osd_wr_addr_o),
    .osd_wr_data_o(osd_wr_data_o)
);

endmodule
