`timescale 1ns / 1ps

// UART transport, framed protocol endpoint, and threshold update source.
module sobel_uart_control #(
    parameter integer CLK_FREQ_HZ = 96_000_000,
    parameter integer BAUD_RATE   = 115_200,
    parameter [9:0] DEFAULT_THRESHOLD = 10'd256,
    parameter [9:0] MAX_THRESHOLD     = 10'd1020
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       uart_rx_i,
    output wire       uart_tx_o,
    output wire [9:0] threshold_o,
    output wire       threshold_valid_o,
    input  wire       threshold_ready_i,
    input wire [1:0] display_mode_i,
    input wire [7:0] history_count_i,
    input wire [7:0] video_fault_i,
    input wire display_changed_i,
    output wire [1:0] display_request_o,
    output wire display_valid_o,
    output wire [1:0] history_control_o,
    output wire [7:0] history_age_a_o,
    output wire [7:0] history_age_b_o,
    output wire       osd_wr_en_o,
    output wire [5:0] osd_wr_addr_o,
    output wire [7:0] osd_wr_data_o
);

wire [7:0] rx_data;
wire       rx_valid;
wire [7:0] tx_data;
wire       tx_valid;
wire       tx_ready;

uart_rx_byte #(
    .CLK_FREQ_HZ(CLK_FREQ_HZ),
    .BAUD_RATE  (BAUD_RATE)
) u_uart_rx_byte (
    .clk           (clk),
    .rst_n         (rst_n),
    .rx_i          (uart_rx_i),
    .data_o        (rx_data),
    .data_valid_o  (rx_valid),
    .frame_error_o ()
);

uart_tx_byte #(
    .CLK_FREQ_HZ(CLK_FREQ_HZ),
    .BAUD_RATE  (BAUD_RATE)
) u_uart_tx_byte (
    .clk          (clk),
    .rst_n        (rst_n),
    .data_i       (tx_data),
    .data_valid_i (tx_valid),
    .data_ready_o (tx_ready),
    .tx_o         (uart_tx_o)
);

fpga_control_protocol #(
    .DEFAULT_THRESHOLD(DEFAULT_THRESHOLD),
    .MAX_THRESHOLD    (MAX_THRESHOLD)
) u_fpga_control_protocol (
    .display_mode_i(display_mode_i),.history_count_i(history_count_i),
    .video_fault_i(video_fault_i),.display_changed_i(display_changed_i),
    .display_request_o(display_request_o),.display_valid_o(display_valid_o),
    .history_control_o(history_control_o),.history_age_a_o(history_age_a_o),
    .history_age_b_o(history_age_b_o),
    .clk                (clk),
    .rst_n              (rst_n),
    .rx_data_i          (rx_data),
    .rx_valid_i         (rx_valid),
    .tx_data_o          (tx_data),
    .tx_valid_o         (tx_valid),
    .tx_ready_i         (tx_ready),
    .threshold_o        (threshold_o),
    .threshold_valid_o  (threshold_valid_o),
    .threshold_ready_i  (threshold_ready_i),
    .osd_wr_en_o        (osd_wr_en_o),
    .osd_wr_addr_o      (osd_wr_addr_o),
    .osd_wr_data_o      (osd_wr_data_o)
);

endmodule
