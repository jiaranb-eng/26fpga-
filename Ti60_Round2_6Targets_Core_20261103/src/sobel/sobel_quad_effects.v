`timescale 1ns/1ps

// Four aligned full-resolution effects for the pre-DDR camera clock domain.
module sobel_quad_effects #(
    parameter WIDTH = 1280,
    parameter HEIGHT = 720,
    parameter [23:0] EDGE_COLOR = 24'hFF0000
)(
    input              clk,
    input              rst_n,
    input      [9:0]   threshold_i,
    input              sink_sop,
    input              sink_eop,
    input              sink_valid,
    input      [23:0]  sink_rgb,
    output             source_sop,
    output             source_eop,
    output             source_valid,
    output     [23:0]  source_original_rgb,
    output     [23:0]  source_direct_sobel_rgb,
    output     [23:0]  source_median_sobel_rgb,
    output     [23:0]  source_overlay_rgb,
    output             source_direct_edge,
    output             source_median_edge
);

localparam integer RGB2GRAY_LATENCY = 7;
localparam integer MEDIAN_LATENCY = 9;
localparam integer SOBEL_LATENCY = 4;
localparam integer MATRIX_RAM_LATENCY = 1;
localparam integer TOTAL_LATENCY = RGB2GRAY_LATENCY + MEDIAN_LATENCY +
                                   SOBEL_LATENCY + 2 * MATRIX_RAM_LATENCY;
localparam integer DIRECT_EDGE_LATENCY = RGB2GRAY_LATENCY + SOBEL_LATENCY +
                                         MATRIX_RAM_LATENCY;
localparam integer RGB_ALIGNMENT_LATENCY = TOTAL_LATENCY - MATRIX_RAM_LATENCY;
localparam integer DIRECT_ALIGNMENT_LATENCY = TOTAL_LATENCY -
                                               DIRECT_EDGE_LATENCY -
                                               MATRIX_RAM_LATENCY;

wire       gray_sop;
wire       gray_eop;
wire       gray_valid;
wire [7:0] gray_data;

wire       median_sop;
wire       median_eop;
wire       median_valid;
wire       median_window_valid;
wire [7:0] median_data;

wire       direct_edge_sop;
wire       direct_edge_eop;
wire       direct_edge_valid;
wire [7:0] direct_edge_data;
wire       median_edge_sop;
wire       median_edge_eop;
wire       median_edge_valid;
wire [7:0] median_edge_data;

wire [2:0] direct_full_control;
wire [7:0] direct_full_data;
wire [7:0] direct_spatial_data;
wire       direct_spatial_valid;
wire [8:0] direct_aligned_bus;

wire [23:0] spatial_rgb;
wire        spatial_rgb_valid;
wire [24:0] aligned_rgb_bus;
wire [2:0]  source_control;

wire direct_edge;
wire median_edge;
wire original_valid;

sobel_rgb2gray u_rgb2gray (
    .clk          (clk),
    .rst_n        (rst_n),
    .sink_sop     (sink_sop),
    .sink_valid   (sink_valid),
    .sink_data    (sink_rgb),
    .sink_eop     (sink_eop),
    .source_sop   (gray_sop),
    .source_valid (gray_valid),
    .source_data  (gray_data),
    .source_eop   (gray_eop)
);

sobel_edge_proc #(
    .WIDTH  (WIDTH),
    .HEIGHT (HEIGHT)
) u_direct_sobel (
    .clk          (clk),
    .rst_n        (rst_n),
    .edge_en      (1'b1),
    .bin_en       (1'b1),
    .threshold_i  (threshold_i),
    .sink_sop     (gray_sop),
    .sink_eop     (gray_eop),
    .sink_valid   (gray_valid),
    .sink_data    (gray_data),
    .source_sop   (direct_edge_sop),
    .source_eop   (direct_edge_eop),
    .source_valid (direct_edge_valid),
    .source_data  (direct_edge_data)
);

median_filter_3x3 #(
    .WIDTH  (WIDTH),
    .HEIGHT (HEIGHT)
) u_median_filter (
    .clk                 (clk),
    .rst_n               (rst_n),
    .sink_sop            (gray_sop),
    .sink_eop            (gray_eop),
    .sink_valid          (gray_valid),
    .sink_data           (gray_data),
    .source_sop          (median_sop),
    .source_eop          (median_eop),
    .source_valid        (median_valid),
    .source_window_valid (median_window_valid),
    .source_data         (median_data)
);

sobel_edge_proc #(
    .WIDTH  (WIDTH),
    .HEIGHT (HEIGHT)
) u_median_sobel (
    .clk          (clk),
    .rst_n        (rst_n),
    .edge_en      (1'b1),
    .bin_en       (1'b1),
    .threshold_i  (threshold_i),
    .sink_sop     (median_sop),
    .sink_eop     (median_eop),
    .sink_valid   (median_valid),
    .sink_data    (median_data),
    .source_sop   (median_edge_sop),
    .source_eop   (median_edge_eop),
    .source_valid (median_edge_valid),
    .source_data  (median_edge_data)
);

sobel_shift_regs #(
    .DWIDTH     (3),
    .DELAY_DUTY(DIRECT_EDGE_LATENCY)
) u_direct_full_control_delay (
    .clk   (clk),
    .rst_n (rst_n),
    .idata ({sink_sop, sink_eop, sink_valid}),
    .odata (direct_full_control)
);

assign direct_full_data = direct_edge_valid ? direct_edge_data : 8'd0;

sobel_spatial_align_1x1 #(
    .WIDTH  (WIDTH),
    .HEIGHT (HEIGHT),
    .BINARY (1)
) u_direct_spatial_align (
    .clk                 (clk),
    .rst_n               (rst_n),
    .sink_sop            (direct_full_control[2]),
    .sink_valid          (direct_full_control[0]),
    .sink_data           (direct_full_data),
    .source_data         (direct_spatial_data),
    .source_window_valid (direct_spatial_valid)
);

sobel_shift_regs #(
    .DWIDTH     (9),
    .DELAY_DUTY(DIRECT_ALIGNMENT_LATENCY)
) u_direct_alignment_delay (
    .clk   (clk),
    .rst_n (rst_n),
    .idata ({direct_spatial_valid, direct_spatial_data}),
    .odata (direct_aligned_bus)
);

rgb_spatial_align_2x2 #(
    .WIDTH  (WIDTH),
    .HEIGHT (HEIGHT)
) u_rgb_spatial_align (
    .clk                 (clk),
    .rst_n               (rst_n),
    .sink_sop            (sink_sop),
    .sink_valid          (sink_valid),
    .sink_data           (sink_rgb),
    .source_data         (spatial_rgb),
    .source_window_valid (spatial_rgb_valid)
);

sobel_shift_regs #(
    .DWIDTH     (25),
    .DELAY_DUTY(RGB_ALIGNMENT_LATENCY)
) u_rgb_alignment_delay (
    .clk   (clk),
    .rst_n (rst_n),
    .idata ({spatial_rgb_valid, spatial_rgb}),
    .odata (aligned_rgb_bus)
);

sobel_shift_regs #(
    .DWIDTH     (3),
    .DELAY_DUTY(TOTAL_LATENCY)
) u_source_control_delay (
    .clk   (clk),
    .rst_n (rst_n),
    .idata ({sink_sop, sink_eop, sink_valid}),
    .odata (source_control)
);

assign source_sop   = source_control[2];
assign source_eop   = source_control[1];
assign source_valid = source_control[0];

assign original_valid = source_valid && aligned_rgb_bus[24];
assign direct_edge = direct_aligned_bus[8] &&
                     (direct_aligned_bus[7:0] != 8'd0);
assign median_edge = median_edge_valid && (median_edge_data != 8'd0);

assign source_original_rgb = original_valid ? aligned_rgb_bus[23:0] : 24'd0;
assign source_direct_sobel_rgb = (source_valid && direct_edge) ?
                                 24'hFFFFFF : 24'h000000;
assign source_median_sobel_rgb = (source_valid && median_edge) ?
                                 24'hFFFFFF : 24'h000000;
assign source_overlay_rgb = !original_valid ? 24'h000000 :
                            median_edge ? EDGE_COLOR : aligned_rgb_bus[23:0];
assign source_direct_edge = source_valid && direct_edge;
assign source_median_edge = source_valid && median_edge;

wire unused_direct_markers = direct_edge_sop ^ direct_edge_eop;
wire unused_median_markers = median_edge_sop ^ median_edge_eop;
wire unused_median_window_valid = median_window_valid;

endmodule
