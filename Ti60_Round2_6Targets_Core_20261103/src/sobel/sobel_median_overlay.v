`timescale 1ns/1ps

// Post-DDR video pipeline: RGB-to-gray, 3x3 median, Sobel and red overlay.
module sobel_median_overlay #(
    parameter WIDTH = 1280,
    parameter HEIGHT = 720,
    parameter [23:0] EDGE_COLOR = 24'hFF0000
)(
    input              clk,
    input              rst_n,
    input              sink_sop,
    input              sink_eop,
    input              sink_valid,
    input      [23:0]  sink_rgb,
    input      [9:0]   threshold_i,
    output             source_sop,
    output             source_eop,
    output             source_valid,
    output             source_edge,
    output     [23:0]  source_rgb
);

localparam integer RGB2GRAY_LATENCY = 7;
localparam integer MEDIAN_LATENCY = 9;
localparam integer SOBEL_LATENCY = 4;
localparam integer MATRIX_RAM_LATENCY = 1;
localparam integer TOTAL_LATENCY = RGB2GRAY_LATENCY + MEDIAN_LATENCY +
                                   SOBEL_LATENCY + 2 * MATRIX_RAM_LATENCY;
localparam integer RGB_ALIGNMENT_LATENCY = TOTAL_LATENCY - MATRIX_RAM_LATENCY;

wire       gray_sop;
wire       gray_eop;
wire       gray_valid;
wire [7:0] gray_data;
wire       median_sop;
wire       median_eop;
wire       median_valid;
wire       median_window_valid;
wire [7:0] median_data;
wire       edge_sop;
wire       edge_eop;
wire       edge_valid;
wire [7:0] edge_data;
wire [23:0] spatial_rgb;
wire        spatial_valid;
wire [24:0] aligned_rgb_bus;
wire [2:0]  source_control;

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

median_filter_3x3 #(
    .WIDTH  (WIDTH),
    .HEIGHT (HEIGHT)
) u_median_filter_3x3 (
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
) u_sobel_edge_proc (
    .clk          (clk),
    .rst_n        (rst_n),
    .edge_en      (1'b1),
    .bin_en       (1'b1),
    .threshold_i  (threshold_i),
    .sink_sop     (median_sop),
    .sink_eop     (median_eop),
    .sink_valid   (median_valid),
    .sink_data    (median_data),
    .source_sop   (edge_sop),
    .source_eop   (edge_eop),
    .source_valid (edge_valid),
    .source_data  (edge_data)
);

rgb_spatial_align_2x2 #(
    .WIDTH  (WIDTH),
    .HEIGHT (HEIGHT)
) u_rgb_spatial_align_2x2 (
    .clk                 (clk),
    .rst_n               (rst_n),
    .sink_sop            (sink_sop),
    .sink_valid          (sink_valid),
    .sink_data           (sink_rgb),
    .source_data         (spatial_rgb),
    .source_window_valid (spatial_valid)
);

sobel_shift_regs #(
    .DWIDTH     (25),
    .DELAY_DUTY(RGB_ALIGNMENT_LATENCY)
) u_rgb_alignment_delay (
    .clk   (clk),
    .rst_n (rst_n),
    .idata ({spatial_valid, spatial_rgb}),
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
assign source_edge  = edge_valid && (edge_data != 8'd0);
assign source_rgb   = (!source_valid || !aligned_rgb_bus[24]) ? 24'h000000 :
                      source_edge ? EDGE_COLOR : aligned_rgb_bus[23:0];

wire unused_median_window_valid = median_window_valid;
wire unused_edge_sop = edge_sop;
wire unused_edge_eop = edge_eop;

endmodule
