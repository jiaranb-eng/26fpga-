`timescale 1ns/1ps

// 3x3 streaming median filter with a nine-stage compare pipeline.
// Each pipeline stage contains at most one level of 8-bit comparators.
module median_filter_3x3 #(
    parameter WIDTH = 1280,
    parameter HEIGHT = 720
)(
    input             clk,
    input             rst_n,
    input             sink_sop,
    input             sink_eop,
    input             sink_valid,
    input      [7:0]  sink_data,
    output            source_sop,
    output            source_eop,
    output            source_valid,
    output            source_window_valid,
    output     [7:0]  source_data
);

localparam integer PIPELINE_LATENCY = 9;
localparam integer STREAM_LATENCY = PIPELINE_LATENCY + 1;

wire [7:0] p11, p12, p13;
wire [7:0] p21, p22, p23;
wire [7:0] p31, p32, p33;
wire       matrix_valid;

sobel_matrix_3x3 #(
    .WIDTH  (WIDTH),
    .HEIGHT (HEIGHT)
) u_matrix_3x3 (
    .clk          (clk),
    .rst_n        (rst_n),
    .sink_sop     (sink_sop),
    .sink_eop     (sink_eop),
    .sink_valid   (sink_valid),
    .sink_data    (sink_data),
    .matrix_sop   (),
    .matrix_eop   (),
    .matrix_valid (matrix_valid),
    .matrix_p11   (p11),
    .matrix_p12   (p12),
    .matrix_p13   (p13),
    .matrix_p21   (p21),
    .matrix_p22   (p22),
    .matrix_p23   (p23),
    .matrix_p31   (p31),
    .matrix_p32   (p32),
    .matrix_p33   (p33)
);

// Stages 1..3 sort each row independently using three compare levels.
reg [7:0] r0_lo_s1, r0_hi_s1, r0_c_s1;
reg [7:0] r1_lo_s1, r1_hi_s1, r1_c_s1;
reg [7:0] r2_lo_s1, r2_hi_s1, r2_c_s1;
reg [7:0] r0_lo_s2, r0_midc_s2, r0_hi_s2;
reg [7:0] r1_lo_s2, r1_midc_s2, r1_hi_s2;
reg [7:0] r2_lo_s2, r2_midc_s2, r2_hi_s2;
reg [7:0] r0_lo_s3, r0_mid_s3, r0_hi_s3;
reg [7:0] r1_lo_s3, r1_mid_s3, r1_hi_s3;
reg [7:0] r2_lo_s3, r2_mid_s3, r2_hi_s3;

// Stages 4..6 form max(row minima), median(row medians), min(row maxima).
reg [7:0] low_max01_s4;
reg [7:0] mid_lo01_s4, mid_hi01_s4, mid_2_s4;
reg [7:0] high_min01_s4, high_2_s4, low_2_s4;
reg [7:0] cand_a_s5, mid_lo_s5, mid_mid_s5, cand_c_s5;
reg [7:0] cand_a_s6, cand_b_s6, cand_c_s6;

// Stages 7..9 select the median of the three remaining candidates.
reg [7:0] final_lo_s7, final_hi_s7, final_c_s7;
reg [7:0] final_lo_s8, final_midc_s8;
reg [7:0] median_s9;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r0_lo_s1 <= 8'd0; r0_hi_s1 <= 8'd0; r0_c_s1 <= 8'd0;
        r1_lo_s1 <= 8'd0; r1_hi_s1 <= 8'd0; r1_c_s1 <= 8'd0;
        r2_lo_s1 <= 8'd0; r2_hi_s1 <= 8'd0; r2_c_s1 <= 8'd0;
        r0_lo_s2 <= 8'd0; r0_midc_s2 <= 8'd0; r0_hi_s2 <= 8'd0;
        r1_lo_s2 <= 8'd0; r1_midc_s2 <= 8'd0; r1_hi_s2 <= 8'd0;
        r2_lo_s2 <= 8'd0; r2_midc_s2 <= 8'd0; r2_hi_s2 <= 8'd0;
        r0_lo_s3 <= 8'd0; r0_mid_s3 <= 8'd0; r0_hi_s3 <= 8'd0;
        r1_lo_s3 <= 8'd0; r1_mid_s3 <= 8'd0; r1_hi_s3 <= 8'd0;
        r2_lo_s3 <= 8'd0; r2_mid_s3 <= 8'd0; r2_hi_s3 <= 8'd0;
        low_max01_s4 <= 8'd0;
        mid_lo01_s4 <= 8'd0; mid_hi01_s4 <= 8'd0; mid_2_s4 <= 8'd0;
        high_min01_s4 <= 8'd0; high_2_s4 <= 8'd0; low_2_s4 <= 8'd0;
        cand_a_s5 <= 8'd0; mid_lo_s5 <= 8'd0; mid_mid_s5 <= 8'd0; cand_c_s5 <= 8'd0;
        cand_a_s6 <= 8'd0; cand_b_s6 <= 8'd0; cand_c_s6 <= 8'd0;
        final_lo_s7 <= 8'd0; final_hi_s7 <= 8'd0; final_c_s7 <= 8'd0;
        final_lo_s8 <= 8'd0; final_midc_s8 <= 8'd0;
        median_s9 <= 8'd0;
    end else begin
        r0_lo_s1 <= (p11 <= p12) ? p11 : p12;
        r0_hi_s1 <= (p11 <= p12) ? p12 : p11;
        r0_c_s1  <= p13;
        r1_lo_s1 <= (p21 <= p22) ? p21 : p22;
        r1_hi_s1 <= (p21 <= p22) ? p22 : p21;
        r1_c_s1  <= p23;
        r2_lo_s1 <= (p31 <= p32) ? p31 : p32;
        r2_hi_s1 <= (p31 <= p32) ? p32 : p31;
        r2_c_s1  <= p33;

        r0_lo_s2   <= r0_lo_s1;
        r0_midc_s2 <= (r0_hi_s1 <= r0_c_s1) ? r0_hi_s1 : r0_c_s1;
        r0_hi_s2   <= (r0_hi_s1 <= r0_c_s1) ? r0_c_s1 : r0_hi_s1;
        r1_lo_s2   <= r1_lo_s1;
        r1_midc_s2 <= (r1_hi_s1 <= r1_c_s1) ? r1_hi_s1 : r1_c_s1;
        r1_hi_s2   <= (r1_hi_s1 <= r1_c_s1) ? r1_c_s1 : r1_hi_s1;
        r2_lo_s2   <= r2_lo_s1;
        r2_midc_s2 <= (r2_hi_s1 <= r2_c_s1) ? r2_hi_s1 : r2_c_s1;
        r2_hi_s2   <= (r2_hi_s1 <= r2_c_s1) ? r2_c_s1 : r2_hi_s1;

        r0_lo_s3  <= (r0_lo_s2 <= r0_midc_s2) ? r0_lo_s2 : r0_midc_s2;
        r0_mid_s3 <= (r0_lo_s2 <= r0_midc_s2) ? r0_midc_s2 : r0_lo_s2;
        r0_hi_s3  <= r0_hi_s2;
        r1_lo_s3  <= (r1_lo_s2 <= r1_midc_s2) ? r1_lo_s2 : r1_midc_s2;
        r1_mid_s3 <= (r1_lo_s2 <= r1_midc_s2) ? r1_midc_s2 : r1_lo_s2;
        r1_hi_s3  <= r1_hi_s2;
        r2_lo_s3  <= (r2_lo_s2 <= r2_midc_s2) ? r2_lo_s2 : r2_midc_s2;
        r2_mid_s3 <= (r2_lo_s2 <= r2_midc_s2) ? r2_midc_s2 : r2_lo_s2;
        r2_hi_s3  <= r2_hi_s2;

        low_max01_s4  <= (r0_lo_s3 >= r1_lo_s3) ? r0_lo_s3 : r1_lo_s3;
        low_2_s4      <= r2_lo_s3;
        mid_lo01_s4   <= (r0_mid_s3 <= r1_mid_s3) ? r0_mid_s3 : r1_mid_s3;
        mid_hi01_s4   <= (r0_mid_s3 <= r1_mid_s3) ? r1_mid_s3 : r0_mid_s3;
        mid_2_s4      <= r2_mid_s3;
        high_min01_s4 <= (r0_hi_s3 <= r1_hi_s3) ? r0_hi_s3 : r1_hi_s3;
        high_2_s4     <= r2_hi_s3;

        cand_a_s5   <= (low_max01_s4 >= low_2_s4) ? low_max01_s4 : low_2_s4;
        mid_lo_s5   <= mid_lo01_s4;
        mid_mid_s5  <= (mid_hi01_s4 <= mid_2_s4) ? mid_hi01_s4 : mid_2_s4;
        cand_c_s5   <= (high_min01_s4 <= high_2_s4) ? high_min01_s4 : high_2_s4;

        cand_a_s6 <= cand_a_s5;
        cand_b_s6 <= (mid_lo_s5 >= mid_mid_s5) ? mid_lo_s5 : mid_mid_s5;
        cand_c_s6 <= cand_c_s5;

        final_lo_s7 <= (cand_a_s6 <= cand_b_s6) ? cand_a_s6 : cand_b_s6;
        final_hi_s7 <= (cand_a_s6 <= cand_b_s6) ? cand_b_s6 : cand_a_s6;
        final_c_s7  <= cand_c_s6;

        final_lo_s8   <= final_lo_s7;
        final_midc_s8 <= (final_hi_s7 <= final_c_s7) ? final_hi_s7 : final_c_s7;

        median_s9 <= (final_lo_s8 >= final_midc_s8) ? final_lo_s8 : final_midc_s8;
    end
end

sobel_shift_regs #(
    .DWIDTH     (3),
    .DELAY_DUTY(STREAM_LATENCY)
) u_stream_control_delay (
    .clk   (clk),
    .rst_n (rst_n),
    .idata ({sink_sop, sink_eop, sink_valid}),
    .odata ({source_sop, source_eop, source_valid})
);

sobel_shift_regs #(
    .DWIDTH     (1),
    .DELAY_DUTY(PIPELINE_LATENCY)
) u_window_control_delay (
    .clk   (clk),
    .rst_n (rst_n),
    .idata (matrix_valid),
    .odata (source_window_valid)
);

assign source_data = source_window_valid ? median_s9 : 8'd0;

endmodule
