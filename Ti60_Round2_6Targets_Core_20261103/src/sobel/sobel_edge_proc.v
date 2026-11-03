`timescale 1ns/1ps

// Four-direction Sobel core. Arithmetic and output selection match the source project.
module sobel_edge_proc #(
    parameter WIDTH = 1280,
    parameter HEIGHT = 720
)(
    input             clk,
    input             rst_n,
    input             edge_en,
    input             bin_en,
    input      [9:0]  threshold_i,
    input             sink_sop,
    input             sink_eop,
    input             sink_valid,
    input      [7:0]  sink_data,
    output            source_sop,
    output            source_eop,
    output            source_valid,
    output     [7:0]  source_data
);

wire [7:0] matrix_p11, matrix_p12, matrix_p13;
wire [7:0] matrix_p21, matrix_p22, matrix_p23;
wire [7:0] matrix_p31, matrix_p32, matrix_p33;
wire       matrix_sop, matrix_eop, matrix_valid;
wire [7:0] matrix_p22_dly;

sobel_matrix_3x3 #(
    .WIDTH(WIDTH),
    .HEIGHT(HEIGHT)
) u_matrix_3x3 (
    .clk          (clk),
    .rst_n        (rst_n),
    .sink_sop     (sink_sop),
    .sink_eop     (sink_eop),
    .sink_valid   (sink_valid),
    .sink_data    (sink_data),
    .matrix_sop   (matrix_sop),
    .matrix_eop   (matrix_eop),
    .matrix_valid (matrix_valid),
    .matrix_p11   (matrix_p11),
    .matrix_p12   (matrix_p12),
    .matrix_p13   (matrix_p13),
    .matrix_p21   (matrix_p21),
    .matrix_p22   (matrix_p22),
    .matrix_p23   (matrix_p23),
    .matrix_p31   (matrix_p31),
    .matrix_p32   (matrix_p32),
    .matrix_p33   (matrix_p33)
);

reg [9:0] g0_p, g0_n, g0;
reg [9:0] g45_p, g45_n, g45;
reg [9:0] g90_p, g90_n, g90;
reg [9:0] g135_p, g135_n, g135;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        g0_p <= 10'd0;
        g0_n <= 10'd0;
        g0   <= 10'd0;
    end else begin
        g0_p <= matrix_p11 + {1'b0, matrix_p12, 1'b0} + matrix_p13;
        g0_n <= matrix_p31 + {1'b0, matrix_p32, 1'b0} + matrix_p33;
        g0   <= (g0_p >= g0_n) ? g0_p - g0_n : g0_n - g0_p;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        g45_p <= 10'd0;
        g45_n <= 10'd0;
        g45   <= 10'd0;
    end else begin
        g45_p <= matrix_p23 + {1'b0, matrix_p33, 1'b0} + matrix_p32;
        g45_n <= matrix_p12 + {1'b0, matrix_p11, 1'b0} + matrix_p21;
        g45   <= (g45_p >= g45_n) ? g45_p - g45_n : g45_n - g45_p;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        g90_p <= 10'd0;
        g90_n <= 10'd0;
        g90   <= 10'd0;
    end else begin
        g90_p <= matrix_p13 + {1'b0, matrix_p23, 1'b0} + matrix_p33;
        g90_n <= matrix_p11 + {1'b0, matrix_p21, 1'b0} + matrix_p31;
        g90   <= (g90_p >= g90_n) ? g90_p - g90_n : g90_n - g90_p;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        g135_p <= 10'd0;
        g135_n <= 10'd0;
        g135   <= 10'd0;
    end else begin
        g135_p <= matrix_p12 + {1'b0, matrix_p13, 1'b0} + matrix_p23;
        g135_n <= matrix_p21 + {1'b0, matrix_p31, 1'b0} + matrix_p32;
        g135   <= (g135_p >= g135_n) ? g135_p - g135_n : g135_n - g135_p;
    end
end

reg [9:0] gmax_0_45;
reg [9:0] gmax_90_135;
reg [9:0] gmax;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        gmax_0_45   <= 10'd0;
        gmax_90_135 <= 10'd0;
    end else begin
        gmax_0_45   <= (g0 >= g45) ? g0 : g45;
        gmax_90_135 <= (g90 >= g135) ? g90 : g135;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        gmax <= 10'd0;
    else
        gmax <= (gmax_0_45 >= gmax_90_135) ? gmax_0_45 : gmax_90_135;
end

assign source_data = edge_en ?
                     (bin_en ? ((gmax > threshold_i) ? 8'hff : 8'h00) :
                     ((gmax > 10'd510) ? 8'hff : gmax[8:1])) : matrix_p22_dly;

sobel_shift_regs #(
    .DWIDTH(3),
    .DELAY_DUTY(4)
) u_ctrl_delay (
    .clk   (clk),
    .rst_n (rst_n),
    .idata ({matrix_sop, matrix_valid, matrix_eop}),
    .odata ({source_sop, source_valid, source_eop})
);

sobel_shift_regs #(
    .DWIDTH(8),
    .DELAY_DUTY(4)
) u_center_delay (
    .clk   (clk),
    .rst_n (rst_n),
    .idata (matrix_p22),
    .odata (matrix_p22_dly)
);

endmodule
