`timescale 1ns/1ps

// Five-stage 16x16 unsigned multiplier structure from the source Sobel project.
module sobel_unsigned_multiplier_16x16_pp (
    input              clk,
    input              rst_n,
    input      [15:0]  a,
    input      [15:0]  b,
    output reg [31:0]  p
);

function [7:0] mult_4x4;
    input [3:0] a4;
    input [3:0] b4;
    begin
        mult_4x4 = a4 * b4;
    end
endfunction

wire [3:0] a0 = a[3:0];
wire [3:0] a1 = a[7:4];
wire [3:0] a2 = a[11:8];
wire [3:0] a3 = a[15:12];
wire [3:0] b0 = b[3:0];
wire [3:0] b1 = b[7:4];
wire [3:0] b2 = b[11:8];
wire [3:0] b3 = b[15:12];

reg [7:0] pp00, pp01, pp02, pp03;
reg [7:0] pp10, pp11, pp12, pp13;
reg [7:0] pp20, pp21, pp22, pp23;
reg [7:0] pp30, pp31, pp32, pp33;

reg [31:0] app00, app01, app02, app03;
reg [31:0] app10, app11, app12, app13;
reg [31:0] app20, app21, app22, app23;
reg [31:0] app30, app31, app32, app33;

reg [31:0] sum1, sum2, sum3, sum4;
reg [31:0] sum;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pp00 <= 8'd0; pp01 <= 8'd0; pp02 <= 8'd0; pp03 <= 8'd0;
        pp10 <= 8'd0; pp11 <= 8'd0; pp12 <= 8'd0; pp13 <= 8'd0;
        pp20 <= 8'd0; pp21 <= 8'd0; pp22 <= 8'd0; pp23 <= 8'd0;
        pp30 <= 8'd0; pp31 <= 8'd0; pp32 <= 8'd0; pp33 <= 8'd0;
    end else begin
        pp00 <= mult_4x4(a0, b0); pp01 <= mult_4x4(a0, b1);
        pp02 <= mult_4x4(a0, b2); pp03 <= mult_4x4(a0, b3);
        pp10 <= mult_4x4(a1, b0); pp11 <= mult_4x4(a1, b1);
        pp12 <= mult_4x4(a1, b2); pp13 <= mult_4x4(a1, b3);
        pp20 <= mult_4x4(a2, b0); pp21 <= mult_4x4(a2, b1);
        pp22 <= mult_4x4(a2, b2); pp23 <= mult_4x4(a2, b3);
        pp30 <= mult_4x4(a3, b0); pp31 <= mult_4x4(a3, b1);
        pp32 <= mult_4x4(a3, b2); pp33 <= mult_4x4(a3, b3);
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        app00 <= 32'd0; app01 <= 32'd0; app02 <= 32'd0; app03 <= 32'd0;
        app10 <= 32'd0; app11 <= 32'd0; app12 <= 32'd0; app13 <= 32'd0;
        app20 <= 32'd0; app21 <= 32'd0; app22 <= 32'd0; app23 <= 32'd0;
        app30 <= 32'd0; app31 <= 32'd0; app32 <= 32'd0; app33 <= 32'd0;
    end else begin
        app00 <= {24'd0, pp00};       app01 <= {20'd0, pp01, 4'd0};
        app02 <= {16'd0, pp02, 8'd0}; app03 <= {12'd0, pp03, 12'd0};
        app10 <= {20'd0, pp10, 4'd0}; app11 <= {16'd0, pp11, 8'd0};
        app12 <= {12'd0, pp12, 12'd0}; app13 <= {8'd0, pp13, 16'd0};
        app20 <= {16'd0, pp20, 8'd0}; app21 <= {12'd0, pp21, 12'd0};
        app22 <= {8'd0, pp22, 16'd0}; app23 <= {4'd0, pp23, 20'd0};
        app30 <= {12'd0, pp30, 12'd0}; app31 <= {8'd0, pp31, 16'd0};
        app32 <= {4'd0, pp32, 20'd0}; app33 <= {pp33, 24'd0};
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sum1 <= 32'd0;
        sum2 <= 32'd0;
        sum3 <= 32'd0;
        sum4 <= 32'd0;
    end else begin
        sum1 <= app00 + app01 + app02 + app03;
        sum2 <= app10 + app11 + app12 + app13;
        sum3 <= app20 + app21 + app22 + app23;
        sum4 <= app30 + app31 + app32 + app33;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        sum <= 32'd0;
    else
        sum <= sum1 + sum2 + sum3 + sum4;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        p <= 32'd0;
    else
        p <= sum;
end

endmodule

module sobel_rgb2gray_mult (
    input             clock,
    input      [7:0]  dataa,
    input      [9:0]  datab,
    output     [17:0] result
);

wire [31:0] product;

sobel_unsigned_multiplier_16x16_pp u_multiplier (
    .clk   (clock),
    .rst_n (1'b1),
    .a     ({8'd0, dataa}),
    .b     ({6'd0, datab}),
    .p     (product)
);

assign result = product[17:0];

endmodule
