`timescale 1ns / 1ps
// Description: Capture stable HSIO parallel bytes half a cycle before CSI parsing.
// 描述：在字节时钟下降沿采样四通道并行数据，供下一上升沿译码。
// Clock: byte_clk_i falling edge; the downstream parser uses its rising edge.
// Reset: none for data-only storage; the receiver owns packet-valid reset.
module sc233_byte_capture (
    input wire byte_clk_i,
    input wire [31:0] bytes_i,
    output reg [31:0] bytes_o
);
    // HSIO changes parallel bytes after the rising edge; sample the stable half cycle.
    always @(negedge byte_clk_i) begin
        bytes_o <= bytes_i;
    end
endmodule
