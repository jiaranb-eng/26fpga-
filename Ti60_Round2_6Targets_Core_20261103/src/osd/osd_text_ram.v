`timescale 1ns/1ps

// 双时钟字符存储器：系统时钟域更新字符，像素时钟域同步读取字符。
// Dual-clock text memory: the system domain updates cells and the pixel domain reads them.
//
// The registered read port follows the Efinix Titanium inference template so
// that the array maps to embedded RAM instead of asynchronous LUT logic.
module osd_text_ram (
    input  wire       wr_clk_i,
    input  wire       wr_en_i,
    input  wire [5:0] wr_addr_i,
    input  wire [7:0] wr_data_i,
    input  wire       rd_clk_i,
    input wire rd_enable_i,
    input  wire [5:0] rd_addr_i,
    output reg  [7:0] rd_data_o
);

(* syn_ramstyle = "block_ram" *) reg [7:0] text_memory [0:63];

initial begin
    $readmemh("src/osd/osd_text_init.mem", text_memory);
end

always @(posedge wr_clk_i) begin
    if (wr_en_i) begin
        text_memory[wr_addr_i] <= wr_data_i;
    end
end

always @(posedge rd_clk_i) if(rd_enable_i) begin
    rd_data_o <= text_memory[rd_addr_i];
end

endmodule
