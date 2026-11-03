`timescale 1ns / 1ps

// 四次实例化 Efinity 2025.1 生成的 W0_FIFO_32；不替代或修改官方 IP。
module quad_write_fifo_bank #(
    parameter COMPACT_EDGES = 0
) (
    input  wire         write_clk_i,
    input  wire         read_clk_i,
    input  wire         async_reset_i,
    input  wire         write_enable_0_i,
    input  wire         write_enable_1_i,
    input  wire         write_enable_2_i,
    input  wire         write_enable_3_i,
    input  wire [31:0]  write_data_0_i,
    input  wire [31:0]  write_data_1_i,
    input  wire [31:0]  write_data_2_i,
    input  wire [31:0]  write_data_3_i,
    input  wire         read_enable_0_i,
    input  wire         read_enable_1_i,
    input  wire         read_enable_2_i,
    input  wire         read_enable_3_i,
    output wire [127:0] read_data_0_o,
    output wire [127:0] read_data_1_o,
    output wire [127:0] read_data_2_o,
    output wire [127:0] read_data_3_o,
    output wire [9:0]   read_count_0_o,
    output wire [9:0]   read_count_1_o,
    output wire [9:0]   read_count_2_o,
    output wire [9:0]   read_count_3_o,
    output wire         empty_0_o,
    output wire         empty_1_o,
    output wire         empty_2_o,
    output wire         empty_3_o,
    output wire         overflow_0_o,
    output wire         overflow_1_o,
    output wire         overflow_2_o,
    output wire         overflow_3_o,
    output wire         underflow_0_o,
    output wire         underflow_1_o,
    output wire         underflow_2_o,
    output wire         underflow_3_o,
    output wire         reset_busy_o
);

wire reset_busy_0;
wire reset_busy_1;
wire reset_busy_2;
wire reset_busy_3;

assign reset_busy_o = reset_busy_0 || reset_busy_1 ||
                      reset_busy_2 || reset_busy_3;

W0_FIFO_32 u_write_fifo_0 (
    .prog_full_o(), .full_o(), .empty_o(empty_0_o), .prog_empty_o(),
    .wr_clk_i(write_clk_i), .rd_clk_i(read_clk_i),
    .wr_en_i(write_enable_0_i), .rd_en_i(read_enable_0_i),
    .wdata(write_data_0_i), .rst_busy(reset_busy_0),
    .rdata(read_data_0_o), .a_rst_i(async_reset_i),
    .wr_datacount_o(), .rd_datacount_o(read_count_0_o),
    .underflow_o(underflow_0_o), .overflow_o(overflow_0_o)
);

quad_edge_write_fifo #(.COMPACT(COMPACT_EDGES)) u_write_fifo_1 (
    .prog_full_o(), .full_o(), .empty_o(empty_1_o), .prog_empty_o(),
    .wr_clk_i(write_clk_i), .rd_clk_i(read_clk_i),
    .wr_en_i(write_enable_1_i), .rd_en_i(read_enable_1_i),
    .wdata(write_data_1_i), .rst_busy(reset_busy_1),
    .rdata(read_data_1_o), .a_rst_i(async_reset_i),
    .wr_datacount_o(), .rd_datacount_o(read_count_1_o),
    .underflow_o(underflow_1_o), .overflow_o(overflow_1_o)
);

quad_edge_write_fifo #(.COMPACT(COMPACT_EDGES)) u_write_fifo_2 (
    .prog_full_o(), .full_o(), .empty_o(empty_2_o), .prog_empty_o(),
    .wr_clk_i(write_clk_i), .rd_clk_i(read_clk_i),
    .wr_en_i(write_enable_2_i), .rd_en_i(read_enable_2_i),
    .wdata(write_data_2_i), .rst_busy(reset_busy_2),
    .rdata(read_data_2_o), .a_rst_i(async_reset_i),
    .wr_datacount_o(), .rd_datacount_o(read_count_2_o),
    .underflow_o(underflow_2_o), .overflow_o(overflow_2_o)
);

W0_FIFO_32 u_write_fifo_3 (
    .prog_full_o(), .full_o(), .empty_o(empty_3_o), .prog_empty_o(),
    .wr_clk_i(write_clk_i), .rd_clk_i(read_clk_i),
    .wr_en_i(write_enable_3_i), .rd_en_i(read_enable_3_i),
    .wdata(write_data_3_i), .rst_busy(reset_busy_3),
    .rdata(read_data_3_o), .a_rst_i(async_reset_i),
    .wr_datacount_o(), .rd_datacount_o(read_count_3_o),
    .underflow_o(underflow_3_o), .overflow_o(overflow_3_o)
);

endmodule

// 黑白RGB565的无损位编码；FIFO控制、复位和CDC均由原有官方实现承担。
module quad_edge_write_fifo #(
    parameter COMPACT = 0
) (
    input wire wr_clk_i,
    input wire rd_clk_i,
    input wire a_rst_i,
    input wire wr_en_i,
    input wire rd_en_i,
    input wire [31:0] wdata,
    output wire [127:0] rdata,
    output wire [11:0] wr_datacount_o,
    output wire [9:0] rd_datacount_o,
    output wire prog_full_o,
    output wire full_o,
    output wire empty_o,
    output wire prog_empty_o,
    output wire rst_busy,
    output wire underflow_o,
    output wire overflow_o
);
    generate if (COMPACT) begin : binary_pixels
        wire [7:0] edge_pixels; // 按原端序返回八个二值像素。
        genvar pixel;
        for (pixel=0; pixel<8; pixel=pixel+1) begin : expand_rgb565
            assign rdata[pixel*16+:16] = {16{edge_pixels[pixel]}};
        end
        // 精确绑定当前工程IP UUID；官方源文件不修改，保持其参数与FWFT合同。
        efx_fifo_top_55a3570120784ad19e9ed29adc4b0850 #(
            .SYNC_CLK(0), .SYNC_STAGE(3), .DATA_WIDTH(2),
            .MODE("FWFT"), .OUTPUT_REG(0), .PIPELINE_REG(1),
            .DEPTH(2048), .FAMILY("TITANIUM"), .ASYM_WIDTH_RATIO(6),
            .PROG_FULL_ASSERT(1536), .PROGRAMMABLE_FULL("STATIC_SINGLE"),
            .PROG_FULL_NEGATE(1536), .PROGRAMMABLE_EMPTY("STATIC_SINGLE"),
            .PROG_EMPTY_ASSERT(127), .PROG_EMPTY_NEGATE(128),
            .OPTIONAL_FLAGS(0), .BYPASS_RESET_SYNC(0), .ENDIANESS(0),
            .RAM_STYLE("block_ram"), .OVERFLOW_PROTECT(1), .UNDERFLOW_PROTECT(1)
        ) official_edge_fifo (
            .wr_clk_i(wr_clk_i), .rd_clk_i(rd_clk_i), .a_rst_i(a_rst_i),
            .wr_en_i(wr_en_i), .rd_en_i(rd_en_i), .wdata({wdata[31],wdata[15]}),
            .rdata(edge_pixels), .rst_busy(rst_busy),
            .wr_datacount_o(wr_datacount_o), .rd_datacount_o(rd_datacount_o),
            .prog_full_o(prog_full_o), .full_o(full_o), .empty_o(empty_o),
            .prog_empty_o(prog_empty_o), .underflow_o(underflow_o), .overflow_o(overflow_o)
        );
    end else begin : full_color_pixels
        W0_FIFO_32 official_color_fifo (
            .wr_clk_i(wr_clk_i), .rd_clk_i(rd_clk_i), .a_rst_i(a_rst_i),
            .wr_en_i(wr_en_i), .rd_en_i(rd_en_i), .wdata(wdata), .rdata(rdata),
            .rst_busy(rst_busy), .wr_datacount_o(wr_datacount_o),
            .rd_datacount_o(rd_datacount_o), .prog_full_o(prog_full_o),
            .full_o(full_o), .empty_o(empty_o), .prog_empty_o(prog_empty_o),
            .underflow_o(underflow_o), .overflow_o(overflow_o)
        );
    end endgenerate
endmodule
