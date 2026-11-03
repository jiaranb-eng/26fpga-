`timescale 1ns / 1ps
// Select binary coverage reduction while preserving legacy gray/full-scan contracts.
// 活动二值画面使用区域OR缩放；历史灰度/全扫描参数仍复用原始模块。
module canny_display_render #(
    parameter MAX_OBJECTS=4,parameter LABEL_BITS=MAX_OBJECTS>4 ? 4 : 3,
    parameter SUPPORT_COMPARE=0,
    parameter KIND_BITS=3, parameter SUPPORT_LABELS=0,
    parameter FULL_SCAN_ENABLE = 0,
    parameter SUPPORT_GRAY = 0,
    parameter SOURCE_WIDTH = 1920,
    parameter SOURCE_HEIGHT = 1200,
    parameter DISPLAY_WIDTH = 1280,
    parameter DISPLAY_HEIGHT = 720,
    parameter SCALE_NUM = 5,
    parameter SCALE_DEN = 3,
    parameter ROW_STRIDE = SOURCE_WIDTH / 4
) (
    input wire compare_i,output wire compare_left_o,
    input wire color_enable_i,input wire [MAX_OBJECTS*KIND_BITS-1:0] object_kind_i,
    output wire [13:0] label_address_o,input wire [4*LABEL_BITS-1:0] label_data_i,
    input wire full_scan_i,
    input wire gray_i,
    input wire clk_i,
    input wire rst_n_i,
    input wire start_i,
    input wire [31:0] base_address_i,
    output wire busy_o,
    output wire done_o,
    output wire read_request_o,
    input wire read_request_ready_i,
    output wire [31:0] read_address_o,
    input wire read_valid_i,
    output wire read_ready_o,
    input wire [63:0] read_data_i,
    output wire frame_active_o,
    output wire valid_o,
    input wire ready_i,
    output wire sop_o,
    output wire eop_o,
    output wire [63:0] rgb565_o
);
    generate if (SUPPORT_GRAY || FULL_SCAN_ENABLE) begin : legacy_formats
        assign label_address_o=0;assign compare_left_o=0;
        canny_render #(
            .FULL_SCAN_ENABLE(FULL_SCAN_ENABLE), .SUPPORT_GRAY(SUPPORT_GRAY),
            .SOURCE_WIDTH(SOURCE_WIDTH), .SOURCE_HEIGHT(SOURCE_HEIGHT),
            .DISPLAY_WIDTH(DISPLAY_WIDTH), .DISPLAY_HEIGHT(DISPLAY_HEIGHT),
            .SCALE_NUM(SCALE_NUM), .SCALE_DEN(SCALE_DEN), .ROW_STRIDE(ROW_STRIDE)
        ) legacy_inst (
            .full_scan_i(full_scan_i), .gray_i(gray_i), .clk_i(clk_i), .rst_n_i(rst_n_i),
            .start_i(start_i), .base_address_i(base_address_i), .busy_o(busy_o), .done_o(done_o),
            .read_request_o(read_request_o), .read_request_ready_i(read_request_ready_i), .read_address_o(read_address_o),
            .read_valid_i(read_valid_i), .read_ready_o(read_ready_o), .read_data_i(read_data_i),
            .frame_active_o(frame_active_o), .valid_o(valid_o), .ready_i(ready_i),
            .sop_o(sop_o), .eop_o(eop_o), .rgb565_o(rgb565_o)
        );
    end else begin : binary_coverage
        canny_coverage_render #(.MAX_OBJECTS(MAX_OBJECTS),.LABEL_BITS(LABEL_BITS),
            .SUPPORT_LABELS(SUPPORT_LABELS),.KIND_BITS(KIND_BITS),.SUPPORT_COMPARE(SUPPORT_COMPARE),
            .SOURCE_WIDTH(SOURCE_WIDTH), .SOURCE_HEIGHT(SOURCE_HEIGHT),
            .DISPLAY_WIDTH(DISPLAY_WIDTH), .DISPLAY_HEIGHT(DISPLAY_HEIGHT),
            .SCALE_NUM(SCALE_NUM), .SCALE_DEN(SCALE_DEN), .ROW_STRIDE(ROW_STRIDE)
        ) coverage_inst (
            .compare_i(compare_i),.compare_left_o(compare_left_o),
            .color_enable_i(color_enable_i),.object_kind_i(object_kind_i),.label_address_o(label_address_o),.label_data_i(label_data_i),
            .clk_i(clk_i), .rst_n_i(rst_n_i), .start_i(start_i), .base_address_i(base_address_i),
            .busy_o(busy_o), .done_o(done_o), .read_request_o(read_request_o),
            .read_request_ready_i(read_request_ready_i), .read_address_o(read_address_o),
            .read_valid_i(read_valid_i), .read_ready_o(read_ready_o), .read_data_i(read_data_i),
            .frame_active_o(frame_active_o), .valid_o(valid_o), .ready_i(ready_i),
            .sop_o(sop_o), .eop_o(eop_o), .rgb565_o(rgb565_o)
        );
    end endgenerate
endmodule
