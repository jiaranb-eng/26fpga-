`timescale 1ns / 1ps
// Current-frame bounded-influence intensity estimation followed by edge detection.
// Keep the original four-pixel stream and two-bit DDR candidate contract.
module canny_frontend #(
    parameter WIDTH = 1920,
    parameter HEIGHT = 1200,
    parameter [10:0] LOW_THRESHOLD = 11'd10,
    parameter [10:0] HIGH_THRESHOLD = 11'd20,
    parameter RUNTIME_THRESHOLDS = 0
) (
    input wire clk_i,
    input wire rst_n_i,
    input wire valid_i,
    input wire sop_i,
    input wire eop_i,
    input wire [31:0] data_i,
    input wire [10:0] low_threshold_i,
    input wire [10:0] high_threshold_i,
    output wire valid_o,
    output wire sop_o,
    output wire eop_o,
    output wire [7:0] state_o,
    output wire overlap_error_o
);
    reg [10:0] frame_low, frame_high;
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin frame_low<=LOW_THRESHOLD; frame_high<=HIGH_THRESHOLD; end
        else if (valid_i && sop_i) begin
            frame_low <= RUNTIME_THRESHOLDS ? low_threshold_i : LOW_THRESHOLD;
            frame_high <= RUNTIME_THRESHOLDS ? high_threshold_i : HIGH_THRESHOLD;
        end
    end
    wire [4:0] errors;
    wire intensity_valid, intensity_sop, intensity_eop;
    wire horizontal_valid, horizontal_sop, horizontal_eop;
    wire vertical_valid, vertical_sop, vertical_eop;
    wire gradient_valid, gradient_sop, gradient_eop;
    wire [39:0] intensity_data, horizontal_data, vertical_data;
    wire [67:0] gradient_data;
    canny_raw_intensity #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) intensity_inst (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.valid_i(valid_i),.sop_i(sop_i),.eop_i(eop_i),.data_i(data_i),
        .valid_o(intensity_valid),.sop_o(intensity_sop),.eop_o(intensity_eop),.data_o(intensity_data),.overlap_error_o(errors[0])
    );
    canny_bounded_filter #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.VERTICAL(0)) horizontal_inst (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.valid_i(intensity_valid),.sop_i(intensity_sop),.eop_i(intensity_eop),.data_i(intensity_data),
        .valid_o(horizontal_valid),.sop_o(horizontal_sop),.eop_o(horizontal_eop),.data_o(horizontal_data),.overlap_error_o(errors[1])
    );
    canny_bounded_filter #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.VERTICAL(1)) vertical_inst (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.valid_i(horizontal_valid),.sop_i(horizontal_sop),.eop_i(horizontal_eop),.data_i(horizontal_data),
        .valid_o(vertical_valid),.sop_o(vertical_sop),.eop_o(vertical_eop),.data_o(vertical_data),.overlap_error_o(errors[2])
    );
    canny_region_gradient #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) gradient_inst (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.valid_i(vertical_valid),.sop_i(vertical_sop),.eop_i(vertical_eop),.data_i(vertical_data),
        .valid_o(gradient_valid),.sop_o(gradient_sop),.eop_o(gradient_eop),.data_o(gradient_data),.overlap_error_o(errors[3])
    );
    canny_region_classify #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.LOW_THRESHOLD(LOW_THRESHOLD),.HIGH_THRESHOLD(HIGH_THRESHOLD),.RUNTIME_THRESHOLDS(1)) classify_inst (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.valid_i(gradient_valid),.sop_i(gradient_sop),.eop_i(gradient_eop),.data_i(gradient_data),
        .low_threshold_i(frame_low),.high_threshold_i(frame_high),
        .valid_o(valid_o),.sop_o(sop_o),.eop_o(eop_o),.state_o(state_o),.overlap_error_o(errors[4])
    );
    assign overlap_error_o = |errors;
endmodule
