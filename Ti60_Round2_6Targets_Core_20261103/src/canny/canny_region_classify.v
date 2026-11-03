`timescale 1ns / 1ps
// Quadratic ridge position selection with a half-open two-pixel footprint.
// A concave peak lies within (-1,1] of each retained sample.
// Runtime thresholds are captured on the first input group of each frame.
module canny_region_classify #(
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
    input wire [67:0] data_i,
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
    wire [67:0] northwest, north, northeast, west, center, east, southwest, south, southeast;
    wire window_valid, window_sop, window_eop;
    canny_window #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.BITS(17),.REPLICATE(0)) window_inst (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.valid_i(valid_i),.sop_i(sop_i),.eop_i(eop_i),.data_i(data_i),
        .valid_o(window_valid),.sop_o(window_sop),.eop_o(window_eop),
        .northwest_o(northwest),.north_o(north),.northeast_o(northeast),
        .west_o(west),.center_o(center),.east_o(east),
        .southwest_o(southwest),.south_o(south),.southeast_o(southeast),.overlap_error_o(overlap_error_o)
    );
    reg [4:0] valid_pipe, sop_pipe, eop_pipe;
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin valid_pipe<=0; sop_pipe<=0; eop_pipe<=0; end
        else begin
            valid_pipe <= {valid_pipe[3:0],window_valid};
            sop_pipe <= {sop_pipe[3:0],window_sop};
            eop_pipe <= {eop_pipe[3:0],window_eop};
        end
    end
    genvar lane;
    generate for (lane=0; lane<4; lane=lane+1) begin : parallel_pixel
        wire horizontal = center[lane*17+15];
        wire opposite = center[lane*17+16];
        reg [12:0] before_axis, after_axis, before_diagonal, after_diagonal;
        reg [12:0] magnitude;
        reg [1:0] weight;
        reg [13:0] before_value, after_value, center_value;
        reg [16:0] amplitude;
        reg [15:0] before_triple, after_triple;
        reg [13:0] before_delay, after_delay;
        reg [15:0] center_quad, center_quad_delay;
        reg [15:0] before_bound, after_bound;
        reg curved, curved_delay;
        reg [16:0] amplitude_delay, amplitude_aligned;
        reg [1:0] result;
        always @(posedge clk_i) begin
            before_axis <= horizontal ? west[lane*17+:13] : north[lane*17+:13];
            after_axis <= horizontal ? east[lane*17+:13] : south[lane*17+:13];
            before_diagonal <= opposite ? (horizontal ? southwest[lane*17+:13] : northeast[lane*17+:13]) : northwest[lane*17+:13];
            after_diagonal <= opposite ? (horizontal ? northeast[lane*17+:13] : southwest[lane*17+:13]) : southeast[lane*17+:13];
            magnitude <= center[lane*17+:13];
            weight <= center[lane*17+13+:2];
            if (weight == 0) begin
                before_value <= {before_axis,1'b0};
                after_value <= {after_axis,1'b0};
            end else if (weight == 1) begin
                before_value <= {1'b0,before_axis} + {1'b0,before_diagonal};
                after_value <= {1'b0,after_axis} + {1'b0,after_diagonal};
            end else begin
                before_value <= {before_diagonal,1'b0};
                after_value <= {after_diagonal,1'b0};
            end
            center_value <= {magnitude,1'b0};
            amplitude <= {1'b0,magnitude,3'b0} + {2'b0,magnitude,2'b0} + {4'b0,magnitude};
            // Three-point parabola vertex d=(R-L)/(2*(2*C-L-R)).
            // Test -1<d<=1 without division; discard flat or convex profiles.
            before_triple <= {2'b0,before_value} + {1'b0,before_value,1'b0};
            after_triple <= {2'b0,after_value} + {1'b0,after_value,1'b0};
            before_delay <= before_value;
            after_delay <= after_value;
            center_quad <= {center_value,2'b0};
            curved <= {center_value,1'b0} > ({1'b0,before_value}+{1'b0,after_value});
            amplitude_delay <= amplitude;
            before_bound <= before_triple + {2'b0,after_delay};
            after_bound <= after_triple + {2'b0,before_delay};
            center_quad_delay <= center_quad;
            curved_delay <= curved;
            amplitude_aligned <= amplitude_delay;
            if (center_quad_delay > before_bound && center_quad_delay >= after_bound && curved_delay && amplitude_aligned > {1'b0,frame_low,5'b0})
                result <= amplitude_aligned > {1'b0,frame_high,5'b0} ? 2'b11 : 2'b01;
            else result <= 2'b00;
        end
        assign state_o[lane*2+:2] = result;
    end endgenerate
    assign valid_o = valid_pipe[4];
    assign sop_o = sop_pipe[4];
    assign eop_o = eop_pipe[4];
endmodule
