`timescale 1ns/1ps
// OpenCV NMS tie breaking is asymmetric on axial edges and strict on diagonals.
// State bits: 00 rejected, 01 weak candidate, 11 connected strong seed.
// This stage deliberately does not claim to perform hysteresis.
module canny_classify #(
    parameter WIDTH=1920, parameter HEIGHT=1200,
    parameter [10:0] LOW_THRESHOLD=11'd50,
    parameter [10:0] HIGH_THRESHOLD=11'd100,
    parameter RUNTIME_THRESHOLDS=0
) (
    input wire clk_i, input wire rst_n_i,
    input wire valid_i, input wire sop_i, input wire eop_i,
    input wire [51:0] data_i,
    input wire [10:0] low_threshold_i, input wire [10:0] high_threshold_i,
    output wire valid_o, output wire sop_o, output wire eop_o,
    output wire [7:0] data_o, output wire overlap_error_o
);
    wire [51:0] nw, nn, ne, ww, cc, ee, sw, ss, se;
    wire valid, sop, eop;
    canny_window #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.BITS(13),.REPLICATE(0)) window (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.valid_i(valid_i),.sop_i(sop_i),.eop_i(eop_i),.data_i(data_i),
        .valid_o(valid),.sop_o(sop),.eop_o(eop),.northwest_o(nw),.north_o(nn),.northeast_o(ne),
        .west_o(ww),.center_o(cc),.east_o(ee),.southwest_o(sw),.south_o(ss),.southeast_o(se),
        .overlap_error_o(overlap_error_o)
    );
    reg [2:0] valid_pipe, sop_pipe, eop_pipe;
    reg [10:0] frame_low,frame_high;
    wire [10:0] active_low=RUNTIME_THRESHOLDS ? frame_low : LOW_THRESHOLD;
    wire [10:0] active_high=RUNTIME_THRESHOLDS ? frame_high : HIGH_THRESHOLD;
    // Snapshot at this stage's window SOP, not the raw camera SOP. Previous
    // frame tail still compares against its own thresholds on this edge;
    // the new first magnitude reaches comparison on the following edge.
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin frame_low<=LOW_THRESHOLD; frame_high<=HIGH_THRESHOLD; end
        else if(valid && sop) begin frame_low<=low_threshold_i; frame_high<=high_threshold_i; end
    end
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            valid_pipe <= 0; sop_pipe <= 0; eop_pipe <= 0;
        end else begin
            valid_pipe <= {valid_pipe[1:0],valid};
            sop_pipe <= {sop_pipe[1:0],sop};
            eop_pipe <= {eop_pipe[1:0],eop};
        end
    end
    genvar lane;
    generate for(lane=0;lane<4;lane=lane+1) begin : classify_pixel
        reg [10:0] before_value, after_value, magnitude;
        reg diagonal;
        reg maximum, eligible, strong_seed;
        reg [1:0] classification;
        always @(posedge clk_i) begin
            magnitude <= cc[lane*13+:11];
            diagonal <= cc[lane*13+12];
            case(cc[lane*13+11+:2])
                0: begin before_value <= ww[lane*13+:11]; after_value <= ee[lane*13+:11]; end
                1: begin before_value <= nn[lane*13+:11]; after_value <= ss[lane*13+:11]; end
                2: begin before_value <= nw[lane*13+:11]; after_value <= se[lane*13+:11]; end
                3: begin before_value <= ne[lane*13+:11]; after_value <= sw[lane*13+:11]; end
            endcase
            maximum <= magnitude > before_value && (diagonal ? magnitude > after_value : magnitude >= after_value);
            eligible <= magnitude > active_low;
            strong_seed <= magnitude > active_high;
            classification <= maximum && eligible ? {strong_seed,1'b1} : 2'b00;
        end
        assign data_o[lane*2+:2] = classification;
    end endgenerate
    assign valid_o = valid_pipe[2];
    assign sop_o = sop_pipe[2];
    assign eop_o = eop_pipe[2];
endmodule
