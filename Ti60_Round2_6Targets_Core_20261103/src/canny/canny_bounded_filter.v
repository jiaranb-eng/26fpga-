`timescale 1ns / 1ps
// Limit each neighbour's influence before averaging; retain Q2 intensity.
// The seven-tap kernel is [1 1 1 2 1 1 1]/8 and has no variable divisor.
module canny_bounded_filter #(
    parameter WIDTH = 1920,
    parameter HEIGHT = 1200,
    parameter VERTICAL = 0
) (
    input wire clk_i,
    input wire rst_n_i,
    input wire valid_i,
    input wire sop_i,
    input wire eop_i,
    input wire [39:0] data_i,
    output wire valid_o,
    output wire sop_o,
    output wire eop_o,
    output wire [39:0] data_o,
    output wire overlap_error_o
);
    wire window_valid, window_sop, window_eop;
    wire [279:0] taps;
    canny_axis_window #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.BITS(10),.VERTICAL(VERTICAL)) window_inst (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.valid_i(valid_i),.sop_i(sop_i),.eop_i(eop_i),.data_i(data_i),
        .valid_o(window_valid),.sop_o(window_sop),.eop_o(window_eop),.taps_o(taps),.overlap_error_o(overlap_error_o)
    );
    reg [5:0] valid_pipe, sop_pipe, eop_pipe;
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin valid_pipe<=0; sop_pipe<=0; eop_pipe<=0; end
        else begin
            valid_pipe <= {valid_pipe[4:0],window_valid};
            sop_pipe <= {sop_pipe[4:0],window_sop};
            eop_pipe <= {eop_pipe[4:0],window_eop};
        end
    end
    genvar lane;
    genvar neighbour;
    generate for (lane=0; lane<4; lane=lane+1) begin : parallel_pixel
        reg [9:0] center_delay [0:4];
        wire signed [10:0] limited [0:5];
        reg signed [11:0] pair_left, pair_middle, pair_right;
        reg signed [12:0] sum_left, sum_right;
        reg signed [13:0] total;
        wire signed [13:0] rounded = total + 14'sd4;
        wire signed [13:0] correction = rounded >>> 3;
        wire signed [13:0] result_sum = $signed({4'b0,center_delay[4]}) + correction;
        reg [9:0] result;
        integer stage;
        for (neighbour=0; neighbour<6; neighbour=neighbour+1) begin : bounded_neighbour
            localparam TAP = neighbour < 3 ? neighbour : neighbour+1;
            reg signed [10:0] difference;
            reg signed [10:0] clipped;
            always @(posedge clk_i) begin
                difference <= $signed({1'b0,taps[(TAP*4+lane)*10+:10]}) - $signed({1'b0,taps[(3*4+lane)*10+:10]});
                if (difference > 11'sd16) clipped <= 11'sd16;
                else if (difference < -11'sd16) clipped <= -11'sd16;
                else clipped <= difference;
            end
            assign limited[neighbour] = clipped;
        end
        always @(posedge clk_i) begin
            center_delay[0] <= taps[(3*4+lane)*10+:10];
            for (stage=1; stage<5; stage=stage+1) center_delay[stage] <= center_delay[stage-1];
            pair_left <= {limited[0][10],limited[0]} + {limited[1][10],limited[1]};
            pair_middle <= {limited[2][10],limited[2]} + {limited[3][10],limited[3]};
            pair_right <= {limited[4][10],limited[4]} + {limited[5][10],limited[5]};
            sum_left <= {pair_left[11],pair_left} + {pair_middle[11],pair_middle};
            sum_right <= {pair_right[11],pair_right};
            total <= {sum_left[12],sum_left} + {sum_right[12],sum_right};
            result <= result_sum[9:0];
        end
        assign data_o[lane*10+:10] = result;
    end endgenerate
    assign valid_o = valid_pipe[5];
    assign sop_o = sop_pipe[5];
    assign eop_o = eop_pipe[5];
endmodule
