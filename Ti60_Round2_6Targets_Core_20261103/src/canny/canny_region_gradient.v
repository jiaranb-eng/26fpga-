`timescale 1ns / 1ps
// Q2 Sobel derivative and a fixed-cost approximation of Euclidean strength.
// Three interpolation weights avoid a divider and general-purpose multipliers.
module canny_region_gradient #(
    parameter WIDTH = 1920,
    parameter HEIGHT = 1200
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
    output wire [67:0] data_o,
    output wire overlap_error_o
);
    wire [39:0] northwest, north, northeast, west, east, southwest, south, southeast;
    wire window_valid, window_sop, window_eop;
    canny_window #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.BITS(10),.REPLICATE(1)) window_inst (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.valid_i(valid_i),.sop_i(sop_i),.eop_i(eop_i),.data_i(data_i),
        .valid_o(window_valid),.sop_o(window_sop),.eop_o(window_eop),
        .northwest_o(northwest),.north_o(north),.northeast_o(northeast),
        .west_o(west),.center_o(),.east_o(east),
        .southwest_o(southwest),.south_o(south),.southeast_o(southeast),.overlap_error_o(overlap_error_o)
    );
    reg [3:0] valid_pipe, sop_pipe, eop_pipe;
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin valid_pipe<=0; sop_pipe<=0; eop_pipe<=0; end
        else begin
            valid_pipe <= {valid_pipe[2:0],window_valid};
            sop_pipe <= {sop_pipe[2:0],window_sop};
            eop_pipe <= {eop_pipe[2:0],window_eop};
        end
    end
    genvar lane;
    generate for (lane=0; lane<4; lane=lane+1) begin : parallel_pixel
        reg [11:0] left_sum, right_sum, top_sum, bottom_sum;
        reg [11:0] abs_x, abs_y;
        reg opposite_sign;
        reg [11:0] major, minor;
        reg horizontal_delay, opposite_delay;
        wire [13:0] minor_three = {2'b0,minor} + {1'b0,minor,1'b0} + 14'd4;
        wire [13:0] major_three = {2'b0,major} + {1'b0,major,1'b0};
        reg [12:0] magnitude;
        reg [1:0] weight;
        reg horizontal, opposite;
        always @(posedge clk_i) begin
            left_sum <= {2'b0,northwest[lane*10+:10]} + {1'b0,west[lane*10+:10],1'b0} + {2'b0,southwest[lane*10+:10]};
            right_sum <= {2'b0,northeast[lane*10+:10]} + {1'b0,east[lane*10+:10],1'b0} + {2'b0,southeast[lane*10+:10]};
            top_sum <= {2'b0,northwest[lane*10+:10]} + {1'b0,north[lane*10+:10],1'b0} + {2'b0,northeast[lane*10+:10]};
            bottom_sum <= {2'b0,southwest[lane*10+:10]} + {1'b0,south[lane*10+:10],1'b0} + {2'b0,southeast[lane*10+:10]};
            abs_x <= right_sum >= left_sum ? right_sum-left_sum : left_sum-right_sum;
            abs_y <= bottom_sum >= top_sum ? bottom_sum-top_sum : top_sum-bottom_sum;
            opposite_sign <= (right_sum < left_sum) ^ (bottom_sum < top_sum);
            major <= abs_x >= abs_y ? abs_x : abs_y;
            minor <= abs_x >= abs_y ? abs_y : abs_x;
            horizontal_delay <= abs_x >= abs_y;
            opposite_delay <= opposite_sign;
            magnitude <= {1'b0,major} + {2'b0,minor_three[13:3]};
            if ({minor,2'b0} < {2'b0,major}) weight <= 0;
            else if ({minor,2'b0} < major_three) weight <= 1;
            else weight <= 2;
            horizontal <= horizontal_delay;
            opposite <= opposite_delay;
        end
        assign data_o[lane*17+:17] = {opposite,horizontal,weight,magnitude};
    end endgenerate
    assign valid_o = valid_pipe[3];
    assign sop_o = sop_pipe[3];
    assign eop_o = eop_pipe[3];
endmodule
