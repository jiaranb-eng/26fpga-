`timescale 1ns/1ps
// Direction quantization and L1 magnitude follow OpenCV 4.12 canny.cpp.
// Apache-2.0; attribution and upstream source are included in research/opencv.
// Direction: 0 horizontal, 1 vertical, 2 NW/SE, 3 NE/SW.
module canny_gradient #(parameter WIDTH=1920, parameter HEIGHT=1200) (
    input wire clk_i, input wire rst_n_i,
    input wire valid_i, input wire sop_i, input wire eop_i,
    input wire [31:0] data_i,
    output wire valid_o, output wire sop_o, output wire eop_o,
    output wire [51:0] data_o, output wire overlap_error_o
);
    wire [31:0] nw, nn, ne, ww, cc, ee, sw, ss, se;
    wire valid, sop, eop;
    canny_window #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.BITS(8),.REPLICATE(1)) window (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.valid_i(valid_i),.sop_i(sop_i),.eop_i(eop_i),.data_i(data_i),
        .valid_o(valid),.sop_o(sop),.eop_o(eop),.northwest_o(nw),.north_o(nn),.northeast_o(ne),
        .west_o(ww),.center_o(cc),.east_o(ee),.southwest_o(sw),.south_o(ss),.southeast_o(se),
        .overlap_error_o(overlap_error_o)
    );
    reg [4:0] valid_pipe, sop_pipe, eop_pipe;
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            valid_pipe <= 0; sop_pipe <= 0; eop_pipe <= 0;
        end else begin
            valid_pipe <= {valid_pipe[3:0],valid};
            sop_pipe <= {sop_pipe[3:0],sop};
            eop_pipe <= {eop_pipe[3:0],eop};
        end
    end
    genvar lane;
    generate for(lane=0;lane<4;lane=lane+1) begin : gradient_pixel
        reg [9:0] left_sum, right_sum, top_sum, bottom_sum;
        reg [9:0] abs_x, abs_y;
        reg opposite_sign;
        reg [10:0] magnitude;
        reg [26:0] tan_low, scaled_y, scaled_x;
        reg sign_delay;
        reg [26:0] tan_high;
        reg horizontal, diagonal_sign;
        reg [26:0] y_delay;
        reg [10:0] magnitude_delay;
        reg [12:0] result;
        always @(posedge clk_i) begin
            left_sum <= {2'b0,nw[lane*8+:8]} + {1'b0,ww[lane*8+:8],1'b0} + {2'b0,sw[lane*8+:8]};
            right_sum <= {2'b0,ne[lane*8+:8]} + {1'b0,ee[lane*8+:8],1'b0} + {2'b0,se[lane*8+:8]};
            top_sum <= {2'b0,nw[lane*8+:8]} + {1'b0,nn[lane*8+:8],1'b0} + {2'b0,ne[lane*8+:8]};
            bottom_sum <= {2'b0,sw[lane*8+:8]} + {1'b0,ss[lane*8+:8],1'b0} + {2'b0,se[lane*8+:8]};
            abs_x <= right_sum >= left_sum ? right_sum-left_sum : left_sum-right_sum;
            abs_y <= bottom_sum >= top_sum ? bottom_sum-top_sum : top_sum-bottom_sum;
            opposite_sign <= (right_sum < left_sum) ^ (bottom_sum < top_sum);
            magnitude <= {1'b0,abs_x} + {1'b0,abs_y};
            tan_low <= abs_x * 14'd13573;
            scaled_y <= {abs_y,15'b0};
            scaled_x <= {abs_x,16'b0};
            sign_delay <= opposite_sign;
            tan_high <= tan_low + scaled_x;
            horizontal <= scaled_y < tan_low;
            y_delay <= scaled_y;
            diagonal_sign <= sign_delay;
            magnitude_delay <= magnitude;
            if (horizontal)
                result <= {2'd0,magnitude_delay};
            else if (y_delay > tan_high)
                result <= {2'd1,magnitude_delay};
            else
                result <= {1'b1,diagonal_sign,magnitude_delay};
        end
        assign data_o[lane*13+:13] = result;
    end endgenerate
    assign valid_o = valid_pipe[4];
    assign sop_o = sop_pipe[4];
    assign eop_o = eop_pipe[4];
endmodule
