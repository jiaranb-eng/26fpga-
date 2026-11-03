`timescale 1ns/1ps
// Bayer-balanced intensity estimate with Q2 retained after the binomial sum.
module canny_raw_intensity #(parameter WIDTH=1920, parameter HEIGHT=1200) (
    input wire clk_i, input wire rst_n_i,
    input wire valid_i, input wire sop_i, input wire eop_i,
    input wire [31:0] data_i,
    output wire valid_o, output wire sop_o, output wire eop_o,
    output wire [39:0] data_o, output wire overlap_error_o
);
    wire [31:0] nw, nn, ne, ww, cc, ee, sw, ss, se;
    wire valid, sop, eop;
    canny_window #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.BITS(8),.REPLICATE(1)) window (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.valid_i(valid_i),.sop_i(sop_i),.eop_i(eop_i),.data_i(data_i),
        .valid_o(valid),.sop_o(sop),.eop_o(eop),.northwest_o(nw),.north_o(nn),.northeast_o(ne),
        .west_o(ww),.center_o(cc),.east_o(ee),.southwest_o(sw),.south_o(ss),.southeast_o(se),
        .overlap_error_o(overlap_error_o)
    );
    reg [2:0] valid_pipe, sop_pipe, eop_pipe;
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
    generate for(lane=0;lane<4;lane=lane+1) begin : gaussian_pixel
        reg [9:0] top_sum, middle_sum, bottom_sum;
        reg [11:0] total;
        reg [9:0] rounded;
        always @(posedge clk_i) begin
            top_sum <= {2'b0,nw[lane*8+:8]} + {1'b0,nn[lane*8+:8],1'b0} + {2'b0,ne[lane*8+:8]};
            middle_sum <= {2'b0,ww[lane*8+:8]} + {1'b0,cc[lane*8+:8],1'b0} + {2'b0,ee[lane*8+:8]};
            bottom_sum <= {2'b0,sw[lane*8+:8]} + {1'b0,ss[lane*8+:8],1'b0} + {2'b0,se[lane*8+:8]};
            total <= {2'b0,top_sum} + {1'b0,middle_sum,1'b0} + {2'b0,bottom_sum};
            rounded <= (total + 12'd2) >> 2;
        end
        assign data_o[lane*10+:10] = rounded;
    end endgenerate
    assign valid_o = valid_pipe[2];
    assign sop_o = sop_pipe[2];
    assign eop_o = eop_pipe[2];
endmodule
