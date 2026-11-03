`timescale 1ns/1ps
// Four parallel copies of the supplied VIP_RAW8_RGB888 bilinear arithmetic.
// The BGGR phase is expressed at the centered window coordinates; unlike the
// reference's display-side line delay, output coordinates stay on the center.
// Reference algorithm copyright: CrazyBingo Corporation, 2013-20xx.
module sc233_rgb_quad #(parameter WIDTH=1920, HEIGHT=1200,
    parameter C_DISPLAY_CORRECTION=1) (
    input wire clk_i, input wire rst_n_i, input wire ce_i,
    input wire valid_i, input wire sop_i, input wire eop_i,
    input wire [31:0] raw_i,
    output wire valid_o, output wire eop_o, output wire [63:0] rgb565_o
);
    wire [31:0] nw,nn,ne,ww,cc,ee,sw,ss,se;
    wire win_valid,win_sop,win_eop;
    sc233_rgb_window #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.BITS(8),.REPLICATE(1)) window (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.ce_i(ce_i),.valid_i(valid_i),.sop_i(sop_i),.eop_i(eop_i),.data_i(raw_i),
        .valid_o(win_valid),.sop_o(win_sop),.eop_o(win_eop),
        .northwest_o(nw),.north_o(nn),.northeast_o(ne),.west_o(ww),.center_o(cc),.east_o(ee),
        .southwest_o(sw),.south_o(ss),.southeast_o(se),.overlap_error_o()
    );
    reg [10:0] group_index;
    reg row_odd;
    reg [1:0] valid_pipe,eop_pipe;
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin group_index<=0;row_odd<=0;valid_pipe<=0;eop_pipe<=0;end
        else if(ce_i) begin
            valid_pipe<={valid_pipe[0],win_valid};eop_pipe<={eop_pipe[0],win_eop};
            if(win_valid) begin
                if(win_sop) begin group_index<=1;row_odd<=0;end
                else if(group_index==WIDTH/4-1) begin group_index<=0;row_odd<=!row_odd;end
                else group_index<=group_index+1'b1;
            end
        end
    end
    genvar lane;
    generate for(lane=0;lane<4;lane=lane+1) begin: pixel
        localparam LANE_ODD=(lane%2)==1;
        wire [9:0] diagonal={2'd0,nw[lane*8+:8]}+{2'd0,ne[lane*8+:8]}+{2'd0,sw[lane*8+:8]}+{2'd0,se[lane*8+:8]};
        wire [9:0] cross_sum={2'd0,nn[lane*8+:8]}+{2'd0,ww[lane*8+:8]}+{2'd0,ee[lane*8+:8]}+{2'd0,ss[lane*8+:8]};
        wire [8:0] horizontal={1'b0,ww[lane*8+:8]}+{1'b0,ee[lane*8+:8]};
        wire [8:0] vertical={1'b0,nn[lane*8+:8]}+{1'b0,ss[lane*8+:8]};
        reg [7:0] red,green,blue;
        wire [15:0] display_pixel;
        // 在原有打包寄存级前校正彩色画面，不改变CE或valid/eop延迟。
        if(C_DISPLAY_CORRECTION) begin: corrected_color
            rgb_display_balance balance (
                .red_i(red),.green_i(green),.blue_i(blue),.pixel_o(display_pixel)
            );
        end else begin: linear_color
            assign display_pixel={red[7:3],green[7:2],blue[7:3]};
        end
        reg [15:0] packed_pixel;
        always @(posedge clk_i) if(ce_i) begin
            case({row_odd,LANE_ODD})
                2'b00: begin red<=diagonal[9:2];green<=cross_sum[9:2];blue<=cc[lane*8+:8];end
                2'b01: begin red<=vertical[8:1];green<=cc[lane*8+:8];blue<=horizontal[8:1];end
                2'b10: begin red<=horizontal[8:1];green<=cc[lane*8+:8];blue<=vertical[8:1];end
                2'b11: begin red<=cc[lane*8+:8];green<=cross_sum[9:2];blue<=diagonal[9:2];end
            endcase
            packed_pixel<=display_pixel;
        end
        assign rgb565_o[lane*16+:16]=packed_pixel;
    end endgenerate
    assign valid_o=valid_pipe[1];assign eop_o=eop_pipe[1];
endmodule
