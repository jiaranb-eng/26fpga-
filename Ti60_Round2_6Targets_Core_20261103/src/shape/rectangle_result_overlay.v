`timescale 1ns / 1ps
// 将识别结果烘焙进对应的二值显示帧；字模仍使用原工程的同步 ROM。
// Burn annotations into the same retained Canny frame before DDR publication.
module rectangle_result_overlay (
    input wire clk_i,
    input wire rst_n_i,
    input wire prepare_i,
    input wire found_i,
    input wire overflow_i,
    input wire [10:0] left_i,
    input wire [10:0] top_i,
    input wire [10:0] width_i,
    input wire [10:0] height_i,
    input wire [21:0] area_i,
    output wire prepared_o,
    input wire frame_i,
    input wire valid_i,
    input wire sop_i,
    input wire eop_i,
    input wire [63:0] pixels_i,
    output wire ready_o,
    input wire ready_i,
    output wire frame_o,
    output wire valid_o,
    output wire sop_o,
    output wire eop_o,
    output wire [63:0] pixels_o
);
    localparam ST_IDLE=0, ST_DIVIDE=1, ST_BCD_START=2, ST_BCD=3, ST_READY=4;
    reg [2:0] state,next_state;
    reg [4:0] bit_count;
    reg found,overflow;
    reg [10:0] bound_left,bound_top,bound_right,bound_bottom;
    reg [21:0] divide_shift,divide_quotient;
    reg [11:0] divide_remainder,divide_denominator;
    wire [12:0] trial_remainder={divide_remainder,divide_shift[21]};
    wire divide_subtract=trial_remainder>={1'b0,divide_denominator};
    reg [21:0] number_shift [0:5];
    reg [27:0] number_bcd [0:5];
    reg [27:0] corrected_bcd [0:5];
    integer number,nibble;
    always @(*) begin
        for(number=0;number<6;number=number+1) begin
            corrected_bcd[number]=number_bcd[number];
            for(nibble=0;nibble<7;nibble=nibble+1) begin
                if(number_bcd[number][nibble*4+:4]>=5) begin
                    corrected_bcd[number][nibble*4+:4]=number_bcd[number][nibble*4+:4]+4'd3;
                end
            end
        end
    end
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin state<=ST_IDLE; end
        else begin state<=next_state; end
    end
    always @(*) begin
        next_state=state;
        case(state)
            ST_IDLE,ST_READY: if(prepare_i) begin next_state=ST_DIVIDE; end
            ST_DIVIDE: if(bit_count==21) begin next_state=ST_BCD_START; end
            ST_BCD_START: begin next_state=ST_BCD; end
            ST_BCD: if(bit_count==21) begin next_state=ST_READY; end
            default: begin next_state=ST_IDLE; end
        endcase
    end
    integer field_index;
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            found<=0; overflow<=0; bound_left<=0; bound_top<=0; bound_right<=0; bound_bottom<=0;
            divide_shift<=0; divide_quotient<=0; divide_remainder<=0; divide_denominator<=1; bit_count<=0;
            for(field_index=0;field_index<6;field_index=field_index+1) begin
                number_shift[field_index]<=0; number_bcd[field_index]<=0;
            end
        end else begin
            case(state)
                ST_IDLE,ST_READY: if(prepare_i) begin
                    found<=found_i; overflow<=overflow_i;
                    bound_left<=left_i; bound_top<=top_i;
                    bound_right<=left_i+width_i-1'b1; bound_bottom<=top_i+height_i-1'b1;
                    number_shift[0]<=found_i ? left_i+(width_i>>1) : 0;
                    number_shift[1]<=found_i ? top_i+(height_i>>1) : 0;
                    number_shift[2]<=found_i ? width_i : 0;
                    number_shift[3]<=found_i ? height_i : 0;
                    number_shift[4]<=found_i ? area_i : 0;
                    divide_shift<=found_i ? width_i*22'd1000 : 0;
                    divide_denominator<=height_i==0 ? 12'd1 : {1'b0,height_i};
                    divide_quotient<=0; divide_remainder<=0; bit_count<=0;
                end
                ST_DIVIDE: begin
                    divide_shift<={divide_shift[20:0],1'b0};
                    divide_quotient<={divide_quotient[20:0],divide_subtract};
                    divide_remainder<=divide_subtract ? trial_remainder-divide_denominator : trial_remainder;
                    bit_count<=bit_count+1'b1;
                end
                ST_BCD_START: begin
                    number_shift[5]<=divide_quotient; bit_count<=0;
                    for(field_index=0;field_index<6;field_index=field_index+1) begin number_bcd[field_index]<=0; end
                end
                ST_BCD: begin
                    bit_count<=bit_count+1'b1;
                    for(field_index=0;field_index<6;field_index=field_index+1) begin
                        number_bcd[field_index]<={corrected_bcd[field_index][26:0],number_shift[field_index][21]};
                        number_shift[field_index]<={number_shift[field_index][20:0],1'b0};
                    end
                end
                default: begin end
            endcase
        end
    end

    reg [10:0] group_x,pixel_y;
    reg [1:0] valid_pipe;
    reg [1:0] sop_pipe,eop_pipe;
    wire advance=!valid_pipe[1] || ready_i;
    wire accept=valid_i && advance;
    wire [10:0] current_x=sop_i ? 11'd0 : group_x;
    wire [10:0] current_y=sop_i ? 11'd0 : pixel_y;
    wire in_panel=current_x>=32 && current_x<896 && current_y>=112 && current_y<208;
    wire [10:0] panel_x=current_x-32;
    wire [10:0] panel_y=current_y-112;
    wire [4:0] text_column=panel_x[9:5];
    wire [1:0] text_row=panel_y[6:5];
    reg [7:0] glyph;
    wire [15:0] font_bits;
    // 每个四像素组与 32 像素字符边界对齐，一次 ROM 读取供四路使用。
    always @(*) begin
        glyph=" ";
        case(text_row)
            0: case(text_column)
                0: glyph=overflow ? "O" : (found ? "R" : "N");
                1: glyph=overflow ? "V" : (found ? "E" : "O");
                2: glyph=overflow ? "F" : (found ? "C" : "N");
                3: glyph=overflow ? " " : (found ? "T" : "E");
                5: glyph="C"; 6: glyph="X"; 7: glyph=":";
                8: glyph=8'h30+number_bcd[0][15:12]; 9: glyph=8'h30+number_bcd[0][11:8];
                10: glyph=8'h30+number_bcd[0][7:4]; 11: glyph=8'h30+number_bcd[0][3:0];
                13: glyph="C"; 14: glyph="Y"; 15: glyph=":";
                16: glyph=8'h30+number_bcd[1][15:12]; 17: glyph=8'h30+number_bcd[1][11:8];
                18: glyph=8'h30+number_bcd[1][7:4]; 19: glyph=8'h30+number_bcd[1][3:0];
                default: begin end
            endcase
            1: case(text_column)
                0: glyph="W"; 1: glyph=":";
                2: glyph=8'h30+number_bcd[2][15:12]; 3: glyph=8'h30+number_bcd[2][11:8];
                4: glyph=8'h30+number_bcd[2][7:4]; 5: glyph=8'h30+number_bcd[2][3:0];
                7: glyph="H"; 8: glyph=":";
                9: glyph=8'h30+number_bcd[3][15:12]; 10: glyph=8'h30+number_bcd[3][11:8];
                11: glyph=8'h30+number_bcd[3][7:4]; 12: glyph=8'h30+number_bcd[3][3:0];
                14: glyph="A"; 15: glyph=":";
                16: glyph=8'h30+number_bcd[4][27:24]; 17: glyph=8'h30+number_bcd[4][23:20];
                18: glyph=8'h30+number_bcd[4][19:16]; 19: glyph=8'h30+number_bcd[4][15:12];
                20: glyph=8'h30+number_bcd[4][11:8]; 21: glyph=8'h30+number_bcd[4][7:4];
                22: glyph=8'h30+number_bcd[4][3:0];
                default: begin end
            endcase
            2: case(text_column)
                0: glyph="A"; 1: glyph="R"; 2: glyph=":";
                3: glyph=8'h30+number_bcd[5][19:16]; 4: glyph=8'h30+number_bcd[5][15:12];
                5: glyph=".";
                6: glyph=8'h30+number_bcd[5][11:8]; 7: glyph=8'h30+number_bcd[5][7:4];
                8: glyph=8'h30+number_bcd[5][3:0];
                10: glyph="B"; 11: glyph="B"; 12: glyph="O"; 13: glyph="X";
                15: glyph="P"; 16: glyph="X";
                default: begin end
            endcase
            default: begin end
        endcase
    end
    osd_font_rom font (
        .clk_i(clk_i),.enable_i(advance),.glyph_id_i(glyph),
        .glyph_row_i(panel_y[4:1]),.row_bits_o(font_bits)
    );
    reg panel_delay;
    reg [3:0] glyph_column_delay;
    reg [3:0] box_delay;
    reg [63:0] input_delay,output_pixels;
    reg [3:0] box_now;
    reg [11:0] lane_x;
    integer lane;
    always @(*) begin
        box_now=0;
        lane_x=0;
        for(lane=0;lane<4;lane=lane+1) begin
            lane_x={1'b0,current_x}+lane;
            if(found && lane_x>=bound_left && lane_x<=bound_right && current_y>=bound_top && current_y<=bound_bottom) begin
                box_now[lane]=((current_y-bound_top<2 || bound_bottom-current_y<2) &&
                    (lane_x-bound_left<24 || bound_right-lane_x<24)) ||
                    ((lane_x-bound_left<2 || bound_right-lane_x<2) &&
                    (current_y-bound_top<24 || bound_bottom-current_y<24));
            end
        end
    end
    integer output_lane;
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            group_x<=0; pixel_y<=0; valid_pipe<=0; sop_pipe<=0; eop_pipe<=0;
            panel_delay<=0; glyph_column_delay<=0; box_delay<=0; input_delay<=0; output_pixels<=0;
        end else if(advance) begin
            valid_pipe<={valid_pipe[0],valid_i};
            sop_pipe<={sop_pipe[0],sop_i};
            eop_pipe<={eop_pipe[0],eop_i};
            panel_delay<=in_panel;
            glyph_column_delay<=panel_x[4:1];
            box_delay<=box_now; input_delay<=pixels_i;
            for(output_lane=0;output_lane<4;output_lane=output_lane+1) begin
                output_pixels[(3-output_lane)*16+:16]<=panel_delay ?
                    (font_bits[15-glyph_column_delay-(output_lane/2)] ? 16'hffff : 16'h0000) :
                    (box_delay[output_lane] ? 16'hffff : input_delay[(3-output_lane)*16+:16]);
            end
            if(accept) begin
                if(current_x==1916) begin group_x<=0; pixel_y<=current_y+1'b1; end
                else begin group_x<=current_x+4; pixel_y<=current_y; end
            end
        end
    end
    assign prepared_o=state==ST_READY;
    assign ready_o=advance;
    assign valid_o=valid_pipe[1];
    assign sop_o=sop_pipe[1];
    assign eop_o=eop_pipe[1];
    assign pixels_o=output_pixels;
    assign frame_o=frame_i || (|valid_pipe);
endmodule
