`timescale 1ns / 1ps
// Description: Draw four complete colored boxes and per-object geometry.
// 描述：在同帧边缘画面叠加四个闭合彩色框、类别、中心、宽高及外接框面积。
// Clock: clk_i, 96 MHz; Reset: rst_n_i, asynchronous active low.
// TestBench: verification/multi_shape_iteration/tb_shape_overlay.v
module shape_result_overlay #(
    parameter MAX_OBJECTS=4,
    parameter SHARED_FONT=0, parameter KIND_BITS=2
) (
    input wire clk_i,
    input wire rst_n_i,
    input wire prepare_i,
    input wire [2:0] count_i,
    input wire [MAX_OBJECTS*KIND_BITS-1:0] kinds_i,
    input wire overflow_i,
    input wire full_i,
    input wire [MAX_OBJECTS*11-1:0] lefts_i,tops_i,widths_i,heights_i,
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
    output wire [63:0] pixels_o,
    output wire [10:0] font_address_o,
    output wire font_enable_o,
    input wire [15:0] font_pixels_i
);
    // Decimal conversion is shared across objects before rendering starts.
    localparam IDLE=0,LOAD=1,CONVERT=2,SAVE=3,READY=4;
    reg [2:0] state,next_state;
    reg [2:0] count;
    reg [MAX_OBJECTS*KIND_BITS-1:0] kinds;
    reg overflow,full;
    reg [2:0] object_index;
    reg [4:0] bit_count;
    reg [10:0] left [0:MAX_OBJECTS-1];
    reg [10:0] top [0:MAX_OBJECTS-1];
    reg [10:0] right [0:MAX_OBJECTS-1];
    reg [10:0] bottom [0:MAX_OBJECTS-1];
    reg [10:0] width [0:MAX_OBJECTS-1];
    reg [10:0] height [0:MAX_OBJECTS-1];
    reg [21:0] area [0:MAX_OBJECTS-1];
    reg [21:0] number_shift [0:4];
    reg [27:0] number_bcd [0:4];
    reg [27:0] corrected_bcd [0:4];
    reg [27:0] center_x_bcd [0:MAX_OBJECTS-1];
    reg [27:0] center_y_bcd [0:MAX_OBJECTS-1];
    reg [27:0] width_bcd [0:MAX_OBJECTS-1];
    reg [27:0] height_bcd [0:MAX_OBJECTS-1];
    reg [27:0] area_bcd [0:MAX_OBJECTS-1];
    integer number,nibble;
    always @(*) begin
        for(number=0;number<5;number=number+1) begin
            corrected_bcd[number]=number_bcd[number];
            for(nibble=0;nibble<7;nibble=nibble+1) begin
                if(number_bcd[number][nibble*4+:4]>=5)
                    corrected_bcd[number][nibble*4+:4]=number_bcd[number][nibble*4+:4]+4'd3;
            end
        end
    end
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) state<=IDLE;
        else state<=next_state;
    end
    always @(*) begin
        next_state=state;
        case(state)
            IDLE,READY: if(prepare_i) next_state=LOAD;
            LOAD: next_state=CONVERT;
            CONVERT: if(bit_count==21) next_state=SAVE;
            SAVE: next_state=object_index==MAX_OBJECTS-1 ? READY : LOAD;
            default: next_state=IDLE;
        endcase
    end
    integer item,field;
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            count<=0;kinds<=0;overflow<=0;full<=0;object_index<=0;bit_count<=0;
            for(item=0;item<MAX_OBJECTS;item=item+1) begin
                left[item]<=0;top[item]<=0;right[item]<=0;bottom[item]<=0;
                width[item]<=0;height[item]<=0;area[item]<=0;
                center_x_bcd[item]<=0;center_y_bcd[item]<=0;width_bcd[item]<=0;height_bcd[item]<=0;area_bcd[item]<=0;
            end
            for(field=0;field<5;field=field+1) begin number_shift[field]<=0;number_bcd[field]<=0;end
        end else begin
            for(item=0;item<MAX_OBJECTS;item=item+1) area[item]<=width[item]*height[item];
            case(state)
                IDLE,READY: if(prepare_i) begin
                    count<=count_i;kinds<=kinds_i;overflow<=overflow_i;full<=full_i;object_index<=0;
                    for(item=0;item<MAX_OBJECTS;item=item+1) begin
                        left[item]<=lefts_i[item*11+:11];top[item]<=tops_i[item*11+:11];
                        width[item]<=widths_i[item*11+:11];height[item]<=heights_i[item*11+:11];
                        right[item]<=lefts_i[item*11+:11]+widths_i[item*11+:11]-1'b1;
                        bottom[item]<=tops_i[item*11+:11]+heights_i[item*11+:11]-1'b1;
                    end
                end
                LOAD: begin
                    number_shift[0]<=left[object_index]+(width[object_index]>>1);
                    number_shift[1]<=top[object_index]+(height[object_index]>>1);
                    number_shift[2]<=width[object_index];number_shift[3]<=height[object_index];
                    // Width/height are stable since PREPARE; use their product here.
                    number_shift[4]<=width[object_index]*height[object_index];
                    bit_count<=0;
                    for(field=0;field<5;field=field+1) number_bcd[field]<=0;
                end
                CONVERT: begin
                    bit_count<=bit_count+1'b1;
                    for(field=0;field<5;field=field+1) begin
                        number_bcd[field]<={corrected_bcd[field][26:0],number_shift[field][21]};
                        number_shift[field]<={number_shift[field][20:0],1'b0};
                    end
                end
                SAVE: begin
                    center_x_bcd[object_index]<=number_bcd[0];center_y_bcd[object_index]<=number_bcd[1];
                    width_bcd[object_index]<=number_bcd[2];height_bcd[object_index]<=number_bcd[3];
                    area_bcd[object_index]<=number_bcd[4];object_index<=object_index+1'b1;
                end
                default: begin end
            endcase
        end
    end
    // A two-stage elastic pixel pipeline retains data and markers under stalls.
    reg [10:0] group_x,pixel_y;
    reg [1:0] valid_pipe,sop_pipe,eop_pipe;
    wire advance=!valid_pipe[1] || ready_i;
    wire [10:0] current_x=sop_i ? 11'd0 : group_x;
    wire [10:0] current_y=sop_i ? 11'd0 : pixel_y;
    // Double each ROM pixel in both axes; keep the original font and pipeline.
    wire [10:0] panel_x=current_x-32;
    wire [10:0] panel_y=current_y-112;
    wire [2:0] text_row=panel_y[7:5];
    wire [5:0] text_column=panel_x[10:5];
    wire in_panel=current_x>=32 && current_x<1568 && current_y>=112 && current_y<112+(MAX_OBJECTS+1)*32;
    wire status_row=panel_y>=MAX_OBJECTS*32;
    wire [KIND_BITS-1:0] text_kind=kinds[text_row*KIND_BITS+:KIND_BITS];
    reg [7:0] glyph;
    reg [15:0] text_color;
    wire [15:0] font_bits;
    always @(*) begin
        glyph=" ";
        text_color=text_kind==1 ? 16'h87f0 : (text_kind==2 ? 16'h87ff : (text_kind==3 ? 16'hfff0 : (text_kind==4 ? 16'hfc1f : (text_kind==5 ? 16'hfdc8 : (text_kind==6 ? 16'h7dff : (text_kind==7 ? 16'hffff : 16'hffe0))))));
        if(status_row) begin
            text_color=overflow ? 16'hfa08 : 16'hffff;
            case(text_column)
                0:glyph="N";1:glyph=":";2:glyph=8'h30+count;
                4:glyph=overflow ? "O" : "O";5:glyph=overflow ? "V" : "K";6:glyph=overflow ? "F" : " ";
                8:glyph=full ? "F" : " ";9:glyph=full ? "U" : " ";10:glyph=full ? "L" : " ";11:glyph=full ? "L" : " ";
                14:glyph="B";15:glyph="B";16:glyph="O";17:glyph="X";19:glyph="P";20:glyph="X";
                default: begin end
            endcase
        end else if(text_row<count) begin
            case(text_column)
                0:glyph=text_kind==7 ? "H" : text_kind==8 ? "S" : text_kind==1 ? "R" : (text_kind==2 ? "C" : (text_kind==3 ? "T" : "N"));
                1:glyph=text_kind==7 ? "E" : text_kind==8 ? "T" : text_kind>3 ? (text_kind==4 ? "2" : (text_kind==5 ? "4" : "7")) : (text_kind==1 ? "E" : (text_kind==2 ? "I" : "R"));
                2:glyph=text_kind==7 ? "X" : text_kind==8 ? "A" : text_kind>3 ? " " : (text_kind==1 ? "C" : (text_kind==2 ? "R" : "I"));
                3:glyph=text_kind==7 ? " " : text_kind==8 ? "R" : text_kind>3 ? " " : (text_kind==1 ? "T" : (text_kind==2 ? "C" : " "));
                4:glyph=8'h31+text_row;
                6:glyph="X";7:glyph=":";
                8:glyph=8'h30+center_x_bcd[text_row][15:12];9:glyph=8'h30+center_x_bcd[text_row][11:8];
                10:glyph=8'h30+center_x_bcd[text_row][7:4];11:glyph=8'h30+center_x_bcd[text_row][3:0];
                13:glyph="Y";14:glyph=":";
                15:glyph=8'h30+center_y_bcd[text_row][15:12];16:glyph=8'h30+center_y_bcd[text_row][11:8];
                17:glyph=8'h30+center_y_bcd[text_row][7:4];18:glyph=8'h30+center_y_bcd[text_row][3:0];
                20:glyph="W";21:glyph=":";
                22:glyph=8'h30+width_bcd[text_row][15:12];23:glyph=8'h30+width_bcd[text_row][11:8];
                24:glyph=8'h30+width_bcd[text_row][7:4];25:glyph=8'h30+width_bcd[text_row][3:0];
                27:glyph="H";28:glyph=":";
                29:glyph=8'h30+height_bcd[text_row][15:12];30:glyph=8'h30+height_bcd[text_row][11:8];
                31:glyph=8'h30+height_bcd[text_row][7:4];32:glyph=8'h30+height_bcd[text_row][3:0];
                34:glyph="A";35:glyph=":";
                36:glyph=8'h30+area_bcd[text_row][27:24];37:glyph=8'h30+area_bcd[text_row][23:20];
                38:glyph=8'h30+area_bcd[text_row][19:16];39:glyph=8'h30+area_bcd[text_row][15:12];
                40:glyph=8'h30+area_bcd[text_row][11:8];41:glyph=8'h30+area_bcd[text_row][7:4];42:glyph=8'h30+area_bcd[text_row][3:0];
                default: begin end
            endcase
        end
    end
    // The external ROM advances only with the same elastic pipeline enable.
    assign font_address_o={glyph[7] ? 7'h3f : glyph[6:0], panel_y[4:1]};
    assign font_enable_o=advance;
    generate if(SHARED_FONT) begin : shared_font
        assign font_bits=font_pixels_i;
    end else begin : local_font
        osd_font_rom font (.clk_i(clk_i),.enable_i(advance),.glyph_id_i(glyph),.glyph_row_i(panel_y[4:1]),.row_bits_o(font_bits));
    end endgenerate
    reg [3:0] box_now,box_delay;
    reg [63:0] color_now,color_delay,input_delay,output_pixels;
    reg panel_delay;
    reg [3:0] glyph_column_delay;
    reg [15:0] text_color_delay;
    reg [11:0] lane_x;
    integer lane,box;
    always @(*) begin
        box_now=0;color_now=0;lane_x=0;
        for(lane=0;lane<4;lane=lane+1) begin
            lane_x={1'b0,current_x}+lane;
            for(box=0;box<MAX_OBJECTS;box=box+1) begin
                if(box<count && lane_x>=left[box] && lane_x<=right[box] && current_y>=top[box] && current_y<=bottom[box]) begin
                    if(lane_x=={1'b0,left[box]} || lane_x==({1'b0,left[box]}+12'd1) || lane_x=={1'b0,right[box]} || (lane_x+12'd1)=={1'b0,right[box]} || current_y==top[box] || current_y==(top[box]+11'd1) || current_y==bottom[box] || (current_y+11'd1)==bottom[box]) begin
                        box_now[lane]=1;
                        color_now[(3-lane)*16+:16]=kinds[box*KIND_BITS+:KIND_BITS]==1 ? 16'h87f0 : (kinds[box*KIND_BITS+:KIND_BITS]==2 ? 16'h87ff : (kinds[box*KIND_BITS+:KIND_BITS]==3 ? 16'hfff0 : (kinds[box*KIND_BITS+:KIND_BITS]==4 ? 16'hfc1f : (kinds[box*KIND_BITS+:KIND_BITS]==5 ? 16'hfdc8 : (kinds[box*KIND_BITS+:KIND_BITS]==6 ? 16'h7dff : (kinds[box*KIND_BITS+:KIND_BITS]==7 ? 16'hffff : 16'hffe0))))));
                    end
                end
            end
        end
    end
    integer output_lane;
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            group_x<=0;pixel_y<=0;valid_pipe<=0;sop_pipe<=0;eop_pipe<=0;
            panel_delay<=0;glyph_column_delay<=0;text_color_delay<=0;box_delay<=0;color_delay<=0;input_delay<=0;output_pixels<=0;
        end else if(advance) begin
            valid_pipe<={valid_pipe[0],valid_i};sop_pipe<={sop_pipe[0],sop_i};eop_pipe<={eop_pipe[0],eop_i};
            panel_delay<=in_panel;glyph_column_delay<=panel_x[4:1];text_color_delay<=text_color;
            box_delay<=box_now;color_delay<=color_now;input_delay<=pixels_i;
            for(output_lane=0;output_lane<4;output_lane=output_lane+1) begin
                output_pixels[(3-output_lane)*16+:16]<=box_delay[output_lane] ? color_delay[(3-output_lane)*16+:16] :
                    (panel_delay && font_bits[15-glyph_column_delay-(output_lane>>1)] ? text_color_delay : input_delay[(3-output_lane)*16+:16]);
            end
            if(valid_i) begin
                if(current_x==1916) begin group_x<=0;pixel_y<=current_y+1'b1;end
                else begin group_x<=current_x+4;pixel_y<=current_y;end
            end
        end
    end
    assign prepared_o=state==READY;
    assign ready_o=advance;
    assign valid_o=valid_pipe[1];
    assign sop_o=sop_pipe[1];
    assign eop_o=eop_pipe[1];
    assign pixels_o=output_pixels;
    assign frame_o=frame_i || (|valid_pipe);
endmodule
