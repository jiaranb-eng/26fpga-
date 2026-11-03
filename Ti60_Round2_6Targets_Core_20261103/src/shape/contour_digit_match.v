`timescale 1ns/1ps
// Normalized 12x16 component contours, serial comparison of digits 2, 4 and 7.
// Original geometric decisions are evaluated before this optional fallback.
module contour_digit_match #(
    parameter EXTENDED_SHAPES=0, parameter KIND_BITS=EXTENDED_SHAPES ? 4 : 3,
    parameter TEMPLATE_FILE="src/shape/contour_templates_packed.mem"
) (
    input wire clk_i,input wire rst_n_i,input wire start_i,input wire cancel_i,
    input wire [15:0] count_i,input wire geometry_only_i,
    input wire [7:0] left_i,right_i,top_i,bottom_i,
    output wire [10:0] address_o,input wire [15:0] point_i,
    output wire done_o,output wire [KIND_BITS-1:0] kind_o
);
    localparam IDLE=0,POINT_REQUEST=1,POINT_WAIT=2,POINT_LOAD=3,
        NORMALIZE=4,STORE=5,ASPECT=6,TEMPLATE_START=7,BIT_REQUEST=8,
        BIT_SCORE=9,TEMPLATE_SAVE=10,NEXT_TEMPLATE=11,DECIDE=12,FINISH=13;
    reg [3:0] state,next_state;
    reg [191:0] component_bits,compare_bits;
    reg [11:0] point_index,point_count;
    reg [7:0] origin_x,origin_y;
    reg [8:0] span_x,span_y;
    reg [12:0] numerator_x,numerator_y;
    reg [3:0] cell_x,cell_y;
    reg [14:0] aspect_remaining;
    reg [7:0] aspect;
    reg [5:0] template_index;
    reg [7:0] bit_index;
    reg [10:0] rom_word_address;reg [2:0] rom_lane;
    // Five-bit words use every data bit of one 10K block (53x192 bits).
    (* syn_romstyle="logic" *) reg [4:0] template_memory[0:2035];
    reg [4:0] reference_word;
    reg [2:0] reference_lane;
    wire reference_bit=reference_word[reference_lane];
    reg [7:0] reference_aspect;
    reg [10:0] score;
    reg [10:0] score_two,score_four,score_seven;
    reg [KIND_BITS-1:0] result_kind;
    reg [10:0] score_hexagon,score_star,score_veto;
    wire [7:0] aspect_difference=aspect>=reference_aspect ? aspect-reference_aspect : reference_aspect-aspect;
    wire [10:0] aspect_penalty={3'd0,aspect_difference}+{2'd0,aspect_difference,1'b0};
    wire normalize_done=numerator_x<span_x && numerator_y<span_y;
    wire [7:0] normalized_address={cell_y,3'b000}+{cell_y,2'b00}+cell_x;
    initial $readmemb(TEMPLATE_FILE,template_memory);
    always @(posedge clk_i) begin
        reference_word<=template_memory[rom_word_address];
        reference_lane<=rom_lane;
    end
    always @(*) begin
        reference_aspect=0;
        case(template_index)
            6'd0: reference_aspect=8'd45;
            6'd1: reference_aspect=8'd43;
            6'd2: reference_aspect=8'd45;
            6'd3: reference_aspect=8'd44;
            6'd4: reference_aspect=8'd42;
            6'd5: reference_aspect=8'd44;
            6'd6: reference_aspect=8'd42;
            6'd7: reference_aspect=8'd44;
            6'd8: reference_aspect=8'd42;
            6'd9: reference_aspect=8'd48;
            6'd10: reference_aspect=8'd48;
            6'd11: reference_aspect=8'd48;
            6'd12: reference_aspect=8'd46;
            6'd13: reference_aspect=8'd47;
            6'd14: reference_aspect=8'd47;
            6'd15: reference_aspect=8'd47;
            6'd16: reference_aspect=8'd45;
            6'd17: reference_aspect=8'd47;
            6'd18: reference_aspect=8'd45;
            6'd19: reference_aspect=8'd45;
            6'd20: reference_aspect=8'd45;
            6'd21: reference_aspect=8'd44;
            6'd22: reference_aspect=8'd45;
            6'd23: reference_aspect=8'd43;
            6'd24: reference_aspect=8'd43;
            6'd25: reference_aspect=8'd43;
            6'd26: reference_aspect=8'd44;
            6'd27: reference_aspect=8'd72;
            6'd28: reference_aspect=8'd72;
            6'd29: reference_aspect=8'd67;
            6'd30: reference_aspect=8'd67;
            6'd31: reference_aspect=8'd60;
            6'd32: reference_aspect=8'd60;
            6'd33: reference_aspect=8'd56;
            6'd34: reference_aspect=8'd56;
            6'd35: reference_aspect=8'd60;
            6'd36: reference_aspect=8'd60;
            6'd37: reference_aspect=8'd67;
            6'd38: reference_aspect=8'd67;
            6'd39: reference_aspect=8'd67;
            6'd40: reference_aspect=8'd67;
            6'd41: reference_aspect=8'd62;
            6'd42: reference_aspect=8'd62;
            6'd43: reference_aspect=8'd65;
            6'd44: reference_aspect=8'd65;
            6'd45: reference_aspect=8'd65;
            6'd46: reference_aspect=8'd65;
            6'd47: reference_aspect=8'd60;
            6'd48: reference_aspect=8'd62;
            6'd49: reference_aspect=8'd64;
            6'd50: reference_aspect=8'd64;
            6'd51: reference_aspect=8'd65;
            6'd52: reference_aspect=8'd64;
            default: reference_aspect=0;
        endcase
    end
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) state<=IDLE;
        else if(cancel_i) state<=IDLE;
        else state<=next_state;
    end
    always @(*) begin
        next_state=state;
        case(state)
            IDLE:if(start_i) next_state=count_i==0 ? FINISH : POINT_REQUEST;
            POINT_REQUEST:next_state=POINT_WAIT;
            POINT_WAIT:next_state=POINT_LOAD;
            POINT_LOAD:next_state=NORMALIZE;
            NORMALIZE:if(normalize_done) next_state=STORE;
            STORE:next_state=point_index+1==point_count ? ASPECT : POINT_REQUEST;
            ASPECT:if(aspect_remaining<span_y || aspect==255) next_state=TEMPLATE_START;
            TEMPLATE_START:next_state=BIT_REQUEST;
            BIT_REQUEST:next_state=BIT_SCORE;
            BIT_SCORE:next_state=bit_index==191 ? TEMPLATE_SAVE : BIT_REQUEST;
            TEMPLATE_SAVE:next_state=NEXT_TEMPLATE;
            NEXT_TEMPLATE:next_state=template_index==(EXTENDED_SHAPES ? 52 : 26) ? DECIDE : TEMPLATE_START;
            DECIDE:next_state=FINISH;
            FINISH:next_state=IDLE;
            default:next_state=IDLE;
        endcase
    end
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            component_bits<=0;compare_bits<=0;point_index<=0;point_count<=0;
            origin_x<=0;origin_y<=0;span_x<=1;span_y<=1;
            numerator_x<=0;numerator_y<=0;cell_x<=0;cell_y<=0;
            aspect_remaining<=0;aspect<=0;template_index<=0;bit_index<=0;rom_word_address<=0;rom_lane<=0;
            score_hexagon<=2047;score_star<=2047;score_veto<=2047;
            score<=0;score_two<=2047;score_four<=2047;score_seven<=2047;result_kind<=0;
        end else if(!cancel_i) begin
            case(state)
                IDLE:if(start_i) begin
                    component_bits<=0;point_index<=0;point_count<=count_i[11:0];
                    origin_x<=left_i;origin_y<=top_i;
                    span_x<={1'b0,right_i}-{1'b0,left_i}+9'd1;
                    span_y<={1'b0,bottom_i}-{1'b0,top_i}+9'd1;
                    aspect_remaining<=({1'b0,right_i}-{1'b0,left_i}+9'd1)<<6;
                    aspect<=0;template_index<=(EXTENDED_SHAPES && geometry_only_i) ? 6'd27 : 6'd0;
                    rom_word_address<=(EXTENDED_SHAPES && geometry_only_i) ? 11'd1036 : 11'd0;
                    rom_lane<=(EXTENDED_SHAPES && geometry_only_i) ? 3'd4 : 3'd0;
                    score_two<=2047;score_four<=2047;score_seven<=2047;result_kind<=0;
                    score_hexagon<=2047;score_star<=2047;score_veto<=2047;
                end
                POINT_LOAD:begin
                    numerator_x<=({5'd0,point_i[7:0]}-{5'd0,origin_x})*13'd12;
                    numerator_y<=({5'd0,point_i[15:8]}-{5'd0,origin_y})<<4;
                    cell_x<=0;cell_y<=0;
                end
                NORMALIZE:begin
                    if(numerator_x>=span_x) begin numerator_x<=numerator_x-span_x;cell_x<=cell_x+1'b1;end
                    if(numerator_y>=span_y) begin numerator_y<=numerator_y-span_y;cell_y<=cell_y+1'b1;end
                end
                STORE:begin component_bits[normalized_address]<=1'b1;point_index<=point_index+1'b1;end
                ASPECT:if(aspect_remaining>=span_y && aspect!=255) begin
                    aspect_remaining<=aspect_remaining-span_y;aspect<=aspect+1'b1;
                end
                TEMPLATE_START:begin compare_bits<=component_bits;score<=aspect_penalty;bit_index<=0;end
                BIT_SCORE:begin
                    score<=score+(compare_bits[0]^reference_bit);
                    compare_bits<=compare_bits>>1;bit_index<=bit_index+1'b1;if(rom_lane==4) begin rom_lane<=0;rom_word_address<=rom_word_address+1'b1;end
                    else rom_lane<=rom_lane+1'b1;
                end
                TEMPLATE_SAVE:begin
                    if(template_index<9 && score<score_two) score_two<=score;
                    else if(template_index>=9 && template_index<18 && score<score_four) score_four<=score;
                    else if(template_index>=18 && template_index<27 && score<score_seven) score_seven<=score;
                    else if(template_index>=27 && template_index<39 && score<score_hexagon) score_hexagon<=score;
                    else if(template_index>=39 && template_index<49 && score<score_star) score_star<=score;
                    else if(template_index>=49 && score<score_veto) score_veto<=score;
                end
                NEXT_TEMPLATE:template_index<=template_index+1'b1;
                DECIDE:begin
                    if(score_two<=55 && score_two+8<=score_four && score_two+8<=score_seven) result_kind<=4;
                    else if(score_four<=55 && score_four+8<=score_two && score_four+8<=score_seven) result_kind<=5;
                    else if(score_seven<=55 && score_seven+8<=score_two && score_seven+8<=score_four) result_kind<=6;
                    else if(EXTENDED_SHAPES && score_hexagon<=32 && score_hexagon+6<=score_star && score_hexagon<=score_veto+8) result_kind<=7;
                    else if(EXTENDED_SHAPES && score_star<=42 && score_star+6<=score_hexagon && score_star+5<=score_veto) result_kind<=8;
                end
                default:begin end
            endcase
        end
    end
    assign address_o=point_index[10:0];
    assign done_o=state==FINISH;
    assign kind_o=result_kind;
endmodule
