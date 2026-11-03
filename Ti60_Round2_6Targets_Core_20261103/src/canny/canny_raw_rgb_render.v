`timescale 1ns/1ps
// RAW workspace -> four-pixel Bayer interpolation -> full-field RGB565.
// DDR prefetch overlaps processing. Two words are staged in registers;
// centered-window line buffers are on chip, all complete frames are in DDR.
module canny_raw_rgb_render #(
    parameter SUPPORT_COMPARE=0,
    parameter SOURCE_WIDTH=1920, SOURCE_HEIGHT=1200,
    parameter DISPLAY_WIDTH=1920, DISPLAY_HEIGHT=1080,
    parameter SCALE_NUM=10, SCALE_DEN=9
) (
    input wire compare_i,
    input wire clk_i, input wire rst_n_i,
    input wire start_i, input wire [31:0] base_address_i,
    input wire background_enable_i, input wire [7:0] background_boxes_i,
    input wire [87:0] background_left_i, input wire [87:0] background_top_i,
    input wire [87:0] background_right_i, input wire [87:0] background_bottom_i,
    output wire done_o,
    output wire read_request_o, input wire read_request_ready_i,
    output wire [31:0] read_address_o,
    input wire read_valid_i, output wire read_ready_o, input wire [63:0] read_data_i,
    output reg frame_active_o, output reg valid_o, input wire ready_i,
    output reg [63:0] rgb565_o
);
    wire compare_mode=SUPPORT_COMPARE && compare_i;
    localparam ROW_WORDS=SOURCE_WIDTH/8, GROUPS=SOURCE_WIDTH/4;
    localparam FRAME_GROUPS=GROUPS*SOURCE_HEIGHT;
    localparam ACTIVE_WIDTH=SOURCE_WIDTH*SCALE_DEN/SCALE_NUM;
    localparam BAR_GROUPS=(DISPLAY_WIDTH-ACTIVE_WIDTH)/8;
    localparam IDLE=0,RUN=1,LEFT=2,RIGHT=3,DRAIN=4,FINISH=5;
    reg [2:0] state;
    reg [31:0] fetch_address;
    reg [10:0] fetch_row;
    reg [8:0] fetch_remaining;
    reg [1:0] queue_count;
    reg [63:0] head_word,tail_word;
    reg input_half;
    reg [20:0] input_groups;
    reg [10:0] rgb_group,rgb_row;
    reg [3:0] x_phase,y_phase;
    reg left_done,last_row;
    reg [7:0] bar_index;
    reg [1:0] pack_count;
    reg [63:0] pack_data;
    wire output_space=!valid_o || ready_i;
    wire selected_row=((y_phase*SCALE_DEN+SCALE_NUM-1)/SCALE_NUM)<(((y_phase+1)*SCALE_DEN+SCALE_NUM-1)/SCALE_NUM);
    wire rgb_valid,rgb_eop;
    wire [63:0] rgb_pixels;
    wire need_left=rgb_valid && rgb_group==0 && selected_row && !left_done && BAR_GROUPS!=0;
    wire step=state==RUN && output_space && !need_left;
    wire compare_available,compare_accept,compare_request_space;
    wire [31:0] compare_head;
    wire input_valid=(compare_mode ? compare_available : queue_count!=0) && input_groups<FRAME_GROUPS;
    wire input_fire=step && input_valid;
    wire pop_word=input_fire && input_half && !compare_mode;
    wire push_word=read_valid_i && read_ready_o;
    wire legacy_push=push_word && !compare_mode;
    wire fetch_fire=read_request_o && read_request_ready_i;
    wire bayer_reset=rst_n_i && frame_active_o;
    sc233_rgb_quad #(.WIDTH(SOURCE_WIDTH),.HEIGHT(SOURCE_HEIGHT)) bayer (
        .clk_i(clk_i),.rst_n_i(bayer_reset),.ce_i(step),.valid_i(input_valid),
        .sop_i(input_groups==0),.eop_i(input_groups==FRAME_GROUPS-1),
        .raw_i(compare_mode ? compare_head : (input_half ? head_word[63:32] : head_word[31:0])),
        .valid_o(rgb_valid),.eop_o(rgb_eop),.rgb565_o(rgb_pixels)
    );
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            fetch_address<=0;fetch_row<=0;fetch_remaining<=0;queue_count<=0;
            head_word<=0;tail_word<=0;input_half<=0;input_groups<=0;
        end else if(state==IDLE && start_i) begin
            fetch_address<=base_address_i;fetch_row<=0;fetch_remaining<=0;
            queue_count<=0;input_half<=0;input_groups<=0;
        end else begin
            if(fetch_fire) begin fetch_address<=fetch_address+32'd2048;fetch_row<=fetch_row+1'b1;fetch_remaining<=ROW_WORDS;end
            else if(push_word) fetch_remaining<=fetch_remaining-1'b1;
            if(input_fire) begin input_half<=!input_half;input_groups<=input_groups+1'b1;end
            case({legacy_push,pop_word})
                2'b10: begin
                    if(queue_count==0) head_word<=read_data_i;else tail_word<=read_data_i;
                    queue_count<=queue_count+1'b1;
                end
                2'b01: begin head_word<=tail_word;queue_count<=queue_count-1'b1;end
                2'b11: begin head_word<=queue_count==1 ? read_data_i : tail_word;tail_word<=read_data_i;end
                default: begin end
            endcase
        end
    end
    // Compact selected lanes into a pixel queue, then emit four at a time.
    reg [111:0] assembled;
    reg [2:0] assembled_count;
    integer lane,phase,object_index;
    reg [10:0] sample_x;
    reg box_border;
    reg [15:0] sample_pixel;
    always @(*) begin
        assembled={48'd0,pack_data};assembled_count={1'b0,pack_count};phase=0;sample_pixel=0;sample_x=0;box_border=0;
        for(lane=0;lane<4;lane=lane+1) begin
            phase=(x_phase+lane)%SCALE_NUM;
            if(selected_row && (((phase*SCALE_DEN+SCALE_NUM-1)/SCALE_NUM)<(((phase+1)*SCALE_DEN+SCALE_NUM-1)/SCALE_NUM))) begin
                sample_pixel=(rgb_row<2 || rgb_row>=SOURCE_HEIGHT-2 || (rgb_group==0 && lane<2) || (rgb_group==GROUPS-1 && lane>=2)) ? 16'd0 : rgb_pixels[lane*16+:16];
                // 框在源坐标叠加后与彩色像素一起缩放，历史帧保留对应帧的框。
                sample_x=rgb_group*4+lane;
                box_border=0;
                for(object_index=0;object_index<8;object_index=object_index+1) begin
                    if(background_boxes_i[object_index] &&
                       sample_x>=background_left_i[object_index*11+:11] && sample_x<=background_right_i[object_index*11+:11] &&
                       rgb_row>=background_top_i[object_index*11+:11] && rgb_row<=background_bottom_i[object_index*11+:11] &&
                       (sample_x<background_left_i[object_index*11+:11]+11'd3 || sample_x+11'd3>background_right_i[object_index*11+:11] ||
                        rgb_row<background_top_i[object_index*11+:11]+11'd3 || rgb_row+11'd3>background_bottom_i[object_index*11+:11])) box_border=1;
                end
                if(background_enable_i && box_border) sample_pixel=16'h07e0;
                assembled[assembled_count*16+:16]=sample_pixel;
                assembled_count=assembled_count+1'b1;
            end
        end
    end
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            state<=IDLE;frame_active_o<=0;valid_o<=0;rgb565_o<=0;
            rgb_group<=0;rgb_row<=0;x_phase<=0;y_phase<=0;
            left_done<=0;last_row<=0;bar_index<=0;pack_count<=0;pack_data<=0;
        end else begin
            if(valid_o && ready_i) valid_o<=0;
            if(output_space) case(state)
                IDLE: if(start_i) begin
                    frame_active_o<=1;rgb_group<=0;rgb_row<=0;x_phase<=0;y_phase<=0;
                    left_done<=0;last_row<=0;bar_index<=0;pack_count<=0;pack_data<=0;state<=RUN;
                end
                RUN: if(need_left) begin bar_index<=0;state<=LEFT;end
                else if(rgb_valid) begin
                    if(assembled_count>=4) begin
                        rgb565_o<={assembled[15:0],assembled[31:16],assembled[47:32],assembled[63:48]};
                        valid_o<=1;pack_data<={16'd0,assembled[111:64]};pack_count<=assembled_count-4;
                    end else begin pack_data<=assembled[63:0];pack_count<=assembled_count[1:0];end
                    if(rgb_group==GROUPS-1) begin
                        rgb_group<=0;rgb_row<=rgb_row+1'b1;x_phase<=0;y_phase<=y_phase==SCALE_NUM-1 ? 0 : y_phase+1'b1;
                        left_done<=0;last_row<=rgb_eop;
                        if(selected_row && BAR_GROUPS!=0) begin bar_index<=0;state<=RIGHT;end
                        else if(rgb_eop) state<=DRAIN;
                    end else begin rgb_group<=rgb_group+1'b1;x_phase<=(x_phase+4)%SCALE_NUM;end
                end
                LEFT: begin
                    rgb565_o<=0;valid_o<=1;bar_index<=bar_index+1'b1;
                    if(bar_index==BAR_GROUPS-1) begin left_done<=1;state<=RUN;end
                end
                RIGHT: begin
                    rgb565_o<=0;valid_o<=1;bar_index<=bar_index+1'b1;
                    if(bar_index==BAR_GROUPS-1) state<=last_row ? DRAIN : RUN;
                end
                DRAIN: begin frame_active_o<=0;state<=FINISH;end
                FINISH: state<=IDLE;
                default: state<=IDLE;
            endcase
        end
    end
    generate if(SUPPORT_COMPARE) begin : paired_prefetch
        // Serialize each 64-bit response into two 32-bit entries. A whole
        // outstanding RAW row always fits, even while its output is blocked.
        (* syn_ramstyle="block_ram" *) reg [31:0] memory[0:511];
        reg [8:0] read_pointer,write_pointer;
        reg [9:0] count;
        reg upper_pending,head_valid;
        reg [31:0] upper_word,head;
        wire take=compare_mode && input_fire;
        wire push=compare_mode && push_word;
        wire write_entry=upper_pending || push;
        wire [8:0] read_address=take ? read_pointer+1'b1 : read_pointer;
        assign compare_head=head;
        assign compare_available=count!=0 && head_valid;
        assign compare_accept=!upper_pending && count<=510;
        assign compare_request_space=!upper_pending && count<=512-GROUPS;
        always @(posedge clk_i) begin
            head<=memory[read_address];
            if(write_entry) memory[write_pointer]<=upper_pending ? upper_word : read_data_i[31:0];
        end
        always @(posedge clk_i or negedge rst_n_i) begin
            if(!rst_n_i) begin read_pointer<=0;write_pointer<=0;count<=0;upper_pending<=0;head_valid<=0;upper_word<=0;end
            else if(state==IDLE && start_i) begin read_pointer<=0;write_pointer<=0;count<=0;upper_pending<=0;head_valid<=0;end
            else begin
                head_valid<=count!=0 && !(take && count==1);
                if(take) read_pointer<=read_pointer+1'b1;
                if(write_entry) write_pointer<=write_pointer+1'b1;
                if(upper_pending) upper_pending<=0;
                if(push) begin upper_word<=read_data_i[63:32];upper_pending<=1;end
                case({write_entry,take})
                    2'b10:count<=count+1'b1;
                    2'b01:count<=count-1'b1;
                    default:begin end
                endcase
            end
        end
    end else begin : no_pair_prefetch
        assign compare_head=0;assign compare_available=0;assign compare_accept=0;assign compare_request_space=0;
    end endgenerate
    assign done_o=state==FINISH && output_space;
    assign read_request_o=frame_active_o && fetch_row<SOURCE_HEIGHT && fetch_remaining==0 && (!compare_mode || compare_request_space);
    assign read_address_o=fetch_address;
    assign read_ready_o=fetch_remaining!=0 && (compare_mode ? compare_accept : (queue_count<2 || pop_word));
endmodule
