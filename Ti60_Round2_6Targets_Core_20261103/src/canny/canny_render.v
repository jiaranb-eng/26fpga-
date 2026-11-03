`timescale 1ns/1ps
// Preserve the full 1920x1200 field of view without stretching it. The image is
// scaled by nearest-neighbor sampling and centered with black side bars.
// 720p: 1152x720 + 64-pixel bars; 1080p: 1728x1080 + 96-pixel bars.
// Four RGB565 output pixels per camera clock, earliest pixel in high bits.
module canny_render #(
    parameter FULL_SCAN_ENABLE=0,
    parameter SUPPORT_GRAY=0,
    parameter SOURCE_WIDTH=1920,
    parameter SOURCE_HEIGHT=1200,
    parameter DISPLAY_WIDTH=1280,
    parameter DISPLAY_HEIGHT=720,
    parameter SCALE_NUM=5,
    parameter SCALE_DEN=3,
    parameter ROW_STRIDE=SOURCE_WIDTH/4
) (
    input wire full_scan_i,
    input wire gray_i,
    input wire clk_i, input wire rst_n_i,
    input wire start_i, input wire [31:0] base_address_i,
    output wire busy_o, output wire done_o,
    output wire read_request_o, input wire read_request_ready_i,
    output wire [31:0] read_address_o,
    input wire read_valid_i, output wire read_ready_o, input wire [63:0] read_data_i,
    output wire frame_active_o, output wire valid_o, input wire ready_i,
    output wire sop_o, output wire eop_o, output wire [63:0] rgb565_o
);
    localparam ROW_WORDS=SOURCE_WIDTH/32;
    localparam CACHE_WORDS=SUPPORT_GRAY ? SOURCE_WIDTH/8 : ROW_WORDS;
    wire gray_mode=SUPPORT_GRAY && gray_i;
    wire [8:0] load_words=gray_mode ? SOURCE_WIDTH/8 : ROW_WORDS;
    localparam ROW_BYTES=ROW_STRIDE;
    localparam ACTIVE_WIDTH=SOURCE_WIDTH*SCALE_DEN/SCALE_NUM;
    localparam LEFT_GROUPS=(DISPLAY_WIDTH-ACTIVE_WIDTH)/8;
    localparam ACTIVE_GROUPS=ACTIVE_WIDTH/4;
    localparam GROUPS=DISPLAY_WIDTH/4;
    localparam IDLE=0, REQUEST=1, LOAD=2, PIXELS=3, FLUSH_FIRST=4, FLUSH_LAST=5, NEXT_ROW=6, FINISH=7;
    reg [2:0] state,next_state;
    reg [7:0] load_index;
    reg [10:0] group_index,display_row,source_column;
    reg [3:0] x_phase,y_phase;
    reg [31:0] row_address;
    localparam CACHE_BITS=SUPPORT_GRAY ? 64 : 32;
    wire [CACHE_BITS-1:0] first_read,second_read;
    wire [7:0] cache_address=source_column>=SOURCE_WIDTH ? load_words-1'b1 : (gray_mode ? source_column[10:3] : {2'b00,source_column[10:5]});
    wire [7:0] adjacent_address=cache_address==load_words-1 ? cache_address : cache_address+1'b1;
    wire full_scan=FULL_SCAN_ENABLE && full_scan_i;
    wire [10:0] last_display_row=full_scan ? SOURCE_HEIGHT-1 : DISPLAY_HEIGHT-1;
    wire active_group=full_scan || (group_index>=LEFT_GROUPS && group_index<LEFT_GROUPS+ACTIVE_GROUPS);
    wire [7:0] x_advance=full_scan ? 4 : (x_phase+4*SCALE_NUM)/SCALE_DEN;
    wire [3:0] x_next_phase=full_scan ? 0 : (x_phase+4*SCALE_NUM)%SCALE_DEN;
    wire [3:0] y_advance=full_scan ? 1 : (y_phase+SCALE_NUM)/SCALE_DEN;
    wire [3:0] y_next_phase=full_scan ? 0 : (y_phase+SCALE_NUM)%SCALE_DEN;
    reg active_delay;
    reg frame_active;
    reg [1:0] valid_pipe,sop_pipe,eop_pipe;
    wire [2*CACHE_BITS-1:0] cached_pixels={second_read,first_read};
    wire advance=!valid_pipe[1] || ready_i;
    generate if(SUPPORT_GRAY) begin : gray_cache
        // Retained for projects that explicitly enable the legacy gray mode.
        (* ram_style="block" *) reg [63:0] first_cache [0:CACHE_WORDS-1];
        (* ram_style="block" *) reg [63:0] second_cache [0:CACHE_WORDS-1];
        reg [63:0] first_data,second_data;
        always @(posedge clk_i) if(advance) begin
            if(state==LOAD && read_valid_i) begin
                first_cache[load_index]<=read_data_i;
                second_cache[load_index]<=read_data_i;
            end
            first_data<=first_cache[cache_address];
            second_data<=second_cache[adjacent_address];
        end
        assign first_read=first_data;
        assign second_read=second_data;
    end else begin : binary_cache
        // HDMI binary output needs only the connected bit, not eligibility.
        // Bank even/odd words once instead of duplicating a whole source row.
        localparam BANK_WORDS=(ROW_WORDS+1)/2;
        (* ram_style="block" *) reg [31:0] even_cache [0:BANK_WORDS-1];
        (* ram_style="block" *) reg [31:0] odd_cache [0:BANK_WORDS-1];
        reg [31:0] even_read,odd_read;
        reg word_odd,same_word;
        wire [31:0] connected_word;
        wire [7:0] even_address=(cache_address[0] ? adjacent_address : cache_address)>>1;
        wire [7:0] odd_address=(cache_address[0] ? cache_address : adjacent_address)>>1;
        genvar edge_pixel;
        for(edge_pixel=0;edge_pixel<32;edge_pixel=edge_pixel+1) begin : connected_bits
            assign connected_word[edge_pixel]=read_data_i[edge_pixel*2+1];
        end
        always @(posedge clk_i) if(advance) begin
            if(state==LOAD && read_valid_i) begin
                if(load_index[0]) odd_cache[load_index>>1]<=connected_word;
                else even_cache[load_index>>1]<=connected_word;
            end
            even_read<=even_cache[even_address];
            odd_read<=odd_cache[odd_address];
            word_odd<=cache_address[0];
            same_word<=cache_address==adjacent_address;
        end
        assign first_read=word_odd ? odd_read : even_read;
        assign second_read=same_word ? first_read : (word_odd ? even_read : odd_read);
    end endgenerate
    genvar lane;
    generate for(lane=0;lane<4;lane=lane+1) begin : display_pixel
        wire [3:0] offset=full_scan ? lane : (x_phase+lane*SCALE_NUM)/SCALE_DEN;
        reg [5:0] pixel_index;
        reg [15:0] output_pixel;
        wire [7:0] gray_value=cached_pixels[pixel_index*8+:8];
        wire connected_value=SUPPORT_GRAY ? cached_pixels[pixel_index*2+1] : cached_pixels[pixel_index];
        always @(posedge clk_i) if(advance) begin
            pixel_index<=(gray_mode ? {3'b000,source_column[2:0]} : {1'b0,source_column[4:0]})+offset;
            output_pixel<=!active_delay ? 16'h0000 : (gray_mode ? {gray_value[7:3],gray_value[7:2],gray_value[7:3]} : (connected_value ? 16'hffff : 16'h0000));
        end
        assign rgb565_o[(3-lane)*16+:16]=output_pixel;
    end endgenerate
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin state<=IDLE; valid_pipe<=0; sop_pipe<=0; eop_pipe<=0; active_delay<=0; frame_active<=0; end
        else if(advance) begin
            state<=next_state;
            // A binary state decode can glitch during PIXELS -> FLUSH_FIRST
            // (011 -> 100). This frame marker crosses into the AXI domain;
            // hold it in a flip-flop for the entire accepted output frame.
            if(state==IDLE && start_i) frame_active<=1'b1;
            else if(state==NEXT_ROW && display_row==last_display_row) frame_active<=1'b0;
            valid_pipe<={valid_pipe[0],state==PIXELS};
            sop_pipe<={sop_pipe[0],state==PIXELS && display_row==0 && group_index==0};
            eop_pipe<={eop_pipe[0],state==PIXELS && display_row==last_display_row && group_index==GROUPS-1};
            active_delay<=active_group;
        end
    end
    always @(*) begin
        next_state=state;
        case(state)
            IDLE: if(start_i) next_state=REQUEST;
            REQUEST: if(read_request_ready_i) next_state=LOAD;
            LOAD: if(read_valid_i && load_index==load_words-1) next_state=PIXELS;
            PIXELS: if(group_index==GROUPS-1) next_state=FLUSH_FIRST;
            FLUSH_FIRST: next_state=FLUSH_LAST;
            FLUSH_LAST: next_state=NEXT_ROW;
            NEXT_ROW: next_state=display_row==last_display_row ? FINISH : REQUEST;
            FINISH: next_state=IDLE;
            default: next_state=IDLE;
        endcase
    end
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            load_index<=0; group_index<=0; display_row<=0; source_column<=0;
            x_phase<=0; y_phase<=0; row_address<=0;
        end else if(advance) begin
            case(state)
                IDLE: if(start_i) begin
                    load_index<=0; group_index<=0; display_row<=0; source_column<=0;
                    x_phase<=0; y_phase<=0; row_address<=base_address_i;
                end
                LOAD: if(read_valid_i) load_index<=load_index==load_words-1 ? 0 : load_index+1'b1;
                PIXELS: begin
                    group_index<=group_index+1'b1;
                    if(active_group) begin source_column<=source_column+x_advance; x_phase<=x_next_phase; end
                end
                NEXT_ROW: begin
                    display_row<=display_row+1'b1; group_index<=0; source_column<=0; x_phase<=0;
                    y_phase<=y_next_phase; row_address<=row_address+(gray_mode ? {y_advance,11'd0} : y_advance*ROW_BYTES);
                end
                default: begin end
            endcase
        end
    end
    assign busy_o=state!=IDLE;
    assign done_o=state==FINISH && advance;
    assign read_request_o=state==REQUEST && advance;
    assign read_address_o=row_address;
    assign read_ready_o=state==LOAD && advance;
    assign frame_active_o=frame_active;
    assign valid_o=valid_pipe[1];
    assign sop_o=sop_pipe[1];
    assign eop_o=eop_pipe[1];
endmodule
