`timescale 1ns / 1ps
// Coverage-preserving binary reduction, one or two source samples per axis.
// 二值区域缩放：每个目标格收集完整源区间，不跳过细边所在的行列。
module canny_coverage_render #(
    parameter MAX_OBJECTS=4,parameter LABEL_BITS=MAX_OBJECTS>4 ? 4 : 3,
    parameter SUPPORT_COMPARE=0,
    parameter KIND_BITS=3, parameter SUPPORT_LABELS=0,
    parameter SOURCE_WIDTH = 1920,
    parameter SOURCE_HEIGHT = 1200,
    parameter DISPLAY_WIDTH = 1920,
    parameter DISPLAY_HEIGHT = 1080,
    parameter SCALE_NUM = 10,
    parameter SCALE_DEN = 9,
    parameter ROW_STRIDE = 512
) (
    input wire compare_i,output wire compare_left_o,
    input wire color_enable_i,input wire [MAX_OBJECTS*KIND_BITS-1:0] object_kind_i,
    output wire [13:0] label_address_o,input wire [4*LABEL_BITS-1:0] label_data_i,
    input wire clk_i,
    input wire rst_n_i,
    input wire start_i,
    input wire [31:0] base_address_i,
    output wire busy_o,
    output wire done_o,
    output wire read_request_o,
    input wire read_request_ready_i,
    output wire [31:0] read_address_o,
    input wire read_valid_i,
    output wire read_ready_o,
    input wire [63:0] read_data_i,
    output wire frame_active_o,
    output wire valid_o,
    input wire ready_i,
    output wire sop_o,
    output wire eop_o,
    output wire [63:0] rgb565_o
);
    wire compare_mode=SUPPORT_COMPARE && compare_i;
    reg [1:0] compare_left_pipe;
    assign compare_left_o=compare_left_pipe[1];
    localparam CACHE_BITS=SUPPORT_LABELS ? 32+4*LABEL_BITS : 32;
    localparam ROW_WORDS = SOURCE_WIDTH / 32;
    localparam BANK_WORDS = (ROW_WORDS + 1) / 2;
    localparam ACTIVE_WIDTH = SOURCE_WIDTH * SCALE_DEN / SCALE_NUM;
    localparam LEFT_GROUPS = (DISPLAY_WIDTH - ACTIVE_WIDTH) / 8;
    localparam ACTIVE_GROUPS = ACTIVE_WIDTH / 4;
    localparam DISPLAY_GROUPS = DISPLAY_WIDTH / 4;
    localparam ST_IDLE = 0, ST_REQUEST = 1, ST_LOAD = 2, ST_SETTLE = 3;
    localparam ST_PIXELS = 4, ST_FLUSH = 5, ST_DRAIN = 6, ST_NEXT = 7, ST_FINISH = 8;
    reg [3:0] state, next_state;
    reg [7:0] load_index;
    reg [10:0] group_index, display_row, source_column;
    reg [3:0] x_phase, y_phase;
    reg [10:0] source_row;
    assign label_address_o=(source_row>>3)*ROW_WORDS+load_index;
    reg [31:0] row_address;
    reg merge_row;
    wire [3:0] row_span = (y_phase + SCALE_NUM) / SCALE_DEN;
    wire [3:0] next_y_phase = (y_phase + SCALE_NUM) % SCALE_DEN;
    wire [3:0] column_advance = (x_phase + 4*SCALE_NUM) / SCALE_DEN;
    wire [3:0] next_x_phase = (x_phase + 4*SCALE_NUM) % SCALE_DEN;
    wire active_row=1'b1;
    wire active_group = group_index >= LEFT_GROUPS && group_index < LEFT_GROUPS + ACTIVE_GROUPS;
    reg [1:0] valid_pipe, sop_pipe, eop_pipe;
    reg active_delay;
    reg frame_active_o_reg;
    wire advance = !valid_pipe[1] || ready_i;
    wire load_fire = state == ST_LOAD && read_valid_i;
    wire [7:0] cache_address = source_column >= SOURCE_WIDTH ? ROW_WORDS-1 : {2'd0, source_column[10:5]};
    wire [7:0] adjacent_address = cache_address == ROW_WORDS-1 ? cache_address : cache_address + 1'b1;
    wire [7:0] even_pixel_address = (cache_address[0] ? adjacent_address : cache_address) >> 1;
    wire [7:0] odd_pixel_address = (cache_address[0] ? cache_address : adjacent_address) >> 1;
    wire [7:0] even_address = state == ST_LOAD ? load_index >> 1 : even_pixel_address;
    wire [7:0] odd_address = state == ST_LOAD ? load_index >> 1 : odd_pixel_address;
    wire [31:0] connected_word;
    (* ram_style = "block" *) reg [CACHE_BITS-1:0] even_cache [0:BANK_WORDS-1];
    (* ram_style = "block" *) reg [CACHE_BITS-1:0] odd_cache [0:BANK_WORDS-1];
    reg [CACHE_BITS-1:0] even_read, odd_read;
    reg word_odd, same_word;
    reg load_pending, load_bank, load_merge;
    reg [7:0] load_address;
    reg [31:0] load_word;
    wire [31:0] old_word = load_bank ? odd_read[31:0] : even_read[31:0];
    wire [31:0] merged_word = load_merge ? (load_word | old_word) : load_word;
    wire [31:0] first_word = word_odd ? odd_read[31:0] : even_read[31:0];
    wire [31:0] second_word = same_word ? first_word : (word_odd ? even_read[31:0] : odd_read[31:0]);
    wire [63:0] cached_pixels = {second_word, first_word};
    wire [CACHE_BITS-1:0] cache_write_word;
    wire [4*LABEL_BITS-1:0] first_labels,second_labels;
    wire [8*LABEL_BITS-1:0] cached_labels={second_labels,first_labels};
    generate if(SUPPORT_LABELS) begin : component_colors
        // At 10/9 reduction every two-source-pixel interval starts at an even
        // coordinate. It never straddles an 8x8 component cell in either axis.
        assign cache_write_word={label_data_i,merged_word};
        assign first_labels=word_odd ? odd_read[CACHE_BITS-1:32] : even_read[CACHE_BITS-1:32];
        assign second_labels=same_word ? first_labels : (word_odd ? even_read[CACHE_BITS-1:32] : odd_read[CACHE_BITS-1:32]);
    end else begin : monochrome
        assign cache_write_word=merged_word;
        assign first_labels=0;assign second_labels=0;
    end endgenerate
    genvar pixel, lane;
    generate for (pixel = 0; pixel < 32; pixel = pixel + 1) begin : connected_bit
        assign connected_word[pixel] = read_data_i[pixel*2+1];
    
end
endgenerate
// 同步旧值读后延迟一拍写回；第二源行与第一源行做OR，而非覆盖第一行。
    always @(posedge clk_i) begin
        if (advance) begin
            even_read <= even_cache[even_address];
            odd_read <= odd_cache[odd_address];
            word_odd <= cache_address[0];
            same_word <= cache_address == adjacent_address;
            if (load_fire) begin
                load_address <= load_index >> 1;
                load_bank <= load_index[0];
                load_merge <= merge_row;
                load_word <= connected_word;
            
end
            if (load_pending && !load_bank) begin
                even_cache[load_address] <= cache_write_word;
            
end
            if (load_pending && load_bank) begin
                odd_cache[load_address] <= cache_write_word;
            
end
        
end
    
end
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
load_pending <= 1'b0; 
end
        else if (advance) begin
load_pending <= load_fire; 
end
    
end
// 每目标像素的半开源区间宽度为1或2。相邻目标区间不重叠也不遗漏。
    generate for (lane = 0; lane < 4; lane = lane + 1) begin : target_pixel
        wire [3:0] offset = (x_phase + lane*SCALE_NUM) / SCALE_DEN;
        wire [3:0] limit = (x_phase + (lane+1)*SCALE_NUM) / SCALE_DEN;
        reg [5:0] pixel_index, next_index;
        reg include_neighbor;
        reg [15:0] pixel_o;
        wire [LABEL_BITS-1:0] label_code=cached_labels[pixel_index[5:3]*LABEL_BITS+:LABEL_BITS];
        wire [2:0] object_slot=label_code>=7 ? label_code-3 : label_code-2;
        wire [KIND_BITS-1:0] class_code=(label_code>=2 && label_code<=5 || MAX_OBJECTS>4 && label_code>=7 && label_code<=MAX_OBJECTS+2) ? object_kind_i[object_slot*KIND_BITS+:KIND_BITS] : 3'd0;
        reg [15:0] edge_color;
        always @(*) begin
            edge_color=16'hffff;
            if(SUPPORT_LABELS && color_enable_i) begin
                case(class_code)
                    1:edge_color=16'h87f0;
                    2:edge_color=16'h87ff;
                    3:edge_color=16'hfff0;
                    4:edge_color=16'hfc1f;
                    5:edge_color=16'hfdc8;
                    6:edge_color=16'h7dff;
                    7:edge_color=16'hffff;
                    8:edge_color=16'hffe0;
                    default:edge_color=16'hffff;
                endcase
            end
        end
        wire covered = cached_pixels[pixel_index] || (include_neighbor && cached_pixels[next_index]);
        always @(posedge clk_i) begin
            if (advance) begin
                pixel_index <= {1'b0, source_column[4:0]} + offset;
                next_index <= {1'b0, source_column[4:0]} + offset + 6'd1;
                include_neighbor <= limit - offset == 2;
                pixel_o <= active_delay && covered ? edge_color : 16'h0000;
            
end
        
end
        assign rgb565_o[(3-lane)*16+:16] = pixel_o;
    
end
endgenerate
// 状态寄存器只在输出能够推进时跳转，背压期间冻结整个两拍读流水。
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
state <= ST_IDLE; 
end
        else if (advance) begin
state <= next_state; 
end
    
end
    always @(*) begin
        next_state = state;
        case (state)
            ST_IDLE: begin
if (start_i) begin
next_state = ST_REQUEST; 
end 
end
            ST_REQUEST: begin
if (read_request_ready_i) begin
next_state = ST_LOAD; 
end 
end
            ST_LOAD: begin
if (read_valid_i && load_index == ROW_WORDS-1) begin
next_state = ST_SETTLE; 
end 
end
            ST_SETTLE: begin
                if (row_span == 2 && !merge_row) begin
next_state = ST_REQUEST; 
end
                else begin
next_state = ST_PIXELS; 
end
            
end
            ST_PIXELS: begin
if (group_index == DISPLAY_GROUPS-1) begin
next_state = ST_FLUSH; 
end 
end
            ST_FLUSH: begin
next_state = ST_DRAIN; 
end
            ST_DRAIN: begin
next_state = ST_NEXT; 
end
            ST_NEXT: begin
                if (display_row == DISPLAY_HEIGHT-1) begin
next_state = ST_FINISH; 
end
                else begin
next_state = ST_REQUEST;
end
            
end
            ST_FINISH: begin
next_state = ST_IDLE; 
end
            default: begin
next_state = ST_IDLE; 
end
        endcase
    
end
// 完整输出帧边界与最后像素握手保持一致，复位后的首次加载不读旧行。
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            load_index <= 8'd0;
            group_index <= 11'd0;
            display_row <= 11'd0;source_row<=0;
            source_column <= 11'd0;
            x_phase <= 4'd0;
            y_phase <= 4'd0;
            row_address <= 32'd0;
            merge_row <= 1'b0;
            valid_pipe <= 2'd0;compare_left_pipe<=0;
            sop_pipe <= 2'd0;
            eop_pipe <= 2'd0;
            active_delay <= 1'b0;
            frame_active_o_reg <= 1'b0;
        
end
else if (advance) begin
            valid_pipe <= {valid_pipe[0], state == ST_PIXELS};
            compare_left_pipe<={compare_left_pipe[0],compare_mode && group_index<DISPLAY_GROUPS/2 && state==ST_PIXELS};
            sop_pipe <= {sop_pipe[0], state == ST_PIXELS && display_row == 0 && group_index == 0};
            eop_pipe <= {eop_pipe[0], state == ST_PIXELS && display_row == DISPLAY_HEIGHT-1 && group_index == DISPLAY_GROUPS-1};
            active_delay <= active_group;
            case (state)
                ST_IDLE: begin
                    if (start_i) begin
                        load_index <= 8'd0;
                        group_index <= 11'd0;
                        display_row <= 11'd0;source_row<=0;
                        source_column <= 11'd0;
                        x_phase <= 4'd0;
                        y_phase <= 4'd0;
                        row_address <= base_address_i;
                        merge_row <= 1'b0;
                        frame_active_o_reg <= 1'b1;
                    
end
                
end
                ST_LOAD: begin
                    if (read_valid_i) begin
                        load_index <= load_index == ROW_WORDS-1 ? 8'd0 : load_index + 1'b1;
                    
end
                
end
                ST_SETTLE: begin
                    if (row_span == 2 && !merge_row) begin
merge_row <= 1'b1; 
end
                
end
                ST_PIXELS: begin
                    group_index <= group_index + 1'b1;
                    if (active_group) begin
                        source_column <= source_column + column_advance;
                        x_phase <= next_x_phase;
                    
end
                
end
                ST_NEXT: begin
                    display_row <= display_row + 1'b1;if(active_row) source_row<=source_row+row_span;
                    group_index <= 11'd0;
                    source_column <= 11'd0;
                    x_phase <= 4'd0;
                    y_phase <= next_y_phase;
                    if(active_row) row_address <= row_address + row_span*ROW_STRIDE;
                    merge_row <= 1'b0;
                    if (display_row == DISPLAY_HEIGHT-1) begin
frame_active_o_reg <= 1'b0; 
end
                
end
                default: begin
end
            endcase
        
end
    
end
    assign busy_o = state != ST_IDLE;
    assign done_o = state == ST_FINISH && advance;
    assign read_request_o = state == ST_REQUEST && advance;
    assign read_address_o = row_address + (merge_row ? ROW_STRIDE : 0);
    assign read_ready_o = state == ST_LOAD && advance;
    assign frame_active_o = frame_active_o_reg;
    assign valid_o = valid_pipe[1];
    assign sop_o = sop_pipe[1];
    assign eop_o = eop_pipe[1];
endmodule
