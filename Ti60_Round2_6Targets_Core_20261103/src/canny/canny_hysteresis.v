`timescale 1ns/1ps
// Complete fixed-point 8-connectivity on a two-bit DDR workspace image.
// Row transactions are explicit; this module contains no DDR behavioral model.
// Alternate forward/reverse Gauss-Seidel sweeps and stop ONLY on no changes.
// The caller must retain exclusive ownership of the workspace until done_o.
module canny_hysteresis #(
    parameter WIDTH=1920,
    parameter HEIGHT=1200,
    parameter BLOCK_PIXELS=192,
    parameter ROW_STRIDE=WIDTH/4
) (
    input wire clk_i, input wire rst_n_i,
    input wire start_i, input wire [31:0] base_address_i,
    output wire busy_o, output wire done_o,
    output wire [31:0] cycles_o, output wire [31:0] sweeps_o,
    output wire read_request_o, input wire read_request_ready_i,
    output wire [31:0] read_address_o,
    input wire read_valid_i, output wire read_ready_o, input wire [63:0] read_data_i,
    output wire write_request_o, input wire write_request_ready_i,
    output wire [31:0] write_address_o,
    output wire write_valid_o, input wire write_ready_i,
    output wire [63:0] write_data_o, output wire write_last_o,
    input wire write_done_i,
    // Metadata for the accepted canonical write stream; no additional DDR read.
    output wire [10:0] feature_row_o,
    output wire [5:0] feature_word_o,
    output wire feature_first_o
);
    localparam BLOCKS=WIDTH/BLOCK_PIXELS;
    localparam ADDRESS_BITS=BLOCKS>1 ? $clog2(BLOCKS) : 1;
    localparam ROW_WORDS=WIDTH/32;
    localparam WORD_ADDRESS_BITS=ROW_WORDS>1 ? $clog2(ROW_WORDS) : 1;
    localparam BLOCK_WORDS=BLOCK_PIXELS/32;
    localparam VECTOR_BITS=2*BLOCK_PIXELS;
    localparam ROW_BYTES=ROW_STRIDE;
    localparam LAST_ROW_OFFSET=(HEIGHT-1)*ROW_BYTES;
    localparam PENDING_BITS=((HEIGHT+31)/32)*32;
    localparam IDLE=0, LOAD_REQUEST=1, LOAD_DATA=2, LOAD_NEXT=3,
        READ_LEFT=4, READ_CENTER=5, READ_RIGHT=6, LINK_START=7, LINK_WAIT=8,
        WRITE_REQUEST=9, EMIT_READ=10, EMIT_DATA=11, WRITE_WAIT=12,
        ROW_NEXT=13, SWEEP_NEXT=14, FINISH=15, SELECT_ROW=16, SKIP_ROW=17,
        READ_LEFT_HIGH=18, READ_CENTER_HIGH=19, READ_RIGHT_HIGH=20;
    reg [4:0] state, next_state;
    reg [1:0] load_bank;
    reg [WORD_ADDRESS_BITS-1:0] load_index, emit_index;
    reg [ADDRESS_BITS-1:0] block_index;
    reg [11:0] row_index;
    reg reverse_scan;
    reg changed;
    reg row_changed;
    reg cache_valid;
    reg [11:0] physical_row;
    // Bit zero is the current row; remaining bits follow the scan direction.
    // Circular rotation preserves the already scanned rows for the next sweep.
    reg [HEIGHT-1:0] pending_scan;
    wire [31:0] pending_head;
    generate if(HEIGHT>=32) begin : full_pending_page
        assign pending_head=pending_scan[31:0];
    end else begin : short_pending_page
        assign pending_head={{(32-HEIGHT){1'b0}},pending_scan};
    end endgenerate
    wire pending_page_nonempty=|pending_head;
    wire page_boundary=reverse_scan ? physical_row[4:0]==31 : physical_row[4:0]==0;
    wire skip_page=page_boundary && !pending_page_nonempty && row_index+32<=HEIGHT;
    wire [11:0] skip_distance=skip_page ? 12'd32 : 12'd1;
    wire skip_finishes=row_index+skip_distance>=HEIGHT;
    reg [31:0] base_address, row_address;
    reg [31:0] cycles, sweeps;
    reg left_connected;
    // 三行共用一个简单双端口RAM：三拍顺序取得北、中、南整块。
    // 只复制块首像素的两位邻接状态，避免为右邻域再开384位读口。
    localparam CACHE_ADDRESS_BITS=$clog2(6*BLOCKS);
    localparam HALF_BITS=VECTOR_BITS/2;
    localparam HALF_WORDS=BLOCK_WORDS/2;
    (* syn_ramstyle="block_ram" *) reg [HALF_BITS-1:0] row_memory [0:6*BLOCKS-1];
    (* syn_ramstyle="registers" *) reg [1:0] block_head [0:3*BLOCKS-1]; // 默认60位，不搬迁整行缓存。
    reg [HALF_BITS-1:0] first_read;
    reg [HALF_BITS-1:0] south_low;
    reg link_write_pending;
    reg [CACHE_ADDRESS_BITS-1:0] link_write_address;
    reg [HALF_BITS-1:0] link_write_high;
    reg [1:0] bank_phase;
    reg [1:0] north_left, south_left;
    reg [1:0] next_north;
    reg [1:0] next_center; // 下一块中行首像素的更新后状态。
    reg [1:0] next_south; // 下一块南行首像素的原始缓存状态。
    reg [VECTOR_BITS-1:0] north_center, center_center, south_center;
    reg [VECTOR_BITS-1:0] load_accumulator;
    wire [VECTOR_BITS-1:0] assembled_input;
    wire [ADDRESS_BITS-1:0] load_group=load_index/BLOCK_WORDS;
    wire [ADDRESS_BITS-1:0] emit_group=emit_index/BLOCK_WORDS;
    wire [ADDRESS_BITS-1:0] input_address = reverse_scan ? BLOCKS-1-load_group : load_group;
    wire [ADDRESS_BITS-1:0] emit_address = reverse_scan ? BLOCKS-1-emit_group : emit_group;
    wire load_complete=load_index%BLOCK_WORDS==BLOCK_WORDS-1;
    reg [CACHE_ADDRESS_BITS-1:0] first_address;
    wire [VECTOR_BITS-1:0] reordered_input;
    wire [63:0] reordered_output;
    wire link_valid, link_ready, link_changed, link_right;
    wire [VECTOR_BITS-1:0] link_data;
    wire [63:0] selected_output=first_read[(reverse_scan ? HALF_WORDS-1-emit_index%HALF_WORDS : emit_index%HALF_WORDS)*64+:64];
    wire cache_load=state==LOAD_DATA && read_valid_i && load_index%HALF_WORDS==HALF_WORDS-1;
    wire cache_link=state==LINK_WAIT && link_valid;
    wire memory_write_enable=cache_load || cache_link || link_write_pending;
    wire [ADDRESS_BITS-1:0] memory_write_address=cache_link ? block_index : input_address;
    wire [HALF_BITS-1:0] input_half;
    wire [HALF_BITS-1:0] input_half_ordered;
    assign input_half=assembled_input[VECTOR_BITS-1:HALF_BITS];
    genvar half_pixel;
    generate for(half_pixel=0;half_pixel<BLOCK_PIXELS/2;half_pixel=half_pixel+1) begin : half_pixel_order
        assign input_half_ordered[half_pixel*2+:2]=reverse_scan ? input_half[(BLOCK_PIXELS/2-1-half_pixel)*2+:2] : input_half[half_pixel*2+:2];
    end endgenerate
    wire write_half=reverse_scan ? !(load_index%BLOCK_WORDS>=HALF_WORDS) : load_index%BLOCK_WORDS>=HALF_WORDS;
    wire [HALF_BITS-1:0] memory_write_data=link_write_pending ? link_write_high : cache_link ? link_data[HALF_BITS-1:0] : input_half_ordered;
    wire [1:0] logical_write_bank=state==LINK_WAIT ? 2'd1 : load_bank;
    wire [2:0] bank_sum={1'b0,bank_phase}+{1'b0,logical_write_bank};
    wire [1:0] physical_write_bank=bank_sum>=3 ? bank_sum-3 : bank_sum;
    wire [1:0] center_bank=bank_phase==2 ? 2'd0 : bank_phase+1'b1;
    wire [1:0] south_bank=bank_phase==0 ? 2'd2 : bank_phase-1'b1;
    wire [ADDRESS_BITS-1:0] next_block=block_index==BLOCKS-1 ? block_index : block_index+1'b1;
    wire [CACHE_ADDRESS_BITS-1:0] write_cache_address=link_write_pending ? link_write_address :
        (physical_write_bank*BLOCKS+memory_write_address)*2+(cache_link ? 1'b0 : write_half);
    wire [WORD_ADDRESS_BITS-1:0] read_emit_index=(state==EMIT_DATA && write_ready_i && emit_index!=ROW_WORDS-1) ? emit_index+1'b1 : emit_index;
    wire [ADDRESS_BITS-1:0] read_emit_group=read_emit_index/BLOCK_WORDS;
    wire [ADDRESS_BITS-1:0] read_emit_block=reverse_scan ? BLOCKS-1-read_emit_group : read_emit_group;
    wire read_emit_half=reverse_scan ? !(read_emit_index%BLOCK_WORDS>=HALF_WORDS) : read_emit_index%BLOCK_WORDS>=HALF_WORDS;
    genvar pixel;
    reg [HEIGHT-1:0] pending_updated;
    wire [HEIGHT-1:0] pending_reverse;
    wire [HEIGHT-1:0] pending_rotate_one=(pending_scan>>1)|(pending_scan<<(HEIGHT-1));
    wire [HEIGHT-1:0] updated_rotate_one=(pending_updated>>1)|(pending_updated<<(HEIGHT-1));
    wire [HEIGHT-1:0] pending_rotate_page;
    wire [HEIGHT-1:0] pending_rotate_tail;
    genvar row_flag;
    generate
        for(row_flag=0;row_flag<HEIGHT;row_flag=row_flag+1) begin : reverse_row_flags
            if(row_flag==0) assign pending_reverse[row_flag]=pending_scan[0];
            else assign pending_reverse[row_flag]=pending_scan[HEIGHT-row_flag];
        end
        if(HEIGHT>=32) begin : page_rotation
            assign pending_rotate_page=(pending_scan>>32)|(pending_scan<<(HEIGHT-32));
            assign pending_rotate_tail=(pending_scan>>31)|(pending_scan<<(HEIGHT-31));
        end else begin : no_page_rotation
            assign pending_rotate_page=pending_scan;
            assign pending_rotate_tail=pending_scan;
        end
    endgenerate
    always @(*) begin
        pending_updated=pending_scan;
        pending_updated[0]=row_changed;
        if(row_changed && row_index!=HEIGHT-1) pending_updated[1]=1'b1;
        if(row_changed && row_index!=0) pending_updated[HEIGHT-1]=1'b1;
    end
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) pending_scan<=0;
        else if(state==IDLE && start_i) pending_scan<={HEIGHT{1'b1}};
        else if(state==ROW_NEXT) pending_scan<=row_index==HEIGHT-1 ? pending_updated : updated_rotate_one;
        else if(state==SKIP_ROW) begin
            if(skip_page) pending_scan<=skip_finishes ? pending_rotate_tail : pending_rotate_page;
            else if(!skip_finishes) pending_scan<=pending_rotate_one;
        end else if(state==SWEEP_NEXT && changed) pending_scan<=pending_reverse;
    end
    generate
    if(BLOCK_WORDS==1) begin : direct_input
        assign assembled_input=read_data_i;
    end else begin : assembled_block
        assign assembled_input={read_data_i,load_accumulator[VECTOR_BITS-1:64]};
    end
    for(pixel=0;pixel<BLOCK_PIXELS;pixel=pixel+1) begin : reverse_input_pixels
        assign reordered_input[pixel*2+:2] = reverse_scan ? assembled_input[(BLOCK_PIXELS-1-pixel)*2+:2] : assembled_input[pixel*2+:2];
    end
    for(pixel=0;pixel<32;pixel=pixel+1) begin : reverse_output_pixels
        assign reordered_output[pixel*2+:2] = reverse_scan ? selected_output[(31-pixel)*2+:2] : selected_output[pixel*2+:2];
    end endgenerate
    wire [1:0] north_right = block_index==BLOCKS-1 ? 2'b0 : next_north;
    wire [1:0] south_right = block_index==BLOCKS-1 ? 2'b0 : next_south;
    wire [VECTOR_BITS+3:0] north_neighbors = row_index==0 ? {(VECTOR_BITS+4){1'b0}} : {north_right,north_center,(block_index==0 ? 2'b0 : north_left)};
    wire [VECTOR_BITS+3:0] south_neighbors = row_index==HEIGHT-1 ? {(VECTOR_BITS+4){1'b0}} : {south_right,first_read,south_low,(block_index==0 ? 2'b0 : south_left)};
    canny_link_block #(.PIXELS(BLOCK_PIXELS),.ITERATIONS($clog2(BLOCK_PIXELS)-3)) link_block (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.valid_i(state==LINK_START),.ready_o(link_ready),
        .north_i(north_neighbors),.center_i(center_center),.south_i(south_neighbors),
        .left_connected_i(left_connected),
        .right_connected_i(block_index==BLOCKS-1 ? 1'b0 : next_center[1]),
        .valid_o(link_valid),.data_o(link_data),.changed_o(link_changed),.right_connected_o(link_right)
    );
    always @(*) begin
        first_address=(center_bank*BLOCKS+block_index)*2;
        case(state)
            READ_LEFT:first_address=(bank_phase*BLOCKS+block_index)*2;
            READ_LEFT_HIGH:first_address=(bank_phase*BLOCKS+block_index)*2+1;
            READ_CENTER:first_address=(center_bank*BLOCKS+block_index)*2;
            READ_CENTER_HIGH:first_address=(center_bank*BLOCKS+block_index)*2+1;
            READ_RIGHT:first_address=(south_bank*BLOCKS+block_index)*2;
            READ_RIGHT_HIGH,LINK_START:first_address=(south_bank*BLOCKS+block_index)*2+1;
            EMIT_READ,EMIT_DATA:first_address=(center_bank*BLOCKS+read_emit_block)*2+read_emit_half;
            default:begin end
        endcase
    end
    // Six half-vector reads replace three full-vector reads. The exact carry
    // closure uses three fewer cycles, preserving every external row handshake.
    always @(posedge clk_i) begin
        if(memory_write_enable) row_memory[write_cache_address]<=memory_write_data;
        first_read<=row_memory[first_address];
        if(memory_write_enable && !write_cache_address[0])
            block_head[write_cache_address>>1]<=memory_write_data[1:0];
    end
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin link_write_pending<=0;link_write_address<=0;link_write_high<=0;south_low<=0;end
        else begin
            link_write_pending<=cache_link;
            if(cache_link) begin link_write_address<=write_cache_address+1'b1;link_write_high<=link_data[VECTOR_BITS-1:HALF_BITS];end
            if(state==READ_RIGHT_HIGH) south_low<=first_read;
        end
    end
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) state<=IDLE;
        else state<=next_state;
    end
    always @(*) begin
        next_state=state;
        case(state)
            IDLE: if(start_i) next_state=SELECT_ROW;
            SELECT_ROW: if(!pending_scan[0]) next_state=SKIP_ROW;
                        else if(cache_valid && row_index==HEIGHT-1) next_state=READ_LEFT;
                        else next_state=LOAD_REQUEST;
            SKIP_ROW: next_state=skip_finishes ? SWEEP_NEXT : SELECT_ROW;
            LOAD_REQUEST: if(read_request_ready_i) next_state=LOAD_DATA;
            LOAD_DATA: if(read_valid_i && load_index==ROW_WORDS-1) next_state=LOAD_NEXT;
            LOAD_NEXT: if(load_bank==2 || (load_bank==1 && row_index==HEIGHT-1)) next_state=READ_LEFT;
                       else next_state=LOAD_REQUEST;
            READ_LEFT: next_state=READ_LEFT_HIGH;
            READ_LEFT_HIGH: next_state=READ_CENTER;
            READ_CENTER: next_state=READ_CENTER_HIGH;
            READ_CENTER_HIGH: next_state=READ_RIGHT;
            READ_RIGHT: next_state=READ_RIGHT_HIGH;
            READ_RIGHT_HIGH: next_state=LINK_START;
            LINK_START: if(link_ready) next_state=LINK_WAIT;
            LINK_WAIT: if(link_valid) next_state=block_index==BLOCKS-1 ? WRITE_REQUEST : READ_LEFT;
            WRITE_REQUEST: if(write_request_ready_i) next_state=EMIT_READ;
            EMIT_READ: next_state=EMIT_DATA;
            EMIT_DATA: if(write_ready_i) begin
                if(emit_index==ROW_WORDS-1) next_state=WRITE_WAIT;
                else if(emit_index%BLOCK_WORDS==BLOCK_WORDS-1) next_state=EMIT_READ;
            end
            WRITE_WAIT: if(write_done_i) next_state=ROW_NEXT;
            ROW_NEXT: next_state=row_index==HEIGHT-1 ? SWEEP_NEXT : SELECT_ROW;
            SWEEP_NEXT: next_state=changed ? SELECT_ROW : FINISH;
            FINISH: next_state=IDLE;
            default: next_state=IDLE;
        endcase
    end
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            load_bank<=1; load_index<=0; block_index<=0; emit_index<=0;
            row_index<=0; reverse_scan<=0; changed<=0;
            base_address<=0; row_address<=0; cycles<=0; sweeps<=0;
            left_connected<=0; north_left<=0; south_left<=0; next_north<=0;
            next_center<=0; next_south<=0;
            north_center<=0; center_center<=0; south_center<=0;
            load_accumulator<=0;
            bank_phase<=0;
            row_changed<=0; cache_valid<=0; physical_row<=0;
        end else begin
            if(state!=IDLE && state!=FINISH) cycles<=cycles+1'b1;
            case(state)
                IDLE: if(start_i) begin
                    base_address<=base_address_i; row_address<=base_address_i;
                    row_index<=0; reverse_scan<=0; changed<=0; cycles<=0; sweeps<=1;
                    load_bank<=1; load_index<=0; block_index<=0; emit_index<=0; left_connected<=0;
                    bank_phase<=0;
                    row_changed<=0; cache_valid<=0; physical_row<=0;
                end
                SELECT_ROW: begin
                    row_changed<=0;
                    if(!cache_valid) begin
                        bank_phase<=0;
                        load_bank<=row_index==0 ? 2'd1 : 2'd0;
                    end
                end
                SKIP_ROW: begin
                    cache_valid<=0;
                    if(!skip_finishes) begin
                        row_index<=row_index+skip_distance;
                        physical_row<=reverse_scan ? physical_row-skip_distance : physical_row+skip_distance;
                        row_address<=reverse_scan ? row_address-skip_distance*ROW_BYTES : row_address+skip_distance*ROW_BYTES;
                    end
                end
                LOAD_DATA: if(read_valid_i) begin
                    load_index<=load_index==ROW_WORDS-1 ? 0 : load_index+1'b1;
                    load_accumulator<=assembled_input;
                end
                LOAD_NEXT: if(next_state==LOAD_REQUEST) load_bank<=load_bank+1'b1;
                READ_LEFT_HIGH:north_center[HALF_BITS-1:0]<=first_read;
                READ_CENTER:north_center[VECTOR_BITS-1:HALF_BITS]<=first_read;
                READ_CENTER_HIGH:center_center[HALF_BITS-1:0]<=first_read;
                READ_RIGHT: begin
                    center_center[VECTOR_BITS-1:HALF_BITS]<=first_read;
                    next_north<=block_head[bank_phase*BLOCKS+next_block];
                    next_center<=block_head[center_bank*BLOCKS+next_block];
                    next_south<=block_head[south_bank*BLOCKS+next_block];
                end
                LINK_START: south_center<={first_read,south_low};
                LINK_WAIT: if(link_valid) begin
                    north_left<=north_center[VECTOR_BITS-1:VECTOR_BITS-2];
                    south_left<=south_center[VECTOR_BITS-1:VECTOR_BITS-2];
                    changed<=changed || link_changed;
                    row_changed<=row_changed || link_changed;
                    left_connected<=link_right;
                    block_index<=block_index==BLOCKS-1 ? 0 : block_index+1'b1;
                end
                EMIT_DATA: if(write_ready_i) emit_index<=emit_index==ROW_WORDS-1 ? 0 : emit_index+1'b1;
                ROW_NEXT: begin
                    // A row is stable until its own or an adjacent row's
                    // connected bits change. Keep the changed row pending too:
                    // a newly connected block can seed an earlier block.
                    // Publication follows WRITE_WAIT, so all B responses have
                    // completed before any later read can use the updated row.
                    cache_valid<=1;
                    if(row_index!=HEIGHT-1) begin
                        row_index<=row_index+1'b1;
                        physical_row<=reverse_scan ? physical_row-1'b1 : physical_row+1'b1;
                        row_address<=reverse_scan ? row_address-ROW_BYTES : row_address+ROW_BYTES;
                        load_bank<=2; left_connected<=0;
                        bank_phase<=bank_phase==2 ? 2'd0 : bank_phase+1'b1;
                    end
                end
                SWEEP_NEXT: if(changed) begin
                    reverse_scan<=!reverse_scan;
                    row_address<=reverse_scan ? base_address : base_address+LAST_ROW_OFFSET;
                    row_index<=0; load_bank<=1; changed<=0; left_connected<=0; sweeps<=sweeps+1'b1;
                    bank_phase<=0;
                    cache_valid<=0;
                    physical_row<=reverse_scan ? 12'd0 : HEIGHT-1;
                end
                default: begin end
            endcase
        end
    end
    assign feature_row_o=physical_row[10:0];
    assign feature_word_o=emit_index;
    assign feature_first_o=sweeps==1;
    assign busy_o=state!=IDLE;
    assign done_o=state==FINISH;
    assign cycles_o=cycles;
    assign sweeps_o=sweeps;
    assign read_request_o=state==LOAD_REQUEST;
    assign read_address_o=load_bank==1 ? row_address :
        load_bank==0 ? (reverse_scan ? row_address+ROW_BYTES : row_address-ROW_BYTES) :
                      (reverse_scan ? row_address-ROW_BYTES : row_address+ROW_BYTES);
    assign read_ready_o=state==LOAD_DATA;
    assign write_request_o=state==WRITE_REQUEST;
    assign write_address_o=row_address;
    assign write_valid_o=state==EMIT_DATA;
    assign write_data_o=reordered_output;
    assign write_last_o=emit_index==ROW_WORDS-1;
endmodule
