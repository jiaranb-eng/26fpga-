`timescale 1ns / 1ps

// 四路写侧控制器：官方 FIFO 完成 CDC，调度器形成最终四宫格地址。
// 读侧保持原工程的 R0_FIFO_16、四帧槽和 128-beat 读取策略。
module quad_axi4_ctrl #(
    parameter C_COMPACT_EDGES = 0,
    parameter C_ID_LEN = 4,
    parameter C_DATA_LEN = 128,
    parameter C_BUF_SIZE = 22,
    parameter C_RD_END_ADDR = 1280 * 720 * 2,
    parameter C_R_BURST_LEN = 128,
    parameter C_R_WIDTH = 16,
    parameter [17:0] C_FRAME_PIXELS = 18'd230400,
    parameter [8:0] C_OUTPUT_HEIGHT = 9'd360,
    parameter [2:0] C_CHUNKS_PER_LINE = 3'd5,
    parameter C_HISTORY_ENABLE = 0,
    parameter C_SLOT_BITS = C_HISTORY_ENABLE ? 5 : 2,
    parameter [31:0] C_BASE_ADDR = 32'h00000000,
    parameter [31:0] C_LINE_BYTES=32'd2560,
    parameter [31:0] C_HALF_LINE_BYTES=32'd1280,
    parameter [31:0] C_BOTTOM_OFFSET=32'd921600
) (
    input wire [1:0] history_control_i,
    input wire [7:0] history_age_a_i,
    input wire [7:0] history_age_b_i,
    output wire [7:0] history_count_o,
    input  wire                    axi_clk,
    input  wire                    axi_reset,
    output wire [31:0]             axi_awaddr,
    output wire [7:0]              axi_awlen,
    output wire                    axi_awvalid,
    input  wire                    axi_awready,
    output wire [C_DATA_LEN-1:0]   axi_wdata,
    output wire [15:0]             axi_wstrb,
    output wire                    axi_wlast,
    output wire                    axi_wvalid,
    input  wire                    axi_wready,
    input  wire [1:0]              axi_bresp,
    input  wire                    axi_bvalid,
    output wire                    axi_bready,
    output wire [C_ID_LEN-1:0]     axi_arid,
    output wire [31:0]             axi_araddr,
    output wire [7:0]              axi_arlen,
    output reg                     axi_arvalid,
    input  wire                    axi_arready,
    input  wire [C_DATA_LEN-1:0]   axi_rdata,
    input  wire                    axi_rlast,
    input  wire                    axi_rvalid,
    output wire                    axi_rready,
    input  wire                    wframe_pclk,
    input  wire                    wframe_reset_n,
    input  wire                    wframe_sop,
    input  wire                    wframe_eop,
    input  wire                    wframe_valid_0,
    input  wire                    wframe_valid_1,
    input  wire                    wframe_valid_2,
    input  wire                    wframe_valid_3,
    input  wire                    wframe_data_en_0,
    input  wire                    wframe_data_en_1,
    input  wire                    wframe_data_en_2,
    input  wire                    wframe_data_en_3,
    input  wire [31:0]             wframe_data_0,
    input  wire [31:0]             wframe_data_1,
    input  wire [31:0]             wframe_data_2,
    input  wire [31:0]             wframe_data_3,
    input  wire                    rframe_pclk,
    input  wire                    rframe_vsync,
    input  wire                    rframe_data_en,
    output wire [C_R_WIDTH-1:0]    rframe_data,
    output wire [31:0]             committed_frame_count_o,
    output wire [31:0]             dropped_frame_count_o,
    output wire [31:0]             tp_o
);

// -------------------------------------------------------------------------
// 四路官方异步 FIFO 与源时钟域帧检查
// -------------------------------------------------------------------------
wire [127:0] fifo_data_0;
wire [127:0] fifo_data_1;
wire [127:0] fifo_data_2;
wire [127:0] fifo_data_3;
wire [9:0] fifo_count_0;
wire [9:0] fifo_count_1;
wire [9:0] fifo_count_2;
wire [9:0] fifo_count_3;
wire fifo_empty_0;
wire fifo_empty_1;
wire fifo_empty_2;
wire fifo_empty_3;
wire fifo_overflow_0;
wire fifo_overflow_1;
wire fifo_overflow_2;
wire fifo_overflow_3;
wire fifo_underflow_0;
wire fifo_underflow_1;
wire fifo_underflow_2;
wire fifo_underflow_3;
wire fifo_reset_busy;
wire fifo_read_0;
wire fifo_read_1;
wire fifo_read_2;
wire fifo_read_3;

reg [3:0] fifo_reset_count;
wire fifo_soft_reset = (fifo_reset_count != 4'd0);
wire fifo_async_reset = axi_reset || fifo_soft_reset;

reg [1:0] frame_guard_reset_release;
always @(posedge wframe_pclk or negedge wframe_reset_n) begin
    if (!wframe_reset_n)
        frame_guard_reset_release <= 2'b00;
    else if (fifo_soft_reset)
        frame_guard_reset_release <= 2'b00;
    else
        frame_guard_reset_release <= {frame_guard_reset_release[0], 1'b1};
end
wire frame_guard_reset_n = frame_guard_reset_release[1];

quad_write_fifo_bank #(.COMPACT_EDGES(C_COMPACT_EDGES)) u_write_fifo_bank (
    .write_clk_i(wframe_pclk), .read_clk_i(axi_clk),
    .async_reset_i(fifo_async_reset),
    .write_enable_0_i(wframe_data_en_0),
    .write_enable_1_i(wframe_data_en_1),
    .write_enable_2_i(wframe_data_en_2),
    .write_enable_3_i(wframe_data_en_3),
    .write_data_0_i(wframe_data_0), .write_data_1_i(wframe_data_1),
    .write_data_2_i(wframe_data_2), .write_data_3_i(wframe_data_3),
    .read_enable_0_i(fifo_read_0), .read_enable_1_i(fifo_read_1),
    .read_enable_2_i(fifo_read_2), .read_enable_3_i(fifo_read_3),
    .read_data_0_o(fifo_data_0), .read_data_1_o(fifo_data_1),
    .read_data_2_o(fifo_data_2), .read_data_3_o(fifo_data_3),
    .read_count_0_o(fifo_count_0), .read_count_1_o(fifo_count_1),
    .read_count_2_o(fifo_count_2), .read_count_3_o(fifo_count_3),
    .empty_0_o(fifo_empty_0), .empty_1_o(fifo_empty_1),
    .empty_2_o(fifo_empty_2), .empty_3_o(fifo_empty_3),
    .overflow_0_o(fifo_overflow_0), .overflow_1_o(fifo_overflow_1),
    .overflow_2_o(fifo_overflow_2), .overflow_3_o(fifo_overflow_3),
    .underflow_0_o(fifo_underflow_0), .underflow_1_o(fifo_underflow_1),
    .underflow_2_o(fifo_underflow_2), .underflow_3_o(fifo_underflow_3),
    .reset_busy_o(fifo_reset_busy)
);

wire frame_done_toggle_pclk;
wire frame_good_pclk;
wire frame_active_pclk;
wire [31:0] source_good_frame_count;
wire [31:0] source_bad_frame_count;

quad_frame_guard #(
    .EXPECTED_PIXELS(C_FRAME_PIXELS)
) u_frame_guard (
    .clk_i(wframe_pclk), .rst_n_i(frame_guard_reset_n),
    .sink_sop_i(wframe_sop), .sink_eop_i(wframe_eop),
    .sink_valid_0_i(wframe_valid_0), .sink_valid_1_i(wframe_valid_1),
    .sink_valid_2_i(wframe_valid_2), .sink_valid_3_i(wframe_valid_3),
    .fifo_overflow_0_i(fifo_overflow_0),
    .fifo_overflow_1_i(fifo_overflow_1),
    .fifo_overflow_2_i(fifo_overflow_2),
    .fifo_overflow_3_i(fifo_overflow_3),
    .frame_done_toggle_o(frame_done_toggle_pclk),
    .frame_good_o(frame_good_pclk), .frame_active_o(frame_active_pclk),
    .good_frame_count_o(source_good_frame_count),
    .bad_frame_count_o(source_bad_frame_count)
);

reg frame_start_toggle_pclk;
always @(posedge wframe_pclk or negedge wframe_reset_n) begin
    if (!wframe_reset_n)
        frame_start_toggle_pclk <= 1'b0;
    else if (wframe_sop && (wframe_valid_0 || wframe_valid_1 ||
             wframe_valid_2 || wframe_valid_3))
        frame_start_toggle_pclk <= !frame_start_toggle_pclk;
end

(* async_reg = "true" *) reg [2:0] frame_start_sync;
(* async_reg = "true" *) reg [2:0] frame_done_sync;
(* async_reg = "true" *) reg [1:0] frame_good_sync_pipe;

wire frame_start_event = frame_start_sync[2] ^ frame_start_sync[1];
wire frame_done_event = frame_done_sync[2] ^ frame_done_sync[1];
wire frame_good_sync = frame_good_sync_pipe[1];

always @(posedge axi_clk or posedge axi_reset) begin
    if (axi_reset) begin
        frame_start_sync <= 3'b000;
        frame_done_sync <= 3'b000;
        frame_good_sync_pipe <= 2'b00;
    end else begin
        frame_start_sync <= {frame_start_sync[1:0], frame_start_toggle_pclk};
        frame_done_sync <= {frame_done_sync[1:0], frame_done_toggle_pclk};
        frame_good_sync_pipe <= {frame_good_sync_pipe[0], frame_good_pclk};
    end
end

// -------------------------------------------------------------------------
// 写调度、帧槽提交和错误丢弃
// -------------------------------------------------------------------------
wire [C_SLOT_BITS-1:0] write_frame_index;
wire [C_SLOT_BITS-1:0] read_frame_index;
wire [C_SLOT_BITS-1:0] history_read_b;
wire history_compare,history_valid;
reg       write_frame_commit;
reg       read_frame_advance;

wire [31:0] write_frame_base = C_BASE_ADDR + {write_frame_index, {C_BUF_SIZE{1'b0}}};
frame_history_ring #(.C_SLOT_BITS(C_SLOT_BITS)) history_slots (
    .clk_i(axi_clk),.rst_n_i(!axi_reset),.publish_i(write_frame_commit),
    .kind_i(1'b0),.boundary_i(read_frame_advance),
    .control_i(C_HISTORY_ENABLE ? history_control_i : 2'd0),
    .age_a_i(C_HISTORY_ENABLE ? history_age_a_i : 8'd0),
    .age_b_i(C_HISTORY_ENABLE ? history_age_b_i : 8'd0),
    .write_slot_o(write_frame_index),.read_slot_a_o(read_frame_index),
    .read_slot_b_o(history_read_b),.compare_o(history_compare),
    .valid_o(history_valid),.count_o(history_count_o)
);

reg scheduler_start;
reg scheduler_abort;
wire scheduler_busy;
wire scheduler_done;
wire scheduler_error;
wire [1:0] scheduler_route;
wire [2:0] scheduler_chunk;
wire [8:0] scheduler_line;

quad_write_scheduler #(
    .OUTPUT_HEIGHT(C_OUTPUT_HEIGHT),
    .LINE_BYTES(C_LINE_BYTES),.HALF_LINE_BYTES(C_HALF_LINE_BYTES),.BOTTOM_OFFSET(C_BOTTOM_OFFSET),
    .CHUNKS_PER_LINE(C_CHUNKS_PER_LINE)
) u_write_scheduler (
    .clk_i(axi_clk), .reset_i(axi_reset),
    .start_i(scheduler_start), .abort_i(scheduler_abort),
    .frame_base_i(write_frame_base),
    .fifo_count_0_i(fifo_count_0), .fifo_count_1_i(fifo_count_1),
    .fifo_count_2_i(fifo_count_2), .fifo_count_3_i(fifo_count_3),
    .fifo_data_0_i(fifo_data_0), .fifo_data_1_i(fifo_data_1),
    .fifo_data_2_i(fifo_data_2), .fifo_data_3_i(fifo_data_3),
    .fifo_read_0_o(fifo_read_0), .fifo_read_1_o(fifo_read_1),
    .fifo_read_2_o(fifo_read_2), .fifo_read_3_o(fifo_read_3),
    .axi_awaddr_o(axi_awaddr), .axi_awlen_o(axi_awlen),
    .axi_awvalid_o(axi_awvalid), .axi_awready_i(axi_awready),
    .axi_wdata_o(axi_wdata), .axi_wstrb_o(axi_wstrb),
    .axi_wlast_o(axi_wlast), .axi_wvalid_o(axi_wvalid),
    .axi_wready_i(axi_wready), .axi_bresp_i(axi_bresp),
    .axi_bvalid_i(axi_bvalid), .axi_bready_o(axi_bready),
    .busy_o(scheduler_busy), .frame_done_o(scheduler_done),
    .frame_error_o(scheduler_error), .active_route_o(scheduler_route),
    .active_chunk_o(scheduler_chunk), .active_line_o(scheduler_line)
);

reg start_pending;
reg frame_end_seen;
reg frame_good_seen;
reg scheduler_done_pending;
reg frame_bad_seen;
reg [31:0] committed_frame_count;
reg [31:0] dropped_frame_count;

wire current_good_available = frame_good_seen ||
                              (frame_done_event && frame_good_sync);
wire current_frame_done = frame_end_seen || frame_done_event;

always @(posedge axi_clk or posedge axi_reset) begin
    if (axi_reset) begin
        fifo_reset_count <= 4'd15;
        scheduler_start <= 1'b0;
        scheduler_abort <= 1'b0;
        start_pending <= 1'b0;
        frame_end_seen <= 1'b0;
        frame_good_seen <= 1'b0;
        scheduler_done_pending <= 1'b0;
        frame_bad_seen <= 1'b0;
        write_frame_commit <= 1'b0;
        committed_frame_count <= 32'd0;
        dropped_frame_count <= 32'd0;
    end else begin
        scheduler_start <= 1'b0;
        scheduler_abort <= 1'b0;
        write_frame_commit <= 1'b0;

        if (fifo_reset_count != 4'd0)
            fifo_reset_count <= fifo_reset_count - 1'b1;

        if (frame_start_event) begin
            start_pending <= 1'b1;
            frame_end_seen <= 1'b0;
            frame_good_seen <= 1'b0;
            frame_bad_seen <= 1'b0;
        end

        if (frame_done_event) begin
            frame_end_seen <= 1'b1;
            frame_good_seen <= frame_good_sync;
            if (!frame_good_sync && scheduler_busy) begin
                frame_bad_seen <= 1'b1;
                scheduler_abort <= 1'b1;
            end
        end

        if (start_pending && !scheduler_busy &&
            (fifo_reset_count == 4'd0) && !fifo_reset_busy) begin
            if (current_frame_done && !current_good_available) begin
                start_pending <= 1'b0;
                dropped_frame_count <= dropped_frame_count + 1'b1;
                fifo_reset_count <= 4'd15;
                scheduler_done_pending <= 1'b0;
            end else begin
                scheduler_start <= 1'b1;
                start_pending <= 1'b0;
                scheduler_done_pending <= 1'b0;
            end
        end

        if (scheduler_done) begin
            if (scheduler_error || frame_bad_seen ||
                (current_frame_done && !current_good_available)) begin
                dropped_frame_count <= dropped_frame_count + 1'b1;
                fifo_reset_count <= 4'd15;
                scheduler_done_pending <= 1'b0;
            end else if (current_good_available) begin
                write_frame_commit <= 1'b1;
                committed_frame_count <= committed_frame_count + 1'b1;
                fifo_reset_count <= 4'd15;
                scheduler_done_pending <= 1'b0;
            end else begin
                scheduler_done_pending <= 1'b1;
            end
        end else if (scheduler_done_pending && frame_done_event) begin
            if (frame_good_sync) begin
                write_frame_commit <= 1'b1;
                committed_frame_count <= committed_frame_count + 1'b1;
            end else begin
                dropped_frame_count <= dropped_frame_count + 1'b1;
            end
            fifo_reset_count <= 4'd15;
            scheduler_done_pending <= 1'b0;
        end
    end
end

assign committed_frame_count_o = committed_frame_count;
assign dropped_frame_count_o = dropped_frame_count;

// -------------------------------------------------------------------------
// 垂直消隐仅发出重启请求；ARVALID 必须握手，R 必须排空至 RLAST 才复位 FIFO。
// -------------------------------------------------------------------------
reg rframe_vsync_delayed;
always @(posedge rframe_pclk) begin
    if (axi_reset)
        rframe_vsync_delayed <= 1'b0;
    else
        rframe_vsync_delayed <= rframe_vsync;
end

wire rframe_vsync_falling = rframe_vsync_delayed && !rframe_vsync;
reg read_epoch;
always @(posedge rframe_pclk or posedge axi_reset) begin
    if (axi_reset) read_epoch <= 1'b0;
    else if (rframe_vsync_falling) read_epoch <= !read_epoch;
end
(* async_reg="true" *) reg [2:0] read_epoch_sync;
reg read_epoch_seen;
wire read_restart = read_epoch_sync[2] != read_epoch_seen;
reg restart_pending;
reg read_fifo_reset;
reg reader_has_frame;
reg [4:0] read_wait_count;
wire read_fifo_async_reset = axi_reset || read_fifo_reset;

wire read_fifo_full;
wire read_fifo_empty;
wire read_fifo_busy;
wire [C_R_WIDTH-1:0] read_fifo_data;
reg read_fifo_write_enable;
reg [C_DATA_LEN-1:0] read_fifo_write_data;
(* async_reg="true" *) reg [2:0] reader_valid_sync;
reg pixel_frame_bad;
always @(posedge rframe_pclk or posedge axi_reset) begin
    if (axi_reset) begin reader_valid_sync<=0; pixel_frame_bad<=0; end
    else begin
        reader_valid_sync<={reader_valid_sync[1:0],reader_has_frame};
        if (rframe_vsync_falling) pixel_frame_bad<=0;
        else if (rframe_data_en && reader_valid_sync[2] && read_fifo_empty) pixel_frame_bad<=1;
    end
end
wire reader_take = rframe_data_en && reader_valid_sync[2] && !pixel_frame_bad && !read_fifo_empty;
assign rframe_data = reader_valid_sync[2] && !pixel_frame_bad && !read_fifo_empty ? read_fifo_data : {C_R_WIDTH{1'b0}};

R0_FIFO_16 u_read_fifo (
    .wr_clk_i(axi_clk), .wr_en_i(read_fifo_write_enable),
    .wdata(read_fifo_write_data), .a_rst_i(read_fifo_async_reset),
    .prog_full_o(read_fifo_full),.rst_busy(read_fifo_busy),
    .rd_clk_i(rframe_pclk), .rd_en_i(reader_take),
    .rdata(read_fifo_data), .empty_o(read_fifo_empty), .almost_empty_o()
);

localparam [2:0] ST_IDLE=3'd0,ST_ADDRESS=3'd1,ST_DATA=3'd2,ST_RESET=3'd3,ST_SETTLE=3'd4;
reg [2:0] state_current,state_next;
reg [C_BUF_SIZE-1:0] read_address_offset;
reg [31:0] read_bus_address;
localparam [C_BUF_SIZE-1:0] READ_ADDRESS_INCREMENT =
    C_R_BURST_LEN * (C_DATA_LEN / 8);
always @(*) begin
    state_next=state_current;
    case(state_current)
        ST_IDLE: begin
            if(restart_pending || read_restart) state_next=ST_RESET;
            else if(reader_has_frame && read_address_offset<C_RD_END_ADDR && !read_fifo_full) state_next=ST_ADDRESS;
        end
        ST_ADDRESS: if(axi_arvalid && axi_arready) state_next=ST_DATA;
        ST_DATA: if(axi_rvalid && axi_rlast) state_next=ST_IDLE;
        ST_RESET: if(read_wait_count==7) state_next=ST_SETTLE;
        ST_SETTLE: if(read_wait_count==31 && !read_fifo_busy) state_next=ST_IDLE;
        default: state_next=ST_IDLE;
    endcase
end
always @(posedge axi_clk or posedge axi_reset) begin
    if(axi_reset) state_current<=ST_IDLE;
    else state_current<=state_next;
end
always @(posedge axi_clk or posedge axi_reset) begin
    if(axi_reset) begin
        read_epoch_sync<=0;read_epoch_seen<=0;restart_pending<=1;
        read_fifo_reset<=1;reader_has_frame<=0;read_wait_count<=0;
        read_frame_advance<=0;read_address_offset<=0;read_bus_address<=C_BASE_ADDR;
        axi_arvalid<=0;read_fifo_write_enable<=0;read_fifo_write_data<=0;
    end else begin
        read_epoch_sync<={read_epoch_sync[1:0],read_epoch};
        read_frame_advance<=0;
        read_fifo_write_enable<=0;
        if(read_restart) begin read_epoch_seen<=read_epoch_sync[2];restart_pending<=1;reader_has_frame<=0;end
        if(axi_rvalid && state_current==ST_DATA && !restart_pending && !read_restart && !read_fifo_reset) begin
            read_fifo_write_enable<=1;read_fifo_write_data<=axi_rdata;
        end
        case(state_current)
            ST_IDLE: begin
                if(state_next==ST_RESET) begin
                    restart_pending<=0;read_fifo_reset<=1;reader_has_frame<=0;
                    read_wait_count<=0;read_address_offset<=0;read_frame_advance<=1;
                end else if(state_next==ST_ADDRESS) begin
                    read_bus_address<=C_BASE_ADDR+{(history_compare && read_address_offset>=C_RD_END_ADDR/2 ? history_read_b : read_frame_index),read_address_offset};
                    axi_arvalid<=1;
                end
            end
            ST_ADDRESS: if(axi_arvalid && axi_arready) begin
                axi_arvalid<=0;read_address_offset<=read_address_offset+READ_ADDRESS_INCREMENT;
            end
            ST_RESET: begin
                if(read_wait_count==7) begin read_fifo_reset<=0;read_wait_count<=0;end
                else read_wait_count<=read_wait_count+1'b1;
            end
            ST_SETTLE: begin
                if(read_wait_count!=31) read_wait_count<=read_wait_count+1'b1;
                if(state_next==ST_IDLE) reader_has_frame<=history_valid && !restart_pending && !read_restart;
            end
            default: begin end
        endcase
    end
end

assign axi_arid = {C_ID_LEN{1'b0}};
assign axi_arlen = C_R_BURST_LEN - 1'b1;
assign axi_araddr = read_bus_address;
assign axi_rready = 1'b1;


assign tp_o = {
    fifo_overflow_3, fifo_overflow_2, fifo_overflow_1, fifo_overflow_0,
    fifo_underflow_3, fifo_underflow_2, fifo_underflow_1, fifo_underflow_0,
    scheduler_busy, scheduler_error, frame_active_pclk, frame_good_sync,
    scheduler_line[7:0], scheduler_chunk, scheduler_route,
    read_fifo_empty, fifo_reset_busy, fifo_soft_reset
};

wire unused_source_counters = source_good_frame_count[0] ^
                              source_bad_frame_count[0];

endmodule
