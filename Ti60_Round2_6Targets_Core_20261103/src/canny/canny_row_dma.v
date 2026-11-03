`timescale 1ns/1ps
// One complete two-bit image row per command, split at AXI 4 KiB boundaries.
// WIDTH must be a multiple of 64, with at most 256 AXI beats per row. Row
// scheduling remain in camera_clk_i; the official FIFOs implement the CDC.
// Command fields remain stable from request toggle through returned acknowledge.
module canny_row_dma #(parameter WIDTH=1920, parameter PADDING_BEATS=0, parameter SUPPORT_GRAY=0) (
    input wire camera_clk_i, input wire camera_rst_n_i,
    input wire axi_clk_i, input wire axi_rst_n_i,
    input wire read_gray_i,
    input wire read_request_i, output wire read_request_ready_o,
    input wire [31:0] read_address_i,
    output wire read_valid_o, input wire read_ready_i, output wire [63:0] read_data_o,
    input wire write_request_i, output wire write_request_ready_o,
    input wire [31:0] write_address_i,
    input wire write_valid_i, output wire write_ready_o,
    input wire [63:0] write_data_i, input wire write_last_i,
    output wire write_done_o, output wire error_o,
    output wire [31:0] axi_awaddr_o, output wire [7:0] axi_awlen_o,
    output wire axi_awvalid_o, input wire axi_awready_i,
    output wire [127:0] axi_wdata_o, output wire [15:0] axi_wstrb_o,
    output wire axi_wlast_o, output wire axi_wvalid_o, input wire axi_wready_i,
    input wire [1:0] axi_bresp_i, input wire axi_bvalid_i, output wire axi_bready_o,
    output wire [31:0] axi_araddr_o, output wire [7:0] axi_arlen_o,
    output wire axi_arvalid_o, input wire axi_arready_i,
    input wire [127:0] axi_rdata_i, input wire axi_rlast_i,
    input wire [1:0] axi_rresp_i, input wire axi_rvalid_i, output wire axi_rready_o
);
    localparam PAYLOAD_BEATS=WIDTH/64;
    localparam BEATS=PAYLOAD_BEATS+PADDING_BEATS;
    localparam IDLE=0, CAPTURE=1, READ_ADDRESS=2, READ_DATA=3,
        WRITE_FILL=4, WRITE_ADDRESS=5, WRITE_DATA=6, WRITE_RESPONSE=7;
    reg [2:0] state,next_state;
    reg request_toggle, request_write,request_gray;
    reg [8:0] active_payload_beats;
    reg [31:0] request_address;
    reg acknowledge_toggle;
    (* async_reg="true" *) reg [2:0] request_sync;
    (* async_reg="true" *) reg [2:0] acknowledge_sync;
    reg acknowledge_delayed;
    reg [31:0] active_address;
    reg [7:0] beat_count;
    reg [8:0] remaining_beats,burst_beats;
    reg [8:0] transferred_beats;
    wire padding_beat=transferred_beats>=active_payload_beats;
    wire [8:0] command_beats=request_gray ? 9'd128 : BEATS;
    wire [8:0] page_beats=9'd256-{1'b0,request_address[11:4]};
    wire burst_last={1'b0,beat_count}+9'd1==burst_beats;
    wire row_last=remaining_beats==burst_beats;
    reg [9:0] read_words_remaining,write_words_remaining;
    reg camera_error;
    reg error;
    (* async_reg="true" *) reg [1:0] error_sync;
    wire fifo_reset=!camera_rst_n_i || !axi_rst_n_i;
    wire command_idle=request_toggle==acknowledge_sync[2] && read_words_remaining==0;
    wire write_fifo_full,write_fifo_empty,write_fifo_busy;
    wire [127:0] write_fifo_data;
    wire [9:0] write_fifo_count;
    wire write_fifo_underflow,write_fifo_overflow;
    wire read_fifo_full,read_fifo_empty,read_fifo_busy;
    wire read_fifo_underflow,read_fifo_overflow;
    wire write_fire=axi_wvalid_o && axi_wready_i;
    wire read_fire=axi_rvalid_i && axi_rready_o;
    // ENDianESS=0 in the existing official FIFO emits the earliest word first
    // from its most significant slice. Swap halves to preserve canonical DDR
    // layout: pixel zero is in the least-significant two bits of each beat.
    W0_FIFO_64 write_fifo (
        .wr_clk_i(camera_clk_i),.rd_clk_i(axi_clk_i),.a_rst_i(fifo_reset),
        .wr_en_i(write_valid_i && write_ready_o),.rd_en_i(write_fire && !padding_beat),.wdata(write_data_i),
        .rdata(write_fifo_data),.full_o(write_fifo_full),.empty_o(write_fifo_empty),
        .rst_busy(write_fifo_busy),.rd_datacount_o(write_fifo_count),.wr_datacount_o(),
        .almost_full_o(),.prog_full_o(),.wr_ack_o(),.almost_empty_o(),.prog_empty_o(),.rd_valid_o(),
        .underflow_o(write_fifo_underflow),.overflow_o(write_fifo_overflow)
    );
    Canny_Row_Read_FIFO read_fifo (
        .wr_clk_i(axi_clk_i),.rd_clk_i(camera_clk_i),.a_rst_i(fifo_reset),
        .wr_en_i(read_fire && !padding_beat),.rd_en_i(read_valid_o && read_ready_i),
        .wdata({axi_rdata_i[63:0],axi_rdata_i[127:64]}),.rdata(read_data_o),
        .full_o(read_fifo_full),.empty_o(read_fifo_empty),.rst_busy(read_fifo_busy),
        .rd_datacount_o(),.wr_datacount_o(),.almost_full_o(),.prog_full_o(),
        .wr_ack_o(),.almost_empty_o(),.rd_valid_o(),
        .underflow_o(read_fifo_underflow),.overflow_o(read_fifo_overflow)
    );
    always @(posedge camera_clk_i or negedge camera_rst_n_i) begin
        if(!camera_rst_n_i) begin
            request_toggle<=0; request_write<=0; request_gray<=0; request_address<=0;
            acknowledge_sync<=0; acknowledge_delayed<=0; error_sync<=0;
            read_words_remaining<=0; write_words_remaining<=0; camera_error<=0;
        end else begin
            acknowledge_sync<={acknowledge_sync[1:0],acknowledge_toggle};
            acknowledge_delayed<=acknowledge_sync[2];
            error_sync<={error_sync[0],error};
            if(write_request_i && write_request_ready_o) begin
                request_address<=write_address_i; request_write<=1; request_gray<=0;
                request_toggle<=!request_toggle;
                write_words_remaining<=PAYLOAD_BEATS*2;
            end else if(read_request_i && read_request_ready_o) begin
                request_address<=read_address_i; request_write<=0; request_gray<=SUPPORT_GRAY && read_gray_i;
                request_toggle<=!request_toggle;
                read_words_remaining<=(SUPPORT_GRAY && read_gray_i) ? 10'd240 : PAYLOAD_BEATS*2;
            end
            if(write_valid_i && write_ready_o) begin
                write_words_remaining<=write_words_remaining-1'b1;
                if(write_last_i!=(write_words_remaining==1)) camera_error<=1;
            end
            if(read_valid_o && read_ready_i) read_words_remaining<=read_words_remaining-1'b1;
        end
    end
    always @(posedge axi_clk_i or negedge axi_rst_n_i) begin
        if(!axi_rst_n_i) begin state<=IDLE; request_sync<=0; end
        else begin state<=next_state; request_sync<={request_sync[1:0],request_toggle}; end
    end
    always @(*) begin
        next_state=state;
        case(state)
            IDLE: if(request_sync[2]!=acknowledge_toggle) next_state=CAPTURE;
            CAPTURE: next_state=request_write ? WRITE_FILL : READ_ADDRESS;
            READ_ADDRESS: if(axi_arready_i) next_state=READ_DATA;
            READ_DATA: if(read_fire && burst_last) next_state=row_last ? IDLE : READ_ADDRESS;
            // Reserve the shared bus only after the entire payload is staged.
            // A camera-domain producer pause after AW would otherwise block
            // all display reads until WLAST/B, despite display read priority.
            // The official read-domain count is in 128-bit beats. Padding is
            // generated locally and does not occupy this FIFO.
            WRITE_FILL: if(write_fifo_count>=PAYLOAD_BEATS && !write_fifo_busy) next_state=WRITE_ADDRESS;
            WRITE_ADDRESS: if(axi_awready_i) next_state=WRITE_DATA;
            WRITE_DATA: if(write_fire && burst_last) next_state=WRITE_RESPONSE;
            WRITE_RESPONSE: if(axi_bvalid_i) next_state=row_last ? IDLE : WRITE_ADDRESS;
            default: next_state=IDLE;
        endcase
    end
    always @(posedge axi_clk_i or negedge axi_rst_n_i) begin
        if(!axi_rst_n_i) begin
            acknowledge_toggle<=0; active_address<=0; beat_count<=0; error<=0;
            remaining_beats<=0; burst_beats<=0;
            transferred_beats<=0; active_payload_beats<=PAYLOAD_BEATS;
        end else begin
            if(state==CAPTURE) begin
                active_address<=request_address; beat_count<=0;
                transferred_beats<=0;
                remaining_beats<=command_beats; active_payload_beats<=request_gray ? 9'd120 : PAYLOAD_BEATS;
                burst_beats<=page_beats<command_beats ? page_beats : command_beats;
                if(request_address[3:0]!=0) error<=1;
            end
            if(read_fire || write_fire) begin
                beat_count<=beat_count+1'b1;
                transferred_beats<=transferred_beats+1'b1;
            end
            if(state==READ_DATA && read_fire) begin
                if(axi_rlast_i!=burst_last || axi_rresp_i!=0) error<=1;
                if(burst_last && row_last) acknowledge_toggle<=request_sync[2];
            end
            if(state==WRITE_RESPONSE && axi_bvalid_i) begin
                if(row_last) acknowledge_toggle<=request_sync[2];
                if(axi_bresp_i!=0) error<=1;
            end
            // A row is shorter than one page, so only one split is necessary.
            // Keep the command unacknowledged until both sub-bursts complete.
            if((state==READ_DATA && read_fire && burst_last) ||
               (state==WRITE_RESPONSE && axi_bvalid_i)) begin
                beat_count<=0;
                if(!row_last) begin
                    active_address<=active_address+{burst_beats,4'b0000};
                    remaining_beats<=remaining_beats-burst_beats;
                    burst_beats<=remaining_beats-burst_beats;
                end
            end
            if(write_fifo_underflow || read_fifo_overflow) error<=1;
        end
    end
    assign read_request_ready_o=command_idle && !write_request_i && !read_fifo_busy;
    assign write_request_ready_o=command_idle && !write_fifo_busy;
    assign write_ready_o=!write_fifo_full && !write_fifo_busy && write_words_remaining!=0;
    assign read_valid_o=!read_fifo_empty && !read_fifo_busy && read_words_remaining!=0;
    assign write_done_o=(acknowledge_sync[2]!=acknowledge_delayed) && request_write;
    assign error_o=error_sync[1] || camera_error || write_fifo_overflow || read_fifo_underflow;
    assign axi_awaddr_o=active_address;
    assign axi_awlen_o=burst_beats-1'b1;
    assign axi_awvalid_o=state==WRITE_ADDRESS;
    assign axi_wdata_o=padding_beat ? 128'd0 : {write_fifo_data[63:0],write_fifo_data[127:64]};
    assign axi_wstrb_o=16'hffff;
    assign axi_wlast_o=burst_last;
    assign axi_wvalid_o=state==WRITE_DATA && (padding_beat || !write_fifo_empty);
    assign axi_bready_o=state==WRITE_RESPONSE;
    assign axi_araddr_o=active_address;
    assign axi_arlen_o=burst_beats-1'b1;
    assign axi_arvalid_o=state==READ_ADDRESS;
    assign axi_rready_o=state==READ_DATA && (padding_beat || !read_fifo_full) && !read_fifo_busy;
endmodule
