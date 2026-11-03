`timescale 1ns/1ps
// Three exclusively owned DDR candidate-map slots. A camera frame is accepted
// only at SOP after receiving a grant; no mid-frame enable or partial publish.
// Published slots remain busy until the processing/rendering owner releases them.
module canny_capture #(
    parameter SUPPORT_COMPARE=0,
    parameter WIDTH=1920, parameter HEIGHT=1200,
    parameter [31:0] BASE_ADDRESS=32'h02000000,
    parameter [31:0] SLOT_BYTES=32'h00100000,
    parameter PAD_ROWS=0, parameter SUPPORT_GRAY=0, parameter SLOTS=3
) (
    input wire [1:0] compare_i,input wire compare_start_i,input wire compare_valid_i,input wire compare_eop_i,
    output wire admit_ready_o,output wire [1:0] completed_compare_o,
    input wire camera_clk_i, input wire camera_rst_n_i,
    input wire axi_clk_i, input wire axi_rst_n_i,
    input wire valid_i, input wire sop_i, input wire eop_i, input wire [7:0] state_i,
    output wire completed_valid_o, input wire completed_ready_i,
    output wire [1:0] completed_slot_o, output wire [31:0] completed_address_o,
    input wire raw_i, input wire [31:0] raw_data_i,
    output wire completed_raw_o,
    input wire background_tag_i, output wire completed_background_tag_o,
    input wire release_i, input wire [1:0] release_slot_i,
    // Camera-domain raw-frame boundary and integrity result.
    input wire frame_check_i, input wire abort_frame_i,
    output wire dropped_frame_o,
    output wire error_o,
    output wire [31:0] axi_awaddr_o, output wire [7:0] axi_awlen_o,
    output wire axi_awvalid_o, input wire axi_awready_i,
    output wire [127:0] axi_wdata_o, output wire [15:0] axi_wstrb_o,
    output wire axi_wlast_o, output wire axi_wvalid_o, input wire axi_wready_i,
    input wire [1:0] axi_bresp_i, input wire axi_bvalid_i, output wire axi_bready_o
);
    localparam FRAME_BEATS=WIDTH*HEIGHT/64;
    localparam BURST_BEATS=32;
    localparam PAYLOAD_BEATS=PAD_ROWS ? WIDTH/64 : BURST_BEATS;
    localparam ALLOCATE=0, WAIT_DATA=1, ADDRESS=2, DATA=3, RESPONSE=4, PUBLISH=5,
        VERIFY=6, RECOVER=7, RESET_FIFO=8, SETTLE_FIFO=9;
    reg [3:0] state,next_state;
    reg [3:0] free_slots;
    reg [1:0] slot;
    reg [31:0] slot_address,address;
    reg [18:0] completed_beats;
    reg [7:0] beat_count;
    reg grant_toggle;
    (* async_reg="true" *) reg [2:0] grant_sync;
    reg accepted_grant,capturing;
    reg capture_error,frame_finished,frame_checked,compare_reserved;
    reg fifo_recovery_reset,slot_owned,dropped_frame;
    reg [5:0] reset_count;
    (* async_reg="true" *) reg [2:0] recovery_sync;
    // The sensor byte clock can stop in vertical blank. Hold the reset level
    // until its assertion has crossed to that domain and returned to AXI.
    (* async_reg="true" *) reg [2:0] recovery_ack_sync;
    (* async_reg="true" *) reg [2:0] finished_sync,checked_sync;
    reg [20:0] input_count;
    reg error;
    (* async_reg="true" *) reg [2:0] capture_error_sync;
    wire take_frame=valid_i && sop_i && grant_sync[2]!=accepted_grant && !capture_error && !recovery_sync[2];
    wire write_enable=valid_i && (capturing || take_frame) && !capture_error && !recovery_sync[2];
    // A source boundary belongs to this grant only until its first check.
    // Later camera frames can arrive while the last B response or publication
    // is stalled. They must never revoke a complete, already checked image.
    wire check_owned_frame=take_frame || (!frame_checked && (capturing || frame_finished || compare_reserved));
    wire fifo_empty,fifo_full,fifo_busy,fifo_overflow,fifo_underflow;
    wire [127:0] fifo_data;
    wire [9:0] fifo_count;

    reg captured_raw;reg [1:0] captured_compare;
    (* async_reg="true" *) reg [5:0] compare_sync;
    wire compare_done,compare_safe,compare_fifo_busy,compare_camera_error,compare_bus_error;
    wire [31:0] primary_awaddr_o;wire [7:0] primary_awlen_o;
    wire primary_awvalid_o,primary_awready_i,primary_wvalid_o,primary_wready_i,primary_wlast_o;
    wire [127:0] primary_wdata_o;wire [15:0] primary_wstrb_o;
    wire [1:0] primary_bresp_i;wire primary_bvalid_i,primary_bready_o;
    wire data_fire=primary_wvalid_o && primary_wready_i;
    assign admit_ready_o=grant_sync[2]!=accepted_grant && !capture_error && !recovery_sync[2];
    assign completed_compare_o=SUPPORT_COMPARE ? compare_sync[5:4] : 2'b00;
    reg captured_background_tag;
    (* async_reg="true" *) reg [2:0] background_tag_sync;
    (* async_reg="true" *) reg [2:0] raw_kind_sync;
    wire [8:0] payload_beats=(SUPPORT_GRAY && raw_kind_sync[2]) ? WIDTH/16 : PAYLOAD_BEATS;
    wire [8:0] burst_beats=(SUPPORT_GRAY && raw_kind_sync[2]) ? 128 : BURST_BEATS;
    wire [18:0] frame_beats=(SUPPORT_GRAY && raw_kind_sync[2]) ? WIDTH*HEIGHT/16 : FRAME_BEATS;
    wire padding_beat=beat_count>=payload_beats;
    reg [1:0] pack_index;
    reg [31:0] pack_accumulator,packed_data;
    reg packed_valid;
    always @(posedge camera_clk_i or negedge camera_rst_n_i) begin
        if(!camera_rst_n_i) begin
            captured_raw<=0;captured_compare<=0; captured_background_tag<=0; pack_index<=0; pack_accumulator<=0; packed_data<=0; packed_valid<=0;
        end else begin
            packed_valid<=0;
            if(recovery_sync[2]) begin pack_index<=0; pack_accumulator<=0; end
            else if(write_enable) begin
                if(take_frame) begin captured_compare<=SUPPORT_COMPARE ? compare_i : 2'b00;captured_raw<=SUPPORT_GRAY && raw_i; captured_background_tag<=background_tag_i; end
                if(SUPPORT_GRAY && raw_i) begin
                    // FIFO earliest byte is high; DDR canonical pixel zero is low.
                    packed_data<={raw_data_i[7:0],raw_data_i[15:8],raw_data_i[23:16],raw_data_i[31:24]};
                    packed_valid<=1; pack_index<=0;
                end else if(take_frame) begin
                    pack_accumulator<={24'd0,state_i}; pack_index<=1;
                end else begin
                    pack_accumulator<={pack_accumulator[23:0],state_i}; pack_index<=pack_index+1'b1;
                    if(pack_index==3) begin packed_data<={pack_accumulator[23:0],state_i}; packed_valid<=1; end
                end
            end
        end
    end
    always @(posedge axi_clk_i or negedge axi_rst_n_i) begin
        if(!axi_rst_n_i) raw_kind_sync<=0;
        else raw_kind_sync<={raw_kind_sync[1:0],captured_raw};
    end
    always @(posedge axi_clk_i or negedge axi_rst_n_i) begin
        if(!axi_rst_n_i) compare_sync<=0;
        else compare_sync<={compare_sync[3:0],captured_compare};
    end
    assign completed_raw_o=SUPPORT_GRAY && raw_kind_sync[2];
    // 请求标记从SOP保持到完整发布；与像素格式采用相同的稳定数据跨域。
    always @(posedge axi_clk_i or negedge axi_rst_n_i) begin
        if(!axi_rst_n_i) background_tag_sync<=0;
        else background_tag_sync<={background_tag_sync[1:0],captured_background_tag};
    end
    assign completed_background_tag_o=background_tag_sync[2];
    W0_FIFO_32 candidate_fifo (
        .wr_clk_i(camera_clk_i),.rd_clk_i(axi_clk_i),.a_rst_i(!camera_rst_n_i || !axi_rst_n_i || fifo_recovery_reset),
        .wr_en_i(packed_valid && !capture_error && !recovery_sync[2]),.rd_en_i(data_fire && !padding_beat),.wdata(packed_data),.rdata(fifo_data),
        .rst_busy(fifo_busy),.full_o(fifo_full),.empty_o(fifo_empty),.rd_datacount_o(fifo_count),
        .wr_datacount_o(),.prog_empty_o(),
        .underflow_o(fifo_underflow),.overflow_o(fifo_overflow)
    );
    genvar octet;
    generate for(octet=0;octet<16;octet=octet+1) begin : canonical_byte_order
        assign primary_wdata_o[octet*8+:8]=padding_beat ? 8'd0 : fifo_data[(15-octet)*8+:8];
    end endgenerate
    always @(posedge camera_clk_i or negedge camera_rst_n_i) begin
        if(!camera_rst_n_i) begin
            grant_sync<=0; accepted_grant<=0; capturing<=0; input_count<=0; capture_error<=0;
            recovery_sync<=0; frame_finished<=0; frame_checked<=0;compare_reserved<=0;
        end else begin
            grant_sync<={grant_sync[1:0],grant_toggle};
            recovery_sync<={recovery_sync[1:0],fifo_recovery_reset};
            if(recovery_sync[2]) begin
                capturing<=0; input_count<=0; capture_error<=0;
                frame_finished<=0; frame_checked<=0;compare_reserved<=0;
                // Retire even an unused grant before AXI allocates again.
                accepted_grant<=grant_sync[2];
            end else begin
                if(SUPPORT_COMPARE && compare_start_i) begin compare_reserved<=1;frame_checked<=0;frame_finished<=0;end
                if(SUPPORT_COMPARE && compare_camera_error && compare_reserved) begin capture_error<=1;capturing<=0;end
                if(take_frame) begin
                    if(!SUPPORT_COMPARE || !compare_i) compare_reserved<=0;
                    accepted_grant<=grant_sync[2]; capturing<=1; input_count<=1;
                    frame_finished<=0; frame_checked<=0;
                end else if(write_enable) input_count<=input_count+1'b1;
                if(frame_check_i && check_owned_frame) frame_checked<=1;
                if(write_enable && eop_i) begin
                    capturing<=0; frame_finished<=1;
                    if(input_count!=WIDTH*HEIGHT/4-1) capture_error<=1;
                end
                if(write_enable && (fifo_full || fifo_busy)) begin capture_error<=1; capturing<=0; end
                if(packed_valid && (fifo_full || fifo_busy)) begin capture_error<=1; capturing<=0; end
                if(capturing && sop_i) begin capture_error<=1; capturing<=0; end
                if(abort_frame_i && check_owned_frame) begin capture_error<=1; capturing<=0; end
            end
        end
    end
    always @(posedge axi_clk_i or negedge axi_rst_n_i) begin
        if(!axi_rst_n_i) state<=ALLOCATE;
        else state<=next_state;
    end
    always @(*) begin
        next_state=state;
        case(state)
            ALLOCATE: if(capture_error_sync[2]) next_state=RECOVER;
                else if(free_slots!=0 && !fifo_busy && !compare_fifo_busy && !error) next_state=WAIT_DATA;
            WAIT_DATA: if(capture_error_sync[2]) next_state=RECOVER;
                else if(fifo_count>=payload_beats && !fifo_busy) next_state=ADDRESS;
            ADDRESS: if(primary_awready_i) next_state=DATA;
            DATA: if(data_fire && beat_count==burst_beats-1) next_state=RESPONSE;
            RESPONSE: if(primary_bvalid_i) begin
                if(capture_error_sync[2] || primary_bresp_i!=0) next_state=RECOVER;
                else next_state=completed_beats+payload_beats==frame_beats ? VERIFY : WAIT_DATA;
            end
            VERIFY: if(capture_error_sync[2]) next_state=RECOVER;
                else if(finished_sync[2] && checked_sync[2] && (!completed_compare_o || compare_done) && !error) next_state=PUBLISH;
            PUBLISH: if(completed_ready_i) next_state=ALLOCATE;
            RECOVER: next_state=RESET_FIFO;
            RESET_FIFO: if(reset_count==15 && recovery_ack_sync[2] && compare_safe) next_state=SETTLE_FIFO;
            SETTLE_FIFO: if(reset_count==63 && !fifo_busy && !capture_error_sync[2] && !recovery_ack_sync[2]) next_state=ALLOCATE;
            default: next_state=ALLOCATE;
        endcase
    end
    always @(posedge axi_clk_i or negedge axi_rst_n_i) begin
        if(!axi_rst_n_i) begin
            free_slots<=SLOTS==4 ? 4'b1111 : 4'b0111; slot<=0; slot_address<=BASE_ADDRESS; address<=BASE_ADDRESS;
            completed_beats<=0; beat_count<=0; grant_toggle<=0; error<=0; capture_error_sync<=0;
            fifo_recovery_reset<=0; reset_count<=0; slot_owned<=0; dropped_frame<=0;
            finished_sync<=0; checked_sync<=0;
            recovery_ack_sync<=0;
        end else begin
            recovery_ack_sync<={recovery_ack_sync[1:0],recovery_sync[2]};
            capture_error_sync<={capture_error_sync[1:0],capture_error};
            finished_sync<={finished_sync[1:0],frame_finished};
            checked_sync<={checked_sync[1:0],frame_checked};
            if(capture_error_sync[2]) dropped_frame<=1;
            if(state==PUBLISH && completed_ready_i) slot_owned<=0;
            // Never reset a FIFO or reclaim a slot until AW/W/B are complete.
            if(state==RECOVER) begin
                fifo_recovery_reset<=1; reset_count<=0;
                if(slot_owned) free_slots[slot]<=1;
                slot_owned<=0;
            end
            if(state==RESET_FIFO) begin
                if(reset_count!=15) reset_count<=reset_count+1'b1;
                if(next_state==SETTLE_FIFO) begin fifo_recovery_reset<=0; reset_count<=0; end
            end
            if(state==SETTLE_FIFO && reset_count!=63) reset_count<=reset_count+1'b1;
            if(release_i) free_slots[release_slot_i]<=1;
            if(state==ALLOCATE && next_state==WAIT_DATA) begin
                if(free_slots[0]) begin slot<=0; slot_address<=BASE_ADDRESS; address<=BASE_ADDRESS; free_slots[0]<=0; end
                else if(free_slots[1]) begin slot<=1; slot_address<=BASE_ADDRESS+SLOT_BYTES; address<=BASE_ADDRESS+SLOT_BYTES; free_slots[1]<=0; end
                else if(free_slots[2]) begin slot<=2; slot_address<=BASE_ADDRESS+2*SLOT_BYTES; address<=BASE_ADDRESS+2*SLOT_BYTES; free_slots[2]<=0; end
                else begin slot<=3; slot_address<=BASE_ADDRESS+3*SLOT_BYTES; address<=BASE_ADDRESS+3*SLOT_BYTES; free_slots[3]<=0; end
                grant_toggle<=!grant_toggle; completed_beats<=0; beat_count<=0; slot_owned<=1;
            end
            if(state==ADDRESS) beat_count<=0;
            if(data_fire) beat_count<=beat_count+1'b1;
            if(state==RESPONSE && primary_bvalid_i) begin
                completed_beats<=completed_beats+payload_beats;
                address<=address+{burst_beats,4'b0000};
                if(primary_bresp_i!=0) error<=1;
            end
            if(fifo_underflow || compare_bus_error) error<=1;
        end
    end
    assign completed_valid_o=state==PUBLISH && !error;
    assign completed_slot_o=slot;
    assign completed_address_o=slot_address;
    assign error_o=error;
    assign dropped_frame_o=dropped_frame;
    assign primary_awaddr_o=address;
    assign primary_awlen_o=burst_beats-1;
    assign primary_awvalid_o=state==ADDRESS;
    assign primary_wstrb_o=16'hffff;
    assign primary_wlast_o=beat_count==burst_beats-1;
    assign primary_wvalid_o=state==DATA && (padding_beat || !fifo_empty);
    assign primary_bready_o=state==RESPONSE;
    generate if(SUPPORT_COMPARE) begin : paired_raw_plane
        wire [31:0] raw_address;wire [7:0] raw_length;
        wire raw_awvalid,raw_awready,raw_wvalid,raw_wready,raw_wlast,raw_bvalid,raw_bready;
        wire [127:0] raw_data;wire [15:0] raw_strobe;wire [1:0] raw_response;
        canny_compare_capture #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) raw_capture(
            .camera_clk_i(camera_clk_i),.camera_rst_n_i(camera_rst_n_i),.axi_clk_i(axi_clk_i),.axi_rst_n_i(axi_rst_n_i),
            .start_i(compare_start_i),.valid_i(compare_valid_i),.eop_i(compare_eop_i),.raw_i(raw_data_i),
            .slot_address_i(slot_address),.abort_i(fifo_recovery_reset),.safe_o(compare_safe),.fifo_busy_o(compare_fifo_busy),
            .camera_error_o(compare_camera_error),.done_o(compare_done),.error_o(compare_bus_error),
            .axi_awaddr_o(raw_address),.axi_awlen_o(raw_length),.axi_awvalid_o(raw_awvalid),.axi_awready_i(raw_awready),
            .axi_wdata_o(raw_data),.axi_wstrb_o(raw_strobe),.axi_wlast_o(raw_wlast),.axi_wvalid_o(raw_wvalid),.axi_wready_i(raw_wready),
            .axi_bresp_i(raw_response),.axi_bvalid_i(raw_bvalid),.axi_bready_o(raw_bready));
        axi_pair_write_mux writer_mux(
            .clk_i(axi_clk_i),.rst_n_i(axi_rst_n_i),
            .s_awaddr_i({raw_address,primary_awaddr_o}),.s_awlen_i({raw_length,primary_awlen_o}),
            .s_awvalid_i({raw_awvalid,primary_awvalid_o}),.s_awready_o({raw_awready,primary_awready_i}),
            .s_wdata_i({raw_data,primary_wdata_o}),.s_wstrb_i({raw_strobe,primary_wstrb_o}),
            .s_wlast_i({raw_wlast,primary_wlast_o}),.s_wvalid_i({raw_wvalid,primary_wvalid_o}),.s_wready_o({raw_wready,primary_wready_i}),
            .s_bresp_o({raw_response,primary_bresp_i}),.s_bvalid_o({raw_bvalid,primary_bvalid_i}),.s_bready_i({raw_bready,primary_bready_o}),
            .m_awaddr_o(axi_awaddr_o),.m_awlen_o(axi_awlen_o),.m_awvalid_o(axi_awvalid_o),.m_awready_i(axi_awready_i),
            .m_wdata_o(axi_wdata_o),.m_wstrb_o(axi_wstrb_o),.m_wlast_o(axi_wlast_o),.m_wvalid_o(axi_wvalid_o),.m_wready_i(axi_wready_i),
            .m_bresp_i(axi_bresp_i),.m_bvalid_i(axi_bvalid_i),.m_bready_o(axi_bready_o));
    end else begin : single_plane
        assign compare_done=1;assign compare_safe=1;assign compare_fifo_busy=0;assign compare_camera_error=0;assign compare_bus_error=0;
        assign axi_awaddr_o=primary_awaddr_o;
        assign axi_awlen_o=primary_awlen_o;
        assign axi_awvalid_o=primary_awvalid_o;
        assign primary_awready_i=axi_awready_i;
        assign axi_wdata_o=primary_wdata_o;
        assign axi_wstrb_o=primary_wstrb_o;
        assign axi_wlast_o=primary_wlast_o;
        assign axi_wvalid_o=primary_wvalid_o;
        assign primary_wready_i=axi_wready_i;
        assign primary_bresp_i=axi_bresp_i;
        assign primary_bvalid_i=axi_bvalid_i;
        assign axi_bready_o=primary_bready_o;
    end endgenerate
endmodule
