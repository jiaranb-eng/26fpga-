`timescale 1ns / 1ps

// Extensible framed command protocol for FPGA runtime controls.
// Frame: A5 5A VER SEQ TYPE CMD LEN PAYLOAD... CRC8.
// CRC8 uses polynomial 0x07 over VER through the final payload byte.
module fpga_control_protocol #(
    parameter [9:0] DEFAULT_THRESHOLD = 10'd256,
    parameter [9:0] MAX_THRESHOLD     = 10'd1020,
    parameter integer FRAME_TIMEOUT_CYCLES = 960_000
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] rx_data_i,
    input  wire       rx_valid_i,
    output reg  [7:0] tx_data_o,
    output wire       tx_valid_o,
    input  wire       tx_ready_i,
    output reg  [9:0] threshold_o,
    output reg        threshold_valid_o,
    input  wire       threshold_ready_i,
    output reg [10:0] canny_low_o, canny_high_o,
    output reg canny_valid_o,
    input wire canny_ready_i,
    input  wire [2:0] display_mode_i,
    input  wire [7:0] history_count_i,
    input  wire [7:0] video_fault_i,
    input  wire       display_changed_i,
    output reg  [2:0] display_request_o,
    output reg        display_valid_o,
    output reg  [1:0] history_control_o,
    output reg  [7:0] history_age_a_o,
    output reg  [7:0] history_age_b_o,
    output reg [7:0] background_threshold_o,
    output reg background_relearn_o,
    input wire background_valid_i, input wire background_pending_i,
    input wire [31:0] background_epoch_i, input wire [31:0] background_frames_i,
    input wire [31:0] background_cycles_i, input wire [3:0] background_count_i,
    input wire background_overflow_i,
    output reg background_mask_capture_o,
    output reg [13:0] background_mask_address_o,
    input wire [7:0] background_mask_data_i,
    input wire background_mask_ready_i, input wire background_mask_busy_i,
    input wire [7:0] background_mask_epoch_i,

    output wire       osd_wr_en_o,
    output wire [5:0] osd_wr_addr_o,
    output wire [7:0] osd_wr_data_o
);

localparam [7:0] PROTOCOL_VERSION = 8'h01;
localparam [7:0] TYPE_COMMAND     = 8'h01;
localparam [7:0] TYPE_RESPONSE    = 8'h02;

localparam [7:0] CMD_PING          = 8'h00;
localparam [7:0] CMD_GET_THRESHOLD = 8'h10;
localparam [7:0] CMD_SET_THRESHOLD = 8'h11;
localparam [7:0] CMD_STEP_THRESHOLD = 8'h12;
localparam [7:0] CMD_CANNY_GET = 8'h13, CMD_CANNY_SET = 8'h14;
localparam [7:0] CMD_OSD_WRITE       = 8'h20;
localparam [7:0] CMD_DISPLAY        = 8'h30;
localparam [7:0] CMD_HISTORY        = 8'h31;
localparam [7:0] CMD_STATUS         = 8'h32;
localparam [7:0] CMD_BACKGROUND_STATUS = 8'h40;
localparam [7:0] CMD_BACKGROUND_RELEARN = 8'h41;
localparam [7:0] CMD_BACKGROUND_THRESHOLD = 8'h42;
localparam [7:0] CMD_MASK_CAPTURE = 8'h43;
localparam [7:0] CMD_MASK_READ = 8'h44;
localparam [13:0] MASK_CELLS = 14'd9000;

localparam [7:0] STATUS_OK          = 8'h00;
localparam [7:0] STATUS_BAD_VERSION = 8'h01;
localparam [7:0] STATUS_BAD_TYPE    = 8'h02;
localparam [7:0] STATUS_BAD_LENGTH  = 8'h03;
localparam [7:0] STATUS_BAD_COMMAND = 8'h04;
localparam [7:0] STATUS_RANGE_ERROR = 8'h05;
localparam [7:0] STATUS_BUSY        = 8'h06;

localparam [3:0] RX_SOF0    = 4'd0;
localparam [3:0] RX_SOF1    = 4'd1;
localparam [3:0] RX_VERSION = 4'd2;
localparam [3:0] RX_SEQ     = 4'd3;
localparam [3:0] RX_TYPE    = 4'd4;
localparam [3:0] RX_CMD     = 4'd5;
localparam [3:0] RX_LEN     = 4'd6;
localparam [3:0] RX_PAYLOAD = 4'd7;
localparam [3:0] RX_CRC     = 4'd8;

reg [3:0] rx_state;
reg [7:0] rx_crc;
reg [7:0] packet_version;
reg [7:0] packet_seq;
reg [7:0] packet_type;
reg [7:0] packet_command;
reg [7:0] packet_length;
reg [4:0] payload_index;
reg [7:0] payload [0:15];
reg [19:0] timeout_count;

reg       osd_commit_active;
reg [5:0] osd_commit_addr;
reg [4:0] osd_commit_index;

reg       response_active;
reg [4:0] response_position;
reg [7:0] response_crc;
reg [7:0] response_seq;
reg [7:0] response_command;
reg [7:0] response_status;
reg [9:0] response_threshold;
reg response_extended;
reg response_canny;
reg [10:0] response_canny_low,response_canny_high;
reg [2:0] response_display;
reg [1:0] response_history;
reg [7:0] response_count,response_age_a,response_age_b,response_fault;
reg response_background;
reg [7:0] response_background_threshold,response_background_flags;
reg [3:0] response_background_count;
reg [31:0] response_background_epoch,response_background_frames,response_background_cycles;
reg response_mask;
reg [7:0] response_mask_epoch;
reg [13:0] response_mask_address;
reg [95:0] response_mask_data;
reg mask_read_active, mask_read_wait;
reg [3:0] mask_read_index;
wire [4:0] response_last = (response_background || response_mask) ? 5'd23 : (response_canny ? 5'd12 : (response_extended ? 5'd16 : 5'd10));

wire [15:0] requested_threshold = {payload[1], payload[0]};
assign tx_valid_o = response_active;
assign osd_wr_en_o   = osd_commit_active;
assign osd_wr_addr_o = osd_commit_addr;
assign osd_wr_data_o = payload[osd_commit_index];

function [7:0] crc8_next;
    input [7:0] crc_i;
    input [7:0] data_i;
    integer bit_number;
    reg [7:0] crc_work;
    begin
        crc_work = crc_i ^ data_i;
        for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
            if (crc_work[7]) begin
                crc_work = (crc_work << 1) ^ 8'h07;
            end else begin
                crc_work = crc_work << 1;
            end
        end
        crc8_next = crc_work;
    end
endfunction

always @* begin
    if(response_canny && response_position>=8) begin
        case(response_position)
            5'd8: tx_data_o=response_canny_low[7:0];
            5'd9: tx_data_o={5'd0,response_canny_low[10:8]};
            5'd10: tx_data_o=response_canny_high[7:0];
            5'd11: tx_data_o={5'd0,response_canny_high[10:8]};
            default: tx_data_o=response_crc;
        endcase
    end else if(response_mask && response_position>=8) begin
        case(response_position)
            5'd8: tx_data_o=response_mask_epoch;
            5'd9: tx_data_o=response_mask_address[7:0];
            5'd10: tx_data_o={2'd0,response_mask_address[13:8]};
            5'd11: tx_data_o=response_mask_data[7:0];
            5'd12: tx_data_o=response_mask_data[15:8];
            5'd13: tx_data_o=response_mask_data[23:16];
            5'd14: tx_data_o=response_mask_data[31:24];
            5'd15: tx_data_o=response_mask_data[39:32];
            5'd16: tx_data_o=response_mask_data[47:40];
            5'd17: tx_data_o=response_mask_data[55:48];
            5'd18: tx_data_o=response_mask_data[63:56];
            5'd19: tx_data_o=response_mask_data[71:64];
            5'd20: tx_data_o=response_mask_data[79:72];
            5'd21: tx_data_o=response_mask_data[87:80];
            5'd22: tx_data_o=response_mask_data[95:88];
            default: tx_data_o=response_crc;
        endcase
    end else if(response_background && response_position>=8) begin
        case(response_position)
            5'd8: tx_data_o=response_background_threshold;
            5'd9: tx_data_o=response_background_flags;
            5'd10: tx_data_o={4'd0,response_background_count};
            5'd11: tx_data_o=response_background_epoch[7:0];
            5'd12: tx_data_o=response_background_epoch[15:8];
            5'd13: tx_data_o=response_background_epoch[23:16];
            5'd14: tx_data_o=response_background_epoch[31:24];
            5'd15: tx_data_o=response_background_frames[7:0];
            5'd16: tx_data_o=response_background_frames[15:8];
            5'd17: tx_data_o=response_background_frames[23:16];
            5'd18: tx_data_o=response_background_frames[31:24];
            5'd19: tx_data_o=response_background_cycles[7:0];
            5'd20: tx_data_o=response_background_cycles[15:8];
            5'd21: tx_data_o=response_background_cycles[23:16];
            5'd22: tx_data_o=response_background_cycles[31:24];
            default: tx_data_o=response_crc;
        endcase
    end else begin
    case (response_position)
        4'd0: tx_data_o = 8'ha5;
        4'd1: tx_data_o = 8'h5a;
        4'd2: tx_data_o = PROTOCOL_VERSION;
        4'd3: tx_data_o = response_seq;
        4'd4: tx_data_o = TYPE_RESPONSE;
        4'd5: tx_data_o = response_command;
        4'd6: tx_data_o = (response_background || response_mask) ? 8'd16 : (response_canny ? 8'd5 : (response_extended ? 8'd9 : 8'd3));
        4'd7: tx_data_o = response_status;
        4'd8: tx_data_o = response_threshold[7:0];
        4'd9: tx_data_o = {6'd0, response_threshold[9:8]};
        5'd10: tx_data_o = response_extended ? {5'd0,response_display} : response_crc;
        5'd11: tx_data_o = {6'd0,response_history};
        5'd12: tx_data_o = response_count;
        5'd13: tx_data_o = response_age_a;
        5'd14: tx_data_o = response_age_b;
        5'd15: tx_data_o = response_fault;
        default: tx_data_o = response_crc;
    endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_state           <= RX_SOF0;
        rx_crc             <= 8'd0;
        packet_version     <= 8'd0;
        packet_seq         <= 8'd0;
        packet_type        <= 8'd0;
        packet_command     <= 8'd0;
        packet_length      <= 8'd0;
        payload_index      <= 5'd0;
        timeout_count      <= 20'd0;
        response_active    <= 1'b0;
        response_position  <= 4'd0;
        response_crc       <= 8'd0;
        response_seq       <= 8'd0;
        response_command   <= 8'd0;
        response_status    <= STATUS_OK;
        response_threshold <= DEFAULT_THRESHOLD;
        threshold_o        <= DEFAULT_THRESHOLD;
        canny_low_o<=11'd10;canny_high_o<=11'd20;canny_valid_o<=0;
        response_canny<=0;response_canny_low<=10;response_canny_high<=20;
        threshold_valid_o  <= 1'b0;
        osd_commit_active   <= 1'b0;
        osd_commit_addr     <= 6'd0;
        osd_commit_index    <= 5'd0;
        display_request_o<=3'd3;display_valid_o<=0;
        background_threshold_o<=8'd12; background_relearn_o<=0;
        background_mask_capture_o<=0; background_mask_address_o<=0;
        response_mask<=0; response_mask_epoch<=0; response_mask_address<=0; response_mask_data<=0;
        mask_read_active<=0; mask_read_wait<=0; mask_read_index<=0;
        response_background<=0; response_background_threshold<=12; response_background_flags<=0;
        response_background_count<=0; response_background_epoch<=0; response_background_frames<=0; response_background_cycles<=0;
        history_control_o<=0;history_age_a_o<=0;history_age_b_o<=0;
        response_extended<=0;response_display<=0;response_history<=0;
        response_count<=0;response_age_a<=0;response_age_b<=0;response_fault<=0;
    end else begin
        threshold_valid_o <= 1'b0;
        canny_valid_o<=0;
        display_valid_o<=0;
        background_relearn_o<=0;
        background_mask_capture_o<=0;

        // 每个地址等待同步RAM一拍再采样；整个响应先锁存后交给邮箱。
        if(mask_read_active) begin
            if(mask_read_wait) mask_read_wait<=0;
            else begin
                response_mask_data[mask_read_index*8+:8] <= background_mask_address_o < MASK_CELLS ? background_mask_data_i : 8'd0;
                if(mask_read_index==11) begin
                    mask_read_active<=0;
                    response_active<=1;
                end else begin
                    background_mask_address_o<=background_mask_address_o+1'b1;
                    mask_read_index<=mask_read_index+1'b1;
                    mask_read_wait<=1;
                end
            end
        end

        if (osd_commit_active) begin
            if (osd_commit_index + 1'b1 == packet_length) begin
                osd_commit_active <= 1'b0;
            end else begin
                osd_commit_addr  <= osd_commit_addr + 1'b1;
                osd_commit_index <= osd_commit_index + 1'b1;
            end
        end

        if (response_active && tx_ready_i) begin
            if ((response_position >= 5'd2) && (response_position < response_last)) begin
                response_crc <= crc8_next(response_crc, tx_data_o);
            end
            if (response_position == response_last) begin
                response_active   <= 1'b0;
                response_position <= 4'd0;
            end else begin
                response_position <= response_position + 1'b1;
            end
        end

        if ((rx_state != RX_SOF0) && (rx_state != RX_SOF1) && !rx_valid_i) begin
            if (timeout_count == FRAME_TIMEOUT_CYCLES - 1) begin
                rx_state      <= RX_SOF0;
                timeout_count <= 20'd0;
            end else begin
                timeout_count <= timeout_count + 1'b1;
            end
        end else if (rx_valid_i) begin
            timeout_count <= 20'd0;
        end

        if (rx_valid_i) begin
            case (rx_state)
                RX_SOF0: begin
                    if (rx_data_i == 8'ha5) begin
                        rx_state <= RX_SOF1;
                    end
                end

                RX_SOF1: begin
                    if (rx_data_i == 8'h5a) begin
                        rx_state <= RX_VERSION;
                    end else if (rx_data_i != 8'ha5) begin
                        rx_state <= RX_SOF0;
                    end
                end

                RX_VERSION: begin
                    packet_version <= rx_data_i;
                    rx_crc         <= crc8_next(8'd0, rx_data_i);
                    rx_state       <= RX_SEQ;
                end

                RX_SEQ: begin
                    packet_seq <= rx_data_i;
                    rx_crc     <= crc8_next(rx_crc, rx_data_i);
                    rx_state   <= RX_TYPE;
                end

                RX_TYPE: begin
                    packet_type <= rx_data_i;
                    rx_crc      <= crc8_next(rx_crc, rx_data_i);
                    rx_state    <= RX_CMD;
                end

                RX_CMD: begin
                    packet_command <= rx_data_i;
                    rx_crc         <= crc8_next(rx_crc, rx_data_i);
                    rx_state       <= RX_LEN;
                end

                RX_LEN: begin
                    packet_length <= rx_data_i;
                    payload_index <= 5'd0;
                    rx_crc        <= crc8_next(rx_crc, rx_data_i);
                    if (rx_data_i == 8'd0) begin
                        rx_state <= RX_CRC;
                    end else if (rx_data_i <= 8'd16) begin
                        rx_state <= RX_PAYLOAD;
                    end else begin
                        rx_state <= RX_SOF0;
                    end
                end

                RX_PAYLOAD: begin
                    payload[payload_index] <= rx_data_i;
                    rx_crc <= crc8_next(rx_crc, rx_data_i);
                    if (payload_index + 1'b1 == packet_length) begin
                        rx_state <= RX_CRC;
                    end else begin
                        payload_index <= payload_index + 1'b1;
                    end
                end

                RX_CRC: begin
                    rx_state <= (rx_data_i == 8'ha5) ? RX_SOF1 : RX_SOF0;
                    if ((rx_data_i == rx_crc) && !response_active && !mask_read_active) begin
                        response_active    <= 1'b1;
                        response_position  <= 4'd0;
                        response_crc       <= 8'd0;
                        response_seq       <= packet_seq;
                        response_command   <= packet_command;
                        response_status    <= STATUS_OK;
                        response_threshold <= threshold_o;
                        response_canny<=packet_command==CMD_CANNY_GET || packet_command==CMD_CANNY_SET;
                        response_canny_low<=canny_low_o;response_canny_high<=canny_high_o;
                        response_extended<=packet_command==CMD_DISPLAY || packet_command==CMD_HISTORY || packet_command==CMD_STATUS;
                        response_background<=packet_command==CMD_BACKGROUND_STATUS || packet_command==CMD_BACKGROUND_RELEARN || packet_command==CMD_BACKGROUND_THRESHOLD;
                        response_mask<=packet_command==CMD_MASK_READ;
                        response_mask_epoch<=background_mask_epoch_i;
                        response_mask_address<=requested_threshold[13:0];
                        response_mask_data<=0;
                        response_background_threshold<=background_threshold_o;
                        response_background_flags<={5'd0,background_overflow_i,background_pending_i,background_valid_i};
                        response_background_count<=background_count_i; response_background_epoch<=background_epoch_i;
                        response_background_frames<=background_frames_i; response_background_cycles<=background_cycles_i;
                        response_display<=display_mode_i;response_history<=history_control_o;
                        response_count<=history_count_i;response_age_a<=history_age_a_o;
                        response_age_b<=history_age_b_o;response_fault<=video_fault_i;

                        if (packet_version != PROTOCOL_VERSION) begin
                            response_status <= STATUS_BAD_VERSION;
                        end else if (packet_type != TYPE_COMMAND) begin
                            response_status <= STATUS_BAD_TYPE;
                        end else begin
                            case (packet_command)
                                CMD_CANNY_GET: if(packet_length!=0) response_status<=STATUS_BAD_LENGTH;
                                CMD_CANNY_SET: begin
                                    if(packet_length!=4) response_status<=STATUS_BAD_LENGTH;
                                    else if({payload[1],payload[0]}>{payload[3],payload[2]} ||
                                            {payload[3],payload[2]}>16'd2047) response_status<=STATUS_RANGE_ERROR;
                                    else if(!canny_ready_i) response_status<=STATUS_BUSY;
                                    else begin
                                        canny_low_o<={payload[1][2:0],payload[0]};
                                        canny_high_o<={payload[3][2:0],payload[2]};canny_valid_o<=1;
                                        response_canny_low<={payload[1][2:0],payload[0]};
                                        response_canny_high<={payload[3][2:0],payload[2]};
                                    end
                                end
                                CMD_MASK_CAPTURE: begin
                                    if(packet_length!=0) response_status<=STATUS_BAD_LENGTH;
                                    else if(display_mode_i!=3'd4 || !background_valid_i || background_pending_i || background_mask_busy_i)
                                        response_status<=STATUS_BUSY;
                                    else background_mask_capture_o<=1;
                                end
                                CMD_MASK_READ: begin
                                    if(packet_length!=2) response_status<=STATUS_BAD_LENGTH;
                                    else if(requested_threshold>=MASK_CELLS) response_status<=STATUS_RANGE_ERROR;
                                    else if(!background_mask_ready_i || background_mask_busy_i) response_status<=STATUS_BUSY;
                                    else begin
                                        response_active<=0;
                                        mask_read_active<=1;mask_read_wait<=1;mask_read_index<=0;
                                        background_mask_address_o<=requested_threshold[13:0];
                                    end
                                end
                                CMD_BACKGROUND_STATUS: begin
                                    if(packet_length!=0) response_status<=STATUS_BAD_LENGTH;
                                end
                                CMD_BACKGROUND_RELEARN: begin
                                    if(packet_length!=0) response_status<=STATUS_BAD_LENGTH;
                                    else if(display_mode_i!=3'd4 || background_pending_i) response_status<=STATUS_BUSY;
                                    else begin
                                        background_relearn_o<=1;
                                        history_control_o<=0;history_age_a_o<=0;history_age_b_o<=0;
                                    end
                                end
                                CMD_BACKGROUND_THRESHOLD: begin
                                    if(packet_length!=1) response_status<=STATUS_BAD_LENGTH;
                                    else begin background_threshold_o<=payload[0];response_background_threshold<=payload[0];end
                                end
                                CMD_DISPLAY: begin
                                    if(packet_length!=8'd1) response_status<=STATUS_BAD_LENGTH;
                                    else if(payload[0]>8'd6) response_status<=STATUS_RANGE_ERROR;
                                    else begin
                                        display_request_o<=payload[0][2:0];display_valid_o<=1;
                                        history_control_o<=0;history_age_a_o<=0;history_age_b_o<=0;
                                        response_history<=0;response_age_a<=0;response_age_b<=0;
                                    end
                                end
                                CMD_HISTORY: begin
                                    if(packet_length!=8'd3) response_status<=STATUS_BAD_LENGTH;
                                    else if(payload[0]>8'd2) response_status<=STATUS_RANGE_ERROR;
                                    else if(payload[0]!=0 && (history_count_i==0 || display_changed_i)) response_status<=STATUS_BUSY;
                                    else if(payload[0]!=0 && (payload[1]>=history_count_i || payload[2]>=history_count_i)) response_status<=STATUS_RANGE_ERROR;
                                    else begin
                                        history_control_o<=payload[0][1:0];
                                        history_age_a_o<=payload[1];history_age_b_o<=payload[2];
                                        response_history<=payload[0][1:0];
                                        response_age_a<=payload[1];response_age_b<=payload[2];
                                    end
                                end
                                CMD_STATUS: if(packet_length!=0) response_status<=STATUS_BAD_LENGTH;
                                CMD_PING,
                                CMD_GET_THRESHOLD: begin
                                    if (packet_length != 8'd0) begin
                                        response_status <= STATUS_BAD_LENGTH;
                                    end
                                end

                                CMD_SET_THRESHOLD: begin
                                    if (packet_length != 8'd2) begin
                                        response_status <= STATUS_BAD_LENGTH;
                                    end else if (requested_threshold > MAX_THRESHOLD) begin
                                        response_status <= STATUS_RANGE_ERROR;
                                    end else if (!threshold_ready_i) begin
                                        response_status <= STATUS_BUSY;
                                    end else begin
                                        threshold_o        <= requested_threshold[9:0];
                                        threshold_valid_o  <= 1'b1;
                                        response_threshold <= requested_threshold[9:0];
                                    end
                                end

                                CMD_STEP_THRESHOLD: begin
                                    if (packet_length != 8'd1) begin
                                        response_status <= STATUS_BAD_LENGTH;
                                    end else if ((payload[0] != 8'h01) &&
                                                 (payload[0] != 8'hff)) begin
                                        response_status <= STATUS_RANGE_ERROR;
                                    end else if (!threshold_ready_i) begin
                                        response_status <= STATUS_BUSY;
                                    end else if (payload[0] == 8'h01) begin
                                        if (threshold_o < MAX_THRESHOLD) begin
                                            threshold_o        <= threshold_o + 1'b1;
                                            response_threshold <= threshold_o + 1'b1;
                                            threshold_valid_o  <= 1'b1;
                                        end
                                    end else if (threshold_o != 10'd0) begin
                                        threshold_o        <= threshold_o - 1'b1;
                                        response_threshold <= threshold_o - 1'b1;
                                        threshold_valid_o  <= 1'b1;
                                    end
                                end

                                CMD_OSD_WRITE: begin
                                    if ((packet_length < 8'd2) ||
                                        (packet_length > 8'd16)) begin
                                        response_status <= STATUS_BAD_LENGTH;
                                    end else if (({1'b0, payload[0]} +
                                                  {1'b0, packet_length} - 9'd1) > 9'd64) begin
                                        response_status <= STATUS_RANGE_ERROR;
                                    end else if (osd_commit_active) begin
                                        response_status <= STATUS_BUSY;
                                    end else begin
                                        osd_commit_active <= 1'b1;
                                        osd_commit_addr   <= payload[0][5:0];
                                        osd_commit_index  <= 5'd1;
                                    end
                                end

                                default: response_status <= STATUS_BAD_COMMAND;
                            endcase
                        end
                    end
                end

                default: rx_state <= RX_SOF0;
            endcase
        end
        // 实体按键和串口切换共享同一显示选择器；任何视图改变均结束历史播放。
        if(display_changed_i) begin
            history_control_o<=0;history_age_a_o<=0;history_age_b_o<=0;
        end
    end
end

endmodule
