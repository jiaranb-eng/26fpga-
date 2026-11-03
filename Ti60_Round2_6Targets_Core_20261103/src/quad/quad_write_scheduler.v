`timescale 1ns / 1ps

// 以 16 个 128-bit beat 为服务量，严格执行 P0、P1、P2、P3 轮询。
// 地址直接形成最终 1280x720 RGB565 raster，不在读侧再次拼接。
module quad_write_scheduler #(
    parameter [8:0] OUTPUT_HEIGHT = 9'd360,
    parameter [2:0] CHUNKS_PER_LINE = 3'd5,
    parameter [7:0] BURST_BEATS = 8'd16,
    parameter [31:0] LINE_BYTES = 32'd2560,
    parameter [31:0] HALF_LINE_BYTES = 32'd1280,
    parameter [31:0] BOTTOM_OFFSET = 32'd921600
) (
    input  wire         clk_i,
    input  wire         reset_i,
    input  wire         start_i,
    input  wire         abort_i,
    input  wire [31:0]  frame_base_i,
    input  wire [9:0]   fifo_count_0_i,
    input  wire [9:0]   fifo_count_1_i,
    input  wire [9:0]   fifo_count_2_i,
    input  wire [9:0]   fifo_count_3_i,
    input  wire [127:0] fifo_data_0_i,
    input  wire [127:0] fifo_data_1_i,
    input  wire [127:0] fifo_data_2_i,
    input  wire [127:0] fifo_data_3_i,
    output wire         fifo_read_0_o,
    output wire         fifo_read_1_o,
    output wire         fifo_read_2_o,
    output wire         fifo_read_3_o,
    output reg  [31:0]  axi_awaddr_o,
    output wire [7:0]   axi_awlen_o,
    output wire         axi_awvalid_o,
    input  wire         axi_awready_i,
    output wire [127:0] axi_wdata_o,
    output wire [15:0]  axi_wstrb_o,
    output wire         axi_wlast_o,
    output wire         axi_wvalid_o,
    input  wire         axi_wready_i,
    input  wire [1:0]   axi_bresp_i,
    input  wire         axi_bvalid_i,
    output wire         axi_bready_o,
    output reg          busy_o,
    output reg          frame_done_o,
    output reg          frame_error_o,
    output reg  [1:0]   active_route_o,
    output reg  [2:0]   active_chunk_o,
    output reg  [8:0]   active_line_o
);

localparam [2:0] STATE_IDLE = 3'd0;
localparam [2:0] STATE_WAIT = 3'd1;
localparam [2:0] STATE_ADDRESS = 3'd2;
localparam [2:0] STATE_DATA = 3'd3;
localparam [2:0] STATE_RESPONSE = 3'd4;

reg [2:0] state;
reg [7:0] beat_count;
reg [31:0] top_line_base;
reg [31:0] bottom_line_base;
reg abort_pending;

wire all_routes_ready =
    (fifo_count_0_i >= BURST_BEATS) &&
    (fifo_count_1_i >= BURST_BEATS) &&
    (fifo_count_2_i >= BURST_BEATS) &&
    (fifo_count_3_i >= BURST_BEATS);
wire write_handshake = axi_wvalid_o && axi_wready_i;

reg [127:0] selected_fifo_data;
always @(*) begin
    case (active_route_o)
        2'd0: selected_fifo_data = fifo_data_0_i;
        2'd1: selected_fifo_data = fifo_data_1_i;
        2'd2: selected_fifo_data = fifo_data_2_i;
        default: selected_fifo_data = fifo_data_3_i;
    endcase
end

assign axi_awlen_o = BURST_BEATS - 1'b1;
assign axi_awvalid_o = (state == STATE_ADDRESS);
assign axi_wdata_o = selected_fifo_data;
assign axi_wstrb_o = 16'hffff;
assign axi_wlast_o = (beat_count == BURST_BEATS - 1'b1);
assign axi_wvalid_o = (state == STATE_DATA);
assign axi_bready_o = (state == STATE_RESPONSE);

assign fifo_read_0_o = write_handshake && (active_route_o == 2'd0);
assign fifo_read_1_o = write_handshake && (active_route_o == 2'd1);
assign fifo_read_2_o = write_handshake && (active_route_o == 2'd2);
assign fifo_read_3_o = write_handshake && (active_route_o == 2'd3);

always @(posedge clk_i or posedge reset_i) begin
    if (reset_i) begin
        state          <= STATE_IDLE;
        beat_count     <= 8'd0;
        abort_pending  <= 1'b0;
        top_line_base  <= 32'd0;
        bottom_line_base <= 32'd0;
        axi_awaddr_o   <= 32'd0;
        busy_o         <= 1'b0;
        frame_done_o   <= 1'b0;
        frame_error_o  <= 1'b0;
        active_route_o <= 2'd0;
        active_chunk_o <= 3'd0;
        active_line_o  <= 9'd0;
    end else begin
        frame_done_o  <= 1'b0;
        frame_error_o <= 1'b0;

        if (abort_i && busy_o)
            abort_pending <= 1'b1;

        // WAIT 状态不存在已接受但尚未完成的 AXI 事务，可安全终止。
        if ((abort_i || abort_pending) && busy_o &&
            (state == STATE_WAIT)) begin
            state         <= STATE_IDLE;
            busy_o        <= 1'b0;
            frame_done_o  <= 1'b1;
            frame_error_o <= 1'b1;
            abort_pending <= 1'b0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    if (start_i) begin
                        top_line_base    <= frame_base_i;
                        bottom_line_base <= frame_base_i + BOTTOM_OFFSET;
                        active_route_o   <= 2'd0;
                        active_chunk_o   <= 3'd0;
                        active_line_o    <= 9'd0;
                        beat_count       <= 8'd0;
                        abort_pending    <= 1'b0;
                        busy_o           <= 1'b1;
                        state            <= STATE_WAIT;
                    end
                end

                STATE_WAIT: begin
                    // 只在一轮的 P0 起点预留四路配额；P1~P3 必须完成本轮。
                    if ((active_route_o != 2'd0) || all_routes_ready) begin
                        case (active_route_o)
                            2'd0: axi_awaddr_o <= top_line_base +
                                                   {active_chunk_o, 8'd0};
                            2'd1: axi_awaddr_o <= top_line_base +
                                                   HALF_LINE_BYTES +
                                                   {active_chunk_o, 8'd0};
                            2'd2: axi_awaddr_o <= bottom_line_base +
                                                   {active_chunk_o, 8'd0};
                            default: axi_awaddr_o <= bottom_line_base +
                                                      HALF_LINE_BYTES +
                                                      {active_chunk_o, 8'd0};
                        endcase
                        state <= STATE_ADDRESS;
                    end
                end

                STATE_ADDRESS: begin
                    if (axi_awready_i) begin
                        beat_count <= 8'd0;
                        state <= STATE_DATA;
                    end
                end

                STATE_DATA: begin
                    if (write_handshake) begin
                        if (axi_wlast_o)
                            state <= STATE_RESPONSE;
                        else
                            beat_count <= beat_count + 1'b1;
                    end
                end

                STATE_RESPONSE: begin
                    if (axi_bvalid_i) begin
                        if ((axi_bresp_i != 2'b00) || abort_pending || abort_i) begin
                            busy_o        <= 1'b0;
                            frame_done_o  <= 1'b1;
                            frame_error_o <= 1'b1;
                            abort_pending <= 1'b0;
                            state         <= STATE_IDLE;
                        end else if ((active_route_o == 2'd3) &&
                                     (active_chunk_o == CHUNKS_PER_LINE - 1'b1) &&
                                     (active_line_o == OUTPUT_HEIGHT - 1'b1)) begin
                            busy_o       <= 1'b0;
                            frame_done_o <= 1'b1;
                            abort_pending <= 1'b0;
                            state        <= STATE_IDLE;
                        end else if (active_route_o != 2'd3) begin
                            active_route_o <= active_route_o + 1'b1;
                            state <= STATE_WAIT;
                        end else begin
                            active_route_o <= 2'd0;
                            if (active_chunk_o == CHUNKS_PER_LINE - 1'b1) begin
                                active_chunk_o <= 3'd0;
                                active_line_o <= active_line_o + 1'b1;
                                top_line_base <= top_line_base + LINE_BYTES;
                                bottom_line_base <= bottom_line_base + LINE_BYTES;
                            end else begin
                                active_chunk_o <= active_chunk_o + 1'b1;
                            end
                            state <= STATE_WAIT;
                        end
                    end
                end

                default: begin
                    state  <= STATE_IDLE;
                    busy_o <= 1'b0;
                end
            endcase
        end
    end
end

endmodule
