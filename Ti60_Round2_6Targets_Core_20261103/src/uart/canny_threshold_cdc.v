`timescale 1ns / 1ps

// Bundled-data request/acknowledge CDC for the 22-bit paired Canny thresholds.
// The source payload remains stable until the destination acknowledges it.
module canny_threshold_cdc #(
    parameter [21:0] DEFAULT_THRESHOLD = {11'd20,11'd10}
) (
    input  wire       src_clk,
    input  wire       src_rst_n,
    input  wire [21:0] src_threshold_i,
    input  wire       src_valid_i,
    output wire       src_ready_o,
    input  wire       dst_clk,
    input  wire       dst_rst_n,
    output reg  [21:0] dst_threshold_o
);

reg [21:0] source_payload;
reg       request_toggle;
reg       acknowledge_toggle;
(* async_reg = "true" *) reg [1:0] acknowledge_sync;
(* async_reg = "true" *) reg [2:0] request_sync;
reg [21:0] payload_sync_1;
reg [21:0] payload_sync_2;

assign src_ready_o = (acknowledge_sync[1] == request_toggle);

always @(posedge src_clk or negedge src_rst_n) begin
    if (!src_rst_n) begin
        source_payload     <= DEFAULT_THRESHOLD;
        request_toggle     <= 1'b0;
        acknowledge_sync   <= 2'b00;
    end else begin
        acknowledge_sync <= {acknowledge_sync[0], acknowledge_toggle};
        if (src_valid_i && src_ready_o) begin
            source_payload <= src_threshold_i;
            request_toggle <= !request_toggle;
        end
    end
end

always @(posedge dst_clk or negedge dst_rst_n) begin
    if (!dst_rst_n) begin
        request_sync       <= 3'b000;
        payload_sync_1     <= DEFAULT_THRESHOLD;
        payload_sync_2     <= DEFAULT_THRESHOLD;
        acknowledge_toggle <= 1'b0;
        dst_threshold_o    <= DEFAULT_THRESHOLD;
    end else begin
        request_sync   <= {request_sync[1:0], request_toggle};
        payload_sync_1 <= source_payload;
        payload_sync_2 <= payload_sync_1;
        if (request_sync[2] != acknowledge_toggle) begin
            dst_threshold_o    <= payload_sync_2;
            acknowledge_toggle <= request_sync[2];
        end
    end
end

endmodule
