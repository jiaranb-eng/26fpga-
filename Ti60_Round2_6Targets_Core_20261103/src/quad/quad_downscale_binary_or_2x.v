`timescale 1ns/1ps

// Downscales a binary edge map by OR-reducing each 2x2 pixel block.
module quad_downscale_binary_or_2x #(
    parameter INPUT_WIDTH  = 1280,
    parameter INPUT_HEIGHT = 720
)(
    input      clk_i,
    input      rst_n_i,
    input      sink_sop_i,
    input      sink_eop_i,
    input      sink_valid_i,
    input      sink_edge_i,
    output reg source_sop_o,
    output reg source_eop_o,
    output reg source_valid_o,
    output reg source_edge_o
);

localparam OUTPUT_WIDTH = INPUT_WIDTH / 2;

reg [10:0] x_count;
reg [9:0]  y_count;
reg        left_edge;
reg        previous_pair_edge;
reg        edge_row_or [0:OUTPUT_WIDTH-1];

always @(posedge clk_i or negedge rst_n_i) begin
    if (!rst_n_i) begin
        x_count            <= 11'd0;
        y_count            <= 10'd0;
        left_edge          <= 1'b0;
        previous_pair_edge <= 1'b0;
        source_sop_o       <= 1'b0;
        source_eop_o       <= 1'b0;
        source_valid_o     <= 1'b0;
        source_edge_o      <= 1'b0;
    end else begin
        source_sop_o   <= 1'b0;
        source_eop_o   <= 1'b0;
        source_valid_o <= 1'b0;

        if (sink_valid_i && sink_sop_i) begin
            x_count   <= 11'd1;
            y_count   <= 10'd0;
            left_edge <= sink_edge_i;
        end else if (sink_valid_i) begin
            if (!x_count[0]) begin
                left_edge <= sink_edge_i;
                if (y_count[0])
                    previous_pair_edge <= edge_row_or[x_count[10:1]];
            end else if (!y_count[0]) begin
                edge_row_or[x_count[10:1]] <= left_edge | sink_edge_i;
            end else begin
                source_sop_o   <= (x_count == 11'd1) && (y_count == 10'd1);
                source_eop_o   <= (x_count == INPUT_WIDTH - 1) &&
                                  (y_count == INPUT_HEIGHT - 1);
                source_valid_o <= 1'b1;
                source_edge_o  <= previous_pair_edge |
                                  left_edge | sink_edge_i;
            end

            if (x_count == INPUT_WIDTH - 1) begin
                x_count <= 11'd0;
                y_count <= (y_count == INPUT_HEIGHT - 1) ?
                           10'd0 : y_count + 1'b1;
            end else begin
                x_count <= x_count + 1'b1;
            end
        end
    end
end

wire unused_sink_eop = sink_eop_i;

endmodule
