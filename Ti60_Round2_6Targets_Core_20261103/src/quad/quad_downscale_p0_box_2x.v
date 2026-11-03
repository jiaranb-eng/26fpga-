`timescale 1ns/1ps

// Downscales an RGB888 frame by two in each dimension with a 2x2 box filter.
// The average is rounded to nearest by adding two before the divide-by-four.
module quad_downscale_p0_box_2x #(
    parameter INPUT_WIDTH  = 1280,
    parameter INPUT_HEIGHT = 720
)(
    input             clk_i,
    input             rst_n_i,
    input             sink_sop_i,
    input             sink_eop_i,
    input             sink_valid_i,
    input      [23:0] sink_rgb_i,
    output reg        source_sop_o,
    output reg        source_eop_o,
    output reg        source_valid_o,
    output reg [23:0] source_rgb_o
);

localparam OUTPUT_WIDTH = INPUT_WIDTH / 2;

reg [10:0] x_count;
reg [9:0]  y_count;
reg [23:0] left_rgb;
reg [8:0]  previous_red_sum;
reg [8:0]  previous_green_sum;
reg [8:0]  previous_blue_sum;

reg [8:0] red_row_sum   [0:OUTPUT_WIDTH-1];
reg [8:0] green_row_sum [0:OUTPUT_WIDTH-1];
reg [8:0] blue_row_sum  [0:OUTPUT_WIDTH-1];

wire [9:0] red_total = {1'b0, previous_red_sum} +
                       {2'b00, left_rgb[23:16]} +
                       {2'b00, sink_rgb_i[23:16]};
wire [9:0] green_total = {1'b0, previous_green_sum} +
                         {2'b00, left_rgb[15:8]} +
                         {2'b00, sink_rgb_i[15:8]};
wire [9:0] blue_total = {1'b0, previous_blue_sum} +
                        {2'b00, left_rgb[7:0]} +
                        {2'b00, sink_rgb_i[7:0]};

wire [9:0] red_rounded = red_total + 10'd2;
wire [9:0] green_rounded = green_total + 10'd2;
wire [9:0] blue_rounded = blue_total + 10'd2;

always @(posedge clk_i or negedge rst_n_i) begin
    if (!rst_n_i) begin
        x_count            <= 11'd0;
        y_count            <= 10'd0;
        left_rgb           <= 24'd0;
        previous_red_sum   <= 9'd0;
        previous_green_sum <= 9'd0;
        previous_blue_sum  <= 9'd0;
        source_sop_o       <= 1'b0;
        source_eop_o       <= 1'b0;
        source_valid_o     <= 1'b0;
        source_rgb_o       <= 24'd0;
    end else begin
        source_sop_o   <= 1'b0;
        source_eop_o   <= 1'b0;
        source_valid_o <= 1'b0;

        if (sink_valid_i && sink_sop_i) begin
            x_count  <= 11'd1;
            y_count  <= 10'd0;
            left_rgb <= sink_rgb_i;
        end else if (sink_valid_i) begin
            if (!x_count[0]) begin
                left_rgb <= sink_rgb_i;
                if (y_count[0]) begin
                    previous_red_sum   <= red_row_sum[x_count[10:1]];
                    previous_green_sum <= green_row_sum[x_count[10:1]];
                    previous_blue_sum  <= blue_row_sum[x_count[10:1]];
                end
            end else if (!y_count[0]) begin
                red_row_sum[x_count[10:1]] <=
                    {1'b0, left_rgb[23:16]} + {1'b0, sink_rgb_i[23:16]};
                green_row_sum[x_count[10:1]] <=
                    {1'b0, left_rgb[15:8]} + {1'b0, sink_rgb_i[15:8]};
                blue_row_sum[x_count[10:1]] <=
                    {1'b0, left_rgb[7:0]} + {1'b0, sink_rgb_i[7:0]};
            end else begin
                source_sop_o   <= (x_count == 11'd1) && (y_count == 10'd1);
                source_eop_o   <= (x_count == INPUT_WIDTH - 1) &&
                                  (y_count == INPUT_HEIGHT - 1);
                source_valid_o <= 1'b1;
                source_rgb_o   <= {red_rounded[9:2],
                                   green_rounded[9:2],
                                   blue_rounded[9:2]};
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
