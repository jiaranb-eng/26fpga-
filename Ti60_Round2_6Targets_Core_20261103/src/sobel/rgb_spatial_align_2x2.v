`timescale 1ns/1ps

// Produces the RGB pixel located two rows and two columns before sink_data.
// This compensates the two causal 3x3 windows used by median and Sobel.
module rgb_spatial_align_2x2 #(
    parameter WIDTH = 1280,
    parameter HEIGHT = 720
)(
    input              clk,
    input              rst_n,
    input              sink_sop,
    input              sink_valid,
    input      [23:0]  sink_data,
    output     [23:0]  source_data,
    output             source_window_valid
);

localparam integer COUNT_WIDTH = 12;

wire [23:0] row1_data;
wire [23:0] row2_data;
reg  [23:0] row1_d1;
reg  [23:0] row1_d2;
reg  [COUNT_WIDTH-1:0] xcnt;
reg  [COUNT_WIDTH-1:0] ycnt;
reg                    source_window_valid_r;

// The storage is intentionally not reset; validity masks its power-up data.
sobel_linebuffer #(
    .DISTANCE(WIDTH),
    .DWIDTH(24),
    .TAPS_NUM(2)
) u_rgb_linebuffers (
    .clken    (sink_valid),
    .clock    (clk),
    .reset_n  (rst_n),
    .frame_start(sink_sop),
    .shiftin  (sink_data),
    .shiftout (),
    .taps     ({row1_data, row2_data})
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        row1_d1 <= 24'd0;
        row1_d2 <= 24'd0;
    end else if (sink_valid) begin
        row1_d2 <= row1_d1;
        row1_d1 <= row1_data;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        xcnt <= {COUNT_WIDTH{1'b0}};
        ycnt <= {COUNT_WIDTH{1'b0}};
        source_window_valid_r <= 1'b0;
    end else begin
        source_window_valid_r <= sink_valid && !sink_sop &&
                                 (xcnt >= 2) && (ycnt >= 2);
        if (sink_valid && sink_sop) begin
            xcnt <= (WIDTH > 1) ? {{(COUNT_WIDTH-1){1'b0}}, 1'b1}
                                : {COUNT_WIDTH{1'b0}};
            ycnt <= {COUNT_WIDTH{1'b0}};
        end else if (sink_valid && (xcnt >= WIDTH - 1)) begin
            xcnt <= {COUNT_WIDTH{1'b0}};
            ycnt <= (ycnt >= HEIGHT - 1) ? {COUNT_WIDTH{1'b0}} : ycnt + 1'b1;
        end else if (sink_valid) begin
            xcnt <= xcnt + 1'b1;
        end
    end
end

assign source_data = row1_d2;
assign source_window_valid = source_window_valid_r;

wire [23:0] unused_row2_data = row2_data;

endmodule
