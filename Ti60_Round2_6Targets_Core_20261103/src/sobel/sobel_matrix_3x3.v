`timescale 1ns/1ps

// 3x3 streaming window for an 8-bit image.  A window is valid only after
// two complete rows and two pixels of the current row have been accepted.
module sobel_matrix_3x3 #(
    parameter WIDTH = 1280,
    parameter HEIGHT = 720
)(
    input              clk,
    input              rst_n,
    input              sink_sop,
    input              sink_eop,
    input              sink_valid,
    input      [7:0]   sink_data,
    output             matrix_sop,
    output             matrix_eop,
    output             matrix_valid,
    output     [7:0]   matrix_p11,
    output     [7:0]   matrix_p12,
    output     [7:0]   matrix_p13,
    output     [7:0]   matrix_p21,
    output     [7:0]   matrix_p22,
    output     [7:0]   matrix_p23,
    output     [7:0]   matrix_p31,
    output     [7:0]   matrix_p32,
    output     [7:0]   matrix_p33
);

localparam integer COUNT_WIDTH = 12;

wire [7:0] row1_data;
wire [7:0] row2_data;

// The shift RAM is intentionally not reset.  matrix_valid is suppressed
// until both previous rows are filled, so RAM power-up contents stay hidden.
sobel_linebuffer #(
    .DISTANCE(WIDTH),
    .DWIDTH(8),
    .TAPS_NUM(2)
) u_linebuffers (
    .clken    (sink_valid),
    .clock    (clk),
    .reset_n  (rst_n),
    .frame_start(sink_sop),
    .shiftin  (sink_data),
    .shiftout (),
    .taps     ({row1_data, row2_data})
);

reg [7:0] row1_d1;
reg [7:0] row1_d2;
reg [7:0] row2_d1;
reg [7:0] row2_d2;
reg [7:0] row3_d1;
reg [7:0] row3_d2;
reg [7:0] current_data;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        row1_d1 <= 8'd0;
        row1_d2 <= 8'd0;
        row2_d1 <= 8'd0;
        row2_d2 <= 8'd0;
        row3_d1 <= 8'd0;
        row3_d2 <= 8'd0;
        current_data <= 8'd0;
    end else if (sink_valid) begin
        row1_d2 <= row1_d1;
        row1_d1 <= row1_data;
        row2_d2 <= row2_d1;
        row2_d1 <= row2_data;
        row3_d2 <= row3_d1;
        row3_d1 <= current_data;
        current_data <= sink_data;
    end
end

// At the accepting edge for coordinate (x,y), this window covers rows
// y-2..y and columns x-2..x.  The arithmetic samples it on the same edge.
assign matrix_p11 = row1_d2;
assign matrix_p12 = row1_d1;
assign matrix_p13 = row1_data;
assign matrix_p21 = row2_d2;
assign matrix_p22 = row2_d1;
assign matrix_p23 = row2_data;
assign matrix_p31 = row3_d2;
assign matrix_p32 = row3_d1;
assign matrix_p33 = current_data;

reg [COUNT_WIDTH-1:0] xcnt;
reg [COUNT_WIDTH-1:0] ycnt;
reg                   matrix_sop_r;
reg                   matrix_eop_r;
reg                   matrix_valid_r;

// xcnt and ycnt describe the accepted pixel currently on sink_data.
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        xcnt <= {COUNT_WIDTH{1'b0}};
        ycnt <= {COUNT_WIDTH{1'b0}};
        matrix_sop_r   <= 1'b0;
        matrix_eop_r   <= 1'b0;
        matrix_valid_r <= 1'b0;
    end else begin
        matrix_valid_r <= sink_valid &&
                          (sink_sop ? 1'b0 : ((xcnt >= 2) && (ycnt >= 2)));
        matrix_sop_r   <= sink_valid && !sink_sop &&
                          (xcnt == 2) && (ycnt == 2);
        matrix_eop_r   <= sink_valid && !sink_sop &&
                          (xcnt == WIDTH - 1) && (ycnt == HEIGHT - 1);

        if (sink_valid && sink_sop) begin
            if (WIDTH > 1)
                xcnt <= {{(COUNT_WIDTH-1){1'b0}}, 1'b1};
            else
                xcnt <= {COUNT_WIDTH{1'b0}};
            ycnt <= {COUNT_WIDTH{1'b0}};
        end else if (sink_valid && (xcnt >= WIDTH - 1)) begin
            xcnt <= {COUNT_WIDTH{1'b0}};
            if (ycnt >= HEIGHT - 1)
                ycnt <= {COUNT_WIDTH{1'b0}};
            else
                ycnt <= ycnt + 1'b1;
        end else if (sink_valid) begin
            xcnt <= xcnt + 1'b1;
        end
    end
end

// The block-RAM outputs and these controls describe the same accepted pixel.
assign matrix_valid = matrix_valid_r;
assign matrix_sop   = matrix_sop_r;
assign matrix_eop   = matrix_eop_r;

wire unused_sink_eop = sink_eop;

endmodule
