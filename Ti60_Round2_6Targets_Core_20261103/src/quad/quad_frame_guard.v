`timescale 1ns / 1ps

// 在缩放像素时钟域核对四路帧长度、标记对齐和 FIFO 溢出状态。
// frame_good_o 在 frame_done_toggle_o 翻转前锁存，并保持到下一次帧结束。
module quad_frame_guard #(
    parameter [17:0] EXPECTED_PIXELS = 18'd230400
) (
    input  wire clk_i,
    input  wire rst_n_i,
    input  wire sink_sop_i,
    input  wire sink_eop_i,
    input  wire sink_valid_0_i,
    input  wire sink_valid_1_i,
    input  wire sink_valid_2_i,
    input  wire sink_valid_3_i,
    input  wire fifo_overflow_0_i,
    input  wire fifo_overflow_1_i,
    input  wire fifo_overflow_2_i,
    input  wire fifo_overflow_3_i,
    output reg  frame_done_toggle_o,
    output reg  frame_good_o,
    output reg  frame_active_o,
    output reg  [31:0] good_frame_count_o,
    output reg  [31:0] bad_frame_count_o
);

reg [17:0] pixel_count_0;
reg [17:0] pixel_count_1;
reg [17:0] pixel_count_2;
reg [17:0] pixel_count_3;
reg        frame_bad;

wire all_valid = sink_valid_0_i && sink_valid_1_i &&
                 sink_valid_2_i && sink_valid_3_i;
wire any_valid = sink_valid_0_i || sink_valid_1_i ||
                 sink_valid_2_i || sink_valid_3_i;
wire any_overflow = fifo_overflow_0_i || fifo_overflow_1_i ||
                    fifo_overflow_2_i || fifo_overflow_3_i;
wire final_counts_match =
    (pixel_count_0 == EXPECTED_PIXELS - 1'b1) &&
    (pixel_count_1 == EXPECTED_PIXELS - 1'b1) &&
    (pixel_count_2 == EXPECTED_PIXELS - 1'b1) &&
    (pixel_count_3 == EXPECTED_PIXELS - 1'b1);

always @(posedge clk_i or negedge rst_n_i) begin
    if (!rst_n_i) begin
        pixel_count_0      <= 18'd0;
        pixel_count_1      <= 18'd0;
        pixel_count_2      <= 18'd0;
        pixel_count_3      <= 18'd0;
        frame_bad          <= 1'b0;
        frame_done_toggle_o <= 1'b0;
        frame_good_o       <= 1'b0;
        frame_active_o     <= 1'b0;
        good_frame_count_o <= 32'd0;
        bad_frame_count_o  <= 32'd0;
    end else begin
        if (frame_active_o && any_overflow)
            frame_bad <= 1'b1;

        if (sink_sop_i && any_valid) begin
            // 新 SOP 总是重建计数相位；活动帧中出现 SOP 会使前一帧失效。
            pixel_count_0  <= sink_valid_0_i ? 18'd1 : 18'd0;
            pixel_count_1  <= sink_valid_1_i ? 18'd1 : 18'd0;
            pixel_count_2  <= sink_valid_2_i ? 18'd1 : 18'd0;
            pixel_count_3  <= sink_valid_3_i ? 18'd1 : 18'd0;
            frame_bad      <= frame_active_o || !all_valid || any_overflow;
            frame_active_o <= 1'b1;
        end else if (frame_active_o && any_valid) begin
            if (sink_valid_0_i)
                pixel_count_0 <= pixel_count_0 + 1'b1;
            if (sink_valid_1_i)
                pixel_count_1 <= pixel_count_1 + 1'b1;
            if (sink_valid_2_i)
                pixel_count_2 <= pixel_count_2 + 1'b1;
            if (sink_valid_3_i)
                pixel_count_3 <= pixel_count_3 + 1'b1;

            if (!all_valid)
                frame_bad <= 1'b1;

            if (sink_eop_i) begin
                frame_good_o <= all_valid && final_counts_match &&
                                !frame_bad && !any_overflow;
                frame_done_toggle_o <= !frame_done_toggle_o;
                frame_active_o <= 1'b0;
                if (all_valid && final_counts_match &&
                    !frame_bad && !any_overflow)
                    good_frame_count_o <= good_frame_count_o + 1'b1;
                else
                    bad_frame_count_o <= bad_frame_count_o + 1'b1;
            end
        end
    end
end

endmodule
