`timescale 1ns / 1ps
// Seven samples on one axis, four adjacent pixels per accepted clock.
// Vertical storage is six synchronous line memories; borders repeat the nearest pixel.
module canny_axis_window #(
    parameter WIDTH = 1920,
    parameter HEIGHT = 1200,
    parameter BITS = 10,
    parameter VERTICAL = 0
) (
    input wire clk_i,
    input wire rst_n_i,
    input wire valid_i,
    input wire sop_i,
    input wire eop_i,
    input wire [4*BITS-1:0] data_i,
    output wire valid_o,
    output wire sop_o,
    output wire eop_o,
    output wire [28*BITS-1:0] taps_o,
    output wire overlap_error_o
);
    localparam GROUPS = WIDTH / 4;
    localparam ADDR_BITS = GROUPS > 1 ? $clog2(GROUPS) : 1;
    generate if (VERTICAL) begin : vertical_axis
        reg [ADDR_BITS-1:0] column;
        reg [11:0] row;
        reg flushing;
        reg error;
        wire issue = valid_i || flushing;
        wire [ADDR_BITS-1:0] address = valid_i && sop_i ? 0 : column;
        wire [11:0] issue_row = valid_i && sop_i ? 0 : row;
        reg sample_valid;
        reg sample_flush;
        reg [ADDR_BITS-1:0] sample_column;
        reg [11:0] sample_row;
        reg [4*BITS-1:0] bottom_read;
        wire [4*BITS-1:0] line_read [0:5];
        wire [4*BITS-1:0] bottom_pixel = sample_flush ? line_read[0] : bottom_read;
        wire [4*BITS-1:0] first_row = sample_row == 3 ? line_read[2] :
            sample_row == 4 ? line_read[3] : line_read[4];
        reg output_valid;
        reg output_sop;
        reg output_eop;
        reg [28*BITS-1:0] output_taps;
        genvar bank;
        for (bank=0; bank<6; bank=bank+1) begin : line_bank
            (* ram_style = "block" *) reg [4*BITS-1:0] memory [0:GROUPS-1];
            reg [4*BITS-1:0] read_word;
            wire [4*BITS-1:0] write_word;
            if (bank == 0) begin : first_bank
                assign write_word = bottom_pixel;
            end else begin : delayed_bank
                assign write_word = line_read[bank-1];
            end
            always @(posedge clk_i) begin
                if (issue) read_word <= memory[address];
                if (sample_valid) memory[sample_column] <= write_word;
            end
            assign line_read[bank] = read_word;
        end
        genvar position;
        for (position=0; position<7; position=position+1) begin : vertical_tap
            if (position == 6) begin : incoming_row
                always @(posedge clk_i)
                    output_taps[position*4*BITS+:4*BITS] <= bottom_pixel;
            end else begin : stored_row
                always @(posedge clk_i)
                    output_taps[position*4*BITS+:4*BITS] <= sample_row < 6-position ? first_row : line_read[5-position];
            end
        end
        always @(posedge clk_i) begin
            if (issue) bottom_read <= data_i;
        end
        always @(posedge clk_i or negedge rst_n_i) begin
            if (!rst_n_i) begin
                column <= 0;
                row <= 0;
                flushing <= 0;
                error <= 0;
                sample_valid <= 0;
                sample_flush <= 0;
                sample_column <= 0;
                sample_row <= 0;
                output_valid <= 0;
                output_sop <= 0;
                output_eop <= 0;
            end else begin
                sample_valid <= issue;
                sample_flush <= flushing;
                sample_column <= address;
                sample_row <= issue_row;
                output_valid <= sample_valid && sample_row >= 3;
                output_sop <= sample_valid && sample_row == 3 && sample_column == 0;
                output_eop <= sample_valid && sample_row == HEIGHT+2 && sample_column == GROUPS-1;
                if (flushing && valid_i) error <= 1;
                if (issue) begin
                    if (address == GROUPS-1) begin
                        column <= 0;
                        row <= issue_row + 1'b1;
                        if (flushing && issue_row == HEIGHT+2) flushing <= 0;
                    end else begin
                        column <= address + 1'b1;
                        row <= issue_row;
                    end
                    if (valid_i && eop_i) flushing <= 1;
                end
            end
        end
        assign valid_o = output_valid;
        assign sop_o = output_sop;
        assign eop_o = output_eop;
        assign taps_o = output_taps;
        assign overlap_error_o = error;
    end else begin : horizontal_axis
        reg [ADDR_BITS-1:0] column;
        reg [11:0] row;
        wire [ADDR_BITS-1:0] issue_column = valid_i && sop_i ? 0 : column;
        wire [11:0] issue_row = valid_i && sop_i ? 0 : row;
        reg [ADDR_BITS-1:0] previous_column;
        reg [11:0] previous_row;
        reg [4*BITS-1:0] previous_group;
        reg [3*BITS-1:0] left_pixels;
        reg tail_pending;
        wire emit_middle = valid_i && issue_column != 0;
        wire emit = emit_middle || tail_pending;
        wire [3*BITS-1:0] right_pixels = tail_pending ? {3{previous_group[3*BITS+:BITS]}} : data_i[0+:3*BITS];
        wire [10*BITS-1:0] span = {right_pixels,previous_group,left_pixels};
        reg output_valid;
        reg output_sop;
        reg output_eop;
        reg [28*BITS-1:0] output_taps;
        genvar tap;
        genvar lane;
        for (tap=0; tap<7; tap=tap+1) begin : horizontal_tap
            for (lane=0; lane<4; lane=lane+1) begin : parallel_pixel
                always @(posedge clk_i)
                    output_taps[(tap*4+lane)*BITS+:BITS] <= span[(tap+lane)*BITS+:BITS];
            end
        end
        always @(posedge clk_i or negedge rst_n_i) begin
            if (!rst_n_i) begin
                column <= 0;
                row <= 0;
                previous_column <= 0;
                previous_row <= 0;
                previous_group <= 0;
                left_pixels <= 0;
                tail_pending <= 0;
                output_valid <= 0;
                output_sop <= 0;
                output_eop <= 0;
            end else begin
                output_valid <= emit;
                output_sop <= emit && previous_column == 0 && previous_row == 0;
                output_eop <= emit && previous_column == GROUPS-1 && previous_row == HEIGHT-1;
                tail_pending <= valid_i && issue_column == GROUPS-1;
                if (valid_i) begin
                    previous_group <= data_i;
                    previous_column <= issue_column;
                    previous_row <= issue_row;
                    left_pixels <= issue_column == 0 ? {3{data_i[0+:BITS]}} : previous_group[BITS+:3*BITS];
                    if (issue_column == GROUPS-1) begin
                        column <= 0;
                        row <= issue_row + 1'b1;
                    end else begin
                        column <= issue_column + 1'b1;
                        row <= issue_row;
                    end
                end
            end
        end
        assign valid_o = output_valid;
        assign sop_o = output_sop;
        assign eop_o = output_eop;
        assign taps_o = output_taps;
        assign overlap_error_o = 1'b0;
    end endgenerate
endmodule
