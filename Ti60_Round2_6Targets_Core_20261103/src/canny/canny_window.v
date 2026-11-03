`timescale 1ns/1ps

// Four adjacent pixels per clock, with centered 3x3 windows.
// Two rotating lines; writes retire one clock after reads to avoid collisions.
// No memory reset is needed: row tags mask stale contents after every SOP.
module canny_window #(
    parameter WIDTH = 1920,
    parameter HEIGHT = 1200,
    parameter BITS = 8,
    parameter REPLICATE = 1
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
    output wire [4*BITS-1:0] northwest_o,
    output wire [4*BITS-1:0] north_o,
    output wire [4*BITS-1:0] northeast_o,
    output wire [4*BITS-1:0] west_o,
    output wire [4*BITS-1:0] center_o,
    output wire [4*BITS-1:0] east_o,
    output wire [4*BITS-1:0] southwest_o,
    output wire [4*BITS-1:0] south_o,
    output wire [4*BITS-1:0] southeast_o,
    output wire overlap_error_o
);
    localparam GROUPS = WIDTH / 4;
    localparam ADDR_BITS = $clog2(GROUPS);
    (* ram_style = "block" *) reg [4*BITS-1:0] first_line [0:GROUPS-1];
    (* ram_style = "block" *) reg [4*BITS-1:0] second_line [0:GROUPS-1];
    reg bank_phase;
    reg [ADDR_BITS-1:0] column;
    reg [11:0] row;
    reg flushing;
    reg overlap_error;
    wire issue = valid_i || flushing;
    wire [ADDR_BITS-1:0] address = sop_i && valid_i ? {ADDR_BITS{1'b0}} : column;
    wire [11:0] issue_row = sop_i && valid_i ? 12'd0 : row;
    reg sample_valid;
    reg [ADDR_BITS-1:0] sample_column;
    reg [11:0] sample_row;
    reg sample_flush;
    reg sample_phase;
    wire issue_phase = sop_i && valid_i ? 1'b0 : bank_phase;
    reg [4*BITS-1:0] first_read, second_read;
    wire [4*BITS-1:0] top_read = sample_phase ? second_read : first_read;
    wire [4*BITS-1:0] middle_read = sample_phase ? first_read : second_read;
    reg [4*BITS-1:0] bottom_read;

    reg write_pending; // 写提交独立于CE停顿，仅延后一个物理时钟。
    reg write_bank; // 保存与待写像素对应的物理行。
    reg [ADDR_BITS-1:0] write_address; // 待写列地址，避免覆盖同拍读取的旧行。
    reg [4*BITS-1:0] write_pixels; // 一组像素，不增加行缓存。
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) write_pending <= 1'b0;
        else write_pending <= issue && !flushing;
    end
    always @(posedge clk_i) begin
        if (issue && !flushing) begin
            write_bank <= issue_phase;
            write_address <= address;
            write_pixels <= data_i;
        end
        if (write_pending && !write_bank) first_line[write_address] <= write_pixels;
        if (write_pending && write_bank) second_line[write_address] <= write_pixels;
    end


    always @(posedge clk_i) begin
        if (issue) begin
            first_read <= first_line[address];
            second_read <= second_line[address];
            bottom_read <= data_i;
        end
    end
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            column <= 0;
            row <= 0;
            bank_phase <= 0;
            sample_phase <= 0;
            flushing <= 0;
            sample_valid <= 0;
            sample_column <= 0;
            sample_row <= 0;
            sample_flush <= 0;
            overlap_error <= 0;
        end else begin
            sample_valid <= issue;
            sample_column <= address;
            sample_row <= issue_row;
            sample_flush <= flushing;
            sample_phase <= issue_phase;
            if (flushing && valid_i)
                overlap_error <= 1;
            if (issue) begin
                if (address == GROUPS-1) begin
                    column <= 0;
                    row <= issue_row + 1'b1;
                    bank_phase <= !issue_phase;
                    if (flushing)
                        flushing <= 0;
                end else begin
                    column <= address + 1'b1;
                    row <= issue_row;
                    bank_phase <= issue_phase;
                end
                if (valid_i && eop_i)
                    flushing <= 1;
            end
        end
    end

    wire [4*BITS-1:0] selected_top = sample_row == 1 ?
        (REPLICATE ? middle_read : {4*BITS{1'b0}}) : top_read;
    wire [4*BITS-1:0] selected_bottom = sample_flush ?
        (REPLICATE ? middle_read : {4*BITS{1'b0}}) : bottom_read;
    reg [4*BITS-1:0] previous_top;
    reg [4*BITS-1:0] previous_middle;
    reg [4*BITS-1:0] previous_bottom;
    reg [BITS-1:0] left_top;
    reg [BITS-1:0] left_middle;
    reg [BITS-1:0] left_bottom;
    reg [ADDR_BITS-1:0] previous_column;
    reg [11:0] previous_row;
    reg tail_pending;
    wire emit_middle = sample_valid && sample_row != 0 && sample_column != 0;
    wire emit_window = emit_middle || tail_pending;
    wire [BITS-1:0] right_top = tail_pending ?
        (REPLICATE ? previous_top[4*BITS-1:3*BITS] : {BITS{1'b0}}) : selected_top[BITS-1:0];
    wire [BITS-1:0] right_middle = tail_pending ?
        (REPLICATE ? previous_middle[4*BITS-1:3*BITS] : {BITS{1'b0}}) : middle_read[BITS-1:0];
    wire [BITS-1:0] right_bottom = tail_pending ?
        (REPLICATE ? previous_bottom[4*BITS-1:3*BITS] : {BITS{1'b0}}) : selected_bottom[BITS-1:0];
    reg output_valid;
    reg output_sop;
    reg output_eop;
    reg [4*BITS-1:0] northwest, north, northeast;
    reg [4*BITS-1:0] west, center, east;
    reg [4*BITS-1:0] southwest, south, southeast;
    genvar lane;
    generate for (lane=0; lane<4; lane=lane+1) begin : pixel_window
        always @(posedge clk_i) begin
            north[lane*BITS+:BITS] <= previous_top[lane*BITS+:BITS];
            center[lane*BITS+:BITS] <= previous_middle[lane*BITS+:BITS];
            south[lane*BITS+:BITS] <= previous_bottom[lane*BITS+:BITS];
        end
        if (lane == 0) begin : left_boundary
            always @(posedge clk_i) begin
                northwest[lane*BITS+:BITS] <= left_top;
                west[lane*BITS+:BITS] <= left_middle;
                southwest[lane*BITS+:BITS] <= left_bottom;
            end
        end else begin : left_interior
            always @(posedge clk_i) begin
                northwest[lane*BITS+:BITS] <= previous_top[(lane-1)*BITS+:BITS];
                west[lane*BITS+:BITS] <= previous_middle[(lane-1)*BITS+:BITS];
                southwest[lane*BITS+:BITS] <= previous_bottom[(lane-1)*BITS+:BITS];
            end
        end
        if (lane == 3) begin : right_boundary
            always @(posedge clk_i) begin
                northeast[lane*BITS+:BITS] <= right_top;
                east[lane*BITS+:BITS] <= right_middle;
                southeast[lane*BITS+:BITS] <= right_bottom;
            end
        end else begin : right_interior
            always @(posedge clk_i) begin
                northeast[lane*BITS+:BITS] <= previous_top[(lane+1)*BITS+:BITS];
                east[lane*BITS+:BITS] <= previous_middle[(lane+1)*BITS+:BITS];
                southeast[lane*BITS+:BITS] <= previous_bottom[(lane+1)*BITS+:BITS];
            end
        end
    end endgenerate
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            previous_top <= 0;
            previous_middle <= 0;
            previous_bottom <= 0;
            left_top <= 0;
            left_middle <= 0;
            left_bottom <= 0;
            previous_column <= 0;
            previous_row <= 0;
            tail_pending <= 0;
            output_valid <= 0;
            output_sop <= 0;
            output_eop <= 0;
        end else begin
            tail_pending <= sample_valid && sample_row != 0 && sample_column == GROUPS-1;
            output_valid <= emit_window;
            output_sop <= emit_window && previous_column == 0 && previous_row == 1;
            output_eop <= emit_window && previous_column == GROUPS-1 && previous_row == HEIGHT;
            if (sample_valid) begin
                previous_top <= selected_top;
                previous_middle <= middle_read;
                previous_bottom <= selected_bottom;
                previous_column <= sample_column;
                previous_row <= sample_row;
                if (sample_column == 0) begin
                    left_top <= REPLICATE ? selected_top[BITS-1:0] : {BITS{1'b0}};
                    left_middle <= REPLICATE ? middle_read[BITS-1:0] : {BITS{1'b0}};
                    left_bottom <= REPLICATE ? selected_bottom[BITS-1:0] : {BITS{1'b0}};
                end else begin
                    left_top <= previous_top[4*BITS-1:3*BITS];
                    left_middle <= previous_middle[4*BITS-1:3*BITS];
                    left_bottom <= previous_bottom[4*BITS-1:3*BITS];
                end
            end
        end
    end
    assign valid_o = output_valid;
    assign sop_o = output_sop;
    assign eop_o = output_eop;
    assign northwest_o = northwest;
    assign north_o = north;
    assign northeast_o = northeast;
    assign west_o = west;
    assign center_o = center;
    assign east_o = east;
    assign southwest_o = southwest;
    assign south_o = south;
    assign southeast_o = southeast;
    assign overlap_error_o = overlap_error;
endmodule
