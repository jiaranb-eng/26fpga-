`timescale 1ns/1ps

// HDMI 前的末级文字叠加：透明背景上绘制 32x32 像素字符。
// Final pre-HDMI text overlay: draw 32x32-pixel glyphs over transparent video.
//
// The character-RAM/font-ROM pipeline is adapted from the Apache-2.0
// VGA-CharGen architecture.  Both memories use synchronous reads for Efinix
// embedded-RAM inference and deterministic pixel/control alignment.
module osd_text_overlay #(
    parameter integer H_ACTIVE = 1280,
    parameter integer V_ACTIVE = 720,
    parameter integer OSD_X     = 32,
    parameter integer OSD_Y     = 32,
    parameter [23:0]  TEXT_COLOR = 24'hFFFF00,
    parameter integer MODE_LABEL_ENABLE = 0,
    parameter integer SHARED_FONT = 0
) (
    input  wire        pixel_clk_i,
    input wire pixel_enable_i,
    input  wire        pixel_rst_n_i,
    input  wire [2:0]  mode_i,
    input  wire        video_vs_i,
    input  wire        video_hs_i,
    input  wire        video_de_i,
    input  wire [23:0] video_rgb_i,
    input  wire        text_wr_clk_i,
    input  wire        text_wr_en_i,
    input  wire [5:0]  text_wr_addr_i,
    input  wire [7:0]  text_wr_data_i,
    output reg         video_vs_o,
    output reg         video_hs_o,
    output reg         video_de_o,
    output reg  [23:0] video_rgb_o,
    output wire [10:0] font_address_o,
    output wire font_enable_o,
    input wire [15:0] font_pixels_i
);

localparam [10:0] H_LAST       = H_ACTIVE - 1;
localparam [10:0]  V_LAST       = V_ACTIVE - 1;
localparam [10:0] OSD_X_FIRST  = OSD_X;
localparam [10:0] OSD_X_LIMIT  = OSD_X + 1024;
localparam [10:0]  OSD_Y_FIRST  = OSD_Y;
localparam [10:0]  OSD_Y_LIMIT  = OSD_Y + 64;

reg  [10:0] pixel_x;
reg  [10:0]  pixel_y;
reg         video_vs_d;
reg         video_de_d;

wire        frame_start;
wire        osd_window_active;
wire [10:0] osd_local_x;
wire [10:0]  osd_local_y;
wire [5:0]  text_rd_addr;
wire [7:0]  text_rd_data;
wire [15:0] font_row_bits;
wire [7:0]  font_glyph_id;
reg  [7:0]  mode_glyph_stage_one;
reg         mode_label_stage_one;

// Reuse existing ASCII glyphs. The first row identifies the displayed mode;
// the second row retains the character RAM / UART text interface.
function [7:0] mode_label_char;
    input [2:0] mode;
    input [4:0] column;
    begin
        mode_label_char = 8'h20;
        case (mode)
            2'd0: case (column)
                5'd0: mode_label_char = "D";
                5'd1: mode_label_char = "V";
                5'd2: mode_label_char = "P";
                5'd4: mode_label_char = "2";
                5'd5: mode_label_char = "X";
                5'd6: mode_label_char = "2";
                default: mode_label_char = 8'h20;
            endcase
            2'd1: case (column)
                5'd0: mode_label_char = "C";
                5'd1: mode_label_char = "A";
                5'd2: mode_label_char = "N";
                5'd3: mode_label_char = "N";
                5'd4: mode_label_char = "Y";
                default: mode_label_char = 8'h20;
            endcase
            2'd2: case (column)
                5'd0: mode_label_char = "M";
                5'd1: mode_label_char = "I";
                5'd2: mode_label_char = "P";
                5'd3: mode_label_char = "I";
                5'd5: mode_label_char = "R";
                5'd6: mode_label_char = "G";
                5'd7: mode_label_char = "B";
                default: mode_label_char = 8'h20;
            endcase
            2'd3: case (column)
                5'd0: mode_label_char = "S";
                5'd1: mode_label_char = "H";
                5'd2: mode_label_char = "A";
                5'd3: mode_label_char = "P";
                5'd4: mode_label_char = "E";
                5'd5: mode_label_char = "S";
                default: mode_label_char = " ";
            endcase
            3'd4: case (column)
                5'd0: mode_label_char = "B";
                5'd1: mode_label_char = "G";
                5'd3: mode_label_char = "D";
                5'd4: mode_label_char = "I";
                5'd5: mode_label_char = "F";
                5'd6: mode_label_char = "F";
                default: mode_label_char = " ";
            endcase
            default: mode_label_char = 8'h20;
        endcase
    end
endfunction

reg         window_stage_one;
reg  [3:0]  glyph_x_stage_one;
reg  [3:0]  glyph_y_stage_one;
reg  [26:0] video_stage_one;

reg         window_stage_two;
reg  [3:0]  glyph_x_stage_two;
reg  [26:0] video_stage_two;

assign frame_start = video_vs_i && !video_vs_d;

assign osd_window_active = video_de_i &&
                           (pixel_x >= OSD_X_FIRST) &&
                           (pixel_x < OSD_X_LIMIT) &&
                           (pixel_y >= OSD_Y_FIRST) &&
                           (pixel_y < OSD_Y_LIMIT);

assign osd_local_x = pixel_x - OSD_X_FIRST;
assign osd_local_y = pixel_y - OSD_Y_FIRST;
assign text_rd_addr = osd_window_active ?
                      {osd_local_y[5], osd_local_x[9:5]} : 6'd0;
assign font_glyph_id = (MODE_LABEL_ENABLE && mode_label_stage_one) ?
                       mode_glyph_stage_one : text_rd_data;

osd_text_ram u_osd_text_ram (
    .wr_clk_i  (text_wr_clk_i),
    .wr_en_i   (text_wr_en_i),
    .wr_addr_i (text_wr_addr_i),
    .wr_data_i (text_wr_data_i),
    .rd_clk_i  (pixel_clk_i),
    .rd_enable_i(pixel_enable_i),
    .rd_addr_i (text_rd_addr),
    .rd_data_o (text_rd_data)
);

// Keep the original glyph fallback and CE timing when the storage is shared.
assign font_address_o = {font_glyph_id[7] ? 7'h3f : font_glyph_id[6:0], glyph_y_stage_one};
assign font_enable_o = pixel_enable_i;
generate if (SHARED_FONT) begin : shared_font
    assign font_row_bits = font_pixels_i;
end else begin : local_font
osd_font_rom u_osd_font_rom (
    .clk_i       (pixel_clk_i),
    .enable_i(pixel_enable_i),
    .glyph_id_i  (font_glyph_id),
    .glyph_row_i (glyph_y_stage_one),
    .row_bits_o  (font_row_bits)
);
end endgenerate

always @(posedge pixel_clk_i or negedge pixel_rst_n_i) begin
    if (!pixel_rst_n_i) begin
        pixel_x          <= 11'd0;
        pixel_y          <= 11'd0;
        video_vs_d       <= 1'b0;
        video_de_d       <= 1'b0;
        window_stage_one <= 1'b0;
        mode_glyph_stage_one <= 8'h20;
        mode_label_stage_one <= 1'b0;
        glyph_x_stage_one <= 4'd0;
        glyph_y_stage_one <= 4'd0;
        video_stage_one  <= 27'd0;
        window_stage_two <= 1'b0;
        glyph_x_stage_two <= 4'd0;
        video_stage_two  <= 27'd0;
        video_vs_o       <= 1'b0;
        video_hs_o       <= 1'b0;
        video_de_o       <= 1'b0;
        video_rgb_o      <= 24'd0;
    end else if(pixel_enable_i) begin
        video_vs_d <= video_vs_i;
        video_de_d <= video_de_i;

        if (frame_start) begin
            pixel_x <= 11'd0;
            pixel_y <= 11'd0;
        end else begin
            if (video_de_i) begin
                if (pixel_x == H_LAST) begin
                    pixel_x <= 11'd0;
                end else begin
                    pixel_x <= pixel_x + 1'b1;
                end
            end else begin
                pixel_x <= 11'd0;
            end

            if (video_de_d && !video_de_i) begin
                if (pixel_y == V_LAST) begin
                    pixel_y <= 11'd0;
                end else begin
                    pixel_y <= pixel_y + 1'b1;
                end
            end
        end

        window_stage_one  <= osd_window_active;
        // mode_i is the committed pixel-domain mode, changed during display
        // reset/blanking. Match the synchronous text RAM latency and CE.
        mode_glyph_stage_one <= mode_label_char(mode_i, text_rd_addr[4:0]);
        mode_label_stage_one <= !text_rd_addr[5];
        glyph_x_stage_one <= osd_local_x[4:1];
        glyph_y_stage_one <= osd_local_y[4:1];
        video_stage_one   <= {video_vs_i, video_hs_i, video_de_i, video_rgb_i};

        window_stage_two  <= window_stage_one;
        glyph_x_stage_two <= glyph_x_stage_one;
        video_stage_two   <= video_stage_one;

        video_vs_o  <= video_stage_two[26];
        video_hs_o  <= video_stage_two[25];
        video_de_o  <= video_stage_two[24];
        video_rgb_o <= (window_stage_two &&
                        font_row_bits[15 - glyph_x_stage_two]) ?
                       TEXT_COLOR : video_stage_two[23:0];
    end
end

endmodule
