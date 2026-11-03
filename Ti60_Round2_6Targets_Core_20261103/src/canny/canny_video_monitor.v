`timescale 1ns/1ps
// Diagnose the live camera in a configurable 96-pixel display margin.
// Milestones cross as independent bits. Counts use a held payload and a
// request/acknowledge toggle, so multi-bit frame counts remain coherent.
module canny_video_monitor #(parameter TONE_MAP_ENABLE=0, parameter SHOW_PROGRESS=0,
    parameter integer DIAGNOSTIC_X=0) (
    input wire original_mode_i,
    input wire camera_clk_i, input wire camera_rst_n_i,
    input wire pixel_clk_i, input wire pixel_rst_n_i,
    input wire video_rst_n_i, input wire pixel_enable_i, input wire overlay_enable_i,
    input wire sensor_vsync_i, input wire sensor_href_i, input wire sensor_valid_i,
    input wire calibration_done_i, input wire calibration_pass_i,
    input wire [13:0] path_status_i, input wire [3:0] display_status_i,
    input wire [3:0] detail_fault_i,
    input wire frame_publish_toggle_i,
    input wire vsync_i, input wire hsync_i, input wire valid_i,
    input wire [23:0] rgb_i,
    output reg vsync_o, output reg hsync_o, output reg valid_o,
    output reg [23:0] rgb_o
);
    reg sensor_vsync_delay,raw_seen;
    reg [23:0] clock_count;
    reg [5:0] frame_count;
    reg [23:0] raw_count,qualified_count;
    reg [47:0] count_payload;
    reg count_request,count_acknowledge,frame_observed,size_error;
    (* async_reg="true" *) reg [2:0] acknowledge_sync;
    (* async_reg="true" *) reg [2:0] request_sync;
    wire frame_end=sensor_vsync_delay && !sensor_vsync_i;
    wire qualified_valid=sensor_valid_i && sensor_vsync_i && sensor_href_i;
    always @(posedge camera_clk_i or negedge camera_rst_n_i) begin
        if(!camera_rst_n_i) begin
            sensor_vsync_delay<=0; raw_seen<=0; clock_count<=0; frame_count<=0;
            raw_count<=0; qualified_count<=0; count_payload<=0;
            count_request<=0; acknowledge_sync<=0; frame_observed<=0; size_error<=0;
        end else begin
            sensor_vsync_delay<=sensor_vsync_i;
            clock_count<=clock_count+1'b1;
            acknowledge_sync<={acknowledge_sync[1:0],count_acknowledge};
            if(sensor_valid_i) begin
                raw_seen<=1'b1;
                if(!(&raw_count)) raw_count<=raw_count+1'b1;
            end
            if(qualified_valid && !(&qualified_count)) qualified_count<=qualified_count+1'b1;
            if(frame_end) begin
                frame_count<=frame_count+1'b1;
                raw_count<=0; qualified_count<=0; frame_observed<=1'b1;
                // Ignore a first interval that starts halfway through a frame.
                if(frame_observed && qualified_count!=24'd576000) size_error<=1'b1;
                if(count_request==acknowledge_sync[2]) begin
                    count_payload<={raw_count,qualified_count};
                    count_request<=!count_request;
                end
            end
        end
    end
    reg [47:0] displayed_counts;
    // One toggle per complete framebuffer publication, independent of HDMI
    // refresh. Sampled in the pixel domain; no multi-bit counter crosses CDC.
    (* async_reg="true" *) reg [2:0] publish_sync;
    reg publish_seen;
    reg [23:0] published_frames;
    always @(posedge pixel_clk_i or negedge pixel_rst_n_i) begin
        if(!pixel_rst_n_i) begin publish_sync<=0; publish_seen<=0; published_frames<=0; end
        else begin
            publish_sync<={publish_sync[1:0],frame_publish_toggle_i};
            publish_seen<=publish_sync[2];
            if(publish_sync[2]!=publish_seen) published_frames<=published_frames+1'b1;
        end
    end
    always @(posedge pixel_clk_i or negedge pixel_rst_n_i) begin
        if(!pixel_rst_n_i) begin request_sync<=0; count_acknowledge<=0; displayed_counts<=0; end
        else begin
            request_sync<={request_sync[1:0],count_request};
            if(request_sync[2]!=count_acknowledge) begin
                displayed_counts<=count_payload;
                count_acknowledge<=request_sync[2];
            end
        end
    end
    wire [27:0] status={detail_fault_i,path_status_i[13:12],size_error,path_status_i[11],
        display_status_i[3],path_status_i[10:8],display_status_i[2:0],
        path_status_i[5],path_status_i[6],path_status_i[4:0],path_status_i[7],
        raw_seen,frame_count[5],clock_count[23],calibration_pass_i,calibration_done_i};
    (* async_reg="true" *) reg [27:0] status_meta,status_sync,status_stable;
    always @(posedge pixel_clk_i or negedge pixel_rst_n_i) begin
        if(!pixel_rst_n_i) begin status_meta<=0; status_sync<=0; status_stable<=0; end
        else begin status_meta<=status; status_sync<=status_meta; status_stable<=status_sync; end
    end
    reg [11:0] column,row;
    wire [3:0] detail_stable=status_stable[27:24];
    reg valid_delay;
    always @(posedge pixel_clk_i or negedge video_rst_n_i) begin
        if(!video_rst_n_i) begin column<=0; row<=0; valid_delay<=0; end
        else if(pixel_enable_i) begin
            valid_delay<=valid_i;
            if(valid_i) column<=column+1'b1;
            else column<=0;
            if(!vsync_i) row<=0;
            else if(valid_delay && !valid_i) row<=row+1'b1;
        end
    end
    // Two six-digit hexadecimal rows: raw DE groups then VS/HS/DE groups.
    // A full 1920x1200 frame contains 08CA00 groups, four pixels per group.
    wire [11:0] diagnostic_column=column-DIAGNOSTIC_X;
    wire diagnostic_margin=column>=DIAGNOSTIC_X && column<DIAGNOSTIC_X+96;
    wire progress_area=SHOW_PROGRESS && diagnostic_margin && row>=1056 && row<1076;
    wire [23:0] digit_line=progress_area ? published_frames :
        (row[5] ? displayed_counts[47:24] : displayed_counts[23:0]);
    reg [3:0] digit;
    always @(*) begin
        case(diagnostic_column[6:4])
            0: digit=digit_line[23:20]; 1: digit=digit_line[19:16];
            2: digit=digit_line[15:12]; 3: digit=digit_line[11:8];
            4: digit=digit_line[7:4]; 5: digit=digit_line[3:0];
            default: digit=0;
        endcase
    end
    reg [14:0] glyph;
    always @(*) begin
        case(digit)
            4'h0: glyph=15'b111_101_101_101_111;
            4'h1: glyph=15'b010_110_010_010_111;
            4'h2: glyph=15'b111_001_111_100_111;
            4'h3: glyph=15'b111_001_111_001_111;
            4'h4: glyph=15'b101_101_111_001_001;
            4'h5: glyph=15'b111_100_111_001_111;
            4'h6: glyph=15'b111_100_111_101_111;
            4'h7: glyph=15'b111_001_010_010_010;
            4'h8: glyph=15'b111_101_111_101_111;
            4'h9: glyph=15'b111_101_111_001_111;
            4'ha: glyph=15'b111_101_111_101_101;
            4'hb: glyph=15'b110_101_110_101_110;
            4'hc: glyph=15'b111_100_100_100_111;
            4'hd: glyph=15'b110_101_101_101_110;
            4'he: glyph=15'b111_100_111_100_111;
            default: glyph=15'b111_100_111_100_100;
        endcase
    end
    reg [2:0] glyph_row;
    always @(*) begin
        case(row[4:2])
            0: glyph_row=glyph[14:12]; 1: glyph_row=glyph[11:9];
            2: glyph_row=glyph[8:6]; 3: glyph_row=glyph[5:3];
            4: glyph_row=glyph[2:0]; default: glyph_row=0;
        endcase
    end
    reg glyph_pixel;
    always @(*) begin
        case(diagnostic_column[3:2])
            0: glyph_pixel=glyph_row[2]; 1: glyph_pixel=glyph_row[1];
            2: glyph_pixel=glyph_row[0]; default: glyph_pixel=0;
        endcase
    end
    wire status_area=column>=DIAGNOSTIC_X+8 && column<DIAGNOSTIC_X+88 && row<768 && row[4:0]>=4 && row[4:0]<28;
    wire digit_area=diagnostic_margin && row>=800 && row<864;
    // Four distinct fault bars below the count digits: write, RRESP,
    // starvation, then nonbinary/incorrect-length framebuffer input.
    wire detail_area=column>=DIAGNOSTIC_X+8 && column<DIAGNOSTIC_X+88 && row>=896 && row<1024 && row[4:0]>=4 && row[4:0]<28;
    wire status_on=row[9:5]<24 ? status_stable[row[9:5]] : 1'b0;
    wire status_fault=row[9:5]>=16 && row[9:5]<=21;
    // W3C CSS Color 4 lin_sRGB transfer function, sampled at the 64
    // levels retained by the RGB565 green channel. Black and white remain
    // exact; the Canny input and binary display never pass through this LUT.
    function [7:0] linear_gray_to_srgb;
        input [5:0] level;
        begin
            case(level)
                6'd0: linear_gray_to_srgb=8'd0;
                6'd1: linear_gray_to_srgb=8'd34;
                6'd2: linear_gray_to_srgb=8'd50;
                6'd3: linear_gray_to_srgb=8'd62;
                6'd4: linear_gray_to_srgb=8'd71;
                6'd5: linear_gray_to_srgb=8'd80;
                6'd6: linear_gray_to_srgb=8'd87;
                6'd7: linear_gray_to_srgb=8'd94;
                6'd8: linear_gray_to_srgb=8'd100;
                6'd9: linear_gray_to_srgb=8'd106;
                6'd10: linear_gray_to_srgb=8'd111;
                6'd11: linear_gray_to_srgb=8'd116;
                6'd12: linear_gray_to_srgb=8'd121;
                6'd13: linear_gray_to_srgb=8'd125;
                6'd14: linear_gray_to_srgb=8'd130;
                6'd15: linear_gray_to_srgb=8'd134;
                6'd16: linear_gray_to_srgb=8'd138;
                6'd17: linear_gray_to_srgb=8'd142;
                6'd18: linear_gray_to_srgb=8'd146;
                6'd19: linear_gray_to_srgb=8'd149;
                6'd20: linear_gray_to_srgb=8'd153;
                6'd21: linear_gray_to_srgb=8'd156;
                6'd22: linear_gray_to_srgb=8'd160;
                6'd23: linear_gray_to_srgb=8'd163;
                6'd24: linear_gray_to_srgb=8'd166;
                6'd25: linear_gray_to_srgb=8'd169;
                6'd26: linear_gray_to_srgb=8'd172;
                6'd27: linear_gray_to_srgb=8'd175;
                6'd28: linear_gray_to_srgb=8'd178;
                6'd29: linear_gray_to_srgb=8'd181;
                6'd30: linear_gray_to_srgb=8'd183;
                6'd31: linear_gray_to_srgb=8'd186;
                6'd32: linear_gray_to_srgb=8'd189;
                6'd33: linear_gray_to_srgb=8'd191;
                6'd34: linear_gray_to_srgb=8'd194;
                6'd35: linear_gray_to_srgb=8'd197;
                6'd36: linear_gray_to_srgb=8'd199;
                6'd37: linear_gray_to_srgb=8'd201;
                6'd38: linear_gray_to_srgb=8'd204;
                6'd39: linear_gray_to_srgb=8'd206;
                6'd40: linear_gray_to_srgb=8'd209;
                6'd41: linear_gray_to_srgb=8'd211;
                6'd42: linear_gray_to_srgb=8'd213;
                6'd43: linear_gray_to_srgb=8'd215;
                6'd44: linear_gray_to_srgb=8'd218;
                6'd45: linear_gray_to_srgb=8'd220;
                6'd46: linear_gray_to_srgb=8'd222;
                6'd47: linear_gray_to_srgb=8'd224;
                6'd48: linear_gray_to_srgb=8'd226;
                6'd49: linear_gray_to_srgb=8'd228;
                6'd50: linear_gray_to_srgb=8'd230;
                6'd51: linear_gray_to_srgb=8'd232;
                6'd52: linear_gray_to_srgb=8'd234;
                6'd53: linear_gray_to_srgb=8'd236;
                6'd54: linear_gray_to_srgb=8'd238;
                6'd55: linear_gray_to_srgb=8'd240;
                6'd56: linear_gray_to_srgb=8'd242;
                6'd57: linear_gray_to_srgb=8'd244;
                6'd58: linear_gray_to_srgb=8'd246;
                6'd59: linear_gray_to_srgb=8'd248;
                6'd60: linear_gray_to_srgb=8'd250;
                6'd61: linear_gray_to_srgb=8'd251;
                6'd62: linear_gray_to_srgb=8'd253;
                6'd63: linear_gray_to_srgb=8'd255;
                default: linear_gray_to_srgb=8'd0;
            endcase
        end
    endfunction
    wire [7:0] display_gray=linear_gray_to_srgb(rgb_i[15:10]);
    reg [23:0] selected_rgb;
    always @(*) begin
        selected_rgb=(TONE_MAP_ENABLE && original_mode_i) ? {3{display_gray}} : rgb_i;
        if(overlay_enable_i && status_area)
            selected_rgb=status_on ? (status_fault ? 24'hff2020 : 24'h20ff20) : 24'h303030;
        else if(overlay_enable_i && (digit_area || progress_area)) selected_rgb=glyph_pixel ? 24'hffffff : 24'h000020;
        else if(overlay_enable_i && detail_area) selected_rgb=detail_stable[row[6:5]] ? 24'hff2020 : 24'h303030;
    end
    // Register RGB and every control together; no video/control skew.
    always @(posedge pixel_clk_i or negedge video_rst_n_i) begin
        if(!video_rst_n_i) begin vsync_o<=0; hsync_o<=0; valid_o<=0; rgb_o<=0; end
        else if(pixel_enable_i) begin vsync_o<=vsync_i; hsync_o<=hsync_i; valid_o<=valid_i; rgb_o<=selected_rgb; end
    end
endmodule
