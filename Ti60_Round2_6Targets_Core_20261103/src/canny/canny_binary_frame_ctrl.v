`timescale 1ns/1ps
// Lossless black/white framebuffer codec around the existing official-FIFO
// controller. Camera processing and HDMI timing remain in their own domains.
module canny_binary_frame_ctrl #(
    parameter C_BASE_ADDR=32'h01000000,
    parameter C_RD_END_ADDR=1920*1080*2,
    parameter C_W_WIDTH=64, C_R_WIDTH=16, C_ID_LEN=4,
    parameter C_READ_BANKS=2, parameter SUPPORT_GRAY=0, parameter SUPPORT_RGB=0,
    parameter SUPPORT_PALETTE=0, parameter C_HISTORY_ENABLE=0,
    parameter C_COMPARE_SPLIT=522240, parameter C_GRAY_COMPARE_SPLIT=2088960
) (
    input wire [1:0] history_control_i,
    input wire [7:0] history_age_a_i, input wire [7:0] history_age_b_i,
    output wire [7:0] history_count_o,
    input wire axi_clk, input wire axi_reset,
    output wire [31:0] axi_awaddr, output wire [7:0] axi_awlen,
    output wire axi_awvalid, input wire axi_awready,
    output wire [127:0] axi_wdata, output wire [15:0] axi_wstrb,
    output wire axi_wlast, output wire axi_wvalid, input wire axi_wready,
    input wire [C_ID_LEN-1:0] axi_bid, input wire [1:0] axi_bresp,
    input wire axi_bvalid, output wire axi_bready,
    output wire [C_ID_LEN-1:0] axi_arid, output wire [31:0] axi_araddr,
    output wire [7:0] axi_arlen, output wire axi_arvalid, input wire axi_arready,
    input wire [C_ID_LEN-1:0] axi_rid, input wire [127:0] axi_rdata,
    input wire [1:0] axi_rresp, input wire axi_rlast, input wire axi_rvalid,
    output wire axi_rready,
    input wire wframe_gray_i, input wire rframe_gray_i,
    input wire wframe_pclk, input wire wframe_vsync,
    input wire wframe_data_en, input wire [63:0] wframe_data,
    output wire wframe_ready_o, output wire error_o,
    output wire [3:0] debug_status_o, output wire [3:0] detail_fault_o,
    output wire frame_publish_toggle_o,
    input wire rframe_pclk, input wire rframe_vsync,
    input wire rframe_data_en, output wire [15:0] rframe_data,
    output wire [31:0] tp_o
);
    // The production image is a multiple of 64 pixels. Pad only the DDR tail
    // to a complete 2048-byte burst; those padding bits are never displayed.
    localparam INPUT_GROUPS=C_RD_END_ADDR/8;
    localparam PACKED_BYTES=SUPPORT_PALETTE ? C_RD_END_ADDR/4 : C_RD_END_ADDR/16;
    localparam PADDED_BYTES=((PACKED_BYTES+2047)/2048)*2048;
    localparam PACKED_WORDS=PADDED_BYTES/8;
    // The nonbinary slot stores full RGB565 when enabled, otherwise RAW8 gray.
    // Both fit the existing 4 MiB display slot; the Canny codec is unchanged.
    localparam GRAY_BYTES=SUPPORT_RGB ? C_RD_END_ADDR : C_RD_END_ADDR/2;
    localparam GRAY_PADDED=((GRAY_BYTES+2047)/2048)*2048;
    reg frame_gray;
    wire [19:0] target_words=frame_gray ? GRAY_PADDED/8 : PACKED_WORDS;
    wire [31:0] gray_group={wframe_data[58:53],2'b00,wframe_data[42:37],2'b00,wframe_data[26:21],2'b00,wframe_data[10:5],2'b00};
    wire read_gray=SUPPORT_GRAY && rframe_gray_i;
    localparam WAIT_FRAME=0, RECEIVE=1, PAD=2, WAIT_COMMIT=3, FAULT=4;
    reg [2:0] state;
    reg [3:0] pack_index;
    reg [63:0] pack_accumulator,packed_data;
    reg packed_valid,packed_frame,format_error;
    reg [19:0] input_groups;
    reg [19:0] words_sent;
    wire packed_ready,core_error,frame_commit;
    wire [3:0] core_status;
    wire [2:0] core_fault;
    reg commit_toggle;
    assign frame_publish_toggle_o=commit_toggle;
    (* async_reg="true" *) reg [2:0] commit_sync;
    reg commit_seen;
    wire [3:0] edge_bits={wframe_data[63],wframe_data[47],wframe_data[31],wframe_data[15]};
    wire binary_input=(wframe_data[63:48]==16'h0000 || wframe_data[63:48]==16'hffff) &&
        (wframe_data[47:32]==16'h0000 || wframe_data[47:32]==16'hffff) &&
        (wframe_data[31:16]==16'h0000 || wframe_data[31:16]==16'hffff) &&
        (wframe_data[15:0]==16'h0000 || wframe_data[15:0]==16'hffff);
    // Four-bit color indices retain compression while preserving annotation colors.
    // The legacy one-bit format remains selectable for existing configurations.
    wire [15:0] palette_group;
    wire [3:0] palette_valid;
    genvar color_lane;
    generate for(color_lane=0;color_lane<4;color_lane=color_lane+1) begin : palette_encode
        wire [15:0] color_pixel=wframe_data[(3-color_lane)*16+:16];
        reg [3:0] color_index;
        reg color_valid;
        // Preserve all legacy colors; remaining four-bit entries hold bright OSD.
        always @(*) begin
            color_valid=1'b1;
            case(color_pixel)
                16'h0000:color_index=4'd0;
                16'hffff:color_index=4'd1;
                16'h07e0:color_index=4'd2;
                16'h07ff:color_index=4'd3;
                16'hffe0:color_index=4'd4;
                16'hf800:color_index=4'd5;
                16'hf81f:color_index=4'd6;
                16'hfd20:color_index=4'd7;
                16'h001f:color_index=4'd8;
                16'h87f0:color_index=4'd9;
                16'h87ff:color_index=4'd10;
                16'hfff0:color_index=4'd11;
                16'hfc1f:color_index=4'd12;
                16'hfdc8:color_index=4'd13;
                16'h7dff:color_index=4'd14;
                16'hfa08:color_index=4'd15;
                default:begin color_index=4'd0;color_valid=1'b0;end
            endcase
        end
        assign palette_group[(3-color_lane)*4+:4]=color_index;
        assign palette_valid[color_lane]=color_valid;
    end endgenerate
    wire packed_input_valid=SUPPORT_PALETTE ? (&palette_valid) : binary_input;
    // Refill the output register on the same edge that its previous word is
    // accepted. A forced empty cycle would cap RGB at two pixels per clock.
    assign wframe_ready_o=state==RECEIVE && (!packed_valid || packed_ready);
    always @(posedge axi_clk or posedge axi_reset) begin
        if(axi_reset) commit_toggle<=0;
        else if(frame_commit) commit_toggle<=!commit_toggle;
    end
    always @(posedge wframe_pclk or posedge axi_reset) begin
        if(axi_reset) begin
            state<=WAIT_FRAME; pack_index<=0; pack_accumulator<=0; packed_data<=0;
            packed_valid<=0; packed_frame<=0; format_error<=0;
            input_groups<=0; words_sent<=0; frame_gray<=0; commit_sync<=0; commit_seen<=0;
        end else begin
            commit_sync<={commit_sync[1:0],commit_toggle};
            if(packed_valid && packed_ready) begin packed_valid<=0; words_sent<=words_sent+1'b1; end
            case(state)
                WAIT_FRAME: if(wframe_vsync) begin
                    frame_gray<=SUPPORT_GRAY && wframe_gray_i;
                    packed_frame<=1; pack_index<=0; input_groups<=0; words_sent<=0;
                    pack_accumulator<=0; state<=RECEIVE;
                end
                RECEIVE: begin
                    if(wframe_data_en && wframe_ready_o) begin
                        input_groups<=input_groups+1'b1;
                        if(frame_gray) begin
                            if(SUPPORT_RGB) begin
                                packed_data<=wframe_data; packed_valid<=1; pack_index<=0;
                            end else begin
                            pack_accumulator<={pack_accumulator[31:0],gray_group};
                            pack_index<=pack_index==1 ? 0 : 1;
                            if(pack_index==1) begin packed_data<={pack_accumulator[31:0],gray_group}; packed_valid<=1; end
                            end
                        end else if(SUPPORT_PALETTE) begin
                            pack_accumulator<={pack_accumulator[47:0],palette_group};
                            pack_index<=pack_index==3 ? 0 : pack_index+1'b1;
                            if(pack_index==3) begin packed_data<={pack_accumulator[47:0],palette_group};packed_valid<=1;end
                        end else begin
                            pack_accumulator<={pack_accumulator[59:0],edge_bits};
                            pack_index<=pack_index+1'b1;
                            if(pack_index==15) begin packed_data<={pack_accumulator[59:0],edge_bits}; packed_valid<=1; end
                        end
                        if((!frame_gray && !packed_input_valid) || input_groups>=INPUT_GROUPS) begin format_error<=1; state<=FAULT; end
                    end
                    if(!wframe_vsync) begin
                        if(input_groups==INPUT_GROUPS && pack_index==0) state<=PAD;
                        else begin format_error<=1; state<=FAULT; end
                    end
                end
                PAD: if(!packed_valid) begin
                    if(words_sent==target_words) begin packed_frame<=0; state<=WAIT_COMMIT; end
                    else begin packed_data<=0; packed_valid<=1; end
                end
                WAIT_COMMIT: if(commit_sync[2]!=commit_seen) begin
                    commit_seen<=commit_sync[2]; state<=WAIT_FRAME;
                end
                FAULT: begin end
                default: state<=FAULT;
            endcase
        end
    end
    reg [3:0] bit_index;
    reg rframe_vsync_d;
    wire [15:0] packed_pixels;
    wire read_word=rframe_data_en && (read_gray ? (SUPPORT_RGB || bit_index[0]) : (SUPPORT_PALETTE ? bit_index==3 : bit_index==15));
    always @(posedge rframe_pclk or posedge axi_reset) begin
        if(axi_reset) begin bit_index<=0; rframe_vsync_d<=0; end
        else begin
            rframe_vsync_d<=rframe_vsync;
            if(rframe_vsync_d && !rframe_vsync) bit_index<=0;
            else if(rframe_data_en) bit_index<=read_gray ? {3'd0,!bit_index[0]} : (SUPPORT_PALETTE && bit_index==3 ? 0 : bit_index+1'b1);
        end
    end
    wire [7:0] gray_pixel=bit_index[0] ? packed_pixels[7:0] : packed_pixels[15:8];
    reg [15:0] palette_pixel;
    wire [3:0] palette_index=packed_pixels[15-bit_index[1:0]*4-:4];
    always @(*) begin
        case(palette_index)
            1: palette_pixel=16'hffff;
            2: palette_pixel=16'h07e0;
            3: palette_pixel=16'h07ff;
            4: palette_pixel=16'hffe0;
            5: palette_pixel=16'hf800;
            6: palette_pixel=16'hf81f;
            7: palette_pixel=16'hfd20;
            8: palette_pixel=16'h001f;
            9: palette_pixel=16'h87f0;
            10: palette_pixel=16'h87ff;
            11: palette_pixel=16'hfff0;
            12: palette_pixel=16'hfc1f;
            13: palette_pixel=16'hfdc8;
            14: palette_pixel=16'h7dff;
            15: palette_pixel=16'hfa08;
            default: palette_pixel=16'h0000;
        endcase
    end
    assign rframe_data=read_gray ? (SUPPORT_RGB ? packed_pixels : {gray_pixel[7:3],gray_pixel[7:2],gray_pixel[7:3]}) : (SUPPORT_PALETTE ? palette_pixel : (packed_pixels[15-bit_index] ? 16'hffff : 16'h0000));
    assign error_o=core_error || format_error;
    assign detail_fault_o={format_error,core_fault};
    assign debug_status_o={error_o,core_status[2:0]};
    canny_frame_ctrl #(.C_BASE_ADDR(C_BASE_ADDR),.C_RD_END_ADDR(PADDED_BYTES),.C_HISTORY_ENABLE(C_HISTORY_ENABLE),
        .C_COMPARE_SPLIT(C_COMPARE_SPLIT),.C_GRAY_COMPARE_SPLIT(C_GRAY_COMPARE_SPLIT),
        .C_GRAY_END_ADDR(SUPPORT_GRAY ? GRAY_PADDED : 0),.C_W_WIDTH(64),.C_R_WIDTH(16),.C_ID_LEN(C_ID_LEN),.C_READ_BANKS(C_READ_BANKS)) storage (
        .history_control_i(history_control_i),.history_age_a_i(history_age_a_i),
        .history_age_b_i(history_age_b_i),.history_count_o(history_count_o),
        .wframe_gray_i(frame_gray),.rframe_gray_i(read_gray),
        .axi_clk(axi_clk),.axi_reset(axi_reset),
        .axi_awaddr(axi_awaddr),.axi_awlen(axi_awlen),.axi_awvalid(axi_awvalid),.axi_awready(axi_awready),
        .axi_wdata(axi_wdata),.axi_wstrb(axi_wstrb),.axi_wlast(axi_wlast),.axi_wvalid(axi_wvalid),.axi_wready(axi_wready),
        .axi_bid(axi_bid),.axi_bresp(axi_bresp),.axi_bvalid(axi_bvalid),.axi_bready(axi_bready),
        .axi_arid(axi_arid),.axi_araddr(axi_araddr),.axi_arlen(axi_arlen),.axi_arvalid(axi_arvalid),.axi_arready(axi_arready),
        .axi_rid(axi_rid),.axi_rdata(axi_rdata),.axi_rresp(axi_rresp),.axi_rlast(axi_rlast),.axi_rvalid(axi_rvalid),.axi_rready(axi_rready),
        .wframe_pclk(wframe_pclk),.wframe_vsync(packed_frame),.wframe_data_en(packed_valid),.wframe_data(packed_data),
        .wframe_ready_o(packed_ready),.error_o(core_error),.debug_status_o(core_status),
        .detail_fault_o(core_fault),.frame_published_o(frame_commit),
        .rframe_pclk(rframe_pclk),.rframe_vsync(rframe_vsync),.rframe_data_en(read_word),.rframe_data(packed_pixels),.tp_o(tp_o)
    );
endmodule
