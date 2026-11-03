`timescale 1ns / 1ps
// Canny 写回旁路提取特征，完成匹配后只绘制一次同一个 DDR 工作帧。
// The map is accumulated during hysteresis; annotations belong to that same job.
module canny_rectangle_render #(
    parameter MAX_OBJECTS=4,parameter LABEL_BITS=MAX_OBJECTS>4 ? 4 : 3,
    parameter SUPPORT_COMPARE=0,
    parameter SUPPORT_RECTANGLE=0,
    parameter ENABLE_EXTENDED_SHAPES=0,
    parameter SHARED_FONT=0,
    parameter SUPPORT_GRAY=0,
    parameter DISPLAY_WIDTH=1920,
    parameter DISPLAY_HEIGHT=1080,
    parameter SCALE_NUM=10,
    parameter SCALE_DEN=9,
    parameter ROW_STRIDE=512
) (
    input wire compare_i,output wire compare_left_o,
    input wire rectangle_i,
    input wire feature_start_i,
    input wire feature_valid_i,
    input wire [63:0] feature_data_i,
    input wire [10:0] feature_row_i,
    input wire [5:0] feature_word_i,
    input wire feature_first_i,
    input wire gray_i,
    input wire clk_i,
    input wire rst_n_i,
    input wire start_i,
    input wire [31:0] base_address_i,
    output wire busy_o,
    output wire done_o,
    output wire read_request_o,
    input wire read_request_ready_i,
    output wire [31:0] read_address_o,
    input wire read_valid_i,
    output wire read_ready_o,
    input wire [63:0] read_data_i,
    output wire frame_active_o,
    output wire valid_o,
    input wire ready_i,
    output wire sop_o,
    output wire eop_o,
    output wire [63:0] rgb565_o,
    output wire [10:0] font_address_o,
    output wire font_enable_o,
    input wire [15:0] font_pixels_i
);
    generate if(SUPPORT_RECTANGLE) begin : rectangle_enabled
        localparam ST_IDLE=0,ST_FIRST=1,ST_ANALYZE=2,ST_PREPARE=3,ST_SECOND=4,ST_DRAIN=5,ST_FINISH=6,ST_COORDINATES=7;
        reg [2:0] state,next_state;
        (* async_reg="true" *) reg [2:0] request_sync;
        reg rectangle_job,renderer_start;
        reg [31:0] job_address;
        reg job_gray,job_compare;
        wire renderer_compare_left;
        assign compare_left_o=renderer_compare_left && !rectangle_job;
        wire renderer_done,renderer_frame,renderer_valid,renderer_ready,renderer_sop,renderer_eop;
        wire [63:0] renderer_pixels;
        wire detector_done,found,overflow;
        wire [10:0] left,top,width,height;
        wire [21:0] area;
        wire [2:0] object_count;
        localparam KIND_BITS=ENABLE_EXTENDED_SHAPES ? 4 : 3;
        wire [MAX_OBJECTS*KIND_BITS-1:0] object_kind;
        wire [13:0] label_address;wire [4*LABEL_BITS-1:0] label_data;
        wire [MAX_OBJECTS*11-1:0] object_left,object_top,object_width,object_height;
        wire objects_full;
        wire [MAX_OBJECTS*11-1:0] display_left,display_top,display_width,display_height;
        reg [6:0] result_valid_pipe;
        wire prepared,overlay_ready,overlay_frame,overlay_valid;
        wire overlay_sop,overlay_eop;
        wire [63:0] overlay_pixels;
        wire second_pass=rectangle_job && (state==ST_SECOND || state==ST_DRAIN);
        always @(posedge clk_i or negedge rst_n_i) begin
            if(!rst_n_i) begin request_sync<=0; state<=ST_IDLE; end
            else begin request_sync<={request_sync[1:0],rectangle_i}; state<=next_state; end
        end
        always @(*) begin
            next_state=state;
            case(state)
                ST_IDLE: if(start_i) begin next_state=(request_sync[2] && !gray_i && !compare_i) ? ST_ANALYZE : ST_FIRST; end
                ST_FIRST: if(renderer_done) begin next_state=ST_FINISH; end
                ST_ANALYZE: if(detector_done) begin next_state=ST_COORDINATES; end
                ST_COORDINATES: if(result_valid_pipe[6]) begin next_state=ST_PREPARE; end
                ST_PREPARE: if(prepared) begin next_state=ST_SECOND; end
                ST_SECOND: if(renderer_done) begin next_state=ST_DRAIN; end
                ST_DRAIN: if(!overlay_frame) begin next_state=ST_FINISH; end
                ST_FINISH: begin next_state=ST_IDLE; end
                default: begin next_state=ST_IDLE; end
            endcase
        end
        always @(posedge clk_i or negedge rst_n_i) begin
            if(!rst_n_i) begin rectangle_job<=0; renderer_start<=0; job_address<=0; job_gray<=0;job_compare<=0; end
            else begin
                renderer_start<=0;
                if(state==ST_IDLE && start_i) begin
                    rectangle_job<=request_sync[2] && !gray_i && !compare_i;
                    job_address<=base_address_i; job_gray<=gray_i;job_compare<=compare_i; renderer_start<=!(request_sync[2] && !gray_i && !compare_i);
                end
                if(state==ST_PREPARE && prepared) begin renderer_start<=1; end
            end
        end
        // Each object's coordinates use the same registered source-to-HDMI transform.
        always @(posedge clk_i or negedge rst_n_i) begin
            if(!rst_n_i) result_valid_pipe<=0;
            else result_valid_pipe<={result_valid_pipe[5:0],detector_done};
        end
        genvar slot;
        for(slot=0;slot<MAX_OBJECTS;slot=slot+1) begin : scale_objects
            reg [10:0] source_right,source_bottom;
            reg [15:0] left_scaled,top_scaled,right_scaled,bottom_scaled;
            reg [10:0] divided_left,divided_top,divided_right,divided_bottom;
            reg [10:0] scaled_left,scaled_top,scaled_width,scaled_height;
            (* syn_multstyle="block" *) wire [31:0] left_reciprocal=left_scaled*16'd52429;
            (* syn_multstyle="block" *) wire [31:0] top_reciprocal=top_scaled*16'd52429;
            (* syn_multstyle="block" *) wire [31:0] right_reciprocal=right_scaled*16'd52429;
            (* syn_multstyle="block" *) wire [31:0] bottom_reciprocal=bottom_scaled*16'd52429;
            always @(posedge clk_i or negedge rst_n_i) begin
                if(!rst_n_i) begin
                    source_right<=0;source_bottom<=0;
                    left_scaled<=0;top_scaled<=0;right_scaled<=0;bottom_scaled<=0;
                    divided_left<=0;divided_top<=0;divided_right<=0;divided_bottom<=0;
                    scaled_left<=0;scaled_top<=0;scaled_width<=0;scaled_height<=0;
                end else begin
                    source_right<=object_left[slot*11+:11]+object_width[slot*11+:11];
                    source_bottom<=object_top[slot*11+:11]+object_height[slot*11+:11];
                    left_scaled<=object_left[slot*11+:11]*16'd9;
                    top_scaled<=object_top[slot*11+:11]*16'd9;
                    right_scaled<=source_right*16'd9+16'd9;
                    bottom_scaled<=source_bottom*16'd9+16'd9;
                    divided_left<=left_reciprocal[29:19];divided_top<=top_reciprocal[29:19];
                    divided_right<=right_reciprocal[29:19];divided_bottom<=bottom_reciprocal[29:19];
                    scaled_left<=divided_left+96;scaled_top<=divided_top;
                    scaled_width<=divided_right-divided_left;scaled_height<=divided_bottom-divided_top;
                end
            end
            assign display_left[slot*11+:11]=scaled_left;
            assign display_top[slot*11+:11]=scaled_top;
            assign display_width[slot*11+:11]=scaled_width;
            assign display_height[slot*11+:11]=scaled_height;
        end
        assign renderer_ready=rectangle_job ? overlay_ready : ready_i;
        canny_display_render #(.MAX_OBJECTS(MAX_OBJECTS),.SUPPORT_COMPARE(SUPPORT_COMPARE),.SUPPORT_LABELS(1),.KIND_BITS(KIND_BITS),.SUPPORT_GRAY(SUPPORT_GRAY),.DISPLAY_WIDTH(DISPLAY_WIDTH),.DISPLAY_HEIGHT(DISPLAY_HEIGHT),
            .SCALE_NUM(SCALE_NUM),.SCALE_DEN(SCALE_DEN),.ROW_STRIDE(ROW_STRIDE)) renderer (
            .compare_i(job_compare),.compare_left_o(renderer_compare_left),
            .color_enable_i(rectangle_job && object_count!=0),.object_kind_i(object_kind),.label_address_o(label_address),.label_data_i(label_data),
            .full_scan_i(1'b0),.gray_i(job_gray),.clk_i(clk_i),.rst_n_i(rst_n_i),.start_i(renderer_start),.base_address_i(job_address),
            .busy_o(),.done_o(renderer_done),.read_request_o(read_request_o),.read_request_ready_i(read_request_ready_i),
            .read_address_o(read_address_o),.read_valid_i(read_valid_i),.read_ready_o(read_ready_o),.read_data_i(read_data_i),
            .frame_active_o(renderer_frame),.valid_o(renderer_valid),.ready_i(renderer_ready),
            .sop_o(renderer_sop),.eop_o(renderer_eop),.rgb565_o(renderer_pixels)
        );
        rectangle_feature_match #(.MAX_OBJECTS(MAX_OBJECTS),.INPUT_WIDTH(1920),.INPUT_HEIGHT(1200),.WRITE_TAP(1),.ENABLE_SHAPES(1),.ENABLE_DIGITS(1),.ENABLE_EXTENDED_SHAPES(ENABLE_EXTENDED_SHAPES)) detector (
            .label_enable_i(rectangle_job),.label_address_i(label_address),.label_data_o(label_data),
            .clk_i(clk_i),.rst_n_i(rst_n_i),.start_i(feature_start_i),
            .analyze_i(state==ST_IDLE && start_i && request_sync[2] && !gray_i && !compare_i),
            .tap_valid_i(feature_valid_i),.tap_data_i(feature_data_i),
            .tap_row_i(feature_row_i),.tap_word_i(feature_word_i),.tap_first_i(feature_first_i),
            .valid_i(1'b0),.pixels_i(64'b0),.eop_i(1'b0),
            .ready_o(),.done_o(detector_done),.found_o(found),.overflow_o(overflow),
            .left_o(left),.top_o(top),.width_o(width),.height_o(height),.area_o(area),.angle_o(),.cycles_o(),
            .object_count_o(object_count),.object_kind_o(object_kind),.object_left_o(object_left),.object_top_o(object_top),
            .object_width_o(object_width),.object_height_o(object_height),.objects_full_o(objects_full)
        );
        shape_result_overlay #(.MAX_OBJECTS(MAX_OBJECTS),.SHARED_FONT(SHARED_FONT),.KIND_BITS(KIND_BITS)) overlay (
            .font_address_o(font_address_o),.font_enable_o(font_enable_o),.font_pixels_i(font_pixels_i),
            .clk_i(clk_i),.rst_n_i(rst_n_i),.prepare_i(result_valid_pipe[6]),.count_i(object_count),.kinds_i(object_kind),.overflow_i(overflow),.full_i(objects_full),
            .lefts_i(display_left),.tops_i(display_top),.widths_i(display_width),.heights_i(display_height),.prepared_o(prepared),
            .frame_i(renderer_frame && second_pass),.valid_i(renderer_valid && second_pass),
            .sop_i(renderer_sop),.eop_i(renderer_eop),.pixels_i(renderer_pixels),.ready_o(overlay_ready),.ready_i(ready_i),
            .frame_o(overlay_frame),.valid_o(overlay_valid),.sop_o(overlay_sop),.eop_o(overlay_eop),.pixels_o(overlay_pixels)
        );
        assign busy_o=state!=ST_IDLE;
        assign done_o=state==ST_FINISH;
        assign frame_active_o=rectangle_job ? (overlay_frame && second_pass) : renderer_frame;
        assign valid_o=rectangle_job ? (overlay_valid && second_pass) : renderer_valid;
        assign rgb565_o=rectangle_job ? overlay_pixels : renderer_pixels;
        assign sop_o=rectangle_job ? overlay_sop : renderer_sop;
        assign eop_o=rectangle_job ? overlay_eop : renderer_eop;
    end else begin : rectangle_disabled
        assign font_address_o=11'd0;
        assign font_enable_o=1'b0;
        canny_display_render #(.MAX_OBJECTS(MAX_OBJECTS),.SUPPORT_COMPARE(SUPPORT_COMPARE),.SUPPORT_GRAY(SUPPORT_GRAY),.DISPLAY_WIDTH(DISPLAY_WIDTH),.DISPLAY_HEIGHT(DISPLAY_HEIGHT),
            .SCALE_NUM(SCALE_NUM),.SCALE_DEN(SCALE_DEN),.ROW_STRIDE(ROW_STRIDE)) renderer (
            .compare_i(compare_i),.compare_left_o(compare_left_o),
            .gray_i(gray_i),.clk_i(clk_i),.rst_n_i(rst_n_i),.start_i(start_i),.base_address_i(base_address_i),
            .busy_o(busy_o),.done_o(done_o),.read_request_o(read_request_o),.read_request_ready_i(read_request_ready_i),
            .read_address_o(read_address_o),.read_valid_i(read_valid_i),.read_ready_o(read_ready_o),.read_data_i(read_data_i),
            .frame_active_o(frame_active_o),.valid_o(valid_o),.ready_i(ready_i),.sop_o(sop_o),.eop_o(eop_o),.rgb565_o(rgb565_o)
        );
    end endgenerate
endmodule
