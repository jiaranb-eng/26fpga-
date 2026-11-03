`timescale 1ns/1ps
// Camera clock owns the streaming frontend and capture ingress.
// Complete DDR jobs and display output run on the continuous AXI clock.
// Master zero writes candidate maps; master one serves iterative row access.
module canny_camera_path #(
    parameter MAX_OBJECTS=4,
    parameter SUPPORT_COMPARE=0,
    parameter SUPPORT_RECTANGLE=0,
    parameter ENABLE_EXTENDED_SHAPES=0,
    parameter SHARED_FONT=0,
    parameter SUPPORT_RGB=0,
    parameter DISPLAY_WIDTH=1280, parameter DISPLAY_HEIGHT=720,
    parameter SCALE_NUM=5, parameter SCALE_DEN=3
) (
    output wire [10:0] font_address_o,
    output wire font_enable_o,
    input wire [15:0] font_pixels_i,
    input wire [1:0] compare_request_i,
    input wire rectangle_request_i,
    input wire raw_request_i, output wire display_gray_o,
    input wire background_request_i, input wire background_relearn_i,
    input wire [7:0] background_threshold_i,
    output wire background_valid_o, output wire background_pending_o,
    output wire [31:0] background_epoch_o, output wire [31:0] background_frames_o,
    output wire [31:0] background_cycles_o, output wire [3:0] background_count_o,
    output wire background_overflow_o,
    input wire background_mask_capture_i,
    input wire [13:0] background_mask_address_i,
    output wire [7:0] background_mask_data_o,
    output wire background_mask_ready_o, output wire background_mask_busy_o,
    output wire [7:0] background_mask_epoch_o,

    input wire camera_clk_i, input wire camera_rst_n_i,
    input wire axi_clk_i, input wire axi_rst_n_i,
    input wire frame_vsync_i, input wire frame_href_i, input wire pixel_valid_i,
    input wire [31:0] gray_i,
    input wire [10:0] low_threshold_i, input wire [10:0] high_threshold_i,
    output wire display_frame_o, output wire display_valid_o, output wire [63:0] display_data_o,
    input wire display_ready_i,
    output wire error_o, output wire [31:0] last_cycles_o, output wire [31:0] last_sweeps_o,
    output wire [13:0] debug_status_o,
    output wire [63:0] axi_awaddr_o, output wire [15:0] axi_awlen_o,
    output wire [1:0] axi_awvalid_o, input wire [1:0] axi_awready_i,
    output wire [255:0] axi_wdata_o, output wire [31:0] axi_wstrb_o,
    output wire [1:0] axi_wlast_o, output wire [1:0] axi_wvalid_o, input wire [1:0] axi_wready_i,
    input wire [3:0] axi_bresp_i, input wire [1:0] axi_bvalid_i, output wire [1:0] axi_bready_o,
    output wire [63:0] axi_araddr_o, output wire [15:0] axi_arlen_o,
    output wire [1:0] axi_arvalid_o, input wire [1:0] axi_arready_i,
    input wire [255:0] axi_rdata_i, input wire [1:0] axi_rlast_i,
    input wire [3:0] axi_rresp_i, input wire [1:0] axi_rvalid_i, output wire [1:0] axi_rready_o
);
    localparam FRAME_GROUPS=1920*1200/4;
    reg [20:0] pixel_count;
    // DDR calibration may finish halfway through an already running sensor
    // frame. Observe a real vertical blank before accepting the next frame;
    // otherwise the capture FIFO receives a truncated first image and latches
    // a permanent frame-length error at the following SOP.
    reg camera_frame_armed;
    reg frame_vsync_delayed;
    wire sensor_group=frame_vsync_i && frame_href_i && pixel_valid_i;
    wire sensor_frame_end=frame_vsync_delayed && !frame_vsync_i;
    wire sensor_frame_bad=sensor_frame_end && camera_frame_armed && pixel_count!=FRAME_GROUPS;
    // Reset only the image frontend after a malformed raw frame. DDR jobs
    // already owning complete slots continue independently.
    reg frontend_rst_n;
    always @(posedge camera_clk_i or negedge camera_rst_n_i) begin
        if(!camera_rst_n_i) begin frame_vsync_delayed<=0; frontend_rst_n<=0; end
        else begin frame_vsync_delayed<=frame_vsync_i; frontend_rst_n<=!sensor_frame_bad; end
    end
    wire camera_valid=camera_frame_armed && frame_vsync_i && frame_href_i && pixel_valid_i && pixel_count<FRAME_GROUPS;
    always @(posedge camera_clk_i or negedge camera_rst_n_i) begin
        if(!camera_rst_n_i) begin
            pixel_count<=0;
            camera_frame_armed<=0;
        end else if(!frame_vsync_i) begin
            pixel_count<=0;
            camera_frame_armed<=1;
        end
        else if(camera_frame_armed && sensor_group && pixel_count<=FRAME_GROUPS) pixel_count<=pixel_count+1'b1;
    end
    wire candidate_valid,candidate_sop,candidate_eop,frontend_error;
    wire [7:0] candidate;
    // Sample mode once at the first accepted sensor group. A raw frame is
    // deferred if the previous Canny frontend is still flushing its border.
    (* async_reg="true" *) reg [2:0] raw_request_sync;
    reg frame_is_raw,raw_frame_admitted,frontend_busy;
    reg frontend_frame_admitted;
    wire capture_admit;wire [1:0] captured_compare;
    (* async_reg="true" *) reg [5:0] compare_request_sync;
    reg [1:0] frame_is_compare,frontend_frame_compare;reg compare_frame_admitted;
    reg [1:0] pending_compare,job_compare,workspace_compare;
    wire first_group=camera_valid && pixel_count==0;
    wire [1:0] compare_mode_for_frame=SUPPORT_COMPARE ? (first_group ? compare_request_sync[5:4] : frame_is_compare) : 2'b00;
    wire compare_for_frame=|compare_mode_for_frame;
    wire compare_ingress=first_group ? (compare_for_frame && capture_admit && !frontend_busy) : compare_frame_admitted;
    wire raw_for_frame=!compare_for_frame && (first_group ? raw_request_sync[2] : frame_is_raw);
    wire raw_ingress=!compare_for_frame && (first_group ? (raw_request_sync[2] && !frontend_busy) : raw_frame_admitted);
    // The sensor clock may stop in vertical blank before the border flush
    // ends. Reject that entire next frame instead of overlapping two frames;
    // a later grant must never start the frontend halfway down the image.
    wire frontend_admit=first_group ? !frontend_busy : frontend_frame_admitted;
    wire frontend_valid=camera_valid && frontend_admit && !raw_for_frame && (!compare_for_frame || compare_ingress);
    wire capture_input_valid=raw_ingress ? camera_valid : candidate_valid;
    wire capture_input_sop=raw_ingress ? pixel_count==0 : candidate_sop;
    wire capture_input_eop=raw_ingress ? pixel_count==FRAME_GROUPS-1 : candidate_eop;
    always @(posedge camera_clk_i or negedge camera_rst_n_i) begin
        if(!camera_rst_n_i) begin frontend_frame_admitted<=0;compare_request_sync<=0;frame_is_compare<=0;compare_frame_admitted<=0;frontend_frame_compare<=0;raw_request_sync<=0; frame_is_raw<=0; raw_frame_admitted<=0; frontend_busy<=0; end
        else begin
            raw_request_sync<={raw_request_sync[1:0],raw_request_i};
            compare_request_sync<={compare_request_sync[3:0],compare_request_i};
            if(first_group) frontend_frame_admitted<=!frontend_busy;
            else if(!frame_vsync_i) frontend_frame_admitted<=0;
            if(first_group) begin frame_is_compare<=SUPPORT_COMPARE ? compare_request_sync[5:4] : 2'b00;compare_frame_admitted<=compare_ingress;end
            else if(!frame_vsync_i) compare_frame_admitted<=0;
            if(first_group && frontend_valid) frontend_frame_compare<=compare_mode_for_frame;
            if(first_group) begin frame_is_raw<=raw_request_sync[2]; raw_frame_admitted<=raw_request_sync[2] && !frontend_busy; end
            else if(!frame_vsync_i) raw_frame_admitted<=0;
            if(!frontend_rst_n) frontend_busy<=0;
            else begin
                if(frontend_valid && pixel_count==0) frontend_busy<=1;
                if(candidate_valid && candidate_eop) frontend_busy<=0;
            end
        end
    end
    canny_frontend #(.LOW_THRESHOLD(11'd10),.HIGH_THRESHOLD(11'd20),.RUNTIME_THRESHOLDS(1)) frontend (
        .low_threshold_i(low_threshold_i),.high_threshold_i(high_threshold_i),
        .clk_i(camera_clk_i),.rst_n_i(frontend_rst_n),.valid_i(frontend_valid),
        .sop_i(camera_valid && pixel_count==0),.eop_i(camera_valid && pixel_count==FRAME_GROUPS-1),.data_i(gray_i),
        .valid_o(candidate_valid),.sop_o(candidate_sop),.eop_o(candidate_eop),.state_o(candidate),.overlap_error_o(frontend_error)
    );
    wire captured_valid,capture_ready,capture_error,capture_dropped,captured_raw;
    reg pending_raw,job_raw,workspace_gray;
    wire captured_background_tag;
    reg background_request_tag,background_saved_tag,background_valid;
    reg [31:0] background_address,background_epoch,background_frames,background_cycles,job_cycles;
    reg [1:0] background_slot,job_old_background_slot;
    reg pending_background_tag,job_background,job_pin_background,job_had_background;
    reg [31:0] job_background_address;
    (* async_reg="true" *) reg [2:0] background_tag_camera;
    reg workspace_background;
    wire pin_pending_frame=background_request_i && pending_raw && pending_background_tag==background_request_tag &&
        (!background_valid || background_saved_tag!=background_request_tag);
    assign background_valid_o=background_valid;
    assign background_pending_o=background_request_tag!=background_saved_tag;
    assign background_epoch_o=background_epoch;
    assign background_frames_o=background_frames;
    assign background_cycles_o=background_cycles;
    // 重新采集请求在摄像头SOP处锁存，禁止把请求之前的排队帧当作新背景。
    always @(posedge camera_clk_i or negedge camera_rst_n_i) begin
        if(!camera_rst_n_i) background_tag_camera<=0;
        else background_tag_camera<={background_tag_camera[1:0],background_request_tag};
    end
    wire [1:0] captured_slot;
    wire [31:0] captured_address;
    reg pending;
    reg [1:0] pending_slot,active_slot;
    reg [31:0] pending_address,job_address;
    reg worker_busy,job_toggle;
    reg done_toggle;
    (* async_reg="true" *) reg [2:0] done_sync;
    reg done_seen;
    wire release_slot=done_sync[2]!=done_seen;
    assign capture_ready=!pending;
    canny_capture #(.SUPPORT_COMPARE(SUPPORT_COMPARE),.PAD_ROWS(1),.SUPPORT_GRAY(1),.SLOT_BYTES(32'h00400000),.SLOTS(4)) capture (
        .compare_i(raw_ingress ? 2'b00 : frontend_frame_compare),.compare_start_i(first_group && compare_ingress),
        .compare_valid_i(camera_valid && compare_ingress),.compare_eop_i(camera_valid && pixel_count==FRAME_GROUPS-1),
        .admit_ready_o(capture_admit),.completed_compare_o(captured_compare),
        .camera_clk_i(camera_clk_i),.camera_rst_n_i(camera_rst_n_i),.axi_clk_i(axi_clk_i),.axi_rst_n_i(axi_rst_n_i),
        .valid_i(capture_input_valid),.sop_i(capture_input_sop),.eop_i(capture_input_eop),.state_i(candidate),
        .raw_i(raw_ingress),.raw_data_i(gray_i),.completed_raw_o(captured_raw),
        .background_tag_i(background_tag_camera[2]),.completed_background_tag_o(captured_background_tag),
        .completed_valid_o(captured_valid),.completed_ready_i(capture_ready),.completed_slot_o(captured_slot),.completed_address_o(captured_address),
        .release_i(release_slot && (!job_pin_background || job_had_background)),
        .release_slot_i(job_pin_background ? job_old_background_slot : active_slot),.error_o(capture_error),
        .frame_check_i(sensor_frame_end),.abort_frame_i(sensor_frame_bad),.dropped_frame_o(capture_dropped),
        .axi_awaddr_o(axi_awaddr_o[31:0]),.axi_awlen_o(axi_awlen_o[7:0]),.axi_awvalid_o(axi_awvalid_o[0]),.axi_awready_i(axi_awready_i[0]),
        .axi_wdata_o(axi_wdata_o[127:0]),.axi_wstrb_o(axi_wstrb_o[15:0]),.axi_wlast_o(axi_wlast_o[0]),
        .axi_wvalid_o(axi_wvalid_o[0]),.axi_wready_i(axi_wready_i[0]),.axi_bresp_i(axi_bresp_i[1:0]),.axi_bvalid_i(axi_bvalid_i[0]),.axi_bready_o(axi_bready_o[0])
    );
    always @(posedge axi_clk_i or negedge axi_rst_n_i) begin
        if(!axi_rst_n_i) begin
            pending_compare<=0;job_compare<=0;pending<=0; pending_raw<=0; job_raw<=0; pending_slot<=0; pending_address<=0; active_slot<=0;
            job_address<=0; worker_busy<=0; job_toggle<=0; done_sync<=0; done_seen<=0;
            background_request_tag<=0; background_saved_tag<=0; background_valid<=0;
            background_address<=0; background_epoch<=0; background_slot<=0;
            pending_background_tag<=0; job_background<=0; job_pin_background<=0;
            job_had_background<=0; job_old_background_slot<=0; job_background_address<=0;
        end else begin
            if(background_relearn_i && !background_pending_o) background_request_tag<=!background_request_tag;
            done_sync<={done_sync[1:0],done_toggle};
            if(release_slot) begin done_seen<=done_sync[2]; worker_busy<=0; end
            if(captured_valid && capture_ready) begin
                pending<=1;pending_compare<=captured_compare; pending_raw<=captured_raw; pending_slot<=captured_slot;
                pending_address<=captured_address; pending_background_tag<=captured_background_tag;
            end
            if(!worker_busy && pending) begin
                pending<=0;job_compare<=pending_compare; job_raw<=pending_raw; active_slot<=pending_slot; job_address<=pending_address;
                worker_busy<=1; job_toggle<=!job_toggle;
                job_background<=background_request_i && pending_raw && (background_valid || pin_pending_frame);
                job_pin_background<=pin_pending_frame;
                job_had_background<=background_valid; job_old_background_slot<=background_slot;
                job_background_address<=pin_pending_frame ? pending_address : background_address;
                if(pin_pending_frame) begin
                    background_valid<=1; background_slot<=pending_slot; background_address<=pending_address;
                    background_saved_tag<=pending_background_tag; background_epoch<=background_epoch+1'b1;
                end
            end
        end
    end
    (* async_reg="true" *) reg [2:0] job_sync;
    reg job_seen;
    reg [1:0] processing_stage;
    reg hysteresis_start,render_start;
    reg background_start;
    wire background_done,background_read_request,background_read_ready;
    wire [31:0] background_read_address;
    wire [7:0] background_boxes;
    wire [87:0] background_left,background_top,background_right,background_bottom;
    wire [3:0] background_work_count;
    wire background_work_overflow;
    reg [3:0] background_published_count;
    reg background_published_overflow;
    assign background_count_o=background_published_count;
    assign background_overflow_o=background_published_overflow;
    reg [31:0] workspace_address;
    wire hysteresis_done,render_done;
    wire hysteresis_read_request,hysteresis_write_request;
    wire [31:0] hysteresis_read_address,hysteresis_write_address;
    wire hysteresis_read_ready,hysteresis_write_valid,hysteresis_write_last;
    wire [63:0] hysteresis_write_data;
    wire render_read_request,render_read_ready;
    wire [31:0] render_read_address;
    wire row_read_request_ready,row_read_valid,row_write_request_ready,row_write_ready,row_write_done,row_error;
    wire [63:0] row_read_data;
    wire render_frame,render_valid;
    (* async_reg="true" *) reg [1:0] capture_fault_sync;
    reg camera_fault;
    (* async_reg="true" *) reg [1:0] frontend_fault_sync;
    always @(posedge axi_clk_i or negedge axi_rst_n_i) begin
        if(!axi_rst_n_i) begin capture_fault_sync<=0; frontend_fault_sync<=0; camera_fault<=0; end
        else begin
            capture_fault_sync<={capture_fault_sync[0],capture_error};
            frontend_fault_sync<={frontend_fault_sync[0],frontend_error};
            if(frontend_fault_sync[1] || row_error || capture_fault_sync[1]) camera_fault<=1;
        end
    end
    always @(posedge axi_clk_i or negedge axi_rst_n_i) begin
        if(!axi_rst_n_i) begin
            job_sync<=0; job_seen<=0; processing_stage<=0; hysteresis_start<=0;
            render_start<=0; workspace_address<=0; workspace_gray<=0; done_toggle<=0;
            background_start<=0; workspace_background<=0;workspace_compare<=0;
            background_frames<=0; background_cycles<=0; job_cycles<=0;
            background_published_count<=0; background_published_overflow<=0;
        end else begin
            job_sync<={job_sync[1:0],job_toggle};
            hysteresis_start<=0; render_start<=0; background_start<=0;
            if(processing_stage!=0) job_cycles<=job_cycles+1'b1;
            if(processing_stage==0 && job_sync[2]!=job_seen && !camera_fault) begin
                job_seen<=job_sync[2]; workspace_address<=job_address; workspace_gray<=job_raw;
                workspace_background<=job_background;workspace_compare<=job_compare; job_cycles<=0;
                if(job_background) begin background_start<=1; processing_stage<=3; end
                else if(job_raw) begin render_start<=1; processing_stage<=2; end
                else begin hysteresis_start<=1; processing_stage<=1; end
            end
            if(processing_stage==1 && hysteresis_done && !camera_fault && !row_error) begin render_start<=1; processing_stage<=2; end
            if(processing_stage==3 && background_done && !camera_fault && !row_error) begin render_start<=1; processing_stage<=2; end
            if(processing_stage==2 && render_done) begin
                processing_stage<=0; done_toggle<=!done_toggle;
                if(workspace_background) begin
                    background_frames<=background_frames+1'b1; background_cycles<=job_cycles+1'b1;
                    background_published_count<=background_work_count; background_published_overflow<=background_work_overflow;
                end
            end
        end
    end
    wire [10:0] feature_row;
    background_frame_worker background_worker (
        .background_mask_capture_i(background_mask_capture_i),
        .background_mask_address_i(background_mask_address_i),.background_mask_data_o(background_mask_data_o),
        .background_mask_ready_o(background_mask_ready_o),.background_mask_busy_o(background_mask_busy_o),
        .background_mask_epoch_o(background_mask_epoch_o),

        .i_clk(axi_clk_i),.i_rst_n(axi_rst_n_i),.i_start(background_start),.i_threshold(background_threshold_i),
        .i_current_address(workspace_address),.i_background_address(job_background_address),.o_done(background_done),
        .o_read_request(background_read_request),.i_read_request_ready(row_read_request_ready && processing_stage==3),
        .o_read_address(background_read_address),.i_read_valid(row_read_valid && processing_stage==3),
        .o_read_ready(background_read_ready),.i_read_data(row_read_data),.o_box_valid(background_boxes),
        .o_left(background_left),.o_top(background_top),.o_right(background_right),.o_bottom(background_bottom),
        .o_count(background_work_count),.o_overflow(background_work_overflow)
    );
    wire [5:0] feature_word;
    wire feature_first;
    canny_hysteresis #(.ROW_STRIDE(512)) hysteresis (
        .clk_i(axi_clk_i),.rst_n_i(axi_rst_n_i),.start_i(hysteresis_start),.base_address_i(workspace_address),
        .feature_row_o(feature_row),.feature_word_o(feature_word),.feature_first_o(feature_first),
        .busy_o(),.done_o(hysteresis_done),.cycles_o(last_cycles_o),.sweeps_o(last_sweeps_o),
        .read_request_o(hysteresis_read_request),.read_request_ready_i(row_read_request_ready && processing_stage==1),.read_address_o(hysteresis_read_address),
        .read_valid_i(row_read_valid && processing_stage==1),.read_ready_o(hysteresis_read_ready),.read_data_i(row_read_data),
        .write_request_o(hysteresis_write_request),.write_request_ready_i(row_write_request_ready),.write_address_o(hysteresis_write_address),
        .write_valid_o(hysteresis_write_valid),.write_ready_i(row_write_ready),.write_data_o(hysteresis_write_data),.write_last_o(hysteresis_write_last),.write_done_i(row_write_done)
    );
    wire raw_color=SUPPORT_RGB && workspace_gray;
    wire mono_done,mono_request,mono_ready,mono_frame,mono_valid,mono_compare_left;
    wire rgb_done,rgb_request,rgb_ready,rgb_frame,rgb_valid;
    wire [31:0] mono_address,rgb_address;
    wire [63:0] mono_data,rgb_data;
    wire select_rgb_request=workspace_compare ? (!mono_request && rgb_request) : raw_color;
    reg row_owner_rgb,mono_completed,rgb_completed;
    wire select_rgb_data=workspace_compare ? row_owner_rgb : raw_color;
    wire mono_output_ready=display_ready_i && !camera_fault && (workspace_compare ? rgb_valid : !raw_color);
    wire rgb_output_ready=display_ready_i && !camera_fault && (workspace_compare ? mono_valid : raw_color);
    always @(posedge axi_clk_i or negedge axi_rst_n_i) begin
        if(!axi_rst_n_i) begin row_owner_rgb<=0;mono_completed<=0;rgb_completed<=0;end
        else begin
            if(processing_stage==2 && render_read_request && row_read_request_ready) row_owner_rgb<=select_rgb_request;
            if(render_start || processing_stage!=2) begin mono_completed<=0;rgb_completed<=0;end
            else begin
                if(mono_done) mono_completed<=1;
                if(rgb_done) rgb_completed<=1;
            end
        end
    end
    assign render_done=workspace_compare ? (mono_completed && rgb_completed) : (raw_color ? rgb_done : mono_done);
    assign render_read_request=select_rgb_request ? rgb_request : mono_request;
    assign render_read_ready=select_rgb_data ? rgb_ready : mono_ready;
    assign render_read_address=select_rgb_request ? rgb_address : mono_address;
    assign render_frame=workspace_compare ? mono_frame : (raw_color ? rgb_frame : mono_frame);
    assign render_valid=workspace_compare ? (mono_valid && rgb_valid) : (raw_color ? rgb_valid : mono_valid);
    wire [63:0] paired_pixels;
    genvar overlay_lane;
    generate for(overlay_lane=0;overlay_lane<4;overlay_lane=overlay_lane+1) begin:yellow_edges
        wire [15:0] edge_pixel=mono_data[overlay_lane*16+:16];
        wire [15:0] source_pixel=rgb_data[overlay_lane*16+:16];
        assign paired_pixels[overlay_lane*16+:16]=workspace_compare[1] ?
            (edge_pixel!=0 ? 16'hffe0 : source_pixel) :
            (mono_compare_left ? source_pixel : edge_pixel);
    end endgenerate
    assign display_data_o=workspace_compare ? paired_pixels : (raw_color ? rgb_data : mono_data);
    canny_rectangle_render #(.MAX_OBJECTS(MAX_OBJECTS),.ENABLE_EXTENDED_SHAPES(ENABLE_EXTENDED_SHAPES),.SUPPORT_COMPARE(SUPPORT_COMPARE),.SUPPORT_RECTANGLE(SUPPORT_RECTANGLE),.SHARED_FONT(SHARED_FONT),.SUPPORT_GRAY(!SUPPORT_RGB),.DISPLAY_WIDTH(DISPLAY_WIDTH),.DISPLAY_HEIGHT(DISPLAY_HEIGHT),.SCALE_NUM(SCALE_NUM),.SCALE_DEN(SCALE_DEN),.ROW_STRIDE(512)) render (
        .font_address_o(font_address_o),.font_enable_o(font_enable_o),.font_pixels_i(font_pixels_i),
        .compare_i(|workspace_compare),.compare_left_o(mono_compare_left),
        .rectangle_i(rectangle_request_i && !workspace_compare),
        .feature_start_i(hysteresis_start),
        .feature_valid_i(hysteresis_write_valid && row_write_ready),
        .feature_data_i(hysteresis_write_data),.feature_row_i(feature_row),
        .feature_word_i(feature_word),.feature_first_i(feature_first),
        .gray_i(workspace_gray),.clk_i(axi_clk_i),.rst_n_i(axi_rst_n_i),.start_i(render_start && !raw_color),.base_address_i(workspace_address),.busy_o(),.done_o(mono_done),
        .read_request_o(mono_request),.read_request_ready_i(row_read_request_ready && processing_stage==2 && !select_rgb_request),.read_address_o(mono_address),
        .read_valid_i(row_read_valid && processing_stage==2 && !select_rgb_data),.read_ready_o(mono_ready),.read_data_i(row_read_data),
        .frame_active_o(mono_frame),.valid_o(mono_valid),.ready_i(mono_output_ready),.sop_o(),.eop_o(),.rgb565_o(mono_data)
    );
    generate if(SUPPORT_RGB) begin: original_color
        canny_raw_rgb_render #(.SUPPORT_COMPARE(SUPPORT_COMPARE),.DISPLAY_WIDTH(DISPLAY_WIDTH),.DISPLAY_HEIGHT(DISPLAY_HEIGHT),.SCALE_NUM(SCALE_NUM),.SCALE_DEN(SCALE_DEN)) renderer (
            .compare_i(|workspace_compare),
            .background_enable_i(workspace_background),.background_boxes_i(background_boxes),
            .background_left_i(background_left),.background_top_i(background_top),
            .background_right_i(background_right),.background_bottom_i(background_bottom),
            .clk_i(axi_clk_i),.rst_n_i(axi_rst_n_i),.start_i(render_start && (raw_color || workspace_compare)),.base_address_i(workspace_address+(workspace_compare ? 32'h00100000 : 32'd0)),.done_o(rgb_done),
            .read_request_o(rgb_request),.read_request_ready_i(row_read_request_ready && processing_stage==2 && select_rgb_request),.read_address_o(rgb_address),
            .read_valid_i(row_read_valid && processing_stage==2 && select_rgb_data),.read_ready_o(rgb_ready),.read_data_i(row_read_data),
            .frame_active_o(rgb_frame),.valid_o(rgb_valid),.ready_i(rgb_output_ready),.rgb565_o(rgb_data)
        );
    end else begin: original_gray
        assign rgb_done=0; assign rgb_request=0; assign rgb_ready=0;
        assign rgb_address=0; assign rgb_frame=0; assign rgb_valid=0; assign rgb_data=0;
    end endgenerate
    canny_row_dma #(.PADDING_BEATS(2),.SUPPORT_GRAY(1)) row_dma (
        .camera_clk_i(axi_clk_i),.camera_rst_n_i(axi_rst_n_i),.axi_clk_i(axi_clk_i),.axi_rst_n_i(axi_rst_n_i),
        .read_gray_i((processing_stage==2 && (workspace_compare ? select_rgb_request : workspace_gray)) || processing_stage==3),
        .read_request_i(processing_stage==3 ? background_read_request : (processing_stage==1 ? hysteresis_read_request : render_read_request)),.read_request_ready_o(row_read_request_ready),
        .read_address_i(processing_stage==3 ? background_read_address : (processing_stage==1 ? hysteresis_read_address : render_read_address)),
        .read_valid_o(row_read_valid),.read_ready_i(processing_stage==3 ? background_read_ready : (processing_stage==1 ? hysteresis_read_ready : render_read_ready)),.read_data_o(row_read_data),
        .write_request_i(hysteresis_write_request),.write_request_ready_o(row_write_request_ready),.write_address_i(hysteresis_write_address),
        .write_valid_i(hysteresis_write_valid),.write_ready_o(row_write_ready),.write_data_i(hysteresis_write_data),.write_last_i(hysteresis_write_last),.write_done_o(row_write_done),.error_o(row_error),
        .axi_awaddr_o(axi_awaddr_o[63:32]),.axi_awlen_o(axi_awlen_o[15:8]),.axi_awvalid_o(axi_awvalid_o[1]),.axi_awready_i(axi_awready_i[1]),
        .axi_wdata_o(axi_wdata_o[255:128]),.axi_wstrb_o(axi_wstrb_o[31:16]),.axi_wlast_o(axi_wlast_o[1]),.axi_wvalid_o(axi_wvalid_o[1]),.axi_wready_i(axi_wready_i[1]),
        .axi_bresp_i(axi_bresp_i[3:2]),.axi_bvalid_i(axi_bvalid_i[1]),.axi_bready_o(axi_bready_o[1]),
        .axi_araddr_o(axi_araddr_o[63:32]),.axi_arlen_o(axi_arlen_o[15:8]),.axi_arvalid_o(axi_arvalid_o[1]),.axi_arready_i(axi_arready_i[1]),
        .axi_rdata_i(axi_rdata_i[255:128]),.axi_rlast_i(axi_rlast_i[1]),.axi_rresp_i(axi_rresp_i[3:2]),.axi_rvalid_i(axi_rvalid_i[1]),.axi_rready_o(axi_rready_o[1])
    );
    assign axi_araddr_o[31:0]=0;
    assign axi_arlen_o[7:0]=0;
    assign axi_arvalid_o[0]=0;
    assign axi_rready_o[0]=1;
    assign display_gray_o=workspace_gray || workspace_compare;
    assign display_frame_o=render_frame && !camera_fault;
    assign display_valid_o=render_valid && !camera_fault && !row_error;
    assign error_o=camera_fault;
    // Sticky milestones survive a later fault. Each bit is synchronized
    // independently by the display monitor; none controls the algorithm.
    reg [5:0] camera_milestones;
    reg capture_seen;
    always @(posedge camera_clk_i or negedge camera_rst_n_i) begin
        if(!camera_rst_n_i) camera_milestones[1:0]<=0;
        else begin
            if(camera_valid && pixel_count==FRAME_GROUPS-1) camera_milestones[0]<=1;
            if(candidate_valid && candidate_eop) camera_milestones[1]<=1;
        end
    end
    always @(posedge axi_clk_i or negedge axi_rst_n_i) begin
        if(!axi_rst_n_i) camera_milestones[5:2] <= 4'b0;
        else begin
            if(hysteresis_start) camera_milestones[2] <= 1'b1;
            if(hysteresis_done) camera_milestones[3] <= 1'b1;
            if(render_done) camera_milestones[4] <= 1'b1;
            if(display_valid_o && display_ready_i) camera_milestones[5] <= 1'b1;
        end
    end
    always @(posedge axi_clk_i or negedge axi_rst_n_i) begin
        if(!axi_rst_n_i) capture_seen <= 1'b0;
        else if(captured_valid && capture_ready) capture_seen <= 1'b1;
    end
    assign debug_status_o = {processing_stage==2,processing_stage==1,
        camera_fault,row_error,(capture_error || capture_dropped),frontend_error,camera_frame_armed,
        camera_milestones[5:2],capture_seen,camera_milestones[1:0]};
endmodule
