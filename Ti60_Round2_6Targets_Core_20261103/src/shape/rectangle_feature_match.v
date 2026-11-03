`timescale 1ns / 1ps
// 矩形特征匹配：缩小边缘图、八连通候选、旋转边界模板及拒识。
// Rectangle feature matcher. All storage and processing use the camera clock.
// The destructive map belongs to one complete Canny job. The caller retains
// its DDR workspace until analysis and same-frame annotated rendering complete.
module rectangle_feature_match #(
    parameter MAX_OBJECTS=4, parameter LABEL_BITS=MAX_OBJECTS>4 ? 4 : 3,
    parameter WRITE_TAP = 0,
    parameter ENABLE_SHAPES = 0,
    parameter ENABLE_DIGITS = 0,
    parameter ENABLE_EXTENDED_SHAPES = 0,
    parameter KIND_BITS = ENABLE_EXTENDED_SHAPES ? 4 : (ENABLE_DIGITS ? 3 : 2),
    parameter MAX_ANALYSIS_CYCLES = 960000,
    parameter INPUT_WIDTH = 1920,
    parameter INPUT_HEIGHT = 1080,
    parameter POINT_LIMIT = 2048
) (
    input wire label_enable_i,input wire [13:0] label_address_i,
    output wire [4*LABEL_BITS-1:0] label_data_o,
    input wire clk_i,
    input wire rst_n_i,
    input wire start_i,
    input wire analyze_i,
    input wire tap_valid_i,
    input wire [63:0] tap_data_i,
    input wire [10:0] tap_row_i,
    input wire [5:0] tap_word_i,
    input wire tap_first_i,
    input wire valid_i,
    input wire [63:0] pixels_i,
    input wire eop_i,
    output wire ready_o,
    output wire done_o,
    output wire found_o,
    output wire overflow_o,
    output wire [10:0] left_o,
    output wire [10:0] top_o,
    output wire [10:0] width_o,
    output wire [10:0] height_o,
    output wire [21:0] area_o,
    output wire [3:0] angle_o,
    output wire [2:0] object_count_o,
    output wire [MAX_OBJECTS*KIND_BITS-1:0] object_kind_o,
    output wire [MAX_OBJECTS*11-1:0] object_left_o, object_top_o, object_width_o, object_height_o,
    output wire objects_full_o,
    output wire [31:0] cycles_o
);
    localparam GRID_WIDTH = INPUT_WIDTH / 8;
    localparam GRID_HEIGHT = INPUT_HEIGHT / 8;
    localparam GRID_SIZE = GRID_WIDTH * GRID_HEIGHT;
    localparam ANALYSIS_BITS = $clog2(MAX_ANALYSIS_CYCLES+1);
    localparam ST_IDLE=0, ST_CAPTURE=1, ST_SCAN_REQ=2, ST_SCAN_WAIT=3;
    localparam ST_SCAN_TEST=4, ST_SEED=5, ST_POP_REQ=6, ST_POP_WAIT=7;
    localparam ST_POP_LOAD=8, ST_NEIGHBOR=9, ST_NEIGHBOR_WAIT=10;
    localparam ST_NEIGHBOR_TEST=11, ST_NEXT_NEIGHBOR=12, ST_COMPONENT=13;
    localparam ST_ANGLE=14, ST_POINT_REQ=15, ST_POINT_WAIT=16;
    localparam ST_MULTIPLY=17, ST_PROJECT=18, ST_ACCUMULATE=19;
    localparam ST_RANGE=20, ST_SCORE_START=21, ST_DECIDE=22;
    localparam ST_NEXT_ANGLE=23, ST_NEXT_SCAN=24, ST_FINISH=25;
    localparam ST_HIST_CLEAR=26, ST_HIST_WRITE=27, ST_HIST_READ=28;
    localparam ST_HIST_WAIT=29, ST_HIST_SCAN=30, ST_TRIM_RANGE=31;
    localparam ST_FIT_BOUNDS=32, ST_FIT_AREA=33, ST_FIT_RESULT=34;
    localparam ST_SHAPE_START=35, ST_SHAPE_WAIT=36;
    localparam ST_BRIDGE_REQ=37, ST_BRIDGE_WAIT=38, ST_BRIDGE_TEST=39, ST_BRIDGE_DECIDE=40;
    localparam ST_DIGIT_START=41,ST_DIGIT_WAIT=42;
    localparam ST_LABEL_REQUEST=43,ST_LABEL_WAIT=44,ST_LABEL_LOAD=45,ST_LABEL_READ=46,ST_LABEL_WRITE=47;
    reg [15:0] label_address;
    reg [LABEL_BITS-1:0] component_label;

    reg [5:0] state, next_state;
    wire labeling=state>=ST_LABEL_REQUEST && state<=ST_LABEL_WRITE;
    wire digit_stage=state==ST_DIGIT_START || state==ST_DIGIT_WAIT;
    wire [10:0] digit_address;
    wire digit_done;
    wire [KIND_BITS-1:0] digit_kind;
    reg circle_probe;
    wire shape_done;
    wire [1:0] shape_kind;
    reg [3:0] digit_attempts;
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin digit_attempts<=0;circle_probe<=0;end
        else if(start_i) begin digit_attempts<=0;circle_probe<=0;end
        else begin
            if(state==ST_COMPONENT) circle_probe<=0;
            if(ENABLE_EXTENDED_SHAPES && state==ST_SHAPE_WAIT && shape_done && shape_kind==2) circle_probe<=1;
            if(state==ST_DIGIT_START && !circle_probe) digit_attempts<=digit_attempts+1'b1;
        end
    end
    // Keep immutable adjacency during destructive flood fill so a visited edge
    // cannot masquerade as an endpoint. This RAM is eliminated in legacy mode.
    // 0 empty; 1 unvisited; 6 visited/unclassified; 2..5,7,8 six independent slots.
    wire [3:0] original_word;
    wire [4*LABEL_BITS-1:0] read_codes,selected_codes;
    reg [4*LABEL_BITS-1:0] neighbor_codes;
    reg [4*LABEL_BITS-1:0] write_codes;
    wire [3:0] write_low,write_middle,write_high;
    assign label_data_o=read_codes;
    reg [1:0] original_lane;
    wire original_edge=original_word[original_lane];
    reg [7:0] source_neighbors, bridge_neighbors;
    reg [2:0] bridge_probe;
    reg signed [9:0] probe_x, probe_y;
    reg [15:0] probe_address;
    reg probe_inside;
    reg [7:0] source_direction_mask, bridge_direction_mask;
    reg source_endpoint, bridge_endpoint;
    wire source_bridge_ok=source_endpoint && (source_neighbors & ~source_direction_mask)==0;
    wire bridge_accept=bridge_endpoint && (bridge_neighbors & ~bridge_direction_mask)==0;
    wire probing=state==ST_BRIDGE_REQ || state==ST_BRIDGE_WAIT || state==ST_BRIDGE_TEST;
    wire bridge_state=probing || state==ST_BRIDGE_DECIDE;
    assign selected_codes=bridge_state ? neighbor_codes : read_codes;
    reg [2:0] object_count;
    reg [MAX_OBJECTS*KIND_BITS-1:0] object_kind;
    reg [MAX_OBJECTS*11-1:0] object_left,object_top,object_width,object_height;
    reg objects_full;
    reg [KIND_BITS-1:0] candidate_kind;
    // Keep all legacy geometry when the four visible slots are exhausted.
    // An evicted digit's component labels are cleared before reusing its slot.
    localparam ST_EVICT_READ=48,ST_EVICT_WAIT=49,ST_EVICT_WRITE=50;
    wire evicting=state>=ST_EVICT_READ && state<=ST_EVICT_WRITE;
    reg [13:0] evict_address;
    reg replace_available;
    reg [2:0] replace_slot;
    integer search_slot;
    always @(*) begin
        replace_available=0;replace_slot=0;
        for(search_slot=MAX_OBJECTS-1;search_slot>=0;search_slot=search_slot-1) begin
            if(ENABLE_DIGITS && object_kind[search_slot*KIND_BITS+:KIND_BITS]>=4 && object_kind[search_slot*KIND_BITS+:KIND_BITS]<=6) begin
                replace_available=1;replace_slot=search_slot;
            end
        end
    end
    wire replace_digit=object_count==MAX_OBJECTS && (candidate_kind<=3 || candidate_kind>=7) && replace_available;
    wire accept_object=object_count<MAX_OBJECTS || replace_digit;
    wire [2:0] result_slot=object_count<MAX_OBJECTS ? object_count : replace_slot;
    wire extension_result=candidate_kind>3 && (state==ST_FIT_AREA || state==ST_FIT_RESULT);
    reg [7:0] candidate_left,candidate_top;
    reg [8:0] candidate_width,candidate_height;
    wire [10:0] shape_address;

    // 一位图是唯一的缩小帧缓存；访问前由完整输入帧覆盖初始化。
    wire [3:0] map_word;
    reg [1:0] map_lane;
    wire map_read=map_word[map_lane];
    reg [15:0] map_address;
    reg map_write;
    reg map_write_value;
    // 队列同时保存整个候选点集，匹配阶段重复读取，不重新访问 DDR。
    (* ram_style="block" *) reg [15:0] point_memory [0:POINT_LIMIT-1];
    reg [15:0] point_read;
    reg [10:0] point_address;
    reg [15:0] queue_head, queue_tail;
    reg [11:0] point_index;
    reg [15:0] enqueue_point;
    reg enqueue;

    reg [10:0] capture_group, capture_row;
    reg [15:0] capture_address;
    reg half_edge;
    wire group_edge = pixels_i[63] | pixels_i[47] | pixels_i[31] | pixels_i[15];
    wire accepted = valid_i && state==ST_CAPTURE;
    reg [15:0] scan_address, neighbor_address;
    reg [7:0] scan_x, scan_y, current_x, current_y;
    reg [7:0] neighbor_x, neighbor_y;
    reg neighbor_inside;
    reg [4:0] neighbor_index;
    reg [7:0] bound_left, bound_right, bound_top, bound_bottom;
    reg [8:0] bound_width, bound_height;
    reg [17:0] bound_area, best_area;
    reg found, overflow;
    reg [10:0] result_left, result_top, result_width, result_height;
    reg [21:0] result_area;
    reg [3:0] result_angle, template_angle;
    reg [31:0] elapsed, result_cycles;
    reg [ANALYSIS_BITS-1:0] analysis_cycles;
    wire analysis_active=state!=ST_IDLE && state!=ST_CAPTURE && state!=ST_FINISH;
    wire analysis_timeout=analysis_active && analysis_cycles>=MAX_ANALYSIS_CYCLES-1;

    reg [5:0] cosine, sine;
    reg [13:0] product_xc, product_ys, product_yc, product_xs;
    reg signed [15:0] project_u, project_v;
    reg signed [15:0] minimum_u, maximum_u, minimum_v, maximum_v;
    reg [15:0] span_u, span_v, tolerance_u, tolerance_v;
    reg scoring;
    reg [11:0] support_count, left_count, right_count, top_count, bottom_count;
    reg [3:0] corner_mask;
    reg [3:0] side_middle_mask;
    // A short nearby edge can join the contour through gap tolerance and move
    // its extrema. Retry the same angle using 1/16 and 15/16 projection ranks.
    // Two 512-bin memories retain one angle only; the analysis deadline stays fixed.
    (* ram_style="block" *) reg [11:0] histogram_u [0:511];
    (* ram_style="block" *) reg [11:0] histogram_v [0:511];
    reg [11:0] histogram_u_read, histogram_v_read;
    reg [8:0] histogram_index;
    reg [15:0] cumulative_u, cumulative_v;
    reg low_u_seen, high_u_seen, low_v_seen, high_v_seen;
    reg trimmed_fit;
    reg [7:0] fit_left, fit_right, fit_top, fit_bottom;
    wire histogram_scanning=state==ST_HIST_READ || state==ST_HIST_WAIT || state==ST_HIST_SCAN;
    wire [8:0] histogram_u_address=(state==ST_HIST_CLEAR || histogram_scanning) ? histogram_index : project_u[13:5];
    wire [8:0] histogram_v_address=(state==ST_HIST_CLEAR || histogram_scanning) ? histogram_index : (project_v[13:5]^9'h100);
    wire [15:0] cumulative_u_next=cumulative_u+histogram_u_read;
    wire [15:0] cumulative_v_next=cumulative_v+histogram_v_read;
    wire [15:0] trim_count=queue_tail>>4;
    wire [15:0] upper_rank=queue_tail-trim_count;
    wire signed [15:0] histogram_u_value=$signed({2'b00,histogram_index,5'b00000});
    wire signed [15:0] histogram_v_value=histogram_u_value-16'sd8192;
    wire signed [16:0] distance_left=$signed({project_u[15],project_u})-$signed({minimum_u[15],minimum_u});
    wire signed [16:0] distance_right=$signed({maximum_u[15],maximum_u})-$signed({project_u[15],project_u});
    wire signed [16:0] distance_top=$signed({project_v[15],project_v})-$signed({minimum_v[15],minimum_v});
    wire signed [16:0] distance_bottom=$signed({maximum_v[15],maximum_v})-$signed({project_v[15],project_v});
    wire signed [16:0] signed_tolerance_u=$signed({1'b0,tolerance_u});
    wire signed [16:0] signed_tolerance_v=$signed({1'b0,tolerance_v});
    wire inside_extended=distance_left>=-signed_tolerance_u && distance_right>=-signed_tolerance_u &&
        distance_top>=-signed_tolerance_v && distance_bottom>=-signed_tolerance_v;
    wire inside_core=distance_left>=0 && distance_right>=0 && distance_top>=0 && distance_bottom>=0;
    wire middle_u=(project_u-minimum_u)>$signed({2'b00,span_u[15:2]}) &&
        (maximum_u-project_u)>$signed({2'b00,span_u[15:2]});
    wire middle_v=(project_v-minimum_v)>$signed({2'b00,span_v[15:2]}) &&
        (maximum_v-project_v)>$signed({2'b00,span_v[15:2]});
    wire near_left = distance_left<=signed_tolerance_u && (!trimmed_fit || inside_extended);
    wire near_right = distance_right<=signed_tolerance_u && (!trimmed_fit || inside_extended);
    wire near_top = distance_top<=signed_tolerance_v && (!trimmed_fit || inside_extended);
    wire near_bottom = distance_bottom<=signed_tolerance_v && (!trimmed_fit || inside_extended);
    wire support_ok = {support_count,3'b000} >= ({3'b000,queue_tail}*7);
    wire sides_ok = {left_count,4'b0000} >= queue_tail &&
                    {right_count,4'b0000} >= queue_tail &&
                    {top_count,4'b0000} >= queue_tail &&
                    {bottom_count,4'b0000} >= queue_tail;
    // One weak/missing corner is allowed; four sides and boundary support remain required.
    wire corners_ok = corner_mask==4'b1111 || corner_mask==4'b1110 ||
        corner_mask==4'b1101 || corner_mask==4'b1011 || corner_mask==4'b0111;
    wire template_ok = support_ok && sides_ok && corners_ok && side_middle_mask==4'b1111;
    wire last_point = point_index+1'b1==queue_tail;
    wire full_queue = queue_tail-queue_head>=POINT_LIMIT-1;
    wire oversized = queue_tail>=POINT_LIMIT;
    reg frontier_fault;

    generate if(ENABLE_DIGITS) begin : numeric_fallback
        contour_digit_match #(.EXTENDED_SHAPES(ENABLE_EXTENDED_SHAPES),.KIND_BITS(KIND_BITS)) classifier (
            .geometry_only_i(circle_probe),
            .clk_i(clk_i),.rst_n_i(rst_n_i),.start_i(state==ST_DIGIT_START),.cancel_i(start_i || analysis_timeout),
            .count_i(queue_tail),.left_i(bound_left),.right_i(bound_right),.top_i(bound_top),.bottom_i(bound_bottom),
            .address_o(digit_address),.point_i(point_read),.done_o(digit_done),.kind_o(digit_kind));
    end else begin : no_digits
        assign digit_address=0;assign digit_done=0;assign digit_kind=0;
    end endgenerate
    generate if(ENABLE_SHAPES) begin : additional_shapes
        contour_shape_match classifier (
            .clk_i(clk_i),.rst_n_i(rst_n_i),.start_i(state==ST_SHAPE_START),
            .cancel_i(analysis_timeout || start_i),.count_i(queue_tail),
            .left_i(bound_left),.right_i(bound_right),.top_i(bound_top),.bottom_i(bound_bottom),
            .address_o(shape_address),.point_i(point_read),.done_o(shape_done),.kind_o(shape_kind)
        );
    end else begin : rectangle_only
        assign shape_address=0;
        assign shape_done=0;
        assign shape_kind=0;
    end endgenerate
    // Four independent same-frame results; no temporal reuse of missing objects.
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            object_count<=0;objects_full<=0;candidate_kind<=1;
            candidate_left<=0;candidate_top<=0;candidate_width<=0;candidate_height<=0;
        end else begin
            if(start_i || analysis_timeout) begin
                object_count<=0;objects_full<=0;
            end else if(state==ST_FIT_RESULT) begin
                if(accept_object) begin
                    if(object_count<MAX_OBJECTS) object_count<=object_count+1'b1;
                    else objects_full<=1;
                end else objects_full<=1;
            end
            if(state==ST_COMPONENT) begin
                candidate_kind<=1;candidate_left<=bound_left;candidate_top<=bound_top;
                candidate_width<={1'b0,bound_right}-{1'b0,bound_left}+1'b1;
                candidate_height<={1'b0,bound_bottom}-{1'b0,bound_top}+1'b1;
            end
            if(state==ST_SHAPE_WAIT && shape_done && shape_kind!=0) candidate_kind<=shape_kind;
            if(state==ST_DIGIT_WAIT && digit_done && digit_kind!=0) candidate_kind<=digit_kind;
        end
    end
    genvar stored_slot;
    generate for(stored_slot=0;stored_slot<MAX_OBJECTS;stored_slot=stored_slot+1) begin: result_registers
        always @(posedge clk_i or negedge rst_n_i) begin
            if(!rst_n_i) begin
                object_kind[stored_slot*KIND_BITS+:KIND_BITS]<=0;
                object_left[stored_slot*11+:11]<=0;object_top[stored_slot*11+:11]<=0;
                object_width[stored_slot*11+:11]<=0;object_height[stored_slot*11+:11]<=0;
            end else begin
                if(start_i || analysis_timeout) object_kind[stored_slot*KIND_BITS+:KIND_BITS]<=0;
                else if(state==ST_FIT_RESULT && accept_object && result_slot==stored_slot) object_kind[stored_slot*KIND_BITS+:KIND_BITS]<=candidate_kind;
                if(!(start_i || analysis_timeout) && state==ST_FIT_RESULT && accept_object && result_slot==stored_slot) begin
                    object_left[stored_slot*11+:11]<={candidate_left,3'b000};object_top[stored_slot*11+:11]<={candidate_top,3'b000};
                    object_width[stored_slot*11+:11]<=candidate_width<<3;object_height[stored_slot*11+:11]<=candidate_height<<3;
                end
            end
        end
    end endgenerate
    assign object_count_o=frontier_fault ? 0 : object_count;
    assign object_kind_o=object_kind;
    assign object_left_o=object_left;
    assign object_top_o=object_top;
    assign object_width_o=object_width;
    assign object_height_o=object_height;
    assign objects_full_o=objects_full;


    // Synchronous read precedes ST_HIST_WRITE by one full cycle. Each point
    // takes at least seven cycles, so repeated bin addresses see the last write.
    always @(posedge clk_i) begin
        histogram_u_read<=histogram_u[histogram_u_address];
        histogram_v_read<=histogram_v[histogram_v_address];
        if(state==ST_HIST_CLEAR) begin
            histogram_u[histogram_index]<=0;
            histogram_v[histogram_index]<=0;
        end else if(state==ST_HIST_WRITE) begin
            histogram_u[histogram_u_address]<=histogram_u_read+1'b1;
            histogram_v[histogram_v_address]<=histogram_v_read+1'b1;
        end
    end

    // 约 5.6 度一档；两个投影轴保持正交，避免角度量化破坏直角特征。
    always @(*) begin
        case(template_angle)
            0: begin cosine=32; sine=0; end
            1: begin cosine=32; sine=3; end
            2: begin cosine=31; sine=6; end
            3: begin cosine=31; sine=9; end
            4: begin cosine=30; sine=12; end
            5: begin cosine=28; sine=15; end
            6: begin cosine=27; sine=18; end
            7: begin cosine=25; sine=20; end
            8: begin cosine=23; sine=23; end
            9: begin cosine=20; sine=25; end
            10: begin cosine=18; sine=27; end
            11: begin cosine=15; sine=28; end
            12: begin cosine=12; sine=30; end
            13: begin cosine=9; sine=31; end
            14: begin cosine=6; sine=31; end
            default: begin cosine=3; sine=32; end
        endcase
    end

    // Chebyshev radius two connects a one-cell gap without inventing contour points.
    reg signed [9:0] neighbor_signed_x,neighbor_signed_y;
    always @(*) begin
        neighbor_signed_x=$signed({2'b00,current_x});
        neighbor_signed_y=$signed({2'b00,current_y});
        case(neighbor_index)
            0: begin neighbor_signed_x=$signed({2'b00,current_x})-2; neighbor_signed_y=$signed({2'b00,current_y})-2; end
            1: begin neighbor_signed_x=$signed({2'b00,current_x})-1; neighbor_signed_y=$signed({2'b00,current_y})-2; end
            2: begin neighbor_signed_x=$signed({2'b00,current_x})+0; neighbor_signed_y=$signed({2'b00,current_y})-2; end
            3: begin neighbor_signed_x=$signed({2'b00,current_x})+1; neighbor_signed_y=$signed({2'b00,current_y})-2; end
            4: begin neighbor_signed_x=$signed({2'b00,current_x})+2; neighbor_signed_y=$signed({2'b00,current_y})-2; end
            5: begin neighbor_signed_x=$signed({2'b00,current_x})-2; neighbor_signed_y=$signed({2'b00,current_y})-1; end
            6: begin neighbor_signed_x=$signed({2'b00,current_x})-1; neighbor_signed_y=$signed({2'b00,current_y})-1; end
            7: begin neighbor_signed_x=$signed({2'b00,current_x})+0; neighbor_signed_y=$signed({2'b00,current_y})-1; end
            8: begin neighbor_signed_x=$signed({2'b00,current_x})+1; neighbor_signed_y=$signed({2'b00,current_y})-1; end
            9: begin neighbor_signed_x=$signed({2'b00,current_x})+2; neighbor_signed_y=$signed({2'b00,current_y})-1; end
            10: begin neighbor_signed_x=$signed({2'b00,current_x})-2; neighbor_signed_y=$signed({2'b00,current_y})+0; end
            11: begin neighbor_signed_x=$signed({2'b00,current_x})-1; neighbor_signed_y=$signed({2'b00,current_y})+0; end
            12: begin neighbor_signed_x=$signed({2'b00,current_x})+1; neighbor_signed_y=$signed({2'b00,current_y})+0; end
            13: begin neighbor_signed_x=$signed({2'b00,current_x})+2; neighbor_signed_y=$signed({2'b00,current_y})+0; end
            14: begin neighbor_signed_x=$signed({2'b00,current_x})-2; neighbor_signed_y=$signed({2'b00,current_y})+1; end
            15: begin neighbor_signed_x=$signed({2'b00,current_x})-1; neighbor_signed_y=$signed({2'b00,current_y})+1; end
            16: begin neighbor_signed_x=$signed({2'b00,current_x})+0; neighbor_signed_y=$signed({2'b00,current_y})+1; end
            17: begin neighbor_signed_x=$signed({2'b00,current_x})+1; neighbor_signed_y=$signed({2'b00,current_y})+1; end
            18: begin neighbor_signed_x=$signed({2'b00,current_x})+2; neighbor_signed_y=$signed({2'b00,current_y})+1; end
            19: begin neighbor_signed_x=$signed({2'b00,current_x})-2; neighbor_signed_y=$signed({2'b00,current_y})+2; end
            20: begin neighbor_signed_x=$signed({2'b00,current_x})-1; neighbor_signed_y=$signed({2'b00,current_y})+2; end
            21: begin neighbor_signed_x=$signed({2'b00,current_x})+0; neighbor_signed_y=$signed({2'b00,current_y})+2; end
            22: begin neighbor_signed_x=$signed({2'b00,current_x})+1; neighbor_signed_y=$signed({2'b00,current_y})+2; end
            23: begin neighbor_signed_x=$signed({2'b00,current_x})+2; neighbor_signed_y=$signed({2'b00,current_y})+2; end
            default: begin end
        endcase
        if(ENABLE_SHAPES) begin
            case(neighbor_index)
                0: begin neighbor_signed_x=$signed({2'b00,current_x})+(-1); neighbor_signed_y=$signed({2'b00,current_y})+(-1); end
                1: begin neighbor_signed_x=$signed({2'b00,current_x})+(0); neighbor_signed_y=$signed({2'b00,current_y})+(-1); end
                2: begin neighbor_signed_x=$signed({2'b00,current_x})+(1); neighbor_signed_y=$signed({2'b00,current_y})+(-1); end
                3: begin neighbor_signed_x=$signed({2'b00,current_x})+(-1); neighbor_signed_y=$signed({2'b00,current_y})+(0); end
                4: begin neighbor_signed_x=$signed({2'b00,current_x})+(1); neighbor_signed_y=$signed({2'b00,current_y})+(0); end
                5: begin neighbor_signed_x=$signed({2'b00,current_x})+(-1); neighbor_signed_y=$signed({2'b00,current_y})+(1); end
                6: begin neighbor_signed_x=$signed({2'b00,current_x})+(0); neighbor_signed_y=$signed({2'b00,current_y})+(1); end
                7: begin neighbor_signed_x=$signed({2'b00,current_x})+(1); neighbor_signed_y=$signed({2'b00,current_y})+(1); end
                8: begin neighbor_signed_x=$signed({2'b00,current_x})+(-2); neighbor_signed_y=$signed({2'b00,current_y})+(-2); end
                9: begin neighbor_signed_x=$signed({2'b00,current_x})+(-1); neighbor_signed_y=$signed({2'b00,current_y})+(-2); end
                10: begin neighbor_signed_x=$signed({2'b00,current_x})+(0); neighbor_signed_y=$signed({2'b00,current_y})+(-2); end
                11: begin neighbor_signed_x=$signed({2'b00,current_x})+(1); neighbor_signed_y=$signed({2'b00,current_y})+(-2); end
                12: begin neighbor_signed_x=$signed({2'b00,current_x})+(2); neighbor_signed_y=$signed({2'b00,current_y})+(-2); end
                13: begin neighbor_signed_x=$signed({2'b00,current_x})+(-2); neighbor_signed_y=$signed({2'b00,current_y})+(-1); end
                14: begin neighbor_signed_x=$signed({2'b00,current_x})+(2); neighbor_signed_y=$signed({2'b00,current_y})+(-1); end
                15: begin neighbor_signed_x=$signed({2'b00,current_x})+(-2); neighbor_signed_y=$signed({2'b00,current_y})+(0); end
                16: begin neighbor_signed_x=$signed({2'b00,current_x})+(2); neighbor_signed_y=$signed({2'b00,current_y})+(0); end
                17: begin neighbor_signed_x=$signed({2'b00,current_x})+(-2); neighbor_signed_y=$signed({2'b00,current_y})+(1); end
                18: begin neighbor_signed_x=$signed({2'b00,current_x})+(2); neighbor_signed_y=$signed({2'b00,current_y})+(1); end
                19: begin neighbor_signed_x=$signed({2'b00,current_x})+(-2); neighbor_signed_y=$signed({2'b00,current_y})+(2); end
                20: begin neighbor_signed_x=$signed({2'b00,current_x})+(-1); neighbor_signed_y=$signed({2'b00,current_y})+(2); end
                21: begin neighbor_signed_x=$signed({2'b00,current_x})+(0); neighbor_signed_y=$signed({2'b00,current_y})+(2); end
                22: begin neighbor_signed_x=$signed({2'b00,current_x})+(1); neighbor_signed_y=$signed({2'b00,current_y})+(2); end
                23: begin neighbor_signed_x=$signed({2'b00,current_x})+(2); neighbor_signed_y=$signed({2'b00,current_y})+(2); end
                default: begin end
            endcase
        end
        neighbor_x=neighbor_signed_x[7:0];
        neighbor_y=neighbor_signed_y[7:0];
        neighbor_inside=neighbor_signed_x>=0 && neighbor_signed_x<GRID_WIDTH &&
            neighbor_signed_y>=0 && neighbor_signed_y<GRID_HEIGHT;
        neighbor_address=neighbor_y*GRID_WIDTH+neighbor_x;
    end

    // A bridge joins two local endpoints, never two nearby contour sides.
    always @(*) begin
        source_endpoint=0; bridge_endpoint=0;
        case(source_neighbors)
            8'h01, 8'h02, 8'h03, 8'h04, 8'h05, 8'h06, 8'h07, 8'h08, 8'h09, 8'h0a, 8'h0b, 8'h10, 8'h12, 8'h14, 8'h16, 8'h20, 8'h21, 8'h28, 8'h29, 8'h40, 8'h48, 8'h50, 8'h60, 8'h68, 8'h80, 8'h84, 8'h90, 8'h94, 8'ha0, 8'hc0, 8'hd0, 8'he0: source_endpoint=1;
            default: begin end
        endcase
        case(bridge_neighbors)
            8'h01, 8'h02, 8'h03, 8'h04, 8'h05, 8'h06, 8'h07, 8'h08, 8'h09, 8'h0a, 8'h0b, 8'h10, 8'h12, 8'h14, 8'h16, 8'h20, 8'h21, 8'h28, 8'h29, 8'h40, 8'h48, 8'h50, 8'h60, 8'h68, 8'h80, 8'h84, 8'h90, 8'h94, 8'ha0, 8'hc0, 8'hd0, 8'he0: bridge_endpoint=1;
            default: begin end
        endcase
        source_direction_mask=0; bridge_direction_mask=0;
        case(neighbor_index)
            8: begin source_direction_mask=8'hd0; bridge_direction_mask=8'h0b; end
            9: begin source_direction_mask=8'hf0; bridge_direction_mask=8'h0f; end
            10: begin source_direction_mask=8'he0; bridge_direction_mask=8'h07; end
            11: begin source_direction_mask=8'he8; bridge_direction_mask=8'h17; end
            12: begin source_direction_mask=8'h68; bridge_direction_mask=8'h16; end
            13: begin source_direction_mask=8'hd4; bridge_direction_mask=8'h2b; end
            14: begin source_direction_mask=8'h69; bridge_direction_mask=8'h96; end
            15: begin source_direction_mask=8'h94; bridge_direction_mask=8'h29; end
            16: begin source_direction_mask=8'h29; bridge_direction_mask=8'h94; end
            17: begin source_direction_mask=8'h96; bridge_direction_mask=8'h69; end
            18: begin source_direction_mask=8'h2b; bridge_direction_mask=8'hd4; end
            19: begin source_direction_mask=8'h16; bridge_direction_mask=8'h68; end
            20: begin source_direction_mask=8'h17; bridge_direction_mask=8'he8; end
            21: begin source_direction_mask=8'h07; bridge_direction_mask=8'he0; end
            22: begin source_direction_mask=8'h0f; bridge_direction_mask=8'hf0; end
            23: begin source_direction_mask=8'h0b; bridge_direction_mask=8'hd0; end
        default: begin end
        endcase
        probe_x=0;probe_y=0;
        case(bridge_probe)
            0: begin probe_x=$signed({2'b00,neighbor_x})+(-1); probe_y=$signed({2'b00,neighbor_y})+(-1); end
            1: begin probe_x=$signed({2'b00,neighbor_x})+(0); probe_y=$signed({2'b00,neighbor_y})+(-1); end
            2: begin probe_x=$signed({2'b00,neighbor_x})+(1); probe_y=$signed({2'b00,neighbor_y})+(-1); end
            3: begin probe_x=$signed({2'b00,neighbor_x})+(-1); probe_y=$signed({2'b00,neighbor_y})+(0); end
            4: begin probe_x=$signed({2'b00,neighbor_x})+(1); probe_y=$signed({2'b00,neighbor_y})+(0); end
            5: begin probe_x=$signed({2'b00,neighbor_x})+(-1); probe_y=$signed({2'b00,neighbor_y})+(1); end
            6: begin probe_x=$signed({2'b00,neighbor_x})+(0); probe_y=$signed({2'b00,neighbor_y})+(1); end
            7: begin probe_x=$signed({2'b00,neighbor_x})+(1); probe_y=$signed({2'b00,neighbor_y})+(1); end
            default: begin end
        endcase
        probe_inside=probe_x>=0 && probe_x<GRID_WIDTH && probe_y>=0 && probe_y<GRID_HEIGHT;
        probe_address=probe_y*GRID_WIDTH+probe_x;
    end

    // 同步单口图存储；捕获每八个横向像素合并一次，纵向八行原位 OR。
    always @(*) begin
        map_address=scan_address;
        map_write=1'b0;
        map_write_value=1'b0;
        enqueue=1'b0;
        enqueue_point={neighbor_y,neighbor_x};
        point_address=digit_stage ? digit_address : (state==ST_SHAPE_START || state==ST_SHAPE_WAIT) ? shape_address : point_index[10:0];
        if(labeling) map_address=label_address;
        if(state==ST_POP_REQ || state==ST_POP_WAIT || state==ST_POP_LOAD) begin
            point_address=queue_head[10:0];
        end
        if(state==ST_CAPTURE && !WRITE_TAP) begin
            map_address=capture_address;
            map_write=accepted && capture_group[0];
            map_write_value=half_edge || group_edge || ((capture_row[2:0]!=0) && map_read);
        end else if(state==ST_SEED) begin
            map_write=1'b1;
            enqueue=1'b1;
            enqueue_point={scan_y,scan_x};
        end else if(state==ST_NEIGHBOR || state==ST_NEIGHBOR_WAIT || state==ST_NEIGHBOR_TEST || bridge_state) begin
            map_address=neighbor_address;
            if(((state==ST_NEIGHBOR_TEST && (!ENABLE_SHAPES || neighbor_index<8)) ||
                (state==ST_BRIDGE_DECIDE && bridge_accept)) && neighbor_inside && map_read && !full_queue) begin
                map_write=1'b1;
                enqueue=1'b1;
            end
        end
    end
    // Four cells per RAM word match one canonical 32-pixel Canny write beat.
    // Tap addresses cannot repeat on adjacent cycles; row revisits are >=60 beats apart.
    wire [13:0] tap_address=(tap_row_i>>3)*(GRID_WIDTH/4)+tap_word_i;
    reg [13:0] tap_address_delay;
    reg [3:0] tap_bits_delay;
    reg tap_write_delay,tap_clear_delay;
    wire [3:0] pooled_bits;
    genvar cell_index;
    generate for(cell_index=0;cell_index<4;cell_index=cell_index+1) begin : pool_cells
        assign pooled_bits[cell_index]=|(tap_data_i[cell_index*16+:16] & 16'haaaa);
    end endgenerate
    wire tap_accept=WRITE_TAP && state==ST_CAPTURE && tap_valid_i;
    wire [13:0] memory_read_address=(WRITE_TAP && state==ST_CAPTURE) ? tap_address : map_address[15:2];
    wire [13:0] original_address=probing ? probe_address[15:2] : memory_read_address;
    wire [1:0] original_read_lane=probing ? probe_address[1:0] : map_address[1:0];
    wire [9:0] component_perimeter=({2'b0,bound_right}-{2'b0,bound_left})+
        ({2'b0,bound_bottom}-{2'b0,bound_top})+10'd2;
    wire dense_component=ENABLE_SHAPES && queue_tail>{component_perimeter,2'b00};
    wire memory_write=tap_write_delay || map_write || state==ST_LABEL_WRITE || state==ST_EVICT_WRITE;
    wire [13:0] memory_write_address=evicting ? evict_address : (tap_write_delay ? tap_address_delay : map_address[15:2]);
    wire [3:0] bit_mask=4'b0001<<map_address[1:0];
    wire [3:0] memory_write_data=tap_write_delay ?
        ((tap_clear_delay ? 4'b0 : map_word) | tap_bits_delay) :
        ((map_word & ~bit_mask) | (map_write_value ? bit_mask : 4'b0));
    wire [13:0] cell_read_address=evicting ? evict_address : ((state==ST_IDLE && label_enable_i) ? label_address_i : original_address);
    genvar label_lane;
    generate for(label_lane=0;label_lane<4;label_lane=label_lane+1) begin : cell_encoding
        assign original_word[label_lane]=read_codes[label_lane*LABEL_BITS+:LABEL_BITS]!=0;
        assign map_word[label_lane]=selected_codes[label_lane*LABEL_BITS+:LABEL_BITS]==1;
    end endgenerate
    // A 16-bit synchronous word packs four complete labels and retains immutable
    // adjacency, including rejected oversized components. No visited-code reuse.
    generate if(MAX_OBJECTS>4) begin: packed_labels
        (* syn_ramstyle="block_ram" *) reg [15:0] cell_memory[0:GRID_SIZE/4-1];
        reg [15:0] code_read;
        assign read_codes=code_read;
        always @(posedge clk_i) begin
            code_read<=cell_memory[cell_read_address];
            if(memory_write) cell_memory[memory_write_address]<=write_codes;
        end
    end else begin: legacy_labels
        (* syn_ramstyle="block_ram" *) reg [3:0] cell_low[0:GRID_SIZE/4-1];
        (* syn_ramstyle="block_ram" *) reg [3:0] cell_middle[0:GRID_SIZE/4-1];
        (* syn_ramstyle="block_ram" *) reg [3:0] cell_high[0:GRID_SIZE/4-1];
        reg [3:0] low_read,middle_read,high_read;
        genvar bit_lane;
        for(bit_lane=0;bit_lane<4;bit_lane=bit_lane+1) begin: plane_encoding
            assign read_codes[bit_lane*3+:3]={high_read[bit_lane],middle_read[bit_lane],low_read[bit_lane]};
            assign write_low[bit_lane]=write_codes[bit_lane*3];
            assign write_middle[bit_lane]=write_codes[bit_lane*3+1];
            assign write_high[bit_lane]=write_codes[bit_lane*3+2];
        end
        always @(posedge clk_i) begin
            low_read<=cell_low[cell_read_address];middle_read<=cell_middle[cell_read_address];high_read<=cell_high[cell_read_address];
            if(memory_write) begin
                cell_low[memory_write_address]<=write_low;cell_middle[memory_write_address]<=write_middle;cell_high[memory_write_address]<=write_high;
            end
        end
    end endgenerate
    integer write_lane;
    always @(*) begin
        write_codes=selected_codes;
        if(tap_write_delay || state==ST_CAPTURE) begin
            for(write_lane=0;write_lane<4;write_lane=write_lane+1)
                write_codes[write_lane*LABEL_BITS+:LABEL_BITS]={2'b00,memory_write_data[write_lane]};
        end else if(state==ST_EVICT_WRITE) begin
            for(write_lane=0;write_lane<4;write_lane=write_lane+1)
                if(read_codes[write_lane*LABEL_BITS+:LABEL_BITS]==component_label) write_codes[write_lane*LABEL_BITS+:LABEL_BITS]=3'd6;
        end else if(state==ST_LABEL_WRITE) write_codes[map_address[1:0]*LABEL_BITS+:LABEL_BITS]=component_label;
        else if(map_write) write_codes[map_address[1:0]*LABEL_BITS+:LABEL_BITS]=map_write_value ? 3'd1 : 3'd6;
    end
    always @(posedge clk_i) begin
        original_lane<=original_read_lane;
        map_lane<=map_address[1:0];
        if(state==ST_NEIGHBOR_TEST) neighbor_codes<=read_codes;
        point_read<=point_memory[point_address];
        if(enqueue) point_memory[queue_tail[10:0]]<=enqueue_point;
    end
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin label_address<=0;component_label<=0;evict_address<=0;end
        else begin
            if(state==ST_FIT_RESULT) begin component_label<=result_slot+4'd2+(result_slot>=4 ? 4'd1 : 4'd0);evict_address<=0;end
            if(state==ST_EVICT_WRITE) evict_address<=evict_address+1'b1;
            if(state==ST_LABEL_LOAD) label_address<=point_read[15:8]*GRID_WIDTH+point_read[7:0];
        end
    end
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            tap_address_delay<=0; tap_bits_delay<=0; tap_write_delay<=0; tap_clear_delay<=0;
        end else begin
            tap_write_delay<=tap_accept;
            tap_address_delay<=tap_address;
            tap_bits_delay<=pooled_bits;
            tap_clear_delay<=tap_first_i && tap_row_i[2:0]==0;
        end
    end

    // 状态转移与数据更新分开，便于审查同步 RAM 的读延迟。
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin state<=ST_IDLE; end
        else begin state<=next_state; end
    end
    always @(*) begin
        next_state=state;
        case(state)
            ST_IDLE: if(start_i) begin next_state=ST_CAPTURE; end
            ST_CAPTURE: if(WRITE_TAP ? analyze_i : (accepted && eop_i)) begin next_state=ST_SCAN_REQ; end
            ST_SCAN_REQ: begin next_state=ST_SCAN_WAIT; end
            ST_SCAN_WAIT: begin next_state=ST_SCAN_TEST; end
            ST_SCAN_TEST: begin next_state=map_read ? ST_SEED : ST_NEXT_SCAN; end
            ST_SEED: begin next_state=ST_POP_REQ; end
            ST_POP_REQ: begin next_state=ST_POP_WAIT; end
            ST_POP_WAIT: begin next_state=ST_POP_LOAD; end
            ST_POP_LOAD: begin next_state=ST_NEIGHBOR; end
            ST_NEIGHBOR: begin next_state=neighbor_inside ? ST_NEIGHBOR_WAIT : ST_NEXT_NEIGHBOR; end
            ST_NEIGHBOR_WAIT: begin next_state=ST_NEIGHBOR_TEST; end
            ST_NEIGHBOR_TEST: begin
                if(map_read && neighbor_inside && full_queue) next_state=ST_FINISH;
                else if(ENABLE_SHAPES && neighbor_index>=8 && map_read && neighbor_inside && source_bridge_ok) next_state=ST_BRIDGE_REQ;
                else next_state=ST_NEXT_NEIGHBOR;
            end
            ST_BRIDGE_REQ: next_state=ST_BRIDGE_WAIT;
            ST_BRIDGE_WAIT: next_state=ST_BRIDGE_TEST;
            ST_BRIDGE_TEST: next_state=bridge_probe==7 ? ST_BRIDGE_DECIDE : ST_BRIDGE_REQ;
            ST_BRIDGE_DECIDE: next_state=ST_NEXT_NEIGHBOR;
            ST_NEXT_NEIGHBOR: if(neighbor_index==23) begin
                next_state=queue_head==queue_tail ? ST_COMPONENT : ST_POP_REQ;
            end else begin next_state=ST_NEIGHBOR; end
            ST_COMPONENT: begin
                next_state=(!oversized && !dense_component && queue_tail>=40 && bound_right-bound_left>=15 && bound_bottom-bound_top>=15) ? (ENABLE_SHAPES ? ST_SHAPE_START : ST_ANGLE) : ST_NEXT_SCAN;
            end
            ST_SHAPE_START: next_state=ST_SHAPE_WAIT;
            ST_SHAPE_WAIT: if(shape_done) next_state=(ENABLE_EXTENDED_SHAPES && shape_kind==2) ? ST_DIGIT_START : (shape_kind==0 ? ST_ANGLE : ST_FIT_AREA);
            ST_ANGLE: begin next_state=ST_HIST_CLEAR; end
            ST_HIST_CLEAR: if(histogram_index==511) begin next_state=ST_POINT_REQ; end
            ST_POINT_REQ: begin next_state=ST_POINT_WAIT; end
            ST_POINT_WAIT: begin next_state=ST_MULTIPLY; end
            ST_MULTIPLY: begin next_state=ST_PROJECT; end
            ST_PROJECT: begin next_state=ST_ACCUMULATE; end
            ST_ACCUMULATE: begin
                if(!scoring) begin next_state=ST_HIST_WRITE; end
                else if(last_point) begin next_state=ST_DECIDE; end
                else begin next_state=ST_POINT_REQ; end
            end
            ST_HIST_WRITE: begin next_state=point_index==queue_tail ? ST_RANGE : ST_POINT_REQ; end
            ST_RANGE: begin next_state=ST_SCORE_START; end
            ST_SCORE_START: begin next_state=span_u>=480 && span_v>=480 ? ST_POINT_REQ : ST_NEXT_ANGLE; end
            ST_DECIDE: begin next_state=template_ok ? ST_FIT_BOUNDS : (trimmed_fit ? ST_NEXT_ANGLE : ST_HIST_READ); end
            ST_HIST_READ: begin next_state=ST_HIST_WAIT; end
            ST_HIST_WAIT: begin next_state=ST_HIST_SCAN; end
            ST_HIST_SCAN: begin next_state=histogram_index==511 ? ST_TRIM_RANGE : ST_HIST_READ; end
            ST_TRIM_RANGE: begin next_state=ST_SCORE_START; end
            ST_FIT_BOUNDS: begin next_state=ST_FIT_AREA; end
            ST_FIT_AREA: begin next_state=ST_FIT_RESULT; end
            ST_FIT_RESULT: begin next_state=replace_digit ? ST_EVICT_READ : (accept_object ? ST_LABEL_REQUEST : ST_NEXT_SCAN); end
            ST_EVICT_READ:next_state=ST_EVICT_WAIT;
            ST_EVICT_WAIT:next_state=ST_EVICT_WRITE;
            ST_EVICT_WRITE:next_state=evict_address==GRID_SIZE/4-1 ? ST_LABEL_REQUEST : ST_EVICT_READ;
            ST_LABEL_REQUEST:next_state=ST_LABEL_WAIT;
            ST_LABEL_WAIT:next_state=ST_LABEL_LOAD;
            ST_LABEL_LOAD:next_state=ST_LABEL_READ;
            ST_LABEL_READ:next_state=ST_LABEL_WRITE;
            ST_LABEL_WRITE:next_state=last_point ? ST_NEXT_SCAN : ST_LABEL_REQUEST;
            ST_NEXT_ANGLE: begin next_state=template_angle==15 ?
                (ENABLE_DIGITS && digit_attempts<MAX_OBJECTS && object_count<MAX_OBJECTS ? ST_DIGIT_START : ST_NEXT_SCAN) : ST_ANGLE; end
            ST_DIGIT_START:next_state=ST_DIGIT_WAIT;
            ST_DIGIT_WAIT:if(digit_done) next_state=(digit_kind==0 && !circle_probe) ? ST_NEXT_SCAN : ST_FIT_AREA;
            ST_NEXT_SCAN: begin next_state=scan_address==GRID_SIZE-1 ? ST_FINISH : ST_SCAN_REQ; end
            ST_FINISH: begin next_state=ST_IDLE; end
            default: begin next_state=ST_IDLE; end
        endcase
        // Recognition has a fixed time budget; clutter cannot stall HDMI updates indefinitely.
        if(analysis_timeout) next_state=ST_FINISH;
    end
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            capture_group<=0; capture_row<=0; capture_address<=0; half_edge<=0;
            scan_address<=0; scan_x<=0; scan_y<=0;
            current_x<=0; current_y<=0; neighbor_index<=0;
            source_neighbors<=0;bridge_neighbors<=0;bridge_probe<=0;
            queue_head<=0; queue_tail<=0; point_index<=0;
            bound_left<=0; bound_right<=0; bound_top<=0; bound_bottom<=0;
            bound_width<=0; bound_height<=0; bound_area<=0; best_area<=0;
            found<=0; overflow<=0; frontier_fault<=0; result_left<=0; result_top<=0;
            result_width<=0; result_height<=0; result_area<=0; result_angle<=0;
            template_angle<=0; scoring<=0; elapsed<=0; result_cycles<=0;
            analysis_cycles<=0;
            product_xc<=0; product_ys<=0; product_yc<=0; product_xs<=0;
            project_u<=0; project_v<=0;
            minimum_u<=0; maximum_u<=0; minimum_v<=0; maximum_v<=0;
            span_u<=0; span_v<=0; tolerance_u<=0; tolerance_v<=0;
            support_count<=0; left_count<=0; right_count<=0; top_count<=0; bottom_count<=0; corner_mask<=0;
            side_middle_mask<=0;
            histogram_index<=0; cumulative_u<=0; cumulative_v<=0;
            low_u_seen<=0; high_u_seen<=0; low_v_seen<=0; high_v_seen<=0; trimmed_fit<=0;
            fit_left<=0; fit_right<=0; fit_top<=0; fit_bottom<=0;
        end else begin
            if(state!=ST_IDLE) begin elapsed<=elapsed+1'b1; end
            if(state==ST_IDLE || state==ST_CAPTURE) analysis_cycles<=0;
            else if(analysis_active && !digit_stage && !labeling && !evicting && !extension_result) analysis_cycles<=analysis_cycles+1'b1;
            case(state)
                ST_IDLE: if(start_i) begin
                    capture_group<=0; capture_row<=0; capture_address<=0; half_edge<=0;
                    scan_address<=0; scan_x<=0; scan_y<=0; queue_head<=0; queue_tail<=0;
                    found<=0; overflow<=0; frontier_fault<=0; best_area<=0; elapsed<=0;
                    result_left<=0; result_top<=0; result_width<=0; result_height<=0; result_area<=0;
                end
                ST_CAPTURE: if(WRITE_TAP && start_i) begin
                    found<=0; overflow<=0; frontier_fault<=0; best_area<=0; elapsed<=0;
                    scan_address<=0; scan_x<=0; scan_y<=0; queue_head<=0; queue_tail<=0;
                    result_left<=0; result_top<=0; result_width<=0; result_height<=0; result_area<=0;
                end else if(accepted && !WRITE_TAP) begin
                    half_edge<=group_edge;
                    if(capture_group==INPUT_WIDTH/4-1) begin
                        capture_group<=0; capture_row<=capture_row+1'b1;
                        if(capture_row[2:0]==7) begin capture_address<=capture_address+1'b1; end
                        else begin capture_address<=capture_address-(GRID_WIDTH-1); end
                    end else begin
                        capture_group<=capture_group+1'b1;
                        if(capture_group[0]) begin capture_address<=capture_address+1'b1; end
                    end
                end
                ST_SCAN_TEST: if(map_read) begin
                    queue_head<=0; queue_tail<=0;
                    bound_left<=scan_x; bound_right<=scan_x; bound_top<=scan_y; bound_bottom<=scan_y;
                end
                ST_SEED: begin queue_tail<=1; end
                ST_POP_LOAD: begin
                    current_x<=point_read[7:0]; current_y<=point_read[15:8];
                    queue_head<=queue_head+1'b1; neighbor_index<=0;source_neighbors<=0;
                end
                ST_NEIGHBOR_TEST: begin
                    if(ENABLE_SHAPES && neighbor_index<8) source_neighbors[neighbor_index[2:0]]<=neighbor_inside && original_edge;
                    bridge_neighbors<=0;bridge_probe<=0;
                    if(map_read && neighbor_inside) begin
                        if(full_queue) begin overflow<=1; frontier_fault<=1; found<=0; end
                        else if(!ENABLE_SHAPES || neighbor_index<8) begin
                            queue_tail<=queue_tail+1'b1;
                            if(neighbor_x<bound_left) bound_left<=neighbor_x;
                            if(neighbor_x>bound_right) bound_right<=neighbor_x;
                            if(neighbor_y<bound_top) bound_top<=neighbor_y;
                            if(neighbor_y>bound_bottom) bound_bottom<=neighbor_y;
                        end
                    end
                end
                ST_BRIDGE_TEST: begin
                    bridge_neighbors[bridge_probe]<=probe_inside && original_edge;
                    bridge_probe<=bridge_probe+1'b1;
                end
                ST_BRIDGE_DECIDE: if(bridge_accept && map_read && !full_queue) begin
                    queue_tail<=queue_tail+1'b1;
                    if(neighbor_x<bound_left) bound_left<=neighbor_x;
                    if(neighbor_x>bound_right) bound_right<=neighbor_x;
                    if(neighbor_y<bound_top) bound_top<=neighbor_y;
                    if(neighbor_y>bound_bottom) bound_bottom<=neighbor_y;
                end
                ST_NEXT_NEIGHBOR: begin neighbor_index<=neighbor_index+1'b1; end
                ST_COMPONENT: begin
                    if(oversized) overflow<=1;
                    template_angle<=0;
                    bound_width<={1'b0,bound_right}-{1'b0,bound_left}+1'b1;
                    bound_height<={1'b0,bound_bottom}-{1'b0,bound_top}+1'b1;
                end
                ST_ANGLE: begin
                    scoring<=0; point_index<=0;
                    minimum_u<=16'sh7fff; maximum_u<=-16'sh7fff;
                    minimum_v<=16'sh7fff; maximum_v<=-16'sh7fff;
                    histogram_index<=0; trimmed_fit<=0;
                end
                ST_HIST_CLEAR: begin histogram_index<=histogram_index+1'b1; end
                ST_MULTIPLY: begin
                    product_xc<=point_read[7:0]*cosine;
                    product_ys<=point_read[15:8]*sine;
                    product_yc<=point_read[15:8]*cosine;
                    product_xs<=point_read[7:0]*sine;
                end
                ST_PROJECT: begin
                    project_u<=$signed({2'b00,product_xc})+$signed({2'b00,product_ys});
                    project_v<=$signed({2'b00,product_yc})-$signed({2'b00,product_xs});
                end
                ST_ACCUMULATE: begin
                    point_index<=point_index+1'b1;
                    if(!scoring) begin
                        if(project_u<minimum_u) begin minimum_u<=project_u; end
                        if(project_u>maximum_u) begin maximum_u<=project_u; end
                        if(project_v<minimum_v) begin minimum_v<=project_v; end
                        if(project_v>maximum_v) begin maximum_v<=project_v; end
                    end else begin
                        if(near_left || near_right || near_top || near_bottom) begin support_count<=support_count+1'b1; end
                        if(near_left) begin left_count<=left_count+1'b1; end
                        if(near_right) begin right_count<=right_count+1'b1; end
                        if(near_top) begin top_count<=top_count+1'b1; end
                        if(near_bottom) begin bottom_count<=bottom_count+1'b1; end
                        corner_mask<=corner_mask | {near_right && near_bottom,near_left && near_bottom,near_right && near_top,near_left && near_top};
                        side_middle_mask<=side_middle_mask | {near_left && middle_v,near_right && middle_v,
                            near_top && middle_u,near_bottom && middle_u};
                        if(inside_core && (near_left || near_right || near_top || near_bottom)) begin
                            if(point_read[7:0]<fit_left) fit_left<=point_read[7:0];
                            if(point_read[7:0]>fit_right) fit_right<=point_read[7:0];
                            if(point_read[15:8]<fit_top) fit_top<=point_read[15:8];
                            if(point_read[15:8]>fit_bottom) fit_bottom<=point_read[15:8];
                        end
                    end
                end
                ST_RANGE: begin span_u<=maximum_u-minimum_u; span_v<=maximum_v-minimum_v; end
                ST_SCORE_START: begin
                    tolerance_u<=32+(span_v>>4); tolerance_v<=32+(span_u>>4);
                    point_index<=0; scoring<=1; support_count<=0;
                    left_count<=0; right_count<=0; top_count<=0; bottom_count<=0; corner_mask<=0;
                    side_middle_mask<=0;
                    fit_left<=255; fit_right<=0; fit_top<=255; fit_bottom<=0;
                end
                ST_DECIDE: if(!template_ok && !trimmed_fit) begin
                    histogram_index<=0; cumulative_u<=0; cumulative_v<=0;
                    low_u_seen<=0; high_u_seen<=0; low_v_seen<=0; high_v_seen<=0;
                    trimmed_fit<=1;
                end
                ST_HIST_SCAN: begin
                    cumulative_u<=cumulative_u_next; cumulative_v<=cumulative_v_next;
                    histogram_index<=histogram_index+1'b1;
                    if(!low_u_seen && cumulative_u_next>trim_count) begin minimum_u<=histogram_u_value; low_u_seen<=1; end
                    if(!high_u_seen && cumulative_u_next>=upper_rank) begin maximum_u<=histogram_u_value+16'sd31; high_u_seen<=1; end
                    if(!low_v_seen && cumulative_v_next>trim_count) begin minimum_v<=histogram_v_value; low_v_seen<=1; end
                    if(!high_v_seen && cumulative_v_next>=upper_rank) begin maximum_v<=histogram_v_value+16'sd31; high_v_seen<=1; end
                end
                ST_TRIM_RANGE: begin span_u<=maximum_u-minimum_u; span_v<=maximum_v-minimum_v; end
                ST_FIT_BOUNDS: if(trimmed_fit) begin
                    bound_left<=fit_left; bound_top<=fit_top;
                    bound_width<={1'b0,fit_right}-{1'b0,fit_left}+1'b1;
                    bound_height<={1'b0,fit_bottom}-{1'b0,fit_top}+1'b1;
                end
                ST_FIT_AREA: begin bound_area<=bound_width*bound_height; end
                ST_LABEL_WRITE:point_index<=point_index+1'b1;
                ST_FIT_RESULT: begin
                    point_index<=0;
                    if(bound_area>best_area) begin
                    found<=1; best_area<=bound_area;
                    result_left<={bound_left,3'b000}; result_top<={bound_top,3'b000};
                    result_width<=bound_width<<3; result_height<=bound_height<<3;
                    result_area<=bound_area<<6; result_angle<=template_angle;
                    end
                end
                ST_NEXT_ANGLE: begin template_angle<=template_angle+1'b1; end
                ST_NEXT_SCAN: begin
                    scan_address<=scan_address+1'b1;
                    if(scan_x==GRID_WIDTH-1) begin scan_x<=0; scan_y<=scan_y+1'b1; end
                    else begin scan_x<=scan_x+1'b1; end
                end
                ST_FINISH: begin result_cycles<=elapsed; end
                default: begin end
            endcase
            if(analysis_timeout) begin
                overflow<=1;
                frontier_fault<=1;
                found<=0;
            end
        end
    end
    assign ready_o=state==ST_CAPTURE;
    assign done_o=state==ST_FINISH;
    assign found_o=found && !frontier_fault;
    assign overflow_o=overflow && !found;
    assign left_o=result_left;
    assign top_o=result_top;
    assign width_o=result_width;
    assign height_o=result_height;
    assign area_o=result_area;
    assign angle_o=result_angle;
    assign cycles_o=result_cycles;
endmodule
