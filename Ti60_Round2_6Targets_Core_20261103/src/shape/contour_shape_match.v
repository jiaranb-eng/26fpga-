`timescale 1ns / 1ps
// Description: Classify circles and triangles using a shared synchronous point RAM.
// 描述：复用同步轮廓点 RAM，以圆环支持率和三条线段支持率判断形状。
// Clock: clk_i, 96 MHz; Reset: rst_n_i, asynchronous active low.
// TestBench: verification/multi_shape_iteration/tb_multi_shape.v
module contour_shape_match (
    input wire clk_i,
    input wire rst_n_i,
    input wire start_i,
    input wire cancel_i,
    input wire [15:0] count_i,
    input wire [7:0] left_i,
    input wire [7:0] right_i,
    input wire [7:0] top_i,
    input wire [7:0] bottom_i,
    output wire [10:0] address_o,
    input wire [15:0] point_i,
    output wire done_o,
    output wire [1:0] kind_o
);
    // Classification phases share arithmetic and the caller's point storage.
    localparam IDLE=0, PREPARE=1, REQUEST=2, WAIT_RAM=3, OFFSET=4;
    localparam MULTIPLY=5, SUM=6, SCORE=7, PASS_DONE=8, DONE=9;
    reg [3:0] state, next_state;
    reg [1:0] phase, edge_index, kind;
    reg [10:0] point_index;
    reg [15:0] point_count;
    reg [7:0] vertex_ax,vertex_ay,vertex_bx,vertex_by,vertex_cx,vertex_cy;
    reg [7:0] box_left,box_top,span_x,span_y;
    reg [8:0] center_x,center_y;
    reg [8:0] radius,radial_tolerance;
    reg [8:0] radius_low,radius_high;
    reg [17:0] radius_low_square,radius_high_square;
    reg [7:0] side_tolerance;
    reg [15:0] box_area;
    reg circle_aspect;
    reg [7:0] reference_x,reference_y;
    reg signed [10:0] edge_dx,edge_dy,offset_x,offset_y,radial_x,radial_y;
    reg [10:0] edge_abs_x,edge_abs_y;
    reg [21:0] edge_square_x,edge_square_y;
    reg [21:0] edge_length,side_limit;
    reg signed [21:0] product_x,product_y,dot_x,dot_y;
    reg [21:0] radial_square_x,radial_square_y;
    reg signed [22:0] cross_value,dot_value;
    reg [22:0] distance_square,radial_square;
    reg [22:0] greatest_distance,greatest_cross;
    reg [15:0] circle_support,triangle_support;
    reg [15:0] side_support [0:2];
    reg [7:0] sectors;
    reg [2:0] middle_mask;
    reg point_supported;
    reg [7:0] current_x,current_y;
    reg [2:0] sector;
    wire [22:0] cross_magnitude=cross_value[22] ? -cross_value : cross_value;
    wire on_side=cross_magnitude<=side_limit && !dot_value[22] && dot_value<= $signed({1'b0,edge_length});
    wire in_middle=dot_value>$signed({3'b000,edge_length[21:2]}) &&
        dot_value<$signed({1'b0,edge_length})-$signed({3'b000,edge_length[21:2]});
    wire on_circle=radial_square>=radius_low_square && radial_square<=radius_high_square;
    wire circle_ok=circle_aspect && (circle_support*8>=point_count*7) && (&sectors);
    wire triangle_ok=(triangle_support*8>=point_count*7) && (&middle_mask) &&
        (side_support[0]*16>=point_count) && (side_support[1]*16>=point_count) &&
        (side_support[2]*16>=point_count) && (greatest_cross*4>=box_area);
    wire last_point=point_index+1'b1==point_count;

    // RAM addressing stays stable through REQUEST and WAIT_RAM.
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) state<=IDLE;
        else if(cancel_i) state<=IDLE;
        else state<=next_state;
    end
    always @(*) begin
        next_state=state;
        case(state)
            IDLE: if(start_i) next_state=PREPARE;
            PREPARE: next_state=REQUEST;
            REQUEST: next_state=WAIT_RAM;
            WAIT_RAM: next_state=OFFSET;
            OFFSET: next_state=MULTIPLY;
            MULTIPLY: next_state=SUM;
            SUM: next_state=SCORE;
            SCORE: if(phase==3 && edge_index!=2) next_state=PREPARE;
                else next_state=last_point ? PASS_DONE : (phase==3 ? PREPARE : REQUEST);
            PASS_DONE: if(phase==3 || (phase==2 && circle_ok)) next_state=DONE;
                else next_state=PREPARE;
            DONE: next_state=IDLE;
            default: next_state=IDLE;
        endcase
    end
    integer side;
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            phase<=0; edge_index<=0; kind<=0; point_index<=0; point_count<=0;
            vertex_ax<=0;vertex_ay<=0;vertex_bx<=0;vertex_by<=0;vertex_cx<=0;vertex_cy<=0;
            box_left<=0;box_top<=0;span_x<=0;span_y<=0;center_x<=0;center_y<=0;
            radius<=0;radial_tolerance<=0;radius_low<=0;radius_high<=0;
            radius_low_square<=0;radius_high_square<=0;side_tolerance<=0;box_area<=0;circle_aspect<=0;
            reference_x<=0;reference_y<=0;edge_dx<=0;edge_dy<=0;offset_x<=0;offset_y<=0;
            radial_x<=0;radial_y<=0;edge_abs_x<=0;edge_abs_y<=0;edge_square_x<=0;edge_square_y<=0;
            edge_length<=0;side_limit<=0;product_x<=0;product_y<=0;dot_x<=0;dot_y<=0;
            radial_square_x<=0;radial_square_y<=0;cross_value<=0;dot_value<=0;
            distance_square<=0;radial_square<=0;greatest_distance<=0;greatest_cross<=0;
            circle_support<=0;triangle_support<=0;sectors<=0;middle_mask<=0;point_supported<=0;
            current_x<=0;current_y<=0;sector<=0;
            for(side=0;side<3;side=side+1) side_support[side]<=0;
        end else begin
            // These setup registers settle while the first two diameter passes run.
            radius<=({1'b0,span_x}+{1'b0,span_y})>>1;
            radial_tolerance<=3+((span_x>span_y ? span_x : span_y)>>4);
            side_tolerance<=1+((span_x>span_y ? span_x : span_y)>>5);
            radius_low<=radius>radial_tolerance ? radius-radial_tolerance : 0;
            radius_high<=radius+radial_tolerance;
            radius_low_square<=radius_low*radius_low;
            radius_high_square<=radius_high*radius_high;
            box_area<=span_x*span_y;
            circle_aspect<=({2'b0,span_x}*4<={2'b0,span_y}*5) && ({2'b0,span_y}*4<={2'b0,span_x}*5);
            edge_abs_x<=edge_dx[10] ? -edge_dx : edge_dx;
            edge_abs_y<=edge_dy[10] ? -edge_dy : edge_dy;
            edge_square_x<=edge_dx*edge_dx;edge_square_y<=edge_dy*edge_dy;
            edge_length<=edge_square_x+edge_square_y;
            side_limit<=(edge_abs_x+edge_abs_y)*side_tolerance;
            case(state)
                IDLE: if(start_i) begin
                    phase<=0;edge_index<=0;kind<=0;point_index<=0;point_count<=count_i;
                    box_left<=left_i;box_top<=top_i;span_x<=right_i-left_i;span_y<=bottom_i-top_i;
                    center_x<={1'b0,left_i}+right_i;center_y<={1'b0,top_i}+bottom_i;
                    greatest_distance<=0;greatest_cross<=0;circle_support<=0;triangle_support<=0;
                    sectors<=0;middle_mask<=0;point_supported<=0;
                    for(side=0;side<3;side=side+1) side_support[side]<=0;
                end
                PREPARE: begin
                    if(phase==0) begin reference_x<=box_left;reference_y<=box_top;end
                    else if(phase!=3 || edge_index==0) begin
                        reference_x<=vertex_ax;reference_y<=vertex_ay;
                        edge_dx<=$signed({3'b0,vertex_bx})-$signed({3'b0,vertex_ax});
                        edge_dy<=$signed({3'b0,vertex_by})-$signed({3'b0,vertex_ay});
                    end else if(edge_index==1) begin
                        reference_x<=vertex_bx;reference_y<=vertex_by;
                        edge_dx<=$signed({3'b0,vertex_cx})-$signed({3'b0,vertex_bx});
                        edge_dy<=$signed({3'b0,vertex_cy})-$signed({3'b0,vertex_by});
                    end else begin
                        reference_x<=vertex_cx;reference_y<=vertex_cy;
                        edge_dx<=$signed({3'b0,vertex_ax})-$signed({3'b0,vertex_cx});
                        edge_dy<=$signed({3'b0,vertex_ay})-$signed({3'b0,vertex_cy});
                    end
                end
                OFFSET: begin
                    current_x<=point_i[7:0];current_y<=point_i[15:8];
                    offset_x<=$signed({3'b0,point_i[7:0]})-$signed({3'b0,reference_x});
                    offset_y<=$signed({3'b0,point_i[15:8]})-$signed({3'b0,reference_y});
                    radial_x<=$signed({2'b0,point_i[7:0],1'b0})-$signed({2'b0,center_x});
                    radial_y<=$signed({2'b0,point_i[15:8],1'b0})-$signed({2'b0,center_y});
                end
                MULTIPLY: begin
                    product_x<=phase<2 ? offset_x*offset_x : edge_dx*offset_y;
                    product_y<=phase<2 ? offset_y*offset_y : edge_dy*offset_x;
                    dot_x<=edge_dx*offset_x;dot_y<=edge_dy*offset_y;
                    radial_square_x<=radial_x*radial_x;radial_square_y<=radial_y*radial_y;
                    sector<={radial_x[10],radial_y[10],
                        (radial_x[10] ? -radial_x : radial_x)<(radial_y[10] ? -radial_y : radial_y)};
                end
                SUM: begin
                    distance_square<=$unsigned(product_x)+$unsigned(product_y);
                    cross_value<=$signed(product_x)-$signed(product_y);
                    dot_value<=$signed(dot_x)+$signed(dot_y);
                    radial_square<=radial_square_x+radial_square_y;
                end
                SCORE: begin
                    if(phase<2 && distance_square>greatest_distance) begin
                        greatest_distance<=distance_square;
                        if(phase==0) begin vertex_ax<=current_x;vertex_ay<=current_y;end
                        else begin vertex_bx<=current_x;vertex_by<=current_y;end
                    end
                    if(phase==2) begin
                        if(cross_magnitude>greatest_cross) begin
                            greatest_cross<=cross_magnitude;vertex_cx<=current_x;vertex_cy<=current_y;
                        end
                        if(on_circle) begin circle_support<=circle_support+1'b1;sectors[sector]<=1'b1;end
                    end
                    if(phase==3) begin
                        if(on_side) begin
                            side_support[edge_index]<=side_support[edge_index]+1'b1;
                            if(in_middle) middle_mask[edge_index]<=1'b1;
                        end
                        if(edge_index==2) begin
                            if(point_supported || on_side) triangle_support<=triangle_support+1'b1;
                            point_supported<=0;edge_index<=0;
                        end else begin
                            point_supported<=point_supported || on_side;edge_index<=edge_index+1'b1;
                        end
                    end
                    if(phase!=3 || edge_index==2) point_index<=point_index+1'b1;
                end
                PASS_DONE: begin
                    point_index<=0;greatest_distance<=0;
                    if(phase==2 && circle_ok) kind<=2;
                    else if(phase==3) kind<=triangle_ok ? 3 : 0;
                    else phase<=phase+1'b1;
                end
                default: begin end
            endcase
        end
    end
    assign address_o=point_index;
    assign kind_o=kind;
    assign done_o=state==DONE;
endmodule
