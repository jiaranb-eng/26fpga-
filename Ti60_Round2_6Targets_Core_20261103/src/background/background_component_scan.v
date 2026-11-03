`timescale 1ns / 1ps

// 八邻接网格洪泛；完整统计每个区域后，保留面积最大的八个框。
// Flood fill consumes each mask cell once and never allocates output slots to noise.
module background_component_scan #(
    parameter C_GRID_WIDTH = 120,
    parameter C_GRID_HEIGHT = 75,
    parameter C_OBJECT_MIN = 2
) (
    input wire i_clk,
    input wire i_rst_n,
    input wire i_start,
    output wire o_mask_read,
    output wire o_mask_clear,
    output wire [13:0] o_mask_address,
    input wire [1:0] i_mask_data,
    output wire o_done,
    output wire [7:0] o_box_valid,
    output wire [87:0] o_left,
    output wire [87:0] o_top,
    output wire [87:0] o_right,
    output wire [87:0] o_bottom,
    output wire [3:0] o_count,
    output wire o_overflow
);
    localparam C_GRID_CELLS = C_GRID_WIDTH * C_GRID_HEIGHT;
    localparam [4:0] ST_IDLE = 0, ST_SCAN_READ = 1, ST_SCAN_CHECK = 2,
        ST_SEED = 3, ST_POP_READ = 4, ST_POP_LOAD = 5,
        ST_NEIGHBOR_ADDRESS = 6, ST_NEIGHBOR_READ = 7,
        ST_NEIGHBOR_CHECK = 8, ST_NEIGHBOR_NEXT = 9,
        ST_RANK_INIT = 10, ST_RANK = 11, ST_STORE = 12,
        ST_SCAN_NEXT = 13, ST_PUBLISH_INIT = 14,
        ST_PUBLISH = 15, ST_FINISH = 16;

    reg [4:0] reg_state, reg_next_state;
    reg [13:0] reg_scan_index;
    reg [6:0] reg_scan_x, reg_scan_y;
    // 每个网格入队一次；9000 个单元的精确上界，无像素帧缓存。
    reg [13:0] mem_queue [0:C_GRID_CELLS-1];
    reg [13:0] reg_queue_word;
    reg [13:0] reg_queue_head, reg_queue_tail;
    reg [6:0] reg_center_x, reg_center_y;
    reg [13:0] reg_center_index;
    reg [2:0] reg_neighbor;
    reg [6:0] reg_neighbor_x, reg_neighbor_y;
    reg [13:0] reg_neighbor_index;
    reg reg_neighbor_valid;
    reg [6:0] wire_neighbor_x, wire_neighbor_y;
    reg [13:0] wire_neighbor_index;
    reg wire_neighbor_valid;
    reg [6:0] reg_component_left, reg_component_top;
    reg [6:0] reg_component_right, reg_component_bottom;
    reg [13:0] reg_component_cells;
    reg [13:0] reg_component_seeds; // 弱支持只能扩展已有目标，不能绕过原种子面积门槛
    reg [13:0] reg_result_cells [0:7];
    reg [6:0] reg_result_left [0:7];
    reg [6:0] reg_result_top [0:7];
    reg [6:0] reg_result_right [0:7];
    reg [6:0] reg_result_bottom [0:7];
    reg [2:0] reg_rank_index, reg_smallest_index, reg_publish_index;
    reg [13:0] reg_smallest_cells;
    reg [7:0] reg_box_valid_o;
    reg [87:0] reg_left_o, reg_top_o, reg_right_o, reg_bottom_o;
    reg [3:0] reg_count_o;
    reg reg_overflow_o;
    integer slot;
    wire wire_enqueue = reg_state == ST_NEIGHBOR_CHECK && i_mask_data[0];
    wire wire_queue_seed = reg_state == ST_SEED;
    wire [13:0] wire_queue_write_address = wire_queue_seed ? 14'd0 : reg_queue_tail;
    wire [13:0] wire_queue_write_data = wire_queue_seed ?
        {reg_scan_y, reg_scan_x} : {reg_neighbor_y, reg_neighbor_x};

    assign o_mask_read = reg_state == ST_SCAN_READ ||
        (reg_state == ST_NEIGHBOR_READ && reg_neighbor_valid);
    assign o_mask_clear = reg_state == ST_SEED || wire_enqueue;
    assign o_mask_address = (reg_state == ST_SCAN_READ || reg_state == ST_SEED) ?
        reg_scan_index : reg_neighbor_index;
    assign o_done = reg_state == ST_FINISH;
    assign o_box_valid = reg_box_valid_o;
    assign o_left = reg_left_o;
    assign o_top = reg_top_o;
    assign o_right = reg_right_o;
    assign o_bottom = reg_bottom_o;
    assign o_count = reg_count_o;
    assign o_overflow = reg_overflow_o;

    // 八方向地址只由已登记中心坐标计算；越界邻点不会访问 RAM。
    always @(*) begin
        wire_neighbor_x = reg_center_x;
        wire_neighbor_y = reg_center_y;
        wire_neighbor_index = reg_center_index;
        wire_neighbor_valid = 1'b1;
        case (reg_neighbor)
            3'd0: begin
                wire_neighbor_x = reg_center_x - 1'b1;
                wire_neighbor_y = reg_center_y - 1'b1;
                wire_neighbor_index = reg_center_index - C_GRID_WIDTH - 1'b1;
                wire_neighbor_valid = reg_center_x != 0 && reg_center_y != 0;
            end
            3'd1: begin
                wire_neighbor_y = reg_center_y - 1'b1;
                wire_neighbor_index = reg_center_index - C_GRID_WIDTH;
                wire_neighbor_valid = reg_center_y != 0;
            end
            3'd2: begin
                wire_neighbor_x = reg_center_x + 1'b1;
                wire_neighbor_y = reg_center_y - 1'b1;
                wire_neighbor_index = reg_center_index - C_GRID_WIDTH + 1'b1;
                wire_neighbor_valid = reg_center_x != C_GRID_WIDTH-1 && reg_center_y != 0;
            end
            3'd3: begin
                wire_neighbor_x = reg_center_x - 1'b1;
                wire_neighbor_index = reg_center_index - 1'b1;
                wire_neighbor_valid = reg_center_x != 0;
            end
            3'd4: begin
                wire_neighbor_x = reg_center_x + 1'b1;
                wire_neighbor_index = reg_center_index + 1'b1;
                wire_neighbor_valid = reg_center_x != C_GRID_WIDTH-1;
            end
            3'd5: begin
                wire_neighbor_x = reg_center_x - 1'b1;
                wire_neighbor_y = reg_center_y + 1'b1;
                wire_neighbor_index = reg_center_index + C_GRID_WIDTH - 1'b1;
                wire_neighbor_valid = reg_center_x != 0 && reg_center_y != C_GRID_HEIGHT-1;
            end
            3'd6: begin
                wire_neighbor_y = reg_center_y + 1'b1;
                wire_neighbor_index = reg_center_index + C_GRID_WIDTH;
                wire_neighbor_valid = reg_center_y != C_GRID_HEIGHT-1;
            end
            3'd7: begin
                wire_neighbor_x = reg_center_x + 1'b1;
                wire_neighbor_y = reg_center_y + 1'b1;
                wire_neighbor_index = reg_center_index + C_GRID_WIDTH + 1'b1;
                wire_neighbor_valid = reg_center_x != C_GRID_WIDTH-1 && reg_center_y != C_GRID_HEIGHT-1;
            end
            default: wire_neighbor_valid = 1'b0;
        endcase
    end

    // 状态寄存器与组合下一状态独立，同步 RAM 均预留读延迟。
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) reg_state <= ST_IDLE;
        else reg_state <= reg_next_state;
    end
    always @(*) begin
        reg_next_state = reg_state;
        case (reg_state)
            ST_IDLE: if (i_start) reg_next_state = ST_SCAN_READ;
            ST_SCAN_READ: reg_next_state = ST_SCAN_CHECK;
            ST_SCAN_CHECK: reg_next_state = i_mask_data[0] ? ST_SEED : ST_SCAN_NEXT;
            ST_SEED: reg_next_state = ST_POP_READ;
            ST_POP_READ: reg_next_state = ST_POP_LOAD;
            ST_POP_LOAD: reg_next_state = ST_NEIGHBOR_ADDRESS;
            ST_NEIGHBOR_ADDRESS: reg_next_state = ST_NEIGHBOR_READ;
            ST_NEIGHBOR_READ: reg_next_state = reg_neighbor_valid ? ST_NEIGHBOR_CHECK : ST_NEIGHBOR_NEXT;
            ST_NEIGHBOR_CHECK: reg_next_state = ST_NEIGHBOR_NEXT;
            ST_NEIGHBOR_NEXT: begin
                if (reg_neighbor == 7)
                    reg_next_state = reg_queue_head < reg_queue_tail ? ST_POP_READ : ST_RANK_INIT;
                else reg_next_state = ST_NEIGHBOR_ADDRESS;
            end
            ST_RANK_INIT: reg_next_state = reg_component_cells >= C_OBJECT_MIN &&
                reg_component_seeds >= C_OBJECT_MIN ? ST_RANK : ST_SCAN_NEXT;
            ST_RANK: if (reg_rank_index == 7) reg_next_state = ST_STORE;
            ST_STORE: reg_next_state = ST_SCAN_NEXT;
            ST_SCAN_NEXT: reg_next_state = reg_scan_index == C_GRID_CELLS-1 ? ST_PUBLISH_INIT : ST_SCAN_READ;
            ST_PUBLISH_INIT: reg_next_state = ST_PUBLISH;
            ST_PUBLISH: if (reg_publish_index == 7) reg_next_state = ST_FINISH;
            ST_FINISH: reg_next_state = ST_IDLE;
            default: reg_next_state = ST_IDLE;
        endcase
    end

    // 队列为一写一读同步 RAM，无数组复位、无异步读出。
    always @(posedge i_clk) begin
        if (wire_queue_seed || wire_enqueue)
            mem_queue[wire_queue_write_address] <= wire_queue_write_data;
        if (reg_state == ST_POP_READ) reg_queue_word <= mem_queue[reg_queue_head];
    end

    // 仅区域统计与八个候选描述符使用寄存器。
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            reg_scan_index <= 0;
            reg_scan_x <= 0; reg_scan_y <= 0;
            reg_queue_head <= 0; reg_queue_tail <= 0;
            reg_center_x <= 0; reg_center_y <= 0; reg_center_index <= 0;
            reg_neighbor <= 0; reg_neighbor_x <= 0; reg_neighbor_y <= 0;
            reg_neighbor_index <= 0; reg_neighbor_valid <= 0;
            reg_component_left <= 0; reg_component_top <= 0;
            reg_component_right <= 0; reg_component_bottom <= 0; reg_component_cells <= 0;
            reg_component_seeds <= 0;
            reg_rank_index <= 0; reg_smallest_index <= 0; reg_smallest_cells <= 0;
            reg_publish_index <= 0; reg_box_valid_o <= 0; reg_count_o <= 0; reg_overflow_o <= 0;
            reg_left_o <= 0; reg_top_o <= 0; reg_right_o <= 0; reg_bottom_o <= 0;
            for (slot = 0; slot < 8; slot = slot + 1) begin
                reg_result_cells[slot] <= 0;
                reg_result_left[slot] <= 0; reg_result_top[slot] <= 0;
                reg_result_right[slot] <= 0; reg_result_bottom[slot] <= 0;
            end
        end else begin
            case (reg_state)
                ST_IDLE: if (i_start) begin
                    reg_scan_index <= 0; reg_scan_x <= 0; reg_scan_y <= 0;
                    reg_box_valid_o <= 0; reg_count_o <= 0; reg_overflow_o <= 0;
                    reg_left_o <= 0; reg_top_o <= 0; reg_right_o <= 0; reg_bottom_o <= 0;
                    for (slot = 0; slot < 8; slot = slot + 1) reg_result_cells[slot] <= 0;
                end
                ST_SEED: begin
                    reg_queue_head <= 0; reg_queue_tail <= 1;
                    reg_component_left <= reg_scan_x; reg_component_right <= reg_scan_x;
                    reg_component_top <= reg_scan_y; reg_component_bottom <= reg_scan_y;
                    reg_component_cells <= 1;
                    reg_component_seeds <= {13'd0,i_mask_data[1]};
                end
                ST_POP_LOAD: begin
                    reg_queue_head <= reg_queue_head + 1'b1;
                    reg_center_x <= reg_queue_word[6:0];
                    reg_center_y <= reg_queue_word[13:7];
                    reg_center_index <= reg_queue_word[13:7] * C_GRID_WIDTH + reg_queue_word[6:0];
                    reg_neighbor <= 0;
                end
                ST_NEIGHBOR_ADDRESS: begin
                    reg_neighbor_x <= wire_neighbor_x; reg_neighbor_y <= wire_neighbor_y;
                    reg_neighbor_index <= wire_neighbor_index;
                    reg_neighbor_valid <= wire_neighbor_valid;
                end
                ST_NEIGHBOR_CHECK: if (i_mask_data[0]) begin
                    reg_queue_tail <= reg_queue_tail + 1'b1;
                    reg_component_cells <= reg_component_cells + 1'b1;
                    reg_component_seeds <= reg_component_seeds + {13'd0,i_mask_data[1]};
                    if (reg_neighbor_x < reg_component_left) reg_component_left <= reg_neighbor_x;
                    if (reg_neighbor_x > reg_component_right) reg_component_right <= reg_neighbor_x;
                    if (reg_neighbor_y < reg_component_top) reg_component_top <= reg_neighbor_y;
                    if (reg_neighbor_y > reg_component_bottom) reg_component_bottom <= reg_neighbor_y;
                end
                ST_NEIGHBOR_NEXT: reg_neighbor <= reg_neighbor + 1'b1;
                ST_RANK_INIT: begin
                    reg_rank_index <= 0; reg_smallest_index <= 0; reg_smallest_cells <= 14'h3fff;
                end
                ST_RANK: begin
                    reg_rank_index <= reg_rank_index + 1'b1;
                    if (reg_result_cells[reg_rank_index] < reg_smallest_cells) begin
                        reg_smallest_cells <= reg_result_cells[reg_rank_index];
                        reg_smallest_index <= reg_rank_index;
                    end
                end
                ST_STORE: begin
                    if (reg_smallest_cells != 0) reg_overflow_o <= 1;
                    if (reg_component_cells > reg_smallest_cells) begin
                        reg_result_cells[reg_smallest_index] <= reg_component_cells;
                        reg_result_left[reg_smallest_index] <= reg_component_left;
                        reg_result_top[reg_smallest_index] <= reg_component_top;
                        reg_result_right[reg_smallest_index] <= reg_component_right;
                        reg_result_bottom[reg_smallest_index] <= reg_component_bottom;
                    end
                end
                ST_SCAN_NEXT: begin
                    reg_scan_index <= reg_scan_index + 1'b1;
                    if (reg_scan_x == C_GRID_WIDTH-1) begin
                        reg_scan_x <= 0; reg_scan_y <= reg_scan_y + 1'b1;
                    end else reg_scan_x <= reg_scan_x + 1'b1;
                end
                ST_PUBLISH_INIT: begin reg_publish_index <= 0; reg_count_o <= 0; end
                ST_PUBLISH: begin
                    reg_publish_index <= reg_publish_index + 1'b1;
                    if (reg_result_cells[reg_publish_index] != 0) begin
                        reg_box_valid_o[reg_publish_index] <= 1;
                        reg_count_o <= reg_count_o + 1'b1;
                        reg_left_o[reg_publish_index*11+:11] <= {reg_result_left[reg_publish_index],4'b0000};
                        reg_top_o[reg_publish_index*11+:11] <= {reg_result_top[reg_publish_index],4'b0000};
                        reg_right_o[reg_publish_index*11+:11] <= {reg_result_right[reg_publish_index],4'b1111};
                        reg_bottom_o[reg_publish_index*11+:11] <= {reg_result_bottom[reg_publish_index],4'b1111};
                    end
                end
                default: begin end
            endcase
        end
    end
endmodule
