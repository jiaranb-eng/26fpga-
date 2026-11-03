`timescale 1ns / 1ps

// 完整帧槽所有权；仅保存索引和计数，不存储像素，也不替代 DDR IP。
// 写入器必须在最终 B 响应完成后给出 publish_i；boundary_i 必须在旧读事务排空后给出。
module frame_history_ring #(
    parameter C_SLOT_BITS = 5
) (
    input wire clk_i,
    input wire rst_n_i,
    input wire publish_i,
    input wire kind_i,
    input wire boundary_i,
    input wire [1:0] control_i,
    input wire [7:0] age_a_i,
    input wire [7:0] age_b_i,
    output wire [C_SLOT_BITS-1:0] write_slot_o,
    output wire [C_SLOT_BITS-1:0] read_slot_a_o,
    output wire [C_SLOT_BITS-1:0] read_slot_b_o,
    output wire compare_o,
    output wire valid_o,
    output wire [7:0] count_o
);
    localparam [C_SLOT_BITS-1:0] LAST_COUNT = {C_SLOT_BITS{1'b1}};
    reg [C_SLOT_BITS-1:0] reg_writer = 0;
    reg [C_SLOT_BITS-1:0] reg_latest = 0;
    reg [C_SLOT_BITS-1:0] cnt_history = 0;
    reg [C_SLOT_BITS-1:0] reg_reader_a = 0;
    reg [C_SLOT_BITS-1:0] reg_reader_b = 0;
    reg flag_compare = 0;
    reg flag_valid = 0;
    reg reg_kind = 0;
    wire [C_SLOT_BITS-1:0] next_writer;
    wire flag_commit;
    wire [7:0] oldest_age;
    wire [7:0] selected_age_a;
    wire [7:0] selected_age_b;

    assign next_writer = reg_writer + 1'b1;
    assign flag_commit = publish_i && control_i == 2'd0 &&
        next_writer != reg_reader_a && next_writer != reg_reader_b;
    assign oldest_age = {{(8-C_SLOT_BITS){1'b0}},cnt_history} - 8'd1;
    assign selected_age_a = control_i == 2'd0 ? 8'd0 :
        (age_a_i < count_o ? age_a_i : oldest_age);
    assign selected_age_b = control_i == 2'd2 ?
        (age_b_i < count_o ? age_b_i : oldest_age) : selected_age_a;
    assign write_slot_o = reg_writer;
    assign read_slot_a_o = reg_reader_a;
    assign read_slot_b_o = reg_reader_b;
    assign compare_o = flag_compare;
    assign valid_o = flag_valid;
    assign count_o = {{(8-C_SLOT_BITS){1'b0}},cnt_history};

    // 发布与冻结同周期时冻结优先，当前写槽成为可反复覆盖的临时槽。
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            reg_writer <= 0;
            reg_latest <= 0;
            cnt_history <= 0;
            reg_kind <= 0;
        end else if (flag_commit) begin
            reg_latest <= reg_writer;
            reg_writer <= next_writer;
            reg_kind <= kind_i;
            if (kind_i != reg_kind) cnt_history <= {{(C_SLOT_BITS-1){1'b0}},1'b1};
            else if (cnt_history != LAST_COUNT) cnt_history <= cnt_history + 1'b1;
        end
    end

    // 读槽在完整显示周期内不变；同周期发布保守使用上一帧，避免组合旁路。
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            reg_reader_a <= {{(C_SLOT_BITS-2){1'b0}},2'd2};
            reg_reader_b <= {{(C_SLOT_BITS-2){1'b0}},2'd2};
            flag_compare <= 0;
            flag_valid <= 0;
        end else if (boundary_i) begin
            flag_valid <= cnt_history != 0;
            flag_compare <= control_i == 2'd2;
            if (cnt_history != 0) begin
                reg_reader_a <= reg_latest - selected_age_a[C_SLOT_BITS-1:0];
                reg_reader_b <= reg_latest - selected_age_b[C_SLOT_BITS-1:0];
            end
        end
    end
endmodule
