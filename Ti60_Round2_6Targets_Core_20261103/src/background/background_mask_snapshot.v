`timescale 1ns / 1ps

// 保存下一完整作业的网格统计；诊断读取不会反压或改变图像处理。
// Capture one complete diagnostic grid and hold it for synchronous host reads.
module background_mask_snapshot #(
    parameter C_CELLS = 9000
) (
    input wire i_clk,
    input wire i_rst_n,
    input wire i_capture,
    input wire i_frame_start,
    input wire i_frame_done,
    input wire i_cell_valid,
    input wire [13:0] i_cell_address,
    input wire [7:0] i_cell_data,
    input wire [13:0] i_read_address,
    output wire [7:0] o_read_data,
    output wire o_ready,
    output wire o_busy,
    output wire [7:0] o_epoch
);
    localparam [1:0] ST_IDLE = 0, ST_ARMED = 1, ST_CAPTURE = 2, ST_HOLD = 3;
    reg [1:0] reg_state, reg_next_state;
    reg [7:0] mem_snapshot [0:C_CELLS-1];
    reg [7:0] reg_read_data_o;
    reg [7:0] reg_epoch_o;

    assign o_read_data = reg_read_data_o;
    assign o_ready = reg_state == ST_HOLD;
    assign o_busy = reg_state == ST_ARMED || reg_state == ST_CAPTURE;
    assign o_epoch = reg_epoch_o;

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) reg_state <= ST_IDLE;
        else reg_state <= reg_next_state;
    end
    always @(*) begin
        reg_next_state = reg_state;
        case (reg_state)
            ST_IDLE, ST_HOLD: if (i_capture)
                reg_next_state = i_frame_start ? ST_CAPTURE : ST_ARMED;
            ST_ARMED: if (i_frame_start) reg_next_state = ST_CAPTURE;
            ST_CAPTURE: if (i_frame_done) reg_next_state = ST_HOLD;
            default: reg_next_state = ST_IDLE;
        endcase
    end
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) reg_epoch_o <= 0;
        else if (reg_state == ST_CAPTURE && i_frame_done)
            reg_epoch_o <= reg_epoch_o + 1'b1;
    end
    // RAM内容无需复位；只有完整作业结束后，协议才允许读取。
    always @(posedge i_clk) begin
        if (reg_state == ST_CAPTURE && i_cell_valid && i_cell_address < C_CELLS)
            mem_snapshot[i_cell_address] <= i_cell_data;
        if (i_read_address < C_CELLS)
            reg_read_data_o <= mem_snapshot[i_read_address];
        else reg_read_data_o <= 0;
    end
endmodule
