`timescale 1ns / 1ps

// 背景槽与当前槽均为已提交的完整RAW帧，仅缓存背景的一行。
// Reuses the official-FIFO canny_row_dma read channel at 64 bits per transfer.
module background_frame_worker #(
	parameter C_WIDTH = 1920,
	parameter C_HEIGHT = 1200,
	parameter C_ROW_STRIDE = 2048
) (
    input wire background_mask_capture_i,
    input wire [13:0] background_mask_address_i,
    output wire [7:0] background_mask_data_o,
    output wire background_mask_ready_o, output wire background_mask_busy_o,
    output wire [7:0] background_mask_epoch_o,
	input i_clk,
	input i_rst_n,
	input i_start,
	input [7:0] i_threshold,
	input [31:0] i_current_address,
	input [31:0] i_background_address,
	output o_done,
	output o_read_request,
	input i_read_request_ready,
	output [31:0] o_read_address,
	input i_read_valid,
	output o_read_ready,
	input [63:0] i_read_data,
	output [7:0] o_box_valid,
	output [87:0] o_left,
	output [87:0] o_top,
	output [87:0] o_right,
	output [87:0] o_bottom,
	output [3:0] o_count,
	output o_overflow
);
	localparam C_ROW_WORDS = C_WIDTH/8;
	localparam [2:0] ST_IDLE=0, ST_BACKGROUND_REQUEST=1, ST_BACKGROUND_DATA=2,
		ST_CURRENT_REQUEST=3, ST_CURRENT_DATA=4, ST_WAIT=5, ST_DONE=6;
	reg [2:0] reg_state, reg_next_state; // 整行DMA调度阶段
	reg [31:0] reg_current_address, reg_background_address; // 当前行的两个DDR地址
	reg [10:0] reg_row; // 已处理行号
	reg [8:0] reg_word; // 行内64bit地址
	reg [63:0] mem_background [0:C_ROW_WORDS-1]; // 同步背景行RAM
	reg [63:0] reg_background, reg_current; // RAM读与当前数据对齐
	reg reg_pair_valid; // 同位置像素对有效
	wire wire_grid_ready, wire_grid_done;
	wire wire_data_fire;
	assign o_read_request = reg_state==ST_BACKGROUND_REQUEST || reg_state==ST_CURRENT_REQUEST;
	assign o_read_address = reg_state==ST_BACKGROUND_REQUEST ? reg_background_address : reg_current_address;
	assign o_read_ready = reg_state==ST_BACKGROUND_DATA || (reg_state==ST_CURRENT_DATA && wire_grid_ready);
	assign wire_data_fire = i_read_valid && o_read_ready;
	assign o_done = reg_state==ST_DONE;

	// 状态和下一状态保持独立，所有地址推进均以真实握手为准。
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin reg_state<=ST_IDLE; end
		else begin reg_state<=reg_next_state; end
	end
	always @(*) begin
		reg_next_state=reg_state;
		case (reg_state)
			ST_IDLE: begin if (i_start) begin reg_next_state=ST_BACKGROUND_REQUEST; end end
			ST_BACKGROUND_REQUEST: begin if (i_read_request_ready) begin reg_next_state=ST_BACKGROUND_DATA; end end
			ST_BACKGROUND_DATA: begin if (wire_data_fire && reg_word==C_ROW_WORDS-1) begin reg_next_state=ST_CURRENT_REQUEST; end end
			ST_CURRENT_REQUEST: begin if (i_read_request_ready) begin reg_next_state=ST_CURRENT_DATA; end end
			ST_CURRENT_DATA: begin
				if (wire_data_fire && reg_word==C_ROW_WORDS-1) begin
					reg_next_state=reg_row==C_HEIGHT-1 ? ST_WAIT : ST_BACKGROUND_REQUEST;
				end
			end
			ST_WAIT: begin if (wire_grid_done) begin reg_next_state=ST_DONE; end end
			ST_DONE: begin reg_next_state=ST_IDLE; end
			default: begin reg_next_state=ST_IDLE; end
		endcase
	end

	// 先读满背景行，再用同步读口对齐当前行；RAM不带异步复位。
	always @(posedge i_clk) begin
		if (wire_data_fire && reg_state==ST_BACKGROUND_DATA) begin mem_background[reg_word]<=i_read_data; end
		if (wire_data_fire && reg_state==ST_CURRENT_DATA) begin
			reg_background<=mem_background[reg_word]; reg_current<=i_read_data;
		end
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			reg_current_address<=0; reg_background_address<=0;
			reg_row<=0; reg_word<=0; reg_pair_valid<=0;
		end else begin
			reg_pair_valid<=wire_data_fire && reg_state==ST_CURRENT_DATA;
			if (reg_state==ST_IDLE && i_start) begin
				reg_current_address<=i_current_address; reg_background_address<=i_background_address;
				reg_row<=0; reg_word<=0;
			end else if (wire_data_fire) begin
				if (reg_word==C_ROW_WORDS-1) begin
					reg_word<=0;
					if (reg_state==ST_CURRENT_DATA) begin
						reg_row<=reg_row+1'b1;
						reg_current_address<=reg_current_address+C_ROW_STRIDE;
						reg_background_address<=reg_background_address+C_ROW_STRIDE;
					end
				end else begin reg_word<=reg_word+1'b1; end
			end
		end
	end
	background_grid_detect #(.C_WIDTH(C_WIDTH),.C_HEIGHT(C_HEIGHT)) u_background_grid_detect (
        .background_mask_capture_i(background_mask_capture_i),
        .background_mask_address_i(background_mask_address_i),.background_mask_data_o(background_mask_data_o),
        .background_mask_ready_o(background_mask_ready_o),.background_mask_busy_o(background_mask_busy_o),
        .background_mask_epoch_o(background_mask_epoch_o),
		.i_clk(i_clk),.i_rst_n(i_rst_n),.i_start(i_start && reg_state==ST_IDLE),.i_threshold(i_threshold),
		.i_valid(reg_pair_valid),.o_ready(wire_grid_ready),.i_current(reg_current),.i_background(reg_background),
		.o_done(wire_grid_done),.o_box_valid(o_box_valid),.o_left(o_left),.o_top(o_top),
		.o_right(o_right),.o_bottom(o_bottom),.o_count(o_count),.o_overflow(o_overflow)
	);
endmodule
