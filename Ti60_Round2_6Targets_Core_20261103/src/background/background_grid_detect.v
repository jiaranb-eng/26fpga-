`timescale 1ns / 1ps

// 固定背景同位置 RAW8 差分，强差分或大面积同方向弱差分通过单元投票。
// Same-position RAW differences with dense signed votes for low-contrast objects.
// 掩码和列累加使用同步RAM；每帧完整覆盖后才扫描，不复位大数组。
module background_grid_detect #(
	parameter C_WIDTH = 1920,
	parameter C_HEIGHT = 1200,
	parameter C_CELL_MIN = 24,
	parameter C_OBJECT_MIN = 12
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
	input i_valid,
	output o_ready,
	input [63:0] i_current,
	input [63:0] i_background,
	output o_done,
	output [7:0] o_box_valid,
	output [87:0] o_left,
	output [87:0] o_top,
	output [87:0] o_right,
	output [87:0] o_bottom,
	output [3:0] o_count,
	output o_overflow
);
    localparam C_ROW_WORDS = C_WIDTH / 8;
    localparam C_GRID_WIDTH = C_WIDTH / 16;
    localparam C_GRID_HEIGHT = C_HEIGHT / 16;
    localparam C_GRID_CELLS = C_GRID_WIDTH * C_GRID_HEIGHT;
    localparam [2:0] ST_IDLE = 0, ST_PIXELS = 1, ST_DRAIN = 2, ST_SCAN = 3, ST_FINISH = 4;
    reg [2:0] reg_state, reg_next_state;
	reg [8:0] reg_word; // 当前行八像素组地址
	reg [10:0] reg_row; // 全视野输入行号
	reg [13:0] reg_mask_base; // 当前十六行单元带的掩码基址
	reg [7:0] reg_threshold; // 帧内固定差分阈值
	reg [7:0] reg_weak_threshold; // 弱差分门槛 max(3, threshold/4)，帧内固定
	reg [7:0] reg_mean_threshold; // 支持票门槛 max(2, threshold/8)
	reg signed [11:0] reg_mean_limit; // 每颜色64样点的均值比较门槛，最大31*64=1984
	reg [3:0] reg_drain; // 最后输入之后的流水排空计数
	reg [4:0] reg_valid_pipe; // 比较与加法树有效标记
	reg [4:0] reg_half_pipe, reg_last_row_pipe;
	reg [4:0] reg_row_parity_pipe; // 保持两个Bayer行相位，不把不同颜色求平均
	reg [6:0] reg_column_pipe [0:4]; // 每级保持对应单元列
	reg [13:0] reg_index_pipe [0:4]; // 每级保持对应掩码地址
	reg [71:0] reg_accum_pipe [0:4]; // {支持7,四颜色和44,变暗8,变亮8,强票5}
	reg [71:0] mem_accum [0:C_GRID_WIDTH-1]; // 同步列RAM，仅存一条16行单元带
	reg [1:0] mem_mask [0:C_GRID_CELLS-1]; // {强证据种子,候选支持}，扫描时一起清除
	reg [63:0] reg_current, reg_background; // 输入采样寄存器
	reg [7:0] reg_difference [0:7]; // 八个同色感光点的无符号绝对差
	reg [7:0] reg_brighter; // 与绝对差同级的变化方向；相等时不产生弱差分票
	reg [1:0] reg_pairs [0:3]; // 两两前景判断相加
	reg [1:0] reg_bright_pairs [0:3], reg_dark_pairs [0:3]; // 正负方向分别计票，不相加抵消噪声
	reg [1:0] reg_support_pairs [0:3];
	wire signed [4:0] wire_clip [0:7]; // 单样点截断到正负8，抑制异常亮点主导均值
	reg signed [5:0] reg_mean_pairs [0:3];
	reg signed [6:0] reg_mean_halves [0:1];
	reg signed [6:0] reg_mean_hits [0:1], reg_first_mean [0:1];
	reg [2:0] reg_halves [0:1]; // 四像素前景计数
	reg [2:0] reg_bright_halves [0:1], reg_dark_halves [0:1];
	reg [2:0] reg_support_halves [0:1];
	reg [3:0] reg_hits, reg_first_hits; // 八像素计数及单元左半行计数
	reg [3:0] reg_bright_hits, reg_dark_hits, reg_first_bright, reg_first_dark;
	reg [3:0] reg_support_hits, reg_first_support;
	reg reg_commit_valid, reg_commit_last_row;
	reg [5:0] reg_commit_total; // 单元累计，生产配置最多24+16=40
	reg [8:0] reg_commit_bright, reg_commit_dark; // 方向票累计上界160+16=176
	reg [7:0] reg_commit_support; // 支持票饱和64，提交上界80
	reg signed [10:0] reg_commit_mean [0:3]; // 每颜色64样点，截断后和为-512..512
	reg [6:0] reg_commit_column;
	reg [13:0] reg_commit_index;
    reg [1:0] reg_mask_value;
    wire wire_fire;
    wire wire_mask_read, wire_mask_clear, wire_scan_done;
    wire [13:0] wire_mask_address;
    wire wire_mask_pixel_write = reg_commit_valid && reg_commit_last_row;
    wire wire_mask_write = wire_mask_pixel_write || wire_mask_clear;
    wire [13:0] wire_mask_write_address = wire_mask_pixel_write ? reg_commit_index : wire_mask_address;
    wire wire_coherent = reg_threshold != 8'd255 &&
        (reg_commit_bright >= 9'd160 || reg_commit_dark >= 9'd160);
    wire wire_seed = reg_commit_total >= C_CELL_MIN || wire_coherent;
    wire wire_mean_change =
        reg_commit_mean[0] > reg_mean_limit || reg_commit_mean[0] < -reg_mean_limit ||
        reg_commit_mean[1] > reg_mean_limit || reg_commit_mean[1] < -reg_mean_limit ||
        reg_commit_mean[2] > reg_mean_limit || reg_commit_mean[2] < -reg_mean_limit ||
        reg_commit_mean[3] > reg_mean_limit || reg_commit_mean[3] < -reg_mean_limit;
    wire wire_support = reg_threshold != 8'd255 && reg_commit_support >= 8'd64 && wire_mean_change;
    wire [1:0] wire_mask_write_data = wire_mask_pixel_write ?
        {wire_seed,(wire_seed || wire_support)} : 2'b00;
    wire [10:0] wire_diag_abs [0:3];
    wire [10:0] wire_diag_pair_low = wire_diag_abs[0] > wire_diag_abs[1] ? wire_diag_abs[0] : wire_diag_abs[1];
    wire [10:0] wire_diag_pair_high = wire_diag_abs[2] > wire_diag_abs[3] ? wire_diag_abs[2] : wire_diag_abs[3];
    wire [10:0] wire_diag_peak = wire_diag_pair_low > wire_diag_pair_high ? wire_diag_pair_low : wire_diag_pair_high;
    wire [10:0] wire_diag_ceiling = wire_diag_peak + 11'd15;
    wire [4:0] wire_diag_mean = wire_diag_ceiling >= 11'd512 ? 5'd31 : wire_diag_ceiling[8:4];
    wire [7:0] wire_diag_data = {wire_diag_mean,reg_commit_support >= 8'd32,reg_commit_support >= 8'd64,wire_seed};
    integer lane, stage;
    genvar diag_color;
    generate for (diag_color=0; diag_color<4; diag_color=diag_color+1) begin: diagnostic_absolute_means
        assign wire_diag_abs[diag_color] = reg_commit_mean[diag_color][10] ?
            -reg_commit_mean[diag_color] : reg_commit_mean[diag_color];
    end endgenerate
    background_mask_snapshot #(.C_CELLS(C_GRID_CELLS)) u_mask_snapshot (
        .i_clk(i_clk),.i_rst_n(i_rst_n),.i_capture(background_mask_capture_i),
        .i_frame_start(i_start && reg_state == ST_IDLE),.i_frame_done(wire_scan_done),
        .i_cell_valid(wire_mask_pixel_write),.i_cell_address(reg_commit_index),.i_cell_data(wire_diag_data),
        .i_read_address(background_mask_address_i),.o_read_data(background_mask_data_o),
        .o_ready(background_mask_ready_o),.o_busy(background_mask_busy_o),.o_epoch(background_mask_epoch_o)
    );
    genvar clip_lane;

    generate for (clip_lane=0; clip_lane<8; clip_lane=clip_lane+1) begin: clipped_color_samples
        assign wire_clip[clip_lane] = reg_difference[clip_lane] > 8'd8 ?
            (reg_brighter[clip_lane] ? 5'sd8 : -5'sd8) :
            (reg_brighter[clip_lane] ? $signed({1'b0,reg_difference[clip_lane][3:0]}) :
                -$signed({1'b0,reg_difference[clip_lane][3:0]}));
    end endgenerate

    assign o_ready = reg_state == ST_PIXELS;
    assign o_done = reg_state == ST_FINISH;
    assign wire_fire = i_valid && o_ready;

    background_component_scan #(
        .C_GRID_WIDTH(C_GRID_WIDTH), .C_GRID_HEIGHT(C_GRID_HEIGHT), .C_OBJECT_MIN(C_OBJECT_MIN)
    ) u_component_scan (
        .i_clk(i_clk), .i_rst_n(i_rst_n), .i_start(reg_state == ST_DRAIN && reg_drain == 8),
        .o_mask_read(wire_mask_read), .o_mask_clear(wire_mask_clear),
        .o_mask_address(wire_mask_address), .i_mask_data(reg_mask_value),
        .o_done(wire_scan_done), .o_box_valid(o_box_valid),
        .o_left(o_left), .o_top(o_top), .o_right(o_right), .o_bottom(o_bottom),
        .o_count(o_count), .o_overflow(o_overflow)
    );

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) reg_state <= ST_IDLE;
        else reg_state <= reg_next_state;
    end
    always @(*) begin
        reg_next_state = reg_state;
        case (reg_state)
            ST_IDLE: if (i_start) reg_next_state = ST_PIXELS;
            ST_PIXELS: if (wire_fire && reg_word == C_ROW_WORDS-1 && reg_row == C_HEIGHT-1)
                reg_next_state = ST_DRAIN;
            ST_DRAIN: if (reg_drain == 8) reg_next_state = ST_SCAN;
            ST_SCAN: if (wire_scan_done) reg_next_state = ST_FINISH;
            ST_FINISH: reg_next_state = ST_IDLE;
            default: reg_next_state = ST_IDLE;
        endcase
    end

    // 像素期完整覆盖掩码；扫描期读出并在入队时清除已访问单元。
    always @(posedge i_clk) begin
        if (wire_fire)
            reg_accum_pipe[0] <= reg_row[3:0] == 0 ? 72'd0 : mem_accum[reg_word >> 1];
        if (reg_commit_valid)
            mem_accum[reg_commit_column] <= {
                reg_commit_support >= 8'd64 ? 7'd64 : reg_commit_support[6:0],
                reg_commit_mean[3],reg_commit_mean[2],reg_commit_mean[1],reg_commit_mean[0],
                reg_commit_dark >= 9'd160 ? 8'd160 : reg_commit_dark[7:0],
                reg_commit_bright >= 9'd160 ? 8'd160 : reg_commit_bright[7:0],
                reg_commit_total >= C_CELL_MIN ? C_CELL_MIN[4:0] : reg_commit_total[4:0]};
        // 显式复用唯一写口，避免Efinity把像素写与访问清除推断为双写口。
        if (wire_mask_write) mem_mask[wire_mask_write_address] <= wire_mask_write_data;
        if (wire_mask_read) reg_mask_value <= mem_mask[wire_mask_address];
    end
	// 绝对差、两级加法树和元数据同步推进，空拍不会改变像素位置。
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			reg_valid_pipe <= 0; reg_half_pipe <= 0;
			reg_row_parity_pipe <= 0;
			reg_last_row_pipe <= 0;
			reg_commit_valid <= 0; reg_commit_last_row <= 0; reg_commit_total <= 0;
			reg_commit_bright <= 0; reg_commit_dark <= 0; reg_brighter <= 0;
			reg_commit_support <= 0; reg_support_hits <= 0; reg_first_support <= 0;
			reg_bright_hits <= 0; reg_dark_hits <= 0; reg_first_bright <= 0; reg_first_dark <= 0;
			reg_commit_column <= 0; reg_commit_index <= 0;
			reg_current <= 0; reg_background <= 0; reg_hits <= 0; reg_first_hits <= 0;
			for (stage=0; stage<5; stage=stage+1) begin
				reg_column_pipe[stage] <= 0; reg_index_pipe[stage] <= 0;
			end
			for (lane=0; lane<8; lane=lane+1) begin reg_difference[lane] <= 0; end
			for (lane=0; lane<4; lane=lane+1) begin
				reg_pairs[lane] <= 0; reg_bright_pairs[lane] <= 0; reg_dark_pairs[lane] <= 0;
				reg_support_pairs[lane] <= 0; reg_mean_pairs[lane] <= 0; reg_commit_mean[lane] <= 0;
			end
			reg_halves[0] <= 0; reg_halves[1] <= 0;
			reg_bright_halves[0] <= 0; reg_bright_halves[1] <= 0;
			reg_dark_halves[0] <= 0; reg_dark_halves[1] <= 0;
			for (lane=0; lane<2; lane=lane+1) begin
				reg_support_halves[lane] <= 0; reg_mean_halves[lane] <= 0;
				reg_mean_hits[lane] <= 0; reg_first_mean[lane] <= 0;
			end
		end else begin
			reg_valid_pipe <= {reg_valid_pipe[3:0],wire_fire};
			reg_half_pipe <= {reg_half_pipe[3:0],reg_word[0]};
			reg_row_parity_pipe <= {reg_row_parity_pipe[3:0],reg_row[0]};
			reg_last_row_pipe <= {reg_last_row_pipe[3:0],reg_row[3:0] == 15};
			reg_column_pipe[0] <= reg_word[7:1];
			reg_index_pipe[0] <= reg_mask_base + (reg_word >> 1);
			for (stage=1; stage<5; stage=stage+1) begin
				reg_column_pipe[stage] <= reg_column_pipe[stage-1];
				reg_index_pipe[stage] <= reg_index_pipe[stage-1];
			end
			if (wire_fire) begin reg_current <= i_current; reg_background <= i_background; end
			for (lane=0; lane<8; lane=lane+1) begin
				reg_brighter[lane] <= reg_current[lane*8+:8] > reg_background[lane*8+:8];
				reg_difference[lane] <= reg_current[lane*8+:8] >= reg_background[lane*8+:8] ?
					reg_current[lane*8+:8] - reg_background[lane*8+:8] : reg_background[lane*8+:8] - reg_current[lane*8+:8];
			end
			for (lane=0; lane<4; lane=lane+1) begin
				reg_pairs[lane] <= {1'b0,reg_difference[lane*2] > reg_threshold} + {1'b0,reg_difference[lane*2+1] > reg_threshold};
				reg_bright_pairs[lane] <=
					{1'b0,(reg_difference[lane*2] > reg_weak_threshold && reg_brighter[lane*2])} +
					{1'b0,(reg_difference[lane*2+1] > reg_weak_threshold && reg_brighter[lane*2+1])};
				reg_dark_pairs[lane] <=
					{1'b0,(reg_difference[lane*2] > reg_weak_threshold && !reg_brighter[lane*2])} +
					{1'b0,(reg_difference[lane*2+1] > reg_weak_threshold && !reg_brighter[lane*2+1])};
				reg_support_pairs[lane] <= {1'b0,reg_difference[lane*2] > reg_mean_threshold} +
					{1'b0,reg_difference[lane*2+1] > reg_mean_threshold};
				reg_mean_pairs[lane] <=
					$signed({wire_clip[(lane/2)*4+lane%2][4],wire_clip[(lane/2)*4+lane%2]}) +
					$signed({wire_clip[(lane/2)*4+lane%2+2][4],wire_clip[(lane/2)*4+lane%2+2]});
			end
			reg_halves[0] <= {1'b0,reg_pairs[0]} + {1'b0,reg_pairs[1]};
			reg_halves[1] <= {1'b0,reg_pairs[2]} + {1'b0,reg_pairs[3]};
			reg_hits <= {1'b0,reg_halves[0]} + {1'b0,reg_halves[1]};
			reg_bright_halves[0] <= {1'b0,reg_bright_pairs[0]} + {1'b0,reg_bright_pairs[1]};
			reg_bright_halves[1] <= {1'b0,reg_bright_pairs[2]} + {1'b0,reg_bright_pairs[3]};
			reg_dark_halves[0] <= {1'b0,reg_dark_pairs[0]} + {1'b0,reg_dark_pairs[1]};
			reg_dark_halves[1] <= {1'b0,reg_dark_pairs[2]} + {1'b0,reg_dark_pairs[3]};
			reg_bright_hits <= {1'b0,reg_bright_halves[0]} + {1'b0,reg_bright_halves[1]};
			reg_dark_hits <= {1'b0,reg_dark_halves[0]} + {1'b0,reg_dark_halves[1]};
			reg_support_halves[0] <= {1'b0,reg_support_pairs[0]} + {1'b0,reg_support_pairs[1]};
			reg_support_halves[1] <= {1'b0,reg_support_pairs[2]} + {1'b0,reg_support_pairs[3]};
			reg_support_hits <= {1'b0,reg_support_halves[0]} + {1'b0,reg_support_halves[1]};
			reg_mean_halves[0] <= $signed({reg_mean_pairs[0][5],reg_mean_pairs[0]}) +
				$signed({reg_mean_pairs[2][5],reg_mean_pairs[2]});
			reg_mean_halves[1] <= $signed({reg_mean_pairs[1][5],reg_mean_pairs[1]}) +
				$signed({reg_mean_pairs[3][5],reg_mean_pairs[3]});
			reg_mean_hits[0] <= reg_mean_halves[0]; reg_mean_hits[1] <= reg_mean_halves[1];
			reg_commit_valid <= reg_valid_pipe[4] && reg_half_pipe[4];
			reg_commit_last_row <= reg_last_row_pipe[4];
			reg_commit_column <= reg_column_pipe[4]; reg_commit_index <= reg_index_pipe[4];
			reg_commit_total <= {1'b0,reg_accum_pipe[4][4:0]} + {2'b0,reg_first_hits} + {2'b0,reg_hits};
			reg_commit_bright <= {1'b0,reg_accum_pipe[4][12:5]} + {5'd0,reg_first_bright} + {5'd0,reg_bright_hits};
			reg_commit_dark <= {1'b0,reg_accum_pipe[4][20:13]} + {5'd0,reg_first_dark} + {5'd0,reg_dark_hits};
			reg_commit_support <= {1'b0,reg_accum_pipe[4][71:65]} + {4'd0,reg_first_support} + {4'd0,reg_support_hits};
			for (lane=0; lane<4; lane=lane+1) begin
				if (reg_row_parity_pipe[4] == (lane/2))
					reg_commit_mean[lane] <= $signed(reg_accum_pipe[4][21+lane*11+:11]) +
						$signed({{4{reg_first_mean[lane%2][6]}},reg_first_mean[lane%2]}) +
						$signed({{4{reg_mean_hits[lane%2][6]}},reg_mean_hits[lane%2]});
				else reg_commit_mean[lane] <= reg_accum_pipe[4][21+lane*11+:11];
			end
			if (reg_valid_pipe[4] && !reg_half_pipe[4]) begin
				reg_first_hits <= reg_hits; reg_first_bright <= reg_bright_hits; reg_first_dark <= reg_dark_hits;
				reg_first_support <= reg_support_hits;
				reg_first_mean[0] <= reg_mean_hits[0]; reg_first_mean[1] <= reg_mean_hits[1];
			end
		end
	end

	// 列累加RAM读数据经过相同五级管线，无复位RAM内容的必要。
	always @(posedge i_clk) begin
		for (stage=1; stage<5; stage=stage+1) begin reg_accum_pipe[stage] <= reg_accum_pipe[stage-1]; end
	end


    // 像素地址和阈值只在真实输入握手或帧开始边界更新。
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            reg_word <= 0; reg_row <= 0; reg_mask_base <= 0; reg_threshold <= 12; reg_drain <= 0;
            reg_weak_threshold <= 3;
            reg_mean_threshold <= 2; reg_mean_limit <= 12'sd128;
        end else begin
            case (reg_state)
                ST_IDLE: if (i_start) begin
                    reg_word <= 0; reg_row <= 0; reg_mask_base <= 0; reg_threshold <= i_threshold; reg_drain <= 0;
                    reg_weak_threshold <= i_threshold[7:2] < 6'd3 ? 8'd3 : {2'd0,i_threshold[7:2]};
                    reg_mean_threshold <= i_threshold[7:3] < 5'd2 ? 8'd2 : {3'd0,i_threshold[7:3]};
                    reg_mean_limit <= i_threshold[7:3] < 5'd2 ? 12'sd128 : $signed({1'b0,i_threshold[7:3],6'd0});
                end
                ST_PIXELS: if (wire_fire) begin
                    if (reg_word == C_ROW_WORDS-1) begin
                        reg_word <= 0; reg_row <= reg_row + 1'b1;
                        if (reg_row[3:0] == 15) reg_mask_base <= reg_mask_base + C_GRID_WIDTH;
                    end else reg_word <= reg_word + 1'b1;
                end
                ST_DRAIN: reg_drain <= reg_drain + 1'b1;
                default: begin end
            endcase
        end
    end
endmodule
