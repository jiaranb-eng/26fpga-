`timescale 1ns/1ps

// Debounces the two active-low core-board keys and updates the Sobel threshold.
// Each stable press produces one step; holding a key never auto-repeats.
module sobel_threshold_ctrl #(
    parameter [9:0] THRESHOLD_DEFAULT = 10'd256,
    parameter [9:0] THRESHOLD_STEP    = 10'd16,
    parameter [9:0] THRESHOLD_MAX     = 10'd960,
    parameter integer DEBOUNCE_CYCLES = 1500000,
    parameter integer COUNTER_WIDTH   = 21
)(
    input             clk,
    input             rst_n,
    input             key_up_n_i,
    input             key_down_n_i,
    output reg [9:0]  threshold_o
);

// The physical keys are asynchronous to clk_pixel. The ASYNC_REG attribute
// preserves the intended two-flop synchronizers during synthesis and P&R.
(* async_reg = "true" *) reg [1:0] key_up_sync;
(* async_reg = "true" *) reg [1:0] key_down_sync;

reg key_up_stable;
reg key_down_stable;
reg [COUNTER_WIDTH-1:0] key_up_count;
reg [COUNTER_WIDTH-1:0] key_down_count;
reg key_up_press;
reg key_down_press;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        key_up_sync   <= 2'b11;
        key_down_sync <= 2'b11;
    end else begin
        key_up_sync   <= {key_up_sync[0], key_up_n_i};
        key_down_sync <= {key_down_sync[0], key_down_n_i};
    end
end

// A state change is accepted only after the synchronized level remains
// unchanged for DEBOUNCE_CYCLES. A pulse is emitted only on the press edge.
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        key_up_stable <= 1'b1;
        key_up_count  <= {COUNTER_WIDTH{1'b0}};
        key_up_press  <= 1'b0;
    end else begin
        key_up_press <= 1'b0;
        if (key_up_sync[1] == key_up_stable) begin
            key_up_count <= {COUNTER_WIDTH{1'b0}};
        end else if (key_up_count == DEBOUNCE_CYCLES - 1) begin
            key_up_stable <= key_up_sync[1];
            key_up_count  <= {COUNTER_WIDTH{1'b0}};
            if (!key_up_sync[1])
                key_up_press <= 1'b1;
        end else begin
            key_up_count <= key_up_count + {{(COUNTER_WIDTH-1){1'b0}}, 1'b1};
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        key_down_stable <= 1'b1;
        key_down_count  <= {COUNTER_WIDTH{1'b0}};
        key_down_press  <= 1'b0;
    end else begin
        key_down_press <= 1'b0;
        if (key_down_sync[1] == key_down_stable) begin
            key_down_count <= {COUNTER_WIDTH{1'b0}};
        end else if (key_down_count == DEBOUNCE_CYCLES - 1) begin
            key_down_stable <= key_down_sync[1];
            key_down_count  <= {COUNTER_WIDTH{1'b0}};
            if (!key_down_sync[1])
                key_down_press <= 1'b1;
        end else begin
            key_down_count <= key_down_count + {{(COUNTER_WIDTH-1){1'b0}}, 1'b1};
        end
    end
end

// Simultaneous valid presses intentionally cancel so the result is deterministic.
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        threshold_o <= THRESHOLD_DEFAULT;
    end else if (key_up_press && !key_down_press) begin
        if (threshold_o >= THRESHOLD_MAX - THRESHOLD_STEP)
            threshold_o <= THRESHOLD_MAX;
        else
            threshold_o <= threshold_o + THRESHOLD_STEP;
    end else if (key_down_press && !key_up_press) begin
        if (threshold_o <= THRESHOLD_STEP)
            threshold_o <= 10'd0;
        else
            threshold_o <= threshold_o - THRESHOLD_STEP;
    end
end

endmodule
