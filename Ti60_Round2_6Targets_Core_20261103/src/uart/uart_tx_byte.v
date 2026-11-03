`timescale 1ns / 1ps

// UART byte transmitter with a ready/valid byte interface.
module uart_tx_byte #(
    parameter integer CLK_FREQ_HZ = 96_000_000,
    parameter integer BAUD_RATE   = 115_200,
    parameter integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data_i,
    input  wire       data_valid_i,
    output wire       data_ready_o,
    output reg        tx_o
);

reg        busy;
reg [15:0] clock_count;
reg [3:0]  bit_index;
reg [9:0]  shift_frame;

assign data_ready_o = !busy;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        busy        <= 1'b0;
        clock_count <= 16'd0;
        bit_index   <= 4'd0;
        shift_frame <= 10'h3ff;
        tx_o        <= 1'b1;
    end else if (!busy) begin
        tx_o        <= 1'b1;
        clock_count <= 16'd0;
        bit_index   <= 4'd0;
        if (data_valid_i) begin
            shift_frame <= {1'b1, data_i, 1'b0};
            tx_o        <= 1'b0;
            busy        <= 1'b1;
        end
    end else if (clock_count == CLKS_PER_BIT - 1) begin
        clock_count <= 16'd0;
        if (bit_index == 4'd9) begin
            busy      <= 1'b0;
            bit_index <= 4'd0;
            tx_o      <= 1'b1;
        end else begin
            shift_frame <= {1'b1, shift_frame[9:1]};
            tx_o        <= shift_frame[1];
            bit_index   <= bit_index + 1'b1;
        end
    end else begin
        clock_count <= clock_count + 1'b1;
    end
end

endmodule
