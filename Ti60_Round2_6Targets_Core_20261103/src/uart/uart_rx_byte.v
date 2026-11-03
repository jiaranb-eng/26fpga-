`timescale 1ns / 1ps

// UART byte receiver for the 96 MHz system clock domain.
// Format: 115200 baud, 8 data bits, no parity, one stop bit.
module uart_rx_byte #(
    parameter integer CLK_FREQ_HZ = 96_000_000,
    parameter integer BAUD_RATE   = 115_200,
    parameter integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx_i,
    output reg  [7:0] data_o,
    output reg        data_valid_o,
    output reg        frame_error_o
);

localparam [1:0] ST_IDLE  = 2'd0;
localparam [1:0] ST_START = 2'd1;
localparam [1:0] ST_DATA  = 2'd2;
localparam [1:0] ST_STOP  = 2'd3;
localparam integer HALF_BIT = (CLKS_PER_BIT - 1) / 2;

(* async_reg = "true" *) reg [1:0] rx_sync;
reg [1:0]  state;
reg [15:0] clock_count;
reg [2:0]  bit_index;
reg [7:0]  shift_data;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_sync <= 2'b11;
    end else begin
        rx_sync <= {rx_sync[0], rx_i};
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state         <= ST_IDLE;
        clock_count   <= 16'd0;
        bit_index     <= 3'd0;
        shift_data    <= 8'd0;
        data_o        <= 8'd0;
        data_valid_o  <= 1'b0;
        frame_error_o <= 1'b0;
    end else begin
        data_valid_o  <= 1'b0;
        frame_error_o <= 1'b0;

        case (state)
            ST_IDLE: begin
                clock_count <= 16'd0;
                bit_index   <= 3'd0;
                if (!rx_sync[1]) begin
                    state <= ST_START;
                end
            end

            ST_START: begin
                if (clock_count == HALF_BIT) begin
                    clock_count <= 16'd0;
                    if (!rx_sync[1]) begin
                        state <= ST_DATA;
                    end else begin
                        state <= ST_IDLE;
                    end
                end else begin
                    clock_count <= clock_count + 1'b1;
                end
            end

            ST_DATA: begin
                if (clock_count == CLKS_PER_BIT - 1) begin
                    clock_count          <= 16'd0;
                    shift_data[bit_index] <= rx_sync[1];
                    if (bit_index == 3'd7) begin
                        bit_index <= 3'd0;
                        state     <= ST_STOP;
                    end else begin
                        bit_index <= bit_index + 1'b1;
                    end
                end else begin
                    clock_count <= clock_count + 1'b1;
                end
            end

            ST_STOP: begin
                if (clock_count == CLKS_PER_BIT - 1) begin
                    clock_count <= 16'd0;
                    data_o      <= shift_data;
                    if (rx_sync[1]) begin
                        data_valid_o <= 1'b1;
                    end else begin
                        frame_error_o <= 1'b1;
                    end
                    state <= ST_IDLE;
                end else begin
                    clock_count <= clock_count + 1'b1;
                end
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
