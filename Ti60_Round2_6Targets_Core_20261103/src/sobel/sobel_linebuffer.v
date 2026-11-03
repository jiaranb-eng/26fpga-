`timescale 1ns/1ps

// Two-line video buffer implemented as two ping-pong block RAM banks.
//
// Efinity 2025.1 maps the synchronous RAM template below to Titanium
// EFX_RAM10 primitives. At each accepted pixel both banks are read at the
// current column while the older bank is overwritten with the current row.
// READ_FIRST behavior therefore returns rows y-1 and y-2 without a third RAM
// port. The registered RAM read adds one clock of visible latency.
module sobel_linebuffer #(
    parameter DISTANCE = 1280,
    parameter DWIDTH = 8,
    parameter TAPS_NUM = 2
)(
    input                              clken,
    input                              clock,
    input                              reset_n,
    input                              frame_start,
    input      [DWIDTH-1:0]            shiftin,
    output     [DWIDTH-1:0]            shiftout,
    output     [TAPS_NUM*DWIDTH-1:0]   taps
);

function integer clog2;
    input integer value;
    begin
        value = value - 1;
        for (clog2 = 0; value > 0; clog2 = clog2 + 1)
            value = value >> 1;
    end
endfunction

localparam integer ADDR_WIDTH = clog2(DISTANCE);

// This module intentionally supports exactly the two taps required by the
// causal 3x3 window. Keeping the banks independent gives each RAM one read
// and one write access per accepted pixel.
(* syn_ramstyle = "block_ram" *) reg [DWIDTH-1:0] line_ram_even [0:DISTANCE-1];
(* syn_ramstyle = "block_ram" *) reg [DWIDTH-1:0] line_ram_odd  [0:DISTANCE-1];

reg [ADDR_WIDTH-1:0] column_addr;
reg                  write_odd;
reg                  read_write_odd;
reg [DWIDTH-1:0]     even_q;
reg [DWIDTH-1:0]     odd_q;

wire [ADDR_WIDTH-1:0] access_addr = frame_start ? {ADDR_WIDTH{1'b0}}
                                                   : column_addr;
wire access_write_odd = frame_start ? 1'b0 : write_odd;

// Separate registered read and write processes match the official Efinity
// simple-dual-port inference template. Nonblocking writes preserve the old
// word on a same-address read (READ_FIRST), which is the y-2 sample.
always @(posedge clock) begin
    if (clken && !access_write_odd)
        line_ram_even[access_addr] <= shiftin;
end

always @(posedge clock) begin
    if (clken && access_write_odd)
        line_ram_odd[access_addr] <= shiftin;
end

always @(posedge clock) begin
    if (clken) begin
        even_q <= line_ram_even[access_addr];
        odd_q  <= line_ram_odd[access_addr];
    end
end

always @(posedge clock or negedge reset_n) begin
    if (!reset_n) begin
        column_addr    <= {ADDR_WIDTH{1'b0}};
        write_odd      <= 1'b0;
        read_write_odd <= 1'b0;
    end else if (clken) begin
        read_write_odd <= access_write_odd;

        if (frame_start) begin
            column_addr <= (DISTANCE > 1) ? {{(ADDR_WIDTH-1){1'b0}}, 1'b1}
                                          : {ADDR_WIDTH{1'b0}};
            write_odd   <= 1'b0;
        end else if (column_addr == DISTANCE - 1) begin
            column_addr <= {ADDR_WIDTH{1'b0}};
            write_odd   <= ~write_odd;
        end else begin
            column_addr <= column_addr + 1'b1;
        end
    end
end

wire [DWIDTH-1:0] previous_line = read_write_odd ? even_q : odd_q;
wire [DWIDTH-1:0] previous_two  = read_write_odd ? odd_q  : even_q;

assign taps[DWIDTH-1:0]        = previous_line;
assign taps[2*DWIDTH-1:DWIDTH] = previous_two;
assign shiftout                = previous_two;

endmodule
