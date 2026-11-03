`timescale 1ns/1ps

// Delays an 8-bit raster stream by one row and one column using block RAM.
module sobel_spatial_align_1x1 #(
    parameter WIDTH = 1280,
    parameter HEIGHT = 720,
    parameter BINARY = 0
)(
    input             clk,
    input             rst_n,
    input             sink_sop,
    input             sink_valid,
    input      [7:0]  sink_data,
    output     [7:0]  source_data,
    output            source_window_valid
);

function integer clog2;
    input integer value;
    begin
        value = value - 1;
        for (clog2 = 0; value > 0; clog2 = clog2 + 1)
            value = value >> 1;
    end
endfunction

localparam integer X_COUNT_WIDTH = (WIDTH <= 1) ? 1 : clog2(WIDTH);
localparam integer Y_COUNT_WIDTH = (HEIGHT <= 1) ? 1 : clog2(HEIGHT);
localparam integer ADDR_WIDTH = X_COUNT_WIDTH;

// Binary edges carry one independent bit; retain full raster depth and latency.
localparam integer STORAGE_BITS = BINARY ? 1 : 8;
(* syn_ramstyle = "block_ram" *) reg [STORAGE_BITS-1:0] line_ram [0:WIDTH-1];
reg  [ADDR_WIDTH-1:0] column_addr;
reg  [STORAGE_BITS-1:0] previous_row_q;
reg  [STORAGE_BITS-1:0] previous_row_d1;
reg  [X_COUNT_WIDTH-1:0] xcnt;
reg  [Y_COUNT_WIDTH-1:0] ycnt;
reg                    source_window_valid_r;

wire [ADDR_WIDTH-1:0] access_addr = sink_sop ? {ADDR_WIDTH{1'b0}}
                                                    : column_addr;

// Separate synchronous read and write processes follow the Efinity
// simple-dual-port block-RAM inference style used by sobel_linebuffer.
always @(posedge clk) begin
    if (sink_valid)
        line_ram[access_addr] <= sink_data[STORAGE_BITS-1:0];
end

always @(posedge clk) begin
    if (sink_valid)
        previous_row_q <= line_ram[access_addr];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        previous_row_d1 <= {STORAGE_BITS{1'b0}};
    end else if (sink_valid) begin
        previous_row_d1 <= previous_row_q;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        xcnt                  <= {X_COUNT_WIDTH{1'b0}};
        ycnt                  <= {Y_COUNT_WIDTH{1'b0}};
        column_addr           <= {ADDR_WIDTH{1'b0}};
        source_window_valid_r <= 1'b0;
    end else begin
        source_window_valid_r <= sink_valid && !sink_sop &&
                                 (xcnt >= 1) && (ycnt >= 1);

        if (sink_valid && sink_sop) begin
            column_addr <= (WIDTH > 1) ? {{(ADDR_WIDTH-1){1'b0}}, 1'b1}
                                      : {ADDR_WIDTH{1'b0}};
            xcnt <= (WIDTH > 1) ? {{(X_COUNT_WIDTH-1){1'b0}}, 1'b1}
                                : {X_COUNT_WIDTH{1'b0}};
            ycnt <= {Y_COUNT_WIDTH{1'b0}};
        end else if (sink_valid && (xcnt >= WIDTH - 1)) begin
            column_addr <= {ADDR_WIDTH{1'b0}};
            xcnt <= {X_COUNT_WIDTH{1'b0}};
            ycnt <= (ycnt >= HEIGHT - 1) ? {Y_COUNT_WIDTH{1'b0}}
                                         : ycnt + 1'b1;
        end else if (sink_valid) begin
            column_addr <= column_addr + 1'b1;
            xcnt <= xcnt + 1'b1;
        end
    end
end

assign source_data         = BINARY ? {8{previous_row_d1[0]}} : previous_row_d1;
assign source_window_valid = source_window_valid_r;

endmodule
