`timescale 1ns/1ps

// Y = (306*R + 601*G + 117*B) / 1024, matching the source Sobel project.
module sobel_rgb2gray (
    input              clk,
    input              rst_n,
    input              sink_sop,
    input              sink_valid,
    input      [23:0]  sink_data,
    input              sink_eop,
    output             source_sop,
    output             source_valid,
    output     [7:0]   source_data,
    output             source_eop
);

wire [17:0] y_r;
wire [17:0] y_g;
wire [17:0] y_b;
reg  [17:0] add_rg;
reg  [17:0] add_b;
reg  [17:0] add_rgb;

sobel_rgb2gray_mult u_mult_r (
    .clock  (clk),
    .dataa  (sink_data[23:16]),
    .datab  (10'd306),
    .result (y_r)
);

sobel_rgb2gray_mult u_mult_g (
    .clock  (clk),
    .dataa  (sink_data[15:8]),
    .datab  (10'd601),
    .result (y_g)
);

sobel_rgb2gray_mult u_mult_b (
    .clock  (clk),
    .dataa  (sink_data[7:0]),
    .datab  (10'd117),
    .result (y_b)
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        add_rg  <= 18'd0;
        add_b   <= 18'd0;
        add_rgb <= 18'd0;
    end else begin
        add_rg  <= y_r + y_g;
        add_b   <= y_b;
        add_rgb <= add_rg + add_b;
    end
end

assign source_data = add_rgb[17:10];

sobel_shift_regs #(
    .DWIDTH(3),
    .DELAY_DUTY(7)
) u_ctrl_delay (
    .clk   (clk),
    .rst_n (rst_n),
    .idata ({sink_sop, sink_eop, sink_valid}),
    .odata ({source_sop, source_eop, source_valid})
);

endmodule
