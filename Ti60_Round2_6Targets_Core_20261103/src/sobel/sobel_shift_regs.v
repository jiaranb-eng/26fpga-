`timescale 1ns/1ps

module sobel_shift_regs #(
    parameter DWIDTH = 1,
    parameter DELAY_DUTY = 1
)(
    input                    clk,
    input                    rst_n,
    input      [DWIDTH-1:0]  idata,
    output     [DWIDTH-1:0]  odata
);

reg [DWIDTH-1:0] idata_r [0:DELAY_DUTY-1];
integer i;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < DELAY_DUTY; i = i + 1)
            idata_r[i] <= {DWIDTH{1'b0}};
    end else begin
        idata_r[0] <= idata;
        for (i = 1; i < DELAY_DUTY; i = i + 1)
            idata_r[i] <= idata_r[i-1];
    end
end

assign odata = idata_r[DELAY_DUTY-1];

endmodule
