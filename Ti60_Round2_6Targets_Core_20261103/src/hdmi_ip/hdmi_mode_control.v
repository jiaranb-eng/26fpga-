`timescale 1ns/1ps
// Selection is already debounced and transferred to this clock domain.
// Disable the physical lanes while restarting all display-coordinate stages.
// Camera capture, Canny processing, DDR and its AXI transactions keep running.
module hdmi_mode_control #(parameter SETTLE_CYCLES=148500) (
    input wire clk_i, input wire rst_n_i, input wire [2:0] requested_mode_i,
    output reg [2:0] mode_o, output reg mode_1080_o, output reg mode_raw_o, output wire display_rst_n_o,
    output wire pixel_enable_o
);
    localparam COUNT_BITS=$clog2(SETTLE_CYCLES+1);
    reg [COUNT_BITS-1:0] settle_count;
    reg half_pixel;
    reg display_ready;
    assign display_rst_n_o=display_ready;
    assign pixel_enable_o=display_rst_n_o && (mode_1080_o || half_pixel);
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin mode_o<=0; mode_1080_o<=0; mode_raw_o<=0; settle_count<=SETTLE_CYCLES; half_pixel<=0; display_ready<=0; end
        else begin
            if(requested_mode_i!=mode_o) begin
                mode_o<=requested_mode_i; settle_count<=SETTLE_CYCLES; half_pixel<=0; display_ready<=0;
                // Register single-bit mode flags before crossing to camera/AXI.
                mode_1080_o<=requested_mode_i!=0;
                mode_raw_o<=requested_mode_i==2 || requested_mode_i==4 || requested_mode_i==5 || requested_mode_i==6;
            end else if(settle_count!=0) begin settle_count<=settle_count-1'b1; half_pixel<=0; end
            else begin half_pixel<=!half_pixel; display_ready<=1; end
        end
    end
endmodule
