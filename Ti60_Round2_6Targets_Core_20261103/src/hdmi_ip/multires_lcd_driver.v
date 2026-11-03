`timescale 1ns/1ps
// Preserve the accepted FIFO read-ahead and RGB latency, with explicit CE.
// Counters use pixel units; 720p advances every second 148.5 MHz cycle.
module multires_lcd_driver #(parameter DATA_WIDTH=48) (
    input wire clk, input wire rst_n, input wire pixel_enable_i, input wire mode_1080_i,
    output wire lcd_dclk, lcd_blank, lcd_sync,
    output reg lcd_hs, lcd_vs, lcd_en,
    output wire [DATA_WIDTH-1:0] lcd_rgb,
    output wire lcd_request,
    input wire [DATA_WIDTH-1:0] lcd_data
);
    wire [11:0] h_total=mode_1080_i ? 12'd2200 : 12'd1650;
    wire [11:0] h_sync=mode_1080_i ? 12'd44 : 12'd40;
    wire [11:0] h_begin=mode_1080_i ? 12'd192 : 12'd260;
    wire [11:0] h_end=mode_1080_i ? 12'd2112 : 12'd1540;
    wire [10:0] v_total=mode_1080_i ? 11'd1125 : 11'd750;
    wire [10:0] v_begin=mode_1080_i ? 11'd41 : 11'd25;
    wire [10:0] v_end=mode_1080_i ? 11'd1121 : 11'd745;
    reg [11:0] hcnt;
    reg [10:0] vcnt;
    reg request;
    reg [DATA_WIDTH-1:0] rgb;
    wire active_line=vcnt>=v_begin && vcnt<v_end;
    assign lcd_request=request && pixel_enable_i;
    assign lcd_rgb=lcd_en ? rgb : {DATA_WIDTH{1'b0}};
    assign lcd_dclk=~clk;
    assign lcd_blank=lcd_hs && lcd_vs;
    assign lcd_sync=1'b0;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin hcnt<=0; vcnt<=0; lcd_hs<=1; lcd_vs<=1; lcd_en<=0; request<=0; rgb<=0; end
        else if(pixel_enable_i) begin
            if(hcnt==h_total-1'b1) begin
                hcnt<=0;
                if(vcnt==v_total-1'b1) vcnt<=0; else vcnt<=vcnt+1'b1;
                lcd_vs<=vcnt>=5;
            end else hcnt<=hcnt+1'b1;
            lcd_hs<=hcnt>=h_sync;
            lcd_en<=hcnt>=h_begin && hcnt<h_end && active_line;
            request<=hcnt>=h_begin-1'b1 && hcnt<h_end-1'b1 && active_line;
            rgb<=lcd_data;
        end
    end
endmodule
