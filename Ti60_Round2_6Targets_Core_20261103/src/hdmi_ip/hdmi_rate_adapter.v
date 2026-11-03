`timescale 1ns/1ps
// The official 10:1 half-rate serializer always runs at 1.485 Gbit/s.
// 720p repeats each TMDS bit twice, including the forwarded clock lane.
// Its electrical stream is therefore exactly a 742.5 Mbit/s TMDS stream.
// Bit zero is serialized first, as in the board's accepted HDMI path.
module hdmi_rate_adapter (
    input wire clk_i, input wire rst_n_i,
    input wire mode_1080_i, input wire pixel_enable_i,
    input wire [9:0] channel0_i, channel1_i, channel2_i,
    output reg [9:0] clock_o, channel0_o, channel1_o, channel2_o
);
    function [19:0] repeat_bits;
        input [9:0] symbol;
        integer bit_index;
        begin
            for(bit_index=0;bit_index<10;bit_index=bit_index+1)
                repeat_bits[bit_index*2+:2]={2{symbol[bit_index]}};
        end
    endfunction
    wire [19:0] repeated0=repeat_bits(channel0_i);
    wire [19:0] repeated1=repeat_bits(channel1_i);
    wire [19:0] repeated2=repeat_bits(channel2_i);
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin clock_o<=0; channel0_o<=0; channel1_o<=0; channel2_o<=0; end
        else if(mode_1080_i) begin
            clock_o<=10'b1111100000;
            channel0_o<=channel0_i; channel1_o<=channel1_i; channel2_o<=channel2_i;
        end else if(!pixel_enable_i) begin
            // Encoder updated on the preceding enabled edge: first half.
            clock_o<=10'b0000000000;
            channel0_o<=repeated0[9:0]; channel1_o<=repeated1[9:0]; channel2_o<=repeated2[9:0];
        end else begin
            clock_o<=10'b1111111111;
            channel0_o<=repeated0[19:10]; channel1_o<=repeated1[19:10]; channel2_o<=repeated2[19:10];
        end
    end
endmodule
