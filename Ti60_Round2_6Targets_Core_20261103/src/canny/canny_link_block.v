`timescale 1ns/1ps
// Exact bidirectional chunk-carry closure; cycle-compatible with the tested linker.
module canny_link_block #(parameter PIXELS=128, parameter ITERATIONS=$clog2(PIXELS)) (
    input wire clk_i,
    input wire rst_n_i,
    input wire valid_i,
    output wire ready_o,
    input wire [2*PIXELS+3:0] north_i,
    input wire [2*PIXELS-1:0] center_i,
    input wire [2*PIXELS+3:0] south_i,
    input wire left_connected_i,
    input wire right_connected_i,
    output wire valid_o,
    output wire [2*PIXELS-1:0] data_o,
    output wire changed_o,
    output wire right_connected_o
);
    localparam STEPS=ITERATIONS;
    localparam CHUNK=(PIXELS+STEPS-1)/STEPS;
    localparam PAD=CHUNK*STEPS;
    reg busy;
    reg [3:0] step;
    reg [PIXELS-1:0] eligible,original_connected;
    reg [PAD-1:0] forward_eligible,backward_eligible;
    reg [PAD-1:0] forward_seed,backward_seed;
    reg [PAD-1:0] forward_result,backward_result;
    reg forward_carry,backward_carry;
    reg output_valid,output_changed;
    reg [2*PIXELS-1:0] output_data;
    wire [PIXELS-1:0] seed,input_eligible,input_connected;
    wire [PAD-1:0] seed_forward,seed_backward,eligible_forward,eligible_backward;
    wire [CHUNK:0] forward_sum={1'b0,forward_eligible[CHUNK-1:0]}+
        {1'b0,forward_seed[CHUNK-1:0]}+forward_carry;
    wire [CHUNK:0] backward_sum={1'b0,backward_eligible[CHUNK-1:0]}+
        {1'b0,backward_seed[CHUNK-1:0]}+backward_carry;
    wire [CHUNK-1:0] forward_connected=forward_eligible[CHUNK-1:0] &
        (~forward_sum[CHUNK-1:0] | forward_seed[CHUNK-1:0]);
    wire [CHUNK-1:0] backward_connected=backward_eligible[CHUNK-1:0] &
        (~backward_sum[CHUNK-1:0] | backward_seed[CHUNK-1:0]);
    wire [PAD-1:0] forward_next={forward_connected,forward_result[PAD-1:CHUNK]};
    wire [PAD-1:0] backward_next={backward_connected,backward_result[PAD-1:CHUNK]};
    wire [PIXELS-1:0] final_connected;
    genvar pixel;
    generate for(pixel=0;pixel<PIXELS;pixel=pixel+1) begin : seed_neighbors
        assign input_eligible[pixel]=center_i[2*pixel];
        assign input_connected[pixel]=center_i[2*pixel+1];
        wire neighbor_seed=center_i[2*pixel+1] |
            north_i[2*pixel+1] | north_i[2*pixel+3] | north_i[2*pixel+5] |
            south_i[2*pixel+1] | south_i[2*pixel+3] | south_i[2*pixel+5];
        if(pixel==0) assign seed[pixel]=input_eligible[pixel] && (neighbor_seed || left_connected_i);
        else if(pixel==PIXELS-1) assign seed[pixel]=input_eligible[pixel] && (neighbor_seed || right_connected_i);
        else assign seed[pixel]=input_eligible[pixel] && neighbor_seed;
        assign final_connected[pixel]=forward_next[pixel] | backward_next[PIXELS-1-pixel];
    end
    for(pixel=0;pixel<PAD;pixel=pixel+1) begin : padded_directions
        if(pixel<PIXELS) begin : image
            assign seed_forward[pixel]=seed[pixel];
            assign eligible_forward[pixel]=input_eligible[pixel];
            assign seed_backward[pixel]=seed[PIXELS-1-pixel];
            assign eligible_backward[pixel]=input_eligible[PIXELS-1-pixel];
        end else begin : padding
            assign seed_forward[pixel]=0;assign eligible_forward[pixel]=0;
            assign seed_backward[pixel]=0;assign eligible_backward[pixel]=0;
        end
    end endgenerate
    // In an eligible run, adding seed bits propagates a carry up to the next
    // ineligible pixel. Complementing the sum and restoring seeds gives every
    // reached pixel. The reverse scan supplies propagation in the other direction.
    integer index;
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            busy<=0;step<=0;eligible<=0;original_connected<=0;
            forward_eligible<=0;backward_eligible<=0;forward_seed<=0;backward_seed<=0;
            forward_result<=0;backward_result<=0;forward_carry<=0;backward_carry<=0;
            output_valid<=0;output_changed<=0;output_data<=0;
        end else begin
            output_valid<=0;
            if(valid_i && !busy) begin
                busy<=1;step<=0;eligible<=input_eligible;original_connected<=input_connected;
                forward_eligible<=eligible_forward;backward_eligible<=eligible_backward;
                forward_seed<=seed_forward;backward_seed<=seed_backward;
                forward_result<=0;backward_result<=0;forward_carry<=0;backward_carry<=0;
            end else if(busy) begin
                forward_eligible<=forward_eligible>>CHUNK;backward_eligible<=backward_eligible>>CHUNK;
                forward_seed<=forward_seed>>CHUNK;backward_seed<=backward_seed>>CHUNK;
                forward_carry<=forward_sum[CHUNK];backward_carry<=backward_sum[CHUNK];
                forward_result<=forward_next;backward_result<=backward_next;
                if(step==STEPS-1) begin
                    busy<=0;output_valid<=1;output_changed<=|(final_connected & ~original_connected);
                    for(index=0;index<PIXELS;index=index+1)
                        output_data[index*2+:2]<={final_connected[index],eligible[index]};
                end else step<=step+1'b1;
            end
        end
    end
    assign ready_o=!busy;
    assign valid_o=output_valid;
    assign data_o=output_data;
    assign changed_o=output_changed;
    assign right_connected_o=output_data[2*PIXELS-1];
endmodule
