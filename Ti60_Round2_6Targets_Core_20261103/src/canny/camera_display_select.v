`timescale 1ns/1ps
// K1 cycles annotated Canny, plain Canny, original MIPI, then DVP when enabled.
// Legacy configurations keep their original cycles. A held mailbox keeps the two-bit mode
// coherent across clocks; acknowledgment follows its vertical-blank commit.
module camera_display_select #(parameter DEBOUNCE_CYCLES=1920000, parameter ENABLE_RECTANGLE=0,
    parameter SKIP_DVP_DISPLAY=0, parameter COMBINED_EDGE_VIEW=0, parameter UART_CONTROL=0,
    parameter ENABLE_BACKGROUND=0,parameter ENABLE_COMPARE=0,parameter ENABLE_OVERLAY=0) (
    input wire [2:0] remote_mode_i,input wire remote_valid_i,
    output wire [2:0] requested_mode_o,
    input wire system_clk_i,input wire system_rst_n_i,input wire key_n_i,
    input wire pixel_clk_i,input wire pixel_rst_n_i,input wire vsync_i,
    output reg [2:0] mode_o
);
    localparam COUNT_BITS=$clog2(DEBOUNCE_CYCLES+1);
    (* async_reg="true" *) reg [1:0] key_sync;
    reg stable_key;
    reg [COUNT_BITS-1:0] debounce_count;
    reg [2:0] requested_mode,mode_payload;
    reg request_toggle,acknowledge_toggle;
    (* async_reg="true" *) reg [2:0] acknowledge_sync;
    (* async_reg="true" *) reg [3:0] selection_sync;
    (* async_reg="true" *) reg [2:0] payload_meta,payload_sync,payload_stable;
    reg previous_vsync;
    assign requested_mode_o=requested_mode;
    always @(posedge system_clk_i or negedge system_rst_n_i) begin
        if(!system_rst_n_i) begin
            key_sync<=2'b11; stable_key<=1; requested_mode<=ENABLE_RECTANGLE ? 3 : 0; debounce_count<=0;
            mode_payload<=ENABLE_RECTANGLE ? 3 : 0; request_toggle<=0; acknowledge_sync<=0;
        end
        else begin
            key_sync<={key_sync[0],key_n_i};
            acknowledge_sync<={acknowledge_sync[1:0],acknowledge_toggle};
            if(key_sync[1]==stable_key) debounce_count<=0;
            else if(debounce_count==DEBOUNCE_CYCLES-1) begin
                stable_key<=key_sync[1]; debounce_count<=0;
                if(!key_sync[1]) begin
                    if(COMBINED_EDGE_VIEW && ENABLE_RECTANGLE)
                        requested_mode<=requested_mode==3 ? 2 : 3;
                    else if(SKIP_DVP_DISPLAY)
                        requested_mode<=requested_mode==(ENABLE_RECTANGLE ? 3 : 2) ? 1 : requested_mode+1'b1;
                    else if(ENABLE_RECTANGLE) begin
                        // Keep the two Canny views adjacent; every stable press advances once.
                        case(requested_mode)
                            2'd3: requested_mode<=2'd1;
                            2'd1: requested_mode<=ENABLE_OVERLAY ? 3'd6 : (ENABLE_COMPARE ? 3'd5 : 3'd2);
                            3'd6: requested_mode<=ENABLE_COMPARE ? 3'd5 : 3'd2;
                            3'd5: requested_mode<=3'd2;
                            2'd2: requested_mode<=ENABLE_BACKGROUND ? 3'd4 : 3'd0;
                            3'd4: requested_mode<=3'd0;
                            default: requested_mode<=2'd3;
                        endcase
                    end
                    else
                        requested_mode<=requested_mode==(ENABLE_RECTANGLE ? 3 : 2) ? 0 : requested_mode+1'b1;
                end
            end else debounce_count<=debounce_count+1'b1;
            if(UART_CONTROL && remote_valid_i) requested_mode<=remote_mode_i;
            if(request_toggle==acknowledge_sync[2] && mode_payload!=requested_mode) begin
                mode_payload<=requested_mode; request_toggle<=!request_toggle;
            end
        end
    end
    always @(posedge pixel_clk_i or negedge pixel_rst_n_i) begin
        if(!pixel_rst_n_i) begin
            selection_sync<=0; payload_meta<=0; payload_sync<=0; payload_stable<=0;
            previous_vsync<=1; mode_o<=ENABLE_RECTANGLE ? 3 : 0; acknowledge_toggle<=0;
        end
        else begin
            selection_sync<={selection_sync[2:0],request_toggle};
            payload_meta<=mode_payload; payload_sync<=payload_meta; payload_stable<=payload_sync;
            previous_vsync<=vsync_i;
            if(vsync_i && !previous_vsync && selection_sync[3]!=acknowledge_toggle) begin
                mode_o<=payload_stable;
                acknowledge_toggle<=selection_sync[3];
            end
        end
    end
endmodule
