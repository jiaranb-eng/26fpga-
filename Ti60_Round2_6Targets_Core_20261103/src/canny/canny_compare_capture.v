`timescale 1ns/1ps
// The parent capture owns the slot and publishes both planes atomically.
// This writer stores the same sensor frame at slot+1MiB, with 2048-byte rows.
module canny_compare_capture #(
    parameter WIDTH=1920,parameter HEIGHT=1200
) (
    input wire camera_clk_i,input wire camera_rst_n_i,
    input wire axi_clk_i,input wire axi_rst_n_i,
    input wire start_i,input wire valid_i,input wire eop_i,input wire [31:0] raw_i,
    input wire [31:0] slot_address_i,input wire abort_i,
    output wire safe_o,output wire fifo_busy_o,output wire camera_error_o,
    output wire done_o,output wire error_o,
    output wire [31:0] axi_awaddr_o,output wire [7:0] axi_awlen_o,
    output wire axi_awvalid_o,input wire axi_awready_i,
    output wire [127:0] axi_wdata_o,output wire [15:0] axi_wstrb_o,
    output wire axi_wlast_o,output wire axi_wvalid_o,input wire axi_wready_i,
    input wire [1:0] axi_bresp_i,input wire axi_bvalid_i,output wire axi_bready_o
);
    localparam IDLE=0,WAIT_DATA=1,ADDRESS=2,DATA=3,RESPONSE=4,VERIFY=5,HOLD=6;
    localparam PAYLOAD=WIDTH/16;
    reg [2:0] state,next_state;
    reg start_toggle,capturing,finished_camera,camera_error;
    reg [20:0] input_count;
    reg [31:0] packed_data;
    reg packed_valid;
    (* async_reg="true" *) reg [2:0] abort_camera;
    (* async_reg="true" *) reg [2:0] start_sync,finished_sync;
    reg start_seen;
    reg [31:0] address;
    reg [10:0] row_count;
    reg [7:0] beat_count;
    reg bus_error;
    wire fifo_full,fifo_empty,fifo_busy,fifo_overflow,fifo_underflow;
    wire [9:0] fifo_count;
    wire [127:0] fifo_data;
    wire padding=beat_count>=PAYLOAD;
    wire write_fire=axi_wvalid_o && axi_wready_i;
    wire reset_fifo=!camera_rst_n_i || !axi_rst_n_i || (abort_i && safe_o);
    assign safe_o=state==IDLE || state==WAIT_DATA || state==VERIFY || state==HOLD;
    assign done_o=state==HOLD;
    assign error_o=bus_error;
    assign camera_error_o=camera_error;
    assign fifo_busy_o=fifo_busy;
    always @(posedge camera_clk_i or negedge camera_rst_n_i) begin
        if(!camera_rst_n_i) begin
            start_toggle<=0;capturing<=0;finished_camera<=0;camera_error<=0;
            input_count<=0;packed_data<=0;packed_valid<=0;abort_camera<=0;
        end else begin
            abort_camera<={abort_camera[1:0],abort_i};
            packed_valid<=0;
            if(abort_camera[2]) begin capturing<=0;finished_camera<=0;camera_error<=0;input_count<=0;end
            else begin
                if(start_i) begin
                    start_toggle<=!start_toggle;capturing<=1;finished_camera<=0;input_count<=0;
                end
                if(valid_i && (capturing || start_i)) begin
                    packed_data<={raw_i[7:0],raw_i[15:8],raw_i[23:16],raw_i[31:24]};
                    packed_valid<=1;input_count<=start_i ? 21'd1 : input_count+1'b1;
                    if(eop_i) begin
                        capturing<=0;finished_camera<=1;
                        if(input_count!=WIDTH*HEIGHT/4-1) camera_error<=1;
                    end
                    if(fifo_full || fifo_busy) begin camera_error<=1;capturing<=0;end
                end
                if(packed_valid && (fifo_full || fifo_busy || fifo_overflow)) begin camera_error<=1;capturing<=0;end
            end
        end
    end
    W0_FIFO_32 raw_fifo (
        .wr_clk_i(camera_clk_i),.rd_clk_i(axi_clk_i),.a_rst_i(reset_fifo),
        .wr_en_i(packed_valid && !camera_error && !abort_camera[2]),.rd_en_i(write_fire && !padding),
        .wdata(packed_data),.rdata(fifo_data),.full_o(fifo_full),.empty_o(fifo_empty),.rst_busy(fifo_busy),
        .rd_datacount_o(fifo_count),.wr_datacount_o(),.prog_empty_o(),.underflow_o(fifo_underflow),.overflow_o(fifo_overflow)
    );
    genvar octet;
    generate for(octet=0;octet<16;octet=octet+1) begin : canonical_bytes
        assign axi_wdata_o[octet*8+:8]=padding ? 8'd0 : fifo_data[(15-octet)*8+:8];
    end endgenerate
    always @(posedge axi_clk_i or negedge axi_rst_n_i) begin
        if(!axi_rst_n_i) state<=IDLE;
        else state<=next_state;
    end
    always @(*) begin
        next_state=state;
        case(state)
            IDLE,HOLD:if(start_sync[2]!=start_seen && !abort_i) next_state=WAIT_DATA;
            WAIT_DATA:if(fifo_count>=PAYLOAD && !fifo_busy) next_state=ADDRESS;
            ADDRESS:if(axi_awready_i) next_state=DATA;
            DATA:if(write_fire && beat_count==127) next_state=RESPONSE;
            RESPONSE:if(axi_bvalid_i) next_state=abort_i ? IDLE : (row_count==HEIGHT-1 ? VERIFY : WAIT_DATA);
            VERIFY:if(finished_sync[2] && !bus_error) next_state=HOLD;
            default:next_state=IDLE;
        endcase
        // An offered AW or an accepted burst is completed before FIFO reset.
        if(abort_i && safe_o) next_state=IDLE;
    end
    always @(posedge axi_clk_i or negedge axi_rst_n_i) begin
        if(!axi_rst_n_i) begin
            start_sync<=0;finished_sync<=0;start_seen<=0;address<=0;row_count<=0;beat_count<=0;bus_error<=0;
        end else begin
            start_sync<={start_sync[1:0],start_toggle};finished_sync<={finished_sync[1:0],finished_camera};
            if(abort_i && safe_o) begin start_seen<=start_sync[2];row_count<=0;beat_count<=0;end
            else if((state==IDLE || state==HOLD) && start_sync[2]!=start_seen) begin
                start_seen<=start_sync[2];address<=slot_address_i+32'h00100000;row_count<=0;beat_count<=0;
            end
            if(state==ADDRESS) beat_count<=0;
            if(write_fire) beat_count<=beat_count+1'b1;
            if(state==RESPONSE && axi_bvalid_i) begin
                row_count<=row_count+1'b1;address<=address+32'd2048;
                if(axi_bresp_i!=0) bus_error<=1;
            end
            if(fifo_underflow) bus_error<=1;
        end
    end
    assign axi_awaddr_o=address;
    assign axi_awlen_o=8'd127;
    assign axi_awvalid_o=state==ADDRESS;
    assign axi_wstrb_o=16'hffff;
    assign axi_wlast_o=beat_count==127;
    assign axi_wvalid_o=state==DATA && (padding || !fifo_empty);
    assign axi_bready_o=state==RESPONSE;
endmodule
