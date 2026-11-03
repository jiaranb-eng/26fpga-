`timescale 1ns/1ps
// Lock one producer from its first AW offer through the accepted B response.
// With only producer zero active, no additional transaction cycle is inserted.
module axi_pair_write_mux (
    input wire clk_i,input wire rst_n_i,
    input wire [63:0] s_awaddr_i,input wire [15:0] s_awlen_i,input wire [1:0] s_awvalid_i,output wire [1:0] s_awready_o,
    input wire [255:0] s_wdata_i,input wire [31:0] s_wstrb_i,input wire [1:0] s_wlast_i,input wire [1:0] s_wvalid_i,output wire [1:0] s_wready_o,
    output wire [3:0] s_bresp_o,output wire [1:0] s_bvalid_o,input wire [1:0] s_bready_i,
    output wire [31:0] m_awaddr_o,output wire [7:0] m_awlen_o,output wire m_awvalid_o,input wire m_awready_i,
    output wire [127:0] m_wdata_o,output wire [15:0] m_wstrb_o,output wire m_wlast_o,output wire m_wvalid_o,input wire m_wready_i,
    input wire [1:0] m_bresp_i,input wire m_bvalid_i,output wire m_bready_o
);
    reg active,offered,owner,prefer_second;
    wire selected=offered ? owner : ((&s_awvalid_i) ? prefer_second : s_awvalid_i[1]);
    wire aw_fire=m_awvalid_o && m_awready_i;
    wire b_fire=m_bvalid_i && m_bready_o;
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin active<=0;offered<=0;owner<=0;prefer_second<=0;end
        else begin
            if(!active && m_awvalid_o) begin
                owner<=selected;
                if(aw_fire) begin active<=1;offered<=0;end
                else offered<=1;
            end
            if(b_fire) begin active<=0;prefer_second<=!owner;end
        end
    end
    assign m_awaddr_o=selected ? s_awaddr_i[63:32] : s_awaddr_i[31:0];
    assign m_awlen_o=selected ? s_awlen_i[15:8] : s_awlen_i[7:0];
    assign m_awvalid_o=!active && s_awvalid_i[selected];
    assign s_awready_o={!active && selected && m_awready_i,!active && !selected && m_awready_i};
    assign m_wdata_o=owner ? s_wdata_i[255:128] : s_wdata_i[127:0];
    assign m_wstrb_o=owner ? s_wstrb_i[31:16] : s_wstrb_i[15:0];
    assign m_wlast_o=s_wlast_i[owner];
    assign m_wvalid_o=active && s_wvalid_i[owner];
    assign s_wready_o={active && owner && m_wready_i,active && !owner && m_wready_i};
    assign s_bresp_o={m_bresp_i,m_bresp_i};
    assign s_bvalid_o={active && owner && m_bvalid_i,active && !owner && m_bvalid_i};
    assign m_bready_o=active && s_bready_i[owner];
endmodule
