`timescale 1ns/1ps

//`include "ddr3_controller.vh"


module example_top 
(
    // Efinity User TAP内部接口，不增加板上排针接线。
    input wire jtag_inst1_TCK,
    input wire jtag_inst1_TDI,
    input wire jtag_inst1_TMS,
    input wire jtag_inst1_CAPTURE,
    input wire jtag_inst1_SHIFT,
    input wire jtag_inst1_UPDATE,
    input wire jtag_inst1_RESET,
    input wire jtag_inst1_SEL,
    input wire jtag_inst1_DRCK,
    input wire jtag_inst1_RUNTEST,
    output wire jtag_inst1_TDO,
    input wire hdmi_pll_lock,
    output wire hdmi_pll_rstn_o,
    input wire hdmi_ref_99m,
    input wire key_switch_n_i,
	////////////////////////////////////////////////////////////////
	//	External Clock & Reset
	//input 			nrst, 			//	Button K2
	input 			clk_24m,			//	24MHz Crystal
	input 			clk_25m,			//	25MHz Crystal 
	
	
	////////////////////////////////////////////////////////////////
	//	System Clock
	output 			sys_pll_rstn_o, 		
	
	input 			clk_sys,			//	Sys PLL 96MHz 
	input 			clk_pixel,			// Fixed HDMI PLL 148.5 MHz
	input 			clk_pixel_10x,		// HDMI PLL 742.5 MHz, half-rate serializer
	
	input 			sys_pll_lock,		//	Sys PLL Lock
	
	////////////////////////////////////////////////////////////////
	//	MIPI-DSI Clock & Reset
	
	////////////////////////////////////////////////////////////////
	//	DDR Clock
	output 			ddr_pll_rstn_o, 
	
	input 			tdqss_clk,			
	input 			core_clk,			//	DDR PLL 200MHz
	input 			tac_clk,			
	input 			twd_clk,			
	
	input 			ddr_pll_lock,		//	DDR PLL Lock
	
	////////////////////////////////////////////////////////////////
	//	DDR PLL Phase Shift Interface
	output 	[2:0] 	shift,
	output 	[4:0] 	shift_sel,
	output 			shift_ena,
	
	
	
	////////////////////////////////////////////////////////////////
	//	LVDS Clock
	output 			lvds_pll_rstn_o, 
	
	input 			clk_lvds_1x, 
	input 			clk_lvds_7x, 
	input 			clk_27m, 			//	RGB 1X Clock (16MHz)
	input 			clk_54m, 			//	RGB 2X Clock (32MHz, for export control)
	
	input 			lvds_pll_lock, 
	
	
	
	////////////////////////////////////////////////////////////////
	//	DDR Interface Ports
	output 	[15:0] 	addr,
	output 	[2:0] 	ba,
	output 			we,
	output 			reset,
	output 			ras,
	output 			cas,
	output 			odt,
	output 			cke,
	output 			cs,
	
	//	DQ I/O
	input 	[15:0] 	i_dq_hi,
	input 	[15:0] 	i_dq_lo,
	
	output 	[15:0] 	o_dq_hi,
	output 	[15:0] 	o_dq_lo,
	output 	[15:0] 	o_dq_oe,
	
	//	DM O
	output 	[1:0] 	o_dm_hi,
	output 	[1:0] 	o_dm_lo,
	
	//	DQS I/O
	input 	[1:0] 	i_dqs_hi,
	input 	[1:0] 	i_dqs_lo,
	
	input 	[1:0] 	i_dqs_n_hi,
	input 	[1:0] 	i_dqs_n_lo,
	
	output 	[1:0] 	o_dqs_hi,
	output 	[1:0] 	o_dqs_lo,
	
	output 	[1:0] 	o_dqs_n_hi,
	output 	[1:0] 	o_dqs_n_lo,
	
	output 	[1:0] 	o_dqs_oe,
	output 	[1:0] 	o_dqs_n_oe,
	
	//	CK
	output 			clk_p_hi, 
	output 			clk_p_lo, 
	output 			clk_n_hi, 
	output 			clk_n_lo, 
	
	
	
	////////////////////////////////////////////////////////////////
	//	MIPI-CSI Ctl / I2C
	output 			csi_ctl0_o,
	output 			csi_ctl0_oe,
	input 			csi_ctl0_i,
	
	output 			csi_ctl1_o,
	output 			csi_ctl1_oe,
	input 			csi_ctl1_i,
	
	output 			csi_scl_o,
	output 			csi_scl_oe,
	input 			csi_scl_i,
	
	output 			csi_sda_o,
	output 			csi_sda_oe,
	input 			csi_sda_i,
	
	//	MIPI-CSI RXC 
	input 			csi_rxc_lp_p_i,
	input 			csi_rxc_lp_n_i,
	output 			csi_rxc_hs_en_o,
	output 			csi_rxc_hs_term_en_o,
	input 			csi_rxc_i,
	
	//	MIPI-CSI RXD0
	output 			csi_rxd0_rst_o,
	output 			csi_rxd0_hs_en_o,
	output 			csi_rxd0_hs_term_en_o,
	
	input 			csi_rxd0_lp_p_i,
	input 			csi_rxd0_lp_n_i,
	input 	[7:0] 	csi_rxd0_hs_i,
	
	//	MIPI-CSI RXD1
	output 			csi_rxd1_rst_o,
	output 			csi_rxd1_hs_en_o,
	output 			csi_rxd1_hs_term_en_o,
	
	input 			csi_rxd1_lp_n_i,
	input 			csi_rxd1_lp_p_i,
	input 	[7:0] 	csi_rxd1_hs_i,
	
	//	MIPI-CSI RXD2
	output 			csi_rxd2_rst_o,
	output 			csi_rxd2_hs_en_o,
	output 			csi_rxd2_hs_term_en_o,
	
	input 			csi_rxd2_lp_p_i,
	input 			csi_rxd2_lp_n_i,
	input 	[7:0] 	csi_rxd2_hs_i,
	
	//	MIPI-CSI RXD3
	output 			csi_rxd3_rst_o,
	output 			csi_rxd3_hs_en_o,
	output 			csi_rxd3_hs_term_en_o,
	
	input 			csi_rxd3_lp_p_i,
	input 			csi_rxd3_lp_n_i,
	input 	[7:0] 	csi_rxd3_hs_i,
	
	//output 			csi_rxd0_fifo_rd_o, 
	//input 			csi_rxd0_fifo_empty_i, 
	//output 			csi_rxd1_fifo_rd_o, 
	//input 			csi_rxd1_fifo_empty_i, 
	//output 			csi_rxd2_fifo_rd_o, 
	//input 			csi_rxd2_fifo_empty_i, 
	//output 			csi_rxd3_fifo_rd_o, 
	//input 			csi_rxd3_fifo_empty_i, 
	
	
	
	////////////////////////////////////////////////////////////////
	//	DSI PWM & Reset Control 
	output 			dsi_pwm_o,			//	MIPI-DSI LCD PWM
	output 			dsi_resetn_o,		//	MIPI-DSI LCD Reset
	
	//	MIPI-DSI TXC / TXD
	
	
	////////////////////////////////////////////////////////////////
	//	UART Interface
	input 		 	uart_rx_i,			//	Bottom-board CH340N UART, 115200-8-N-1.
	output 		 	uart_tx_o, 
	
	
	output 	[5:0] 	led_o,			//	
	
	
	////////////////////////////////////////////////////////////////
	//	CMOS Sensor
	output 			cmos_sclk,
	input 			cmos_sdat_IN,
	output 			cmos_sdat_OUT,
	output 			cmos_sdat_OE,
	
	//	CMOS Interface
	input 			cmos_pclk,
	input 			cmos_vsync,
	input 			cmos_href,
	input 	[7:0] 	cmos_data,
	// input 			cmos_ctl1,
	// output 			cmos_ctl2,
	// output 			cmos_ctl3,
	
	
	////////////////////////////////////////////////////////////////
	//	HDMI Interface
	output 			hdmi_txc_oe,
	output 			hdmi_txd0_oe,
	output 			hdmi_txd1_oe,
	output 			hdmi_txd2_oe,
	
	output 			hdmi_txc_rst_o,
	output 			hdmi_txd0_rst_o,
	output 			hdmi_txd1_rst_o,
	output 			hdmi_txd2_rst_o,
	
	output 	[9:0] 	hdmi_txc_o,
	output 	[9:0] 	hdmi_txd0_o,
	output 	[9:0] 	hdmi_txd1_o,
	output 	[9:0] 	hdmi_txd2_o,
	
	
	////////////////////////////////////////////////////////////////
	//	LVDS Interface
	output 			lvds_txc_oe,
	output 	[6:0] 	lvds_txc_o,
	output 			lvds_txc_rst_o,
	
	output 			lvds_txd0_oe,
	output 	[6:0] 	lvds_txd0_o,
	output 			lvds_txd0_rst_o,
	
	output 			lvds_txd1_oe,
	output 	[6:0] 	lvds_txd1_o,
	output 			lvds_txd1_rst_o,
	
	output 			lvds_txd2_oe,
	output 	[6:0] 	lvds_txd2_o,
	output 			lvds_txd2_rst_o,
	
	output 			lvds_txd3_oe,
	output 	[6:0] 	lvds_txd3_o,
	output 			lvds_txd3_rst_o,
	
	
	////////////////////////////////////////////////////////////////
	//	RGB LCD 5Inch 800x480
	output 			lcd_tp_sda_o,		//	TP SDA
	output 			lcd_tp_sda_oe,
	input 			lcd_tp_sda_i,
	
	output 			lcd_tp_scl_o,		//	TP SCL
	output 			lcd_tp_scl_oe,
	input 			lcd_tp_scl_i,
	
	output 			lcd_tp_int_o,		//	TP INT
	output 			lcd_tp_int_oe,
	input 			lcd_tp_int_i,
	
	output 			lcd_tp_rst_o,		//	TP RST
	
	output 			lcd_pwm_o,			//	Backlight
	output 			lcd_blen_o,
	
	output 			lcd_vs_o,			//	VS & SSN Mux. Fixed to 1. Use DE-Only mode. 
	output 			lcd_hs_o,			//	HS. Fixed to 1. Use DE-Only mode. 

	output 	[7:0] 	lcd_b7_0_o,			//	B7:B0. 
	output 	[7:0] 	lcd_g7_0_o,			//	G7:G0. Must output 8'hFF when access SPI. 
	output 	[7:0] 	lcd_r7_0_o,			//	R7:R0. 
	
	output 	[7:0] 	lcd_b7_0_oe,		//	B7:B0. 
	output 	[7:0] 	lcd_g7_0_oe,		//	G7:G0. Must output 8'hFF when access SPI. 
	output 	[7:0] 	lcd_r7_0_oe,		//	R7:R0. 

	input 	[7:0] 	lcd_b7_0_i,			//	B7:B0. 
	input 	[7:0] 	lcd_g7_0_i,			//	G7:G0. Must output 8'hFF when access SPI. 
	input 	[7:0] 	lcd_r7_0_i,			//	R7:R0. 
	
	//	SPI Pins
	output 			spi_sck_o, 
	output 			spi_ssn_o 			
);
	
	wire 			csi_rxd0_fifo_rd_o; 
	wire 			csi_rxd0_fifo_empty_i = 0;  
	wire 			csi_rxd1_fifo_rd_o;  
	wire 			csi_rxd1_fifo_empty_i = 0;  
	wire 			csi_rxd2_fifo_rd_o;  
	wire 			csi_rxd2_fifo_empty_i = 0;  
	wire 			csi_rxd3_fifo_rd_o;  
	wire 			csi_rxd3_fifo_empty_i = 0;  
	
	
	parameter 	SIM_DATA 	= 0; 
	
	//	Hardware Configuration
	assign clk_p_hi = 1'b0;	//	DDR3 Clock requires 180 degree shifted. 
	assign clk_p_lo = 1'b1;
	assign clk_n_hi = 1'b1;
	assign clk_n_lo = 1'b0; 
	
	//	System Clock Tree Control
	assign sys_pll_rstn_o = 1'b1; 	//	nrst; 	//	Reset whole system when nrst (K2) is pressed. 
	
	assign hdmi_pll_rstn_o = sys_pll_lock; 
	assign ddr_pll_rstn_o = sys_pll_lock; 
	assign lvds_pll_rstn_o = sys_pll_lock; 
	
	wire 			w_pll_lock = sys_pll_lock && hdmi_pll_lock && ddr_pll_lock && lvds_pll_lock; 
	
	//	Synchronize System Resets. 
	reg 			rstn_sys = 0, rstn_pixel = 0; 
	wire 			rst_sys = ~rstn_sys, rst_pixel = ~rstn_pixel; 
	
	
	reg 			rstn_lvds_1x = 0; 
	wire 			rst_lvds_1x = ~rstn_lvds_1x; 
	
	reg 			rstn_27m = 0, rstn_54m = 0; 
	wire 			rst_27m = ~rstn_27m, rst_54m = ~rstn_54m; 
	
	//	Clock Gen
	always @(posedge clk_27m or negedge w_pll_lock) begin if(~w_pll_lock) rstn_27m <= 0; else rstn_27m <= 1; end
	always @(posedge clk_54m or negedge w_pll_lock) begin if(~w_pll_lock) rstn_54m <= 0; else rstn_54m <= 1; end
	always @(posedge clk_sys or negedge w_pll_lock) begin if(~w_pll_lock) rstn_sys <= 0; else rstn_sys <= 1; end
	always @(posedge clk_pixel or negedge w_pll_lock) begin if(~w_pll_lock) rstn_pixel <= 0; else rstn_pixel <= 1; end
	always @(posedge clk_lvds_1x or negedge w_pll_lock) begin if(~w_pll_lock) rstn_lvds_1x <= 0; else rstn_lvds_1x <= 1; end
	
	
	localparam 	CLOCK_MAIN 	= 96000000; 	//	System clock using 96MHz. 
	
	
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	Flash Burner Control
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	wire 			w_ustick, w_mstick; 
	
	wire  [7:0] 	w_dev_index_o;  
	wire  [7:0] 	w_dev_cmd_o;  
	wire  [31:0] 	w_dev_wdata_o;  
	wire  		w_dev_wvalid_o;  
	wire  		w_dev_rvalid_o;  
	wire 	[31:0] 	w_dev_rdata_i;  
	
	wire 			w_spi_ssn_o, w_spi_sck_o; 
	wire 	[3:0] 	w_spi_data_o, w_spi_data_oe; 
	wire 	[3:0] 	w_spi_data_i; 
	
	//	Flash Control
	reg 			r_flash_en = 0; 		//	0x00:0x00 Enable Flash
	
	always @(posedge clk_sys) begin
		r_flash_en <= (w_dev_wvalid_o && (w_dev_index_o == 8'h00) && (w_dev_cmd_o == 8'h00)) ? w_dev_wdata_o : r_flash_en; 
	end
	
	
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	LCD Data Mux
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	wire 	[7:0] 	w_lcd_b_o, w_lcd_g_o, w_lcd_r_o; 
	
	assign lcd_b7_0_o = r_flash_en ? {4'b0, w_spi_data_o[3:2], 2'b0} : w_lcd_b_o; 
	// P4-15/User_IO12 为 RX；P4-16/User_IO13 为 TX，板上 TXS0108E 转为 3.3V。
	assign lcd_g7_0_o = {1'b0,uart_tx_o,(r_flash_en ? {4'h0,w_spi_data_o[1:0]} : w_lcd_g_o[5:0])};
	assign lcd_r7_0_o = r_flash_en ? {8'h00} : w_lcd_r_o; 
	
	assign lcd_b7_0_oe = r_flash_en ? {4'b0, w_spi_data_oe[3:2], 2'b0} : 8'hFF; 
	assign lcd_g7_0_oe = {2'b01,(r_flash_en ? {4'h0,w_spi_data_oe[1:0]} : 6'h3f)};
	assign lcd_r7_0_oe = r_flash_en ? {8'h00} : 8'hFF; 
	
	assign spi_sck_o = w_spi_sck_o; 
	assign spi_ssn_o = w_spi_ssn_o; 
	assign w_spi_data_i = {lcd_b7_0_i[3:2], lcd_g7_0_i[1:0]}; 
	
	
	
	
	
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	DDR3 Controller
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	wire			w_ddr3_ui_clk = clk_sys;
	wire			w_ddr3_ui_rst = rst_sys;
	wire			w_ddr3_ui_areset = rst_sys;
	wire			w_ddr3_ui_aresetn = rstn_sys;
	

	//	General AXI Interface 
	wire	[3:0] 	w_ddr3_awid;
	wire	[31:0]	w_ddr3_awaddr;
	wire	[7:0]		w_ddr3_awlen;
	wire			w_ddr3_awvalid;
	wire			w_ddr3_awready;
	
	wire 	[3:0]  	w_ddr3_wid;
	wire 	[127:0] 	w_ddr3_wdata;
	wire 	[15:0]	w_ddr3_wstrb;
	wire			w_ddr3_wlast;
	wire			w_ddr3_wvalid;
	wire			w_ddr3_wready;
	
	wire 	[3:0] 	w_ddr3_bid;
	wire 	[1:0] 	w_ddr3_bresp;
	wire			w_ddr3_bvalid;
	wire			w_ddr3_bready;
	
	wire	[3:0] 	w_ddr3_arid;
	wire	[31:0]	w_ddr3_araddr;
	wire	[7:0]		w_ddr3_arlen;
	wire			w_ddr3_arvalid;
	wire			w_ddr3_arready;
	
	wire 	[3:0] 	w_ddr3_rid;
	wire 	[127:0] 	w_ddr3_rdata;
	wire			w_ddr3_rlast;
	wire			w_ddr3_rvalid;
	wire			w_ddr3_rready;
	wire 	[1:0] 	w_ddr3_rresp;
	
	
	//	AXI Interface Request
	wire 	[3:0] 	w_ddr3_aid;
	wire 	[31:0] 	w_ddr3_aaddr;
	wire 	[7:0]  	w_ddr3_alen;
	wire 	[2:0]  	w_ddr3_asize;
	wire 	[1:0]  	w_ddr3_aburst;
	wire 	[1:0]  	w_ddr3_alock;
	wire			w_ddr3_avalid;
	wire			w_ddr3_aready;
	wire			w_ddr3_atype;
	
	wire 			w_ddr3_cal_done, w_ddr3_cal_pass; 
	
	//	Do not issue DDR read / write when ~cal_done. 
	reg 			r_ddr_unlock = 0; 
	always @(posedge w_ddr3_ui_clk or negedge w_ddr3_ui_aresetn) begin
		if(~w_ddr3_ui_aresetn)
			r_ddr_unlock <= 0; 
		else
			r_ddr_unlock <= w_ddr3_cal_done; 
	end
	
	DdrCtrl ddr3_ctl_axi (	
		.core_clk		(core_clk),
		.tac_clk		(tac_clk),
		.twd_clk		(twd_clk),	
		.tdqss_clk		(tdqss_clk),
		
		.reset		(reset),
		.cs			(cs),
		.ras			(ras),
		.cas			(cas),
		.we			(we),
		.cke			(cke),    
		.addr			(addr),
		.ba			(ba),
		.odt			(odt),
		
		.o_dm_hi		(o_dm_hi),
		.o_dm_lo		(o_dm_lo),
		
		.i_dq_hi		(i_dq_hi),
		.i_dq_lo		(i_dq_lo),
		.o_dq_hi		(o_dq_hi),
		.o_dq_lo		(o_dq_lo),
		.o_dq_oe		(o_dq_oe),
		
		.i_dqs_hi		(i_dqs_hi),
		.i_dqs_lo		(i_dqs_lo),
		.i_dqs_n_hi		(i_dqs_n_hi),
		.i_dqs_n_lo		(i_dqs_n_lo),
		.o_dqs_hi		(o_dqs_hi),
		.o_dqs_lo		(o_dqs_lo),
		.o_dqs_n_hi		(o_dqs_n_hi),
		.o_dqs_n_lo		(o_dqs_n_lo),
		.o_dqs_oe		(o_dqs_oe),
		.o_dqs_n_oe		(o_dqs_n_oe),
		
		.clk			(w_ddr3_ui_clk),
		.reset_n		(w_ddr3_ui_aresetn),
		
		.axi_avalid		(w_ddr3_avalid && r_ddr_unlock),	//	Enable command only when unlocked. 
		.axi_aready		(w_ddr3_aready),
		.axi_aaddr		(w_ddr3_aaddr),
		.axi_aid		(w_ddr3_aid),
		.axi_alen		(w_ddr3_alen),
		.axi_asize		(w_ddr3_asize),
		.axi_aburst		(w_ddr3_aburst),
		.axi_alock		(w_ddr3_alock),
		.axi_atype		(w_ddr3_atype),
		
		.axi_wid		(w_ddr3_wid),
		.axi_wvalid		(w_ddr3_wvalid),
		.axi_wready		(w_ddr3_wready),
		.axi_wdata		(w_ddr3_wdata),
		.axi_wstrb		(w_ddr3_wstrb),
		.axi_wlast		(w_ddr3_wlast),
		
		.axi_bvalid		(w_ddr3_bvalid),
		.axi_bready		(w_ddr3_bready),
		.axi_bid		(w_ddr3_bid),
		.axi_bresp		(w_ddr3_bresp),
		
		.axi_rvalid		(w_ddr3_rvalid),
		.axi_rready		(w_ddr3_rready),
		.axi_rdata		(w_ddr3_rdata),
		.axi_rid		(w_ddr3_rid),
		.axi_rresp		(w_ddr3_rresp),
		.axi_rlast		(w_ddr3_rlast),
		
		.shift		(shift),
		.shift_sel		(),
		.shift_ena		(shift_ena),
		
		.cal_ena		(1'b1),
		.cal_done		(w_ddr3_cal_done),
		.cal_pass		(w_ddr3_cal_pass)
	);
	
	assign shift_sel = 5'b00100; 		//	ddr_tac_clk always use PLLOUT[2]. 
	
	
	AXI4_AWARMux #(.AID_LEN(4), .AADDR_LEN(32)) axi4_awar_mux (
		.aclk_i			(w_ddr3_ui_clk), 
		.arst_i			(w_ddr3_ui_rst), 
		
		.awid_i			(w_ddr3_awid),
		.awaddr_i			(w_ddr3_awaddr),
		.awlen_i			(w_ddr3_awlen),
		.awvalid_i			(w_ddr3_awvalid),
		.awready_o			(w_ddr3_awready),
		
		.arid_i			(w_ddr3_arid),
		.araddr_i			(w_ddr3_araddr),
		.arlen_i			(w_ddr3_arlen),
		.arvalid_i			(w_ddr3_arvalid),
		.arready_o			(w_ddr3_arready),
		
		.aid_o			(w_ddr3_aid),
		.aaddr_o			(w_ddr3_aaddr),
		.alen_o			(w_ddr3_alen),
		.atype_o			(w_ddr3_atype),
		.avalid_o			(w_ddr3_avalid),
		.aready_i			(w_ddr3_aready)
	);
	
	assign w_ddr3_asize = 4; 		//	Fixed 128 bits (16 bytes, size = 4)
	assign w_ddr3_aburst = 1; 
	assign w_ddr3_alock = 0; 
	
	//assign led_o[1:0] = {w_ddr3_cal_pass, w_ddr3_cal_done}; 
	
	
	
	
	
	
	
	
	
	// ////////////////////////////////////////////////////////////////
	// //	I2C Config (SC130GS)
	
	// //  i2c timing controller module of 16Bit
	// wire            [ 7:0]          sc130_i2c_config_index;
	// wire            [23:0]          sc130_i2c_config_data;
	// wire            [ 7:0]          sc130_i2c_config_size;
	// wire                            sc130_i2c_config_done;
	
	// i2c_timing_ctrl_16bit
	// #(
	//     .CLK_FREQ           (CLOCK_MAIN),                              //  100 MHz
	//     .I2C_FREQ           (50_000    )                               //  10 KHz(<= 400KHz)
	// ) u_i2c_timing_ctrl_16bit (
	//     //  global clock
	//     .clk                (clk_sys                 ),                          //  96MHz
	//     .rst_n              (rstn_sys                ),                          //  system reset

	//     //  i2c interface
	//     .i2c_sclk           (csi_scl_o               ),                          //  i2c clock
	//     .i2c_sdat_IN        (csi_sda_i               ),
	//     .i2c_sdat_OUT       (csi_sda_o               ),
	//     .i2c_sdat_OE        (csi_sda_oe              ),

	//     //  i2c config data
	//     .i2c_config_index   (sc130_i2c_config_index        ),                          //  i2c config reg index, read 2 reg and write xx reg
	//     .i2c_config_data    ({8'h60, sc130_i2c_config_data}),                     //  i2c config data
	//     .i2c_config_size    (sc130_i2c_config_size         ),                          //  i2c config data counte
	//     .i2c_config_done    (sc130_i2c_config_done         )                          //  i2c config timing complete
	// );
	// assign csi_scl_oe = 1; 

	// //  I2C Configure Data of SC130GS
	// I2C_SC130GS_12801024_4Lanes_Config u_I2C_SC130GS_12801024_4Lanes_Config 
	// (
	//     .LUT_INDEX  (sc130_i2c_config_index   ),
	//     .LUT_DATA   (sc130_i2c_config_data    ),
	//     .LUT_SIZE   (sc130_i2c_config_size    )
	// );
	
//wire  i2c_config_done = 1'b1;
//----------------------------------------------
//i2c timing controller module of 16Bit
	wire	[ 7:0]          i2c_config_index;
	wire	[23:0]          i2c_config_data;
	wire	[ 7:0]          i2c_config_size;
	wire	                i2c_config_done;
	wire	[23:0]          sensor_config_data;                     //  {addr,data}
	wire	                sensor_config_data_valid;
	i2c_timing_ctrl_reg16_dat8_wronly
	#(
		.CLK_FREQ       (CLOCK_MAIN ),                                      //  120 MHz
		.I2C_FREQ       (100_000    )                                       //  100 KHz(<= 400KHz)
	)
	u_i2c_timing_ctrl_reg16_dat8_wronly
	(
		//  global clock
		.clk                        (clk_sys                    ),          //  120MHz
		//.rst_n                      (0                 ),          //  system reset
		//.rst_n                      (1                 ),          //  system reset
		.rst_n                      (rstn_sys                 ),          //  system reset
		//.rst_n                      (rst_sys                 ),          //  system reset
	
		//  i2c interface
		.i2c_sclk                   (cmos_sclk                  ),          //  i2c clock
		.i2c_sdat_i                 (cmos_sdat_IN              ),
		.i2c_sdat_o                 (cmos_sdat_OUT              ),
		.i2c_sdat_oe                (cmos_sdat_OE               ),
	
		//  i2c config data
		.i2c_config_index           (i2c_config_index           ),          //  i2c config reg index, read 2 reg and write xx reg
		.i2c_config_data            ({8'h60, i2c_config_data}   ),          //  i2c config data
		.i2c_config_size            (i2c_config_size            ),          //  i2c config data counte
		.i2c_config_done            (i2c_config_done            )           //  i2c config timing complete
	);
	
	
	//----------------------------------
	//I2C Configure Data of SC1336
	I2C_SC1336_1280720_Config    u_I2C_SC1336_1280720_Config   //Auto/Manual Exposure
	(
		.LUT_INDEX  (i2c_config_index   ),
		.LUT_DATA   (i2c_config_data    ),
		.LUT_SIZE   (i2c_config_size    )
	);
	// ////////////////////////////////////////////////////////////////
	// //	I2C Config (AR0135)
	
	// //  i2c timing controller module of 16Bit
	// wire            [ 7:0]          ar0135_i2c_config_index;
	// wire            [31:0]          ar0135_i2c_config_data;
	// wire            [ 7:0]          ar0135_i2c_config_size;
	// wire                            ar0135_i2c_config_done;
	// wire            [15:0]          ar0135_i2c_rdata;                              //  i2c register data

	// i2c_timing_ctrl_reg16_dat16
	// #(
	//     .SIMU_EN            (0              ),
	//     .I2C_READ_EN        (1              ),                              //  Enable I2C Register read
	//     .CLK_FREQ           (CLOCK_MAIN     ),                              //  120 MHz
	//     .I2C_FREQ           (50_000         )                               //  10 KHz(<= 400KHz)
	// )
	// u_i2c_timing_ctrl_16reg_16bit
	// (
	//     //global clock
	//     .clk                (clk_sys                 ),                          //  96MHz
	//     .rst_n              (rstn_sys                ),                          //  system reset

	//     //i2c interface
	//     .i2c_sclk           (cmos_sclk                  ),                  //  i2c clock
	// 	.i2c_sdat_o 	(cmos_sdat_OUT), 
	// 	.i2c_sdat_oe	(cmos_sdat_OE), 
	// 	.i2c_sdat_i 	(cmos_sdat_IN), 
		
	//     //i2c config data
	//     .i2c_config_index   (ar0135_i2c_config_index           ),                  //  i2c config reg index, read 2 reg and write xx reg
	//     .i2c_config_data    ({8'h20, ar0135_i2c_config_data}   ),                  //  i2c config data
	//     .i2c_config_size    (ar0135_i2c_config_size            ),                  //  i2c config data counte
	//     .i2c_config_done    (ar0135_i2c_config_done            ),                  //  i2c config timing complete
	//     .i2c_rdata          (ar0135_i2c_rdata                  )                   //  i2c register data while read i2c slave
	// );
	
	// //----------------------------------------------------------------------
	// //  I2C Configure Data of AR0135
	// I2C_AR0135_1280720_Config u_I2C_AR0135_1280720_Config
	// (
	//     .LUT_INDEX  (ar0135_i2c_config_index   ),
	//     .LUT_DATA   (ar0135_i2c_config_data    ),
	//     .LUT_SIZE   (ar0135_i2c_config_size    )
	// );
	
	
		
	//	CMOS Interface
	//input 			cmos_pclk,
	//input 			cmos_vsync,
	//input 			cmos_href,
	//input 	[7:0] 	cmos_data,
	//input 			cmos_ctl1,
	//output 			cmos_ctl2,
	//output 			cmos_ctl3,

	
	//	Output LED
	reg 	[3:0]		r_cmos_fv_o = 0; 
	reg 	[1:0] 	r_cmos_rx_vsync0_in = 0; 
	always @(posedge cmos_pclk) begin
		r_cmos_rx_vsync0_in <= {r_cmos_rx_vsync0_in, cmos_vsync}; 
		r_cmos_fv_o <= r_cmos_fv_o + ((r_cmos_rx_vsync0_in == 2'b01) ? 1 : 0); 
	end
	assign led_o[5] = r_cmos_fv_o[3]; 
	
	
	
	
	
	
	
	
	////////////////////////////////////////////////////////////////
	//	System Control. Can be removed for public. 
	
	localparam 	CLK_FREQ 	= 96_000_000; 	//	clk_sys is 96MHz. 
	localparam 	BAUD_RATE 	= 460_800; 		//	Use 460800-8-N-1. 
	
	
	//	SFR I/O Interface
	wire 	[7:0] 	w_sfr_addr_o; 	//	SFR Address (0xFF00 ~ 0xFFFF). 00:Power; 40~5F:Stream0; 60~7F:Stream1. 
	wire 	[7:0] 	w_sfr_wdata_o; 	//	SFR Write Data. 
	wire 			w_sfr_we_o; 		//	SFR WE. 
	reg 	[7:0] 	w_sfr_rdata_i; 	//	Must be valid after sfr_rd_o. 
	wire 			w_sfr_rd_o; 		//	SFR RD. 
	
	
	//	System Control Registers
	reg 			r_dsi_tx_rstn = 0; 	//	DSI TX Reset
	reg 	[7:0] 	r_dsi_pwm = 64; 		//	[6:0]PWM, [7]Pol
	reg 			r_dsi_resetn_o = 0; 	//	DSI Panel Reset
	reg 			r_dsi_data_rstn = 0; 	//	DSI TX Reset
	
	reg 	[3:0] 	r_dsi_lp_p_ovr = 0; 	
	reg 	[3:0] 	r_dsi_lp_n_ovr = 0; 	
	
	
	
	//	AXI-Lite Interface Bridge
	localparam 	CSI_AXILITE_ID 	= 0; 				//	Select DSI_TX when r_axi_sel = DSI_AXILITE_ID. 
	localparam 	DSI_AXILITE_ID 	= 1; 				//	Select DSI_TX when r_axi_sel = DSI_AXILITE_ID. 
	
	reg 	[7:0] 	r_axi_addr = 8'h18; 		//	0xE0 (RW)
	reg 	[31:0] 	r_axi_wdata = 32'h0000000A; 	//	0xE1~0xE4 (RW)
	wire 	[31:0] 	w_axi_rdata; 			//	0xE5~0xE8 (RO)
	reg 	[0:0] 	r_axi_sel = 1; 			//	0xE9[7:2] (RW)
	reg 			r_axi_r1w0 = 0; 			//	0xE9[1] (RW)
	reg 			r_axi_req = 0; 			//	0xE9[0] (WO, Single Cycle)
	reg 			r_axi_req_o = 0; 			//	Delayed of r_axi_req. 
	
	
	//	Buffered AXI Read Data
	reg 	[31:0] 	r_axi_rdata = 0; 		//	Use state machine
	reg 			r_axi_idle = 0; 		//	AXI Idle 
	
	reg 	[3:0] 	rs_axilite = 0; 		//	AXI Access
	wire 	[3:0] 	ws_axilite_idle = 0; 		
	wire 	[3:0] 	ws_axilite_write = 1; 
	wire 	[3:0] 	ws_axilite_read = 2; 
	wire 	[3:0] 	ws_axilite_endread = 3; 
	
	reg 			r_axi_awvalid = 0, r_axi_wvalid = 0, r_axi_arvalid = 0; 
	wire 			w_axi_awready, w_axi_wready, w_axi_arready, w_axi_rvalid; 
	
	
	reg 	[3:0] 	rc_axi_init = 0; 

	always @(posedge clk_sys or posedge rst_sys) begin
		if(rst_sys) begin
			r_dsi_tx_rstn <= 0; 
			r_dsi_pwm <= 64; 
			r_axi_req <= 0; 
			r_dsi_resetn_o <= 0; 
			r_dsi_data_rstn <= 0; 
			
			rs_axilite <= 0; 
			r_axi_awvalid <= 0; 
			r_axi_wvalid <= 0;
			r_axi_arvalid <= 0; 
			
			r_dsi_lp_p_ovr <= 0; 
			r_dsi_lp_n_ovr <= 0; 
			
			rc_axi_init <= 0; 
			r_axi_idle <= 0; 
			
		end else begin
			r_dsi_tx_rstn <= 1; 
			r_dsi_resetn_o <= 1; 
			r_dsi_data_rstn <= 1; 
			
		end
	end
	
	assign dsi_resetn_o = r_dsi_resetn_o; 
	
	assign csi_ctl0_oe = 0; 
	assign csi_ctl1_oe = 0; 
	
	
	PWMLite dsi_pwm (		//	#(.ENABLE_TICK(0), .PWM_BITS(7)) 
		.clk_i			(clk_sys),
		.rst_i			(rst_sys),
		.pwm_i			(r_dsi_pwm[6:0]),
		.pol_i			(r_dsi_pwm[7]), 
		.pwm_o			(dsi_pwm_o)
	);






	////////////////////////////////////////////////////////////////
	//	MIPI CSI RX
	
	//	The CSI RXC shall not be inverted. Data can be inverted with swapped LP data and flipped HS data. 
	localparam 	CSI_RXD_INV 	= 4'b1111; 
	localparam 	CSI_DATA_WIDTH 	= 8; 			
	localparam 	CSI_STRB_WIDTH 	= CSI_DATA_WIDTH / 8; 

	
	////////////////////////////////////////////////////////////////
	//	MIPI-CSI Crop
	
	wire			XYCrop_frame_vsync; 
	wire			XYCrop_frame_href;
	wire			XYCrop_frame_de;
	wire	[63:0]	XYCrop_frame_Gray;

	Sensor_Image_XYCrop
	#(
		.IMAGE_HSIZE_SOURCE (1280 / CSI_STRB_WIDTH),
		.IMAGE_VSIZE_SOURCE (720	 ),
		.IMAGE_HSIZE_TARGET (1280 / CSI_STRB_WIDTH),
		.IMAGE_YSIZE_TARGET (720 	 ),
		.PIXEL_DATA_WIDTH	(CSI_DATA_WIDTH) 		//	32		 )
	)
	u_Sensor_Image_XYCrop
	(
		//	globel clock
		.clk			(cmos_pclk),			//	image pixel clock
		.rst_n		(rstn_sys),			//	system reset
		
		//CMOS Sensor interface
		.image_in_vsync (cmos_vsync		),			//H : Data Valid; L : Frame Sync(Set it by register)
		.image_in_href	(cmos_href		),			//H : Data vaild, L : Line Sync
		.image_in_de	(cmos_href		), 			//H : Data Enable, L : Line Sync
		.image_in_data	(cmos_data),			//8 bits cmos data input
		
		.image_out_vsync(XYCrop_frame_vsync ),			//H : Data Valid; L : Frame Sync(Set it by register)
		.image_out_href (XYCrop_frame_href	),			//H : Data vaild, L : Line Sync
		.image_out_de	(XYCrop_frame_de	), 			//H : Data Enable, L : Line Sync
		.image_out_data (XYCrop_frame_Gray	)			//8 bits cmos data input	
	);

	reg			r_XYCrop_frame_vsync = 0; 
	reg			r_XYCrop_frame_href = 0;
	reg			r_XYCrop_frame_de = 0;
	reg	[63:0]	r_XYCrop_frame_Gray = 0;
	
	always @(posedge cmos_pclk) begin
		r_XYCrop_frame_vsync <= XYCrop_frame_vsync; 
		r_XYCrop_frame_href <= XYCrop_frame_href;
		r_XYCrop_frame_de <= XYCrop_frame_de;
		r_XYCrop_frame_Gray <= XYCrop_frame_Gray;
	end
	
	//	Data Write Assignment
	wire			cmos_frame_vsync = r_XYCrop_frame_vsync;                     //  cmos frame data vsync valid signal
	wire			cmos_frame_href = r_XYCrop_frame_href && r_XYCrop_frame_de;	 //  cmos frame data href vaild  signal
	wire	[63:0]	cmos_frame_Gray = r_XYCrop_frame_Gray; 
	wire 			cmos_vsync_end;

	// Pre-DDR image pipeline.  Bayer conversion, AWB and all four effects run
	// entirely in the sensor pixel-clock domain before the write-side FIFO.
	wire        pre_rgb_vsync;
	wire        pre_rgb_href;
	wire        pre_rgb_hsync;
	wire [7:0]  pre_rgb_raw_r;
	wire [7:0]  pre_rgb_raw_g;
	wire [7:0]  pre_rgb_raw_b;
	(* async_reg = "true" *) reg [1:0] cmos_reset_release = 2'b00;
	wire cmos_pipeline_rst_n = cmos_reset_release[1];

	// cmos_pclk 域采用异步置位、同步释放的本地复位链。
	always @(posedge cmos_pclk or negedge rstn_sys) begin
		if (!rstn_sys)
			cmos_reset_release <= 2'b00;
		else
			cmos_reset_release <= {cmos_reset_release[0], 1'b1};
	end

	VIP_RAW8_RGB888 #(
		.IMG_HDISP (1280),
		.IMG_VDISP (720)
	) u_pre_ddr_bayer2rgb (
		.clk               (cmos_pclk),
		.rst_n             (cmos_pipeline_rst_n),
		.mirror            (2'b00),
		.per_frame_vsync   (cmos_frame_vsync),
		.per_frame_href    (cmos_frame_href),
		.per_frame_hsync   (r_XYCrop_frame_href),
		.per_img_RAW       (cmos_frame_Gray[7:0]),
		.post_frame_vsync  (pre_rgb_vsync),
		.post_frame_href   (pre_rgb_href),
		.post_frame_hsync  (pre_rgb_hsync),
		.post_img_red      (pre_rgb_raw_r),
		.post_img_green    (pre_rgb_raw_g),
		.post_img_blue     (pre_rgb_raw_b)
	);

	wire [17:0] pre_rgb_r_mult = pre_rgb_raw_r * 10'd409;
	wire [17:0] pre_rgb_b_mult = pre_rgb_raw_b * 9'd384;
	wire [7:0]  pre_rgb_r = pre_rgb_r_mult[17:16] ?
	                            8'hff : pre_rgb_r_mult[15:8];
	wire [7:0]  pre_rgb_g = pre_rgb_raw_g;
	wire [7:0]  pre_rgb_b = pre_rgb_b_mult[17:16] ?
	                            8'hff : pre_rgb_b_mult[15:8];

	reg [10:0] pre_rgb_x = 11'd0;
	reg [9:0]  pre_rgb_y = 10'd0;

	always @(posedge cmos_pclk or negedge cmos_pipeline_rst_n) begin
		if (!cmos_pipeline_rst_n || !pre_rgb_vsync) begin
			pre_rgb_x <= 11'd0;
			pre_rgb_y <= 10'd0;
		end else if (pre_rgb_href) begin
			if (pre_rgb_x == 11'd1279) begin
				pre_rgb_x <= 11'd0;
				pre_rgb_y <= (pre_rgb_y == 10'd719) ?
				             10'd0 : pre_rgb_y + 1'b1;
			end else begin
				pre_rgb_x <= pre_rgb_x + 1'b1;
			end
		end
	end

	wire pre_rgb_sop = pre_rgb_href &&
	                   (pre_rgb_x == 11'd0) && (pre_rgb_y == 10'd0);
	wire pre_rgb_eop = pre_rgb_href &&
	                   (pre_rgb_x == 11'd1279) && (pre_rgb_y == 10'd719);

	wire        quad_sop;
	wire        quad_eop;
	wire        quad_valid;
	(* keep, syn_keep *) wire [23:0] quad_original_rgb;
	(* keep, syn_keep *) wire [23:0] quad_direct_sobel_rgb;
	(* keep, syn_keep *) wire [23:0] quad_median_sobel_rgb;
	(* keep, syn_keep *) wire [23:0] quad_overlay_rgb;
	wire        quad_direct_edge;
	wire        quad_median_edge;
	reg  [9:0]  sobel_threshold;
	wire [9:0]  sobel_threshold_cdc_value;
	wire [9:0]  sobel_threshold_sys;
	wire        sobel_threshold_valid_sys;
	wire        sobel_threshold_ready_sys;
	wire        osd_text_wr_en_sys;
	wire [5:0]  osd_text_wr_addr_sys;
	wire [7:0]  osd_text_wr_data_sys;
	localparam [9:0] SOBEL_DEFAULT_THRESHOLD = 10'd64;
	localparam ENABLE_QUAD_MOSAIC = 1'b1;

	// 阈值仅在输入帧 SOP 锁存，避免同一帧中途改变检测强度。
	always @(posedge cmos_pclk or negedge cmos_pipeline_rst_n) begin
		if (!cmos_pipeline_rst_n)
			sobel_threshold <= SOBEL_DEFAULT_THRESHOLD;
		else if (pre_rgb_sop)
			sobel_threshold <= sobel_threshold_cdc_value;
	end

	(* syn_preserve = "true" *) sobel_quad_effects #(
		.WIDTH      (1280),
		.HEIGHT     (720),
		.EDGE_COLOR (24'hff0000)
	) u_pre_ddr_quad_effects (
		.clk                      (cmos_pclk),
		.rst_n                    (cmos_pipeline_rst_n),
		.threshold_i              (sobel_threshold),
		.sink_sop                 (pre_rgb_sop),
		.sink_eop                 (pre_rgb_eop),
		.sink_valid               (pre_rgb_href),
		.sink_rgb                 ({pre_rgb_r, pre_rgb_g, pre_rgb_b}),
		.source_sop               (quad_sop),
		.source_eop               (quad_eop),
		.source_valid             (quad_valid),
		.source_original_rgb      (quad_original_rgb),
		.source_direct_sobel_rgb  (quad_direct_sobel_rgb),
		.source_median_sobel_rgb  (quad_median_sobel_rgb),
		.source_overlay_rgb       (quad_overlay_rgb),
		.source_direct_edge       (quad_direct_edge),
		.source_median_edge       (quad_median_edge)
	);

	// 四个独立 2x2 缩放器保持每路固定资源与一致输出节拍。
	wire downscale_sop_0;
	wire downscale_eop_0;
	wire downscale_valid_0;
	wire [23:0] downscale_rgb_0;
	wire downscale_sop_1;
	wire downscale_eop_1;
	wire downscale_valid_1;
	wire downscale_edge_1;
	wire downscale_sop_2;
	wire downscale_eop_2;
	wire downscale_valid_2;
	wire downscale_edge_2;
	wire downscale_sop_3;
	wire downscale_eop_3;
	wire downscale_valid_3;
	wire [23:0] downscale_rgb_3;

	quad_downscale_p0_box_2x u_quad_downscale_p0 (
		.clk_i(cmos_pclk), .rst_n_i(cmos_pipeline_rst_n),
		.sink_sop_i(quad_sop), .sink_eop_i(quad_eop),
		.sink_valid_i(quad_valid), .sink_rgb_i(quad_original_rgb),
		.source_sop_o(downscale_sop_0), .source_eop_o(downscale_eop_0),
		.source_valid_o(downscale_valid_0), .source_rgb_o(downscale_rgb_0)
	);

	quad_downscale_binary_or_2x u_quad_downscale_p1 (
		.clk_i(cmos_pclk), .rst_n_i(cmos_pipeline_rst_n),
		.sink_sop_i(quad_sop), .sink_eop_i(quad_eop),
		.sink_valid_i(quad_valid), .sink_edge_i(quad_direct_edge),
		.source_sop_o(downscale_sop_1), .source_eop_o(downscale_eop_1),
		.source_valid_o(downscale_valid_1), .source_edge_o(downscale_edge_1)
	);

	quad_downscale_binary_or_2x u_quad_downscale_p2 (
		.clk_i(cmos_pclk), .rst_n_i(cmos_pipeline_rst_n),
		.sink_sop_i(quad_sop), .sink_eop_i(quad_eop),
		.sink_valid_i(quad_valid), .sink_edge_i(quad_median_edge),
		.source_sop_o(downscale_sop_2), .source_eop_o(downscale_eop_2),
		.source_valid_o(downscale_valid_2), .source_edge_o(downscale_edge_2)
	);

	// P0背景与P2掩码在同一有效拍完成；复用结果，避免重复缓存整行。
	assign downscale_sop_3 = downscale_sop_0;
	assign downscale_eop_3 = downscale_eop_0;
	assign downscale_valid_3 = downscale_valid_0;
	assign downscale_rgb_3 = downscale_edge_2 ? 24'hff0000 : downscale_rgb_0;

	wire [15:0] downscale_rgb565_0 = {
		downscale_rgb_0[23:19], downscale_rgb_0[15:10], downscale_rgb_0[7:3]
	};
	wire [15:0] downscale_rgb565_1 = downscale_edge_1 ? 16'hffff : 16'h0000;
	wire [15:0] downscale_rgb565_2 = downscale_edge_2 ? 16'hffff : 16'h0000;
	wire [15:0] downscale_rgb565_3 = {
		downscale_rgb_3[23:19], downscale_rgb_3[15:10], downscale_rgb_3[7:3]
	};
	wire packed_valid_0;
	wire packed_valid_1;
	wire packed_valid_2;
	wire packed_valid_3;
	wire [31:0] packed_data_0;
	wire [31:0] packed_data_1;
	wire [31:0] packed_data_2;
	wire [31:0] packed_data_3;

	rgb565_pair_packer u_quad_packer_0 (
		.clk(cmos_pclk), .rst_n(cmos_pipeline_rst_n),
		.sink_sop(downscale_sop_0), .sink_valid(downscale_valid_0),
		.sink_data(downscale_rgb565_0),
		.source_valid(packed_valid_0), .source_data(packed_data_0)
	);
	rgb565_pair_packer u_quad_packer_1 (
		.clk(cmos_pclk), .rst_n(cmos_pipeline_rst_n),
		.sink_sop(downscale_sop_1), .sink_valid(downscale_valid_1),
		.sink_data(downscale_rgb565_1),
		.source_valid(packed_valid_1), .source_data(packed_data_1)
	);
	rgb565_pair_packer u_quad_packer_2 (
		.clk(cmos_pclk), .rst_n(cmos_pipeline_rst_n),
		.sink_sop(downscale_sop_2), .sink_valid(downscale_valid_2),
		.sink_data(downscale_rgb565_2),
		.source_valid(packed_valid_2), .source_data(packed_data_2)
	);
	rgb565_pair_packer u_quad_packer_3 (
		.clk(cmos_pclk), .rst_n(cmos_pipeline_rst_n),
		.sink_sop(downscale_sop_3), .sink_valid(downscale_valid_3),
		.sink_data(downscale_rgb565_3),
		.source_valid(packed_valid_3), .source_data(packed_data_3)
	);

	wire [15:0] quad_overlay_rgb565 = {
		quad_overlay_rgb[23:19],
		quad_overlay_rgb[15:10],
		quad_overlay_rgb[7:3]
	};
	wire        packed_overlay_valid;
	wire [31:0] packed_overlay_data;

	rgb565_pair_packer u_rgb565_pair_packer (
		.clk          (cmos_pclk),
		.rst_n        (cmos_pipeline_rst_n),
		.sink_sop     (quad_sop),
		.sink_valid   (quad_valid),
		.sink_data    (quad_overlay_rgb565),
		.source_valid (packed_overlay_valid),
		.source_data  (packed_overlay_data)
	);

	// The pair packer adds one register stage beyond the four-effect stream.
	wire pre_ddr_frame_vsync;
	sobel_shift_regs #(
		.DWIDTH     (1),
		.DELAY_DUTY (23)
	) u_pre_ddr_vsync_delay (
		.clk   (cmos_pclk),
		.rst_n (cmos_pipeline_rst_n),
		.idata (pre_rgb_vsync),
		.odata (pre_ddr_frame_vsync)
	);

	wire unused_pre_rgb_hsync = pre_rgb_hsync;
	wire unused_quad_effects = quad_eop ^ quad_direct_sobel_rgb[0] ^
	                           quad_median_sobel_rgb[0] ^
	                           downscale_eop_1 ^ downscale_eop_2 ^
	                           downscale_eop_3;

	












	////////////////////////////////////////////////////////////////
	//	DDR R/W Control
	

	wire                            lcd_de;
	wire                            lcd_hs;      
	wire                            lcd_vs;
	wire 					  lcd_request; 
	wire            [7:0]           lcd_red, lcd_red2;
	wire            [7:0]           lcd_green, lcd_green2;
	wire            [7:0]           lcd_blue, lcd_blue2;
	wire            [15:0]          lcd_data; 
    // Declare shared module-scope nets before either generate branch uses them.
    (* keep, syn_keep *) wire [15:0] dvp_lcd_data;
    wire select_canny, display_rst_n, pixel_enable;
    wire [2:0] requested_mode,selected_mode;
    wire select_raw;
    wire [9:0] encoded0,encoded1,encoded2;
    // Canny scratch and display transport, all masters released after DDR calibration.
    wire [127:0] camera_axi_awaddr;
    wire [31:0] camera_axi_awlen;
    wire [3:0] camera_axi_awvalid;
    wire [3:0] camera_axi_awready;
    wire [511:0] camera_axi_wdata;
    wire [63:0] camera_axi_wstrb;
    wire [3:0] camera_axi_wlast;
    wire [3:0] camera_axi_wvalid;
    wire [3:0] camera_axi_wready;
    wire [15:0] camera_axi_bid;
    wire [7:0] camera_axi_bresp;
    wire [3:0] camera_axi_bvalid;
    wire [3:0] camera_axi_bready;
    wire [15:0] camera_axi_arid;
    wire [127:0] camera_axi_araddr;
    wire [31:0] camera_axi_arlen;
    wire [3:0] camera_axi_arvalid;
    wire [3:0] camera_axi_arready;
    wire [15:0] camera_axi_rid;
    wire [511:0] camera_axi_rdata;
    wire [7:0] camera_axi_rresp;
    wire [3:0] camera_axi_rlast;
    wire [3:0] camera_axi_rvalid;
    wire [3:0] camera_axi_rready;
	assign w_ddr3_wid = 0; 
	
	wire [31:0] w_axi_tp;
	wire [31:0] quad_committed_frames;
	wire [31:0] quad_dropped_frames;

    // Declare history buses before the generate scope can create implicit nets.
    wire [2:0] serial_display_request,display_requested_sys;
    wire [1:0] history_control_sys;
    wire background_mask_capture,background_mask_ready,background_mask_busy;
    wire [13:0] background_mask_address;
    wire [7:0] background_mask_data,background_mask_epoch;
    wire [7:0] background_threshold;
    wire [10:0] canny_low_sys,canny_high_sys;
    wire canny_threshold_valid,canny_threshold_ready;
    wire [21:0] canny_threshold_camera;
    canny_threshold_cdc u_canny_threshold_cdc (
        .src_clk(clk_sys),.src_rst_n(rstn_sys),
        .src_threshold_i({canny_high_sys,canny_low_sys}),
        .src_valid_i(canny_threshold_valid),.src_ready_o(canny_threshold_ready),
        .dst_clk(w_csi_rx_clk),.dst_rst_n(rstn_sys),.dst_threshold_o(canny_threshold_camera)
    );
    wire background_relearn,background_valid,background_pending,background_overflow;
    wire [31:0] background_epoch,background_frames,background_cycles;
    wire [3:0] background_count;
    wire serial_display_valid;
    wire [7:0] history_age_a_sys,history_age_b_sys,dvp_history_count,mipi_history_count;

	generate
	if (ENABLE_QUAD_MOSAIC) begin: Gen_Quad_Mosaic_Controller
	(* syn_preserve="true" *) quad_axi4_ctrl #(
		.C_COMPACT_EDGES(1),
		.C_HISTORY_ENABLE(1),.C_BASE_ADDR(32'h04000000),
		.C_RD_END_ADDR(1280 * 720 * 2),
		.C_R_WIDTH(16),
		.C_ID_LEN(4)
	) u_quad_axi4_ctrl (
        .history_control_i(history_control_sys),.history_age_a_i(history_age_a_sys),
        .history_age_b_i(history_age_b_sys),.history_count_o(dvp_history_count),
		.axi_clk(w_ddr3_ui_clk),
		.axi_reset(w_ddr3_ui_rst || !r_ddr_unlock),
		.axi_awaddr(camera_axi_awaddr[0 +: 32]), .axi_awlen(camera_axi_awlen[0 +: 8]),
		.axi_awvalid(camera_axi_awvalid[0]), .axi_awready(camera_axi_awready[0]),
		.axi_wdata(camera_axi_wdata[0 +: 128]), .axi_wstrb(camera_axi_wstrb[0 +: 16]),
		.axi_wlast(camera_axi_wlast[0]), .axi_wvalid(camera_axi_wvalid[0]),
		.axi_wready(camera_axi_wready[0]),
		.axi_bresp(camera_axi_bresp[0 +: 2]), .axi_bvalid(camera_axi_bvalid[0]),
		.axi_bready(camera_axi_bready[0]),
		.axi_arid(camera_axi_arid[0 +: 4]), .axi_araddr(camera_axi_araddr[0 +: 32]),
		.axi_arlen(camera_axi_arlen[0 +: 8]), .axi_arvalid(camera_axi_arvalid[0]),
		.axi_arready(camera_axi_arready[0]), .axi_rdata(camera_axi_rdata[0 +: 128]),
		.axi_rlast(camera_axi_rlast[0]), .axi_rvalid(camera_axi_rvalid[0]),
		.axi_rready(camera_axi_rready[0]),
		.wframe_pclk(cmos_pclk), .wframe_reset_n(cmos_pipeline_rst_n),
		.wframe_sop(downscale_sop_0), .wframe_eop(downscale_eop_0),
		.wframe_valid_0(downscale_valid_0),
		.wframe_valid_1(downscale_valid_1),
		.wframe_valid_2(downscale_valid_2),
		.wframe_valid_3(downscale_valid_3),
		.wframe_data_en_0(packed_valid_0),
		.wframe_data_en_1(packed_valid_1),
		.wframe_data_en_2(packed_valid_2),
		.wframe_data_en_3(packed_valid_3),
		.wframe_data_0(packed_data_0), .wframe_data_1(packed_data_1),
		.wframe_data_2(packed_data_2), .wframe_data_3(packed_data_3),
		.rframe_pclk(clk_pixel), .rframe_vsync(~lcd_vs),
		.rframe_data_en(lcd_request && !select_canny), .rframe_data(dvp_lcd_data),
		.committed_frame_count_o(quad_committed_frames),
		.dropped_frame_count_o(quad_dropped_frames), .tp_o(w_axi_tp)
	);
	end else begin: Gen_Single_P3_Controller
	axi4_ctrl #(.C_RD_END_ADDR(1280 * 720 * 2), .C_W_WIDTH(32), .C_R_WIDTH(16), .C_ID_LEN(4)) u_axi4_ctrl (

		.axi_clk        (w_ddr3_ui_clk            ),
		.axi_reset      (w_ddr3_ui_rst            ),

		.axi_awaddr     (camera_axi_awaddr[0 +: 32]       ),
		.axi_awlen      (camera_axi_awlen[0 +: 8]        ),
		.axi_awvalid    (camera_axi_awvalid[0]      ),
		.axi_awready    (camera_axi_awready[0]      ),

		.axi_wdata      (camera_axi_wdata[0 +: 128]        ),
		.axi_wstrb      (camera_axi_wstrb[0 +: 16]        ),
		.axi_wlast      (camera_axi_wlast[0]        ),
		.axi_wvalid     (camera_axi_wvalid[0]       ),
		.axi_wready     (camera_axi_wready[0]       ),

		.axi_bid        (camera_axi_bid[0 +: 4]          ),
		.axi_bresp      (camera_axi_bresp[0 +: 2]        ),
		.axi_bvalid     (camera_axi_bvalid[0]       ),
		.axi_bready     (camera_axi_bready[0]       ),

		.axi_arid       (camera_axi_arid[0 +: 4]         ),
		.axi_araddr     (camera_axi_araddr[0 +: 32]       ),
		.axi_arlen      (camera_axi_arlen[0 +: 8]        ),
		.axi_arvalid    (camera_axi_arvalid[0]      ),
		.axi_arready    (camera_axi_arready[0]      ),

		.axi_rid        (camera_axi_rid[0 +: 4]          ),
		.axi_rdata      (camera_axi_rdata[0 +: 128]        ),
		.axi_rresp      (camera_axi_rresp[0 +: 2]        ),
		.axi_rlast      (camera_axi_rlast[0]        ),
		.axi_rvalid     (camera_axi_rvalid[0]       ),
		.axi_rready     (camera_axi_rready[0]       ),

		.wframe_pclk    (cmos_pclk          ),
		.wframe_vsync   (pre_ddr_frame_vsync),
		.wframe_data_en (packed_overlay_valid),
		.wframe_data    (packed_overlay_data),
		
		.rframe_pclk    (clk_pixel            ),
		.rframe_vsync   (~lcd_vs             ),		//	Reader VSync. Flush on rising edge. Connect to ~EOF. 
		.rframe_data_en (lcd_request             ),
		.rframe_data    (dvp_lcd_data           ),
		
		.tp_o 		(w_axi_tp)
	);
	assign quad_committed_frames = 32'd0;
	assign quad_dropped_frames = 32'd0;
	end
	endgenerate
	assign led_o[3:0] = w_axi_tp[3:0];
	
	
	
	
	////////////////////////////////////////////////////////////////
	//  LCD Timing Driver
	
	multires_lcd_driver u_lcd_driver
	(
	    //  global clock
	    .clk        (clk_pixel   ),
	    .rst_n      (display_rst_n),
        .pixel_enable_i(pixel_enable), .mode_1080_i(select_canny), 
	    
	    //  lcd interface
	    .lcd_dclk   (               ),
	    .lcd_blank  (               ),
	    .lcd_sync   (               ),
	    .lcd_request(lcd_request    ), 	//	Request data 1 cycle ahead. 
	    .lcd_hs     (lcd_hs         ),
	    .lcd_vs     (lcd_vs         ),
	    .lcd_en     (lcd_de         ),
	    .lcd_rgb    ({lcd_red2,lcd_green2,lcd_blue2, lcd_red,lcd_green,lcd_blue}),
	    
	    //  user interface
	    .lcd_data   ({{lcd_data[15:11], 3'b000,
	                   lcd_data[10:5],  2'b00,
	                   lcd_data[4:0],   3'b000},
	                  {lcd_data[15:11], 3'b000,
	                   lcd_data[10:5],  2'b00,
	                   lcd_data[4:0],   3'b000}})
	);
	
// =========================================================================================================================================
// Post-DDR RGB565 display path
// =========================================================================================================================================
wire       w_rgb_vsync = lcd_vs;
wire       w_rgb_hsync = lcd_hs;
wire       w_rgb_href = lcd_de;
wire [7:0] w_rgb_r = lcd_red;
wire [7:0] w_rgb_g = lcd_green;
wire [7:0] w_rgb_b = lcd_blue;

reg [2:0] display_previous_sys;
always @(posedge clk_sys or negedge rstn_sys) begin
    if(!rstn_sys) display_previous_sys<=2'd3;
    else display_previous_sys<=display_requested_sys;
end
wire display_changed_sys=display_previous_sys!=display_requested_sys;
wire [7:0] selected_history_count=display_requested_sys==0 ? dvp_history_count : mipi_history_count;
// JTAG候选关闭UART命令接收，TX保持空闲，避免双控制源冲突。
assign uart_tx_o = 1'b1;
jtag_video_control #(
    .DEFAULT_THRESHOLD(SOBEL_DEFAULT_THRESHOLD),
    .MAX_THRESHOLD    (10'd1020)
) u_jtag_video_control (
    .clk               (clk_sys),
    .rst_n             (rstn_sys),
    .jtag_tck_i(jtag_inst1_TCK), .jtag_tdi_i(jtag_inst1_TDI),
    .jtag_capture_i(jtag_inst1_CAPTURE), .jtag_shift_i(jtag_inst1_SHIFT),
    .jtag_update_i(jtag_inst1_UPDATE), .jtag_reset_i(jtag_inst1_RESET),
    .jtag_select_i(jtag_inst1_SEL), .jtag_tdo_o(jtag_inst1_TDO),
    .threshold_o       (sobel_threshold_sys),
    .threshold_valid_o (sobel_threshold_valid_sys),
    .threshold_ready_i (sobel_threshold_ready_sys),
    .canny_low_o(canny_low_sys),.canny_high_o(canny_high_sys),
    .canny_valid_o(canny_threshold_valid),.canny_ready_i(canny_threshold_ready),
    .display_mode_i(display_requested_sys),.history_count_i(selected_history_count),
    .video_fault_i({4'd0,canny_display_detail}),.display_changed_i(display_changed_sys),
    .display_request_o(serial_display_request),.display_valid_o(serial_display_valid),
    .history_control_o(history_control_sys),.history_age_a_o(history_age_a_sys),
    .history_age_b_o(history_age_b_sys),
    .background_mask_capture_o(background_mask_capture),
    .background_mask_address_o(background_mask_address),.background_mask_data_i(background_mask_data),
    .background_mask_ready_i(background_mask_ready),.background_mask_busy_i(background_mask_busy),
    .background_mask_epoch_i(background_mask_epoch),
    .background_threshold_o(background_threshold),.background_relearn_o(background_relearn),
    .background_valid_i(background_valid),.background_pending_i(background_pending),
    .background_epoch_i(background_epoch),.background_frames_i(background_frames),
    .background_cycles_i(background_cycles),.background_count_i(background_count),.background_overflow_i(background_overflow),
    .osd_wr_en_o       (osd_text_wr_en_sys),
    .osd_wr_addr_o     (osd_text_wr_addr_sys),
    .osd_wr_data_o     (osd_text_wr_data_sys)
);

sobel_threshold_cdc #(
    .DEFAULT_THRESHOLD(SOBEL_DEFAULT_THRESHOLD)
) u_sobel_threshold_cdc (
    .src_clk         (clk_sys),
    .src_rst_n       (rstn_sys),
    .src_threshold_i (sobel_threshold_sys),
    .src_valid_i     (sobel_threshold_valid_sys),
    .src_ready_o     (sobel_threshold_ready_sys),
    .dst_clk         (cmos_pclk),
    .dst_rst_n       (cmos_pipeline_rst_n),
    .dst_threshold_o (sobel_threshold_cdc_value)
);

wire       sobel_vsync = w_rgb_vsync;
wire       sobel_hsync = w_rgb_hsync;
wire       sobel_de = w_rgb_href;
wire [23:0] sobel_rgb_data = {w_rgb_r, w_rgb_g, w_rgb_b};


wire                                    boundcrop_vs               ;
wire                                    boundcrop_hs               ;
wire                                    boundcrop_de               ;
wire                   [  23:0]         boundcrop_data             ;
FrameBoundCrop #(.SKIP_ROWS(2),.SKIP_COLS(2),.TOTAL_ROWS(720),.TOTAL_COLS(1280)) inst2_FrameCrop(
    .pixel_enable_i(pixel_enable),.mode_1080_i(select_canny),
    .clk_i                             (clk_pixel                 ),
    .rst_i                             (~display_rst_n            ),
	
	.vs_i                              (sobel_vsync               ),
	.hs_i                              (sobel_hsync               ),
	.de_i                              (sobel_de                  ),
	.data_i                            (sobel_rgb_data            ),
	
    .vs_o                              (boundcrop_vs              ),
    .hs_o                              (boundcrop_hs              ),
    .de_o                              (boundcrop_de              ),
    .data_o                            (boundcrop_data            ) 
);

wire                                    osd_video_vs               ;
wire                                    osd_video_hs               ;
wire                                    osd_video_de               ;
wire                   [  23:0]         osd_video_data             ;

// Canny diagnostics occupy only its existing left black margin.
wire [13:0] canny_path_status;
wire [3:0] canny_display_status,canny_display_detail;
wire monitor_vs,monitor_hs,monitor_de;
wire [23:0] monitor_rgb;
// Keep OSD as the final processing stage: DDR read -> boundary crop -> Canny monitor -> OSD -> HDMI.
wire [10:0] display_font_address,render_font_address;
wire display_font_enable,render_font_enable;
wire [15:0] display_font_pixels,render_font_pixels;
osd_font_dual_rom shared_font_storage (
    .display_clk_i(clk_pixel),.display_enable_i(display_font_enable),
    .display_address_i(display_font_address),.display_pixels_o(display_font_pixels),
    .render_clk_i(w_ddr3_ui_clk),.render_enable_i(render_font_enable),
    .render_address_i(render_font_address),.render_pixels_o(render_font_pixels)
);
osd_text_overlay #(
    .H_ACTIVE  (1920),
    .V_ACTIVE  (1080),
    .OSD_X     (32),
    .OSD_Y     (32),
    .TEXT_COLOR(24'hFFFF00),
    .MODE_LABEL_ENABLE(1),.SHARED_FONT(1)
) u_osd_text_overlay (
    .font_address_o(display_font_address),.font_enable_o(display_font_enable),.font_pixels_i(display_font_pixels),
    .pixel_clk_i    (clk_pixel),
    .pixel_rst_n_i  (display_rst_n),
    .mode_i        (selected_mode),
    .pixel_enable_i(pixel_enable),
    .video_vs_i     (monitor_vs),
    .video_hs_i     (monitor_hs),
    .video_de_i     (monitor_de),
    .video_rgb_i    (monitor_rgb),
    .text_wr_clk_i  (clk_sys),
    .text_wr_en_i   (osd_text_wr_en_sys),
    .text_wr_addr_i (osd_text_wr_addr_sys),
    .text_wr_data_i (osd_text_wr_data_sys),
    .video_vs_o     (osd_video_vs),
    .video_hs_o     (osd_video_hs),
    .video_de_o     (osd_video_de),
    .video_rgb_o    (osd_video_data)
);



	assign hdmi_txd0_rst_o = ~display_rst_n; 
	assign hdmi_txd1_rst_o = ~display_rst_n; 
	assign hdmi_txd2_rst_o = ~display_rst_n; 
	assign hdmi_txc_rst_o = ~display_rst_n; 
	
	assign hdmi_txd0_oe = display_rst_n; 
	assign hdmi_txd1_oe = display_rst_n; 
	assign hdmi_txd2_oe = display_rst_n; 
	assign hdmi_txc_oe = display_rst_n; 
	

    rgb2dvi #(.ENABLE_OSERDES(0)) u_rgb2dvi
    (
    .pixel_enable_i(pixel_enable),
    .oe_i                              (1                         ),//	Always enable output
    .bitflip_i                         (4'b0000                   ),//	Reverse clock & data lanes. 
		
    .aRst                              (1'b0                      ),
    .aRst_n                            (1'b1                      ),
		
    .PixelClk                          (clk_pixel                 ),//pixel clk = 74.25M
    .SerialClk                         (                          ),//pixel clk *5 = 371.25M
		
    .vid_pVSync                        (~osd_video_vs             ),
    .vid_pHSync                        (~osd_video_hs             ),
    .vid_pVDE                          (osd_video_de              ),
    .vid_pData                         (osd_video_data            ),
		
    .txc_o                             (                          ),
    .txd0_o                            (encoded0                  ),
    .txd1_o                            (encoded1                  ),
    .txd2_o                            (encoded2                  ) 
    );
		
	






	
	

    wire canny_axi_rst_n = !w_ddr3_ui_rst && r_ddr_unlock;
    (* async_reg="true" *) reg [2:0] canny_camera_reset;
    always @(posedge w_csi_rx_clk or negedge canny_axi_rst_n) begin
        if (!canny_axi_rst_n) canny_camera_reset <= 3'b000;
        else canny_camera_reset <= {canny_camera_reset[1:0],1'b1};
    end
    wire canny_display_frame, canny_display_valid, canny_error,canny_display_gray;
    wire canny_display_ready, canny_display_error;
    wire canny_frame_publish_toggle;
    wire [63:0] canny_display_data;
    wire [15:0] canny_lcd_data;
    assign camera_axi_arid[15:8] = 8'd0;
    reg select_rectangle;
    reg select_capture_raw;
    reg [1:0] compare_request_sys;
    // The system-domain mode mailbox is stable until the next user request.
    // Capture latches this level at sensor SOP after its three-stage crossing.
    always @(posedge clk_sys or negedge rstn_sys) begin
        if(!rstn_sys) compare_request_sys<=1'b0;
        else compare_request_sys<=display_requested_sys==3'd6 ? 2'b10 : (display_requested_sys==3'd5 ? 2'b01 : 2'b00);
    end
    always @(posedge clk_pixel or negedge rstn_pixel) begin
        if(!rstn_pixel) begin select_rectangle<=1'b1;select_capture_raw<=1'b0;end
        else begin
            select_rectangle<=selected_mode==2'd3;
            select_capture_raw<=selected_mode==3'd2 || selected_mode==3'd4;
        end
    end
    // DVP capture, four effects and DDR writer remain active for coexistence.
    // K1 selects annotated Canny, plain Canny, MIPI color, then DVP four quadrants.
    camera_display_select #(.ENABLE_RECTANGLE(1),.ENABLE_BACKGROUND(1),.ENABLE_COMPARE(1),.ENABLE_OVERLAY(1),.SKIP_DVP_DISPLAY(0),.COMBINED_EDGE_VIEW(0),.UART_CONTROL(1)) display_select (
        .remote_mode_i(serial_display_request),.remote_valid_i(serial_display_valid),.requested_mode_o(display_requested_sys),
        .system_clk_i(clk_sys), .system_rst_n_i(rstn_sys), .key_n_i(key_switch_n_i),
        .pixel_clk_i(clk_pixel), .pixel_rst_n_i(rstn_pixel), .vsync_i(lcd_vs),
        .mode_o(requested_mode)
    );
    hdmi_mode_control display_mode (
        .clk_i(clk_pixel),.rst_n_i(rstn_pixel),.requested_mode_i(requested_mode),.mode_o(selected_mode),
        .mode_1080_o(select_canny),.mode_raw_o(select_raw),.display_rst_n_o(display_rst_n),.pixel_enable_o(pixel_enable)
    );
    hdmi_rate_adapter rate_adapter (
        .clk_i(clk_pixel),.rst_n_i(display_rst_n),.mode_1080_i(select_canny),.pixel_enable_i(pixel_enable),
        .channel0_i(encoded0),.channel1_i(encoded1),.channel2_i(encoded2),
        .clock_o(hdmi_txc_o),.channel0_o(hdmi_txd0_o),.channel1_o(hdmi_txd1_o),.channel2_o(hdmi_txd2_o)
    );
    assign lcd_data = select_canny ? canny_lcd_data : dvp_lcd_data;
    wire sc233_config_rst_n;
    wire sc233_receiver_reset;
    assign {csi_rxd3_rst_o,csi_rxd2_rst_o,csi_rxd1_rst_o,csi_rxd0_rst_o} = {4{sc233_receiver_reset}};
	wire            [ 8:0]          sc2210_i2c_config_index;
	wire            [23:0]          sc2210_i2c_config_data;
	wire            [ 8:0]          sc2210_i2c_config_size;
	wire                            sc2210_i2c_config_done;
	
	sc233_i2c_timing
	#(
	    .CLK_FREQ           (CLOCK_MAIN),                              //  100 MHz
	    .I2C_FREQ           (50_000    )                               //  10 KHz(<= 400KHz)
	) u_sc233_i2c_timing (
	    //  global clock
	    .clk                (clk_sys                 ),                          //  96MHz
	    .rst_n              (sc233_config_rst_n     ),                          // supervised sensor configuration

	    //  i2c interface     
	    .i2c_sclk           (csi_scl_o               ),                          //  i2c clock
	    .i2c_sdat_IN        (csi_sda_i               ),
	    .i2c_sdat_OUT       (csi_sda_o               ),
	    .i2c_sdat_OE        (csi_sda_oe              ),

	    //  i2c config data
	    .i2c_config_index   (sc2210_i2c_config_index        ),                          //  i2c config reg index, read 2 reg and write xx reg
	    .i2c_config_data    ({8'h60, sc2210_i2c_config_data}),                     //  i2c config data
	    .i2c_config_size    (sc2210_i2c_config_size         ),                          //  i2c config data counte
	    .i2c_config_done    (sc2210_i2c_config_done         )                          //  i2c config timing complete
	);
	assign csi_scl_oe = 1; 

	//  I2C Configure Data of SC2210
	I2C_SC233HGS_19201200_4Lanes_Config u_I2C_SC233HSG_19201200_4Lanes_Config
	(
	    .LUT_INDEX  (sc2210_i2c_config_index   ),
	    .LUT_DATA   (sc2210_i2c_config_data    ),
	    .LUT_SIZE   (sc2210_i2c_config_size    )
	);

	localparam 	SC233_CSI_RXD_INV 	= 4'b1111; 
	localparam 	SC233_CSI_DATA_WIDTH 	= 32; 			
	localparam 	SC233_CSI_STRB_WIDTH 	= SC233_CSI_DATA_WIDTH / 8; 

	//	Current implementation supports RAW8 only. 
	wire 			w_csi_rx_clk; 
	wire 			w_csi_rx_vsync0, w_csi_rx_hsync0, w_csi_rx_dvalid; 
	wire 	[63:0] 	w_csi_rx_data; 
	wire 	[47:0] 	w_csi_rx_data_rel_raw; 

    sc233_link_recovery #(.SYS_CLK_HZ(CLOCK_MAIN)) sc233_recovery (
        .system_clk_i(clk_sys), .system_rst_n_i(rstn_sys),
        .config_done_i(sc2210_i2c_config_done),
        .camera_clk_i(w_csi_rx_clk), .camera_rst_n_i(monitor_camera_reset[2]),
        .frame_valid_i(w_csi_rx_vsync0), .line_valid_i(w_csi_rx_hsync0),
        .group_valid_i(w_csi_rx_dvalid),
        .config_rst_n_o(sc233_config_rst_n), .receiver_reset_o(sc233_receiver_reset)
    );
	
	//	AXI Interface
	wire 	[31:0] 	w_csi_axi_rdata; 
	wire 			w_csi_axi_awready, w_csi_axi_wready, w_csi_axi_arready, w_csi_axi_rvalid; 
	
	
	

	
	

	wire 	[7:0] 	w_mipi_d0 = (SC233_CSI_RXD_INV[0] ? 8'hFF : 8'h00) ^ csi_rxd0_hs_i; 
	wire 	[7:0] 	w_mipi_d1 = (SC233_CSI_RXD_INV[1] ? 8'hFF : 8'h00) ^ csi_rxd1_hs_i; 
	wire 	[7:0] 	w_mipi_d2 = (SC233_CSI_RXD_INV[2] ? 8'hFF : 8'h00) ^ csi_rxd2_hs_i; 
	wire 	[7:0] 	w_mipi_d3 = (SC233_CSI_RXD_INV[3] ? 8'hFF : 8'h00) ^ csi_rxd3_hs_i; 
    wire [31:0] csi_stable_bytes;
    sc233_byte_capture csi_byte_capture (
        .byte_clk_i(csi_rxc_i),
        .bytes_i({w_mipi_d3,w_mipi_d2,w_mipi_d1,w_mipi_d0}),
        .bytes_o(csi_stable_bytes)
    );
	MIPIRx4LaneFre mipi0_rx (
		.sclk_i			(clk_sys), 
		.srst_i			(sc233_receiver_reset), 
		
		.mipi_rx_byteclk_i	(csi_rxc_i), 
		.mipi_rx_d0_i		({csi_stable_bytes[0], csi_stable_bytes[1], csi_stable_bytes[2], csi_stable_bytes[3], csi_stable_bytes[4], csi_stable_bytes[5], csi_stable_bytes[6], csi_stable_bytes[7]}), 
		.mipi_rx_d1_i		({csi_stable_bytes[8], csi_stable_bytes[9], csi_stable_bytes[10], csi_stable_bytes[11], csi_stable_bytes[12], csi_stable_bytes[13], csi_stable_bytes[14], csi_stable_bytes[15]}), 
		.mipi_rx_d2_i		({csi_stable_bytes[16], csi_stable_bytes[17], csi_stable_bytes[18], csi_stable_bytes[19], csi_stable_bytes[20], csi_stable_bytes[21], csi_stable_bytes[22], csi_stable_bytes[23]}), 
		.mipi_rx_d3_i		({csi_stable_bytes[24], csi_stable_bytes[25], csi_stable_bytes[26], csi_stable_bytes[27], csi_stable_bytes[28], csi_stable_bytes[29], csi_stable_bytes[30], csi_stable_bytes[31]}), 
		.mipi_rx_clk_lp_p_i	(csi_rxc_lp_p_i), 
		.mipi_rx_clk_lp_n_i	(csi_rxc_lp_n_i), 
		.mipi_rx_d0_lp_p_i	(SC233_CSI_RXD_INV[0] ? csi_rxd0_lp_n_i : csi_rxd0_lp_p_i), 
		.mipi_rx_d0_lp_n_i	(SC233_CSI_RXD_INV[0] ? csi_rxd0_lp_p_i : csi_rxd0_lp_n_i), 
		.mipi_rx_rst_o		(), 
		.mipi_rx_rst_n_o		(), 
		.mipi_rx_clk_hs_en_o	(csi_rxc_hs_en_o), 
		.mipi_rx_dat_hs_en_o	(csi_rxd0_hs_en_o), 
		.mipi_rx_clk_term_en_o	(csi_rxc_hs_term_en_o), 
		.mipi_rx_dat_term_en_o	(csi_rxd0_hs_term_en_o), 
		
		.CLK				(w_csi_rx_clk), 
		.VSYNC			(w_csi_rx_vsync0), 
		.HSYNC			(w_csi_rx_hsync0), 
		.DE				(w_csi_rx_dvalid), 
		.DAT				(w_csi_rx_data_rel_raw)
	);
	assign w_csi_rx_data = {2{w_csi_rx_data_rel_raw[47:40], w_csi_rx_data_rel_raw[35:28], w_csi_rx_data_rel_raw[23:16], w_csi_rx_data_rel_raw[11:4]}}; 
	assign {csi_rxd3_hs_en_o, csi_rxd2_hs_en_o, csi_rxd1_hs_en_o} = {3{csi_rxd0_hs_en_o}}; 
	assign {csi_rxd3_hs_term_en_o, csi_rxd2_hs_term_en_o, csi_rxd1_hs_term_en_o} = {3{csi_rxd0_hs_term_en_o}}; 
	
	
    canny_binary_frame_ctrl #(.C_HISTORY_ENABLE(1),.SUPPORT_PALETTE(1),.SUPPORT_RGB(1),.SUPPORT_GRAY(1),.C_READ_BANKS(2), .C_BASE_ADDR(32'h10000000), .C_RD_END_ADDR(1920*1080*2), .C_W_WIDTH(64), .C_R_WIDTH(16), .C_ID_LEN(4)) canny_display_controller (
        .history_control_i(history_control_sys),.history_age_a_i(history_age_a_sys),
        .history_age_b_i(history_age_b_sys),.history_count_o(mipi_history_count),
        .frame_publish_toggle_o(canny_frame_publish_toggle),
        .wframe_gray_i(canny_display_gray),.rframe_gray_i(select_raw),
        .axi_awaddr(camera_axi_awaddr[32 +: 32]),
        .axi_awlen(camera_axi_awlen[8 +: 8]),
        .axi_awvalid(camera_axi_awvalid[1]),
        .axi_awready(camera_axi_awready[1]),
        .axi_wdata(camera_axi_wdata[128 +: 128]),
        .axi_wstrb(camera_axi_wstrb[16 +: 16]),
        .axi_wlast(camera_axi_wlast[1]),
        .axi_wvalid(camera_axi_wvalid[1]),
        .axi_wready(camera_axi_wready[1]),
        .axi_bid(camera_axi_bid[4 +: 4]),
        .axi_bresp(camera_axi_bresp[2 +: 2]),
        .axi_bvalid(camera_axi_bvalid[1]),
        .axi_bready(camera_axi_bready[1]),
        .axi_arid(camera_axi_arid[4 +: 4]),
        .axi_araddr(camera_axi_araddr[32 +: 32]),
        .axi_arlen(camera_axi_arlen[8 +: 8]),
        .axi_arvalid(camera_axi_arvalid[1]),
        .axi_arready(camera_axi_arready[1]),
        .axi_rid(camera_axi_rid[4 +: 4]),
        .axi_rdata(camera_axi_rdata[128 +: 128]),
        .axi_rresp(camera_axi_rresp[2 +: 2]),
        .axi_rlast(camera_axi_rlast[1]),
        .axi_rvalid(camera_axi_rvalid[1]),
        .axi_rready(camera_axi_rready[1]),
        .axi_clk(w_ddr3_ui_clk),
        .axi_reset(!canny_axi_rst_n),
        .wframe_pclk(w_ddr3_ui_clk),
        .wframe_vsync(canny_display_frame),
        .wframe_data_en(canny_display_valid),
        .wframe_data(canny_display_data),
        .wframe_ready_o(canny_display_ready),
        .error_o(canny_display_error),
        .debug_status_o(canny_display_status),.detail_fault_o(canny_display_detail),
        .rframe_pclk(clk_pixel),
        .rframe_vsync(~lcd_vs),
        .rframe_data_en(lcd_request && select_canny),
        .rframe_data(canny_lcd_data),
        .tp_o()
    );
    canny_camera_path #(.MAX_OBJECTS(6),.ENABLE_EXTENDED_SHAPES(1),.SUPPORT_COMPARE(1),.SUPPORT_RECTANGLE(1),.SHARED_FONT(1),.SUPPORT_RGB(1),.DISPLAY_WIDTH(1920), .DISPLAY_HEIGHT(1080), .SCALE_NUM(10), .SCALE_DEN(9)) canny_path (
        .compare_request_i(compare_request_sys),
        .font_address_o(render_font_address),.font_enable_o(render_font_enable),.font_pixels_i(render_font_pixels),
        .rectangle_request_i(select_rectangle),
        .background_request_i(display_requested_sys==3'd4),.background_relearn_i(background_relearn),
        .background_mask_capture_i(background_mask_capture),
        .background_mask_address_i(background_mask_address),.background_mask_data_o(background_mask_data),
        .background_mask_ready_o(background_mask_ready),.background_mask_busy_o(background_mask_busy),
        .background_mask_epoch_o(background_mask_epoch),
        .background_threshold_i(background_threshold),.background_valid_o(background_valid),
        .background_pending_o(background_pending),.background_epoch_o(background_epoch),
        .background_frames_o(background_frames),.background_cycles_o(background_cycles),
        .background_count_o(background_count),.background_overflow_o(background_overflow),
        .raw_request_i(select_capture_raw),.display_gray_o(canny_display_gray),
        .axi_awaddr_o(camera_axi_awaddr[64 +: 64]),
        .axi_awlen_o(camera_axi_awlen[16 +: 16]),
        .axi_awvalid_o(camera_axi_awvalid[2 +: 2]),
        .axi_awready_i(camera_axi_awready[2 +: 2]),
        .axi_wdata_o(camera_axi_wdata[256 +: 256]),
        .axi_wstrb_o(camera_axi_wstrb[32 +: 32]),
        .axi_wlast_o(camera_axi_wlast[2 +: 2]),
        .axi_wvalid_o(camera_axi_wvalid[2 +: 2]),
        .axi_wready_i(camera_axi_wready[2 +: 2]),
        .axi_bresp_i(camera_axi_bresp[4 +: 4]),
        .axi_bvalid_i(camera_axi_bvalid[2 +: 2]),
        .axi_bready_o(camera_axi_bready[2 +: 2]),
        .axi_araddr_o(camera_axi_araddr[64 +: 64]),
        .axi_arlen_o(camera_axi_arlen[16 +: 16]),
        .axi_arvalid_o(camera_axi_arvalid[2 +: 2]),
        .axi_arready_i(camera_axi_arready[2 +: 2]),
        .axi_rdata_i(camera_axi_rdata[256 +: 256]),
        .axi_rresp_i(camera_axi_rresp[4 +: 4]),
        .axi_rlast_i(camera_axi_rlast[2 +: 2]),
        .axi_rvalid_i(camera_axi_rvalid[2 +: 2]),
        .axi_rready_o(camera_axi_rready[2 +: 2]),
        .camera_clk_i(w_csi_rx_clk),
        .camera_rst_n_i(canny_camera_reset[2]),
        .axi_clk_i(w_ddr3_ui_clk),
        .axi_rst_n_i(canny_axi_rst_n),
        .frame_vsync_i(w_csi_rx_vsync0),
        .frame_href_i(w_csi_rx_hsync0),
        .pixel_valid_i(w_csi_rx_dvalid),
        .gray_i(w_csi_rx_data[31:0]),
        .low_threshold_i(canny_threshold_camera[10:0]), .high_threshold_i(canny_threshold_camera[21:11]),
        .display_frame_o(canny_display_frame),
        .display_valid_o(canny_display_valid),
        .display_data_o(canny_display_data),
        .display_ready_i(canny_display_ready),
        .error_o(canny_error),
        .debug_status_o(canny_path_status),
        .last_cycles_o(),
        .last_sweeps_o()
    );
    (* async_reg="true" *) reg [2:0] monitor_camera_reset;
    always @(posedge w_csi_rx_clk or negedge rstn_sys) begin
        if(!rstn_sys) monitor_camera_reset<=3'b000;
        else monitor_camera_reset<={monitor_camera_reset[1:0],1'b1};
    end
    canny_video_monitor #(.TONE_MAP_ENABLE(0),.SHOW_PROGRESS(1),.DIAGNOSTIC_X(1824)) video_monitor (
        .frame_publish_toggle_i(canny_frame_publish_toggle),
        .original_mode_i(select_raw),
        .camera_clk_i(w_csi_rx_clk),.camera_rst_n_i(monitor_camera_reset[2]),
        .pixel_clk_i(clk_pixel),.pixel_rst_n_i(rstn_pixel),
        .video_rst_n_i(display_rst_n),.pixel_enable_i(pixel_enable),.overlay_enable_i(1'b0), // 暂时隐藏诊断条；恢复时改回 selected_mode==1 || selected_mode==3。
        .sensor_vsync_i(w_csi_rx_vsync0),.sensor_href_i(w_csi_rx_hsync0),.sensor_valid_i(w_csi_rx_dvalid),
        .calibration_done_i(w_ddr3_cal_done),.calibration_pass_i(w_ddr3_cal_pass),
        .path_status_i(canny_path_status),.display_status_i(canny_display_status),.detail_fault_i(canny_display_detail),
        .vsync_i(boundcrop_vs),.hsync_i(boundcrop_hs),.valid_i(boundcrop_de),.rgb_i(boundcrop_data),
        .vsync_o(monitor_vs),.hsync_o(monitor_hs),.valid_o(monitor_de),.rgb_o(monitor_rgb)
    );
    camera_axi_shared camera_interconnect (
        .clk(w_ddr3_ui_clk),
        .rst_n(canny_axi_rst_n),
        .s_axi_awvalid(camera_axi_awvalid),
        .s_axi_awaddr(camera_axi_awaddr),
        .s_axi_awlock(8'd0),
        .s_axi_awready(camera_axi_awready),
        .s_axi_arvalid(camera_axi_arvalid),
        .s_axi_araddr(camera_axi_araddr),
        .s_axi_arlock(8'd0),
        .s_axi_arready(camera_axi_arready),
        .s_axi_wvalid(camera_axi_wvalid),
        .s_axi_wlast(camera_axi_wlast),
        .s_axi_wid(16'd0),
        .s_axi_bready(camera_axi_bready),
        .s_axi_bresp(camera_axi_bresp),
        .s_axi_rready(camera_axi_rready),
        .s_axi_bid(camera_axi_bid),
        .s_axi_rid(camera_axi_rid),
        .s_axi_wdata(camera_axi_wdata),
        .s_axi_rdata(camera_axi_rdata),
        .s_axi_rresp(camera_axi_rresp),
        .s_axi_bvalid(camera_axi_bvalid),
        .s_axi_rvalid(camera_axi_rvalid),
        .s_axi_rlast(camera_axi_rlast),
        .s_axi_wstrb(camera_axi_wstrb),
        .m_axi_awvalid(w_ddr3_awvalid),
        .m_axi_awaddr(w_ddr3_awaddr),
        .m_axi_awlock(),
        .m_axi_awready(w_ddr3_awready),
        .m_axi_arvalid(w_ddr3_arvalid),
        .m_axi_araddr(w_ddr3_araddr),
        .m_axi_arlock(),
        .m_axi_arready(w_ddr3_arready),
        .m_axi_wvalid(w_ddr3_wvalid),
        .m_axi_wlast(w_ddr3_wlast),
        .m_axi_bready(w_ddr3_bready),
        .m_axi_bresp(w_ddr3_bresp),
        .m_axi_rready(w_ddr3_rready),
        .m_axi_bid(w_ddr3_bid),
        .m_axi_rid(w_ddr3_rid),
        .m_axi_wdata(w_ddr3_wdata),
        .m_axi_rdata(w_ddr3_rdata),
        .m_axi_rresp(w_ddr3_rresp),
        .m_axi_bvalid(w_ddr3_bvalid),
        .m_axi_rvalid(w_ddr3_rvalid),
        .m_axi_rlast(w_ddr3_rlast),
        .m_axi_wstrb(w_ddr3_wstrb),
        .m_axi_wready(w_ddr3_wready),
        .s_axi_wready(camera_axi_wready),
        .s_axi_awprot(16'd0),
        .s_axi_awcache(16'd0),
        .s_axi_awqos(16'd0),
        .s_axi_awuser(4'd0),
        .s_axi_arqos(16'd0),
        .s_axi_arcache(16'd0),
        .m_axi_awprot(),
        .s_axi_arid(camera_axi_arid),
        .s_axi_arsize({4{3'd4}}),
        .s_axi_arlen(camera_axi_arlen),
        .s_axi_arburst({4{2'b01}}),
        .s_axi_arprot(16'd0),
        .s_axi_awid(16'd0),
        .s_axi_awburst({4{2'b01}}),
        .s_axi_awlen(camera_axi_awlen),
        .s_axi_awsize({4{3'd4}}),
        .m_axi_awid(w_ddr3_awid),
        .m_axi_awburst(),
        .m_axi_awlen(w_ddr3_awlen),
        .m_axi_awsize(),
        .m_axi_awcache(),
        .m_axi_awqos(),
        .m_axi_awuser(),
        .m_axi_arprot(),
        .m_axi_arburst(),
        .m_axi_arlen(w_ddr3_arlen),
        .m_axi_arsize(),
        .m_axi_arcache(),
        .m_axi_arqos(),
        .m_axi_aruser(),
        .m_axi_awregion(),
        .m_axi_arregion(),
        .m_axi_arid(w_ddr3_arid),
        .m_axi_wuser(),
        .m_axi_ruser(1'd0),
        .m_axi_buser(1'd0),
        .s_axi_aruser(4'd0),
        .s_axi_wuser(4'd0),
        .s_axi_buser(),
        .s_axi_ruser()
    );

endmodule


