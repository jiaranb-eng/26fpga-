// Derived from the supplied SC1336 board axi4_ctrl.v, preserving its reader
// and four-slot addressing. Canny-specific changes: camera FIFO ready and
// successful B-response completion before write-slot publication.
// C_W_WIDTH must remain 64; generated Efinix FIFO implementations are unmodified.
//  by CrazyBird
module canny_frame_ctrl #(
	parameter 	C_ID_LEN      = 8, 
			C_DATA_LEN    = 128, 
			C_DATA_SIZE   = 4, 	//	C_DATA_LEN=8:0, 16:1, 32:2, 64:3, 128:4, 256:5, 512:6, 1024:7
			
			C_BURST_LEN   = 128,
			
			C_STRB_LEN    = C_DATA_LEN / 8, 
			C_ADDR_INC    = C_BURST_LEN * C_STRB_LEN,
			
			C_BASE_ADDR   = 32'h00000000, 	//	Start at 0x00000000 by default
			C_BUF_SIZE    = 22, 			//	Allocate 2^22 bytes (4MB) by default. The module takes 4*4MB by default. 
			C_RD_END_ADDR = 1280 * 720,
			
			C_W_WIDTH     = 64, 
			C_R_WIDTH     = 8,
            // Stripe whole bursts across unchanged vendor FIFOs. Eight banks
            // absorb DDR service gaps without dropping the rest of a frame.
            C_READ_BANKS  = 8,
            C_GRAY_END_ADDR = 0,
            C_HISTORY_ENABLE = 0,
            C_SLOT_BITS = C_HISTORY_ENABLE ? 5 : 2,
            C_COMPARE_SPLIT = 522240,
            C_GRAY_COMPARE_SPLIT = 2088960
)(
    input wire [1:0] history_control_i,
    input wire [7:0] history_age_a_i,
    input wire [7:0] history_age_b_i,
    output wire [7:0] history_count_o,
    input wire wframe_gray_i, input wire rframe_gray_i,
    input  wire                 axi_clk         ,
    input  wire                 axi_reset       ,
    
    output       [C_ID_LEN-1:0]      axi_awid        ,
    output       [31:0]      axi_awaddr      ,
    output       [ 7:0]      axi_awlen       ,
    output       [ 2:0]      axi_awsize      ,
    output       [ 1:0]      axi_awburst     ,
    output                   axi_awlock      ,
    output       [ 3:0]      axi_awcache     ,
    output       [ 2:0]      axi_awprot      ,
    output       [ 3:0]      axi_awqos       ,
    output reg                  axi_awvalid     ,
    input  wire                 axi_awready     ,
    
    output       [C_DATA_LEN-1:0]      axi_wdata       ,
    output       [C_STRB_LEN-1:0]      axi_wstrb       ,
    output                   axi_wlast       ,
    output reg                  axi_wvalid      ,
    input  wire                 axi_wready      ,
    
    input  wire     [C_ID_LEN-1:0]      axi_bid         ,
    input  wire     [ 1:0]      axi_bresp       ,
    input  wire                 axi_bvalid      ,
    output                   axi_bready      ,
    
    output       [C_ID_LEN-1:0]      axi_arid        ,
    output       [31:0]      axi_araddr      ,
    output       [ 7:0]      axi_arlen       ,
    output       [ 2:0]      axi_arsize      ,
    output       [ 1:0]      axi_arburst     ,
    output                   axi_arlock      ,
    output       [ 3:0]      axi_arcache     ,
    output       [ 2:0]      axi_arprot      ,
    output       [ 3:0]      axi_arqos       ,
    output reg                  axi_arvalid     ,
    input  wire                 axi_arready     ,
    
    input  wire     [C_ID_LEN-1:0]      axi_rid         ,
    input  wire     [C_DATA_LEN-1:0]      axi_rdata       ,
    input  wire     [ 1:0]      axi_rresp       ,
    input  wire                 axi_rlast       ,
    input  wire                 axi_rvalid      ,
    output                   axi_rready      ,
    
    input  wire                 wframe_pclk     ,
    input  wire                 wframe_vsync    ,		//	Writter VSync. Flush on falling edge. Connect to EOF. 
    output wire                 wframe_ready_o,
    output wire                 error_o,
    output wire [3:0]            debug_status_o,
    output wire [2:0]            detail_fault_o,
    output wire                 frame_published_o,
    input  wire                 wframe_data_en  ,
    input  wire     [C_W_WIDTH-1:0]      wframe_data     ,
    
    input  wire                 rframe_pclk     ,
    input  wire                 rframe_vsync    ,		//	Reader VSync. Flush on falling edge. Connect to ~EOF. 
    input  wire                 rframe_data_en  ,
    output wire     [C_R_WIDTH-1:0]      rframe_data     ,
    
    output 		[31:0] 	tp_o
);
	
	initial begin
		axi_awvalid <= 0; 
		axi_wvalid <= 0; 
		axi_arvalid <= 0; 
	end

	assign axi_awid    = {C_ID_LEN{1'b0}};
	assign axi_awlen   = C_BURST_LEN - 1'b1;
	assign axi_awsize  = C_DATA_SIZE;
	assign axi_awburst = 2'b01;	//	INCR
	assign axi_awlock  = 1'b0;
	assign axi_awcache = 0;
	assign axi_awprot  = 3'b0;
	assign axi_awqos   = 4'b0;
	assign axi_wstrb   = 32'hFFFFFFFF;
	assign axi_bready  = 1; 
	    
	assign axi_arid    = {C_ID_LEN{1'b0}};
	assign axi_arlen   = C_BURST_LEN - 1'b1;
	assign axi_arsize  = C_DATA_SIZE;
	assign axi_arburst = 2'b01;	//	INCR
	assign axi_arlock  = 1'b0;
	assign axi_arcache = 0;
	assign axi_arprot  = 3'b0;
	assign axi_arqos   = 4'b0;
	assign axi_rready  = 1; 
	
	
	
	
	////////////////////////////////////////////////////////////////
	//	R/W Schedule
    // Format belongs to a complete published slot, never to a live button.
    (* async_reg="true" *) reg [2:0] write_gray_sync,desired_gray_sync;
    reg [(1<<C_SLOT_BITS)-1:0] slot_gray;
    reg reader_gray;
    wire [31:0] write_end_addr=(C_GRAY_END_ADDR!=0 && write_gray_sync[2]) ? C_GRAY_END_ADDR : C_RD_END_ADDR;
    wire [31:0] read_end_addr=reader_gray ? C_GRAY_END_ADDR : C_RD_END_ADDR;
	wire [C_SLOT_BITS-1:0] rc_wframe_index;
	wire [C_SLOT_BITS-1:0] rc_rframe_index;
    wire [C_SLOT_BITS-1:0] history_read_b;
    wire history_compare,history_valid;
    wire [31:0] compare_split = reader_gray ? C_GRAY_COMPARE_SPLIT : C_COMPARE_SPLIT;

    always @(posedge axi_clk or posedge axi_reset) begin
        if(axi_reset) begin write_gray_sync<=0; desired_gray_sync<=0; slot_gray<=0; end
        else begin
            write_gray_sync<={write_gray_sync[1:0],C_GRAY_END_ADDR!=0 && wframe_gray_i};
            desired_gray_sync<={desired_gray_sync[1:0],C_GRAY_END_ADDR!=0 && rframe_gray_i};
            if(frame_published_o) slot_gray[rc_wframe_index]<=write_gray_sync[2];
        end
    end
	reg 			r_wframe_inc = 0, r_rframe_inc = 0; 
    frame_history_ring #(.C_SLOT_BITS(C_SLOT_BITS)) history_slots (
        .clk_i(axi_clk),.rst_n_i(!axi_reset),.publish_i(r_wframe_inc),
        .kind_i(write_gray_sync[2]),.boundary_i(r_rframe_inc),
        .control_i(C_HISTORY_ENABLE ? history_control_i : 2'd0),
        .age_a_i(C_HISTORY_ENABLE ? history_age_a_i : 8'd0),
        .age_b_i(C_HISTORY_ENABLE ? history_age_b_i : 8'd0),
        .write_slot_o(rc_wframe_index),.read_slot_a_o(rc_rframe_index),
        .read_slot_b_o(history_read_b),.compare_o(history_compare),
        .valid_o(history_valid),.count_o(history_count_o)
    );
	
	
	
	////////////////////////////////////////////////////////////////
	//	AXI Writter

	wire                w_wfifo_pempty, w_wfifo_empty;
    wire writer_full,writer_busy;
    reg writer_error;
 
	wire 				w_wfifo_ren; 
	wire 	[C_DATA_LEN-1:0] 	w_wfifo_rdata; 

	//	On EOF when ~empty, flush the last data then reset FIFO. 
	reg 	[3:0] 	rs_w = 0; 
	wire 	[3:0] 	ws_w_idle = 0; 
	wire 	[3:0] 	ws_w_wdata = 1; 
	wire 	[3:0] 	ws_w_winc = 2; 
	wire 	[3:0] 	ws_w_eof = 3;
    wire [3:0] ws_w_response = 4;
    wire [3:0] ws_w_error = 5; 

	//	Write Pointer. Burst bytes is always C_ADDR_INC. 
	reg 	[C_BUF_SIZE-1:0] 	rc_w_ptr = 0; 
	reg 	[0:0] 		rc_w_eof = 0; 
	reg 				r_w_rst = 1; 
	
	//	Burst Control
	reg 	[7:0] 	rc_burst = 0; 	//	Compare to C_BURST_LEN - 1. 
	
	//	EOF Monitor. 
    // Register in the source domain before the asynchronous crossing. A
    // combinational FSM frame decode must never drive an EOF edge detector.
    reg wframe_active_reg;
    always @(posedge wframe_pclk or posedge axi_reset) begin
        if(axi_reset) wframe_active_reg <= 1'b0;
        else wframe_active_reg <= wframe_vsync;
    end
    (* async_reg="true" *) reg [2:0] r_wframe_sync = 0;
    reg r_wframe_previous;
	reg 			r_weof_pending = 0; 

	always @(posedge axi_clk or posedge axi_reset) begin
		if(axi_reset) begin
			rs_w <= 0; 
			rc_w_ptr <= 0; 
			rc_w_eof <= 0; 
			r_w_rst <= 1; 
			axi_awvalid <= 0; 
			axi_wvalid <= 0; 
			r_wframe_sync <= 0; 
            r_wframe_previous <= 0;
			r_weof_pending <= 0; 
			r_wframe_inc <= 0; 
			
		end else begin
			rc_w_eof <= 0; 
			r_wframe_inc <= 0; 
			
			//	Raise EOF on falling edge of VSYNC. 
            r_wframe_sync <= {r_wframe_sync[1:0],wframe_active_reg};
            r_wframe_previous <= r_wframe_sync[2];
			if(r_wframe_previous && !r_wframe_sync[2]) begin
				r_weof_pending <= 1; 
			end else begin
			end
			
			if(axi_awready) begin
				axi_awvalid <= 0; 
			end else begin
			end
			
			case (rs_w)
				ws_w_idle: begin
						rc_burst <= 0; 
						r_w_rst <= 0; 
						
						if(~w_wfifo_pempty) begin
							axi_awvalid <= 1; 
							axi_wvalid <= 1; 
							rs_w <= ws_w_wdata; 
						
						end else if(r_weof_pending && (~w_wfifo_empty)) begin
							//	There's some data in the FIFO. write last. 
							axi_awvalid <= 1; 
							axi_wvalid <= 1; 
							rs_w <= ws_w_wdata; 
							
						end else if(r_weof_pending) begin
							//	EOF. Reset FIFO. Increment write pointer. 
							r_w_rst <= 1; 
							if(rc_w_ptr==write_end_addr) begin
                            r_wframe_inc <= 1;
                            rs_w <= ws_w_eof;
                        end else rs_w <= ws_w_error; 
							
						end else begin
						end
					end
					
				ws_w_eof: begin
						//	Wait for some cycles to release reset. 
						r_weof_pending <= 0; 
						rc_w_ptr <= 0; 
						
						rc_w_eof <= rc_w_eof + 1; 
						if(&rc_w_eof)
							rs_w <= ws_w_idle; 
						else begin
						end
					end
					
				ws_w_wdata: begin
						rc_burst <= rc_burst + axi_wready; 
						
						//	On last transaction terminate write. 
						if(axi_wlast && axi_wready) begin
							axi_wvalid <= 0; 
							rs_w <= ws_w_response; 
						end else begin
						end
					end
					
				ws_w_response: begin
                        if(axi_bvalid) rs_w <= axi_bresp==2'b00 ? ws_w_winc : ws_w_error;
                    end
                    ws_w_error: begin
                        // A failed write must never publish this frame slot.
                        axi_awvalid <= 0;
                        axi_wvalid <= 0;
                    end
                    ws_w_winc: begin
						rc_w_ptr <= rc_w_ptr + C_ADDR_INC; 
						rs_w <= ws_w_idle; 
					end
			endcase
		end
	end
	assign axi_awaddr = C_BASE_ADDR + {rc_wframe_index, rc_w_ptr}; 
	assign axi_wdata = w_wfifo_rdata; 
	assign axi_wlast = (rc_burst >= C_BURST_LEN - 1); 
	
	assign w_wfifo_ren = axi_wvalid && axi_wready; 
	
	
	assign tp_o[1:0] = {rs_w[1:0]}; 
	assign tp_o[2] = axi_wvalid; 
	assign tp_o[3] = wframe_data_en; 
	
	
	generate
		if(C_W_WIDTH == 64) begin: Gen_FIFO_W64
			W0_FIFO_64 u_W0_FIFO_64 
			(
			    .wr_clk_i       (wframe_pclk    ),
			    .wr_en_i        (wframe_data_en && wframe_ready_o),
			    .wdata          (wframe_data    ),
			    
			    .a_rst_i 	(r_w_rst), 

			    .rd_clk_i       (axi_clk        ),
			    .rd_en_i        (w_wfifo_ren     ),
			    .rdata          (w_wfifo_rdata    ),
			    .prog_empty_o	(w_wfifo_pempty),
			    .empty_o 		(w_wfifo_empty),
                .full_o(writer_full), .rst_busy(writer_busy)
			);
		end else if(C_W_WIDTH == 32) begin: Gen_FIFO_W32
			W0_FIFO_32 u_W0_FIFO_32
			(
			    .wr_clk_i       (wframe_pclk    ),
			    .wr_en_i        (wframe_data_en && wframe_ready_o),
			    .wdata          (wframe_data    ),
			    
			    .a_rst_i 	(r_w_rst), 

			    .rd_clk_i       (axi_clk        ),
			    .rd_en_i        (w_wfifo_ren     ),
			    .rdata          (w_wfifo_rdata    ),
			    .prog_empty_o	(w_wfifo_pempty),
			    .empty_o 		(w_wfifo_empty)
			);
		end else if(C_W_WIDTH == 8) begin: Gen_FIFO_W8
			W0_FIFO_8 u_W0_FIFO_8 
			(
			    .wr_clk_i       (wframe_pclk    ),
			    .wr_en_i        (wframe_data_en && wframe_ready_o),
			    .wdata          (wframe_data    ),
			    
			    .a_rst_i 	(r_w_rst), 

			    .rd_clk_i       (axi_clk        ),
			    .rd_en_i        (w_wfifo_ren     ),
			    .rdata          (w_wfifo_rdata    ),
			    .prog_empty_o	(w_wfifo_pempty),
			    .empty_o 		(w_wfifo_empty)
			);
		end
	endgenerate




















// Pixel-domain frame boundary is transferred as a toggle. The AXI reader
// never cancels an asserted ARVALID or forgets an accepted read at a boundary.
reg rframe_vsync_dly;
reg reader_epoch;
wire rframe_vsync_neg = rframe_vsync_dly && !rframe_vsync;
always @(posedge rframe_pclk or posedge axi_reset) begin
    if(axi_reset) begin rframe_vsync_dly<=0; reader_epoch<=0; end
    else begin
        rframe_vsync_dly<=rframe_vsync;
        if(rframe_vsync_neg) reader_epoch<=!reader_epoch;
    end
end
(* async_reg="true" *) reg [2:0] reader_epoch_sync;
reg reader_epoch_seen;
wire reader_restart = reader_epoch_sync[2]!=reader_epoch_seen;
reg restart_pending;
reg read_fifo_reset;
wire w_rfifo_rst = axi_reset || read_fifo_reset;
wire w_rfifo_empty,rfifo_wfull,rfifo_busy;
wire [C_R_WIDTH-1:0] rframe_fifo_data;
reg rfifo_wenb;
reg [C_DATA_LEN-1:0] rfifo_wdata;
localparam READ_BANK_BITS = C_READ_BANKS<=1 ? 1 : $clog2(C_READ_BANKS);
localparam READ_BURST_PIXELS = C_BURST_LEN*C_DATA_LEN/C_R_WIDTH;
localparam READ_PIXEL_BITS = READ_BURST_PIXELS<=1 ? 1 : $clog2(READ_BURST_PIXELS);
reg [READ_BANK_BITS-1:0] read_fill_bank,rfifo_write_bank,read_pixel_bank;
reg [READ_PIXEL_BITS-1:0] read_burst_pixel;
wire [C_READ_BANKS-1:0] read_bank_empty,read_bank_full,read_bank_busy;
wire [C_READ_BANKS*C_R_WIDTH-1:0] read_bank_data;
assign w_rfifo_empty=read_bank_empty[read_pixel_bank];
assign rfifo_wfull=read_bank_full[read_fill_bank];
assign rfifo_busy=|read_bank_busy;
assign rframe_fifo_data=read_bank_data[read_pixel_bank*C_R_WIDTH+:C_R_WIDTH];
reg completed_frame_seen,reader_has_frame;
reg reader_axi_error;
reg reader_starved;
(* async_reg="true" *) reg [2:0] reader_starved_sync;
assign error_o = writer_error || reader_axi_error || reader_starved_sync[2];
assign detail_fault_o={reader_starved_sync[2],reader_axi_error,writer_error};
assign frame_published_o=r_wframe_inc;
(* async_reg="true" *) reg [2:0] reader_frame_sync;
reg pixel_frame_bad;
reg reader_data_seen;

// Once a pixel deadline is missed, continuing the same FIFO would move every
// later pixel to another coordinate. Blank the remaining frame and re-align
// at the next vertical blank. Read priority prevents routine starvation.
always @(posedge rframe_pclk or posedge axi_reset) begin
    if(axi_reset) begin
        reader_frame_sync<=0; pixel_frame_bad<=0;
        reader_data_seen<=0; reader_starved<=0;
    end else begin
        reader_frame_sync<={reader_frame_sync[1:0],reader_has_frame};
        if(rframe_vsync_neg) pixel_frame_bad<=0;
        else if(rframe_data_en && reader_frame_sync[2] && w_rfifo_empty) begin
            pixel_frame_bad<=1;
            reader_starved<=1;
        end
        if(rframe_data_en && reader_frame_sync[2] && !w_rfifo_empty && !pixel_frame_bad)
            reader_data_seen<=1;
    end
end
wire reader_take = rframe_data_en && reader_frame_sync[2] && !pixel_frame_bad && !w_rfifo_empty;
// FIFO payload order is unchanged inside a burst. Switch the read bank only
// after consuming every pixel of that burst, including its last pixel.
always @(posedge rframe_pclk or posedge w_rfifo_rst) begin
    if(w_rfifo_rst) begin read_pixel_bank<=0; read_burst_pixel<=0; end
    else if(reader_take) begin
        if(read_burst_pixel==READ_BURST_PIXELS-1) begin
            read_burst_pixel<=0;
            read_pixel_bank<=read_pixel_bank==C_READ_BANKS-1 ? 0 : read_pixel_bank+1'b1;
        end else read_burst_pixel<=read_burst_pixel+1'b1;
    end
end
assign rframe_data = reader_frame_sync[2] && !pixel_frame_bad && !w_rfifo_empty
    ? rframe_fifo_data : {C_R_WIDTH{1'b0}};
assign debug_status_o = {error_o,reader_data_seen,reader_has_frame,completed_frame_seen};

genvar read_bank;
generate for(read_bank=0;read_bank<C_READ_BANKS;read_bank=read_bank+1) begin: read_fifo_bank
    if(C_R_WIDTH==8) begin: Gen_RFIFO_8
        R0_FIFO_8 u_R0_FIFO_8 (
            .wr_clk_i(axi_clk),.wr_en_i(rfifo_wenb && rfifo_write_bank==read_bank),.wdata(rfifo_wdata),
            .a_rst_i(w_rfifo_rst),.prog_full_o(read_bank_full[read_bank]),.rst_busy(read_bank_busy[read_bank]),
            .rd_clk_i(rframe_pclk),.rd_en_i(reader_take && read_pixel_bank==read_bank),
            .rdata(read_bank_data[read_bank*C_R_WIDTH+:C_R_WIDTH]),.empty_o(read_bank_empty[read_bank])
        );
    end else if(C_R_WIDTH==16) begin: Gen_RFIFO_16
        R0_FIFO_16 u_R0_FIFO_16 (
            .wr_clk_i(axi_clk),.wr_en_i(rfifo_wenb && rfifo_write_bank==read_bank),.wdata(rfifo_wdata),
            .a_rst_i(w_rfifo_rst),.prog_full_o(read_bank_full[read_bank]),.rst_busy(read_bank_busy[read_bank]),
            .rd_clk_i(rframe_pclk),.rd_en_i(reader_take && read_pixel_bank==read_bank),
            .rdata(read_bank_data[read_bank*C_R_WIDTH+:C_R_WIDTH]),.empty_o(read_bank_empty[read_bank])
        );
    end
end endgenerate

localparam S_READ_IDLE=3'd0, S_READ_ADDR=3'd1, S_READ_DATA=3'd2,
    S_READ_RESET=3'd3, S_READ_SETTLE=3'd4;
reg [2:0] rd_state;
reg [4:0] reset_count;
reg [C_BUF_SIZE-1:0] araddr;
reg [31:0] read_address;
assign axi_araddr=read_address;

always @(posedge axi_clk or posedge axi_reset) begin
    if(axi_reset) begin
        reader_epoch_sync<=0; reader_epoch_seen<=0; restart_pending<=1;
        reader_starved_sync<=0; completed_frame_seen<=0; reader_has_frame<=0;
        read_fifo_reset<=1; rd_state<=S_READ_IDLE; reset_count<=0;
        araddr<=0; read_address<=C_BASE_ADDR; axi_arvalid<=0;
        r_rframe_inc<=0; rfifo_wenb<=0; rfifo_wdata<=0; reader_axi_error<=0;
        read_fill_bank<=0; rfifo_write_bank<=0; reader_gray<=0;
    end else begin
        reader_epoch_sync<={reader_epoch_sync[1:0],reader_epoch};
        reader_starved_sync<={reader_starved_sync[1:0],reader_starved};
        r_rframe_inc<=0;
        rfifo_wenb<=0;
        if(r_wframe_inc) completed_frame_seen<=1;
        if(reader_restart) begin
            reader_epoch_seen<=reader_epoch_sync[2];
            restart_pending<=1;
            reader_has_frame<=0;
        end
        // RREADY remains asserted to finish an in-flight old-frame burst.
        // Discard that payload when a new epoch is pending; reset only after
        // RLAST, including the registered FIFO write pipeline.
        if(axi_rvalid && rd_state==S_READ_DATA) begin
            if(axi_rresp!=2'b00) reader_axi_error<=1;
            if(!restart_pending && !reader_restart && !read_fifo_reset) begin
                rfifo_wenb<=1;
                rfifo_wdata<=axi_rdata;
                rfifo_write_bank<=read_fill_bank;
            end
        end
        case(rd_state)
            S_READ_IDLE: begin
                if(restart_pending || reader_restart) begin
                    restart_pending<=0;
                    read_fifo_reset<=1;
                    reader_has_frame<=0;
                    reset_count<=0;
                    araddr<=0;
                    read_fill_bank<=0;
                    r_rframe_inc<=1;
                    rd_state<=S_READ_RESET;
                end else if(reader_has_frame && araddr<read_end_addr && !rfifo_wfull) begin
                    read_address<=C_BASE_ADDR+{(history_compare && araddr>=compare_split ? history_read_b : rc_rframe_index),araddr};
                    axi_arvalid<=1;
                    rd_state<=S_READ_ADDR;
                end
            end
            S_READ_ADDR: begin
                // Even if a frame boundary arrives now, hold VALID/address
                // until accepted, then drain this transaction before restart.
                if(axi_arready) begin
                    axi_arvalid<=0;
                    araddr<=araddr+C_ADDR_INC;
                    rd_state<=S_READ_DATA;
                end
            end
            S_READ_DATA: if(axi_rvalid && axi_rlast) begin
                rd_state<=S_READ_IDLE;
                read_fill_bank<=read_fill_bank==C_READ_BANKS-1 ? 0 : read_fill_bank+1'b1;
            end
            S_READ_RESET: begin
                if(reset_count==7) begin
                    read_fifo_reset<=0;
                    reset_count<=0;
                    rd_state<=S_READ_SETTLE;
                end else reset_count<=reset_count+1'b1;
            end
            S_READ_SETTLE: begin
                if(reset_count!=31) reset_count<=reset_count+1'b1;
                if(reset_count==31 && !rfifo_busy) begin
                    reader_gray<=slot_gray[rc_rframe_index];
                    reader_has_frame<=history_valid && !restart_pending && !reader_restart && slot_gray[rc_rframe_index]==desired_gray_sync[2];
                    rd_state<=S_READ_IDLE;
                end
            end
            default: begin restart_pending<=1; rd_state<=S_READ_IDLE; end
        endcase
    end
end

assign wframe_ready_o = !writer_full && !writer_busy && !r_w_rst;
    always @(posedge axi_clk or posedge axi_reset) begin
        if(axi_reset) writer_error <= 1'b0;
        else if(axi_bvalid && axi_bready && axi_bresp!=2'b00) writer_error <= 1'b1;
        else if(rs_w==ws_w_error) writer_error <= 1'b1;
    end
endmodule
