`timescale 1ns/1ps

// Re-run the unchanged SC233 register table if no complete input frame arrives.
// This controls only sensor configuration and receiver reset. DDR transactions,
// Canny jobs and HDMI remain under their existing reset/ownership protocols.
module sc233_link_recovery #(
    parameter integer SYS_CLK_HZ = 96000000,
    parameter integer RESET_CYCLES = SYS_CLK_HZ / 100,
    parameter integer CONFIG_TIMEOUT_CYCLES = SYS_CLK_HZ * 2,
    parameter integer FRAME_TIMEOUT_CYCLES = SYS_CLK_HZ / 2,
    parameter integer FRAME_GROUPS = 576000,
    parameter integer STARTUP_QUIET_CYCLES = FRAME_TIMEOUT_CYCLES
) (
    input wire system_clk_i,
    input wire system_rst_n_i,
    input wire config_done_i,
    input wire camera_clk_i,
    input wire camera_rst_n_i,
    input wire frame_valid_i,
    input wire line_valid_i,
    input wire group_valid_i,
    output reg config_rst_n_o,
    output wire receiver_reset_o
);
    // HSIO lane reset is asynchronous, but release belongs to its SLOWCLK.
    // The clock lane is never reset here, so its recovered byte clock can
    // release the data-lane deserializers and the unchanged receiver decoder.
    reg receiver_reset_request;
    (* async_reg = "true" *) reg [2:0] receiver_reset_sync = 3'b111;
    always @(posedge camera_clk_i or posedge receiver_reset_request) begin
        if (receiver_reset_request) receiver_reset_sync <= 3'b111;
        else receiver_reset_sync <= {receiver_reset_sync[1:0], 1'b0};
    end
    assign receiver_reset_o = receiver_reset_sync[2];

    localparam integer HOLD_BITS = $clog2(RESET_CYCLES + 1);
    localparam integer TIMEOUT_BITS = $clog2(CONFIG_TIMEOUT_CYCLES + 1);
    localparam integer GROUP_BITS = $clog2(FRAME_GROUPS + 2);
    reg frame_delay;
    reg frame_started;
    reg [GROUP_BITS-1:0] group_count;
    reg good_frame_toggle;
    reg degraded_toggle;
    reg [3:0] bad_score;
    wire frame_start = frame_valid_i && !frame_delay;
    wire frame_end = !frame_valid_i && frame_delay;
    wire qualified_group = frame_valid_i && line_valid_i && group_valid_i;

    always @(posedge camera_clk_i or negedge camera_rst_n_i) begin
        if (!camera_rst_n_i) begin
            frame_delay <= 1'b0;
            frame_started <= 1'b0;
            group_count <= 0;
            good_frame_toggle <= 1'b0;
            degraded_toggle <= 1'b0;
            bad_score <= 0;
        end else begin
            frame_delay <= frame_valid_i;
            if (frame_start) begin
                frame_started <= 1'b1;
                group_count <= qualified_group ? 1 : 0;
            end else if (qualified_group && group_count <= FRAME_GROUPS) begin
                // Saturate above the legal count: oversized frames cannot wrap
                // around and masquerade as a complete frame.
                group_count <= group_count + 1'b1;
            end
            if (frame_end) begin
                if (frame_started && group_count == FRAME_GROUPS) begin
                    good_frame_toggle <= !good_frame_toggle;
                    if(bad_score!=0) bad_score<=bad_score-1'b1;
                end else if(frame_started) begin
                    // A rare good frame must not keep an otherwise corrupt link alive.
                    // Four consecutive bad frames or sustained mixed corruption trigger repair.
                    if(bad_score>=6) begin degraded_toggle<=!degraded_toggle; bad_score<=0; end
                    else bad_score<=bad_score+2;
                end
                frame_started <= 1'b0;
                group_count <= 0;
            end
            if(receiver_reset_o) begin
                frame_started<=0;
                group_count<=0;
                bad_score<=0;
            end
        end
    end

    (* async_reg = "true" *) reg [2:0] good_frame_sync;
    reg good_frame_seen;
    reg config_done_delay;
    reg [HOLD_BITS-1:0] reset_count;
    reg [TIMEOUT_BITS-1:0] timeout_count;
    (* async_reg = "true" *) reg [2:0] degraded_sync;
    reg degraded_seen;
    localparam integer QUIET_BITS = $clog2(STARTUP_QUIET_CYCLES + 1);
    reg [QUIET_BITS-1:0] startup_quiet_count;
    always @(posedge system_clk_i or negedge system_rst_n_i) begin
        if (!system_rst_n_i) begin
            good_frame_sync <= 0;
            good_frame_seen <= 1'b0;
            config_done_delay <= 1'b0;
            reset_count <= 0;
            timeout_count <= 0;
            config_rst_n_o <= 1'b0;
            receiver_reset_request <= 1'b1;
            degraded_sync<=0; degraded_seen<=0;
            startup_quiet_count<=STARTUP_QUIET_CYCLES;
        end else begin
            good_frame_sync <= {good_frame_sync[1:0], good_frame_toggle};
            good_frame_seen <= good_frame_sync[2];
            config_done_delay <= config_done_i;
            degraded_sync<={degraded_sync[1:0],degraded_toggle};
            degraded_seen<=degraded_sync[2];
            // Restart the transmitter and receiver as one link transaction.
            // A receiver-only pulse discards alignment while the sensor keeps
            // streaming; it must not be used as a frame-quality repair.
            receiver_reset_request <= !config_rst_n_o || !config_done_i;
            if(!config_rst_n_o || !config_done_i)
                startup_quiet_count<=STARTUP_QUIET_CYCLES;
            else if(startup_quiet_count!=0)
                startup_quiet_count<=startup_quiet_count-1'b1;
            if (!config_rst_n_o) begin
                timeout_count <= 0;
                receiver_reset_request <= 1'b1;
                if (reset_count == RESET_CYCLES - 1) begin
                    reset_count <= 0;
                    config_rst_n_o <= 1'b1;
                end else reset_count <= reset_count + 1'b1;
            end else if(config_done_i && degraded_sync[2]!=degraded_seen && startup_quiet_count==0) begin
                timeout_count<=0;
                receiver_reset_request<=1;
                config_rst_n_o<=0;
                reset_count<=0;
            end else if (config_done_i && (!config_done_delay || good_frame_sync[2] != good_frame_seen)) begin
                timeout_count <= 0;
            end else if ((!config_done_i && timeout_count == CONFIG_TIMEOUT_CYCLES - 1) ||
                         (config_done_i && timeout_count >= FRAME_TIMEOUT_CYCLES - 1)) begin
                config_rst_n_o <= 1'b0;
                receiver_reset_request <= 1'b1;
                reset_count <= 0;
                timeout_count <= 0;
            end else begin
                timeout_count <= timeout_count + 1'b1;
            end
        end
    end
endmodule
