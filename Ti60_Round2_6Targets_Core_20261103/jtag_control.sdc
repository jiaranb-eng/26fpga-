# Host uses 1 MHz; analyze User TAP at 10 MHz. No video clocks are relaxed.
create_clock -period 100.000 -name jtag_inst1_TCK [get_ports {jtag_inst1_TCK}]
# Exact Ti60F225 C4 User TAP delays from this candidate's Interface Designer.
set_output_delay -clock jtag_inst1_TCK -max 1.321 [get_ports {jtag_inst1_TDO}]
set_output_delay -clock jtag_inst1_TCK -min 0.495 [get_ports {jtag_inst1_TDO}]
set_input_delay -clock_fall -clock jtag_inst1_TCK -max 0.125 [get_ports {jtag_inst1_CAPTURE}]
set_input_delay -clock_fall -clock jtag_inst1_TCK -min 0.077 [get_ports {jtag_inst1_CAPTURE}]
set_input_delay -clock_fall -clock jtag_inst1_TCK -max 0.660 [get_ports {jtag_inst1_RESET}]
set_input_delay -clock_fall -clock jtag_inst1_TCK -min 0.401 [get_ports {jtag_inst1_RESET}]
set_input_delay -clock_fall -clock jtag_inst1_TCK -max 0.204 [get_ports {jtag_inst1_SEL}]
set_input_delay -clock_fall -clock jtag_inst1_TCK -min 0.025 [get_ports {jtag_inst1_SEL}]
set_input_delay -clock_fall -clock jtag_inst1_TCK -max 0.153 [get_ports {jtag_inst1_UPDATE}]
set_input_delay -clock_fall -clock jtag_inst1_TCK -min 0.074 [get_ports {jtag_inst1_UPDATE}]
set_input_delay -clock_fall -clock jtag_inst1_TCK -max 0.172 [get_ports {jtag_inst1_SHIFT}]
set_input_delay -clock_fall -clock jtag_inst1_TCK -min 0.055 [get_ports {jtag_inst1_SHIFT}]
# TDI is driven on the falling edge by MPSSE. Budget 10 ns of external skew
# at the conservative 10 MHz constraint (actual host 1 MHz). Not a measured
# board delay; physical acceptance must still verify the chosen programmer.
set_input_delay -clock_fall -clock jtag_inst1_TCK -max 10.000 [get_ports {jtag_inst1_TDI}]
set_input_delay -clock_fall -clock jtag_inst1_TCK -min 0.000 [get_ports {jtag_inst1_TDI}]
# First-stage toggle synchronizers alone are asynchronous sampling endpoints.
set_false_path -to [get_pins {u_jtag_video_control/u_jtag_packet_mailbox/reg_request_sync[0]~FF|D}]
set_false_path -to [get_pins {u_jtag_video_control/u_jtag_packet_mailbox/reg_done_sync[0]~FF|D}]
# Held bundled buses have two destination cycles to settle; bound the route
# instead of broadly false-pathing the mailbox data. Common async reset is
# released in clk_sys and locally synchronized for command acceptance in TCK.
set_max_delay -from [get_clocks {jtag_inst1_TCK}] -to [get_clocks {clk_sys}] 8.0
set_min_delay -from [get_clocks {jtag_inst1_TCK}] -to [get_clocks {clk_sys}] 0.0
set_max_delay -from [get_clocks {clk_sys}] -to [get_clocks {jtag_inst1_TCK}] 20.0
set_min_delay -from [get_clocks {clk_sys}] -to [get_clocks {jtag_inst1_TCK}] 0.0
