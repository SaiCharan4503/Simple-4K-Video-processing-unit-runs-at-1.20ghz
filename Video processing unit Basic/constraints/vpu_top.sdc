#-------------------------------------------------------------------------------
# File       : vpu_top.sdc
# Author     : Senior RTL Design & Verification Lead
# Description: Synopsys Design Constraints (SDC) for 5nm TSMC 4K Video
#              Processing Unit (VPU) targeting 2.0 GHz (500 ps period).
#-------------------------------------------------------------------------------

set_units -time ps -capacitance fF -current mA -voltage V -resistance kOhm

#-------------------------------------------------------------------------------
# 1. Clock Definitions
#-------------------------------------------------------------------------------
# Core Processing Clock: 2.0 GHz -> 500.0 ps period (250 ps rise, 500 ps fall)
create_clock -name core_clk   -period 500.0  -waveform {0.0 250.0}  [get_ports core_clk]

# Sensor Pixel Clock: Worst-case 4K @ 120 fps + blanking = 1.20 GHz (833.3 ps)
create_clock -name sensor_clk -period 833.3  -waveform {0.0 416.65} [get_ports sensor_clk]

# APB Control Clock: 200 MHz -> 5000.0 ps period
create_clock -name pclk       -period 5000.0 -waveform {0.0 2500.0} [get_ports pclk]

#-------------------------------------------------------------------------------
# 2. Clock Uncertainty, Transition, and Latency (5nm TSMC Signoff Margins)
#-------------------------------------------------------------------------------
# Setup uncertainty: 35 ps (PLL jitter + OCV/POCV + clock tree skew at 5nm)
# Hold uncertainty : 18 ps
set_clock_uncertainty -setup 35.0 [all_clocks]
set_clock_uncertainty -hold  18.0 [all_clocks]

# Maximum clock transition (slew) & data transition for 5nm standard cells
set_clock_transition 25.0 [all_clocks]
set_max_transition   60.0 [current_design]
set_max_fanout       16   [current_design]

#-------------------------------------------------------------------------------
# 3. Asynchronous Clock Domain Crossing (CDC) Groups & Datapath-Only Constraints
#-------------------------------------------------------------------------------
set_clock_groups -asynchronous \
  -group [get_clocks core_clk] \
  -group [get_clocks sensor_clk] \
  -group [get_clocks pclk]

# Constrain Gray-coded Async FIFO pointers to <= 1 destination clock period
# (500 ps for sensor->core, 833 ps for core->sensor) to bound skew between bits
set_max_delay 450.0 -datapath_only \
  -from [get_cells u_sensor_if/u_async_fifo/wr_gray_reg*] \
  -to   [get_cells u_sensor_if/u_async_fifo/wr_gray_sync1_reg*]

set_max_delay 750.0 -datapath_only \
  -from [get_cells u_sensor_if/u_async_fifo/rd_gray_reg*] \
  -to   [get_cells u_sensor_if/u_async_fifo/rd_gray_sync1_reg*]

# False path on async reset inputs and DFT scan_mode static control
set_false_path -from [get_ports {sensor_arst_n core_arst_n preset_n scan_mode}]

# Quasi-static CSR shadow registers (updated only when core is idle or on SOF)
set_max_delay 500.0 -datapath_only \
  -from [get_cells u_csr/r_*_reg*] \
  -to   [get_cells u_csr/active_cfg_reg*]

#-------------------------------------------------------------------------------
# 4. I/O Port Delays (30% of Clock Period Budget for Boundary Timing)
#-------------------------------------------------------------------------------
# Sensor Input Interface (sensor_clk domain, 833.3 ps period)
set_input_delay -clock sensor_clk -max 250.0 [get_ports {s_vsync s_hsync s_valid s_pixel[*]}]
set_input_delay -clock sensor_clk -min 50.0  [get_ports {s_vsync s_hsync s_valid s_pixel[*]}]

# AXI4-Stream Video Output Interface (core_clk domain, 500.0 ps period)
set_output_delay -clock core_clk -max 150.0 [get_ports {m_axis_tvalid m_axis_tdata[*] m_axis_tuser m_axis_tlast irq_out}]
set_output_delay -clock core_clk -min 25.0  [get_ports {m_axis_tvalid m_axis_tdata[*] m_axis_tuser m_axis_tlast irq_out}]
set_input_delay  -clock core_clk -max 150.0 [get_ports {m_axis_tready}]
set_input_delay  -clock core_clk -min 25.0  [get_ports {m_axis_tready}]

# APB3 Slave Interface (pclk domain, 5000.0 ps period)
set_input_delay  -clock pclk -max 1500.0 [get_ports {paddr[*] psel penable pwrite pwdata[*]}]
set_input_delay  -clock pclk -min 200.0  [get_ports {paddr[*] psel penable pwrite pwdata[*]}]
set_output_delay -clock pclk -max 1500.0 [get_ports {prdata[*] pready pslverr}]
set_output_delay -clock pclk -min 200.0  [get_ports {prdata[*] pready pslverr}]
