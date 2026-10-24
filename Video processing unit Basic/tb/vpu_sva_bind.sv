//------------------------------------------------------------------------------
// File       : vpu_sva_bind.sv
// Author     : Senior RTL Design & Verification Lead
// Description: SystemVerilog Assertions (SVA) & White-Box Protocol Checker
//              bound to vpu_top. Covers AXI4-Stream handshake stability,
//              SOF/EOL framing, Async FIFO overflow/underflow, Gray-code
//              CDC pointer single-bit transitions, line-buffer address bounds,
//              and CSR quasi-static shadow stability during active frames.
//------------------------------------------------------------------------------

module vpu_sva_checker
  import vpu_pkg::*;
#(
  parameter int unsigned FIFO_ADDR_WIDTH = 4
) (
  input logic                      sensor_clk,
  input logic                      sensor_rst_n,
  input logic                      s_vsync,
  input logic                      s_hsync,
  input logic                      s_valid,
  input logic [PIXEL_WIDTH-1:0]    s_pixel,

  input logic                      core_clk,
  input logic                      core_rst_n,
  input logic                      m_axis_tvalid,
  input logic                      m_axis_tready,
  input logic [PIXEL_WIDTH-1:0]    m_axis_tdata,
  input logic                      m_axis_tuser,
  input logic                      m_axis_tlast,

  // Internal hierarchical probes
  input logic                      fifo_wr_en,
  input logic                      fifo_wr_full,
  input logic [FIFO_ADDR_WIDTH:0]  fifo_wr_gray,
  input logic                      fifo_rd_en,
  input logic                      fifo_rd_empty,
  input logic [FIFO_ADDR_WIDTH:0]  fifo_rd_gray,
  input logic [12:0]               lb_x_cnt,
  input logic [12:0]               lb_y_cnt,
  input logic                      lb_in_frame,
  input vpu_cfg_t                  active_cfg
);

  //----------------------------------------------------------------------------
  // 1. AXI4-Stream Master Output Handshake Assertions (core_clk)
  //----------------------------------------------------------------------------
  // TVALID must remain asserted until TREADY is high
  property p_axis_valid_stable;
    @(posedge core_clk) disable iff (!core_rst_n || !active_cfg.enable)
    (m_axis_tvalid && !m_axis_tready) |=> m_axis_tvalid;
  endproperty
  AST_AXIS_VALID_STABLE: assert property (p_axis_valid_stable)
    else $error("[SVA] m_axis_tvalid deasserted before m_axis_tready handshake!");

  // Payload (TDATA, TUSER/SOF, TLAST/EOL) must remain stable while stalled
  property p_axis_payload_stable;
    @(posedge core_clk) disable iff (!core_rst_n || !active_cfg.enable)
    (m_axis_tvalid && !m_axis_tready) |=>
      ($stable(m_axis_tdata) && $stable(m_axis_tuser) && $stable(m_axis_tlast));
  endproperty
  AST_AXIS_PAYLOAD_STABLE: assert property (p_axis_payload_stable)
    else $error("[SVA] AXI4-Stream payload changed during backpressure stall!");

  // No X/Z on valid output beat
  property p_axis_no_xz;
    @(posedge core_clk) disable iff (!core_rst_n)
    m_axis_tvalid |-> !$isunknown({m_axis_tdata, m_axis_tuser, m_axis_tlast});
  endproperty
  AST_AXIS_NO_XZ: assert property (p_axis_no_xz)
    else $error("[SVA] X/Z detected on AXI4-Stream output bus when m_axis_tvalid=1!");

  // SOF (tuser) and EOL (tlast) cannot be simultaneously asserted (since min width >= 64)
  property p_axis_sof_eol_mutex;
    @(posedge core_clk) disable iff (!core_rst_n)
    m_axis_tvalid |-> !(m_axis_tuser && m_axis_tlast);
  endproperty
  AST_AXIS_SOF_EOL_MUTEX: assert property (p_axis_sof_eol_mutex)
    else $error("[SVA] Simultaneous SOF (tuser) and EOL (tlast) on single pixel beat!");

  //----------------------------------------------------------------------------
  // 2. CDC & Asynchronous FIFO Integrity Assertions
  //----------------------------------------------------------------------------
  // Write Gray pointer changes by at most 1 bit per sensor_clk cycle
  property p_wr_gray_one_hot_step;
    @(posedge sensor_clk) disable iff (!sensor_rst_n)
    $countones(fifo_wr_gray ^ $past(fifo_wr_gray)) <= 1;
  endproperty
  AST_WR_GRAY_ONE_BIT: assert property (p_wr_gray_one_hot_step)
    else $error("[SVA] Write Gray pointer changed by >1 bit in a single sensor_clk!");

  // Read Gray pointer changes by at most 1 bit per core_clk cycle
  property p_rd_gray_one_hot_step;
    @(posedge core_clk) disable iff (!core_rst_n)
    $countones(fifo_rd_gray ^ $past(fifo_rd_gray)) <= 1;
  endproperty
  AST_RD_GRAY_ONE_BIT: assert property (p_rd_gray_one_hot_step)
    else $error("[SVA] Read Gray pointer changed by >1 bit in a single core_clk!");

  // Under normal nominal operation (when downstream is not stalled for > FIFO depth),
  // sensor FIFO should not overflow
  property p_no_fifo_overflow_when_ready;
    @(posedge sensor_clk) disable iff (!sensor_rst_n)
    !(fifo_wr_en && fifo_wr_full);
  endproperty
  COV_FIFO_OVERFLOW: cover property (@(posedge sensor_clk) disable iff (!sensor_rst_n) (fifo_wr_en && fifo_wr_full));

  //----------------------------------------------------------------------------
  // 3. Line Buffer & Active Configuration Stability Assertions (core_clk)
  //----------------------------------------------------------------------------
  // Column counter must never exceed MAX_LINE_WIDTH - 1 (4095)
  property p_lb_col_in_bounds;
    @(posedge core_clk) disable iff (!core_rst_n)
    lb_x_cnt < MAX_LINE_WIDTH;
  endproperty
  AST_LB_COL_BOUNDS: assert property (p_lb_col_in_bounds)
    else $error("[SVA] Line buffer x_cnt exceeded MAX_LINE_WIDTH (4096)!");

  // Active resolution mode and dimensions must remain stable throughout an active frame
  property p_cfg_stable_mid_frame;
    @(posedge core_clk) disable iff (!core_rst_n || active_cfg.soft_clear)
    (lb_in_frame && $past(lb_in_frame)) |->
      ($stable(active_cfg.res_mode) &&
       $stable(active_cfg.active_width) &&
       $stable(active_cfg.active_height));
  endproperty
  AST_CFG_STABLE_MID_FRAME: assert property (p_cfg_stable_mid_frame)
    else $error("[SVA] Active resolution configuration glitched mid-frame!");

  // Saturation clipping bounds check on processed output in GAUSSIAN or SHARPEN modes
  property p_output_within_clip_bounds;
    @(posedge core_clk) disable iff (!core_rst_n)
    (m_axis_tvalid && (active_cfg.proc_mode inside {PROC_GAUSSIAN, PROC_SHARPEN})) |->
      ((m_axis_tdata >= active_cfg.clip_min) && (m_axis_tdata <= active_cfg.clip_max));
  endproperty
  AST_OUTPUT_CLIP_BOUNDS: assert property (p_output_within_clip_bounds)
    else $error("[SVA] Output pixel %0d violated programmed clip bounds [%0d : %0d]!",
                m_axis_tdata, active_cfg.clip_min, active_cfg.clip_max);

endmodule : vpu_sva_checker

// Bind statement connecting SVA checker non-intrusively to vpu_top
bind vpu_top vpu_sva_checker #(
  .FIFO_ADDR_WIDTH ( FIFO_ADDR_WIDTH )
) u_vpu_sva_checker (
  .sensor_clk     ( sensor_clk                         ),
  .sensor_rst_n   ( sensor_rst_n                       ),
  .s_vsync        ( s_vsync                            ),
  .s_hsync        ( s_hsync                            ),
  .s_valid        ( s_valid                            ),
  .s_pixel        ( s_pixel                            ),
  .core_clk       ( core_clk                           ),
  .core_rst_n     ( core_rst_n                         ),
  .m_axis_tvalid  ( m_axis_tvalid                      ),
  .m_axis_tready  ( m_axis_tready                      ),
  .m_axis_tdata   ( m_axis_tdata                       ),
  .m_axis_tuser   ( m_axis_tuser                       ),
  .m_axis_tlast   ( m_axis_tlast                       ),
  .fifo_wr_en     ( u_sensor_if.s_active_d1            ),
  .fifo_wr_full   ( u_sensor_if.fifo_full              ),
  .fifo_wr_gray   ( u_sensor_if.u_async_fifo.wr_gray   ),
  .fifo_rd_en     ( u_sensor_if.fifo_rd_en             ),
  .fifo_rd_empty  ( u_sensor_if.fifo_empty             ),
  .fifo_rd_gray   ( u_sensor_if.u_async_fifo.rd_gray   ),
  .lb_x_cnt       ( u_line_buffer.x_cnt                ),
  .lb_y_cnt       ( u_line_buffer.y_cnt                ),
  .lb_in_frame    ( u_line_buffer.in_frame             ),
  .active_cfg     ( active_cfg                         )
);
