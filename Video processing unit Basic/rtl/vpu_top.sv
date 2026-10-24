//------------------------------------------------------------------------------
// File       : vpu_top.sv
// Author     : Senior RTL Design & Verification Lead
// Description: Top-Level 4K Video Processing Unit (VPU) for 5nm TSMC @ 2 GHz.
//              Integrates:
//                1. Reset Synchronizers (async assert, sync deassert per clock)
//                2. 1:1 Sensor Interface + Async CDC FIFO (vpu_sensor_if)
//                3. APB3 Control & Status Register Block (vpu_csr)
//                4. 4K Circular Line Buffer & 3x3 Window Gen (vpu_line_buffer)
//                5. 5-Stage 2 GHz Processing Datapath (vpu_datapath)
//                6. Skid-Buffered AXI4-Stream Video Output (vpu_out_if)
//------------------------------------------------------------------------------

module vpu_top
  import vpu_pkg::*;
#(
  parameter int unsigned FIFO_ADDR_WIDTH = 4
) (
  //----------------------------------------------------------------------------
  // Clocks and Asynchronous Resets
  //----------------------------------------------------------------------------
  input  logic                      sensor_clk,    // Sensor pixel clock (up to 1.2 GHz)
  input  logic                      sensor_arst_n, // Async active-low sensor reset
  input  logic                      core_clk,      // 2.0 GHz main VPU processing clock
  input  logic                      core_arst_n,   // Async active-low core reset
  input  logic                      pclk,          // APB control clock (e.g., 200 MHz)
  input  logic                      preset_n,      // Async active-low APB reset
  input  logic                      scan_mode,     // DFT scan-mode bypass for reset sync

  //----------------------------------------------------------------------------
  // 1:1 Sensor Pixel Input Interface (sensor_clk domain)
  //----------------------------------------------------------------------------
  input  logic                      s_vsync,
  input  logic                      s_hsync,
  input  logic                      s_valid,
  input  logic [PIXEL_WIDTH-1:0]    s_pixel,

  //----------------------------------------------------------------------------
  // Processed Video Output Interface - AXI4-Stream Video (core_clk domain)
  //----------------------------------------------------------------------------
  output logic                      m_axis_tvalid,
  input  logic                      m_axis_tready,
  output logic [PIXEL_WIDTH-1:0]    m_axis_tdata,
  output logic                      m_axis_tuser,  // Start of Frame (SOF)
  output logic                      m_axis_tlast,  // End of Line (EOL)

  //----------------------------------------------------------------------------
  // APB3 Slave Control & Configuration Interface (pclk domain)
  //----------------------------------------------------------------------------
  input  logic [APB_ADDR_WIDTH-1:0] paddr,
  input  logic                      psel,
  input  logic                      penable,
  input  logic                      pwrite,
  input  logic [APB_DATA_WIDTH-1:0] pwdata,
  output logic [APB_DATA_WIDTH-1:0] prdata,
  output logic                      pready,
  output logic                      pslverr,

  //----------------------------------------------------------------------------
  // Interrupt Output (core_clk domain)
  //----------------------------------------------------------------------------
  output logic                      irq_out
);

  //----------------------------------------------------------------------------
  // 1. Reset Synchronizers (Async Assert, 2-Stage Sync Deassert + DFT Scan Mux)
  //----------------------------------------------------------------------------
  (* ASYNC_REG = "TRUE" *) logic [1:0] sensor_rst_sync;
  (* ASYNC_REG = "TRUE" *) logic [1:0] core_rst_sync;
  logic sensor_rst_n, core_rst_n;

  always_ff @(posedge sensor_clk or negedge sensor_arst_n) begin
    if (!sensor_arst_n) sensor_rst_sync <= 2'b00;
    else                sensor_rst_sync <= {sensor_rst_sync[0], 1'b1};
  end
  assign sensor_rst_n = scan_mode ? sensor_arst_n : sensor_rst_sync[1];

  always_ff @(posedge core_clk or negedge core_arst_n) begin
    if (!core_arst_n)   core_rst_sync <= 2'b00;
    else                core_rst_sync <= {core_rst_sync[0], 1'b1};
  end
  assign core_rst_n = scan_mode ? core_arst_n : core_rst_sync[1];

  //----------------------------------------------------------------------------
  // Interconnect Signals
  //----------------------------------------------------------------------------
  vpu_cfg_t              active_cfg;
  vpu_status_t           core_status;

  logic                  sif_valid, sif_ready;
  vpu_stream_t           sif_stream;
  logic                  fifo_ovf_pulse, fifo_udf_pulse;

  logic                  lb_valid, lb_ready;
  logic                  lb_sof, lb_eol;
  logic [PIXEL_WIDTH-1:0] win_p [0:2][0:2];
  logic                  frame_start_pulse, frame_done_pulse, dim_err_pulse;
  logic                  lb_busy;
  logic [15:0]           curr_line_idx, last_line_total;

  logic                  dp_valid, dp_ready;
  vpu_stream_t           dp_stream;

  //----------------------------------------------------------------------------
  // 2. Sensor Interface & Async FIFO (sensor_clk -> core_clk)
  //----------------------------------------------------------------------------
  vpu_sensor_if #(
    .FIFO_ADDR_WIDTH ( FIFO_ADDR_WIDTH )
  ) u_sensor_if (
    .sensor_clk          ( sensor_clk        ),
    .sensor_rst_n        ( sensor_rst_n      ),
    .s_vsync             ( s_vsync           ),
    .s_hsync             ( s_hsync           ),
    .s_valid             ( s_valid           ),
    .s_pixel             ( s_pixel           ),
    .core_clk            ( core_clk          ),
    .core_rst_n          ( core_rst_n        ),
    .core_enable         ( active_cfg.enable ),
    .m_valid             ( sif_valid         ),
    .m_ready             ( sif_ready         ),
    .m_stream            ( sif_stream        ),
    .fifo_overflow_core  ( fifo_ovf_pulse    ),
    .fifo_underflow_core ( fifo_udf_pulse    )
  );

  //----------------------------------------------------------------------------
  // 3. 4K Circular Line Buffer & 3x3 Window Generator (core_clk)
  //----------------------------------------------------------------------------
  vpu_line_buffer u_line_buffer (
    .clk               ( core_clk          ),
    .rst_n             ( core_rst_n        ),
    .cfg               ( active_cfg        ),
    .s_valid           ( sif_valid         ),
    .s_ready           ( sif_ready         ),
    .s_stream          ( sif_stream        ),
    .m_valid           ( lb_valid          ),
    .m_ready           ( lb_ready          ),
    .m_sof             ( lb_sof            ),
    .m_eol             ( lb_eol            ),
    .win_p             ( win_p             ),
    .frame_start_pulse ( frame_start_pulse ),
    .frame_done_pulse  ( frame_done_pulse  ),
    .dim_err_pulse     ( dim_err_pulse     ),
    .busy              ( lb_busy           ),
    .curr_line_idx     ( curr_line_idx     ),
    .last_line_total   ( last_line_total   )
  );

  //----------------------------------------------------------------------------
  // 4. 5-Stage 2 GHz Processing Datapath (core_clk)
  //----------------------------------------------------------------------------
  vpu_datapath u_datapath (
    .clk      ( core_clk   ),
    .rst_n    ( core_rst_n ),
    .cfg      ( active_cfg ),
    .s_valid  ( lb_valid   ),
    .s_ready  ( lb_ready   ),
    .s_sof    ( lb_sof     ),
    .s_eol    ( lb_eol     ),
    .win_p    ( win_p      ),
    .m_valid  ( dp_valid   ),
    .m_ready  ( dp_ready   ),
    .m_stream ( dp_stream  )
  );

  //----------------------------------------------------------------------------
  // 5. Skid-Buffered Output Formatter (core_clk)
  //----------------------------------------------------------------------------
  vpu_out_if u_out_if (
    .clk           ( core_clk          ),
    .rst_n         ( core_rst_n        ),
    .enable        ( active_cfg.enable ),
    .s_valid       ( dp_valid          ),
    .s_ready       ( dp_ready          ),
    .s_stream      ( dp_stream         ),
    .m_axis_tvalid ( m_axis_tvalid     ),
    .m_axis_tready ( m_axis_tready     ),
    .m_axis_tdata  ( m_axis_tdata      ),
    .m_axis_tuser  ( m_axis_tuser      ),
    .m_axis_tlast  ( m_axis_tlast      )
  );

  //----------------------------------------------------------------------------
  // 6. Sticky Status & Telemetry Counters (core_clk)
  //----------------------------------------------------------------------------
  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      core_status.fifo_overflow  <= 1'b0;
      core_status.fifo_underflow <= 1'b0;
      core_status.frame_done     <= 1'b0;
      core_status.dim_err        <= 1'b0;
      core_status.frame_count    <= 32'd0;
    end else if (active_cfg.soft_clear) begin
      core_status.fifo_overflow  <= 1'b0;
      core_status.fifo_underflow <= 1'b0;
      core_status.frame_done     <= 1'b0;
      core_status.dim_err        <= 1'b0;
    end else begin
      if (fifo_ovf_pulse)   core_status.fifo_overflow  <= 1'b1;
      if (fifo_udf_pulse)   core_status.fifo_underflow <= 1'b1;
      if (dim_err_pulse)    core_status.dim_err        <= 1'b1;
      if (frame_done_pulse) begin
        core_status.frame_done  <= 1'b1;
        core_status.frame_count <= core_status.frame_count + 32'd1;
      end
    end
  end

  assign core_status.busy            = lb_busy || dp_valid || m_axis_tvalid;
  assign core_status.curr_line_count = curr_line_idx;
  assign core_status.last_line_count = last_line_total;
  assign irq_out                     = core_status.fifo_overflow ||
                                       core_status.dim_err       ||
                                       core_status.frame_done;

  //----------------------------------------------------------------------------
  // 7. APB3 Control & Status Registers (pclk <-> core_clk)
  //----------------------------------------------------------------------------
  vpu_csr u_csr (
    .pclk              ( pclk              ),
    .preset_n          ( preset_n          ),
    .paddr             ( paddr             ),
    .psel              ( psel              ),
    .penable           ( penable           ),
    .pwrite            ( pwrite            ),
    .pwdata            ( pwdata            ),
    .prdata            ( prdata            ),
    .pready            ( pready            ),
    .pslverr           ( pslverr           ),
    .core_clk          ( core_clk          ),
    .core_rst_n        ( core_rst_n        ),
    .frame_start_pulse ( frame_start_pulse ),
    .active_cfg        ( active_cfg        ),
    .core_status       ( core_status       )
  );

endmodule : vpu_top
