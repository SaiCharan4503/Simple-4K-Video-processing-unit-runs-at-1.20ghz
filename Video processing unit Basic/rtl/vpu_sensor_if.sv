//------------------------------------------------------------------------------
// File       : vpu_sensor_if.sv
// Author     : Senior RTL Design & Verification Lead
// Description: 1:1 Direct Parallel/CSI-Unpacked Sensor Interface & Input Buffer.
//              Captures incoming sensor_clk pixel stream, detects SOF (first
//              active pixel after VSYNC) and EOL (falling edge of HSYNC or
//              explicit last active pixel), crosses into 2 GHz core_clk via
//              vpu_async_fifo, and registers the output stream stage to meet
//              2 GHz timing (< 500 ps period).
//------------------------------------------------------------------------------

module vpu_sensor_if
  import vpu_pkg::*;
#(
  parameter int unsigned FIFO_ADDR_WIDTH = 4
) (
  // Sensor Clock Domain (Isochronous Source)
  input  logic                   sensor_clk,
  input  logic                   sensor_rst_n,
  input  logic                   s_vsync,      // Active-high frame valid / VSYNC envelope
  input  logic                   s_hsync,      // Active-high line valid / HSYNC envelope
  input  logic                   s_valid,      // Active-high pixel valid qualifier
  input  logic [PIXEL_WIDTH-1:0] s_pixel,      // 1:1 10-bit sensor pixel

  // Core Clock Domain (2 GHz)
  input  logic                   core_clk,
  input  logic                   core_rst_n,
  input  logic                   core_enable,

  // Handshake to Downstream Processing Pipeline
  output logic                   m_valid,
  input  logic                   m_ready,
  output vpu_stream_t            m_stream,

  // Telemetry & Error Pulses (in core_clk domain)
  output logic                   fifo_overflow_core,
  output logic                   fifo_underflow_core
);

  //----------------------------------------------------------------------------
  // Sensor Clock Domain: 2-Cycle Input Delay Pipeline to Detect SOF and EOL
  //----------------------------------------------------------------------------
  // Because a standard parallel/unpacked sensor interface drops s_hsync/s_valid
  // AFTER the last pixel of a line, holding 1 pixel in a staging register allows
  // exact lookahead detection of End-of-Line (EOL) when (s_hsync & s_valid) falls.
  logic                   s_vsync_d1;
  logic                   s_active_d1;
  logic [PIXEL_WIDTH-1:0] s_pixel_d1;
  logic                   waiting_for_sof;
  logic                   sof_d1;

  wire s_active = s_vsync && s_hsync && s_valid;

  always_ff @(posedge sensor_clk or negedge sensor_rst_n) begin
    if (!sensor_rst_n) begin
      s_vsync_d1      <= 1'b0;
      s_active_d1     <= 1'b0;
      s_pixel_d1      <= '0;
      waiting_for_sof <= 1'b1;
      sof_d1          <= 1'b0;
    end else begin
      s_vsync_d1 <= s_vsync;

      // Arm SOF detector when VSYNC is low or rises
      if (!s_vsync) begin
        waiting_for_sof <= 1'b1;
      end else if (s_active && waiting_for_sof) begin
        waiting_for_sof <= 1'b0;
      end

      s_active_d1 <= s_active;
      if (s_active) begin
        s_pixel_d1 <= s_pixel;
        sof_d1     <= waiting_for_sof;
      end else begin
        sof_d1     <= 1'b0;
      end
    end
  end

  // EOL is asserted on the staged pixel if it was active and the current cycle
  // is no longer active on the same line (!s_hsync || !s_valid).
  wire eol_d1 = s_active_d1 && (!s_hsync || !s_valid);

  vpu_stream_t fifo_wr_packet;
  assign fifo_wr_packet.sof   = sof_d1;
  assign fifo_wr_packet.eol   = eol_d1;
  assign fifo_wr_packet.pixel = s_pixel_d1;

  //----------------------------------------------------------------------------
  // Asynchronous FIFO (sensor_clk -> core_clk)
  //----------------------------------------------------------------------------
  logic        fifo_full;
  logic        fifo_empty;
  logic        fifo_rd_en;
  vpu_stream_t fifo_rd_packet;
  logic        wr_ovf_pulse_s;

  vpu_async_fifo #(
    .DATA_WIDTH ( $bits(vpu_stream_t) ),
    .ADDR_WIDTH ( FIFO_ADDR_WIDTH     )
  ) u_async_fifo (
    .wr_clk             ( sensor_clk          ),
    .wr_rst_n           ( sensor_rst_n        ),
    .wr_en              ( s_active_d1         ),
    .wr_data            ( fifo_wr_packet      ),
    .wr_full            ( fifo_full           ),
    .wr_overflow_pulse  ( wr_ovf_pulse_s      ),

    .rd_clk             ( core_clk            ),
    .rd_rst_n           ( core_rst_n          ),
    .rd_en              ( fifo_rd_en          ),
    .rd_data            ( fifo_rd_packet      ),
    .rd_empty           ( fifo_empty          ),
    .rd_underflow_pulse ( fifo_underflow_core )
  );

  //----------------------------------------------------------------------------
  // Toggle Synchronizer for Overflow Event (sensor_clk -> core_clk)
  //----------------------------------------------------------------------------
  logic wr_ovf_toggle_s;
  (* ASYNC_REG = "TRUE" *) logic [2:0] wr_ovf_sync_c;

  always_ff @(posedge sensor_clk or negedge sensor_rst_n) begin
    if (!sensor_rst_n) begin
      wr_ovf_toggle_s <= 1'b0;
    end else if (wr_ovf_pulse_s) begin
      wr_ovf_toggle_s <= ~wr_ovf_toggle_s;
    end
  end

  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      wr_ovf_sync_c <= 3'b000;
    end else begin
      wr_ovf_sync_c <= {wr_ovf_sync_c[1:0], wr_ovf_toggle_s};
    end
  end

  assign fifo_overflow_core = wr_ovf_sync_c[2] ^ wr_ovf_sync_c[1];

  //----------------------------------------------------------------------------
  // Core Clock Output Skid/Pipeline Register (2 GHz Timing Isolation)
  //----------------------------------------------------------------------------
  // Decouples async FIFO RAM mux delay from downstream line-buffer logic.
  wire pipe_advance = !m_valid || m_ready;
  assign fifo_rd_en = core_enable && !fifo_empty && pipe_advance;

  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      m_valid  <= 1'b0;
      m_stream <= '0;
    end else if (!core_enable) begin
      m_valid  <= 1'b0;
    end else if (pipe_advance) begin
      m_valid  <= !fifo_empty;
      if (!fifo_empty) begin
        m_stream <= fifo_rd_packet;
      end
    end
  end

endmodule : vpu_sensor_if
