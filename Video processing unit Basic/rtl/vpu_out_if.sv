//------------------------------------------------------------------------------
// File       : vpu_out_if.sv
// Author     : Senior RTL Design & Verification Lead
// Description: Zero-Bubble Skid-Buffered Output Formatter (AXI4-Stream Video
//              compatible). Guarantees registered output signals (m_axis_tdata,
//              tvalid, tuser/sof, tlast/eol) with zero combinational feedthrough
//              between m_axis_tready and upstream logic for 2 GHz STA closure.
//------------------------------------------------------------------------------

module vpu_out_if
  import vpu_pkg::*;
(
  input  logic                   clk,
  input  logic                   rst_n,
  input  logic                   enable,

  // Upstream from Datapath
  input  logic                   s_valid,
  output logic                   s_ready,
  input  vpu_stream_t            s_stream,

  // Downstream AXI4-Stream Video Master Interface (core_clk 2 GHz)
  output logic                   m_axis_tvalid,
  input  logic                   m_axis_tready,
  output logic [PIXEL_WIDTH-1:0] m_axis_tdata,
  output logic                   m_axis_tuser, // SOF (Start of Frame)
  output logic                   m_axis_tlast  // EOL (End of Line)
);

  // Two-entry Carloni/Skid Buffer for full valid/ready timing isolation
  vpu_stream_t data_reg, skid_reg;
  logic        valid_reg, skid_valid_reg;

  // Ready is purely a registered state (no combinational path from m_axis_tready)
  assign s_ready = !skid_valid_reg;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      valid_reg      <= 1'b0;
      skid_valid_reg <= 1'b0;
      data_reg       <= '0;
      skid_reg       <= '0;
    end else if (!enable) begin
      valid_reg      <= 1'b0;
      skid_valid_reg <= 1'b0;
    end else begin
      if (skid_valid_reg) begin
        // Currently holding a skid word; drain when downstream is ready
        if (m_axis_tready) begin
          data_reg       <= skid_reg;
          valid_reg      <= 1'b1;
          skid_valid_reg <= 1'b0;
        end
      end else begin
        // Normal streaming mode
        if (!valid_reg || m_axis_tready) begin
          valid_reg <= s_valid;
          if (s_valid) begin
            data_reg <= s_stream;
          end
        end else if (s_valid) begin
          // Downstream stalled while valid_reg is full -> capture into skid_reg
          skid_reg       <= s_stream;
          skid_valid_reg <= 1'b1;
        end
      end
    end
  end

  assign m_axis_tvalid = valid_reg;
  assign m_axis_tdata  = data_reg.pixel;
  assign m_axis_tuser  = data_reg.sof;
  assign m_axis_tlast  = data_reg.eol;

endmodule : vpu_out_if
