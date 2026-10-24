//------------------------------------------------------------------------------
// File       : vpu_datapath.sv
// Author     : Senior RTL Design & Verification Lead
// Description: 5-Stage Pipelined 2 GHz Video Processing Datapath (5nm TSMC).
//              Designed specifically for < 500 ps cycle time (<= 8 FO4 levels
//              of logic per stage):
//                Stage D1: Horizontal Row Reductions (3x3 Gaussian & Center Delay)
//                Stage D2: Vertical Reduction & High-Pass Detail Extraction
//                Stage D3: Coring Threshold & Sharpening / Gain Multiplication
//                Stage D4: Black-Level Offset & Sharpened Pixel Accumulation
//                Stage D5: Programmable Min/Max Saturation Clipping & Bypass Mux
//------------------------------------------------------------------------------

module vpu_datapath
  import vpu_pkg::*;
(
  input  logic                   clk,
  input  logic                   rst_n,
  input  vpu_cfg_t               cfg,

  // Upstream 3x3 Window Input
  input  logic                   s_valid,
  output logic                   s_ready,
  input  logic                   s_sof,
  input  logic                   s_eol,
  input  logic [PIXEL_WIDTH-1:0] win_p [0:2][0:2],

  // Downstream Processed Pixel Stream Output
  output logic                   m_valid,
  input  logic                   m_ready,
  output vpu_stream_t            m_stream
);

  // Global pipeline stall control
  wire pipe_en = !m_valid || m_ready;
  assign s_ready = pipe_en;

  //----------------------------------------------------------------------------
  // STAGE D1: Horizontal 3-Tap Adder Tree ([1, 2, 1] per row)
  // Logic depth: One 11-bit adder + One 12-bit adder (~120 ps in 5nm)
  //----------------------------------------------------------------------------
  logic                   v_d1, sof_d1, eol_d1;
  logic [PIXEL_WIDTH+1:0] gauss_row0_d1, gauss_row1_d1, gauss_row2_d1; // Max 4*1023 = 4092 (12b)
  logic [PIXEL_WIDTH-1:0] center_d1;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      v_d1          <= 1'b0;
      sof_d1        <= 1'b0;
      eol_d1        <= 1'b0;
      gauss_row0_d1 <= '0;
      gauss_row1_d1 <= '0;
      gauss_row2_d1 <= '0;
      center_d1     <= '0;
    end else if (!cfg.enable || cfg.soft_clear) begin
      v_d1          <= 1'b0;
    end else if (pipe_en) begin
      v_d1   <= s_valid;
      sof_d1 <= s_sof;
      eol_d1 <= s_eol;
      if (s_valid) begin
        gauss_row0_d1 <= {2'b00, win_p[0][0]} + {1'b0, win_p[0][1], 1'b0} + {2'b00, win_p[0][2]};
        gauss_row1_d1 <= {2'b00, win_p[1][0]} + {1'b0, win_p[1][1], 1'b0} + {2'b00, win_p[1][2]};
        gauss_row2_d1 <= {2'b00, win_p[2][0]} + {1'b0, win_p[2][1], 1'b0} + {2'b00, win_p[2][2]};
        center_d1     <= win_p[1][1];
      end
    end
  end

  //----------------------------------------------------------------------------
  // STAGE D2: Vertical [1, 2, 1] Combination & High-Frequency Detail Extraction
  // Logic depth: 14-bit adder + 11-bit subtractor (~140 ps in 5nm)
  //----------------------------------------------------------------------------
  logic                        v_d2, sof_d2, eol_d2;
  logic [PIXEL_WIDTH-1:0]      gauss_norm_d2; // Normalized 10-bit Gaussian (sum + 8) >> 4
  logic [PIXEL_WIDTH-1:0]      center_d2;
  logic signed [PIXEL_WIDTH:0] detail_d2;     // Signed 11-bit: center - gauss_norm

  wire [PIXEL_WIDTH+3:0] gauss_sum_comb = {2'b00, gauss_row0_d1} +
                                          {1'b0,  gauss_row1_d1, 1'b0} +
                                          {2'b00, gauss_row2_d1} +
                                          14'd8; // Rounding half-LSB before >> 4
  wire [PIXEL_WIDTH-1:0] gauss_norm_comb = gauss_sum_comb[PIXEL_WIDTH+3:4];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      v_d2          <= 1'b0;
      sof_d2        <= 1'b0;
      eol_d2        <= 1'b0;
      gauss_norm_d2 <= '0;
      center_d2     <= '0;
      detail_d2     <= '0;
    end else if (!cfg.enable || cfg.soft_clear) begin
      v_d2          <= 1'b0;
    end else if (pipe_en) begin
      v_d2   <= v_d1;
      sof_d2 <= sof_d1;
      eol_d2 <= eol_d1;
      if (v_d1) begin
        gauss_norm_d2 <= gauss_norm_comb;
        center_d2     <= center_d1;
        detail_d2     <= signed'({1'b0, center_d1}) - signed'({1'b0, gauss_norm_comb});
      end
    end
  end

  //----------------------------------------------------------------------------
  // STAGE D3: Noise Coring & Dedicated Multiplier Stage (Gain + Sharp Boost)
  // Isolates multipliers in their own pipeline stage to easily meet 500 ps STA.
  //----------------------------------------------------------------------------
  logic                        v_d3, sof_d3, eol_d3;
  logic [PIXEL_WIDTH-1:0]      center_d3;
  logic [PIXEL_WIDTH-1:0]      edge_mag_d3;
  logic [17:0]                 gain_prod_d3;  // 10b base * 8b UQ2.6 gain = 18b
  logic signed [18:0]          sharp_prod_d3; // 11b signed detail * 8b UQ1.7 = 19b

  // Absolute magnitude of detail for coring comparison and edge mode
  wire [PIXEL_WIDTH-1:0] detail_abs = (detail_d2 < 0) ?
                                      PIXEL_WIDTH'(-detail_d2) :
                                      detail_d2[PIXEL_WIDTH-1:0];
  wire signed [PIXEL_WIDTH:0] cored_detail = (detail_abs > cfg.coring_thresh) ?
                                             detail_d2 : 11'sd0;

  // Base pixel for gain: Gaussian filtered or raw center depending on mode
  wire [PIXEL_WIDTH-1:0] base_pix_comb = (cfg.proc_mode == PROC_GAUSSIAN) ?
                                         gauss_norm_d2 : center_d2;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      v_d3          <= 1'b0;
      sof_d3        <= 1'b0;
      eol_d3        <= 1'b0;
      center_d3     <= '0;
      edge_mag_d3   <= '0;
      gain_prod_d3  <= '0;
      sharp_prod_d3 <= '0;
    end else if (!cfg.enable || cfg.soft_clear) begin
      v_d3          <= 1'b0;
    end else if (pipe_en) begin
      v_d3   <= v_d2;
      sof_d3 <= sof_d2;
      eol_d3 <= eol_d2;
      if (v_d2) begin
        center_d3     <= center_d2;
        edge_mag_d3   <= detail_abs;
        gain_prod_d3  <= base_pix_comb * cfg.digital_gain;
        sharp_prod_d3 <= cored_detail * signed'({1'b0, cfg.sharp_strength});
      end
    end
  end

  //----------------------------------------------------------------------------
  // STAGE D4: Post-Multiplier Scaling, Black-Level Offset & Boost Accumulation
  //----------------------------------------------------------------------------
  logic                   v_d4, sof_d4, eol_d4;
  logic [PIXEL_WIDTH-1:0] center_d4;
  logic [PIXEL_WIDTH-1:0] edge_mag_d4;
  logic signed [13:0]     accum_pix_d4; // Signed 14-bit pre-clip pixel

  wire signed [13:0] scaled_base  = signed'({2'b00, gain_prod_d3[17:6]}); // UQ2.6 >> 6
  wire signed [13:0] scaled_boost = (cfg.proc_mode == PROC_SHARPEN) ?
                                    14'(sharp_prod_d3 >>> 7) : 14'sd0;    // UQ1.7 >>> 7
  wire signed [13:0] offset_ext   = 14'(cfg.black_offset);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      v_d4         <= 1'b0;
      sof_d4       <= 1'b0;
      eol_d4       <= 1'b0;
      center_d4    <= '0;
      edge_mag_d4  <= '0;
      accum_pix_d4 <= '0;
    end else if (!cfg.enable || cfg.soft_clear) begin
      v_d4         <= 1'b0;
    end else if (pipe_en) begin
      v_d4   <= v_d3;
      sof_d4 <= sof_d3;
      eol_d4 <= eol_d3;
      if (v_d3) begin
        center_d4    <= center_d3;
        edge_mag_d4  <= edge_mag_d3;
        accum_pix_d4 <= scaled_base + scaled_boost + offset_ext;
      end
    end
  end

  //----------------------------------------------------------------------------
  // STAGE D5: Programmable Saturation Clipping & Mode Multiplexer
  //----------------------------------------------------------------------------
  wire signed [13:0] clip_min_s = signed'({4'b0000, cfg.clip_min});
  wire signed [13:0] clip_max_s = signed'({4'b0000, cfg.clip_max});

  logic [PIXEL_WIDTH-1:0] clipped_pix_comb;
  always_comb begin
    if (accum_pix_d4 < clip_min_s) begin
      clipped_pix_comb = cfg.clip_min;
    end else if (accum_pix_d4 > clip_max_s) begin
      clipped_pix_comb = cfg.clip_max;
    end else begin
      clipped_pix_comb = accum_pix_d4[PIXEL_WIDTH-1:0];
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      m_valid  <= 1'b0;
      m_stream <= '0;
    end else if (!cfg.enable || cfg.soft_clear) begin
      m_valid  <= 1'b0;
    end else if (pipe_en) begin
      m_valid <= v_d4;
      if (v_d4) begin
        m_stream.sof <= sof_d4;
        m_stream.eol <= eol_d4;
        case (cfg.proc_mode)
          PROC_BYPASS:   m_stream.pixel <= center_d4;
          PROC_EDGE:     m_stream.pixel <= edge_mag_d4;
          default:       m_stream.pixel <= clipped_pix_comb; // GAUSSIAN or SHARPEN
        endcase
      end
    end
  end

endmodule : vpu_datapath
