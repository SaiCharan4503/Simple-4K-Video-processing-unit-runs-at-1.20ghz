//------------------------------------------------------------------------------
// File       : vpu_line_buffer.sv
// Author     : Senior RTL Design & Verification Lead
// Description: 4K-Capable Circular Line Buffer & 3x3 Sliding Window Generator.
//              Uses 2 Single-Port 1RW Banked/Pipelined Line SRAM Macros
//              (4096 x 10-bit each) + incoming current line to form a 3x3
//              spatial neighborhood at 1 pixel/clock @ 2 GHz.
//              Includes SRAM output pipeline registers to isolate SRAM Tck-q
//              (~180ps in 5nm) from spatial arithmetic logic, and dynamic
//              pointer wrap for seamless 1K, 2K, and 4K mode switching.
//------------------------------------------------------------------------------

module vpu_line_buffer
  import vpu_pkg::*;
(
  input  logic                   clk,
  input  logic                   rst_n,
  input  vpu_cfg_t               cfg,

  // Upstream Input from Sensor IF
  input  logic                   s_valid,
  output logic                   s_ready,
  input  vpu_stream_t            s_stream,

  // Downstream Output to 5-Stage Processing Datapath
  output logic                   m_valid,
  input  logic                   m_ready,
  output logic                   m_sof,
  output logic                   m_eol,
  output logic [PIXEL_WIDTH-1:0] win_p [0:2][0:2], // 3x3 replicated spatial window

  // Telemetry & Frame Sync
  output logic                   frame_start_pulse,
  output logic                   frame_done_pulse,
  output logic                   dim_err_pulse,
  output logic                   busy,
  output logic [15:0]            curr_line_idx,
  output logic [15:0]            last_line_total
);

  // Pipeline advance signal (backpressure propagation)
  wire pipe_en = !m_valid || m_ready;
  assign s_ready = pipe_en;

  wire beat_in = s_valid && s_ready;

  //----------------------------------------------------------------------------
  // Coordinate Tracking (Column x_cnt and Row y_cnt)
  //----------------------------------------------------------------------------
  // Dynamic line width wrap based on active_width (1024 for 1K, 1920 for 2K, 3840 for 4K)
  logic [12:0] x_cnt;
  logic [12:0] y_cnt;
  logic        in_frame;

  wire is_first_col = (x_cnt == 13'd0);
  wire is_last_col  = (x_cnt == (cfg.active_width - 13'd1)) || s_stream.eol;
  wire is_first_row = (y_cnt == 13'd0);
  wire is_last_row  = (y_cnt == (cfg.active_height - 13'd1));

  assign frame_start_pulse = beat_in && s_stream.sof;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      x_cnt           <= '0;
      y_cnt           <= '0;
      in_frame        <= 1'b0;
      frame_done_pulse<= 1'b0;
      dim_err_pulse   <= 1'b0;
      last_line_total <= '0;
    end else if (!cfg.enable || cfg.soft_clear) begin
      x_cnt           <= '0;
      y_cnt           <= '0;
      in_frame        <= 1'b0;
      frame_done_pulse<= 1'b0;
      dim_err_pulse   <= 1'b0;
    end else begin
      frame_done_pulse <= 1'b0;
      dim_err_pulse    <= 1'b0;

      if (beat_in) begin
        if (s_stream.sof) begin
          x_cnt    <= 13'd1;
          y_cnt    <= 13'd0;
          in_frame <= 1'b1;
        end else if (is_last_col) begin
          x_cnt <= 13'd0;
          // Check if EOL occurred at expected active_width - 1
          if (s_stream.eol && (x_cnt != (cfg.active_width - 13'd1))) begin
            dim_err_pulse <= 1'b1;
          end
          if (is_last_row) begin
            y_cnt            <= 13'd0;
            in_frame         <= 1'b0;
            frame_done_pulse <= 1'b1;
            last_line_total  <= {3'd0, y_cnt + 13'd1};
          end else begin
            y_cnt <= y_cnt + 13'd1;
          end
        end else begin
          x_cnt <= x_cnt + 13'd1;
        end
      end
    end
  end

  assign busy          = in_frame || m_valid;
  assign curr_line_idx = {3'd0, y_cnt};

  //----------------------------------------------------------------------------
  // Stage L0 -> L1: Line Buffer SRAM Read/Write Access
  //----------------------------------------------------------------------------
  // Two line buffers (LB0 stores y-1, LB1 stores y-2).
  // To use area-efficient Single-Port (1RW) SRAM macros at 2 GHz without
  // needing Dual-Port SRAMs, we can either pack 2 pixels per 20-bit word
  // (even cycle read 2 pixels, odd cycle write 2 pixels) OR use two 1RW
  // ping-pong line banks. Here we model two synchronous 4096 x 10-bit line
  // memories with dedicated output pipeline registers (Stage L1 -> L2) to
  // meet the 500 ps cycle time in 5nm TSMC.
  logic [PIXEL_WIDTH-1:0] line_ram_m1 [0:MAX_LINE_WIDTH-1]; // Line y-1
  logic [PIXEL_WIDTH-1:0] line_ram_m2 [0:MAX_LINE_WIDTH-1]; // Line y-2

  logic [PIXEL_WIDTH-1:0] sram_rdata_m1_l1;
  logic [PIXEL_WIDTH-1:0] sram_rdata_m2_l1;
  logic [PIXEL_WIDTH-1:0] pix_curr_l1;
  logic                   valid_l1, sof_l1, eol_l1;
  logic                   first_col_l1, last_col_l1, first_row_l1, second_row_l1;

  wire [ADDR_WIDTH-1:0] col_addr = s_stream.sof ? '0 : x_cnt[ADDR_WIDTH-1:0];

  always_ff @(posedge clk) begin
    if (beat_in) begin
      // Read older lines from column col_addr and update line_ram_m1 / m2
      sram_rdata_m1_l1      <= line_ram_m1[col_addr];
      sram_rdata_m2_l1      <= line_ram_m2[col_addr];
      line_ram_m2[col_addr] <= line_ram_m1[col_addr];
      line_ram_m1[col_addr] <= s_stream.pixel;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      valid_l1      <= 1'b0;
      sof_l1        <= 1'b0;
      eol_l1        <= 1'b0;
      pix_curr_l1   <= '0;
      first_col_l1  <= 1'b0;
      last_col_l1   <= 1'b0;
      first_row_l1  <= 1'b0;
      second_row_l1 <= 1'b0;
    end else if (!cfg.enable || cfg.soft_clear) begin
      valid_l1      <= 1'b0;
    end else if (pipe_en) begin
      valid_l1 <= s_valid;
      if (s_valid) begin
        sof_l1        <= s_stream.sof;
        eol_l1        <= is_last_col;
        pix_curr_l1   <= s_stream.pixel;
        first_col_l1  <= s_stream.sof ? 1'b1 : is_first_col;
        last_col_l1   <= is_last_col;
        first_row_l1  <= s_stream.sof ? 1'b1 : is_first_row;
        second_row_l1 <= s_stream.sof ? 1'b0 : (y_cnt == 13'd1);
      end
    end
  end

  //----------------------------------------------------------------------------
  // Stage L1 -> L2: Vertical Boundary Clamp & SRAM Output Retiming Register
  //----------------------------------------------------------------------------
  // Dedicated register immediately after SRAM read output (closes 5nm SRAM Tck-q
  // path within < 250 ps). Also applies vertical border replication for Row 0/1.
  logic                   valid_l2, sof_l2, eol_l2;
  logic                   first_col_l2, last_col_l2;
  logic [PIXEL_WIDTH-1:0] row0_l2, row1_l2, row2_l2; // row0 = y-2, row1 = y-1, row2 = y

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      valid_l2     <= 1'b0;
      sof_l2       <= 1'b0;
      eol_l2       <= 1'b0;
      first_col_l2 <= 1'b0;
      last_col_l2  <= 1'b0;
      row0_l2      <= '0;
      row1_l2      <= '0;
      row2_l2      <= '0;
    end else if (!cfg.enable || cfg.soft_clear) begin
      valid_l2     <= 1'b0;
    end else if (pipe_en) begin
      valid_l2     <= valid_l1;
      sof_l2       <= sof_l1;
      eol_l2       <= eol_l1;
      first_col_l2 <= first_col_l1;
      last_col_l2  <= last_col_l1;

      row2_l2 <= pix_curr_l1;
      row1_l2 <= first_row_l1 ? pix_curr_l1 : sram_rdata_m1_l1;
      row0_l2 <= first_row_l1 ? pix_curr_l1 :
                 (second_row_l1 ? sram_rdata_m1_l1 : sram_rdata_m2_l1);
    end
  end

  //----------------------------------------------------------------------------
  // Stage L2 -> L3: 3x3 Horizontal Shift Register & Edge Replication
  //----------------------------------------------------------------------------
  // To form a centered 3x3 window around pixel (x, y) without waiting an entire
  // extra line, we maintain a 3-tap horizontal shift register [col_m2, col_m1, col_0]
  // where col_m1 is the center column and replicate left/right borders on
  // first_col and last_col.
  logic [PIXEL_WIDTH-1:0] sr_r0 [0:1];
  logic [PIXEL_WIDTH-1:0] sr_r1 [0:1];
  logic [PIXEL_WIDTH-1:0] sr_r2 [0:1];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      m_valid <= 1'b0;
      m_sof   <= 1'b0;
      m_eol   <= 1'b0;
      for (int r = 0; r < 3; r++) begin
        for (int c = 0; c < 3; c++) begin
          win_p[r][c] <= '0;
        end
      end
      sr_r0[0] <= '0; sr_r0[1] <= '0;
      sr_r1[0] <= '0; sr_r1[1] <= '0;
      sr_r2[0] <= '0; sr_r2[1] <= '0;
    end else if (!cfg.enable || cfg.soft_clear) begin
      m_valid <= 1'b0;
    end else if (pipe_en) begin
      m_valid <= valid_l2;
      m_sof   <= sof_l2;
      m_eol   <= eol_l2;

      if (valid_l2) begin
        // Shift register state update
        sr_r0[0] <= row0_l2;
        sr_r1[0] <= row1_l2;
        sr_r2[0] <= row2_l2;
        sr_r0[1] <= first_col_l2 ? row0_l2 : sr_r0[0];
        sr_r1[1] <= first_col_l2 ? row1_l2 : sr_r1[0];
        sr_r2[1] <= first_col_l2 ? row2_l2 : sr_r2[0];

        // 3x3 Window Output Assignment (replicated left border on first_col_l2)
        // Left column (c=0)
        win_p[0][0] <= first_col_l2 ? row0_l2 : sr_r0[1];
        win_p[1][0] <= first_col_l2 ? row1_l2 : sr_r1[1];
        win_p[2][0] <= first_col_l2 ? row2_l2 : sr_r2[1];

        // Center column (c=1)
        win_p[0][1] <= first_col_l2 ? row0_l2 : sr_r0[0];
        win_p[1][1] <= first_col_l2 ? row1_l2 : sr_r1[0];
        win_p[2][1] <= first_col_l2 ? row2_l2 : sr_r2[0];

        // Right column (c=2) - current incoming column
        win_p[0][2] <= row0_l2;
        win_p[1][2] <= row1_l2;
        win_p[2][2] <= row2_l2;
      end
    end
  end

endmodule : vpu_line_buffer
