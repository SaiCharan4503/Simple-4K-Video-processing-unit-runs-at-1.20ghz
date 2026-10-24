//------------------------------------------------------------------------------
// File       : vpu_csr.sv
// Author     : Senior RTL Design & Verification Lead
// Description: APB3 Control & Status Register (CSR) block with quasi-static
//              configuration shadowing (updated only on frame boundaries or
//              when core is idle) and CDC synchronization from pclk to core_clk.
//------------------------------------------------------------------------------

module vpu_csr
  import vpu_pkg::*;
(
  // APB3 Slave Interface (pclk domain, e.g., 200 MHz control bus)
  input  logic                      pclk,
  input  logic                      preset_n,
  input  logic [APB_ADDR_WIDTH-1:0] paddr,
  input  logic                      psel,
  input  logic                      penable,
  input  logic                      pwrite,
  input  logic [APB_DATA_WIDTH-1:0] pwdata,
  output logic [APB_DATA_WIDTH-1:0] prdata,
  output logic                      pready,
  output logic                      pslverr,

  // Core Clock Domain (2 GHz)
  input  logic                      core_clk,
  input  logic                      core_rst_n,
  input  logic                      frame_start_pulse, // Shadow latch trigger
  output vpu_cfg_t                  active_cfg,        // Glitch-free active config
  input  vpu_status_t               core_status
);

  // APB is always zero-wait-state ready
  assign pready  = 1'b1;
  assign pslverr = 1'b0;

  wire apb_write = psel && penable && pwrite;
  wire apb_read  = psel && !pwrite;

  //----------------------------------------------------------------------------
  // APB Domain Programmable Registers
  //----------------------------------------------------------------------------
  logic                   r_enable;
  res_mode_e              r_res_mode;
  proc_mode_e             r_proc_mode;
  fps_mode_e              r_fps_mode;
  logic                   r_soft_clear_toggle;
  logic [12:0]            r_act_width;
  logic [12:0]            r_act_height;
  logic [7:0]             r_gain;
  logic signed [9:0]      r_offset;
  logic [7:0]             r_sharp_strength;
  logic [PIXEL_WIDTH-1:0] r_coring_thresh;
  logic [PIXEL_WIDTH-1:0] r_clip_min;
  logic [PIXEL_WIDTH-1:0] r_clip_max;

  always_ff @(posedge pclk or negedge preset_n) begin
    if (!preset_n) begin
      r_enable            <= 1'b0;
      r_res_mode          <= MODE_4K;
      r_proc_mode         <= PROC_SHARPEN;
      r_fps_mode          <= FPS_120;
      r_soft_clear_toggle <= 1'b0;
      r_act_width         <= 13'd3840;
      r_act_height        <= 13'd2160;
      r_gain              <= 8'h40;   // 1.0x in UQ2.6
      r_offset            <= 10'sd0;
      r_sharp_strength    <= 8'h40;   // 0.5x in UQ1.7
      r_coring_thresh     <= 10'd8;
      r_clip_min          <= 10'd0;
      r_clip_max          <= 10'd1023;
    end else if (apb_write) begin
      case (paddr)
        CSR_CTRL: begin
          r_enable    <= pwdata[0];
          r_res_mode  <= res_mode_e'(pwdata[2:1]);
          r_proc_mode <= proc_mode_e'(pwdata[4:3]);
          r_fps_mode  <= fps_mode_e'(pwdata[6:5]);
          if (pwdata[7]) begin
            r_soft_clear_toggle <= ~r_soft_clear_toggle;
          end
          // Auto-populate default dimensions if mode changes and ACT_DIM not manually overridden
          case (res_mode_e'(pwdata[2:1]))
            MODE_1K: begin
              r_act_width  <= 13'd1024;
              r_act_height <= 13'd768;
            end
            MODE_2K: begin
              r_act_width  <= 13'd1920;
              r_act_height <= 13'd1080;
            end
            default: begin
              r_act_width  <= 13'd3840;
              r_act_height <= 13'd2160;
            end
          endcase
        end
        CSR_ACT_DIM: begin
          r_act_width  <= pwdata[12:0];
          r_act_height <= pwdata[28:16];
        end
        CSR_GAIN_OFFSET: begin
          r_offset     <= signed'(pwdata[9:0]);
          r_gain       <= pwdata[23:16];
        end
        CSR_SHARP_CFG: begin
          r_sharp_strength <= pwdata[7:0];
          r_coring_thresh  <= pwdata[17:8];
        end
        CSR_CLIP_CFG: begin
          r_clip_min <= pwdata[9:0];
          r_clip_max <= pwdata[25:16];
        end
        default: ;
      endcase
    end
  end

  //----------------------------------------------------------------------------
  // Status Synchronization (core_clk -> pclk) for APB Readback
  //----------------------------------------------------------------------------
  (* ASYNC_REG = "TRUE" *) logic [4:0]  status_flags_sync1, status_flags_sync2;
  (* ASYNC_REG = "TRUE" *) logic [31:0] frame_cnt_sync1, frame_cnt_sync2;
  (* ASYNC_REG = "TRUE" *) logic [31:0] line_cnt_sync1, line_cnt_sync2;

  always_ff @(posedge pclk or negedge preset_n) begin
    if (!preset_n) begin
      status_flags_sync1 <= '0;
      status_flags_sync2 <= '0;
      frame_cnt_sync1    <= '0;
      frame_cnt_sync2    <= '0;
      line_cnt_sync1     <= '0;
      line_cnt_sync2     <= '0;
    end else begin
      status_flags_sync1 <= {core_status.dim_err,
                             core_status.frame_done,
                             core_status.fifo_underflow,
                             core_status.fifo_overflow,
                             core_status.busy};
      status_flags_sync2 <= status_flags_sync1;
      frame_cnt_sync1    <= core_status.frame_count;
      frame_cnt_sync2    <= frame_cnt_sync1;
      line_cnt_sync1     <= {core_status.last_line_count, core_status.curr_line_count};
      line_cnt_sync2     <= line_cnt_sync1;
    end
  end

  always_comb begin
    prdata = '0;
    if (apb_read) begin
      case (paddr)
        CSR_CTRL:        prdata = {25'd0, r_fps_mode, r_proc_mode, r_res_mode, r_enable};
        CSR_ACT_DIM:     prdata = {3'd0, r_act_height, 3'd0, r_act_width};
        CSR_GAIN_OFFSET: prdata = {8'd0, r_gain, 6'd0, r_offset};
        CSR_SHARP_CFG:   prdata = {14'd0, r_coring_thresh, r_sharp_strength};
        CSR_CLIP_CFG:    prdata = {6'd0, r_clip_max, 6'd0, r_clip_min};
        CSR_STATUS:      prdata = {27'd0, status_flags_sync2};
        CSR_FRAME_CNT:   prdata = frame_cnt_sync2;
        CSR_LINE_CNT:    prdata = line_cnt_sync2;
        default:         prdata = 32'h0000_0000;
      endcase
    end
  end

  //----------------------------------------------------------------------------
  // CDC: pclk -> core_clk Synchronization and Frame-Boundary Shadowing
  //----------------------------------------------------------------------------
  (* ASYNC_REG = "TRUE" *) logic [2:0] enable_sync;
  (* ASYNC_REG = "TRUE" *) logic [2:0] clear_sync;

  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      enable_sync <= 3'b000;
      clear_sync  <= 3'b000;
    end else begin
      enable_sync <= {enable_sync[1:0], r_enable};
      clear_sync  <= {clear_sync[1:0],  r_soft_clear_toggle};
    end
  end

  wire core_enable_synced = enable_sync[2];
  wire core_clear_pulse   = clear_sync[2] ^ clear_sync[1];

  // Quasi-static configuration bus captured only when core is idle (!busy)
  // or at the start of a new frame (frame_start_pulse) to guarantee glitch-free
  // mode switching between 1K, 2K, and 4K.
  always_ff @(posedge core_clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      active_cfg.enable         <= 1'b0;
      active_cfg.res_mode       <= MODE_4K;
      active_cfg.proc_mode      <= PROC_SHARPEN;
      active_cfg.fps_mode       <= FPS_120;
      active_cfg.soft_clear     <= 1'b0;
      active_cfg.active_width   <= 13'd3840;
      active_cfg.active_height  <= 13'd2160;
      active_cfg.digital_gain   <= 8'h40;
      active_cfg.black_offset   <= 10'sd0;
      active_cfg.sharp_strength <= 8'h40;
      active_cfg.coring_thresh  <= 10'd8;
      active_cfg.clip_min       <= 10'd0;
      active_cfg.clip_max       <= 10'd1023;
    end else begin
      active_cfg.enable     <= core_enable_synced;
      active_cfg.soft_clear <= core_clear_pulse;

      if (!core_status.busy || frame_start_pulse) begin
        active_cfg.res_mode       <= r_res_mode;
        active_cfg.proc_mode      <= r_proc_mode;
        active_cfg.fps_mode       <= r_fps_mode;
        active_cfg.active_width   <= r_act_width;
        active_cfg.active_height  <= r_act_height;
        active_cfg.digital_gain   <= r_gain;
        active_cfg.black_offset   <= r_offset;
        active_cfg.sharp_strength <= r_sharp_strength;
        active_cfg.coring_thresh  <= r_coring_thresh;
        active_cfg.clip_min       <= r_clip_min;
        active_cfg.clip_max       <= r_clip_max;
      end
    end
  end

endmodule : vpu_csr
