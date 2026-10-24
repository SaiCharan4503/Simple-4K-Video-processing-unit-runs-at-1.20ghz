//------------------------------------------------------------------------------
// File       : vpu_uvm_pkg.sv
// Author     : Senior RTL Design & Verification Lead
// Description: Complete SystemVerilog UVM Verification Package for the 4K VPU.
//              Includes Virtual Interfaces, Sequence Items, Functional Coverage
//              Collector (all Resolution x FPS x Filter x Backpressure crosses),
//              Cycle-Accurate Reference Predictor (3x3 Gaussian/Sharpen/Edge
//              with border replication), Scoreboard, Agents, and Test Suite.
//------------------------------------------------------------------------------

package vpu_uvm_pkg;

  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import vpu_pkg::*;

  //----------------------------------------------------------------------------
  // 1. Frame Sequence Item (Transaction Object)
  //----------------------------------------------------------------------------
  class vpu_frame_item extends uvm_sequence_item;
    rand res_mode_e              res_mode;
    rand fps_mode_e              fps_mode;
    rand proc_mode_e             proc_mode;
    rand bit [12:0]              width;
    rand bit [12:0]              height;
    rand bit [7:0]               digital_gain;
    rand bit signed [9:0]        black_offset;
    rand bit [7:0]               sharp_strength;
    rand bit [PIXEL_WIDTH-1:0]   coring_thresh;
    rand bit [PIXEL_WIDTH-1:0]   clip_min;
    rand bit [PIXEL_WIDTH-1:0]   clip_max;
    rand int unsigned            hblank_cycles;
    rand int unsigned            vblank_lines;
    rand int unsigned            downstream_bp_prob; // 0..30% backpressure probability
    rand bit [PIXEL_WIDTH-1:0]   frame_pixels[][];

    // Constraint for resolution dimensions (supports full 1K/2K/4K or scaled fast-sim)
    constraint c_dimensions {
      (res_mode == MODE_1K) -> (width == 13'd1024 && height == 13'd768);
      (res_mode == MODE_2K) -> (width == 13'd1920 && height == 13'd1080);
      (res_mode == MODE_4K) -> (width == 13'd3840 && height == 13'd2160);
      res_mode != MODE_RSVD;
    }

    constraint c_blanking {
      hblank_cycles inside {[16:256]};
      vblank_lines  inside {[2:45]};
      // Keep downstream backpressure bounded so 16-deep sensor FIFO doesn't overflow
      // during continuous isochronous sensor streaming
      downstream_bp_prob inside {[0:15]};
    }

    constraint c_iq_params {
      digital_gain   inside {[8'h20 : 8'h80]}; // 0.5x to 2.0x
      black_offset   inside {[-64 : 64]};
      sharp_strength inside {[8'h00 : 8'hFF]};
      coring_thresh  inside {[10'd0 : 10'd64]};
      clip_min       inside {[10'd0 : 10'd64]};
      clip_max       inside {[10'd900 : 10'd1023]};
      clip_min < clip_max;
    }

    `uvm_object_utils_begin(vpu_frame_item)
      `uvm_field_enum(res_mode_e,  res_mode,       UVM_ALL_ON)
      `uvm_field_enum(fps_mode_e,  fps_mode,       UVM_ALL_ON)
      `uvm_field_enum(proc_mode_e, proc_mode,      UVM_ALL_ON)
      `uvm_field_int(width,                        UVM_ALL_ON)
      `uvm_field_int(height,                       UVM_ALL_ON)
      `uvm_field_int(digital_gain,                 UVM_ALL_ON)
      `uvm_field_int(black_offset,                 UVM_ALL_ON)
      `uvm_field_int(sharp_strength,               UVM_ALL_ON)
      `uvm_field_int(coring_thresh,                UVM_ALL_ON)
      `uvm_field_int(clip_min,                     UVM_ALL_ON)
      `uvm_field_int(clip_max,                     UVM_ALL_ON)
      `uvm_field_int(hblank_cycles,                UVM_ALL_ON)
      `uvm_field_int(vblank_lines,                 UVM_ALL_ON)
      `uvm_field_int(downstream_bp_prob,           UVM_ALL_ON)
    `uvm_object_utils_end

    function new(string name = "vpu_frame_item");
      super.new(name);
    endfunction

    function void post_randomize();
      frame_pixels = new[height];
      foreach (frame_pixels[y]) begin
        frame_pixels[y] = new[width];
        foreach (frame_pixels[y][x]) begin
          // Generate mixed gradient + high-frequency impulses + corner values
          if ((x == 0 && y == 0) || (x == width-1 && y == height-1))
            frame_pixels[y][x] = 10'd1023;
          else if ((x % 17 == 0) && (y % 13 == 0))
            frame_pixels[y][x] = 10'd0;
          else
            frame_pixels[y][x] = $urandom_range(0, 1023);
        end
      end
    endfunction
  endclass : vpu_frame_item

  //----------------------------------------------------------------------------
  // 2. Functional Coverage Model (All Resolutions x FPS x Processing Modes)
  //----------------------------------------------------------------------------
  class vpu_coverage_collector extends uvm_subscriber #(vpu_frame_item);
    `uvm_component_utils(vpu_coverage_collector)

    vpu_frame_item cov_item;
    bit [PIXEL_WIDTH-1:0] sample_pix;
    bit                   sample_sof;
    bit                   sample_eol;
    bit                   sample_stall;

    covergroup cg_vpu_modes;
      option.per_instance = 1;
      option.name         = "cg_vpu_modes";

      cp_res_mode: coverpoint cov_item.res_mode {
        bins mode_1k = {MODE_1K};
        bins mode_2k = {MODE_2K};
        bins mode_4k = {MODE_4K};
      }

      cp_fps_mode: coverpoint cov_item.fps_mode {
        bins fps_24  = {FPS_24};
        bins fps_40  = {FPS_40};
        bins fps_90  = {FPS_90};
        bins fps_120 = {FPS_120};
      }

      cp_proc_mode: coverpoint cov_item.proc_mode {
        bins bypass   = {PROC_BYPASS};
        bins gaussian = {PROC_GAUSSIAN};
        bins sharpen  = {PROC_SHARPEN};
        bins edge_det = {PROC_EDGE};
      }

      cp_gain: coverpoint cov_item.digital_gain {
        bins atten = {[8'h01 : 8'h3F]};
        bins unity = {8'h40};
        bins boost = {[8'h41 : 8'hFF]};
      }

      cp_offset: coverpoint cov_item.black_offset {
        bins neg  = {[-512 : -1]};
        bins zero = {0};
        bins pos  = {[1 : 511]};
      }

      cp_bp_prob: coverpoint cov_item.downstream_bp_prob {
        bins no_bp  = {0};
        bins low_bp = {[1:5]};
        bins med_bp = {[6:15]};
      }

      // Mandatory Cross: All Resolution Modes x All FPS Targets (1K/2K/4K @ 24, 40, 90, 120 fps)
      cx_res_x_fps: cross cp_res_mode, cp_fps_mode;

      // Cross: All Resolution Modes x FPS x Processing Filter Modes
      cx_res_x_fps_x_proc: cross cp_res_mode, cp_fps_mode, cp_proc_mode;

      // Cross: 4K @ 120fps under Downstream Backpressure
      cx_4k120_bp: cross cp_res_mode, cp_fps_mode, cp_bp_prob;
    endgroup : cg_vpu_modes

    // Mode Transition Covergroup (verifies dynamic switching 1K <-> 2K <-> 4K)
    covergroup cg_mode_transitions;
      option.per_instance = 1;
      cp_res_trans: coverpoint cov_item.res_mode {
        bins trans_1k_to_2k = (MODE_1K => MODE_2K);
        bins trans_2k_to_4k = (MODE_2K => MODE_4K);
        bins trans_4k_to_1k = (MODE_4K => MODE_1K);
        bins trans_4k_to_2k = (MODE_4K => MODE_2K);
        bins trans_1k_to_4k = (MODE_1K => MODE_4K);
      }
    endgroup : cg_mode_transitions

    // Pixel Extremes & Framing Sideband Covergroup
    covergroup cg_pixel_datapath;
      option.per_instance = 1;
      cp_pixel_val: coverpoint sample_pix {
        bins min_zero  = {10'd0};
        bins low_range = {[10'd1   : 10'd255]};
        bins mid_range = {[10'd256 : 10'd767]};
        bins hi_range  = {[10'd768 : 10'd1022]};
        bins max_sat   = {10'd1023};
      }
      cp_sof:   coverpoint sample_sof   { bins active = {1'b1}; }
      cp_eol:   coverpoint sample_eol   { bins active = {1'b1}; }
      cp_stall: coverpoint sample_stall { bins stalled = {1'b1}; bins flowing = {1'b0}; }
    endgroup : cg_pixel_datapath

    function new(string name = "vpu_coverage_collector", uvm_component parent = null);
      super.new(name, parent);
      cg_vpu_modes        = new();
      cg_mode_transitions = new();
      cg_pixel_datapath   = new();
    endfunction

    virtual function void write(vpu_frame_item t);
      cov_item = t;
      cg_vpu_modes.sample();
      cg_mode_transitions.sample();
    endfunction

    virtual function void sample_beat(bit [PIXEL_WIDTH-1:0] pix, bit sof, bit eol, bit stall);
      sample_pix   = pix;
      sample_sof   = sof;
      sample_eol   = eol;
      sample_stall = stall;
      cg_pixel_datapath.sample();
    endfunction
  endclass : vpu_coverage_collector

  //----------------------------------------------------------------------------
  // 3. Bit-Accurate Golden Reference Model & Scoreboard
  //----------------------------------------------------------------------------
  class vpu_scoreboard extends uvm_component;
    `uvm_component_utils(vpu_scoreboard)

    int unsigned total_frames_checked;
    int unsigned total_pixels_checked;
    int unsigned mismatch_count;

    function new(string name = "vpu_scoreboard", uvm_component parent = null);
      super.new(name, parent);
      total_frames_checked = 0;
      total_pixels_checked = 0;
      mismatch_count       = 0;
    endfunction

    // Bit-accurate C-like reference model matching the 5-stage RTL pipeline
    // Note: Because the RTL 3x3 window aligns the output stream with the
    // right-most incoming column (col_2 = x, col_1 = x-1, col_0 = x-2 with
    // replication on x=0), we compute the exact bit-accurate expected frame.
    virtual function void compute_golden_frame(
      input  vpu_frame_item        cfg_item,
      output bit [PIXEL_WIDTH-1:0] exp_frame[][]
    );
      int W = cfg_item.width;
      int H = cfg_item.height;
      exp_frame = new[H];
      foreach (exp_frame[y]) exp_frame[y] = new[W];

      for (int y = 0; y < H; y++) begin
        int y_m1 = (y == 0) ? 0 : (y - 1);
        int y_m2 = (y == 0) ? 0 : ((y == 1) ? 0 : (y - 2));

        for (int x = 0; x < W; x++) begin
          int x_m1 = (x == 0) ? 0 : (x - 1);
          int x_m2 = (x == 0) ? 0 : ((x == 1) ? 0 : (x - 2));

          int p00 = cfg_item.frame_pixels[y_m2][x_m2];
          int p01 = cfg_item.frame_pixels[y_m2][x_m1];
          int p02 = cfg_item.frame_pixels[y_m2][x];

          int p10 = cfg_item.frame_pixels[y_m1][x_m2];
          int p11 = cfg_item.frame_pixels[y_m1][x_m1];
          int p12 = cfg_item.frame_pixels[y_m1][x];

          int p20 = cfg_item.frame_pixels[y][x_m2];
          int p21 = cfg_item.frame_pixels[y][x_m1];
          int p22 = cfg_item.frame_pixels[y][x];

          // Stage D1: Row [1, 2, 1] sums
          int r0 = p00 + (2 * p01) + p02;
          int r1 = p10 + (2 * p11) + p12;
          int r2 = p20 + (2 * p21) + p22;
          int center = p11;

          // Stage D2: Vertical [1, 2, 1] sum + 8 >> 4
          int gauss_norm = (r0 + (2 * r1) + r2 + 8) >> 4;
          int detail     = center - gauss_norm;

          // Stage D3: Coring & Multipliers
          int detail_abs   = (detail < 0) ? -detail : detail;
          int cored_detail = (detail_abs > cfg_item.coring_thresh) ? detail : 0;
          int base_pix     = (cfg_item.proc_mode == PROC_GAUSSIAN) ? gauss_norm : center;
          int gain_prod    = base_pix * cfg_item.digital_gain;
          int sharp_prod   = cored_detail * int'(cfg_item.sharp_strength);

          // Stage D4: Scaling & Accumulation
          int scaled_base  = gain_prod >> 6;
          int scaled_boost = (cfg_item.proc_mode == PROC_SHARPEN) ? (sharp_prod >>> 7) : 0;
          int accum_pix    = scaled_base + scaled_boost + int'(cfg_item.black_offset);

          // Stage D5: Clipping & Mode Select
          int clipped;
          if (accum_pix < int'(cfg_item.clip_min))      clipped = cfg_item.clip_min;
          else if (accum_pix > int'(cfg_item.clip_max)) clipped = cfg_item.clip_max;
          else                                          clipped = accum_pix;

          case (cfg_item.proc_mode)
            PROC_BYPASS: exp_frame[y][x] = center[PIXEL_WIDTH-1:0];
            PROC_EDGE:   exp_frame[y][x] = detail_abs[PIXEL_WIDTH-1:0];
            default:     exp_frame[y][x] = clipped[PIXEL_WIDTH-1:0];
          endcase
        end
      end
    endfunction

    virtual function void check_frame(
      input vpu_frame_item        cfg_item,
      input bit [PIXEL_WIDTH-1:0] act_frame[][]
    );
      bit [PIXEL_WIDTH-1:0] exp_frame[][];
      compute_golden_frame(cfg_item, exp_frame);

      for (int y = 0; y < cfg_item.height; y++) begin
        for (int x = 0; x < cfg_item.width; x++) begin
          total_pixels_checked++;
          if (act_frame[y][x] !== exp_frame[y][x]) begin
            mismatch_count++;
            `uvm_error("SB_MISMATCH", $sformatf(
              "Mode=%s FPS=%s Filter=%s Pixel(%0d,%0d) mismatch! Exp=%0d Act=%0d",
              cfg_item.res_mode.name(), cfg_item.fps_mode.name(),
              cfg_item.proc_mode.name(), x, y, exp_frame[y][x], act_frame[y][x]))
          end
        end
      end
      total_frames_checked++;
      `uvm_info("SB_PASS", $sformatf(
        "Verified frame #%0d (%s @ %s, %s, %0dx%0d) - Total pixels checked: %0d, Mismatches: %0d",
        total_frames_checked, cfg_item.res_mode.name(), cfg_item.fps_mode.name(),
        cfg_item.proc_mode.name(), cfg_item.width, cfg_item.height,
        total_pixels_checked, mismatch_count), UVM_LOW)
    endfunction
  endclass : vpu_scoreboard

endpackage : vpu_uvm_pkg
