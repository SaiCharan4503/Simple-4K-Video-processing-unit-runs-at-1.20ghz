//------------------------------------------------------------------------------
// File       : vpu_pkg.sv
// Author     : Senior RTL Design & Verification Lead
// Description: Package definitions, parameters, types, and CSR map for the
//              2 GHz 5nm TSMC 4K Video Processing Unit (VPU).
//------------------------------------------------------------------------------

package vpu_pkg;

  //----------------------------------------------------------------------------
  // Global Design Parameters
  //----------------------------------------------------------------------------
  parameter int unsigned PIXEL_WIDTH    = 10;   // 10-bit RAW / Grayscale pixel depth
  parameter int unsigned PPC            = 1;    // 1:1 Sensor Interface (1 Pixel Per Clock)
  parameter int unsigned MAX_LINE_WIDTH = 4096; // Supports up to DCI 4K (4096) & UHD 4K (3840)
  parameter int unsigned MAX_FRAME_H    = 2160; // Supports up to 2160 lines
  parameter int unsigned ADDR_WIDTH     = $clog2(MAX_LINE_WIDTH); // 12-bit line RAM address
  parameter int unsigned APB_ADDR_WIDTH = 12;
  parameter int unsigned APB_DATA_WIDTH = 32;

  //----------------------------------------------------------------------------
  // Resolution Mode Encoding
  //----------------------------------------------------------------------------
  typedef enum logic [1:0] {
    MODE_1K   = 2'b00, // 1024 x 768  (or 1280 x 720 programmable)
    MODE_2K   = 2'b01, // 1920 x 1080 (FHD / 2K)
    MODE_4K   = 2'b10, // 3840 x 2160 (UHD 4K)
    MODE_RSVD = 2'b11  // Reserved (defaults to 4K dimensions)
  } res_mode_e;

  //----------------------------------------------------------------------------
  // Frame Rate Target Encoding (Informational / CSR Telemetry & Verification)
  //----------------------------------------------------------------------------
  typedef enum logic [1:0] {
    FPS_24  = 2'b00,
    FPS_40  = 2'b01,
    FPS_90  = 2'b10,
    FPS_120 = 2'b11
  } fps_mode_e;

  //----------------------------------------------------------------------------
  // Processing Pipeline Bypass / Filter Mode Encoding
  //----------------------------------------------------------------------------
  typedef enum logic [1:0] {
    PROC_BYPASS   = 2'b00, // 1:1 Latency-matched bypass
    PROC_GAUSSIAN = 2'b01, // 3x3 [1 2 1; 2 4 2; 1 2 1]/16 Denoise only
    PROC_SHARPEN  = 2'b10, // 3x3 Gaussian + Unsharp Mask / Laplacian Boost
    PROC_EDGE     = 2'b11  // 3x3 Laplacian Edge Magnitude only
  } proc_mode_e;

  //----------------------------------------------------------------------------
  // Pixel Stream Sideband Structure
  //----------------------------------------------------------------------------
  typedef struct packed {
    logic                   sof;   // Start of Frame (asserted with first pixel of frame)
    logic                   eol;   // End of Line (asserted with last pixel of line)
    logic [PIXEL_WIDTH-1:0] pixel; // 10-bit pixel payload
  } vpu_stream_t;

  //----------------------------------------------------------------------------
  // CSR Register Map Offsets (Byte Addresses)
  //----------------------------------------------------------------------------
  localparam logic [APB_ADDR_WIDTH-1:0] CSR_CTRL        = 12'h000; // [0]: enable, [2:1]: res_mode, [4:3]: proc_mode, [6:5]: fps_mode, [7]: soft_clear
  localparam logic [APB_ADDR_WIDTH-1:0] CSR_ACT_DIM     = 12'h004; // [12:0]: active_width, [28:16]: active_height
  localparam logic [APB_ADDR_WIDTH-1:0] CSR_GAIN_OFFSET = 12'h008; // [9:0]: black_offset (signed 10b), [23:16]: digital_gain (Q2.6, 0x40 = 1.0x)
  localparam logic [APB_ADDR_WIDTH-1:0] CSR_SHARP_CFG   = 12'h00C; // [7:0]: sharp_strength (Q1.7), [19:8]: coring_thresh (10b)
  localparam logic [APB_ADDR_WIDTH-1:0] CSR_CLIP_CFG    = 12'h010; // [9:0]: clip_min, [25:16]: clip_max
  localparam logic [APB_ADDR_WIDTH-1:0] CSR_STATUS      = 12'h014; // RO: [0]: busy, [1]: fifo_overflow_sticky, [2]: fifo_underflow_sticky, [3]: frame_done_sticky, [4]: dim_err_sticky
  localparam logic [APB_ADDR_WIDTH-1:0] CSR_FRAME_CNT   = 12'h018; // RO: [31:0]: completed frames counter
  localparam logic [APB_ADDR_WIDTH-1:0] CSR_LINE_CNT    = 12'h01C; // RO: [15:0]: current line index, [31:16]: last frame total lines

  //----------------------------------------------------------------------------
  // Configuration Struct (Core Clock Domain)
  //----------------------------------------------------------------------------
  typedef struct packed {
    logic                   enable;
    res_mode_e              res_mode;
    proc_mode_e             proc_mode;
    fps_mode_e              fps_mode;
    logic                   soft_clear;
    logic [12:0]            active_width;
    logic [12:0]            active_height;
    logic [7:0]             digital_gain;   // UQ2.6 (64 = 1.0x)
    logic signed [9:0]      black_offset;   // Signed 10-bit offset (-512 to +511)
    logic [7:0]             sharp_strength; // UQ1.7 (128 = 1.0x)
    logic [PIXEL_WIDTH-1:0] coring_thresh;  // Noise floor threshold for sharpening
    logic [PIXEL_WIDTH-1:0] clip_min;
    logic [PIXEL_WIDTH-1:0] clip_max;
  } vpu_cfg_t;

  //----------------------------------------------------------------------------
  // Status Struct (Core Clock Domain -> CSR)
  //----------------------------------------------------------------------------
  typedef struct packed {
    logic        busy;
    logic        fifo_overflow;
    logic        fifo_underflow;
    logic        frame_done;
    logic        dim_err;
    logic [31:0] frame_count;
    logic [15:0] curr_line_count;
    logic [15:0] last_line_count;
  } vpu_status_t;

endpackage : vpu_pkg
