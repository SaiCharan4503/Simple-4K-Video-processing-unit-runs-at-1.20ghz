# 4K Video Processing Unit (VPU) — Production RTL Design & Verification Specification

**Target Process:** TSMC 5nm FinFET (`N5` / `N5P`)  
**Target Operating Frequency:** `2.0 GHz` (`core_clk`, $T_{\text{clk}} = 500\text{ ps}$)  
**Supported Resolutions & Frame Rates:**
- **4K UHD (`3840 × 2160`):** `24 fps`, `40 fps`, `90 fps`, `120 fps` (Line buffers sized to `4096` pixels to also accommodate DCI 4K `4096 × 2160`)
- **2K / FHD (`1920 × 1080`):** Up to `120 fps` (and higher)
- **1K (`1024 × 768` / `1280 × 720`):** Up to `120 fps` (and higher)
- **Sensor Interface:** `1:1` Direct Streaming (`1 Pixel Per Clock` without decimation or resampling)

---

## 0. Explicit Architectural Assumptions & Design Defaults

Because certain system-level parameters were left open in the project prompt, the following production-grade defaults have been chosen and enforced across all RTL, verification, and timing files:

| Parameter | Chosen Default | Engineering Justification |
| :--- | :--- | :--- |
| **Pixel Bit Depth (`PIXEL_WIDTH`)** | **10-bit Unsigned (`[9:0]`, range `0..1023`)** | Industry-standard bit depth for CSI-2 RAW10 / 10-bit luma video sensors (`4K` broadcast & cinema pipelines). Parameterizable in `vpu_pkg.sv` (`10`/`12`/`16`-bit). |
| **Color / Pixel Format** | **Monochrome / Luma (`Y10`) or Bayer-Domain Single-Channel (`10b`)** | Preserves a clean, self-contained VLSI scope (`10-bit` per pixel) without expanding into multi-plane 4:4:4 RGB or 5×5 demosaicing SoC blocks. |
| **Sensor Input Protocol** | **1:1 Parallel / Unpacked CSI-2 Pixel Stream (`s_vsync`, `s_hsync`, `s_valid`, `s_pixel[9:0]`)** | Directly follows a MIPI CSI-2 RX pixel unpacker (`1 pixel/clock`, `PPC = 1`) with active-high frame envelope (`s_vsync`), line envelope (`s_hsync`), and pixel qualifier (`s_valid`). |
| **Processed Output Protocol** | **AXI4-Stream Video (`m_axis_tdata[9:0]`, `tvalid`, `tready`, `tuser` [SOF], `tlast` [EOL])** | Industry-standard AMBA streaming video interface (Xilinx/ARM/Synopsys VIP compatible) supporting lossless downstream backpressure via a 2-entry skid buffer. |
| **Control & Configuration Bus** | **AMBA APB3 Slave (`pclk` @ `200 MHz`, `12-bit` addr, `32-bit` data)** | Lightweight, low-area register interface ideal for academic and production ASIC blocks. |
| **Processing Pipeline Function** | **Programmable 3×3 Spatial Convolution + Unsharp Mask + Gain/Offset + Clipping** | Supports 4 runtime modes (`PROC_BYPASS`, `PROC_GAUSSIAN`, `PROC_SHARPEN`, `PROC_EDGE`) exercising 2D line buffers, adder trees, multipliers, and saturation logic at `2.0 GHz`. |

---

## 1. Top-Level Block Diagram & Major Block Descriptions

```
                         +---------------------------------------------------------------------------------------------------+
                         |                                      vpu_top (5nm TSMC)                                           |
                         |                                                                                                   |
  sensor_clk Domain      |                     core_clk Domain (2.0 GHz, T = 500 ps, 1 Pixel/Clock)                          |
 (Up to 1.20 GHz)        |                                                                                                   |
                         |  +------------------+     +-----------------------+     +--------------------+     +-----------+  |
 s_vsync --------------->|  |  vpu_sensor_if   |     |    vpu_line_buffer    |     |    vpu_datapath    |     |vpu_out_if |  |
 s_hsync --------------->|  |  --------------  |     |  -------------------  |     |  ----------------  |     |---------  |  |
 s_valid --------------->|  | - SOF/EOL Detect |     | - 2x 4096x10b Line    |     |  5-Stage Pipeline: |     | 2-Entry   |  |--> m_axis_tvalid
 s_pixel[9:0] ---------->|  | - 16-Deep Dual-  |====>|   SRAMs (y-1, y-2)    |====>|  D1: Horiz [1 2 1] |====>| Carloni   |--|--> m_axis_tdata[9:0]
                         |  |   Clock Gray     |12b  | - SRAM Retime Reg     |9x10b|  D2: Vert + Detail |12b  | Skid      |  |--> m_axis_tuser (SOF)
 sensor_arst_n ---+      |  |   Async FIFO     |     | - 3x3 Shift Window &  |Win  |  D3: Core & Mult   |     | Buffer    |  |--> m_axis_tlast (EOL)
                  |      |  | - Out Retime Reg |     |   Border Replication  |     |  D4: Offset & Sum  |     |           |<-|--- m_axis_tready
                  v      |  +------------------+     +-----------------------+     |  D5: Clip & Mux    |     +-----------+  |
            +----------+ |           ^                           ^                 +--------------------+           ^        |
            |Rst Sync  | |           |fifo_ovf/udf               |frame_start/done           ^                      |        |
            +----------+ |           +--------------------+      |                           |active_cfg            |        |
                         |                                |      |      +--------------------+----------------------+        |
  pclk Domain (200 MHz)  |                                v      v      |                                                    |
                         |                           +-----------------------------------+                                   |
 paddr/psel/penable ---->|                           |             vpu_csr               |-----------------------------------|--> irq_out
 pwrite/pwdata --------->|                           |  -------------------------------  |                                   |
 prdata/pready <---------|                           | - APB3 Slave Register File        |                                   |
                         |                           | - Frame-Synchronized Shadow Regs  |                                   |
                         |                           | - Status & Frame/Line Telemetry   |                                   |
                         +---------------------------+-----------------------------------+-----------------------------------+
```

### Major Functional Blocks
1. **Sensor Interface & Input Buffer (`rtl/vpu_sensor_if.sv` & `rtl/vpu_async_fifo.sv`):**
   - Operates on `sensor_clk` at the front-end and `core_clk` (`2 GHz`) at the back-end.
   - Stages incoming pixels by 1 cycle in `sensor_clk` to detect **Start-of-Frame (`SOF`)** on the first valid pixel after `s_vsync` rises and **End-of-Line (`EOL`)** via lookahead when `s_hsync && s_valid` deasserts.
   - Packs `{sof, eol, pixel[9:0]}` (`12 bits`) into a **16-deep dual-clock Gray-coded Asynchronous FIFO** (`vpu_async_fifo`) and registers the read output in `core_clk` to isolate FIFO mux paths from the `2 GHz` line buffer.
2. **Line Memory & 3×3 Window Generator (`rtl/vpu_line_buffer.sv`):**
   - Instantiates two `4096 × 10-bit` line buffers storing previous rows `y-1` and `y-2`, combined with incoming row `y` to form 3 vertical taps.
   - Includes a dedicated **SRAM output retiming pipeline stage (`L1 -> L2`)** so the `~180 ps` $T_{\text{ck-q}}$ of the 5nm SRAM macros does not add to the downstream arithmetic combinational path.
   - Feeds a `3 × 3` horizontal shift-register array (`win_p[0:2][0:2]`) with automatic **top/bottom/left/right border replication** at frame boundaries and programmable horizontal wrap (`active_width`) for seamless `1K`, `2K`, and `4K` mode switching.
3. **5-Stage 2 GHz Processing Pipeline (`rtl/vpu_datapath.sv`):**
   - Deeply pipelined across 5 registered stages (`D1`–`D5`) with $\le 8\text{ FO4}$ logic levels per stage ($< 300\text{ ps}$ logic delay per stage in 5nm FinFET):
     - **Stage D1:** Three parallel horizontal `[1, 2, 1]` row adders (`12-bit` results) + center pixel delay.
     - **Stage D2:** Vertical `[1, 2, 1]` sum with rounding (`+8 >> 4`) to produce the normalized `10-bit` Gaussian pixel (`gauss_norm`), plus an `11-bit` signed subtractor extracting high-frequency spatial detail (`detail = center - gauss_norm`).
     - **Stage D3:** Noise coring comparator (`|detail| > coring_thresh`) and two isolated parallel hardware multipliers (`10b × 8b` UQ2.6 Digital Gain and `11b × 8b` UQ1.7 Sharpening Strength).
     - **Stage D4:** Fixed-point shift scaling, signed `10-bit` black-level offset addition, and unsharp mask detail accumulation (`14-bit` signed sum).
     - **Stage D5:** Programmable saturation clipping (`[clip_min, clip_max]`) and mode multiplexer (`PROC_BYPASS`, `PROC_GAUSSIAN`, `PROC_SHARPEN`, `PROC_EDGE`).
4. **Output Formatter (`rtl/vpu_out_if.sv`):**
   - Implements a **2-entry Carloni / Skid Buffer** bridging the internal pipeline to the external `AXI4-Stream Video` master port (`m_axis_*`).
   - Ensures all output ports (`m_axis_tvalid`, `m_axis_tdata`, `m_axis_tuser`, `m_axis_tlast`) are driven directly by flip-flops and breaks any combinational loop from `m_axis_tready`.
5. **Control & Configuration Registers (`rtl/vpu_csr.sv`):**
   - APB3 slave interface (`pclk`) with **frame-synchronized shadow registers** in `core_clk`. Configuration updates written by software take effect only when the pipeline is idle (`!busy`) or on the exact `frame_start_pulse` (`SOF`) boundary, preventing mid-frame tearing during `1K`/`2K`/`4K` mode switches.

---

## 2. Throughput Analysis & Pixels-Per-Clock (PPC) Justification

### Explicit Pixels-Per-Clock (PPC) Assumption
- **Chosen Architecture:** **`PPC = 1` (`1 Pixel Per Clock` in the `2.0 GHz` `core_clk` domain, `10-bit` datapath width).**
- **Justification:**
  1. **Direct 1:1 Sensor Mapping:** The specification requires a `1:1` sensor interface where a single pixel stream maps directly to the processing pipeline without decimation, resampling, or multi-pixel lane packing/unpacking overhead.
  2. **2.0 GHz Peak Throughput:** At `2.0 GHz` and `PPC = 1`, the core datapath sustains a peak processing rate of **`2,000.0 MPixels/sec` (`2.0 GPixels/sec`)**.
  3. **Worst-Case 4K @ 120 fps Margin:** Even at the highest resolution and frame rate (`3840 × 2160 @ 120 fps`) with standard CEA-861 / CVT-RBv2 blanking (~`15%–20%` overhead), the required peak pixel rate is **`1,146.6 MPixels/sec`**, leaving **`42.7%` positive throughput headroom** (`853.4 MPixels/sec` margin) at `PPC = 1`. Widening to `2 PPC` or `4 PPC` would nearly double or quadruple datapath and line-buffer port area unnecessarily.

### Detailed Mode & Frame-Rate Throughput Table

| Mode | Active Resolution ($W \times H$) | Frame Rate (fps) | Active Pixel Rate ($W \times H \times \text{fps}$) | Total Frame Size w/ Blanking ($W_{\text{tot}} \times H_{\text{tot}}$) | Required Peak Sensor Rate w/ Blanking | Core Capacity @ 2 GHz (`1 PPC`) | Core Utilization (%) | Headroom & Timing Verdict |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **1K** | `1024 × 768` | `24 fps` | `18.87 MPix/s` | `1200 × 800` | `23.04 MPix/s` | `2000.0 MPix/s` | `1.15%` | **PASS** (+98.8% margin) |
| **1K** | `1024 × 768` | `60 fps` | `47.19 MPix/s` | `1200 × 800` | `57.60 MPix/s` | `2000.0 MPix/s` | `2.88%` | **PASS** (+97.1% margin) |
| **1K** | `1024 × 768` | `120 fps` | `94.37 MPix/s` | `1200 × 800` | `115.20 MPix/s` | `2000.0 MPix/s` | `5.76%` | **PASS** (+94.2% margin) |
| **2K (FHD)** | `1920 × 1080` | `24 fps` | `49.77 MPix/s` | `2200 × 1125` | `59.40 MPix/s` | `2000.0 MPix/s` | `2.97%` | **PASS** (+97.0% margin) |
| **2K (FHD)** | `1920 × 1080` | `60 fps` | `124.42 MPix/s` | `2200 × 1125` | `148.50 MPix/s` | `2000.0 MPix/s` | `7.43%` | **PASS** (+92.6% margin) |
| **2K (FHD)** | `1920 × 1080` | `120 fps` | `248.83 MPix/s` | `2200 × 1125` | `297.00 MPix/s` | `2000.0 MPix/s` | `14.85%` | **PASS** (+85.2% margin) |
| **4K UHD** | `3840 × 2160` | **`24 fps`** | **`199.07 MPix/s`** | `4200 × 2275` | **`229.32 MPix/s`** | `2000.0 MPix/s` | **`11.47%`** | **PASS** (+88.5% margin) |
| **4K UHD** | `3840 × 2160` | **`40 fps`** | **`331.78 MPix/s`** | `4200 × 2275` | **`382.20 MPix/s`** | `2000.0 MPix/s` | **`19.11%`** | **PASS** (+80.9% margin) |
| **4K UHD** | `3840 × 2160` | **`90 fps`** | **`746.50 MPix/s`** | `4200 × 2275` | **`859.95 MPix/s`** | `2000.0 MPix/s` | **`43.00%`** | **PASS** (+57.0% margin) |
| **4K UHD** | `3840 × 2160` | **`120 fps`** | **`995.33 MPix/s`** | `4200 × 2275` | **`1146.60 MPix/s`** | `2000.0 MPix/s` | **`57.33%`** | **PASS** (+42.7% margin) |

- **Pipeline Latency:** Total deterministic latency from `vpu_line_buffer` input to `vpu_out_if` AXI4-Stream output is **`3` (Line Buffer stages) + `5` (Datapath stages) + `1` (Output register) = `9` `core_clk` cycles (`4.5 ns` at `2.0 GHz`)**.

---

## 3. Clock and Reset Strategy & CDC Architecture

### Clock Domains
1. **`sensor_clk` (`23 MHz` to `1.20 GHz`):** Asynchronous/mesochronous source pixel clock supplied by the sensor interface or CSI-2 RX pixel unpacker.
2. **`core_clk` (`2.0 GHz`, $T = 500\text{ ps}$):** Main VPU line buffer, spatial filter datapath, and AXI4-Stream video output clock.
3. **`pclk` (`200 MHz`, $T = 5.0\text{ ns}$):** APB3 control plane clock for CSR reads/writes.

### Reset Architecture
- Each clock domain has an **asynchronous-assert, synchronous-deassert reset synchronizer** (`2-stage` flip-flop chain with `(* ASYNC_REG = "TRUE" *)` attributes) inside `vpu_top.sv`.
- Includes an explicit **`scan_mode` bypass mux** on `sensor_rst_n` and `core_rst_n` so DFT ATPG scan chains can control resets directly during manufacturing test.

### Clock Domain Crossing (CDC) Table

| Crossing Path | Source -> Dest | Signal Type & Width | Synchronization Scheme | Mathematical / Sizing Justification |
| :--- | :--- | :--- | :--- | :--- |
| **Pixel Stream + Framing (`{sof, eol, pixel[9:0]}`)** | `sensor_clk` -> `core_clk` | `12-bit` Coherent Bus | **16-Deep Dual-Clock Gray-Coded Async FIFO (`vpu_async_fifo.sv`)** with 2-FF (`ASYNC_REG`) pointer synchronizers | Since $f_{\text{core}} (2.0\text{ GHz}) > f_{\text{sensor,max}} (1.2\text{ GHz})$, the read domain drains faster than the write domain fills. CDC round-trip pointer latency is $2 T_{\text{core}} + 2 T_{\text{sensor}} \approx 2.67\text{ ns}$ ($\sim 4$ sensor clocks). A depth of $16$ (`ADDR_WIDTH = 4`) absorbs pointer sync latency plus up to $12$ cycles of downstream AXI4-Stream backpressure without overflow. |
| **FIFO Overflow Event (`wr_overflow_pulse`)** | `sensor_clk` -> `core_clk` | `1-bit` Pulse | **Toggle Synchronizer + 3-FF Edge Detector** (`wr_ovf_toggle_s` -> `wr_ovf_sync_c[2:0]`) | Safely transfers fast `sensor_clk` pulse into `core_clk` sticky status register regardless of clock ratio. |
| **Global Enable & Soft Clear (`r_enable`, `r_soft_clear_toggle`)** | `pclk` -> `core_clk` | `1-bit` Level & `1-bit` Toggle | **3-FF Level Synchronizer & Toggle Edge Detector** (`enable_sync[2:0]`, `clear_sync[2:0]`) | Eliminates metastability at `2 GHz` (`3 stages` give $> 1.0\text{ ns}$ metastability resolution time in 5nm FinFET, achieving $\text{MTBF} > 10^{100}\text{ years}$). |
| **Static/Quasi-Static Config (`res_mode`, `active_width`, `gain`, etc.)** | `pclk` -> `core_clk` | Multi-bit Config Struct (`vpu_cfg_t`) | **Enable/Frame-Start Gated Shadow Registers** (`active_cfg` updated only when `!busy \|\| frame_start_pulse`) | Software programs APB registers prior to asserting `enable` or before the next frame's `SOF`. Sampling only on `!busy` or `frame_start_pulse` prevents multi-bit bus skew and mid-frame tearing. |
| **Status & Telemetry (`busy`, `fifo_overflow`, `frame_count`)** | `core_clk` -> `pclk` | Sticky Status Flags & Monotonic Counters | **2-FF Synchronizer (`ASYNC_REG`)** | Sticky flags remain high until cleared by `soft_clear`; frame count increments once per frame ($\ge 8.33\text{ ms}$ period), so it is quasi-static during APB reads. |

---

## 4. Memory Strategy: Line Buffers vs. Frame Buffers & Mode Switching

### Why Line Buffers Instead of Frame Buffers?
- A full `4K` (`3840 × 2160 × 10-bit`) **frame buffer** requires **`82.94 Mbits` (`10.37 MB`)** per frame, necessitating off-chip LPDDR5/DRAM controllers, multi-channel AXI DMA engines, and tens of megabytes of die area.
- Because the sensor interface is `1:1` streaming and our spatial neighborhood is `3 × 3`, **only 2 previous lines (`y-1` and `y-2`)** need to be stored on-chip while the 3rd line (`y`) streams in live.
- **4K Line Buffer Sizing:**
  - Maximum line width: `MAX_LINE_WIDTH = 4096` pixels (supports both `3840` UHD 4K and `4096` DCI 4K with power-of-2 addressing `ADDR_WIDTH = 12`).
  - Number of line buffers: `2` (`line_ram_m1` for `y-1`, `line_ram_m2` for `y-2`).
  - Total on-chip SRAM capacity:
    $$\text{Total Bits} = 2 \times 4096 \times 10\text{ bits} = 81,920\text{ bits} = 10.0\text{ KiB}$$
  - This represents a **`> 1,000×` memory reduction** compared to a frame buffer (`81.92 Kbits` vs. `82.94 Mbits`) and fits easily in two tiny 5nm TSMC compiled SRAM macros.
- **5nm Physical SRAM Banking for 2.0 GHz Closure:**
  - Standard single-port (`1RW`) high-speed 5nm SRAM compilers support `2.0 GHz` operation when configured as either:
    1. **Two Ping-Pong 1RW Odd/Even Banks (`2048 × 20-bit` each):** Even cycles read 2 pixels (`20b`), odd cycles write 2 pixels (`20b`), achieving `1 pixel/clock` read + write bandwidth using purely single-port `1RW` bitcells (saving ~35% area over dual-port `1R1W` bitcells), or
    2. **Ultra-High-Speed 1R1W Register-File / Pseudo-Dual-Port Macros (`4096 × 10-bit`)** with a dedicated **SRAM Output Retiming Pipeline Register (`Stage L1 -> L2` in `vpu_line_buffer.sv`)**.

### Dynamic Mode Switching Between 1K, 2K, and 4K
1. **Dynamic Column Pointer Modulo Wrap (`x_cnt`):**
   - Rather than hardcoding the SRAM address wrap at `3839` or `4095`, `vpu_line_buffer.sv` compares `x_cnt` against the shadowed `cfg.active_width - 1`:
     - **`MODE_1K`:** `x_cnt` wraps at `1023` (`1024` pixels/line); addresses `[1024:4095]` of the line SRAM are unused (and can be bank-clock-gated in physical implementation to save dynamic power).
     - **`MODE_2K`:** `x_cnt` wraps at `1919` (`1920` pixels/line).
     - **`MODE_4K`:** `x_cnt` wraps at `3839` (`3840` pixels/line).
2. **Glitch-Free Frame-Boundary Switching:**
   - When software writes a new `res_mode` (`MODE_1K`, `MODE_2K`, or `MODE_4K`) to `CSR_CTRL`, `vpu_csr.sv` holds the new dimensions in staging registers (`r_res_mode`, `r_act_width`, `r_act_height`) and transfers them into `active_cfg` **only** when the current frame completes (`!busy`) or on the `frame_start_pulse` (`SOF`) of the incoming frame.
   - Simultaneously, `s_stream.sof` resets `x_cnt <= 1`, `y_cnt <= 0`, and re-initializes top/left border replication for the first line of the new resolution without requiring a hardware reset or flushing valid downstream pixels.

---

## 5. RTL Module Hierarchy & Synthesizable SystemVerilog Implementation

### Module Hierarchy
```
vpu_top                       (rtl/vpu_top.sv)
 ├── vpu_sensor_if            (rtl/vpu_sensor_if.sv)
 │    └── vpu_async_fifo      (rtl/vpu_async_fifo.sv)
 ├── vpu_line_buffer          (rtl/vpu_line_buffer.sv)
 ├── vpu_datapath             (rtl/vpu_datapath.sv)
 ├── vpu_out_if               (rtl/vpu_out_if.sv)
 └── vpu_csr                  (rtl/vpu_csr.sv)
```

### Summary of Port Lists
- **`vpu_top` (`rtl/vpu_top.sv`):**
  - **Clocks & Resets:** `sensor_clk`, `sensor_arst_n`, `core_clk` (`2 GHz`), `core_arst_n`, `pclk` (`200 MHz`), `preset_n`, `scan_mode`
  - **1:1 Sensor Input (`sensor_clk`):** `s_vsync`, `s_hsync`, `s_valid`, `s_pixel[9:0]`
  - **AXI4-Stream Video Output (`core_clk`):** `m_axis_tvalid`, `m_axis_tready`, `m_axis_tdata[9:0]`, `m_axis_tuser` (`SOF`), `m_axis_tlast` (`EOL`)
  - **APB3 Slave Control (`pclk`):** `paddr[11:0]`, `psel`, `penable`, `pwrite`, `pwdata[31:0]`, `prdata[31:0]`, `pready`, `pslverr`, `irq_out`

---

## 6. Verification Plan: UVM Architecture, Scenarios, Coverage & Assertions

### UVM Testbench Architecture (`tb/vpu_uvm_pkg.sv`, `tb/tb_vpu_top.sv`, `tb/vpu_sva_bind.sv`)
```
+-----------------------------------------------------------------------------------+
|                                vpu_uvm_env / tb_vpu_top                           |
|                                                                                   |
|  +----------------------+    +-----------------------+    +--------------------+  |
|  |   Sensor Active      |    |     APB3 Master       |    | AXI4-Stream Video  |  |
|  |   Agent (Driver +    |    |     Config Agent      |    | Slave Agent        |  |
|  |   Rate Generator)    |    |  (CSR Read/Write/     |    | (Backpressure      |  |
|  |  24/40/90/120 fps    |    |   Shadow Mode Switch) |    |  Driver + Monitor) |  |
|  +----------+-----------+    +-----------+-----------+    +---------+----------+  |
|             |                            |                          |             |
|             v                            v                          v             |
|  +-----------------------------------------------------------------------------+  |
|  |                        vpu_scoreboard & Golden C/SV Model                   |  |
|  |  - Bit-accurate 3x3 Gaussian, Unsharp Mask, Coring, Gain/Offset, Clipping   |  |
|  |  - Exact SOF (tuser) and EOL (tlast) coordinate checker                     |  |
|  +-----------------------------------------------------------------------------+  |
|  +-----------------------------------------------------------------------------+  |
|  |                        vpu_coverage_collector & vpu_sva_checker             |  |
|  |  - Crosses: {1K, 2K, 4K} x {24, 40, 90, 120 fps} x {4 Filter Modes} x BP    |  |
|  |  - Mode transition bins: 1K -> 2K -> 4K -> 1K                               |  |
|  +-----------------------------------------------------------------------------+  |
+-----------------------------------------------------------------------------------+
```

### Directed & Constrained-Random Test Scenarios per Mode
1. **`test_4k_fps_sweep` (Directed & Random IQ):** Runs `4K` frames at `24 fps` (`240 MHz`), `40 fps` (`400 MHz`), `90 fps` (`900 MHz`), and `120 fps` (`1.20 GHz`) across all 4 processing modes (`PROC_BYPASS`, `PROC_GAUSSIAN`, `PROC_SHARPEN`, `PROC_EDGE`), verifying zero FIFO overflow/underflow and bit-exact pixel match.
2. **`test_dynamic_mode_switch` (`1K <-> 2K <-> 4K`):** Reprograms `CSR_CTRL.res_mode` and `CSR_ACT_DIM` while a frame is actively streaming to verify that `vpu_csr` holds the shadow update until the next `frame_start_pulse` (`SOF`) and switches cleanly without line-buffer address corruption.
3. **`test_backpressure_stress`:** Applies random downstream stalls (`m_axis_tready` deasserted `1%–15%` of cycles) at `4K @ 120 fps` to verify the `vpu_out_if` Carloni skid buffer and `16-deep` async FIFO backpressure propagation.
4. **`test_fifo_overflow_error_injection`:** Holds `m_axis_tready = 0` for `> 32` sensor clock cycles during active `4K @ 120 fps` streaming to intentionally trigger `fifo_overflow`, verifying sticky `CSR_STATUS[1]` and `irq_out` assertion plus `soft_clear` recovery.
5. **`test_dimension_mismatch_error`:** Sends an early/late `s_hsync` drop (`EOL` at `x != active_width - 1`) to verify `dim_err` detection (`CSR_STATUS[4]`).

### SystemVerilog Assertions (`tb/vpu_sva_bind.sv`)
- `AST_AXIS_VALID_STABLE` & `AST_AXIS_PAYLOAD_STABLE`: Verifies AXI4-Stream handshake stability under stall (`m_axis_tvalid && !m_axis_tready |=> m_axis_tvalid && $stable(payload)`).
- `AST_AXIS_SOF_EOL_MUTEX`: Ensures `m_axis_tuser` (`SOF`) and `m_axis_tlast` (`EOL`) never assert on the same pixel beat.
- `AST_WR_GRAY_ONE_BIT` & `AST_RD_GRAY_ONE_BIT`: Verifies Gray-coded CDC pointers change by at most `1 bit` per clock edge (`$countones(gray ^ $past(gray)) <= 1`).
- `AST_LB_COL_BOUNDS`: Verifies line buffer column address `x_cnt < 4096` at all times.
- `AST_CFG_STABLE_MID_FRAME`: Verifies `active_cfg` dimensions and resolution mode never change while `in_frame == 1`.
- `AST_OUTPUT_CLIP_BOUNDS`: Verifies output pixels strictly respect `[clip_min, clip_max]`.

---

## 7. Synthesis & Static Timing Analysis (STA) Readiness for 5nm TSMC @ 2 GHz

### 2.0 GHz Clock Period Timing Budget ($T_{\text{clk}} = 500.0\text{ ps}$)
In TSMC 5nm FinFET (`N5`, `0.75V` nominal LVT/ULVT standard cells, $1\text{ FO4} \approx 8\text{–}10\text{ ps}$), closing timing at `2.0 GHz` requires keeping every register-to-register path under `8–10` logic levels:

| Timing Budget Component | Allocation (ps) | Notes for 5nm TSMC Signoff (`SSGNP 0.675V -40C / 125C`) |
| :--- | :--- | :--- |
| **Clock Period ($T_{\text{clk}}$)** | **`500.0 ps`** | `2.0 GHz` target |
| **PLL Jitter + Clock Tree Skew + POCV (`set_clock_uncertainty -setup`)** | `-35.0 ps` | Configured in `constraints/vpu_top.sdc` |
| **Flip-Flop Clock-to-Q ($T_{\text{ck-q}}$)** | `-35.0 ps` | 5nm ULVT/LVT D-flip-flop |
| **Flip-Flop Library Setup Time ($T_{\text{setup}}$)** | `-25.0 ps` | 5nm ULVT/LVT D-flip-flop |
| **Maximum Available Combinational Datapath + Wire Delay** | **`405.0 ps`** | Supports **$\le 8\text{ FO4}$ logic levels** + local routing |

### Flagged Critical Paths & Built-In RTL Mitigations
1. **Critical Path #1 — Line Buffer SRAM Read-to-Logic Path (`u_line_buffer`):**
   - **Risk:** Compiled `4096 × 10b` SRAM macros in 5nm exhibit $T_{\text{ck-q}} \approx 170\text{–}210\text{ ps}$, leaving only `~195 ps` before the next clock edge. Feeding SRAM outputs directly into 3×3 spatial adders would fail setup by `> 80 ps`.
   - **RTL Mitigation Implemented:** `vpu_line_buffer.sv` places a dedicated **SRAM Output Retiming Stage (`Stage L1 -> L2`: `sram_rdata_m1_l1` -> `row1_l2`)** with only a 2:1 vertical boundary mux between the SRAM output register and `row0_l2`/`row1_l2`, plus a separate horizontal shift-register stage (`L2 -> L3`) before `vpu_datapath`.
2. **Critical Path #2 — Sharpening & Digital Gain Multipliers (`u_datapath` Stage D3):**
   - **Risk:** Combining a `3×3` adder tree, high-pass subtraction, and an `11b × 8b` multiplier in a single cycle exceeds `480 ps` in worst-case corner.
   - **RTL Mitigation Implemented:** `vpu_datapath.sv` splits the math across 5 dedicated stages (`D1`: row adders, `D2`: vertical sum + subtraction, `D3`: coring + isolated multipliers `gain_prod_d3` and `sharp_prod_d3`, `D4`: post-multiplier shift + offset accumulation, `D5`: saturation clip + mux). Synthesis tools (`Design Compiler NXT` / `Genus`) can map `Stage D3` to Booth-encoded Wallace trees with zero preceding adder delay.
3. **Critical Path #3 — Ready/Backpressure Fanout Cone (`m_axis_tready`):**
   - **Risk:** Direct combinational propagation of external `m_axis_tready` across 8 pipeline stages and 4096-word SRAM chip-enables creates high-fanout RC wire delay (`> 250 ps` in 5nm lower metal layers `M1–M3`).
   - **RTL Mitigation Implemented:** `vpu_out_if.sv` implements a **2-entry Carloni Skid Buffer** where `s_ready = !skid_valid_reg` is driven **purely by a flip-flop** with zero combinational path from the top-level `m_axis_tready` pin.

### Physical Design & Synthesis Guidelines (Design Compiler / Fusion Compiler / Innovus)
- **Retiming & Register Placement:** Enable `set_optimize_registers true` on `vpu_datapath` and place `u_line_buffer` SRAM macros along the left/top core boundary with a `5 µm` halo and dedicated `M4/M5` routing tracks for `win_p[0:2][0:2]`.
- **Multi-Vt Cell Policy:** Restrict `ULVT` cells to `u_datapath` (`Stage D3` multipliers) and `u_line_buffer` address counters (`< 15%` total cell area) to minimize leakage while using `LVT` for general pipeline registers and `SVT` for `u_csr` (`200 MHz` `pclk` domain).
- **Hierarchical Clock Gating (ICG):** All pipeline stages are coded with explicit `if (pipe_en) if (v_dX)` enables so synthesis automatically infers integrated clock-gating cells (`CKLNQD*`), cutting dynamic clock tree power by `~40%` during horizontal/vertical blanking and lower-FPS modes (`24/40 fps`).
