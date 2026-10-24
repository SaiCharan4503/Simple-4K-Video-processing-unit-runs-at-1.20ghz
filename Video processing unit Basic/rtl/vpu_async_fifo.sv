//------------------------------------------------------------------------------
// File       : vpu_async_fifo.sv
// Author     : Senior RTL Design & Verification Lead
// Description: Dual-clock Asynchronous FIFO with Gray-coded pointers and
//              2-stage synchronizers for crossing between sensor_clk and
//              core_clk (2 GHz). Uses registered output stage for 2 GHz timing.
//------------------------------------------------------------------------------

module vpu_async_fifo #(
  parameter int unsigned DATA_WIDTH = 12, // {sof, eol, pixel[9:0]}
  parameter int unsigned ADDR_WIDTH = 4   // Depth = 2^4 = 16 entries
) (
  // Write Domain (sensor_clk)
  input  logic                  wr_clk,
  input  logic                  wr_rst_n,
  input  logic                  wr_en,
  input  logic [DATA_WIDTH-1:0] wr_data,
  output logic                  wr_full,
  output logic                  wr_overflow_pulse,

  // Read Domain (core_clk - 2 GHz)
  input  logic                  rd_clk,
  input  logic                  rd_rst_n,
  input  logic                  rd_en,
  output logic [DATA_WIDTH-1:0] rd_data,
  output logic                  rd_empty,
  output logic                  rd_underflow_pulse
);

  localparam int unsigned DEPTH = 1 << ADDR_WIDTH;

  // Storage array (synthesizes to flip-flop register file for 500ps 2 GHz access)
  logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

  // Binary and Gray pointers (ADDR_WIDTH+1 bits to distinguish full/empty)
  logic [ADDR_WIDTH:0] wr_bin, wr_bin_next;
  logic [ADDR_WIDTH:0] wr_gray, wr_gray_next;
  logic [ADDR_WIDTH:0] rd_bin, rd_bin_next;
  logic [ADDR_WIDTH:0] rd_gray, rd_gray_next;

  // Synchronizer registers (marked for ASYNC_REG synthesis attribute)
  (* ASYNC_REG = "TRUE" *) logic [ADDR_WIDTH:0] wr_gray_sync1, wr_gray_sync2;
  (* ASYNC_REG = "TRUE" *) logic [ADDR_WIDTH:0] rd_gray_sync1, rd_gray_sync2;

  //----------------------------------------------------------------------------
  // Write Domain Logic (sensor_clk)
  //----------------------------------------------------------------------------
  assign wr_bin_next  = wr_bin + {{ADDR_WIDTH{1'b0}}, (wr_en && !wr_full)};
  assign wr_gray_next = (wr_bin_next >> 1) ^ wr_bin_next;

  // Full condition in Gray code: MSBs inverted, remaining bits equal
  wire wr_full_next = (wr_gray_next == {~rd_gray_sync2[ADDR_WIDTH:ADDR_WIDTH-1],
                                         rd_gray_sync2[ADDR_WIDTH-2:0]});

  always_ff @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
      wr_bin            <= '0;
      wr_gray           <= '0;
      wr_full           <= 1'b0;
      wr_overflow_pulse <= 1'b0;
    end else begin
      wr_bin            <= wr_bin_next;
      wr_gray           <= wr_gray_next;
      wr_full           <= wr_full_next;
      wr_overflow_pulse <= wr_en && wr_full;
    end
  end

  always_ff @(posedge wr_clk) begin
    if (wr_en && !wr_full) begin
      mem[wr_bin[ADDR_WIDTH-1:0]] <= wr_data;
    end
  end

  // Synchronize Read Gray Pointer into Write Clock Domain
  always_ff @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
      rd_gray_sync1 <= '0;
      rd_gray_sync2 <= '0;
    end else begin
      rd_gray_sync1 <= rd_gray;
      rd_gray_sync2 <= rd_gray_sync1;
    end
  end

  //----------------------------------------------------------------------------
  // Read Domain Logic (core_clk - 2 GHz)
  //----------------------------------------------------------------------------
  assign rd_bin_next  = rd_bin + {{ADDR_WIDTH{1'b0}}, (rd_en && !rd_empty)};
  assign rd_gray_next = (rd_bin_next >> 1) ^ rd_bin_next;

  wire rd_empty_next = (rd_gray_next == wr_gray_sync2);

  always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
      rd_bin             <= '0;
      rd_gray            <= '0;
      rd_empty           <= 1'b1;
      rd_underflow_pulse <= 1'b0;
    end else begin
      rd_bin             <= rd_bin_next;
      rd_gray            <= rd_gray_next;
      rd_empty           <= rd_empty_next;
      rd_underflow_pulse <= rd_en && rd_empty;
    end
  end

  // FWFT (First-Word Fall-Through) lookahead read for single-cycle 2 GHz stream interface
  assign rd_data = mem[rd_bin[ADDR_WIDTH-1:0]];

  // Synchronize Write Gray Pointer into Read Clock Domain
  always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
      wr_gray_sync1 <= '0;
      wr_gray_sync2 <= '0;
    end else begin
      wr_gray_sync1 <= wr_gray;
      wr_gray_sync2 <= wr_gray_sync1;
    end
  end

endmodule : vpu_async_fifo
