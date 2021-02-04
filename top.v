/*
 *  my_sdram_ctrl - A simple dual ported sdram controller
 *
 *  Copyright (C) 2021  Hirosh Dabui <hirosh@dabui.de>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */
`include "my_sdram_ctrl.v"

`ifdef SYNTHESES
//////////////////////////////////////////////////////////////////////////////////////////
module top(
  input           clk_25M,

  output  [10:0]  sd_addr,
  output          sd_ba,
  inout   [15:0]  sd_data,
  output  [1:0]   sd_dqm,
  output          sd_we,
  output          sd_cas,
  output          sd_ras,
  output          sd_cs,
  output          sd_cke,
  output          sd_clk,
  output  [15:0]   PMOD,
  output  [47:32]  PMODA,
  input   [31:31]  PMODB,
  output  [23:16]  PMODS, /* 23-20 19-16 */

  output reg led
);

/*
   16 | 17 | 18 | 19
   20 | 21 | 22 | 23

   AD   AC   AB  AA
   CA   AG   AF  AE
 */

/* reversing the bit order */
function [6:0] REVERSE (
  input [6:0] d
);
  integer i;

  begin
    for (i = 0; i <= 6; i = i + 1) begin
      REVERSE[6-i] = d[i];
    end
  end
endfunction

wire AA, AB, AC, AD, AE, AF, AG, CA;
assign PMODS = {AD, AC, AB, AA, CA, AG, AF, AE};
assign {AA, AB, AC, AD, AE, AF, AG, CA} = {~REVERSE(segments), ~sel};

reg sel;
always @(posedge clk) begin

  if (cnt[10]) begin
    sel <= sel ^ 1'b1;

    if (sel) begin
      bcd <= state;//dout[3:0];
    end else begin
      bcd <= state;//r_addr[3:0];
    end

  end

end

wire  [6:0] segments;
reg   [3:0] bcd;
/*
always @(posedge clk) begin

  if (reset_n == 1'b0) begin
    bcd <= 0;
  end else begin
    if (button_sync) bcd <= (bcd == ~0) ? 0 : bcd + 1;
  end

end
*/

always @(*) begin
  case (bcd)
    4'b0000: segments = 7'b0111111; // 0 ABCDEF
    4'b0001: segments = 7'b0000110; // 1 BC
    4'b0010: segments = 7'b1011011; // 2 ABDEG
    4'b0011: segments = 7'b1001111; // 3 ABCDG
    4'b0100: segments = 7'b1100110; // 4 BCFG
    4'b0101: segments = 7'b1101101; // 5 ACDFG
    4'b0110: segments = 7'b1111101; // 6 ACDEFG
    4'b0111: segments = 7'b0000111; // 7 ABC
    4'b1000: segments = 7'b1111111; // 8 ABCDEFG
    4'b1001: segments = 7'b1101111; // 9 ABCDFG
    4'b1010: segments = 7'b1110111; // A ABCEFG
    4'b1011: segments = 7'b1111100; // B CDEFG
    4'b1100: segments = 7'b1011000; // C DEG
    4'b1101: segments = 7'b1011110; // D BCDEG
    4'b1110: segments = 7'b1111001; // E ADEFG
    4'b1111: segments = 7'b1110001; // F AEFG
    default:
      segments <= ~0;
  endcase
end

wire button = ~PMODB[31];

reg [1:0] delay_r = 0;

wire button_sync;

reg toggle;

reg [31:0] cnt;
always @(posedge clk) cnt <= cnt + 1;

// button //
assign button_sync = ~delay_r[1] & delay_r[0];
always @(posedge clk) begin
  if (reset_n == 1'b0) begin
    toggle <= 1'b1;
    delay_r <= 2'b00;
    delay_cnt <= 0;
  end else begin
    if (cnt[11]) begin
    delay_r = {delay_r[0], button};
    toggle <= toggle ^ button_sync;
  end
end
end

// lfsr //
reg [15:0] lfsr = ~0;
wire feedback;

assign feedback = lfsr[15] ^ 0;
always @(posedge clk) begin
  if (&cnt[20:5])
    lfsr <= {lfsr[14:0], 1'b0} ^ (feedback ? 16'b1000000001011: 0);
end

reg led_rollover;
assign led = led_rollover;

always @(posedge clk) begin
  if (end_of_mem) led_rollover <= led_rollover ^ 1'b1;
end

`define CLK_64MHZ
`ifdef CLK_64MHZ
  // 64Mhz clock from pll
  localparam SYSTEM_CLK_MHZ = 64;
  wire clk;
  wire locked;
  pll pll_i(
    .clock_in(clk_25M),
    .clock_out(clk),
    .locked(locked)
  );
`else
  localparam SYSTEM_CLK_MHZ = 25;
  wire clk;
  assign clk = clk_25M;
`endif


//wire [15:0] leds = 16'b0000_0000_0000_0000;
wire [15:0] leds;// = SDRAM_A[19:4];//{ {15{1'b0}}, error}; //SDRAM_DQ_IN;
wire [15:0] leds1;// = SDRAM_A[19:4];//{ {15{1'b0}}, error}; //SDRAM_DQ_IN;
//assign      leds = RdData;//SDRAM_DQ_IN;
//assign leds1 = 0;
assign PMOD[15:0] = ((leds & 16'h000f)<<4) | ((leds & 16'h00f0)>>4)
		| ((leds & 16'h0f00)<<4) | ((leds & 16'hf000)>>4);

assign PMOD[23:20] = {!busy,valid,sdram_we_n,sd_clk};
assign PMOD[19:16] = {sdram_dq_in[0], sdram_dq_out[0], read_req, write_req};

assign PMODA[32+:16] = ((leds1 & 16'h000f)<<4) | ((leds1 & 16'h00f0)>>4)
		| ((leds1 & 16'h0f00)<<4) | ((leds1 & 16'hf000)>>4);

  SB_IO #(
    .PIN_TYPE(6'b1010_01),
    .PULLUP(1'b0)
  ) sdram_i [15:0] (
    .PACKAGE_PIN(sd_data),
    .OUTPUT_ENABLE({16{~sdram_we_n}}),
    .D_OUT_0(sdram_dq_out),
    .D_IN_0(sdram_dq_in)
  );

// sdram
assign sd_addr  = sdram_addr;
assign sd_ba    = sdram_ba;
assign sd_dqm   = {sdram_hdqm_n, sdram_ldqm_n};
assign sd_we    = sdram_we_n;
assign sd_cas   = sdram_cas_n;
assign sd_ras   = sdram_ras_n;
assign sd_clk   = sdram_clk;
assign sd_cke   = sdram_cken;
assign sd_cs    = sdram_cs_n;
assign leds = dout;
assign leds1 = w_addr[19:4] | r_addr[19:4];
//////////////////////////////////////////////////////////////////////////////////////////
`else
`timescale 1ns/1ps
`default_nettype none
module top_tb();

localparam SYSTEM_CLK_MHZ=25;
reg clk = 0;
always #5 clk = !clk;

initial
begin

  $dumpfile("top.vcd");
  $dumpvars(0, top_tb);
  //$dumpoff;
  $dumpon;

  repeat(20000) @(posedge clk);
  $finish();
end
`endif

reg clken;

reg [19:0] w_addr;
reg [19:0] r_addr;

reg write_req;
wire write_gnt;

reg read_req;
wire read_gnt;

reg   [15:0] din = 0;
wire  [15:0] dout;
wire  busy;

wire sdram_clk;
wire sdram_cken;
wire sdram_ldqm_n;
wire sdram_hdqm_n;

wire [10:0] sdram_addr;
wire sdram_ba;
wire sdram_cs_n;
wire sdram_we_n;
wire sdram_ras_n;
wire sdram_cas_n;

wire [15:0] sdram_dq_in;
wire [15:0] sdram_dq_out;

wire read_valid;
wire [4:0] sdram_state;

my_sdram_ctrl #(.SDRAM_CLK_FREQ(SYSTEM_CLK_MHZ)) my_sdram_ctrl_i(
  clk,
  clken,
  reset_n,

  w_addr,
  r_addr,

  write_req,
  write_gnt,

  read_req,
  read_gnt,

  din,
  dout,
  busy,

  sdram_clk,
  sdram_cken,
  sdram_ldqm_n,
  sdram_hdqm_n,

  sdram_addr,
  sdram_ba,
  sdram_cs_n,
  sdram_we_n,
  sdram_ras_n,
  sdram_cas_n,

  sdram_dq_in,
  sdram_dq_out,

  read_valid,
  sdram_state
);

reg [3:0]   state;
reg [3:0]   return_state;
reg [31:0]  wait_states;

reg busy_r;
reg read_valid_r;
reg write_gnt_r;
reg read_gnt_r;
always @(posedge clk) begin
  busy_r        <= busy;
  read_valid_r  <= read_valid;
  write_gnt_r   <= write_gnt;
  read_gnt_r    <= read_gnt;
end

wire free = !(!busy_r & busy);
wire valid = (!read_valid_r & read_valid);
wire write_gnt_edge = (!write_gnt_r & write_gnt);
wire read_gnt_edge  = (!read_gnt_r  & read_gnt);


reg [5:0] reset_cnt = 0;
wire reset_n = &reset_cnt;
always @(posedge clk) reset_cnt <= reset_cnt + !reset_n;

localparam OFFSET = 1;
localparam END_OF_MEMORY = ((1<<20)-OFFSET);
always @(posedge clk) begin

  if (reset_n == 1'b0) begin

    state <= 0;
    clken <= 0;

  end else begin

    case (state)

      0: begin
        clken <= 0;
        read_req <= 1'b0;
        write_req <= 1'b0;
        w_addr <= 0;
        r_addr <= 0;
        wait_states <= 12_500_000;//SYSTEM_CLK_MHZ*1000_000; // 100 us
        return_state <= 1;
        state <= 1;
      end

      1: begin
        clken <= 1'b1;
        if (free) begin
          state <= 2; // should be !free
          write_req <= 1;
        end
      end

    2: begin
      din <= w_addr[19:4];

      if (write_gnt_edge) begin
        w_addr <= w_addr + OFFSET;

        if (w_addr == (END_OF_MEMORY)) begin
          din <= 0;
          r_addr <= 0;
          w_addr <= 0;

          write_req <= 0;
          read_req <= 1;

          state <= 3;
        end
      end
      //state <= 10;
      //wait_states <= SYSTEM_CLK_MHZ;
      //return_state <= 2;
    end

    3: begin

      read_req <= 1'b1;
      if (read_gnt_edge) state <= 4;

    end

  4: begin
    read_req <= 1'b1;
    if (valid) begin
      r_addr <= r_addr + OFFSET;

      if (dout != r_addr[19:4]) begin
        state <= 11;
      end else begin

        if (r_addr == (END_OF_MEMORY)) begin
          r_addr <= 0;
          w_addr <= 0;

          write_req <= 1;
          read_req <= 0;

          state <= 2;
        end else begin
          state <= 3;
          wait_states <= SYSTEM_CLK_MHZ*1_0000;
          return_state <= 3;
        end
      end
    end

  end

    10: begin
      if (wait_states == 1) begin
        state <= return_state;
        wait_states <= 0;
      end else begin
        wait_states <= wait_states - 1;
      end
    end

    11: begin
      w_addr = 1<<19;
      r_addr = 1<<0;
    end
  endcase

end

end
endmodule

`ifdef SYNTHESES
 /*
  * PLL configuration
  *
  * This Verilog module was generated automatically
  * using the icepll tool from the IceStorm project.
  * Use at your own risk.
  *
  * Given input frequency:        25.000 MHz
  * Requested output frequency:   64.000 MHz
  * Achieved output frequency:    64.062 MHz
  */

   module pll(
     input  clock_in,
     output clock_out,
     output locked
   );

   SB_PLL40_CORE #(
     .FEEDBACK_PATH("SIMPLE"),
     .DIVR(4'b0000),		// DIVR =  0
     .DIVF(7'b0101000),	// DIVF = 40
     .DIVQ(3'b100),		// DIVQ =  4
     .FILTER_RANGE(3'b010)	// FILTER_RANGE = 2
   ) uut (
     .LOCK(locked),
     .RESETB(1'b1),
     .BYPASS(1'b0),
     .REFERENCECLK(clock_in),
     .PLLOUTCORE(clock_out)
   );

   endmodule
 `endif
