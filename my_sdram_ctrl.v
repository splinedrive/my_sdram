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
`timescale 1ns/1ps
`ifndef SYNTHESES
`default_nettype none
`endif

module my_sdram_ctrl(
           input clk,
           input clken,
           input reset_n,

           input [19:0] w_addr,
           input [19:0] r_addr,
           input write_req,
           output reg write_gnt,

           input read_req,
           output reg read_gnt,

           input [15:0] din,
           output reg [15:0] dout,
           output reg busy,

           output sdram_clk,
           output sdram_cken,
           output sdram_ldqm_n, 		//  lower byte, input/output mask
           output sdram_hdqm_n, 		//  high byte, input/output mask

           output [10:0] sdram_addr, 	//  A0-A10 row address, A0-A7 column address
           output [0:0] sdram_ba, 	// bank select A11
           output sdram_cs_n,
           output sdram_we_n,
           output sdram_ras_n,
           output sdram_cas_n,

           input   [15:0] sdram_dq_in,
           output  [15:0] sdram_dq_out,

           output reg read_valid,
           output [4:0] sdram_state
       );

parameter SDRAM_CLK_FREQ = 64;    //  MHz

localparam
    BURST_LENGTH   = 3'b000,        // 000=1, 001=2, 010=4, 011=8
    ACCESS_TYPE    = 1'b0,          // 0=sequential, 1=interleaved
    CAS_LATENCY    = 3'd2,          // 2/3 allowed, tRCD=20ns -> 3 cycles@128MHz
    OP_MODE        = 2'b00,         // only 00 (standard operation) allowed
    NO_WRITE_BURST = 1'b1,          // 0= write burst enabled, 1=only single access write

    sdram_mode = { 1'b0, NO_WRITE_BURST, OP_MODE, CAS_LATENCY, ACCESS_TYPE, BURST_LENGTH};


`ifndef SYNTHESES
// only for simulation
reg[31:0] clk_counter;
always @(posedge clk) begin
    if (reset_n == 1'b0)
        clk_counter <= 0;
    else
        clk_counter <= clk_counter + 1;
end
`endif

reg [3:0] command;
reg cke;
reg ldqm_n;
reg hdqm_n;
reg [10:0] saddr;
reg [0:0] ba; 		// this is a11

assign sdram_clk = clk;
assign sdram_cken = cke;
assign sdram_addr = saddr;
assign sdram_ldqm_n = ldqm_n;
assign sdram_hdqm_n = hdqm_n;
assign {sdram_cs_n, sdram_ras_n, sdram_cas_n, sdram_we_n} = command;
assign sdram_ba = ba;
assign sdram_state = state;

// ISSI-IS425 datasheet page 16
localparam
    // (CS,RAS,CAS,WE)
    MRS   = 4'b0000, // mode register set
    ACT   = 4'b0011, // bank active
    READ  = 4'b0101, // to have read variant with autoprecharge set A10=H
    WRITE = 4'b0100, // A10=H to have autoprecharge
    BST   = 4'b0110, // burst stop
    PRE   = 4'b0010, // precharge selected bank, A10=H both banks
    REF   = 4'b0001, // auto refresh (cke=H), selfrefresh assign cke=L
    NOP   = 4'b0111,
    DSEL  = 4'b1xxx
    ;

localparam

    WAIT_100US  = 100 * SDRAM_CLK_FREQ, // 64 * 1/64e6 = 1us => 100 * 1us
    // command period, PRE to ACT in ns, e.g. 20ns
    TRP         = $rtoi((20 * SDRAM_CLK_FREQ / 1000) + 1),
    // tRC command period (REF to REF/ACT TO ACT) in ns
    TRC         = $rtoi((66 * SDRAM_CLK_FREQ / 1000) + 1), //
    // tRCD active command to read/write command delay, row-col-delay in ns
    TRCD        = $rtoi((20 * SDRAM_CLK_FREQ / 1000) + 1),
    // tCH command hold time
    TCH         = 1
    ;

initial begin
    $display("Clk frequence: %d MHz", SDRAM_CLK_FREQ);
    $display("WAIT_100US: %d cycles", WAIT_100US);
    $display("TRP: %d cycles", TRP);
    $display("TRC: %d cycles", TRC);
    $display("TRCD: %d cycles", TRCD);
    $display("TCH: %d cycles", TCH);
    $display("CAS_LATENCY: %d cycles", CAS_LATENCY);
end

localparam
    RESET                   = 5'd0,
    INIT_SEQ_PRE_CHARGE_ALL = 5'd1,
    INIT_SEQ_AUTO_REFRESH0  = 5'd2,
    INIT_SEQ_AUTO_REFRESH1  = 5'd3,
    INIT_SEQ_LOAD_MODE      = 5'd4,
    IDLE                    = 5'd5,
    COL_READ                = 5'd6,
    CAS_LATENCY_READ_DONE   = 5'd7,
    COL_WRITE               = 5'd9,
    AUTO_REFRESH            = 5'd10,
    PRE_CHARGE_ALL          = 5'd11,
    WAIT_STATES             = 5'd12
    ;

`ifndef SYNTHESES
reg[255:0] state_name;

always @(*) begin
    case (state)
        RESET: state_name = "RESET";
        INIT_SEQ_PRE_CHARGE_ALL: state_name = "INIT_SEQ_PRE_CHARGE_ALL";
        INIT_SEQ_AUTO_REFRESH0: state_name = "INIT_SEQ_AUTO_REFRESH0";
        INIT_SEQ_AUTO_REFRESH1: state_name = "INIT_SEQ_AUTO_REFRESH1";
        INIT_SEQ_LOAD_MODE: state_name = "INIT_SEQ_LOAD_MODE";
        IDLE: state_name = "IDLE";
        CAS_LATENCY_READ_DONE: state_name = "CAS_LATENCY_READ_DONE";
        COL_READ: state_name = "COL_READ";
        COL_WRITE: state_name = "COL_WRITE";
        AUTO_REFRESH: state_name = "AUTO_REFRESH";
        PRE_CHARGE_ALL: state_name = "PRE_CHARGE_ALL";
        WAIT_STATES: state_name = "WAIT_STATES";
        default: state_name = "RESET";
    endcase
end
`endif


reg [4:0]  state;
reg [4:0]  return_state;
reg [13:0] wait_states;

reg [15:0] din_r;
reg [19:0] addr_r;

wire [19:0] addr_mux;
assign addr_mux = (read_req) ? r_addr : w_addr;

always @(posedge clk) begin

    if (reset_n == 1'b0) begin

        state <= RESET;
        busy  <= 1'b1;
        read_valid <= 1'b0;
        read_gnt <= 1'b0;
        write_gnt <= 1'b0;

    end else begin
        if (clken == 1'b1) begin

            case (state)

                RESET: begin
                    cke     <= 1'b0;

                    saddr   <= 0;
                    wait_states <= WAIT_100US;
                    state <= WAIT_STATES;
                    return_state <= INIT_SEQ_PRE_CHARGE_ALL;
                end

                INIT_SEQ_PRE_CHARGE_ALL: begin
                    cke     <= 1'b1;
                    command <= PRE;

                    saddr[10] <= 1'b1; // select all banks
                    wait_states <= TRP;
                    state <= WAIT_STATES;
                    return_state <= INIT_SEQ_AUTO_REFRESH0;
                end

                INIT_SEQ_AUTO_REFRESH0: begin
                    command <= REF;

                    saddr   <= 0;
                    wait_states <= TRC;
                    state <= WAIT_STATES;
                    return_state <= INIT_SEQ_AUTO_REFRESH1;
                end

                INIT_SEQ_AUTO_REFRESH1: begin
                    command <= REF;

                    saddr   <= 0;
                    wait_states <= TRC;
                    state <= WAIT_STATES;
                    return_state <= INIT_SEQ_LOAD_MODE;
                end

                INIT_SEQ_LOAD_MODE: begin
                    command <= MRS;

                    saddr <= sdram_mode;
                    wait_states <= TCH;
                    state <= WAIT_STATES;

                    return_state <= IDLE;
                end

                IDLE: begin
                    hdqm_n <= 1'b1;
                    ldqm_n <= 1'b1;

                    read_valid <= 1'b0;
                    busy       <= 1'b1;

                    if (read_req | write_req) begin
                        command <= ACT;

                        saddr   <= addr_mux[18:8];
                        ba      <= addr_mux[19];

                        din_r <= din;
                        addr_r <= addr_mux;

                        wait_states <= TRCD;
                        state <= WAIT_STATES;
                        return_state <= read_req ? COL_READ : COL_WRITE;

                        if (read_req) begin
                            read_gnt <= 1'b1;
                        end else begin
                            write_gnt <= 1'b1;
                        end

                    end else begin
                        /* autorefresh */
                        command <= REF;

                        saddr   <= 0;
                        ba <= 0;

                        wait_states <= TRC;
                        state <= WAIT_STATES;

                        return_state <= IDLE;
                    end

                end

                COL_READ:
                begin
                    read_gnt <= 1'b0;
                    write_gnt <= 1'b0;

                    command <= READ;

                    hdqm_n <= 1'b0;
                    ldqm_n <= 1'b0;

                    saddr   <= {3'b100, addr_r[7:0]}; // autoprecharge and column
                    ba      <= addr_r[19];

                    wait_states <= CAS_LATENCY;
                    state <= WAIT_STATES;

                    return_state <= CAS_LATENCY_READ_DONE;
                end

                CAS_LATENCY_READ_DONE:
                begin
                    command <= NOP;

                    hdqm_n <= 1'b1;
                    ldqm_n <= 1'b1;

                    read_valid <= 1'b1;
                    dout <= sdram_dq_in;

                    saddr   <= 0;
                    ba      <= 0;
                    wait_states <= TRP;
                    state <= WAIT_STATES;

                    return_state <= IDLE;
                end

                COL_WRITE:
                begin
                    read_gnt <= 1'b0;
                    write_gnt <= 1'b0;

                    command <= WRITE;

                    hdqm_n <= 1'b0;
                    ldqm_n <= 1'b0;


                    saddr   <= {3'b100, addr_r[7:0]};  // autoprecharge and column
                    ba      <= addr_r[19];

                    wait_states <= TRP;
                    state <= WAIT_STATES;

                    return_state <= IDLE;
                end

                PRE_CHARGE_ALL: begin
                    command <= PRE;
                    saddr[10] <= 1'b1; // select all banks

                    ba <= 0;
                    wait_states <= TRP;
                    state <= WAIT_STATES;

                    return_state <= IDLE;
                end

                WAIT_STATES: begin
                    command <= NOP;
                    saddr   <= 0;

                    if (wait_states == 1) begin
                        if (return_state == IDLE) begin busy <= 1'b0; end
                        state <= return_state;
                        wait_states <= 0;
                    end else begin
                        wait_states <= wait_states - 1;
                    end
                end

                default: begin
                    state <= RESET;
                end

            endcase


        end /* clken */

    end /* reset_n */

end

assign sdram_dq_out = (state == COL_WRITE) ? din_r : 16'hZZZZ;

endmodule
