/*
 * Copyright (c) 2026 aPR0T0
 * SPDX-License-Identifier: Apache-2.0
 *
 * rv32e_core - small multi-cycle RISC-V RV32E CPU (no M, no C, no interrupts).
 *
 *   - 15 x 32-bit registers (x0 = 0), single read port, read in two cycles
 *   - one shared bus for instruction fetch and data (word bus + byte strobes);
 *     a request is held until m_ack, all bus outputs are combinational from
 *     state that is stable while waiting
 *   - shifts are done one bit per clock
 *   - ECALL / EBREAK / illegal instructions / misaligned jumps halt the core
 *     and report a cause code (useful as a firmware "exit" and for debugging)
 *   - FENCE and CSR instructions execute as no-ops (CSR reads return 0)
 *   - debug: `pause` stops the core at the next instruction boundary (before the
 *     fetch, all state kept); a `step` pulse lets exactly one instruction run.
 *     `fetch_entry` pulses when a new instruction boundary is reached, so the
 *     SoC can implement a PC breakpoint.  The reset PC is an input (boot address).
 *
 * Typical CPI ~ 5 plus memory wait states (instruction fetch from QSPI flash).
 */
`default_nettype none

module rv32e_core (
    input  wire        clk,
    input  wire        rst,
    input  wire [31:2] reset_pc,
    input  wire        pause,
    input  wire        step,
    output wire        paused,
    output reg         fetch_entry,
    output wire        m_req,
    output wire        m_we,
    output wire [31:0] m_addr,
    output wire [31:0] m_wdata,
    output wire [3:0]  m_wstrb,
    input  wire [31:0] m_rdata,
    input  wire        m_ack,
    output reg         halted,
    output reg  [3:0]  cause,
    output wire [31:0] pc_o
);
  localparam [2:0] S_FETCH = 3'd0, S_DEC = 3'd1, S_RS2 = 3'd2, S_EXE = 3'd3,
                   S_MEM = 3'd4, S_HALT = 3'd5;

  localparam [6:0] OP_LUI = 7'b0110111, OP_AUIPC = 7'b0010111, OP_JAL = 7'b1101111,
                   OP_JALR = 7'b1100111, OP_BR = 7'b1100011, OP_LD = 7'b0000011,
                   OP_ST = 7'b0100011, OP_IMM = 7'b0010011, OP_OP = 7'b0110011,
                   OP_FENCE = 7'b0001111, OP_SYS = 7'b1110011;

  reg [2:0]  st;
  reg [31:0] ir, a, b;
  reg [31:2] pc;
  reg [4:0]  shcnt;
  reg [31:0] daddr;        // registered data address (breaks the adder -> bus path)
  reg        step_ok;      // one instruction may be fetched while paused

  assign pc_o = {pc, 2'b00};

  // ---------------- decode ----------------
  wire [6:0]  opc   = ir[6:0];
  wire [3:0]  rd    = ir[10:7];
  wire [2:0]  f3    = ir[14:12];
  wire [3:0]  rs1   = ir[18:15];
  wire [3:0]  rs2   = ir[23:20];
  wire        f7b   = ir[30];
  wire [31:0] imm_i = {{20{ir[31]}}, ir[31:20]};
  wire [31:0] imm_s = {{20{ir[31]}}, ir[31:25], ir[11:7]};
  wire [31:0] imm_b = {{19{ir[31]}}, ir[31], ir[7], ir[30:25], ir[11:8], 1'b0};
  wire [31:0] imm_u = {ir[31:12], 12'b0};
  wire [31:0] imm_j = {{11{ir[31]}}, ir[31], ir[19:12], ir[20], ir[30:21], 1'b0};

  wire is_op    = (opc == OP_OP);
  wire is_st    = (opc == OP_ST);
  wire need_rs2 = (opc == OP_BR) || is_st || is_op;
  wire is_shift = ((opc == OP_IMM) || is_op) && (f3 == 3'b001 || f3 == 3'b101);

  // ---------------- ALU ----------------
  wire [31:0] opb     = (is_op || opc == OP_BR) ? b : is_st ? imm_s : imm_i;
  wire        do_sub  = (opc == OP_BR) || ((is_op || opc == OP_IMM) && (f3 == 3'b010 || f3 == 3'b011)) ||
                        (is_op && f3 == 3'b000 && f7b);
  wire [32:0] addsub  = {1'b0, a} + {1'b0, do_sub ? ~opb : opb} + {32'd0, do_sub};
  wire        eq      = (a == opb);
  wire        ltu     = !addsub[32];
  wire        lt      = (a[31] != opb[31]) ? a[31] : addsub[31];

  reg [31:0] alu;
  always @(*) begin
    case (f3)
      3'b010:  alu = {31'd0, lt};
      3'b011:  alu = {31'd0, ltu};
      3'b100:  alu = a ^ opb;
      3'b110:  alu = a | opb;
      3'b111:  alu = a & opb;
      default: alu = addsub[31:0];
    endcase
  end

  reg take;
  always @(*) begin
    case (f3)
      3'b000:  take = eq;
      3'b001:  take = !eq;
      3'b100:  take = lt;
      3'b101:  take = !lt;
      3'b110:  take = ltu;
      3'b111:  take = !ltu;
      default: take = 1'b0;
    endcase
  end

  wire [31:2] pc4    = pc + 30'd1;
  wire [31:0] pc_imm = {pc, 2'b00} + ((opc == OP_JAL) ? imm_j : (opc == OP_BR) ? imm_b : imm_u);

  // ---------------- bus (combinational, stable while waiting) ----------------
  wire   go_fetch = !pause || step_ok;
  assign paused  = !rst && (st == S_FETCH) && !go_fetch;
  assign m_req   = !rst && (((st == S_FETCH) && go_fetch) || (st == S_MEM));
  assign m_we    = (st == S_MEM) && is_st;
  assign m_addr  = (st == S_MEM) ? {daddr[31:2], 2'b00} : {pc, 2'b00};
  assign m_wdata = (f3 == 3'b000) ? {4{b[7:0]}} : (f3 == 3'b001) ? {2{b[15:0]}} : b;
  assign m_wstrb = (f3 == 3'b000) ? (4'b0001 << daddr[1:0]) :
                   (f3 == 3'b001) ? (daddr[1] ? 4'b1100 : 4'b0011) : 4'b1111;

  // load data alignment
  wire [31:0] ld_sh = m_rdata >> {daddr[1:0], 3'b000};
  reg  [31:0] ld_val;
  always @(*) begin
    case (f3)
      3'b000:  ld_val = {{24{ld_sh[7]}}, ld_sh[7:0]};
      3'b001:  ld_val = {{16{ld_sh[15]}}, ld_sh[15:0]};
      3'b100:  ld_val = {24'd0, ld_sh[7:0]};
      3'b101:  ld_val = {16'd0, ld_sh[15:0]};
      default: ld_val = m_rdata;
    endcase
  end

  // ---------------- register file ----------------
  reg  [31:0] rf [1:15];
  wire [3:0]  rf_ra = (st == S_DEC) ? rs1 : rs2;
  wire [31:0] rf_rd = (rf_ra == 4'd0) ? 32'd0 : rf[rf_ra];

  // write-back value and enable
  reg        wb_en;
  reg [31:0] wb_val;
  always @(*) begin
    wb_en  = 1'b0;
    wb_val = alu;
    if (st == S_MEM) begin
      wb_en  = m_ack && !is_st;
      wb_val = ld_val;
    end else if (st == S_EXE) begin
      case (opc)
        OP_LUI:   begin wb_en = 1'b1; wb_val = imm_u; end
        OP_AUIPC: begin wb_en = 1'b1; wb_val = pc_imm; end
        OP_JAL:   begin wb_en = !pc_imm[1]; wb_val = {pc4, 2'b00}; end
        OP_JALR:  begin wb_en = !addsub[1]; wb_val = {pc4, 2'b00}; end
        OP_IMM, OP_OP: begin
          wb_en  = !(is_op && ir[25]) && !(is_shift && shcnt != 5'd0);
          wb_val = is_shift ? a : alu;
        end
        OP_SYS:   begin wb_en = (f3 != 3'b000); wb_val = 32'd0; end
        default:  ;
      endcase
    end
  end

  always @(posedge clk) begin
    if (wb_en && rd != 4'd0) rf[rd] <= wb_val;
  end

  // ---------------- control ----------------
  always @(posedge clk) begin
    if (rst) begin
      st <= S_FETCH; ir <= 32'd0; a <= 32'd0; b <= 32'd0; pc <= reset_pc;
      shcnt <= 5'd0; halted <= 1'b0; cause <= 4'd0; daddr <= 32'd0;
      step_ok <= 1'b0; fetch_entry <= 1'b1;
    end else begin
      fetch_entry <= 1'b0;
      if (step) step_ok <= 1'b1;
      case (st)
        S_FETCH: if (m_ack) begin
          ir      <= m_rdata;
          st      <= S_DEC;
          step_ok <= 1'b0;
        end

        S_DEC: begin
          a     <= rf_rd;
          shcnt <= ir[24:20];
          st    <= need_rs2 ? S_RS2 : S_EXE;
        end

        S_RS2: begin
          b     <= rf_rd;
          shcnt <= rf_rd[4:0];
          st    <= S_EXE;
        end

        S_EXE: begin
          st <= S_FETCH;
          fetch_entry <= 1'b1;
          pc <= pc4;
          case (opc)
            OP_JAL:  if (pc_imm[1]) begin st <= S_HALT; cause <= 4'd4; pc <= pc; end
                     else pc <= pc_imm[31:2];
            OP_JALR: if (addsub[1]) begin st <= S_HALT; cause <= 4'd4; pc <= pc; end
                     else pc <= addsub[31:2];
            OP_BR:   if (take) begin
                       if (pc_imm[1]) begin st <= S_HALT; cause <= 4'd4; pc <= pc; end
                       else pc <= pc_imm[31:2];
                     end
            OP_LD, OP_ST: begin
              daddr <= addsub[31:0];
              pc    <= pc;
              st    <= S_MEM;
              fetch_entry <= 1'b0;
            end
            OP_IMM, OP_OP: begin
              if (is_op && ir[25]) begin
                st <= S_HALT; cause <= 4'd3; pc <= pc;   // M extension not implemented
              end else if (is_shift && shcnt != 5'd0) begin
                // shift one bit per clock, stay in EXE
                a     <= (f3 == 3'b001) ? {a[30:0], 1'b0} : {f7b & a[31], a[31:1]};
                shcnt <= shcnt - 5'd1;
                pc    <= pc;
                st    <= S_EXE;
                fetch_entry <= 1'b0;
              end
            end
            OP_LUI, OP_AUIPC, OP_FENCE: ;
            OP_SYS: begin
              if (f3 == 3'b000) begin
                if (ir[31:20] == 12'd0)      begin st <= S_HALT; cause <= 4'd1; pc <= pc; end // ECALL
                else if (ir[31:20] == 12'd1) begin st <= S_HALT; cause <= 4'd2; pc <= pc; end // EBREAK
              end
            end
            default: begin st <= S_HALT; cause <= 4'd3; pc <= pc; end
          endcase
        end

        S_MEM: if (m_ack) begin
          pc <= pc4;
          st <= S_FETCH;
          fetch_entry <= 1'b1;
        end

        default: halted <= 1'b1;  // S_HALT
      endcase
    end
  end

endmodule
