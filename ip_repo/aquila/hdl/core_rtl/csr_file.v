`timescale 1ns / 1ps 
// =============================================================================
//  Program : csr_file.v
//  Author  : Jin-you Wu
//  Date    : Dec/18/2018
// -----------------------------------------------------------------------------
//  Description:
//  This module implements the Control and Status Register File of the
//  Aquila core (A RISC-V core).
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Dec/16/2019, by Chun-Jen Tsai:
//    Map the CSR addresses of CYCLES & CYCLESH to that of MCYCLES & MCYCLESH's
//    counters so that 'RDCYCLE' & 'RDCYCLEH' pseudo instructions can read the
//    counters.
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2019,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Chiao Tung Uniersity
//                    Hsinchu, Taiwan.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
// =============================================================================

module csr_file #(parameter HART_ID = 0, ADDR_WIDTH = 32, DATA_WIDTH = 32)
(
    input                     clk_i,
    input                     rst_i,

    // from Decode_Eexcute_Pipeline
    input                     is_csr_instr_i,
    input [11: 0]             csr_addr_i,
    input [2: 0]              csr_op_i,
    input [4: 0]              csr_imm_i,

    // from RegisterFile
    input [DATA_WIDTH-1 : 0]  rs1_data_i,

    // Interrupt requests.
    input                     ext_irq_i,
    input                     tmr_irq_i,
    input                     sft_irq_i,
    output                    irq_taken_o,
    output [ADDR_WIDTH-1 : 0] PC_handler_o,
    input  [ADDR_WIDTH-1 : 0] nxt_unexec_PC_i,

    // to Execute_Memory_Pipeline
    output [DATA_WIDTH-1 : 0] csr_data_o,

    // System Jump operation
    input                     sys_jump_i,
    input  [11: 0]            sys_jump_csr_addr_i,
    input  [ADDR_WIDTH-1 : 0] sys_jump_pc_i,
    output [DATA_WIDTH-1 : 0] sys_jump_csr_data_o,

    //MMU
    output mmu_enable_o,
    output [21: 0] root_ppn_o,
    output [ 8: 0] asid_o,
    output [ 1: 0] privilege_lvl_o,

    // Exception requests
    input wire                    exception_vld_i,
    input wire [ 3: 0]            exception_cause_i
);

//==============================================================================================
// Parameter and Integer
//==============================================================================================

// =============================================================================================
//  Machine-level CSRs (v1.10)
//          CSR Name        Address         R/W Attribute       Full Name
`define     CSR_MSTATUS     12'h300         // MRW              Machine Status Register
`define     CSR_MISA        12'h301         // MRW              Machine ISA Register
`define     CSR_MEDELEG     12'h302         // MRW              Machine Exception Delegation Register
`define     CSR_MIDELEG     12'h303         // MRW              Machine Interrupt Delegation Register
`define     CSR_MIE         12'h304         // MRW              Machine Interrupt Enable Register
`define     CSR_MTVEC       12'h305         // MRW              Machine Trap-Vector Base-Address Register
`define     CSR_MCOUNTEREN  12'h306         // MRW              Machine Counter Enable

`define     CSR_MSCRATCH    12'h340         // MRW              Machine Scratch Register
`define     CSR_MEPC        12'h341         // MRW              Machine Exception Program Counter
`define     CSR_MCAUSE      12'h342         // MRW              Machine Cause Register
`define     CSR_MTVAL       12'h343         // MRW              Machine Trap Value Register
`define     CSR_MIP         12'h344         // MRW              Machine Interrupt Pending Register

`define     CSR_MCYCLE      12'hB00         // MRW              Lower 32 bits of Cycle counter
`define     CSR_MCYCLEH     12'hB80         // MRW              Upper 32 bits of Cycle counter
`define     CSR_MINSTRET    12'hB02         // MRW              Lower 32 bits of Instructions-retired counter
`define     CSR_MINSTRETH   12'hB82         // MRW              Upper 32 bits of Instructions-retired counter

`define     CSR_MVENDORID   12'hF11         // MRO              Machine Vendor ID Register
`define     CSR_MARCHID     12'hF12         // MRO              Machine Architecture ID Register
`define     CSR_MIMPID      12'hF13         // MRO              Machine Implementation ID Register
`define     CSR_MHARTID     12'hF14         // MRO              Hart ID Register


// =============================================================================================
//  Supervisor-level CSRs (v1.11)
//          CSR Name        Address         R/W Attribute       Full Name
`define     CSR_SSTATUS     12'h100         // SRW              Supervisor status register
`define     CSR_SEDELEG     12'h102         // SRW              Supervisor Exception Delegation Register
`define     CSR_SIDELEG     12'h103         // SRW              Supervisor Interrupt Delegation Register
`define     CSR_SIE         12'h104         // SRW              Supervisor Interrupt Enable Register
`define     CSR_STVEC       12'h105         // SRW              Supervisor Trap-Vector Base-Address Register
`define     CSR_SCOUNTEREN  12'h106         // SRW              Supervisor Counter Enable

`define     CSR_SSCRATCH    12'h140         // SRW              Supervisor Scratch Register
`define     CSR_SEPC        12'h141         // SRW              Supervisor Exception Program Counter
`define     CSR_SCAUSE      12'h142         // SRW              Supervisor Cause Register
`define     CSR_STVAL       12'h143         // SRW              Supervisor Trap Value Register
`define     CSR_SIP         12'h144         // SRW              Supervisor Interrupt Pending Register

`define     CSR_SATP        12'h180         // SRW              Supervisor address translation and protection

// =============================================================================================
//  User-level CSRs (v1.11)
//          CSR Name        Address         R/W Attribute       Full Name
`define     CSR_CYCLE       12'hC00         // RO               Cycle counter for RDCYCLE instruction.
`define     CSR_TIME        12'hC01         // RO               Timer for RDTIME instruction.
`define     CSR_INSTRET     12'hC02         // RO               Instructions-retired counter for RDINSTRET instruction.
`define     CSR_CYCLEH      12'hC00         // RO               Upper 32 bits of cycle, RV32I only.
`define     CSR_TIMEH       12'hC01         // RO               Upper 32 bits of time, RV32I only.
`define     CSR_INSTRETH    12'hC02         // RO               Upper 32 bits of instret, RV32I only.



`define CSR_RW 2'b01
`define CSR_RS 2'b10
`define CSR_RC 2'b11

//==============================================================================================
// Wire and Reg 
//==============================================================================================
// =============================================================================================
//  CSRs implementations
//
//M-Mode register
reg  [DATA_WIDTH-1 : 0] mstatus_r;
reg  [DATA_WIDTH-1 : 0] misa_r;
reg  [DATA_WIDTH-1 : 0] mie_r;
reg  [DATA_WIDTH-1 : 0] mtvec_r;
reg  [DATA_WIDTH-1 : 0] mscratch_r;
reg  [DATA_WIDTH-1 : 0] mepc_r;
reg  [DATA_WIDTH-1 : 0] mcause_r, mcause_d;
//reg  [DATA_WIDTH-1 : 0] mtval_r;  // for exception
reg  [DATA_WIDTH-1 : 0] mip_r;
reg  [63           : 0] mcycle_r;
//reg  [63 : 0] minstret_r;
wire [DATA_WIDTH-1 : 0] mvendorid = 0;  // Non-commercial implementation, so return 0
wire [DATA_WIDTH-1 : 0] marchid   = 0;
wire [DATA_WIDTH-1 : 0] mimpid    = 0;
wire [DATA_WIDTH-1 : 0] mhartid   = HART_ID;

//S-Mode register
wire  [DATA_WIDTH-1 : 0] sstatus;
reg   [DATA_WIDTH-1 : 0] satp_r;

//U-Mode register
reg  [63           : 0] cycle_r;

//
reg  [1            : 0] privilege_lvl_r;

wire [DATA_WIDTH-1 : 0] mtvec_base;
//wire retired = instr_valid & !stall;

wire is_mret = (sys_jump_csr_addr_i == `CSR_MEPC);
wire is_sret = (sys_jump_csr_addr_i == `CSR_SEPC);
wire is_ecall = (sys_jump_csr_addr_i == `CSR_MTVEC);

reg  [DATA_WIDTH-1 : 0] csr_data;
wire [DATA_WIDTH-1 : 0] csr_inputA = csr_data;
wire [DATA_WIDTH-1 : 0] csr_inputB = csr_op_i[2] ? {27'b0, csr_imm_i} : rs1_data_i;
reg  [DATA_WIDTH-1 : 0] updated_csr_data;

//==============================================================================================
// User Logic                         
//==============================================================================================

// =============================================================================================
//  M-MODE SYSTEM Operations
//
//-----------------------------------------------
// mstatus
//-----------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
// | SD |  WPRI  | TSR | TW | TVM | MXR | SUM | MPRV |  XS  |  FS  | MPP | WPRI | SPP | MPIE | WPRI | SPIE | UPIE | MIE | WPRI | SIE | UIE |
// -----------------------------------------------------------------------------------------------------
// | 31 |30    23| 22  | 21 | 20  | 19  | 18  |  17  |16  15|14  13|12 11|10   9|  8  |  7   |  6   |  5   |  4   |  3  |  2   |  1  |  0  |
// -----------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------
// sstatus
//-----------------------------------------------
// -----------------------------------------------------------------------------------------------------
// | SD |  WPRI  | MXR | SUM | WPRI |  XS  |  FS  | WPRI | SPP | WPRI | SPIE | UPIE | WPRI | SIE | UIE |
// -----------------------------------------------------------------------------------------------------
// | 31 |30    20| 19  | 18  |  17  |16  15|14  13|12   9|  8  |7    6|  5   |  4   |3    2|  1  |  0  |
// -----------------------------------------------------------------------------------------------------

always @(posedge clk_i)
begin
    if (rst_i)
    begin
        mstatus_r <= {19'b0, 2'b11, 3'b0, 1'b1, 3'b0};//MPP <= 2'b11(M-mode) MIE <= 1
    end
    else if (irq_taken_o)
    begin
        mstatus_r <= {mstatus_r[31:8], mstatus_r[3], mstatus_r[6:4], 1'b0, mstatus_r[2:0]};// MPIE <= MIE, MIE <= 0
    end
    else if (sys_jump_i & is_mret)
    begin
        mstatus_r <= {mstatus_r[31:8], mstatus_r[3], mstatus_r[6:4], 1'b1, mstatus_r[2:0]};// MPIE <= MIE, MIE <= 1
    end

    else if (is_csr_instr_i && csr_addr_i == `CSR_MSTATUS)
    begin
        mstatus_r <= {mstatus_r[31:8],updated_csr_data[3], mstatus_r[6:4], updated_csr_data[7], mstatus_r[2:0]};//only update PMIE MIE
    end
end

//-----------------------------------------------
// mie
//-----------------------------------------------
// --------------------------------------------------------------------------------------------
// | WPRI | MEIE | WPRI | SEIE | UEIE | MTIE | WPRI | STIE | UTIE | MSIE | WPRI | SSIE | USIE |
// --------------------------------------------------------------------------------------------
// |31  12|  11  |  10  |  9   |  8   |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |
// --------------------------------------------------------------------------------------------
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        mie_r <= 32'b0;
    end
    else if (is_csr_instr_i && csr_addr_i == `CSR_MIE)
    begin
        mie_r <= {20'b0, updated_csr_data[11], 3'b0, updated_csr_data[7], 3'b0, updated_csr_data[3], 3'b0};//only update MEIE MTIE MSIE
    end
end

//-----------------------------------------------
// mtvec
//-----------------------------------------------
// ---------------
// | BASE | MODE |
// ---------------
// |31   2|1    0|
// ---------------
assign mtvec_base = {mtvec_r[DATA_WIDTH-1 : 2], 2'b00};
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        mtvec_r <= 32'h0; // the entry point of timer ISR
    end
    else if (is_csr_instr_i && csr_addr_i == `CSR_MTVEC)
    begin
        mtvec_r <= updated_csr_data;
    end
end

//-----------------------------------------------
// mtvec
//-----------------------------------------------
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        mscratch_r <= 32'b0;
    end
    else if (is_csr_instr_i && csr_addr_i == `CSR_MSCRATCH)
    begin
        mscratch_r <= updated_csr_data;
    end
end

//-----------------------------------------------
// mepc
//-----------------------------------------------
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        mepc_r <= 32'b0;
    end
    else if (irq_taken_o)
    begin
        mepc_r <= nxt_unexec_PC_i;
    end
    else if (sys_jump_i & is_ecall)
    begin
        mepc_r <= sys_jump_pc_i;
    end
    else if (is_csr_instr_i && csr_addr_i == `CSR_MEPC)
    begin
        mepc_r <= updated_csr_data;
    end
end

//-----------------------------------------------
// mcause
//-----------------------------------------------
// ------------------------------------
// | Interrupt | Excption Code (WLRL) |
// ------------------------------------
// |    31     |30                   0|
// ------------------------------------
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        mcause_r <= 32'b0;
    end
    else if (irq_taken_o)
    begin
        mcause_r <= mcause_d;
    end
    else if (is_csr_instr_i && csr_addr_i == `CSR_MCAUSE)
    begin
        mcause_r <= updated_csr_data;
    end
end

always @(*)
begin
    if(     ext_irq_i & mie_r[11]) mcause_d = {1'b1, 27'b0, 4'd11};
    else if(tmr_irq_i & mie_r[ 7]) mcause_d = {1'b1, 27'b0, 4'd7 };
    else if(tmr_irq_i & mie_r[ 7]) mcause_d = {1'b1, 27'b0, 4'd3 };
    else if(exception_vld_i    ) mcause_d = {1'b0, 27'b0, exception_cause_i};
    else                         mcause_d = {1'b1, 27'b0, 4'd0 };
end

// TODO: mtval for exception handling

//-----------------------------------------------
// mip : read-only
//-----------------------------------------------
// --------------------------------------------------------------------------------------------
// | WPRI | MEIP | WPRI | SEIP | UEIP | MTIP | WPRI | STIP | UTIP | MSIP | WPRI | SSIP | USIP |
// --------------------------------------------------------------------------------------------
// |31  12|  11  |  10  |  9   |  8   |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |
// --------------------------------------------------------------------------------------------
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        mip_r <= 32'b0;
    end
    else
    begin
        //mip_MEIP <= ext_irq_i
        //mip_MTIP <= tmr_irq_i
        //mip_MSIP <= sft_irq_i
        mip_r <= {20'b0, ext_irq_i, 3'b0, tmr_irq_i, 3'b0, sft_irq_i, 3'b0};
    end
end

//-----------------------------------------------
// mcycle, mcycleh
//-----------------------------------------------
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        mcycle_r    <= 64'b0;
    end
    else if (is_csr_instr_i)
    begin
        case (csr_addr_i)
            `CSR_MCYCLE :
                mcycle_r[31 : 0] <= updated_csr_data;
            `CSR_MCYCLEH :
                mcycle_r[63: 32] <= updated_csr_data;
            //Note
            `CSR_CYCLE :
                mcycle_r[31 : 0] <= updated_csr_data;
            `CSR_CYCLEH :
                mcycle_r[63: 32] <= updated_csr_data;
        endcase
    end
    else
    begin
        mcycle_r <= mcycle_r + 1;
    end
end

// TODO: minstret, minstreth

// =============================================================================================
//  S-MODE SYSTEM Operations
//

//-----------------------------------------------
// mstatus
//-----------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
// | SD |  WPRI  | TSR | TW | TVM | MXR | SUM | MPRV |  XS  |  FS  | MPP | WPRI | SPP | MPIE | WPRI | SPIE | UPIE | MIE | WPRI | SIE | UIE |
// -----------------------------------------------------------------------------------------------------
// | 31 |30    23| 22  | 21 | 20  | 19  | 18  |  17  |16  15|14  13|12 11|10   9|  8  |  7   |  6   |  5   |  4   |  3  |  2   |  1  |  0  |
// -----------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------
// sstatus
//-----------------------------------------------
// -----------------------------------------------------------------------------------------------------
// | SD |  WPRI  | MXR | SUM | WPRI |  XS  |  FS  | WPRI | SPP | WPRI | SPIE | UPIE | WPRI | SIE | UIE |
// -----------------------------------------------------------------------------------------------------
// | 31 |30    20| 19  | 18  |  17  |16  15|14  13|12   9|  8  |7    6|  5   |  4   |3    2|  1  |  0  |
// -----------------------------------------------------------------------------------------------------
assign sstatus = {mstatus_r[31], 11'b0, mstatus_r[19:18], 1'b0, mstatus_r[16:13], 4'b0, mstatus_r[ 8], 2'b0, mstatus_r[ 5: 4], 2'b0, mstatus_r[ 1: 0]};

//-----------------------------------------------
// satp
//-----------------------------------------------
// ---------------------------------
// | MODE | ASID(WARL) | PPN(WARL) |
// ---------------------------------
// |  31  |30        22|21        0|
// ---------------------------------
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        satp_r <= 32'b0;
    end
    else if (is_csr_instr_i && csr_addr_i == `CSR_SATP)
    begin
        satp_r <= updated_csr_data;
    end
end



// =============================================================================================
//  U-MODE SYSTEM Operations
//
//-----------------------------------------------
// cycle, cycleh
//-----------------------------------------------
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        cycle_r <= 64'b0;
    end
    else
    begin
        cycle_r <= cycle_r + 1;
    end
end

// =============================================================================================
//  Operations and Multiplexer
//

always @( * )
begin
    case (csr_addr_i)
        `CSR_MSTATUS:
            csr_data = mstatus_r;
        `CSR_MIE:
            csr_data = mie_r;
        `CSR_MTVEC:
            csr_data = mtvec_r;

        `CSR_MSCRATCH:
            csr_data = mscratch_r;
        `CSR_MEPC:
            csr_data = mepc_r;
        `CSR_MCAUSE:
            csr_data = mcause_r;
        //`CSR_MTVAL:       csr_data = mtval_r;
        `CSR_MIP:
            csr_data = mip_r;

        `CSR_MCYCLE:
            csr_data = mcycle_r[31 : 0];
        `CSR_MCYCLEH:
            csr_data = mcycle_r[63: 32];

        //`CSR_MINSTRET:  csr_data = minstret_r[DATA_WIDTH-1 :0];
        //`CSR_MINSTRETH: csr_data = minstret_r[63:32];

        // `CSR_CYCLE:
        //     csr_data = cycle_r[31 : 0];
        // `CSR_CYCLEH:
        //     csr_data = cycle_r[63: 32];
        `CSR_CYCLE:
            csr_data = mcycle_r[31 : 0];
        `CSR_CYCLEH:
            csr_data = mcycle_r[63: 32];

        `CSR_MVENDORID:
            csr_data = mvendorid;
        `CSR_MARCHID:
            csr_data = marchid;
        `CSR_MIMPID:
            csr_data = mimpid;
        `CSR_MHARTID:
            csr_data = mhartid;

        `CSR_SSTATUS:
            csr_data = sstatus;
        `CSR_SATP:
            csr_data = satp_r;

        default :
            csr_data = 0;
    endcase
end

always @( * )
begin
    case (csr_op_i[1: 0])
        `CSR_RW:
            updated_csr_data = csr_inputB;
        `CSR_RS:
            updated_csr_data = csr_inputA | csr_inputB;
        `CSR_RC:
            updated_csr_data = csr_inputA & ~csr_inputB;
        default:
            updated_csr_data = csr_inputA;
    endcase
end

always @(posedge clk_i)
begin
    if (rst_i)
    begin 
        privilege_lvl_r <= 2'b11;//M-Mode
    end
    else
    begin
        if(is_mret)
        begin
            privilege_lvl_r <= mstatus_r[12:11]; //MPP
        end
        else if(is_sret)
        begin
            privilege_lvl_r <= {1'b0, mstatus_r[8]}; //SPP
        end
    end
end

// =============================================================================================
//  Output signals interface
//
assign csr_data_o = csr_data;
assign sys_jump_csr_data_o = is_mret ? mepc_r :
       is_ecall ? mtvec_base :
       32'b0;

// ------------------
// mstatus[3] == mstatus_MIE
// mie[11] == mie_MEIE mie[7] == mie_MTIE mie[3] == mie_MSIE
assign irq_taken_o = (mstatus_r[3] &&
                    ((ext_irq_i & mie_r[11]) || (tmr_irq_i & mie_r[7]) || (sft_irq_i & mie_r[3]) || exception_vld_i));


// ------------------
// mtvec[1:0] == 0 : MODE 0, set PC to BASE
// mtvec[1:0] == 1 : MODE 1, set PC to BASE + 4*casue
// mtvec[1:0] >= 2 : Reserved
assign PC_handler_o = (mtvec_r[1: 0] == 2'b00 || !mcause_d[31]) ? mtvec_base
       : mtvec_base + (mcause_d[30: 0] << 2);


//MMU
assign mmu_enable_o    = satp_r[31];
assign root_ppn_o      = satp_r[21: 0];
assign asid_o          = satp_r[30:22];
assign privilege_lvl_o = privilege_lvl_r;

endmodule   // csr_file
