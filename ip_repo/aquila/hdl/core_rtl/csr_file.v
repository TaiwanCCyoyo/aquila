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
    input  wire                    clk_i,
    input  wire                    rst_i,

    //stall
    input wire                     stall_i,

    // from Decode
    input  wire [11: 0]            csr_rd_addr_i,

    // to Fowarding
    output wire [DATA_WIDTH-1 : 0] csr_data_o,

    //from Memory_WriteBack_Pipeline
    input  wire                    csr_we_i,
    input  wire [11: 0]            csr_we_addr_i,
    input  wire [DATA_WIDTH-1 : 0] csr_we_data_i,


    // Interrupt requests.
    // input  wire                    ext_irq_i, //Machine external interrupt
    // input  wire                    tmr_irq_i, //Machine timer interrupt
    // input  wire                    sft_irq_i, //Machine software interrupt
    output wire                    irq_taken_o,
    output wire [ADDR_WIDTH-1 : 0] PC_handler_o,
    input  wire [ADDR_WIDTH-1 : 0] nxt_unwb_PC_i,

    // System Jump operation
    input  wire                    sys_jump_i,
    input  wire [ 1: 0]            sys_jump_csr_addr_i,
    output wire                    sys_jump_o,
    output wire [DATA_WIDTH-1 : 0] sys_jump_csr_data_o,

    //
    output wire [ 1: 0]            privilege_lvl_o,
    output wire                    tvm_o,
        
    //MMU     
    output wire                    mmu_enable_o,
    output wire [ 1: 0]            ld_st_privilege_lvl_o,
    output wire [21: 0]            root_ppn_o,
    output wire [ 8: 0]            asid_o,
    output wire                    mxr_o,
    output wire                    sum_o,

    // Exception requests     
    input wire                     exp_vld_i,
    input wire                     exp_isinterrupt_i,
    input wire [ 3: 0]             exp_cause_i,
    input wire [31: 0]             exp_tval_i,

    //To Memory
    output wire [ 3: 0]            mstatus_ie_o, //{MIE, WPRI,SIE ,UIE}
    output wire [31: 0]            mie_o,

    //
    input  wire [31: 0]            mip_update_i
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

`define M_MODE 2'b11 
`define S_MODE 2'b01
`define U_MODE 2'b00

`define CSR_RW 2'b01
`define CSR_RS 2'b10
`define CSR_RC 2'b11

// Exception
// -------------------------------------------------
// | Exception Code |       Descript               |
// -------------------------------------------------
// |       0        |Instruction address misaligned|
// -------------------------------------------------
// |       1        |Instruction  access fault     |
// -------------------------------------------------
// |       2        |Illegal instruction           |
// -------------------------------------------------
// |       3        |Breakpoint                    |
// -------------------------------------------------
// |       4        |Load address misaligned       |
// -------------------------------------------------
// |       5        |Load access fault             |
// -------------------------------------------------
// |       6        |Store/AMO address misaligned  |
// -------------------------------------------------
// |       7        |Store/AMO access fault        |
// -------------------------------------------------
// |       8        |Environment call from U-mode  |
// -------------------------------------------------
// |       9        |Environment call from S-mode  |
// -------------------------------------------------
// |      10        |Reserved                      |
// -------------------------------------------------
// |      11        |Environment call from M-mode  |
// -------------------------------------------------
// |      12        |Instruction page fault        |
// -------------------------------------------------
// |      13        |Load page fault               |
// -------------------------------------------------
// |      14        |Reserved                      |
// -------------------------------------------------
// |      15        |Store/AMO page fault          |
// -------------------------------------------------

// Interrupt
// -------------------------------------------------
// | Exception Code |       Descript               |
// -------------------------------------------------
// |       0        |User software interrupt       |
// -------------------------------------------------
// |       1        |Supervisor software interrupt |
// -------------------------------------------------
// |       2        |Reserved                      |
// -------------------------------------------------
// |       3        |Machine software interrupt    |
// -------------------------------------------------
// |       4        |User timer interrupt          |
// -------------------------------------------------
// |       5        |Supervisor timer interrupt    |
// -------------------------------------------------
// |       6        |Reserved                      |
// -------------------------------------------------
// |       7        |Machine timer interrupt       |
// -------------------------------------------------
// |       8        |User external interrupt       |
// -------------------------------------------------
// |       9        |Supervisor external interrupt |
// -------------------------------------------------
// |      10        |Reserved                      |
// -------------------------------------------------
// |      11        |Machine external interrupt    |
// -------------------------------------------------
//==============================================================================================
// Wire and Reg 
//==============================================================================================
// =============================================================================================
//  CSRs implementations
//
//M-Mode register
reg  [DATA_WIDTH-1 : 0] mstatus_r;
//reg  [DATA_WIDTH-1 : 0] misa_r;
reg  [DATA_WIDTH-1 : 0] mie_r;
reg  [DATA_WIDTH-1 : 0] mip_r;
reg  [DATA_WIDTH-1 : 0] mtvec_r;
reg  [DATA_WIDTH-1 : 0] mscratch_r;
reg  [DATA_WIDTH-1 : 0] mepc_r;
reg  [DATA_WIDTH-1 : 0] mcause_r, mcause_d;
reg  [DATA_WIDTH-1 : 0] mtval_r;  // for exception
reg  [DATA_WIDTH-1 : 0] medeleg_r;  // for exception
reg  [DATA_WIDTH-1 : 0] mideleg_r;  // for interrupt
reg  [63           : 0] mcycle_r;
//reg  [63 : 0] minstret_r;
wire [DATA_WIDTH-1 : 0] mvendorid = 0;  // Non-commercial implementation, so return 0
wire [DATA_WIDTH-1 : 0] marchid   = 0;
wire [DATA_WIDTH-1 : 0] mimpid    = 0;
wire [DATA_WIDTH-1 : 0] mhartid   = HART_ID;

//S-Mode register
wire [DATA_WIDTH-1 : 0] sstatus;
wire [DATA_WIDTH-1 : 0] sip;
wire [DATA_WIDTH-1 : 0] sie;
reg  [DATA_WIDTH-1 : 0] satp_r;
reg  [DATA_WIDTH-1 : 0] stvec_r;
reg  [DATA_WIDTH-1 : 0] sscratch_r;
reg  [DATA_WIDTH-1 : 0] sepc_r;
reg  [DATA_WIDTH-1 : 0] scause_r, scause_d;
reg  [DATA_WIDTH-1 : 0] stval_r;  // for exception
reg  [DATA_WIDTH-1 : 0] sedeleg_r;  // for exception
reg  [DATA_WIDTH-1 : 0] sideleg_r;  // for interrupt

//U-Mode register
reg  [63           : 0] cycle_r;

//
reg  [1            : 0] privilege_lvl_r;

wire [DATA_WIDTH-1 : 0] mtvec_base;
wire [DATA_WIDTH-1 : 0] stvec_base;
//wire retired = instr_valid & !stall;

wire is_mret = (sys_jump_csr_addr_i == `M_MODE);
wire is_sret = (sys_jump_csr_addr_i == `S_MODE);
wire is_uret = (sys_jump_csr_addr_i == `U_MODE); // noy implement

reg  [DATA_WIDTH-1 : 0] csr_data;

//Interrupt
reg          trap_to_M;
reg          irq_taken;
reg  [31: 0] PC_handler;

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
reg stall_delay;
always@(posedge clk_i)
begin
    stall_delay <= stall_i;
end

always @(posedge clk_i)
begin
    if (rst_i)
    begin
        mstatus_r <= {19'b0, `M_MODE, 7'b0, 1'b1, 3'b0};//MPP <= 2'b11(M-mode)
    end
    else if(stall_delay)
    begin
        mstatus_r <= mstatus_r;
    end
    else if (irq_taken)
    begin
        if(trap_to_M)
        begin
            mstatus_r <= {mstatus_r[31:13], privilege_lvl_r, mstatus_r[10:8], mstatus_r[3], mstatus_r[6:4], 1'b0, mstatus_r[2:0]};// MPIE <= MIE, MIE <= 0
        end
        else
        begin
            mstatus_r <= {mstatus_r[31:9], privilege_lvl_r[0], mstatus_r[7:6], mstatus_r[1], mstatus_r[4:2], 1'b0, mstatus_r[0]};// SPIE <= SIE, SIE <= 0
        end
    end
    else if (sys_jump_i)
    begin
        if(is_mret)
        begin
            mstatus_r <= {mstatus_r[31:8], 1'b1, mstatus_r[6:4], mstatus_r[7], mstatus_r[2:0]};// MIE <= MPIE, MPIE <= 1
        end
        else if(is_sret)
        begin
            mstatus_r <= {mstatus_r[31:6], 1'b1,mstatus_r[4:2], mstatus_r[5],mstatus_r[0]};// SIE <= SPIE, SPIE <= 1
        end
    end
    else if (csr_we_i)
    begin
        if(csr_we_addr_i == `CSR_MSTATUS)
        begin
            //mstatus_r <= {mstatus_r[31:8],csr_we_data_i[7], mstatus_r[6:4], csr_we_data_i[3], mstatus_r[2:0]};//only update PMIE MIE
            mstatus_r <= csr_we_data_i;
        end
        else if(csr_we_addr_i == `CSR_SSTATUS)
        begin
            mstatus_r <= {csr_we_data_i[31], mstatus_r[30:20], csr_we_data_i[19:18], mstatus_r[17], csr_we_data_i[16:13], mstatus_r[12:9], csr_we_data_i[8], mstatus_r[7:6], csr_we_data_i[5:4], mstatus_r[3:2], csr_we_data_i[1:0]};
        end
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
    else if (csr_we_i && csr_we_addr_i == `CSR_MIE)
    begin
        mie_r <= csr_we_data_i;
    end
end

//-----------------------------------------------
// mip
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
    else if (csr_we_i && csr_we_addr_i == `CSR_MIE)
    begin
        //only SEIP, UEIP, STIP, UTIP, SSIP, USIP writable
        mip_r <= {22'b0, csr_we_data_i[9:8], 2'b0, csr_we_data_i[5:4], 2'b0, csr_we_data_i[1:0]};
    end
    else
    begin
        mip_r <= mip_update_i;
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
    else if (csr_we_i && csr_we_addr_i == `CSR_MTVEC)
    begin
        mtvec_r <= csr_we_data_i;
    end
end

//-----------------------------------------------
// mscratch
//-----------------------------------------------
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        mscratch_r <= 32'b0;
    end
    else if (csr_we_i && csr_we_addr_i == `CSR_MSCRATCH)
    begin
        mscratch_r <= csr_we_data_i;
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
    else if (irq_taken && trap_to_M)
    begin
        mepc_r <= nxt_unwb_PC_i;
    end
    else if (csr_we_i && csr_we_addr_i == `CSR_MEPC)
    begin
        mepc_r <= csr_we_data_i;
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
    else if (irq_taken)
    begin
        mcause_r <= mcause_d;
    end
    else if (csr_we_i && csr_we_addr_i == `CSR_MCAUSE)
    begin
        mcause_r <= csr_we_data_i;
    end
end

always @(*)
begin
    mcause_d = mcause_r;
    if(irq_taken && trap_to_M)
    begin
        mcause_d = {exp_isinterrupt_i, 27'b0, exp_cause_i};     
    end
end



//-----------------------------------------------
// mtval
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
        mtval_r <= 32'b0;
    end
    else if(exp_vld_i && !exp_isinterrupt_i && trap_to_M)
    begin
        if( (exp_cause_i >= 'd0  && exp_cause_i <= 'd1)  ||
            (exp_cause_i >= 'd4  && exp_cause_i <= 'd7)  ||
            (exp_cause_i >= 'd12 && exp_cause_i <= 'd13) ||
            (exp_cause_i == 'd15))
        begin
            mtval_r <= exp_tval_i;
        end
    end
    else if (csr_we_i && csr_we_addr_i == `CSR_MTVAL)
    begin
        mtval_r <= csr_we_data_i;
    end
end

//-----------------------------------------------
// medeleg
//------------------------------------------------

always @(posedge clk_i)
begin
    if (rst_i)
    begin
        medeleg_r <= 32'b0;
    end
    else if (csr_we_i && csr_we_addr_i == `CSR_MEDELEG)
    begin
        medeleg_r <= csr_we_data_i;
    end
end

//-----------------------------------------------
// mideleg
//------------------------------------------------
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        mideleg_r <= 32'b0;
    end
    else if (csr_we_i && csr_we_addr_i == `CSR_MIDELEG)
    begin
        mideleg_r <= csr_we_data_i;
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
    else if (csr_we_i)
    begin
        case (csr_we_addr_i)
            `CSR_MCYCLE :
                mcycle_r[31 : 0] <= csr_we_data_i;
            `CSR_MCYCLEH :
                mcycle_r[63: 32] <= csr_we_data_i;
            //Note
            `CSR_CYCLE :
                mcycle_r[31 : 0] <= csr_we_data_i;
            `CSR_CYCLEH :
                mcycle_r[63: 32] <= csr_we_data_i;
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
// sstatus
//-----------------------------------------------
// -----------------------------------------------------------------------------------------------------
// | SD |  WPRI  | MXR | SUM | WPRI |  XS  |  FS  | WPRI | SPP | WPRI | SPIE | UPIE | WPRI | SIE | UIE |
// -----------------------------------------------------------------------------------------------------
// | 31 |30    20| 19  | 18  |  17  |16  15|14  13|12   9|  8  |7    6|  5   |  4   |3    2|  1  |  0  |
// -----------------------------------------------------------------------------------------------------
assign sstatus = {mstatus_r[31], 11'b0, mstatus_r[19:18], 1'b0, mstatus_r[16:13], 4'b0, mstatus_r[ 8], 2'b0, mstatus_r[ 5: 4], 2'b0, mstatus_r[ 1: 0]};

//-----------------------------------------------
// sip
//-----------------------------------------------
// ----------------------------------------------------------------
// | WPRI | SEIP | UEIP | WPRI | STIP | UTIP | WPRI | SSIP | USIP |
// ----------------------------------------------------------------
// |31  12|  9   |  8   | 7  6 |  5   |  4   | 3  2 |  1   |  0   |
// ----------------------------------------------------------------
assign sip = {20'b0, mip_r[9:8], 2'b0, mip_r[5:4], 2'b0, mip_r[1:0]};

//-----------------------------------------------
// sie
//-----------------------------------------------
// ----------------------------------------------------------------
// | WPRI | SEIE | UEIE | WPRI | STIE | UTIE | WPRI | SSIE | USIE |
// ----------------------------------------------------------------
// |31  10|  9   |  8   | 7  6 |  5   |  4   | 3  2 |  1   |  0   |
// ----------------------------------------------------------------
assign sie = {20'b0, mie_r[9:8], 2'b0, mie_r[5:4], 2'b0, mie_r[1:0]};

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
    else if (csr_we_i && csr_we_addr_i == `CSR_SATP)
    begin
        satp_r <= csr_we_data_i;
    end
end

//-----------------------------------------------
// stvec
//-----------------------------------------------
// ---------------
// | BASE | MODE |
// ---------------
// |31   2|1    0|
// ---------------
assign stvec_base = {stvec_r[DATA_WIDTH-1 : 2], 2'b00};
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        stvec_r <= 32'h0; // the entry point of timer ISR
    end
    else if (csr_we_i && csr_we_addr_i == `CSR_STVEC)
    begin
        stvec_r <= csr_we_data_i;
    end
end

//-----------------------------------------------
// sscratch
//-----------------------------------------------
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        sscratch_r <= 32'b0;
    end
    else if (csr_we_i && csr_we_addr_i == `CSR_SSCRATCH)
    begin
        sscratch_r <= csr_we_data_i;
    end
end

//-----------------------------------------------
// sepc
//-----------------------------------------------
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        sepc_r <= 32'b0;
    end
    else if (irq_taken && !trap_to_M)
    begin
        sepc_r <= nxt_unwb_PC_i;
    end
    else if (csr_we_i && csr_we_addr_i == `CSR_SEPC)
    begin
        sepc_r <= csr_we_data_i;
    end
end

//-----------------------------------------------
// scause
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
        scause_r <= 32'b0;
    end
    else if (irq_taken)
    begin
        scause_r <= scause_d;
    end
    else if (csr_we_i && csr_we_addr_i == `CSR_SCAUSE)
    begin
        scause_r <= csr_we_data_i;
    end
end

always @(*)
begin
    scause_d = scause_r;
    if(irq_taken && !trap_to_M)
    begin
        scause_d = {exp_isinterrupt_i, 27'b0, exp_cause_i};
    end
end

//-----------------------------------------------
// stval
//-----------------------------------------------
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        stval_r <= 32'b0;
    end
    else if(exp_vld_i && !exp_isinterrupt_i && !trap_to_M)
    begin
        if( (exp_cause_i >= 'd0  && exp_cause_i <= 'd1)  ||
            (exp_cause_i >= 'd4  && exp_cause_i <= 'd7)  ||
            (exp_cause_i >= 'd12 && exp_cause_i <= 'd13) ||
            (exp_cause_i == 'd15))
        begin
            stval_r <= exp_tval_i;
        end
    end
    else if (csr_we_i && csr_we_addr_i == `CSR_STVAL)
    begin
        stval_r <= csr_we_data_i;
    end
end

//-----------------------------------------------
// sedeleg
//------------------------------------------------

always @(posedge clk_i)
begin
    if (rst_i)
    begin
        sedeleg_r <= 32'b0;
    end
    else if (csr_we_i && csr_we_addr_i == `CSR_SEDELEG)
    begin
        sedeleg_r <= csr_we_data_i;
    end
end

//-----------------------------------------------
// sideleg
//------------------------------------------------
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        sideleg_r <= 32'b0;
    end
    else if (csr_we_i && csr_we_addr_i == `CSR_SIDELEG)
    begin
        sideleg_r <= csr_we_data_i;
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
    case (csr_rd_addr_i)
        `CSR_MSTATUS:
            csr_data = mstatus_r;
        `CSR_MIE:
            csr_data = mie_r;
        `CSR_MIP:
            csr_data = mip_r;
        `CSR_MTVEC:
            csr_data = mtvec_r;
        `CSR_MSCRATCH:
            csr_data = mscratch_r;
        `CSR_MEPC:
            csr_data = mepc_r;
        `CSR_MCAUSE:
            csr_data = mcause_r;
        `CSR_MTVAL:       
            csr_data = mtval_r;
        `CSR_MCYCLE:
            csr_data = mcycle_r[31 : 0];
        `CSR_MCYCLEH:
            csr_data = mcycle_r[63: 32];
        //`CSR_MINSTRET:  csr_data = minstret_r[DATA_WIDTH-1 :0];
        //`CSR_MINSTRETH: csr_data = minstret_r[63:32];
        `CSR_MVENDORID:
            csr_data = mvendorid;
        `CSR_MARCHID:
            csr_data = marchid;
        `CSR_MIMPID:
            csr_data = mimpid;
        `CSR_MHARTID:
            csr_data = mhartid;
        `CSR_MEDELEG:
            csr_data = medeleg_r;
        `CSR_MIDELEG:
            csr_data = mideleg_r;


        `CSR_SSTATUS:
            csr_data = sstatus;
        `CSR_MIE:
            csr_data = sie;
        `CSR_SIP:
            csr_data = sip;
        `CSR_SATP:
            csr_data = satp_r;
        `CSR_STVEC:
            csr_data = stvec_r;
        `CSR_SSCRATCH:
            csr_data = sscratch_r;
        `CSR_SEPC:
            csr_data = sepc_r;
        `CSR_SCAUSE:
            csr_data = scause_r;
        `CSR_STVAL:
            csr_data = stval_r;
        `CSR_SEDELEG:
            csr_data = sedeleg_r;
        `CSR_SIDELEG:
            csr_data = sideleg_r;

        `CSR_CYCLE:                       // `CSR_CYCLE: 
            csr_data = mcycle_r[31 : 0];  //     csr_data = cycle_r[31 : 0];      
        `CSR_CYCLEH:                      // `CSR_CYCLEH:  
            csr_data = mcycle_r[63: 32];  //     csr_data = cycle_r[63: 32];        

        default :
            csr_data = 0;
    endcase
end

// =============================================================================================
// 
//

// ------------------
// mstatus[3] == mstatus_MIE
// mstatus[2] == mstatus_SIE
// mstatus[1] == mstatus_UIE

// mie[11] == mie_MEIE mie[7] == mie_MTIE mie[3] == mie_MSIE
// mie[ 9] == mie_SEIE mie[5] == mie_STIE mie[1] == mie_SSIE
// mie[ 8] == mie_UEIE mie[4] == mie_UTIE mie[0] == mie_USIE
always@(*) begin
    irq_taken         = 0;
    trap_to_M         = 1;

    if(exp_vld_i)
    begin
        irq_taken = 1;
        if(exp_isinterrupt_i)
        begin
            trap_to_M = (privilege_lvl_r == `M_MODE)?1:~mideleg_r[exp_cause_i];
        end
        else
        begin
            trap_to_M = (privilege_lvl_r == `M_MODE)?1:~medeleg_r[exp_cause_i];
        end
    end
end

// ------------------
// mtvec[1:0] == 0 : MODE 0, set PC to BASE
// mtvec[1:0] == 1 : MODE 1, set PC to BASE + 4*casue
// mtvec[1:0] >= 2 : Reserved


always@(*)
begin
    if(trap_to_M)
    begin
        if(mtvec_r[1: 0] == 2'b00 || !mcause_d[31])
        begin
            PC_handler = mtvec_base;
        end
        else
        begin
            PC_handler = mtvec_base + (mcause_d[30: 0] << 2);
        end
    end
    else
    begin
        if(mtvec_r[1: 0] == 2'b00)
        begin
            PC_handler = stvec_base;
        end
        else
        begin
            PC_handler = stvec_base + (scause_d[30: 0] << 2);
        end
    end
end


always @(posedge clk_i)
begin
    if (rst_i)
    begin 
        privilege_lvl_r <= `M_MODE;//M-Mode
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
        else if(irq_taken)
        begin
            privilege_lvl_r <= (trap_to_M)?`M_MODE:`S_MODE;
        end
    end
end

// =============================================================================================
//  Output signals interface
//
assign csr_data_o          = csr_data;

assign sys_jump_o          = sys_jump_i;
assign sys_jump_csr_data_o = is_mret ? mepc_r :
                             is_sret ? sepc_r :
                             32'b0;

assign irq_taken_o  = irq_taken;
assign PC_handler_o = PC_handler;

//
assign privilege_lvl_o       = privilege_lvl_r;
assign tvm_o                 = mstatus_r[20];//TVM

//MMU
assign mmu_enable_o          = satp_r[31] && privilege_lvl_r != `M_MODE;//M-Mode
assign root_ppn_o            = satp_r[21: 0];
assign asid_o                = satp_r[30:22];
assign ld_st_privilege_lvl_o = (mstatus_r[17])? mstatus_r[12:11] : privilege_lvl_r;//MPRV
assign mxr_o                 = mstatus_r[19];
assign sum_o                 = mstatus_r[18];

//To memory
assign mie_o                 = mie_r;
assign mstatus_ie_o          = mstatus_r[3:0];

endmodule   // csr_file
