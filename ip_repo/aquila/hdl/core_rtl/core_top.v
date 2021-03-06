`timescale 1ns / 1ps
// =============================================================================
//  Program : core_top.v
//  Author  : Jin-you Wu
//  Date    : Dec/19/2018
// -----------------------------------------------------------------------------
//  Description:
//  This is the top-level Aquila IP wrapper for an AXI-based application
//  processor SoC.
//
//  The pipeline architecture of Aquila 1.0 was based on the Microblaze-
//  compatible processor, KernelBlaze, originally designed by Dong-Fong Syu.
//  This file, core_top.v, was derived from CPU.v of KernelBlaze by Dong-Fong
//  on Sep/09/2017.
//
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Oct/16-17/2019, by Chun-Jen Tsai:
//    Unified the memory accesses scheme of the processor core, pushing the
//    address decoding of different memory devices to the SoC level.  Change
//    the initial program counter address from a "PARAMETER' to an input
//    signal, which comes from a system register at the SoC-level.
//
//  Nov/29/2019, by Chun-Jen Tsai:
//    Change the overall pipeline architecture of Aquila. Merges the pipeline
//    register moduels of Fetch, Decode, and Execute stages into the respective
//    moudules.
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

module core_top #(
    parameter ADDR_WIDTH        = 32,
    parameter DATA_WIDTH        = 32,
    parameter HART_ID           = 0,
    parameter COND_ENTRY_NUM    = 24,
    parameter COND_DATA_WIDTH   = 32,
    parameter UNCOND_ENTRY_NUM  = 20,
    parameter UNCOND_DATA_WIDTH = 32
)
(
    // System signals.
    input                       clk_i,
    input                       rst_i,
    input                       stall_i,

    // Program counter address at reset.
    input  [ADDR_WIDTH-1 : 0]   init_pc_addr_i,

    // Instruction memory port.
    input  [DATA_WIDTH-1 : 0]   instruction_i,
    input                       instruction_ready_i,
    output [ADDR_WIDTH-1 : 0]   instruction_addr_o,
    output                      instruction_req_o,

    // Data memory port.
    input  [DATA_WIDTH-1 : 0]   data_read_i,
    input                       data_ready_i,
    output [DATA_WIDTH-1 : 0]   data_write_o,
    output [ADDR_WIDTH-1 : 0]   data_addr_o,
    output                      data_rw_o,      // 0: data read, 1: data write.
    output [DATA_WIDTH/8-1 : 0] data_byte_enable_o,
    output                      data_req_o,
    // output                      data_strobe_o,

    // Interrupt sources.
    input                       ext_irq_i,
    input                       tmr_irq_i,
    input                       sft_irq_i
);

// System operations.
wire                    sys_jump;
wire [11 : 0]           sys_jump_csr_addr;
wire [DATA_WIDTH-1 : 0] sys_jump_csr_data;

// Pipeline control.
wire flush2fet, flush2dec;
wire stall_from_hazard;

// Forwarding unit.
wire [DATA_WIDTH-1 : 0] rs1_fwd, rs2_fwd;

// Program counter.
wire [ADDR_WIDTH-1 : 0] pc;

// Fetch stage signals.
wire [DATA_WIDTH-1 : 0] fet_instr2dec, fet_pc2dec;
wire                    fet_valid2dec;

// Register File.
wire [DATA_WIDTH-1 : 0] rs1_data2dec, rs2_data2dec;

// Decode stage signals.
wire [ 4 : 0]           dec_rs1_addr, dec_rs2_addr;
wire                    dec_illegal_instr;
wire [DATA_WIDTH-1 : 0] dec_pc2exe, dec_imm2exe,
                        dec_rs1_data2fwd, dec_rs2_data2fwd;
wire                    dec_regfile_we2exe, dec_we2exe, dec_re2exe,
                        dec_load_ext_sel2exe, instruction_pc_vld_dec2exe;
wire [ 4 : 0]           dec_rd_addr2exe, dec_rs1_addr2fwd, dec_rs2_addr2fwd,
                        dec_csr_imm2csr;
wire [ 1 : 0]           dec_inA_sel2exe, dec_inB_sel2exe, dec_input_sel2exe;
wire [ 2 : 0]           dec_regfile_sel2exe, dec_operation_sel2exe;
wire [11 : 0]           dec_csr_addr2csr;

wire                    alu_muldiv_sel2exe, shift_sel2exe, is_branch2exe,
                        is_jal2exe, is_jalr2exe;

// Execute stage signals.
wire                    exe_branch_taken;
wire                    stall_from_exe;

wire [DATA_WIDTH-1 : 0] exe_pc2pc, exe_branch_target_addr;
wire                    exe_we, exe_re;
wire                    exe_regfile_we2mem_wb, exe_load_ext_sel2mem_wb;
wire [DATA_WIDTH-1 : 0] exe_rs2_data2mem, exe_addr2mem, exe_p_data;
wire [ 4 : 0]           exe_rd_addr2mem_wb;
wire [ 1 : 0]           exe_input_sel2mem;
wire [ 2 : 0]           exe_regfile_input_sel2mem_wb;

// Memory Access.
wire [DATA_WIDTH-1 : 0] data2mem_wb;
wire [ 3 : 0]           byte_write_sel;
wire                    memory_alignment_exception;

// Memory Writeback Pipeline.
wire [DATA_WIDTH-1 : 0] rd_data2wb;
wire                    rd_we2wb;
wire [ 4 : 0]           rd_addr2wb;

// Branch Prediction Unit & Uncond Branch BHT.
localparam COND_ADDR_WIDTH   = $clog2(COND_ENTRY_NUM),
           UNCOND_ADDR_WIDTH = $clog2(UNCOND_ENTRY_NUM);

wire                    cond_branch_hit_IF, cond_branch_hit_ID,
                        cond_branch_hit_EXE,
                        cond_branch_result_IF, cond_branch_result_ID,
                        cond_branch_result_EXE,
                        uncond_branch_hit_IF, uncond_branch_hit_ID,
                        uncond_branch_hit_EXE,
                        cond_branch_misprediction;

wire [DATA_WIDTH-1 : 0] cond_branch_target_addr, uncond_branch_target_addr;

wire irq_taken;
wire [DATA_WIDTH-1 : 0] PC_handler;

wire [ 1: 0] priv_lvl;

// ----------------------
// fowarding <-> decode
// ----------------------
wire [11: 0]            csr_addr_dec2fwd;
wire [DATA_WIDTH-1 : 0] csr_data_dec2fwd;

// ----------------------
// fowarding <-> execute
// ----------------------
wire [DATA_WIDTH-1 : 0] csr_fwd;

// ----------------------
// csr <-> exe
// ----------------------

// ----------------------
// csr <-> mem
// ----------------------
wire [3 : 0] mstatus_ie_csr2mem;
wire [31: 0] mie_csr2mem;


// ----------------------
// csr <-> mmu
// ----------------------
wire         mmu_enable_csr2mmu;
wire [21: 0] root_ppn_csr2mmu;
wire [ 8: 0] asid_csr2mmu;
wire [ 1: 0] ld_st_privilege_lvl_csr2mmu;
wire         mxr_csr2mmu;
wire         sum_csr2mmu;

// ----------------------
// instruction <-> mmu
// ----------------------
wire         i_req_vld_fet2mmu;
wire [31: 0] i_req_vaddr_fet2mmu;

wire         i_rtrn_vld_mmu2fet;
wire [31: 0] i_rtrn_data_mmu2fet;
wire [31: 0] i_rtrn_vaddr_mmu2fet;
// wire [31: 0] i_rtrn_paddr_mmu2fet;

wire         i_exp_vld_mmu2fet;
wire [ 3: 0] i_exp_cause_mmu2fet;
wire [31: 0] i_exp_tval_mmu2fet;

// ----------------------
// data <-> mmu
// ----------------------
wire         d_req_vld_mem2mmu;        
wire [31: 0] d_req_vaddr_mem2mmu;      
wire [31: 0] d_req_data_mem2mmu;
wire         d_req_rw_mem2mmu;         
wire [ 3: 0] d_req_byte_enable_mem2mmu;

wire         d_rtrn_vld_mmu2mem;
wire         d_exp_vld_mmu2mem;
wire [ 3: 0] d_exp_cause_mmu2mem;
wire [31: 0] d_exp_tval_mmu2mem;

// ----------------------
// instruction <-> decode
// ----------------------
wire         exp_vld_fet2dec;
wire [ 3: 0] exp_cause_fet2dec;
wire [31: 0] exp_tval_fet2dec;

// ----------------------
// decode <-> execute
// ----------------------
wire         csr_we_dec2exe;

wire         sys_jump_dec2exe;
wire [ 1: 0] sys_jump_csr_addr_dec2exe;
wire         exp_vld_dec2exe;
wire [ 3: 0] exp_cause_dec2exe;
wire [31: 0] exp_tval_dec2exe;

wire          sfence_dec2exe;
wire          sfence_type_dec2exe;

// ----------------------
// execute <-> memory_access
// ----------------------
wire         sys_jump_exe2mem;
wire [ 1: 0] sys_jump_csr_addr_exe2mem;
wire         exp_vld_exe2mem;
wire [ 3: 0] exp_cause_exe2mem;
wire [31: 0] exp_tval_exe2mem;
wire [31: 0] instruction_pc_exe2mem;
wire         instruction_pc_vld_exe2mem;
wire         csr_we_exe2mem_wb;
wire [31: 0] csr_we_addr_exe2mem_wb;
wire [31: 0] csr_we_data_exe2mem_wb;


wire          sfence_exe2mem;
wire          sfence_type_exe2mem;
wire [31: 0]  rs1_data_exe2mem;

// ------------------------
// memory_access <-> mem_wb
// ------------------------
wire         sys_jump_mem2mem_wb;
wire [ 1: 0] sys_jump_csr_addr_mem2mem_wb;
wire [31: 0] d_rtrn_data_mmu2mem_wb;
wire         exp_vld_mem2mem_wb;
wire         exp_isinterrupt_mem2wb;
wire [ 3: 0] exp_cause_mem2mem_wb;
wire [31: 0] exp_tval_mem2mem_wb;
wire [31: 0] instruction_pc_mem2mem_wb;
wire         instruction_pc_vld_mem2wb;

// ------------------------
// mem_wb <-> csr
// ------------------------
wire         sys_jump_mem_wb2csr;
wire [ 1: 0] sys_jump_csr_addr_mem_wb2csr;
wire         exp_vld_mem_wb2csr;
wire         exp_isinterrupt_wb2csr;
wire [ 3: 0] exp_cause_mem_wb2csr;
wire [31: 0] exp_tval_mem_wb2csr;
wire [31: 0] instruction_pc_mem_wb2csr;
wire         instruction_pc_vld_wb2csr;
wire         csr_we_mem_wb2csr;
wire [31: 0] csr_we_addr_mem_wb2csr;
wire [31: 0] csr_we_data_mem_wb2csr;
wire [31: 0] mip_update_wb2csr;

// ----------------------
// csr <-> decode
// ----------------------
wire         tvm_csr2dec;
wire [31: 0]  csr_data2dec;


// ----------------------
// controller <-> mmu
// ----------------------
wire          flush2mmu;
wire          flush_type2mmu;
wire [31: 0]  fulsh_vaddr2mmu;
wire [31: 0]  fulsh_asid2mmu;

// ----------------------
// controller <-> memory
// ----------------------
wire          sfence_mem2cont;
wire          sfence_type_mem2cont;
wire [31: 0]  rs1_data_mem2cont;
wire [31: 0]  rs2_data_mem2cont;

// Exception
// wire mem_memory_alignment_exception2mem_wb;
// wire exe_mem_exception_vld2mem_wb, mem_wb_exception_vld2csr;
// wire [3:0] exe_mem_exception_cause2mem_wb, mem_wb_exception_cause2csr;



// =============================================================================
// Finite state machine that controls the processor pipeline stalls.
//
localparam i_IDLE = 0, i_NEXT = 1, i_WAIT = 2;
// localparam d_IDLE = 0, d_WAIT = 1;
reg[ 1: 0] iS, iS_nxt, dS, dS_nxt;

// -----------------------------------------------------------------------------
// The stall signals:
//    # stall_pipeline signal will stall all pipeline registers
//    # stall_from_hazard only stall the Program_Counter and the Fetch stages
//
wire stall_pipeline, stall_for_instr_fetch, stall_for_data_fetch;

assign stall_for_instr_fetch = (iS_nxt == i_WAIT) || (iS == i_IDLE);

 always @(posedge clk_i)
 begin
     if (rst_i)
         iS <= i_IDLE;
     else
         iS <= iS_nxt;
 end

 always @(*)
 begin
     case (iS)
        i_IDLE:
            iS_nxt = i_NEXT;
        i_NEXT: // CJ Tsai 0227_2020: I-fetch when I-memory ready.
            if (i_rtrn_vld_mmu2fet || i_exp_vld_mmu2fet || flush2mmu)
                iS_nxt = i_NEXT;
            else
                iS_nxt = i_WAIT;
        i_WAIT:
            if (i_rtrn_vld_mmu2fet || i_exp_vld_mmu2fet)
                iS_nxt = i_NEXT; // one-cycle delay
            else
                iS_nxt = i_WAIT;
        default:
            iS_nxt = i_IDLE;
     endcase
 end
//  always @(posedge clk_i)
//  begin
//      if (rst_i)
//          iS <= i_NEXT;
//      else
//          iS <= iS_nxt;
//  end

//  always @(*)
//  begin
//      case (iS)
//          i_NEXT: // CJ Tsai 0227_2020: I-fetch when I-memory ready.
//              if (instruction_ready_i || i_exp_vld_mmu2fet)
//                  iS_nxt = i_NEXT;
//              else
//                  iS_nxt = i_WAIT;
//          i_WAIT:
//              if (instruction_ready_i || i_exp_vld_mmu2fet)
//                  iS_nxt = i_NEXT; // one-cycle delay
//              else
//                  iS_nxt = i_WAIT;
//      endcase
//  end

// -----------------------------------------------------------------------------
// Output instruction/data request signals to mmu

assign i_req_vld_fet2mmu   = (iS == i_NEXT)||(iS == i_WAIT && (!i_rtrn_vld_mmu2fet && !i_exp_vld_mmu2fet));
assign i_req_vaddr_fet2mmu = pc;
// assign instruction_req_o   = !(iS == i_WAIT && (i_exp_vld_mmu2fet));
// assign instruction_addr_o = pc;

////////////////////////////////////////////////////////////////////////////////
//                        the following are submodules                        //
////////////////////////////////////////////////////////////////////////////////

// =============================================================================
pipeline_control Pipeline_Control(
    // from the Decode stage
    .rs1_addr_i(dec_rs1_addr),
    .rs2_addr_i(dec_rs2_addr),
    .illegal_instr_i(dec_illegal_instr),

    // from Decode_Execute_Pipeline
    .rd_addr_DEC_EXE_i(dec_rd_addr2exe),
    .is_load_instr_DEC_EXE_i(dec_re2exe),
    .cond_branch_hit_EXE_i(cond_branch_hit_EXE),
    .uncond_branch_hit_EXE_i(uncond_branch_hit_EXE),

    // from Execute
    .branch_taken_i(exe_branch_taken),
    .cond_branch_misprediction_i(cond_branch_misprediction),

    // System Jump operation
    .sys_jump_i(sys_jump),

    // to Fetch stage
    .flush2fet_o(flush2fet),

    // to Decode_Execute_Pipeline
    .flush2dec_o(flush2dec),

    // that flushes Execute_Memory_Pipeline
    .flush2exe_o(flush2exe),

    // that flushes Memory_Writeback_Pipeline
    .flush2mem_o(flush2mem_wb),


    // to Program_Counter, Fetch stage
    .stall_from_hazard_o(stall_from_hazard),

    //from memory
    .sfence_i(sfence_mem2cont),
    .sfence_type_i(sfence_type_mem2cont),  
    .rs1_data_i(rs1_data_mem2cont),  
    .rs2_data_i(rs2_data_mem2cont),

    //to mmu
    .tlb_fulsh_o(flush2mmu),
    .tlb_flush_type_o(flush_type2mmu),
    .tlb_fulsh_vaddr_o(fulsh_vaddr2mmu),
    .tlb_fulsh_asid_o(fulsh_asid2mmu)
);

// =============================================================================
forwarding_unit Forwarding_Unit(
    // from Decode_Execute_Pipeline
    .rs1_addr_i(dec_rs1_addr2fwd),
    .rs2_addr_i(dec_rs2_addr2fwd),
    .csr_addr_i(csr_addr_dec2fwd),
    .rs1_data_i(dec_rs1_data2fwd),
    .rs2_data_i(dec_rs2_data2fwd),
    .csr_data_i(csr_data_dec2fwd),

    // from Execute_Memory_Pipeline
    .regfile_we_EXE_MEM_i(exe_regfile_we2mem_wb),
    .rd_addr_EXE_MEM_i(exe_rd_addr2mem_wb),
    .regfile_input_sel_EXE_MEM_i(exe_regfile_input_sel2mem_wb),
    .p_data_EXE_MEM_i(exe_p_data),
    .csr_addr_EXE_MEM_i(csr_we_addr_exe2mem_wb),
    .csr_we_EXE_MEM_i(csr_we_exe2mem_wb),
    .csr_data_EXE_MEM_i(csr_we_data_exe2mem_wb),

    // from Memory_Writeback_Pipeline
    .regfile_we_MEM_WB_i(rd_we2wb),
    .rd_addr_MEM_WB_i(rd_addr2wb),
    .rd_data_MEM_WB_i(rd_data2wb),
    .csr_addr_MEM_WB_i(csr_we_addr_mem_wb2csr),
    .csr_we_MEM_WB_i(csr_we_mem_wb2csr),
    .csr_data_MEM_WB_i(csr_we_data_mem_wb2csr),

    // to Execute, CSR, Execute_Memory_Pipeline
    .rs1_fwd_o(rs1_fwd),
    .rs2_fwd_o(rs2_fwd),
    .csr_fwd_o(csr_fwd)
);

// =============================================================================
//      # conditional branch predictor
//      # unconditional branch predictor
//
bpu #(
    .ENTRY_NUM(COND_ENTRY_NUM),
    .ADDR_WIDTH(COND_ADDR_WIDTH),
    .DATA_WIDTH(COND_DATA_WIDTH)
)
Branch_Prediction_Unit(
    // Top-level system signals
    .clk_i(clk_i),
    .rst_i(rst_i),
    .stall_i(stall_for_instr_fetch | stall_for_data_fetch | stall_from_exe),

    // from Program_Counter
    .pc_IF_i(pc),

    // from Decode_Execute_Pipeline
    .is_cond_branch_i(is_branch2exe),
    .pc_EXE_i(dec_pc2exe),

    // from Execute
    .branch_taken_i(exe_branch_taken),
    .branch_target_addr_i(exe_branch_target_addr),
    .cond_branch_misprediction_i(cond_branch_misprediction),

    // to Program_Counter
    .cond_branch_hit_o(cond_branch_hit_IF),
    .cond_branch_result_o(cond_branch_result_IF),
    .cond_branch_target_addr_o(cond_branch_target_addr)
);

uncond_BHT #(
    .ENTRY_NUM(UNCOND_ENTRY_NUM),
    .ADDR_WIDTH(UNCOND_ADDR_WIDTH),
    .DATA_WIDTH(UNCOND_DATA_WIDTH)
)
JAL_BHT(
    // Top-level system signals
    .clk_i(clk_i),
    .rst_i(rst_i),
    .stall_i(stall_for_instr_fetch | stall_for_data_fetch | stall_from_exe),

    // from Program_Counter
    .pc_IF_i(pc),

    // from Decode_Execute_Pipeline
    .is_uncond_branch_i(is_jal2exe),
    .pc_EXE_i(dec_pc2exe),

    // from Execute
    .branch_target_addr_i(exe_branch_target_addr),

    // to Program_Counter
    .uncond_branch_hit_o(uncond_branch_hit_IF),
    .uncond_branch_target_addr_o(uncond_branch_target_addr)
);

// =============================================================================
regfile Register_File(
    // Top-level system signals
    .clk_i(clk_i),
    .rst_i(rst_i),

    // from Decode
    .rs1_addr_i(dec_rs1_addr),
    .rs2_addr_i(dec_rs2_addr),

    // from Memory_Writeback_Pipeline
    .regfile_we_i(rd_we2wb),
    .rd_addr_i(rd_addr2wb),
    .rd_data_i(rd_data2wb),

    // to Decode_Execute_Pipeline
    .rs1_data_o(rs1_data2dec),
    .rs2_data_o(rs2_data2dec)
);

// =============================================================================
program_counter Program_Counter(
    // Top-level system signals
    .clk_i(clk_i),
    .rst_i(rst_i),

    // Program counter address at reset
    .init_pc_addr_i(init_pc_addr_i),

    // Interrupt
    .irq_taken_i(irq_taken),
    .PC_handler_i(PC_handler),

    // that stall Program_Counter
    .stall_i((stall_for_instr_fetch | stall_for_data_fetch | stall_from_exe) || stall_from_hazard),

    // from Cond_Branch_Predictor
    .cond_branch_hit_IF_i(cond_branch_hit_IF),
    .cond_branch_result_IF_i(cond_branch_result_IF),
    .cond_branch_target_addr_i(cond_branch_target_addr),

    // from Uncond_Branch_BHT
    .uncond_branch_hit_IF_i(uncond_branch_hit_IF),
    .uncond_branch_target_addr_i(uncond_branch_target_addr),

    // System Jump operation
    .sys_jump_i(sys_jump),
    .sys_jump_csr_data_i(sys_jump_csr_data),

    // frome Decode_Execute_Pipeline
    .cond_branch_hit_EXE_i(cond_branch_hit_EXE),
    .cond_branch_result_EXE_i(cond_branch_result_EXE),
    .uncond_branch_hit_EXE_i(uncond_branch_hit_EXE),

    // from Execute
    .cond_branch_misprediction_i(cond_branch_misprediction),
    .branch_taken_i(exe_branch_taken),
    .branch_target_addr_i(exe_branch_target_addr),
    .branch_restore_addr_i(exe_pc2pc),

    // to Fetch stage, i-cache
    .pc_o(pc),

    .sfence_i(flush2mmu),
    .dec_pc2exe_i(dec_pc2exe)
);

// =============================================================================
fetch Fetch(
    // Top-level system signals
    .clk_i(clk_i),
    .rst_i(rst_i),
    .stall_i((stall_for_instr_fetch | stall_for_data_fetch | stall_from_exe) || stall_from_hazard),

    // from Pipeline_Control and CSR_File
    .flush_i(flush2fet || irq_taken),

    // from Cond_Branch_Predictor
    .cond_branch_hit_IF_i(cond_branch_hit_IF),
    .cond_branch_result_IF_i(cond_branch_result_IF),

    // from Uncond_Branch_BHT
    .uncond_branch_hit_IF_i(uncond_branch_hit_IF),

    // from i-memory
    .instruction_i(i_rtrn_data_mmu2fet),

    // from Program_Counter
    .pc_i(i_rtrn_vaddr_mmu2fet),
    // .ppc_i(i_rtrn_paddr_mmu2fet),

    // to the Decode Stage
    .instruction_o(fet_instr2dec),

    // to the Execute Stage
    .cond_branch_hit_ID_o(cond_branch_hit_ID),
    .cond_branch_result_ID_o(cond_branch_result_ID),
    .uncond_branch_hit_ID_o(uncond_branch_hit_ID),
    .pc_o(fet_pc2dec),
    .instr_valid_o(fet_valid2dec),
    .pc_vld_o(pc_vld_fet2dec),

    //exception from mmu
    .exp_from_mmu_vld_i(i_exp_vld_mmu2fet),
    .exp_from_mmu_cause_i(i_exp_cause_mmu2fet),
    .exp_from_mmu_tval_i(i_exp_tval_mmu2fet),

    //To Decode
    .exp_vld_o(exp_vld_fet2dec),
    .exp_cause_o(exp_cause_fet2dec),
    .exp_tval_o(exp_tval_fet2dec)
);

// fetch Fetch(
//     // Top-level system signals
//     .clk_i(clk_i),
//     .rst_i(rst_i),
//     .stall_i((stall_for_instr_fetch | stall_for_data_fetch | stall_from_exe) || stall_from_hazard),

//     // from Pipeline_Control and CSR_File
//     .flush_i(flush2fet || irq_taken),

//     // from Cond_Branch_Predictor
//     .cond_branch_hit_IF_i(cond_branch_hit_IF),
//     .cond_branch_result_IF_i(cond_branch_result_IF),

//     // from Uncond_Branch_BHT
//     .uncond_branch_hit_IF_i(uncond_branch_hit_IF),

//     // from i-memory
//     .instruction_i(instruction_i),

//     // from Program_Counter
//     .pc_i(pc),
//     .ppc_i(pc),

//     // to the Decode Stage
//     .instruction_o(fet_instr2dec),

//     // to the Execute Stage
//     .cond_branch_hit_ID_o(cond_branch_hit_ID),
//     .cond_branch_result_ID_o(cond_branch_result_ID),
//     .uncond_branch_hit_ID_o(uncond_branch_hit_ID),
//     .pc_o(fet_pc2dec),
//     .instr_valid_o(fet_valid2dec),

//     //exception from mmu
//     .exp_from_mmu_vld_i(i_exp_vld_mmu2fet),
//     .exp_from_mmu_cause_i(i_exp_cause_mmu2fet),
//     .exp_from_mmu_tval_i(i_exp_tval_mmu2fet),

//     //To Decode
//     .exp_vld_o(exp_vld_fet2dec),
//     .exp_cause_o(exp_cause_fet2dec),
//     .exp_tval_o(exp_tval_fet2dec)
// );

// =============================================================================
decode Decode(
    // Top-level system signals
    .clk_i(clk_i),
    .rst_i(rst_i),
    .stall_i(stall_for_instr_fetch | stall_for_data_fetch | stall_from_exe),

    // Processor pipeline flush signal.
    .flush_i(flush2dec || irq_taken),

    // Signals from the Fetch Stage.
    .pc_i(fet_pc2dec),
    .pc_vld_i(pc_vld_fet2dec),
    .instruction_i(fet_instr2dec),
    .instr_valid_i(fet_valid2dec),
    .cond_branch_hit_ID_i(cond_branch_hit_ID),
    .cond_branch_result_ID_i(cond_branch_result_ID),
    .uncond_branch_hit_ID_i(uncond_branch_hit_ID),

    // Instruction operands from the Register File. To be forwarded.
    .rs1_data_i(rs1_data2dec),
    .rs2_data_i(rs2_data2dec),

    // Operand register IDs to the Register File and the Pipeline Controller
    .rs1_addr_o(dec_rs1_addr),
    .rs2_addr_o(dec_rs2_addr),

    // illegal instruction
    .illegal_instr_o(dec_illegal_instr),

    // to Execute
    .pc_o(dec_pc2exe),
    .imm_o(dec_imm2exe),
    .inputA_sel_o(dec_inA_sel2exe),
    .inputB_sel_o(dec_inB_sel2exe),
    .operation_sel_o(dec_operation_sel2exe),
    .alu_muldiv_sel_o(alu_muldiv_sel2exe),
    .shift_sel_o(shift_sel2exe),
    .cond_branch_hit_EXE_o(cond_branch_hit_EXE),
    .cond_branch_result_EXE_o(cond_branch_result_EXE),
    .is_jalr_o(is_jalr2exe),
    .is_csr_instr_o(csr_we_dec2exe),
    .csr_imm_o(dec_csr_imm2csr),
    .instr_pc_valid_o(instruction_pc_vld_dec2exe),

    // to Execute, Cond_Branch_Predictor
    .is_branch_o(is_branch2exe),

    // to Execute, Pipeline_Control, Uncond_Branch_BHT
    .is_jal_o(is_jal2exe),

    // to CSR
    .csr_addr_o(dec_csr_addr2csr),

    //From CSR
    .csr_data_i(csr_data2dec),

    // to Execute_Memory_Pipeline
    .regfile_we_o(dec_regfile_we2exe),
    .regfile_input_sel_o(dec_regfile_sel2exe),
    .mem_we_o(dec_we2exe),
    .mem_re_o(dec_re2exe),
    .mem_input_sel_o(dec_input_sel2exe),
    .mem_load_ext_sel_o(dec_load_ext_sel2exe),

    // to Pipeline_Control
    .uncond_branch_hit_EXE_o(uncond_branch_hit_EXE),

    // to Forwarding_Unit, Pipeline_Control and Execute_Memory_Pipeline
    .rd_addr_o(dec_rd_addr2exe),

    // to Forwarding_Unit
    .rs1_addr2fwd_o(dec_rs1_addr2fwd),
    .rs2_addr2fwd_o(dec_rs2_addr2fwd),
    .rs1_data2fwd_o(dec_rs1_data2fwd),
    .rs2_data2fwd_o(dec_rs2_data2fwd),
    .csr_addr2fwd_o(csr_addr_dec2fwd),
    .csr_data2fwd_o(csr_data_dec2fwd),


    //From CSR
    .tvm_i(tvm_csr2dec),
    .privilege_lvl_i(priv_lvl),

    // System Jump operation
    .sys_jump_o(sys_jump_dec2exe),
    .sys_jump_csr_addr_o(sys_jump_csr_addr_dec2exe),

    //Supervisor Instructions
    .sfence_o(sfence_dec2exe),
    .sfence_type_o(sfence_type_dec2exe),//0=>rs1=x0,1=>rs1!=x0

    //Exception from Fetch
    .exp_vld_i(exp_vld_fet2dec),
    .exp_cause_i(exp_cause_fet2dec),
    .exp_tval_i(exp_tval_fet2dec),

    //Exception to Execute
    .exp_vld_o(exp_vld_dec2exe),
    .exp_cause_o(exp_cause_dec2exe),
    .exp_tval_o(exp_tval_dec2exe)
);

// =============================================================================

execute Execute(
    // Top-level system signals
    .clk_i(clk_i),
    .rst_i(rst_i),

    // Processor pipeline flush signal.
    .flush_i(flush2exe || irq_taken),

    // From the Program Counter unit.
    .pc_i(dec_pc2exe),

    // Pipeline stall signal.
    .stall_i(stall_for_instr_fetch | stall_for_data_fetch | stall_from_exe),

    // Signals from the Decode stage.
    .imm_i(dec_imm2exe),
    .inputA_sel_i(dec_inA_sel2exe),
    .inputB_sel_i(dec_inB_sel2exe),
    .operation_sel_i(dec_operation_sel2exe),
    .alu_muldiv_sel_i(alu_muldiv_sel2exe),
    .shift_sel_i(shift_sel2exe),
    .is_branch_i(is_branch2exe),
    .is_jal_i(is_jal2exe),
    .is_jalr_i(is_jalr2exe),
    .cond_branch_hit_EXE_i(cond_branch_hit_EXE),
    .cond_branch_result_EXE_i(cond_branch_result_EXE),

    .regfile_we_i(dec_regfile_we2exe),
    .regfile_input_sel_i(dec_regfile_sel2exe),
    .mem_we_i(dec_we2exe),
    .mem_re_i(dec_re2exe),
    .mem_input_sel_i(dec_input_sel2exe),
    .mem_load_ext_sel_i(dec_load_ext_sel2exe),
    .rd_addr_i(dec_rd_addr2exe),

    .csr_imm_i(dec_csr_imm2csr),
    .csr_we_i(csr_we_dec2exe),
    .csr_we_addr_i(csr_addr_dec2fwd),

    // Signals from the Forwarding Unit.
    .rs1_data_i(rs1_fwd),
    .rs2_data_i(rs2_fwd),
    .csr_data_i(csr_fwd),

    // Signal to the Program Counter Unit.
    .pc_o(exe_pc2pc),

    // Signal to the Program Counter unit.
    .branch_target_addr_o(exe_branch_target_addr),

    // Singnals to the Pipeline Control and the Branch Prediction units.
    .branch_taken_o(exe_branch_taken),
    .cond_branch_misprediction_o(cond_branch_misprediction),

    // Pipeline stall signal generator, activated when executing
    //    multicycle mul, div and rem instructions.
    .stall_from_exe_o(stall_from_exe),

    // Signals to D-Cache.
    .mem_we_o(exe_we),
    .mem_re_o(exe_re),

    // Signals to Memory Access unit.
    .rs2_data_o(exe_rs2_data2mem),
    .mem_addr_o(exe_addr2mem),
    .mem_input_sel_o(exe_input_sel2mem),

    // Signals to the Memory Writeback and the Forwarding Units.
    .regfile_we_o(exe_regfile_we2mem_wb),
    .regfile_input_sel_o(exe_regfile_input_sel2mem_wb),
    .rd_addr_o(exe_rd_addr2mem_wb),
    .p_data_o(exe_p_data),
    .csr_we_o(csr_we_exe2mem_wb),
    .csr_we_addr_o(csr_we_addr_exe2mem_wb),
    .csr_we_data_o(csr_we_data_exe2mem_wb),

    // Signals to Memory Writeback.
    .mem_load_ext_sel_o(exe_load_ext_sel2mem_wb),

    // System Jump operation
    .sys_jump_i(sys_jump_dec2exe),
    .sys_jump_csr_addr_i(sys_jump_csr_addr_dec2exe),
    .sys_jump_o(sys_jump_exe2mem),
    .sys_jump_csr_addr_o(sys_jump_csr_addr_exe2mem),

    //Supervisor Instructions
    .sfence_i(sfence_dec2exe),
    .sfence_o(sfence_exe2mem),
    .sfence_type_i(sfence_type_dec2exe),
    .sfence_type_o(sfence_type_exe2mem),//0=>rs1=x0,1=>rs1!=x0
    .rs1_data_o(rs1_data_exe2mem),

    //Exception from Decode
    .exp_vld_i(exp_vld_dec2exe),
    .exp_cause_i(exp_cause_dec2exe),
    .exp_tval_i(exp_tval_dec2exe),
    .instruction_pc_vld_i(instruction_pc_vld_dec2exe),

    //Exception to Memory
    .exp_vld_o(exp_vld_exe2mem),
    .exp_cause_o(exp_cause_exe2mem),
    .exp_tval_o(exp_tval_exe2mem),
    .instruction_pc_o(instruction_pc_exe2mem),
    .instruction_pc_vld_o(instruction_pc_vld_exe2mem)
);

// =============================================================================
wire                     req_done_mem2wb;        
wire                     buffer_exp_vld_mem2wb;  
wire [3 : 0]             buffer_exp_cause_mem2wb;
wire [31: 0]             buffer_exp_tval_mem2wb; 
wire [DATA_WIDTH-1 : 0]  buffer_data_mem2wb;

wire [2: 0]             regfile_input_sel_mem2wb;
wire                    regfile_we_mem2wb;
wire [4: 0]             rd_addr_mem2wb;
wire                    mem_load_ext_sel_mem2wb;
wire [1: 0]             mem_addr_alignment_mem2wb;
wire [DATA_WIDTH-1 : 0] p_data_mem2wb;

wire                    exp_vld_mem2wb;
wire [ 3: 0]            exp_cause_mem2wb;
wire [31: 0]            exp_tval_mem2wb;
wire                    sys_jump_mem2wb;
wire [ 1: 0]            sys_jump_csr_addr_mem2wb;
wire [31: 0]            instruction_pc_mem2wb;
wire                    csr_we_mem2wb;
wire [31: 0]            csr_we_addr_mem2wb;
wire [31: 0]            csr_we_data_mem2wb;
wire [DATA_WIDTH-1 : 0] mem_data_mem2wb;
wire [31: 0]            mip_update_mem2wb;


memory_access Memory_Access(
    .clk_i(clk_i),
    .rst_i(rst_i),

    .stall_i(stall_for_instr_fetch | stall_for_data_fetch | stall_from_exe),
    .flush_i(flush2mem_wb || irq_taken),
    .stall_for_data_fetch_o(stall_for_data_fetch),

    // from Execute_Memory_Pipeline
    .unaligned_data_i(exe_rs2_data2mem),      // store value
    .mem_addr_i(exe_addr2mem),
    .mem_input_sel_i(exe_input_sel2mem),
    .exe_we_i(exe_we),
    .exe_re_i(exe_re),

    // to D-memory
    .data_o(d_req_data_mem2mmu),                        // data_write
    .byte_write_sel_o(d_req_byte_enable_mem2mmu),
    .req_vld_o(d_req_vld_mem2mmu),    
    .req_vaddr_o(d_req_vaddr_mem2mmu),       
    .req_rw_o(d_req_rw_mem2mmu),     

    .d_rtrn_vld_i(d_rtrn_vld_mmu2mem),
    .d_rtrn_data_i(d_rtrn_data_mmu2mem_wb),

    // System Jump operation
    .sys_jump_i(sys_jump_exe2mem),
    .sys_jump_csr_addr_i(sys_jump_csr_addr_exe2mem),
    .sys_jump_o(sys_jump_mem2mem_wb),
    .sys_jump_csr_addr_o(sys_jump_csr_addr_mem2mem_wb),

    // Exception signal
    //.memory_alignment_exception_o(memory_alignment_exception),

    // .req_done_o(req_done_mem2wb),       
    // .buffer_exp_vld_o(buffer_exp_vld_mem2wb),
    // .buffer_exp_cause_o(buffer_exp_cause_mem2wb),
    // .buffer_exp_tval_o(buffer_exp_tval_mem2wb), 
    // .buffer_data_o(buffer_data_mem2wb),    

    //exception from execute
    .exp_vld_i(exp_vld_exe2mem),
    .exp_cause_i(exp_cause_exe2mem),
    .exp_tval_i(exp_tval_exe2mem),
    .instruction_pc_i(instruction_pc_exe2mem),
    .instruction_pc_vld_i(instruction_pc_vld_exe2mem),

    //exception from mmu
    .exp_from_mmu_vld_i(d_exp_vld_mmu2mem),
    .exp_from_mmu_cause_i(d_exp_cause_mmu2mem),
    .exp_from_mmu_tval_i(d_exp_tval_mmu2mem),

    //exception to writeback
    .exp_vld_o(exp_vld_mem2mem_wb),
    .exp_isinterrupt_o(exp_isinterrupt_mem2wb),
    .exp_cause_o(exp_cause_mem2mem_wb),
    .exp_tval_o(exp_tval_mem2mem_wb),
    .instruction_pc_o(instruction_pc_mem2mem_wb),
    .instruction_pc_vld_o(instruction_pc_vld_mem2wb),

    .regfile_we_i(exe_regfile_we2mem_wb),
    .rd_addr_i(exe_rd_addr2mem_wb),
    .regfile_input_sel_i(exe_regfile_input_sel2mem_wb),
    .mem_load_ext_sel_i(exe_load_ext_sel2mem_wb),
    .mem_addr_alignment_i(exe_addr2mem[1: 0]),
    .p_data_i(exe_p_data),
    .csr_we_i(csr_we_exe2mem_wb),
    .csr_we_addr_i(csr_we_addr_exe2mem_wb),
    .csr_we_data_i(csr_we_data_exe2mem_wb),

    //
    .sfence_i(sfence_exe2mem),
    .sfence_type_i(sfence_type_exe2mem),  
    .rs1_data_i(rs1_data_exe2mem),  
    .rs2_data_i(exe_rs2_data2mem),

    //
    .sfence_o(sfence_mem2cont),
    .sfence_type_o(sfence_type_mem2cont),  
    .rs1_data_o(rs1_data_mem2cont),  
    .rs2_data_o(rs2_data_mem2cont),

    //
    .regfile_input_sel_o(regfile_input_sel_mem2wb),
    .regfile_we_o(regfile_we_mem2wb),
    .rd_addr_o(rd_addr_mem2wb),
    .mem_load_ext_sel_o(mem_load_ext_sel_mem2wb),
    .mem_addr_alignment_o(mem_addr_alignment_mem2wb),
    .p_data_o(p_data_mem2wb),
    .csr_we_o(csr_we_mem2wb),
    .csr_we_addr_o(csr_we_addr_mem2wb),
    .csr_we_data_o(csr_we_data_mem2wb),
    .mem_data_o(mem_data_mem2wb),
    .mip_update_o(mip_update_mem2wb),

    // Interrupt requests.
    .m_ext_irq_i(ext_irq_i), //Machine external interrupt
    .m_tmr_irq_i(tmr_irq_i), //Machine timer interrupt
    .m_sft_irq_i(sft_irq_i), //Machine software interrupt
    .s_ext_irq_i(), //Supervisor external interrupt
    .s_tmr_irq_i(), //Supervisor timer interrupt
    .s_sft_irq_i(), //Supervisor software interrupt
    .u_ext_irq_i(), //User external interrupt
    .u_tmr_irq_i(), //User timer interrupt
    .u_sft_irq_i(), //User software interrupt

    //From CSR
    .mstatus_ie_i(mstatus_ie_csr2mem), //{MIE, WPRI, SIE, UIE}
    .mie_i(mie_csr2mem)
);

// memory_access Memory_Access(
//     .clk_i(clk_i),
//     .rst_i(rst_i),

//     .stall_i(stall_for_instr_fetch | stall_for_data_fetch | stall_from_exe),
//     .flush_i(flush2mem_wb || irq_taken),
//     .stall_for_data_fetch_o(stall_for_data_fetch),

//     // from Execute_Memory_Pipeline
//     .unaligned_data_i(exe_rs2_data2mem),      // store value
//     .mem_addr_i(exe_addr2mem),
//     .mem_input_sel_i(exe_input_sel2mem),
//     .exe_we_i(exe_we),
//     .exe_re_i(exe_re),

//     // to D-memory
//     .data_o(data_write_o),                        // data_write
//     .byte_write_sel_o(data_byte_enable_o),
//     .req_vld_o(data_req_o),    
//     .req_vaddr_o(data_addr_o),       
//     .req_rw_o(data_rw_o), 


//     .d_rtrn_vld_i(data_ready_i),
//     .d_rtrn_data_i(data_read_i),

//     // System Jump operation
//     .sys_jump_i(sys_jump_exe2mem),
//     .sys_jump_csr_addr_i(sys_jump_csr_addr_exe2mem),
//     .sys_jump_o(sys_jump_mem2mem_wb),
//     .sys_jump_csr_addr_o(sys_jump_csr_addr_mem2mem_wb),

//     // Exception signal
//     //.memory_alignment_exception_o(memory_alignment_exception),

//     // .req_done_o(req_done_mem2wb),       
//     // .buffer_exp_vld_o(buffer_exp_vld_mem2wb),
//     // .buffer_exp_cause_o(buffer_exp_cause_mem2wb),
//     // .buffer_exp_tval_o(buffer_exp_tval_mem2wb), 
//     // .buffer_data_o(buffer_data_mem2wb),    

//     //exception from execute
//     .exp_vld_i(exp_vld_exe2mem),
//     .exp_cause_i(exp_cause_exe2mem),
//     .exp_tval_i(exp_tval_exe2mem),
//     .instruction_pc_i(instruction_pc_exe2mem),
//     .instruction_pc_vld_i(instruction_pc_vld_exe2mem),

//     //exception from mmu
//     .exp_from_mmu_vld_i(d_exp_vld_mmu2mem),
//     .exp_from_mmu_cause_i(d_exp_cause_mmu2mem),
//     .exp_from_mmu_tval_i(d_exp_tval_mmu2mem),

//     //exception to writeback
//     .exp_vld_o(exp_vld_mem2mem_wb),
//     .exp_isinterrupt_o(exp_isinterrupt_mem2wb),
//     .exp_cause_o(exp_cause_mem2mem_wb),
//     .exp_tval_o(exp_tval_mem2mem_wb),
//     .instruction_pc_o(instruction_pc_mem2mem_wb),
//     .instruction_pc_vld_o(instruction_pc_vld_mem2wb),

//     .regfile_we_i(exe_regfile_we2mem_wb),
//     .rd_addr_i(exe_rd_addr2mem_wb),
//     .regfile_input_sel_i(exe_regfile_input_sel2mem_wb),
//     .mem_load_ext_sel_i(exe_load_ext_sel2mem_wb),
//     .mem_addr_alignment_i(exe_addr2mem[1: 0]),
//     .p_data_i(exe_p_data),
//     .csr_we_i(csr_we_exe2mem_wb),
//     .csr_we_addr_i(csr_we_addr_exe2mem_wb),
//     .csr_we_data_i(csr_we_data_exe2mem_wb),

//     //
//     .regfile_input_sel_o(regfile_input_sel_mem2wb),
//     .regfile_we_o(regfile_we_mem2wb),
//     .rd_addr_o(rd_addr_mem2wb),
//     .mem_load_ext_sel_o(mem_load_ext_sel_mem2wb),
//     .mem_addr_alignment_o(mem_addr_alignment_mem2wb),
//     .p_data_o(p_data_mem2wb),
//     .csr_we_o(csr_we_mem2wb),
//     .csr_we_addr_o(csr_we_addr_mem2wb),
//     .csr_we_data_o(csr_we_data_mem2wb),
//     .mem_data_o(mem_data_mem2wb),
//     .mip_update_o(mip_update_mem2wb),

//     // Interrupt requests.
//     .m_ext_irq_i(ext_irq_i), //Machine external interrupt
//     .m_tmr_irq_i(tmr_irq_i), //Machine timer interrupt
//     .m_sft_irq_i(sft_irq_i), //Machine software interrupt
//     .s_ext_irq_i(), //Supervisor external interrupt
//     .s_tmr_irq_i(), //Supervisor timer interrupt
//     .s_sft_irq_i(), //Supervisor software interrupt
//     .u_ext_irq_i(), //User external interrupt
//     .u_tmr_irq_i(), //User timer interrupt
//     .u_sft_irq_i(), //User software interrupt

//     //From CSR
//     .mstatus_ie_i(mstatus_ie_csr2mem), //{MIE, WPRI, SIE, UIE}
//     .mie_i(mie_csr2mem)
// );

// =============================================================================
writeback Writeback(
    // Top-level system signals
    .clk_i(clk_i),
    .rst_i(rst_i),

    // to RegisterFile, Forwarding_Unit
    .rd_we_o(rd_we2wb),
    .rd_addr_o(rd_addr2wb),
    .rd_data_o(rd_data2wb),

    // System Jump operation
    .sys_jump_i(sys_jump_mem2mem_wb),
    .sys_jump_csr_addr_i(sys_jump_csr_addr_mem2mem_wb),
    .sys_jump_o(sys_jump_mem_wb2csr),
    .sys_jump_csr_addr_o(sys_jump_csr_addr_mem_wb2csr),  

    //Exception From Memory
    .exp_vld_i(exp_vld_mem2mem_wb),
    .exp_isinterrupt_i(exp_isinterrupt_mem2wb),
    .exp_cause_i(exp_cause_mem2mem_wb),
    .exp_tval_i(exp_tval_mem2mem_wb),
    .instruction_pc_i(instruction_pc_mem2mem_wb),
    .instruction_pc_vld_i(instruction_pc_vld_mem2wb),

    //To CSR
    .csr_we_o(csr_we_mem_wb2csr),
    .csr_we_addr_o(csr_we_addr_mem_wb2csr),
    .csr_we_data_o(csr_we_data_mem_wb2csr),
    .exp_vld_o(exp_vld_mem_wb2csr),
    .exp_isinterrupt_o(exp_isinterrupt_wb2csr),
    .exp_cause_o(exp_cause_mem_wb2csr),
    .exp_tval_o(exp_tval_mem_wb2csr),
    .instruction_pc_o(instruction_pc_mem_wb2csr),
    .instruction_pc_vld_o(instruction_pc_vld_wb2csr),
    .mip_update_o(mip_update_wb2csr),

    //From Memory
    .regfile_input_sel_i(regfile_input_sel_mem2wb),
    .regfile_we_i(regfile_we_mem2wb),
    .rd_addr_i(rd_addr_mem2wb),
    .mem_load_ext_sel_i(mem_load_ext_sel_mem2wb),
    .mem_addr_alignment_i(mem_addr_alignment_mem2wb),
    .p_data_i(p_data_mem2wb),
    .csr_we_i(csr_we_mem2wb),
    .csr_we_addr_i(csr_we_addr_mem2wb),
    .csr_we_data_i(csr_we_data_mem2wb),
    .mem_data_i(mem_data_mem2wb),
    .mip_update_i(mip_update_mem2wb)

);

// =============================================================================
reg [31: 0] nxt_unwb_PC;
always@(*) begin
    if(instruction_pc_vld_wb2csr)
        nxt_unwb_PC = instruction_pc_mem_wb2csr;
    else if(instruction_pc_vld_mem2wb)
        nxt_unwb_PC = instruction_pc_mem2mem_wb;
    else if(instruction_pc_vld_exe2mem)
        nxt_unwb_PC = instruction_pc_exe2mem;
    else if(instruction_pc_vld_dec2exe)
        nxt_unwb_PC = dec_pc2exe;
    else if(fet_valid2dec)
        nxt_unwb_PC = fet_pc2dec;
    else
        nxt_unwb_PC = i_rtrn_vaddr_mmu2fet;
        // nxt_unwb_PC = pc;
end


csr_file #( .HART_ID(HART_ID) )
CSR(
    // Top-level system signals
    .clk_i(clk_i),
    .rst_i(rst_i),

    //
    .stall_i(stall_for_instr_fetch | stall_for_data_fetch | stall_from_exe),

    // from Decode
    .csr_rd_addr_i(dec_csr_addr2csr),

    // to Decode
    .csr_data_o(csr_data2dec),

    //from Memory_WriteBack_Pipeline
    .csr_we_i(csr_we_mem_wb2csr),
    .csr_we_addr_i(csr_we_addr_mem_wb2csr),
    .csr_we_data_i(csr_we_data_mem_wb2csr),

    // Interrupt
    // .ext_irq_i(ext_irq_i),
    // .tmr_irq_i(tmr_irq_i),
    // .sft_irq_i(sft_irq_i),
    .irq_taken_o(irq_taken),
    .PC_handler_o(PC_handler),
    .nxt_unwb_PC_i(nxt_unwb_PC),

    // System Jump operation
    .sys_jump_i(sys_jump_mem_wb2csr),
    .sys_jump_csr_addr_i(sys_jump_csr_addr_mem_wb2csr),
    .sys_jump_o(sys_jump),
    .sys_jump_csr_data_o(sys_jump_csr_data),

    //
    .privilege_lvl_o(priv_lvl),
    .tvm_o(tvm_csr2dec),

    //MMU
    .mmu_enable_o(mmu_enable_csr2mmu),
    .ld_st_privilege_lvl_o(ld_st_privilege_lvl_csr2mmu),
    .root_ppn_o(root_ppn_csr2mmu),
    .asid_o(asid_csr2mmu),
    .mxr_o(mxr_csr2mmu),
    .sum_o(sum_csr2mmu),

    // Exception requests
    .exp_vld_i(exp_vld_mem_wb2csr),
    .exp_isinterrupt_i(exp_isinterrupt_wb2csr),
    .exp_cause_i(exp_cause_mem_wb2csr),
    .exp_tval_i(exp_tval_mem_wb2csr),

    //To Memory
    .mstatus_ie_o(mstatus_ie_csr2mem), //{MIE, WPRI, SIE ,UIE}
    .mie_o(mie_csr2mem),

    //
    .mip_update_i(mip_update_wb2csr)
);

// =============================================================================
mmu #(
    .INSTR_TLB_ENTRIES(16),
    .DATA_TLB_ENTRIES(16),
    .ASID_WIDTH(1)
) MMU (
    .clk_i(clk_i),
    .rst_i(rst_i),
 
    .flush_i(flush2mmu),
    .flush_type_i(flush_type2mmu),
    .fulsh_vaddr_i(fulsh_vaddr2mmu),
    .fulsh_asid_i(fulsh_asid2mmu),

    //From CSR
    .enable_i(mmu_enable_csr2mmu),
    .root_ppn_i(root_ppn_csr2mmu),
    .asid_i(asid_csr2mmu),
    .privilege_lvl_i(priv_lvl),
    .ld_st_privilege_lvl_i(ld_st_privilege_lvl_csr2mmu),
    .mxr_i(mxr_csr2mmu),
    .sum_i(sum_csr2mmu),

    //From processor to icache
    .i_req_vld_i(i_req_vld_fet2mmu),         // request.
    .i_req_vaddr_i(i_req_vaddr_fet2mmu),     // Memory address.

    .i_req_vld_o(instruction_req_o),           // request.
    .i_req_paddr_o(instruction_addr_o),        // Memory address.

    //From processor to dcache
    .d_req_vld_i(d_req_vld_mem2mmu),         // request.
    .d_req_vaddr_i(d_req_vaddr_mem2mmu),       // Memory address.
    .d_req_data_i(d_req_data_mem2mmu),
    .d_req_rw_i(d_req_rw_mem2mmu),          // 0: data read, 1: data write.
    .d_req_byte_enable_i(d_req_byte_enable_mem2mmu),
    
    //Send request to dcache
    .d_req_vld_o(data_req_o),
    // .d_strobe_o(data_strobe_o),
    .d_req_paddr_o(data_addr_o),
    .d_req_data_o(data_write_o),
    .d_req_rw_o(data_rw_o),          // 0: data read, 1: data write.
    .d_req_byte_enable_o(data_byte_enable_o),    

    //From icache
    .i_rtrn_vld_i(instruction_ready_i),
    .i_rtrn_data_i(instruction_i),

    //From icache to processor
    .i_rtrn_vld_o(i_rtrn_vld_mmu2fet),
    .i_rtrn_vaddr_o(i_rtrn_vaddr_mmu2fet),
    // .i_rtrn_paddr_o(i_rtrn_paddr_mmu2fet),
    .i_rtrn_data_o(i_rtrn_data_mmu2fet),

    //From dcache
    .d_rtrn_vld_i(data_ready_i),
    .d_rtrn_data_i(data_read_i),

    //From dcache to processor
    .d_rtrn_vld_o(d_rtrn_vld_mmu2mem),
    .d_rtrn_data_o(d_rtrn_data_mmu2mem_wb),

    //Exception
    .i_exp_vld_o(i_exp_vld_mmu2fet),
    .i_exp_cause_o(i_exp_cause_mmu2fet),
    .i_exp_tval_o(i_exp_tval_mmu2fet),

    .d_exp_vld_o(d_exp_vld_mmu2mem),
    .d_exp_cause_o(d_exp_cause_mmu2mem),
    .d_exp_tval_o(d_exp_tval_mmu2mem)
);
endmodule // core_top
