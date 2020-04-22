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
                        dec_load_ext_sel2exe, dec_instr_valid2csr;
wire [ 4 : 0]           dec_rd_addr2exe, dec_rs1_addr2fwd, dec_rs2_addr2fwd,
                        dec_csr_imm2csr;
wire [ 1 : 0]           dec_inA_sel2exe, dec_inB_sel2exe, dec_input_sel2exe;
wire [ 2 : 0]           dec_regfile_sel2exe, dec_operation_sel2exe;
wire [11 : 0]           dec_csr_addr2csr;

wire                    alu_muldiv_sel2exe, shift_sel2exe, is_branch2exe,
                        is_jal2exe, is_jalr2exe, is_csr_instr2csr;

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

// Control Status Registers (CSR).
wire [DATA_WIDTH-1 : 0] csr_data2exe;

// Memory Access.
wire [DATA_WIDTH-1 : 0] data_o, data2mem_wb;
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

// ----------------------
// csr <-> mmu
// ----------------------
wire         mmu_enable_csr2mmu = 0;
wire [21: 0] root_ppn_csr2mmu;
wire [ 8: 0] asid_csr2mmu;
wire [ 1: 0] privilege_lvl_csr2mmu;

// ----------------------
// instruction <-> mmu
// ----------------------
wire         i_req_vld_fet2mmu;
wire [31: 0] i_req_vaddr_fet2mmu;

wire         i_exception_vld_mmu2fet;
wire [ 3: 0] i_exception_cause_mmu2fet;

// ----------------------
// data <-> mmu
// ----------------------
wire         d_req_vld_mem2mmu;        
wire [31: 0] d_req_vaddr_mem2mmu;      
wire [31: 0] d_req_data_mem2mmu;
wire         d_req_rw_mem2mmu;         
wire [ 3: 0] d_req_byte_enable_mem2mmu;

wire         d_rtrn_vld_mmu2mem;
wire         d_exception_vld_mmu2mem;

// ------------------------
// memory_access <-> mmu_wb
// ------------------------
wire [31: 0] d_rtrn_data_mmu2mem_wb;
wire [ 3: 0] d_exception_cause_mmu2mem_wb;

// Exception
// wire mem_memory_alignment_exception2mem_wb;
// wire exe_mem_exception_vld2mem_wb, mem_wb_exception_vld2csr;
// wire [3:0] exe_mem_exception_cause2mem_wb, mem_wb_exception_cause2csr;



// =============================================================================
// Finite state machine that controls the processor pipeline stalls.
//
localparam i_NEXT = 0, i_WAIT = 1;
localparam d_IDLE = 0, d_WAIT = 1;
 reg iS, iS_nxt, dS, dS_nxt;

// -----------------------------------------------------------------------------
// The stall signals:
//    # stall_pipeline signal will stall all pipeline registers
//    # stall_from_hazard only stall the Program_Counter and the Fetch stages
//
wire stall_pipeline, stall_for_instr_fetch, stall_for_data_fetch;

assign stall_for_instr_fetch = (!instruction_ready_i);
assign stall_for_data_fetch = (dS_nxt == d_WAIT);

 always @(posedge clk_i)
 begin
     if (rst_i)
         iS <= i_NEXT;
     else
         iS <= iS_nxt;
 end

 always @(*)
 begin
     case (iS)
         i_NEXT: // CJ Tsai 0227_2020: I-fetch when I-memory ready.
             if (instruction_ready_i)
                 iS_nxt = i_NEXT;
             else
                 iS_nxt = i_WAIT;
         i_WAIT:
             if (instruction_ready_i)
                 iS_nxt = i_NEXT; // one-cycle delay
             else
                 iS_nxt = i_WAIT;
     endcase
 end

always @(posedge clk_i)
begin
    if (rst_i)
        dS <= d_IDLE;
    else
        dS <= dS_nxt;
end

always @(*)
begin
    case (dS)
        d_IDLE:
            if (exe_re || exe_we)
                dS_nxt = d_WAIT;
            else
                dS_nxt = d_IDLE;
        d_WAIT:
            if (d_rtrn_vld_mmu2mem)
                dS_nxt = d_IDLE;
            else
                dS_nxt = d_WAIT;
    endcase
end

// -----------------------------------------------------------------------------
// Output instruction/data request signals to mmu
// assign instruction_req_o = !(iS == i_WAIT && instruction_ready_i);
// assign data_req_o = (dS_nxt == d_WAIT);
// assign data_strobe_o = (dS == d_IDLE) && (exe_re | exe_we);

//  Signals from the Memory Access Stage
// assign instruction_addr_o = pc;
// assign data_write_o = data_o;
// assign data_addr_o = exe_addr2mem;
// assign data_rw_o = exe_we;
// assign data_byte_enable_o = byte_write_sel;

assign i_req_vld_fet2mmu   = !(iS == i_WAIT && instruction_ready_i);
assign i_req_vaddr_fet2mmu = pc;

assign d_req_vld_mem2mmu          = (dS_nxt == d_WAIT);      
assign d_req_vaddr_mem2mmu        = exe_addr2mem;      
assign d_req_data_mem2mmu         = data_o;
assign d_req_rw_mem2mmu           = exe_we;
assign d_req_byte_enable_mem2mmu  = byte_write_sel;

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

    // to Program_Counter, Fetch stage
    .stall_from_hazard_o(stall_from_hazard)
);

// =============================================================================
forwarding_unit Forwarding_Unit(
    // from Decode_Execute_Pipeline
    .rs1_addr_i(dec_rs1_addr2fwd),
    .rs2_addr_i(dec_rs2_addr2fwd),
    .rs1_data_i(dec_rs1_data2fwd),
    .rs2_data_i(dec_rs2_data2fwd),

    // from Execute_Memory_Pipeline
    .regfile_we_EXE_MEM_i(exe_regfile_we2mem_wb),
    .rd_addr_EXE_MEM_i(exe_rd_addr2mem_wb),
    .regfile_input_sel_EXE_MEM_i(exe_regfile_input_sel2mem_wb),
    .p_data_EXE_MEM_i(exe_p_data),

    // from Memory_Writeback_Pipeline
    .regfile_we_MEM_WB_i(rd_we2wb),
    .rd_addr_MEM_WB_i(rd_addr2wb),
    .rd_data_MEM_WB_i(rd_data2wb),

    // to Execute, CSR, Execute_Memory_Pipeline
    .rs1_fwd_o(rs1_fwd),
    .rs2_fwd_o(rs2_fwd)
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
    .pc_o(pc)
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
    .instruction_i(instruction_i),

    // from Program_Counter
    .pc_i(pc),

    // to the Decode Stage
    .instruction_o(fet_instr2dec),

    // to the Execute Stage
    .cond_branch_hit_ID_o(cond_branch_hit_ID),
    .cond_branch_result_ID_o(cond_branch_result_ID),
    .uncond_branch_hit_ID_o(uncond_branch_hit_ID),
    .pc_o(fet_pc2dec),
    .instr_valid_o(fet_valid2dec)
);

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

    // System Jump operation
    .sys_jump_o(sys_jump),
    .sys_jump_csr_addr_o(sys_jump_csr_addr),

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

    // to Execute, Cond_Branch_Predictor
    .is_branch_o(is_branch2exe),

    // to Execute, Pipeline_Control, Uncond_Branch_BHT
    .is_jal_o(is_jal2exe),

    // to CSR
    .is_csr_instr_o(is_csr_instr2csr),
    .csr_addr_o(dec_csr_addr2csr),
    .csr_imm_o(dec_csr_imm2csr),
    .instr_valid_o(dec_instr_valid2csr),

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
    .rs2_data2fwd_o(dec_rs2_data2fwd)
);

// =============================================================================
execute Execute(
    // Top-level system signals
    .clk_i(clk_i),
    .rst_i(rst_i),

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

    // Signal from the CSR.
    .csr_data_i(csr_data2exe),

    // Signals from the Forwarding Unit.
    .rs1_data_i(rs1_fwd),
    .rs2_data_i(rs2_fwd),

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

    // Signals to Memory Writeback.
    .mem_load_ext_sel_o(exe_load_ext_sel2mem_wb)
);

// =============================================================================
memory_access Memory_Access(
    // from Execute_Memory_Pipeline
    .unaligned_data_i(exe_rs2_data2mem),      // store value
    .mem_addr_alignment_i(exe_addr2mem[1: 0]),
    .mem_input_sel_i(exe_input_sel2mem),

    // to D-memory
    .data_o(data_o),                        // data_write
    .byte_write_sel_o(byte_write_sel),

    // Exception signal
    .memory_alignment_exception_o(memory_alignment_exception)
);

// =============================================================================
writeback Writeback(
    // Top-level system signals
    .clk_i(clk_i),
    .rst_i(rst_i),
    .stall_i(stall_for_instr_fetch | stall_for_data_fetch | stall_from_exe),

    // from Execute_Memory_Pipeline
    .regfile_we_i(exe_regfile_we2mem_wb),
    .rd_addr_i(exe_rd_addr2mem_wb),
    .regfile_input_sel_i(exe_regfile_input_sel2mem_wb),
    .mem_load_ext_sel_i(exe_load_ext_sel2mem_wb),
    .mem_addr_alignment_i(exe_addr2mem[1: 0]),
    .p_data_i(exe_p_data),

    // from D-memory
    .mem_data_i(d_rtrn_data_mmu2mem_wb),

    // to RegisterFile, Forwarding_Unit
    .rd_we_o(rd_we2wb),
    .rd_addr_o(rd_addr2wb),
    .rd_data_o(rd_data2wb)
);

// =============================================================================
csr_file #( .HART_ID(HART_ID) )
CSR(
    // Top-level system signals
    .clk_i(clk_i),
    .rst_i(rst_i),

    // from Decode_Execute_Pipeline
    .is_csr_instr_i(is_csr_instr2csr),
    .csr_addr_i(dec_csr_addr2csr),
    .csr_op_i(dec_operation_sel2exe),
    .csr_imm_i(dec_csr_imm2csr),

    // from Forwarding_Unit
    .rs1_data_i(rs1_fwd),

    // to Execute_Memory_Pipeline
    .csr_data_o(csr_data2exe),

    // System Jump operation
    .sys_jump_i(sys_jump),
    .sys_jump_csr_addr_i(sys_jump_csr_addr),
    .sys_jump_pc_i(dec_pc2exe),
    .sys_jump_csr_data_o(sys_jump_csr_data),

    // Interrupt
    .ext_irq_i(ext_irq_i),
    .tmr_irq_i(tmr_irq_i),
    .sft_irq_i(sft_irq_i),
    .irq_taken_o(irq_taken),
    .PC_handler_o(PC_handler),
    .nxt_unexec_PC_i(fet_pc2dec),

    //MMU
    .mmu_enable_o(mmu_enable_csr2mmu),
    .root_ppn_o(root_ppn_csr2mmu),
    .asid_o(asid_csr2mmu),
    .privilege_lvl_o(privilege_lvl_csr2mmu),

    // Exception requests
    .exception_vld_i(),
    .exception_cause_i()
);

// =============================================================================
mmu #(
    .INSTR_TLB_ENTRIES(4),
    .DATA_TLB_ENTRIES(4),
    .ASID_WIDTH(1)
) MMU (
    .clk_i(clk_i),
    .rst_i(rst_i),
    .flush_i('b0),

    //From CSR
    .enable_i(mmu_enable_csr2mmu),
    .root_ppn_i(root_ppn_csr2mmu),
    .asid_i(asid_csr2mmu),
    .privilege_lvl_i(privilege_lvl_csr2mmu),

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

    //From dcache
    .d_rtrn_vld_i(data_ready_i),
    .d_rtrn_data_i(data_read_i),

    //From dcache to processor
    .d_rtrn_vld_o(d_rtrn_vld_mmu2mem),
    .d_rtrn_data_o(d_rtrn_data_mmu2mem_wb),

    //Exception
    .i_exception_vld_o(i_exception_vld_mmu2fet),
    .i_exception_cause_o(i_exception_cause_mmu2fet),

    .d_exception_vld_o(d_exception_vld_mmu2mem),
    .d_exception_cause_o(d_exception_cause_mmu2mem_wb)
);
endmodule // core_top
