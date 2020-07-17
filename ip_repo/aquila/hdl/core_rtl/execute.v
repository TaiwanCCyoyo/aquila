`timescale 1ns / 1ps
// =============================================================================
//  Program : execute.v
//  Author  : Jin-you Wu
//  Date    : Dec/19/2018
// -----------------------------------------------------------------------------
//  Description:
//  This is the Execution Unit of the Aquila core (A RISC-V core).
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Nov/29/2019, by Chun-Jen Tsai:
//    Merges the pipeline register module 'execute_memory' into the 'execute'
//    module.
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

module execute #(parameter DATA_WIDTH = 32)
(
    // System Signals
    input  wire                    clk_i,
    input  wire                    rst_i,

    // Processor pipeline flush signal.
    input  wire                    flush_i,

    // Signal from the Program_Counter Unit.
    input  wire [DATA_WIDTH-1 : 0] pc_i,

    // Pipeline stall signal.
    input  wire                    stall_i,

    // Signals from the Decode stage.
    input  wire [DATA_WIDTH-1 : 0] imm_i,
    input  wire [ 1: 0]            inputA_sel_i,
    input  wire [ 1: 0]            inputB_sel_i,
    input  wire [ 2: 0]            operation_sel_i,
    input  wire                    alu_muldiv_sel_i,
    input  wire                    shift_sel_i,
    input  wire                    is_branch_i,
    input  wire                    is_jal_i,
    input  wire                    is_jalr_i,
    input  wire                    cond_branch_hit_EXE_i,
    input  wire                    cond_branch_result_EXE_i,

    input  wire                    regfile_we_i,
    input  wire [2: 0]             regfile_input_sel_i,
    input  wire                    mem_we_i,
    input  wire                    mem_re_i,
    input  wire [1: 0]             mem_input_sel_i,
    input  wire                    mem_load_ext_sel_i,
    input  wire [4: 0]             rd_addr_i,

    input  wire [4: 0]             csr_imm_i,
    input  wire                    csr_we_i,
    input  wire [11: 0]            csr_we_addr_i,

    // Signals from the Forwarding Unit.
    input  wire [DATA_WIDTH-1 : 0] rs1_data_i,
    input  wire [DATA_WIDTH-1 : 0] rs2_data_i,
    input  wire [DATA_WIDTH-1 : 0] csr_data_i,

    // Signal to the Program Counter Unit.
    output wire [DATA_WIDTH-1 : 0] pc_o,
    output wire [DATA_WIDTH-1 : 0] branch_target_addr_o,

    // Singnals to the Pipeline Control and the Branch Prediction units.
    output wire                    branch_taken_o,
    output wire                    cond_branch_misprediction_o,

    // Pipeline stall signal generator, activated when executing
    //    multicycle mul, div and rem instructions.
    output wire                    stall_from_exe_o,

    // Signals to D-memory.
    output reg                     mem_we_o,
    output reg                     mem_re_o,

    // Signals to Memory Alignment unit.
    output reg  [DATA_WIDTH-1 : 0] rs2_data_o,
    output reg  [DATA_WIDTH-1 : 0] mem_addr_o,
    output reg  [1: 0]             mem_input_sel_o,
    
    // Signals to Memory Writeback Pipeline.
    output reg  [2: 0]             regfile_input_sel_o,
    output reg                     regfile_we_o,
    output reg  [4: 0]             rd_addr_o,
    output reg  [DATA_WIDTH-1 : 0] p_data_o,
    output reg                     csr_we_o,
    output reg  [11: 0]            csr_we_addr_o,
    output reg  [DATA_WIDTH-1 : 0] csr_we_data_o,

    // to Memory_Write_Back_Pipeline
    output reg                     mem_load_ext_sel_o,

    // System Jump operation
    input  wire                    sys_jump_i,
    input  wire [ 1: 0]            sys_jump_csr_addr_i,
    output reg                     sys_jump_o,
    output reg  [ 1: 0]            sys_jump_csr_addr_o,

    //Supervisor Instructions
    input  wire                    sfence_i,
    output reg                     sfence_o,
    input  wire                    sfence_type_i,
    output reg                     sfence_type_o,//0=>rs1=x0,1=>rs1!=x0
    output reg [DATA_WIDTH-1 : 0]  rs1_data_o,

    //Exception from Decode
    input  wire                    exp_vld_i,
    input  wire [ 3: 0]            exp_cause_i,
    input  wire [31: 0]            exp_tval_i,
    input  wire                    instruction_pc_vld_i,

    //Exception to Memory
    output reg                     exp_vld_o,
    output reg  [ 3: 0]            exp_cause_o,
    output reg  [31: 0]            exp_tval_o,
    output reg  [31: 0]            instruction_pc_o,
    output reg                     instruction_pc_vld_o
);

reg  [DATA_WIDTH-1 : 0]           inputA, inputB;
wire [DATA_WIDTH-1 : 0]           alu_result;
wire [DATA_WIDTH-1 : 0]           muldiv_result;
wire                              compare_result, stall_from_muldiv, muldiv_ready;

wire [DATA_WIDTH-1 : 0]           result;
wire [DATA_WIDTH-1 : 0]           mem_addr;

always @(*)
begin
    case (inputA_sel_i)
        3'd0: inputA = 0;
        3'd1: inputA = pc_i;
        3'd2: inputA = rs1_data_i;
        default: inputA = 0;
    endcase
end

always @(*)
begin
    case (inputB_sel_i)
        3'd0: inputB = imm_i;
        3'd1: inputB = rs2_data_i;
        3'd2: inputB = ~rs2_data_i + 1'b1;
        default: inputB = 0;
    endcase
end

// branch target address generate by alu adder
wire [2: 0] alu_operation = (is_branch_i | is_jal_i | is_jalr_i)? 3'b000 : operation_sel_i;
wire [2: 0] muldiv_operation = operation_sel_i;
wire muldiv_req = alu_muldiv_sel_i & !muldiv_ready;
wire [2: 0] branch_operation = operation_sel_i;

// ===============================================================================
//  ALU Regular operation
//
alu ALU(
    .a_i(inputA),
    .b_i(inputB),
    .operation_sel_i(alu_operation),
    .shift_sel_i(shift_sel_i),
    .alu_result_o(alu_result)
);

// ===============================================================================
//   MulDiv
//
muldiv MulDiv(
    .clk_i(clk_i),
    .rst_i(rst_i),
    .a_i(inputA),
    .b_i(inputB),
    .req_i(muldiv_req),
    .operation_sel_i(muldiv_operation),
    .muldiv_result_o(muldiv_result),
    .ready_o(muldiv_ready)
);

// ==============================================================================
//  BCU
//
bcu BCU(
    .a_i(rs1_data_i),
    .b_i(rs2_data_i),
    .operation_sel_i(branch_operation),
    .compare_result_o(compare_result)
);

// ===============================================================================
//  AGU & Output signals
//
assign mem_addr = rs1_data_i + imm_i;         // The target addr of memory load/store
assign branch_target_addr_o = alu_result; // The target addr of BRANCH, JAL, JALR
assign pc_o = pc_i + 'd4;                   // The next PC of instruction, and the
                                          // restore PC if mispredicted branch taken.

assign branch_taken_o = (is_branch_i & compare_result) | is_jal_i | is_jalr_i;
assign cond_branch_misprediction_o = cond_branch_hit_EXE_i &
                                    (cond_branch_result_EXE_i ^ branch_taken_o);

assign result = alu_muldiv_sel_i ? muldiv_result : alu_result;
assign stall_from_exe_o = alu_muldiv_sel_i & !muldiv_ready;

// ===============================================================================
//  CSR
//
wire [DATA_WIDTH-1 : 0] csr_inputA = csr_data_i;
wire [DATA_WIDTH-1 : 0] csr_inputB = operation_sel_i[2] ? {27'b0, csr_imm_i} : rs1_data_i;
reg  [DATA_WIDTH-1 : 0] csr_update_data;

always @( * )
begin
    case (operation_sel_i[1: 0])
        `CSR_RW:
            csr_update_data = csr_inputB;
        `CSR_RS:
            csr_update_data = csr_inputA | csr_inputB;
        `CSR_RC:
            csr_update_data = csr_inputA & ~csr_inputB;
        default:
            csr_update_data = csr_inputA;
    endcase
end

// ===============================================================================
//  Pipeline register operations for the Execute stage.

always @(posedge clk_i)
begin
    if (rst_i)
    begin
        mem_we_o            <= 0;
        mem_re_o            <= 0;
        rs2_data_o          <= 0;
        mem_addr_o          <= 0;
        mem_input_sel_o     <= 0;
        mem_load_ext_sel_o  <= 0;
        regfile_input_sel_o <= 0;
        regfile_we_o        <= 0;
        rd_addr_o           <= 0;
        sys_jump_o          <= 0;
        sys_jump_csr_addr_o <= 0;
        exp_vld_o           <= 0;
        exp_cause_o         <= 0;
        exp_tval_o          <= 0;
        instruction_pc_o    <= 0;
        instruction_pc_vld_o<= 0;
        csr_we_o            <= 0;
        csr_we_addr_o       <= 0;
        csr_we_data_o       <= 0;

        sfence_o            <= 0;
        sfence_type_o       <= 0;
        rs1_data_o          <= 0;
    end
    else if (stall_i)
    begin
        mem_we_o            <= mem_we_o;
        mem_re_o            <= mem_re_o;
        rs2_data_o          <= rs2_data_o;
        mem_addr_o          <= mem_addr_o;
        mem_input_sel_o     <= mem_input_sel_o;
        mem_load_ext_sel_o  <= mem_load_ext_sel_o;
        regfile_input_sel_o <= regfile_input_sel_o;
        regfile_we_o        <= regfile_we_o;
        rd_addr_o           <= rd_addr_o;
        sys_jump_o          <= sys_jump_o;
        sys_jump_csr_addr_o <= sys_jump_csr_addr_o;
        exp_vld_o           <= exp_vld_o;
        exp_cause_o         <= exp_cause_o;
        exp_tval_o          <= exp_tval_o;
        instruction_pc_o    <= instruction_pc_o;
        instruction_pc_vld_o<= instruction_pc_vld_o;
        csr_we_o            <= csr_we_o;
        csr_we_addr_o       <= csr_we_addr_o;
        csr_we_data_o       <= csr_we_data_o;

        sfence_o            <= sfence_o;
        sfence_type_o       <= sfence_type_o;
        rs1_data_o          <= rs1_data_o;
    end
    else if(flush_i)
    begin
        mem_we_o            <= 0;
        mem_re_o            <= 0;
        rs2_data_o          <= 0;
        mem_addr_o          <= 0;
        mem_input_sel_o     <= 0;
        mem_load_ext_sel_o  <= 0;
        regfile_input_sel_o <= 0;
        regfile_we_o        <= 0;
        rd_addr_o           <= 0;
        sys_jump_o          <= 0;
        sys_jump_csr_addr_o <= 0;
        exp_vld_o           <= 0;
        exp_cause_o         <= 0;
        exp_tval_o          <= 0;
        instruction_pc_o    <= 0;
        instruction_pc_vld_o<= 0;
        csr_we_o            <= 0;
        csr_we_addr_o       <= 0;
        csr_we_data_o       <= 0;

        sfence_o            <= 0;
        sfence_type_o       <= 0;
        rs1_data_o          <= 0;
    end
    else
    begin
        mem_we_o            <= mem_we_i ;
        mem_re_o            <= mem_re_i;
        rs2_data_o          <= rs2_data_i;
        mem_addr_o          <= mem_addr;
        mem_input_sel_o     <= mem_input_sel_i;
        mem_load_ext_sel_o  <= mem_load_ext_sel_i;
        regfile_input_sel_o <= regfile_input_sel_i;
        regfile_we_o        <= regfile_we_i;
        rd_addr_o           <= rd_addr_i;
        sys_jump_o          <= sys_jump_i;
        sys_jump_csr_addr_o <= sys_jump_csr_addr_i;
        exp_vld_o           <= exp_vld_i;
        exp_cause_o         <= exp_cause_i;
        exp_tval_o          <= exp_tval_i;
        instruction_pc_o    <= pc_i;
        instruction_pc_vld_o<= instruction_pc_vld_i;
        csr_we_o            <= csr_we_i;
        csr_we_addr_o       <= csr_we_addr_i;
        csr_we_data_o       <= csr_update_data;

        sfence_o            <= sfence_i;
        sfence_type_o       <= sfence_type_i;
        rs1_data_o          <= rs1_data_i;
    end
end

always @(posedge clk_i)
begin
    if (rst_i)
    begin
        p_data_o <= 0;  // data from processor
    end
    else if (stall_i)
    begin
        p_data_o <= p_data_o;
    end
    else
    begin
        case (regfile_input_sel_i)
            3'b011: p_data_o <= pc_o;
            3'b100: p_data_o <= result;
            3'b101: p_data_o <= csr_data_i;
            default: p_data_o <= 0;
        endcase
    end
end

endmodule   // execute
