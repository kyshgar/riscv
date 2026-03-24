//-----------------------------------------------------------------
//                         RISC-V Core
//                            V1.0.1
//                     Ultra-Embedded.com
//                     Copyright 2014-2019
//
//                   admin@ultra-embedded.com
//
//                       License: BSD
//-----------------------------------------------------------------
//
// Copyright (c) 2014-2019, Ultra-Embedded.com
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions 
// are met:
//   - Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   - Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer 
//     in the documentation and/or other materials provided with the 
//     distribution.
//   - Neither the name of the author nor the names of its contributors 
//     may be used to endorse or promote products derived from this 
//     software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
// THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
// SUCH DAMAGE.
//-----------------------------------------------------------------
`include "riscv_defs.v"

// ============================================================
// 模块: riscv_decoder
// 功能: 纯组合逻辑指令分类器（Instruction Decoder / Classifier）
//   - 对输入的 32 位指令码进行模式匹配，判断其所属类别：
//       exec   : 整数运算指令（ALU，含立即数和寄存器操作）
//       lsu    : 访存指令（Load / Store）
//       branch : 分支/跳转指令（JAL, JALR, BEQ, BNE 等）
//       mul    : 乘法指令（M 扩展，需 enable_muldiv_i）
//       div    : 除法/取余指令（M 扩展，需 enable_muldiv_i）
//       csr    : CSR 访问及系统指令（ECALL, EBREAK, ERET, WFI, FENCE 等）
//       invalid: 非法指令（不匹配任何已知编码，或取指错误时）
//   - 同时判断目标寄存器 rd 是否有效（有写回操作）
// 注意: 本模块为纯组合逻辑，无时序寄存器
// ============================================================
module riscv_decoder
(
     input                        valid_i         // 当前指令有效（取指结果有效）
    ,input                        fetch_fault_i   // 取指发生错误（总线错误或页错误）
    ,input                        enable_muldiv_i // 是否允许乘除法指令（M 扩展使能）
    ,input  [31:0]                opcode_i        // 32 位指令编码

    ,output                       invalid_o       // 非法指令标志（1=非法）
    ,output                       exec_o          // 整数运算指令标志
    ,output                       lsu_o           // 访存指令标志
    ,output                       branch_o        // 分支/跳转指令标志
    ,output                       mul_o           // 乘法指令标志（M 扩展）
    ,output                       div_o           // 除法/取余指令标志（M 扩展）
    ,output                       csr_o           // CSR/系统指令标志
    ,output                       rd_valid_o      // 目标寄存器 rd 有效（有写回）
);

// 非法指令检测逻辑：
// 当 valid_i=1 时，对指令码进行全量模式匹配；
// 若不匹配任何已知 RV32I/M/Zicsr 指令编码，则判定为非法指令。
// 乘除法指令（MUL/DIV 等）仅在 enable_muldiv_i=1 时被视为合法。
wire invalid_w =    valid_i && 
                   ~(((opcode_i & `INST_ANDI_MASK) == `INST_ANDI)             || // andi
                    ((opcode_i & `INST_ADDI_MASK) == `INST_ADDI)              || // addi
                    ((opcode_i & `INST_SLTI_MASK) == `INST_SLTI)              || // slti
                    ((opcode_i & `INST_SLTIU_MASK) == `INST_SLTIU)            || // sltiu
                    ((opcode_i & `INST_ORI_MASK) == `INST_ORI)                || // ori
                    ((opcode_i & `INST_XORI_MASK) == `INST_XORI)              || // xori
                    ((opcode_i & `INST_SLLI_MASK) == `INST_SLLI)              || // slli
                    ((opcode_i & `INST_SRLI_MASK) == `INST_SRLI)              || // srli
                    ((opcode_i & `INST_SRAI_MASK) == `INST_SRAI)              || // srai
                    ((opcode_i & `INST_LUI_MASK) == `INST_LUI)                || // lui
                    ((opcode_i & `INST_AUIPC_MASK) == `INST_AUIPC)            || // auipc
                    ((opcode_i & `INST_ADD_MASK) == `INST_ADD)                || // add
                    ((opcode_i & `INST_SUB_MASK) == `INST_SUB)                || // sub
                    ((opcode_i & `INST_SLT_MASK) == `INST_SLT)                || // slt
                    ((opcode_i & `INST_SLTU_MASK) == `INST_SLTU)              || // sltu
                    ((opcode_i & `INST_XOR_MASK) == `INST_XOR)                || // xor
                    ((opcode_i & `INST_OR_MASK) == `INST_OR)                  || // or
                    ((opcode_i & `INST_AND_MASK) == `INST_AND)                || // and
                    ((opcode_i & `INST_SLL_MASK) == `INST_SLL)                || // sll
                    ((opcode_i & `INST_SRL_MASK) == `INST_SRL)                || // srl
                    ((opcode_i & `INST_SRA_MASK) == `INST_SRA)                || // sra
                    ((opcode_i & `INST_JAL_MASK) == `INST_JAL)                || // jal
                    ((opcode_i & `INST_JALR_MASK) == `INST_JALR)              || // jalr
                    ((opcode_i & `INST_BEQ_MASK) == `INST_BEQ)                || // beq
                    ((opcode_i & `INST_BNE_MASK) == `INST_BNE)                || // bne
                    ((opcode_i & `INST_BLT_MASK) == `INST_BLT)                || // blt
                    ((opcode_i & `INST_BGE_MASK) == `INST_BGE)                || // bge
                    ((opcode_i & `INST_BLTU_MASK) == `INST_BLTU)              || // bltu
                    ((opcode_i & `INST_BGEU_MASK) == `INST_BGEU)              || // bgeu
                    ((opcode_i & `INST_LB_MASK) == `INST_LB)                  || // lb
                    ((opcode_i & `INST_LH_MASK) == `INST_LH)                  || // lh
                    ((opcode_i & `INST_LW_MASK) == `INST_LW)                  || // lw
                    ((opcode_i & `INST_LBU_MASK) == `INST_LBU)                || // lbu
                    ((opcode_i & `INST_LHU_MASK) == `INST_LHU)                || // lhu
                    ((opcode_i & `INST_LWU_MASK) == `INST_LWU)                || // lwu
                    ((opcode_i & `INST_SB_MASK) == `INST_SB)                  || // sb
                    ((opcode_i & `INST_SH_MASK) == `INST_SH)                  || // sh
                    ((opcode_i & `INST_SW_MASK) == `INST_SW)                  || // sw
                    ((opcode_i & `INST_ECALL_MASK) == `INST_ECALL)            || // ecall
                    ((opcode_i & `INST_EBREAK_MASK) == `INST_EBREAK)          || // ebreak
                    ((opcode_i & `INST_ERET_MASK) == `INST_ERET)              || // eret/mret/sret
                    ((opcode_i & `INST_CSRRW_MASK) == `INST_CSRRW)            || // csrrw
                    ((opcode_i & `INST_CSRRS_MASK) == `INST_CSRRS)            || // csrrs
                    ((opcode_i & `INST_CSRRC_MASK) == `INST_CSRRC)            || // csrrc
                    ((opcode_i & `INST_CSRRWI_MASK) == `INST_CSRRWI)          || // csrrwi
                    ((opcode_i & `INST_CSRRSI_MASK) == `INST_CSRRSI)          || // csrrsi
                    ((opcode_i & `INST_CSRRCI_MASK) == `INST_CSRRCI)          || // csrrci
                    ((opcode_i & `INST_WFI_MASK) == `INST_WFI)                || // wfi
                    ((opcode_i & `INST_FENCE_MASK) == `INST_FENCE)            || // fence
                    ((opcode_i & `INST_IFENCE_MASK) == `INST_IFENCE)          || // fence.i
                    ((opcode_i & `INST_SFENCE_MASK) == `INST_SFENCE)          || // sfence.vma
                    // 以下乘除法指令仅在 M 扩展使能时合法
                    (enable_muldiv_i && (opcode_i & `INST_MUL_MASK) == `INST_MUL)       ||
                    (enable_muldiv_i && (opcode_i & `INST_MULH_MASK) == `INST_MULH)     ||
                    (enable_muldiv_i && (opcode_i & `INST_MULHSU_MASK) == `INST_MULHSU) ||
                    (enable_muldiv_i && (opcode_i & `INST_MULHU_MASK) == `INST_MULHU)   ||
                    (enable_muldiv_i && (opcode_i & `INST_DIV_MASK) == `INST_DIV)       ||
                    (enable_muldiv_i && (opcode_i & `INST_DIVU_MASK) == `INST_DIVU)     ||
                    (enable_muldiv_i && (opcode_i & `INST_REM_MASK) == `INST_REM)       ||
                    (enable_muldiv_i && (opcode_i & `INST_REMU_MASK) == `INST_REMU));

// 输出非法指令标志
assign invalid_o = invalid_w;

// rd_valid_o：目标寄存器有效性判断
// 以下指令类型均有写回操作（rd != x0 时写入结果）：
//   跳转类（JAL/JALR）、立即数加载（LUI/AUIPC）、整数运算立即数和寄存器型、
//   Load 指令、乘除法指令、CSR 读写指令
assign rd_valid_o = ((opcode_i & `INST_JALR_MASK) == `INST_JALR)     || // jalr (rd=返回地址)
                    ((opcode_i & `INST_JAL_MASK) == `INST_JAL)       || // jal  (rd=返回地址)
                    ((opcode_i & `INST_LUI_MASK) == `INST_LUI)       || // lui
                    ((opcode_i & `INST_AUIPC_MASK) == `INST_AUIPC)   || // auipc
                    ((opcode_i & `INST_ADDI_MASK) == `INST_ADDI)     || // addi
                    ((opcode_i & `INST_SLLI_MASK) == `INST_SLLI)     || // slli
                    ((opcode_i & `INST_SLTI_MASK) == `INST_SLTI)     || // slti
                    ((opcode_i & `INST_SLTIU_MASK) == `INST_SLTIU)   || // sltiu
                    ((opcode_i & `INST_XORI_MASK) == `INST_XORI)     || // xori
                    ((opcode_i & `INST_SRLI_MASK) == `INST_SRLI)     || // srli
                    ((opcode_i & `INST_SRAI_MASK) == `INST_SRAI)     || // srai
                    ((opcode_i & `INST_ORI_MASK) == `INST_ORI)       || // ori
                    ((opcode_i & `INST_ANDI_MASK) == `INST_ANDI)     || // andi
                    ((opcode_i & `INST_ADD_MASK) == `INST_ADD)       || // add
                    ((opcode_i & `INST_SUB_MASK) == `INST_SUB)       || // sub
                    ((opcode_i & `INST_SLL_MASK) == `INST_SLL)       || // sll
                    ((opcode_i & `INST_SLT_MASK) == `INST_SLT)       || // slt
                    ((opcode_i & `INST_SLTU_MASK) == `INST_SLTU)     || // sltu
                    ((opcode_i & `INST_XOR_MASK) == `INST_XOR)       || // xor
                    ((opcode_i & `INST_SRL_MASK) == `INST_SRL)       || // srl
                    ((opcode_i & `INST_SRA_MASK) == `INST_SRA)       || // sra
                    ((opcode_i & `INST_OR_MASK) == `INST_OR)         || // or
                    ((opcode_i & `INST_AND_MASK) == `INST_AND)       || // and
                    ((opcode_i & `INST_LB_MASK) == `INST_LB)         || // lb  (load有写回)
                    ((opcode_i & `INST_LH_MASK) == `INST_LH)         || // lh
                    ((opcode_i & `INST_LW_MASK) == `INST_LW)         || // lw
                    ((opcode_i & `INST_LBU_MASK) == `INST_LBU)       || // lbu
                    ((opcode_i & `INST_LHU_MASK) == `INST_LHU)       || // lhu
                    ((opcode_i & `INST_LWU_MASK) == `INST_LWU)       || // lwu
                    ((opcode_i & `INST_MUL_MASK) == `INST_MUL)       || // mul
                    ((opcode_i & `INST_MULH_MASK) == `INST_MULH)     || // mulh
                    ((opcode_i & `INST_MULHSU_MASK) == `INST_MULHSU) || // mulhsu
                    ((opcode_i & `INST_MULHU_MASK) == `INST_MULHU)   || // mulhu
                    ((opcode_i & `INST_DIV_MASK) == `INST_DIV)       || // div
                    ((opcode_i & `INST_DIVU_MASK) == `INST_DIVU)     || // divu
                    ((opcode_i & `INST_REM_MASK) == `INST_REM)       || // rem
                    ((opcode_i & `INST_REMU_MASK) == `INST_REMU)     || // remu
                    ((opcode_i & `INST_CSRRW_MASK) == `INST_CSRRW)   || // csrrw (rd=旧值)
                    ((opcode_i & `INST_CSRRS_MASK) == `INST_CSRRS)   || // csrrs
                    ((opcode_i & `INST_CSRRC_MASK) == `INST_CSRRC)   || // csrrc
                    ((opcode_i & `INST_CSRRWI_MASK) == `INST_CSRRWI) || // csrrwi
                    ((opcode_i & `INST_CSRRSI_MASK) == `INST_CSRRSI) || // csrrsi
                    ((opcode_i & `INST_CSRRCI_MASK) == `INST_CSRRCI);   // csrrci

// exec_o：整数运算指令（ALU 类）
// 包括：立即数运算（ADDI/SLTI/XORI/ORI/ANDI/SLLI/SRLI/SRAI）、
//        寄存器运算（ADD/SUB/SLT/XOR/OR/AND/SLL/SRL/SRA）、
//        大立即数（LUI/AUIPC）
assign exec_o =     ((opcode_i & `INST_ANDI_MASK) == `INST_ANDI)  || // andi
                    ((opcode_i & `INST_ADDI_MASK) == `INST_ADDI)  || // addi
                    ((opcode_i & `INST_SLTI_MASK) == `INST_SLTI)  || // slti
                    ((opcode_i & `INST_SLTIU_MASK) == `INST_SLTIU)|| // sltiu
                    ((opcode_i & `INST_ORI_MASK) == `INST_ORI)    || // ori
                    ((opcode_i & `INST_XORI_MASK) == `INST_XORI)  || // xori
                    ((opcode_i & `INST_SLLI_MASK) == `INST_SLLI)  || // slli
                    ((opcode_i & `INST_SRLI_MASK) == `INST_SRLI)  || // srli
                    ((opcode_i & `INST_SRAI_MASK) == `INST_SRAI)  || // srai
                    ((opcode_i & `INST_LUI_MASK) == `INST_LUI)    || // lui
                    ((opcode_i & `INST_AUIPC_MASK) == `INST_AUIPC)|| // auipc
                    ((opcode_i & `INST_ADD_MASK) == `INST_ADD)    || // add
                    ((opcode_i & `INST_SUB_MASK) == `INST_SUB)    || // sub
                    ((opcode_i & `INST_SLT_MASK) == `INST_SLT)    || // slt
                    ((opcode_i & `INST_SLTU_MASK) == `INST_SLTU)  || // sltu
                    ((opcode_i & `INST_XOR_MASK) == `INST_XOR)    || // xor
                    ((opcode_i & `INST_OR_MASK) == `INST_OR)      || // or
                    ((opcode_i & `INST_AND_MASK) == `INST_AND)    || // and
                    ((opcode_i & `INST_SLL_MASK) == `INST_SLL)    || // sll
                    ((opcode_i & `INST_SRL_MASK) == `INST_SRL)    || // srl
                    ((opcode_i & `INST_SRA_MASK) == `INST_SRA);      // sra

// lsu_o：访存指令（Load/Store 类）
// Load: lb/lh/lw/lbu/lhu/lwu；Store: sb/sh/sw
assign lsu_o =      ((opcode_i & `INST_LB_MASK) == `INST_LB)   || // lb
                    ((opcode_i & `INST_LH_MASK) == `INST_LH)   || // lh
                    ((opcode_i & `INST_LW_MASK) == `INST_LW)   || // lw
                    ((opcode_i & `INST_LBU_MASK) == `INST_LBU) || // lbu
                    ((opcode_i & `INST_LHU_MASK) == `INST_LHU) || // lhu
                    ((opcode_i & `INST_LWU_MASK) == `INST_LWU) || // lwu
                    ((opcode_i & `INST_SB_MASK) == `INST_SB)   || // sb
                    ((opcode_i & `INST_SH_MASK) == `INST_SH)   || // sh
                    ((opcode_i & `INST_SW_MASK) == `INST_SW);     // sw

// branch_o：分支/跳转指令
// 无条件跳转：JAL, JALR；条件分支：BEQ, BNE, BLT, BGE, BLTU, BGEU
assign branch_o =   ((opcode_i & `INST_JAL_MASK) == `INST_JAL)   || // jal
                    ((opcode_i & `INST_JALR_MASK) == `INST_JALR) || // jalr
                    ((opcode_i & `INST_BEQ_MASK) == `INST_BEQ)   || // beq
                    ((opcode_i & `INST_BNE_MASK) == `INST_BNE)   || // bne
                    ((opcode_i & `INST_BLT_MASK) == `INST_BLT)   || // blt
                    ((opcode_i & `INST_BGE_MASK) == `INST_BGE)   || // bge
                    ((opcode_i & `INST_BLTU_MASK) == `INST_BLTU) || // bltu
                    ((opcode_i & `INST_BGEU_MASK) == `INST_BGEU);   // bgeu

// mul_o：乘法指令（M 扩展），必须 enable_muldiv_i=1 才有效
// 包括：MUL, MULH, MULHSU, MULHU
assign mul_o =      enable_muldiv_i &&
                    (((opcode_i & `INST_MUL_MASK) == `INST_MUL)    || // mul
                    ((opcode_i & `INST_MULH_MASK) == `INST_MULH)   || // mulh
                    ((opcode_i & `INST_MULHSU_MASK) == `INST_MULHSU) || // mulhsu
                    ((opcode_i & `INST_MULHU_MASK) == `INST_MULHU));   // mulhu

// div_o：除法/取余指令（M 扩展），必须 enable_muldiv_i=1 才有效
// 包括：DIV, DIVU, REM, REMU
assign div_o =      enable_muldiv_i &&
                    (((opcode_i & `INST_DIV_MASK) == `INST_DIV) || // div
                    ((opcode_i & `INST_DIVU_MASK) == `INST_DIVU) || // divu
                    ((opcode_i & `INST_REM_MASK) == `INST_REM) || // rem
                    ((opcode_i & `INST_REMU_MASK) == `INST_REMU));  // remu

// csr_o：CSR 及系统指令类别
// 包括：CSR 读写（CSRRW/CSRRS/CSRRC/CSRRWI/CSRRSI/CSRRCI）、
//        系统调用/断点（ECALL/EBREAK）、异常返回（ERET）、
//        等待中断（WFI）、内存屏障（FENCE/FENCE.I/SFENCE.VMA）
// 同时，非法指令和取指错误也路由至 CSR 流水线（由 CSR 模块处理异常）
assign csr_o =      ((opcode_i & `INST_ECALL_MASK) == `INST_ECALL)            || // ecall
                    ((opcode_i & `INST_EBREAK_MASK) == `INST_EBREAK)          || // ebreak
                    ((opcode_i & `INST_ERET_MASK) == `INST_ERET)              || // eret/mret/sret
                    ((opcode_i & `INST_CSRRW_MASK) == `INST_CSRRW)            || // csrrw
                    ((opcode_i & `INST_CSRRS_MASK) == `INST_CSRRS)            || // csrrs
                    ((opcode_i & `INST_CSRRC_MASK) == `INST_CSRRC)            || // csrrc
                    ((opcode_i & `INST_CSRRWI_MASK) == `INST_CSRRWI)          || // csrrwi
                    ((opcode_i & `INST_CSRRSI_MASK) == `INST_CSRRSI)          || // csrrsi
                    ((opcode_i & `INST_CSRRCI_MASK) == `INST_CSRRCI)          || // csrrci
                    ((opcode_i & `INST_WFI_MASK) == `INST_WFI)                || // wfi
                    ((opcode_i & `INST_FENCE_MASK) == `INST_FENCE)            || // fence
                    ((opcode_i & `INST_IFENCE_MASK) == `INST_IFENCE)          || // fence.i
                    ((opcode_i & `INST_SFENCE_MASK) == `INST_SFENCE)          || // sfence.vma
                    invalid_w || fetch_fault_i; // 非法指令和取指错误均转异常处理

endmodule
