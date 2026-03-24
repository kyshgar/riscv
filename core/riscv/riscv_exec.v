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

/*
 * 模块功能概述：riscv_exec —— RISC-V 执行单元（EX 阶段）
 *
 * 本模块负责 RISC-V 流水线的执行阶段，主要完成以下工作：
 *   1. 立即数解码：根据指令格式（I/S/B/U/J 型）从指令字中提取并符号扩展立即数；
 *   2. ALU 操作选择：根据指令类型选择 ALU 功能码及操作数，驱动下级 riscv_alu 子模块；
 *   3. 分支条件判断与目标地址计算：
 *        - JAL  : 无条件跳转，目标 = PC + J 型立即数；
 *        - JALR : 间接跳转，目标 = (rs1 + I 型立即数) & ~1；
 *        - Bxx  : 条件分支（BEQ/BNE/BLT/BGE/BLTU/BGEU），目标 = PC + B 型立即数；
 *   4. 将 ALU 结果打拍（D 触发器）后输出至写回总线（writeback_value_o）；
 *   5. 向前级（分支预测/BTB）反馈分支请求、跳转方向及目标地址。
 */
module riscv_exec
(
    // Inputs
     input           clk_i              // 时钟信号
    ,input           rst_i              // 同步/异步复位（高有效）
    ,input           opcode_valid_i     // 当前指令有效标志
    ,input  [ 31:0]  opcode_opcode_i    // 32 位原始指令字
    ,input  [ 31:0]  opcode_pc_i        // 当前指令的 PC 值
    ,input           opcode_invalid_i   // 指令非法标志（来自译码级）
    ,input  [  4:0]  opcode_rd_idx_i    // 目的寄存器索引 rd
    ,input  [  4:0]  opcode_ra_idx_i    // 源寄存器索引 rs1
    ,input  [  4:0]  opcode_rb_idx_i    // 源寄存器索引 rs2
    ,input  [ 31:0]  opcode_ra_operand_i // rs1 操作数值（含前递结果）
    ,input  [ 31:0]  opcode_rb_operand_i // rs2 操作数值（含前递结果）
    ,input           hold_i             // 流水线暂停信号（高有效时禁止 ALU 结果打拍）

    // Outputs
    ,output          branch_request_o      // 分支请求（已确认的跳转或不跳转结果，打拍后输出）
    ,output          branch_is_taken_o     // 分支已跳转标志（打拍后输出）
    ,output          branch_is_not_taken_o // 分支未跳转标志（打拍后输出）
    ,output [ 31:0]  branch_source_o       // 分支指令自身的 PC（用于 BTB 更新）
    ,output          branch_is_call_o      // 本次分支为函数调用（rd=ra=x1）
    ,output          branch_is_ret_o       // 本次分支为函数返回（JALR，rs1=x1，偏移=0）
    ,output          branch_is_jmp_o       // 本次分支为普通跳转（非 call/ret）
    ,output [ 31:0]  branch_pc_o           // 分支目标地址（打拍后输出）
    ,output          branch_d_request_o    // 当拍的直接分支请求（组合逻辑，不打拍）
    ,output [ 31:0]  branch_d_pc_o         // 当拍的直接分支目标地址
    ,output [  1:0]  branch_d_priv_o       // 当拍的目标特权级（本实现固定为 0）
    ,output [ 31:0]  writeback_value_o     // 写回寄存器堆的结果值（ALU 结果打拍）
);



//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

//-------------------------------------------------------------
// Opcode decode —— 立即数解码
// 根据 RISC-V 规范从 32 位指令字中提取各类型立即数
//-------------------------------------------------------------
// U 型立即数：高20位来自指令 [31:12]，低12位置零（用于 LUI/AUIPC）
reg [31:0]  imm20_r;
// I 型立即数：指令 [31:20] 的12位有符号扩展（用于 ADDI/LOAD/JALR 等）
reg [31:0]  imm12_r;
// B 型立即数：条件分支偏移，13位有符号扩展，位域分散在指令中需重新拼接
reg [31:0]  bimm_r;
// J 型立即数：JAL 跳转偏移，21位有符号扩展，位域同样分散需拼接
reg [31:0]  jimm20_r;
// 移位量：指令 [24:20]，用于 SLLI/SRLI/SRAI 等立即数移位指令
reg [4:0]   shamt_r;

// 组合逻辑：从指令字中解码各格式立即数
// I 型：imm[11:0]  = opcode[31:20]，符号扩展至32位
// B 型：imm[12|10:5|4:1|11] 分散于指令中，需按规范拼接后左移1位
// J 型：imm[20|10:1|11|19:12] 分散于指令中，需按规范拼接后左移1位
always @ *
begin
    imm20_r     = {opcode_opcode_i[31:12], 12'b0};
    imm12_r     = {{20{opcode_opcode_i[31]}}, opcode_opcode_i[31:20]};
    bimm_r      = {{19{opcode_opcode_i[31]}}, opcode_opcode_i[31], opcode_opcode_i[7], opcode_opcode_i[30:25], opcode_opcode_i[11:8], 1'b0};
    jimm20_r    = {{12{opcode_opcode_i[31]}}, opcode_opcode_i[19:12], opcode_opcode_i[20], opcode_opcode_i[30:25], opcode_opcode_i[24:21], 1'b0};
    shamt_r     = opcode_opcode_i[24:20];
end

//-------------------------------------------------------------
// Execute - ALU operations —— ALU 操作选择
// 根据指令类型选择 ALU 功能码（alu_func_r）及两个操作数
// R 型指令：操作数均来自寄存器；I 型指令：操作数 B 来自立即数
//-------------------------------------------------------------
// ALU 功能码选择信号（见 riscv_defs.v 中 ALU_xxx 宏定义）
reg [3:0]  alu_func_r;
// ALU 操作数 A（一般为 rs1，LUI 为立即数，JAL/JALR 为 PC）
reg [31:0] alu_input_a_r;
// ALU 操作数 B（R 型为 rs2，I 型为立即数，JAL/JALR 固定为 4）
reg [31:0] alu_input_b_r;

// 组合逻辑：ALU 操作选择
// 对每条 R/I 型整数运算指令进行掩码匹配，选择对应功能码与操作数
// - R 型（ADD/SUB/AND/OR/XOR/SLL/SRL/SRA/SLT/SLTU）：A=rs1, B=rs2
// - I 型（ADDI/ANDI/ORI/XORI/SLTI/SLTIU）：A=rs1, B=imm12
// - 移位立即数（SLLI/SRLI/SRAI）：A=rs1, B={27'b0, shamt}
// - LUI：直接将 imm20 传给 A，ALU 功能码为 NONE（直通）
// - AUIPC：A=PC, B=imm20，ALU 做加法得到 PC+imm20
// - JAL/JALR：A=PC, B=4，ALU 计算返回地址 PC+4
always @ *
begin
    alu_func_r     = `ALU_NONE;
    alu_input_a_r  = 32'b0;
    alu_input_b_r  = 32'b0;

    if ((opcode_opcode_i & `INST_ADD_MASK) == `INST_ADD) // add
    begin
        alu_func_r     = `ALU_ADD;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_AND_MASK) == `INST_AND) // and
    begin
        alu_func_r     = `ALU_AND;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_OR_MASK) == `INST_OR) // or
    begin
        alu_func_r     = `ALU_OR;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_SLL_MASK) == `INST_SLL) // sll
    begin
        alu_func_r     = `ALU_SHIFTL;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_SRA_MASK) == `INST_SRA) // sra
    begin
        alu_func_r     = `ALU_SHIFTR_ARITH;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_SRL_MASK) == `INST_SRL) // srl
    begin
        alu_func_r     = `ALU_SHIFTR;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_SUB_MASK) == `INST_SUB) // sub
    begin
        alu_func_r     = `ALU_SUB;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_XOR_MASK) == `INST_XOR) // xor
    begin
        alu_func_r     = `ALU_XOR;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_SLT_MASK) == `INST_SLT) // slt
    begin
        alu_func_r     = `ALU_LESS_THAN_SIGNED;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_SLTU_MASK) == `INST_SLTU) // sltu
    begin
        alu_func_r     = `ALU_LESS_THAN;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_ADDI_MASK) == `INST_ADDI) // addi
    begin
        alu_func_r     = `ALU_ADD;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = imm12_r;
    end
    else if ((opcode_opcode_i & `INST_ANDI_MASK) == `INST_ANDI) // andi
    begin
        alu_func_r     = `ALU_AND;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = imm12_r;
    end
    else if ((opcode_opcode_i & `INST_SLTI_MASK) == `INST_SLTI) // slti
    begin
        alu_func_r     = `ALU_LESS_THAN_SIGNED;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = imm12_r;
    end
    else if ((opcode_opcode_i & `INST_SLTIU_MASK) == `INST_SLTIU) // sltiu
    begin
        alu_func_r     = `ALU_LESS_THAN;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = imm12_r;
    end
    else if ((opcode_opcode_i & `INST_ORI_MASK) == `INST_ORI) // ori
    begin
        alu_func_r     = `ALU_OR;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = imm12_r;
    end
    else if ((opcode_opcode_i & `INST_XORI_MASK) == `INST_XORI) // xori
    begin
        alu_func_r     = `ALU_XOR;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = imm12_r;
    end
    else if ((opcode_opcode_i & `INST_SLLI_MASK) == `INST_SLLI) // slli
    begin
        alu_func_r     = `ALU_SHIFTL;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = {27'b0, shamt_r};
    end
    else if ((opcode_opcode_i & `INST_SRLI_MASK) == `INST_SRLI) // srli
    begin
        alu_func_r     = `ALU_SHIFTR;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = {27'b0, shamt_r};
    end
    else if ((opcode_opcode_i & `INST_SRAI_MASK) == `INST_SRAI) // srai
    begin
        alu_func_r     = `ALU_SHIFTR_ARITH;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = {27'b0, shamt_r};
    end
    else if ((opcode_opcode_i & `INST_LUI_MASK) == `INST_LUI) // lui
    begin
        alu_input_a_r  = imm20_r;
    end
    else if ((opcode_opcode_i & `INST_AUIPC_MASK) == `INST_AUIPC) // auipc
    begin
        alu_func_r     = `ALU_ADD;
        alu_input_a_r  = opcode_pc_i;
        alu_input_b_r  = imm20_r;
    end     
    else if (((opcode_opcode_i & `INST_JAL_MASK) == `INST_JAL) || ((opcode_opcode_i & `INST_JALR_MASK) == `INST_JALR)) // jal, jalr
    begin
        alu_func_r     = `ALU_ADD;
        alu_input_a_r  = opcode_pc_i;
        alu_input_b_r  = 32'd4;
    end
end


//-------------------------------------------------------------
// ALU —— 算术逻辑单元实例化
// 将功能码和操作数送入 riscv_alu，得到组合逻辑结果 alu_p_w
//-------------------------------------------------------------
wire [31:0]  alu_p_w;
riscv_alu
u_alu
(
    .alu_op_i(alu_func_r),
    .alu_a_i(alu_input_a_r),
    .alu_b_i(alu_input_b_r),
    .alu_p_o(alu_p_w)
);

//-------------------------------------------------------------
// Flop ALU output —— ALU 结果寄存（打拍）
// 在时钟上升沿将 ALU 组合逻辑结果锁存到 result_q
// hold_i 高电平时流水线暂停，result_q 保持不变
//-------------------------------------------------------------
reg [31:0] result_q; // 已锁存的 ALU 结果，直接连接到写回总线
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    result_q  <= 32'b0;
else if (~hold_i)
    result_q <= alu_p_w;

// 写回值：将锁存后的 ALU 结果输出至寄存器堆写回总线
assign writeback_value_o  = result_q;

//-----------------------------------------------------------------
// less_than_signed: Less than operator (signed)
// 有符号小于比较函数
// Inputs: x = left operand, y = right operand
// Return: (int)x < (int)y
// 实现原理：若符号位不同，符号位为 1 的操作数更小；
//           若符号位相同，通过差值的符号位判断大小关系
//-----------------------------------------------------------------
function [0:0] less_than_signed;
    input  [31:0] x;
    input  [31:0] y;
    reg [31:0] v;
begin
    v = (x - y);
    if (x[31] != y[31])
        less_than_signed = x[31];
    else
        less_than_signed = v[31];
end
endfunction

//-----------------------------------------------------------------
// greater_than_signed: Greater than operator (signed)
// 有符号大于比较函数
// Inputs: x = left operand, y = right operand
// Return: (int)x > (int)y
// 实现原理：与 less_than_signed 对称，比较 y-x 的符号位
//-----------------------------------------------------------------
function [0:0] greater_than_signed;
    input  [31:0] x;
    input  [31:0] y;
    reg [31:0] v;
begin
    v = (y - x);
    if (x[31] != y[31])
        greater_than_signed = y[31];
    else
        greater_than_signed = v[31];
end
endfunction

//-------------------------------------------------------------
// Execute - Branch operations —— 分支条件判断与目标计算
// 判断当前指令是否为分支/跳转指令，并计算跳转目标地址
//-------------------------------------------------------------
reg        branch_r;        // 当前指令为分支/跳转类型（组合逻辑）
reg        branch_taken_r;  // 分支跳转条件成立（组合逻辑）
reg [31:0] branch_target_r; // 分支目标地址（组合逻辑）
reg        branch_call_r;   // 是否为函数调用（JAL/JALR 且 rd=x1）
reg        branch_ret_r;    // 是否为函数返回（JALR，rs1=x1，偏移=0）
reg        branch_jmp_r;    // 是否为普通跳转（非 call 也非 ret）

// 组合逻辑：分支类型识别与目标地址计算
// - JAL  : 无条件，目标 = PC + J 型立即数；rd=x1 时标记为 call
// - JALR : 无条件，目标 = (rs1 + I 型立即数) & ~1；rs1=x1 且偏移=0 时为 ret
// - BEQ  : 相等跳转，目标 = PC + B 型立即数
// - BNE  : 不等跳转
// - BLT  : 有符号小于跳转
// - BGE  : 有符号大于等于跳转
// - BLTU : 无符号小于跳转
// - BGEU : 无符号大于等于跳转
always @ *
begin
    branch_r        = 1'b0;
    branch_taken_r  = 1'b0;
    branch_call_r   = 1'b0;
    branch_ret_r    = 1'b0;
    branch_jmp_r    = 1'b0;

    // Default branch_r target is relative to current PC
    branch_target_r = opcode_pc_i + bimm_r;

    if ((opcode_opcode_i & `INST_JAL_MASK) == `INST_JAL) // jal
    begin
        branch_r        = 1'b1;
        branch_taken_r  = 1'b1;
        branch_target_r = opcode_pc_i + jimm20_r;
        branch_call_r   = (opcode_rd_idx_i == 5'd1); // RA
        branch_jmp_r    = 1'b1;
    end
    else if ((opcode_opcode_i & `INST_JALR_MASK) == `INST_JALR) // jalr
    begin
        branch_r            = 1'b1;
        branch_taken_r      = 1'b1;
        branch_target_r     = opcode_ra_operand_i + imm12_r;
        branch_target_r[0]  = 1'b0;
        branch_ret_r        = (opcode_ra_idx_i == 5'd1 && imm12_r[11:0] == 12'b0); // RA
        branch_call_r       = ~branch_ret_r && (opcode_rd_idx_i == 5'd1); // RA
        branch_jmp_r        = ~(branch_call_r | branch_ret_r);
    end
    else if ((opcode_opcode_i & `INST_BEQ_MASK) == `INST_BEQ) // beq
    begin
        branch_r      = 1'b1;
        branch_taken_r= (opcode_ra_operand_i == opcode_rb_operand_i);
    end
    else if ((opcode_opcode_i & `INST_BNE_MASK) == `INST_BNE) // bne
    begin
        branch_r      = 1'b1;    
        branch_taken_r= (opcode_ra_operand_i != opcode_rb_operand_i);
    end
    else if ((opcode_opcode_i & `INST_BLT_MASK) == `INST_BLT) // blt
    begin
        branch_r      = 1'b1;
        branch_taken_r= less_than_signed(opcode_ra_operand_i, opcode_rb_operand_i);
    end
    else if ((opcode_opcode_i & `INST_BGE_MASK) == `INST_BGE) // bge
    begin
        branch_r      = 1'b1;    
        branch_taken_r= greater_than_signed(opcode_ra_operand_i,opcode_rb_operand_i) | (opcode_ra_operand_i == opcode_rb_operand_i);
    end
    else if ((opcode_opcode_i & `INST_BLTU_MASK) == `INST_BLTU) // bltu
    begin
        branch_r      = 1'b1;    
        branch_taken_r= (opcode_ra_operand_i < opcode_rb_operand_i);
    end
    else if ((opcode_opcode_i & `INST_BGEU_MASK) == `INST_BGEU) // bgeu
    begin
        branch_r      = 1'b1;
        branch_taken_r= (opcode_ra_operand_i >= opcode_rb_operand_i);
    end
end

// 分支结果打拍寄存器：将本周期组合逻辑判断结果锁存到下一拍输出
reg        branch_taken_q;  // 已锁存：分支跳转
reg        branch_ntaken_q; // 已锁存：分支未跳转
reg [31:0] pc_x_q;          // 已锁存：下一 PC（跳转目标或 PC+4）
reg [31:0] pc_m_q;          // 已锁存：分支指令自身 PC（用于 BTB 更新）
reg        branch_call_q;   // 已锁存：函数调用标志
reg        branch_ret_q;    // 已锁存：函数返回标志
reg        branch_jmp_q;    // 已锁存：普通跳转标志

// 时序逻辑：在时钟上升沿锁存分支判断结果
// opcode_valid_i 有效时才更新，否则保持复位后的 0 值
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    branch_taken_q   <= 1'b0;
    branch_ntaken_q  <= 1'b0;
    pc_x_q           <= 32'b0;
    pc_m_q           <= 32'b0;
    branch_call_q    <= 1'b0;
    branch_ret_q     <= 1'b0;
    branch_jmp_q     <= 1'b0;
end
else if (opcode_valid_i)
begin
    branch_taken_q   <= branch_r && opcode_valid_i & branch_taken_r;
    branch_ntaken_q  <= branch_r && opcode_valid_i & ~branch_taken_r;
    pc_x_q           <= branch_taken_r ? branch_target_r : opcode_pc_i + 32'd4;
    branch_call_q    <= branch_r && opcode_valid_i && branch_call_r;
    branch_ret_q     <= branch_r && opcode_valid_i && branch_ret_r;
    branch_jmp_q     <= branch_r && opcode_valid_i && branch_jmp_r;
    pc_m_q           <= opcode_pc_i;
end

// 打拍后的分支请求输出：taken 或 not-taken 均需通知前级更新 BTB
assign branch_request_o   = branch_taken_q | branch_ntaken_q;
assign branch_is_taken_o  = branch_taken_q;
assign branch_is_not_taken_o = branch_ntaken_q;
assign branch_source_o    = pc_m_q;   // 分支指令的 PC
assign branch_pc_o        = pc_x_q;   // 分支目标（or PC+4）
assign branch_is_call_o   = branch_call_q;
assign branch_is_ret_o    = branch_ret_q;
assign branch_is_jmp_o    = branch_jmp_q;

// 当拍（组合逻辑）的直接分支请求：供前端在当拍即重定向取指
// branch_d_request_o 在分支确实跳转时有效，无需等到下一拍
assign branch_d_request_o = (branch_r && opcode_valid_i && branch_taken_r);
assign branch_d_pc_o      = branch_target_r;
assign branch_d_priv_o    = 2'b0; // don't care



endmodule
