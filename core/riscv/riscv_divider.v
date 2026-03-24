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

// ============================================================
// 模块: riscv_divider
// 功能: RV32M 除法器，支持以下四种除法/取余指令：
//   DIV  - 有符号整数除法，结果取商
//   DIVU - 无符号整数除法，结果取商
//   REM  - 有符号整数取余
//   REMU - 无符号整数取余
// 实现：非恢复余数法（移位减法迭代），逐位计算商
//   - 启动时将被除数和除数初始化，除数对齐到高位（左移31位）
//   - 每周期将除数右移1位，比较后决定商的当前位是否为1
//   - 总共需要32个周期完成一次除法
//   - 有符号运算时先转为正数计算，最后根据符号决定是否取反
// 状态机：div_busy_q 表示运算进行中，div_complete_w 表示运算完成
// ============================================================
module riscv_divider
(
    // Inputs
     input           clk_i              // 系统时钟
    ,input           rst_i              // 同步复位（高有效）
    ,input           opcode_valid_i     // 当前指令有效标志
    ,input  [ 31:0]  opcode_opcode_i    // 指令编码（用于区分DIV/DIVU/REM/REMU）
    ,input  [ 31:0]  opcode_pc_i        // 当前指令PC（本模块未使用）
    ,input           opcode_invalid_i   // 指令非法标志（本模块未使用）
    ,input  [  4:0]  opcode_rd_idx_i    // 目标寄存器索引（本模块未使用）
    ,input  [  4:0]  opcode_ra_idx_i    // 源寄存器A索引（本模块未使用）
    ,input  [  4:0]  opcode_rb_idx_i    // 源寄存器B索引（本模块未使用）
    ,input  [ 31:0]  opcode_ra_operand_i // 被除数（rs1）
    ,input  [ 31:0]  opcode_rb_operand_i // 除数（rs2）

    // Outputs
    ,output          writeback_valid_o  // 结果有效脉冲（运算完成后拉高一拍）
    ,output [ 31:0]  writeback_value_o  // 除法/取余结果写回值
);



//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

//-------------------------------------------------------------
// Registers / Wires（除法器状态寄存器）
//-------------------------------------------------------------
reg          valid_q;       // 结果有效寄存器：运算完成后输出一拍有效脉冲
reg  [31:0]  wb_result_q;   // 写回结果寄存器：保存最终的商或余数

//-------------------------------------------------------------
// Divider（除法器主体逻辑）
//-------------------------------------------------------------
// 指令类型识别（互斥）
wire inst_div_w         = (opcode_opcode_i & `INST_DIV_MASK) == `INST_DIV;   // 有符号除法
wire inst_divu_w        = (opcode_opcode_i & `INST_DIVU_MASK) == `INST_DIVU; // 无符号除法
wire inst_rem_w         = (opcode_opcode_i & `INST_REM_MASK) == `INST_REM;   // 有符号取余
wire inst_remu_w        = (opcode_opcode_i & `INST_REMU_MASK) == `INST_REMU; // 无符号取余

// 任意除法/取余指令激活标志
wire div_rem_inst_w     = ((opcode_opcode_i & `INST_DIV_MASK) == `INST_DIV)  || 
                          ((opcode_opcode_i & `INST_DIVU_MASK) == `INST_DIVU) ||
                          ((opcode_opcode_i & `INST_REM_MASK) == `INST_REM)  ||
                          ((opcode_opcode_i & `INST_REMU_MASK) == `INST_REMU);

// 有符号运算标志（DIV/REM需要处理符号）
wire signed_operation_w = ((opcode_opcode_i & `INST_DIV_MASK) == `INST_DIV) || ((opcode_opcode_i & `INST_REM_MASK) == `INST_REM);
// 除法运算标志（DIV/DIVU结果取商，REM/REMU结果取余数）
wire div_operation_w    = ((opcode_opcode_i & `INST_DIV_MASK) == `INST_DIV) || ((opcode_opcode_i & `INST_DIVU_MASK) == `INST_DIVU);

reg [31:0] dividend_q;  // 当前被除数（迭代过程中逐步减去除数）
reg [62:0] divisor_q;   // 除数移位寄存器（63位宽，从高位对齐开始逐步右移）
reg [31:0] quotient_q;  // 商寄存器（逐位累积）
reg [31:0] q_mask_q;    // 商位掩码（从 MSB 0x80000000 逐步右移，标识当前计算的商位）
reg        div_inst_q;  // 锁存的除法/取余标志（1=除法取商，0=取余）
reg        div_busy_q;  // 除法器忙状态（1=正在计算，0=空闲）
reg        invert_res_q;// 结果取反标志（有符号运算且结果为负时需要取反）

// 除法启动：当有效指令到来且是除法/取余指令时启动
wire div_start_w    = opcode_valid_i & div_rem_inst_w;
// 除法完成：q_mask_q 移出全部32位变为0时完成（共32次迭代）
wire div_complete_w = !(|q_mask_q) & div_busy_q;

// 除法器状态机及迭代逻辑
always @(posedge clk_i or posedge rst_i)
if (rst_i)
begin
    div_busy_q     <= 1'b0;
    dividend_q     <= 32'b0;
    divisor_q      <= 63'b0;
    invert_res_q   <= 1'b0;
    quotient_q     <= 32'b0;
    q_mask_q       <= 32'b0;
    div_inst_q     <= 1'b0;
end
else if (div_start_w)
begin
    // 启动新的除法运算：初始化各状态寄存器
    div_busy_q     <= 1'b1;
    div_inst_q     <= div_operation_w; // 锁存是除法还是取余

    // 有符号运算：若被除数为负则取其绝对值（先取反加1）
    if (signed_operation_w && opcode_ra_operand_i[31])
        dividend_q <= -opcode_ra_operand_i;
    else
        dividend_q <= opcode_ra_operand_i;

    // 有符号运算：若除数为负则取其绝对值，并将除数对齐到63位高端（左移31位）
    if (signed_operation_w && opcode_rb_operand_i[31])
        divisor_q <= {-opcode_rb_operand_i, 31'b0};
    else
        divisor_q <= {opcode_rb_operand_i, 31'b0};

    // 判断结果符号：
    // DIV：被除数与除数符号不同（且除数非零）时商为负
    // REM：被除数为负时余数为负（RISC-V规范：余数符号与被除数相同）
    invert_res_q  <= (((opcode_opcode_i & `INST_DIV_MASK) == `INST_DIV) && (opcode_ra_operand_i[31] != opcode_rb_operand_i[31]) && |opcode_rb_operand_i) || 
                     (((opcode_opcode_i & `INST_REM_MASK) == `INST_REM) && opcode_ra_operand_i[31]);

    quotient_q     <= 32'b0;           // 商清零
    q_mask_q       <= 32'h80000000;   // 从最高位开始计算
end
else if (div_complete_w)
begin
    // 32次迭代完成，清除忙状态
    div_busy_q <= 1'b0;
end
else if (div_busy_q)
begin
    // 迭代运算：非恢复余数移位除法
    // 若当前余数（dividend_q） >= 当前除数（divisor_q低32位），则减去并置商位为1
    if (divisor_q <= {31'b0, dividend_q})
    begin
        dividend_q <= dividend_q - divisor_q[31:0]; // 更新余数
        quotient_q <= quotient_q | q_mask_q;         // 置当前商位为1
    end

    divisor_q <= {1'b0, divisor_q[62:1]}; // 除数右移1位（准备下次迭代）
    q_mask_q  <= {1'b0, q_mask_q[31:1]};  // 商掩码右移1位（移向下一位）
end

// 最终结果组合逻辑：根据指令类型和符号标志选取商或余数，必要时取反
reg [31:0] div_result_r;
always @ *
begin
    div_result_r = 32'b0;

    if (div_inst_q)
        // 除法指令：输出商，需要时取反（有符号负结果）
        div_result_r = invert_res_q ? -quotient_q : quotient_q;
    else
        // 取余指令：输出余数（dividend_q为最终余数），需要时取反
        div_result_r = invert_res_q ? -dividend_q : dividend_q;
end

// 结果有效脉冲寄存器：在除法完成的下一拍产生一拍有效信号
always @(posedge clk_i or posedge rst_i)
if (rst_i)
    valid_q <= 1'b0;
else
    valid_q <= div_complete_w;

// 写回结果寄存器：在除法完成时锁存最终结果
always @(posedge clk_i or posedge rst_i)
if (rst_i)
    wb_result_q <= 32'b0;
else if (div_complete_w)
    wb_result_q <= div_result_r;

assign writeback_valid_o = valid_q;     // 结果有效输出
assign writeback_value_o  = wb_result_q; // 结果数据输出



endmodule
