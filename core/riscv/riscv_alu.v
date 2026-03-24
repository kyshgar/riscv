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
// 模块: riscv_alu
// 功能: 算术逻辑单元（ALU） - 纯组合逻辑
//       支持加法、减法、按位与/或/异或、逻辑/算术移位、有符号/无符号比较
//       操作码由 riscv_defs.v 中的 ALU_* 宏定义
// ============================================================
module riscv_alu
(
    // Inputs
     input  [  3:0]  alu_op_i  // ALU操作码，选择执行哪种运算（见 ALU_* 宏定义）
    ,input  [ 31:0]  alu_a_i   // 操作数A（通常来自寄存器堆rs1或PC）
    ,input  [ 31:0]  alu_b_i   // 操作数B（来自寄存器堆rs2或立即数）

    // Outputs
    ,output [ 31:0]  alu_p_o   // ALU运算结果输出
);

//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

//-----------------------------------------------------------------
// Registers（中间结果寄存器）
//-----------------------------------------------------------------
reg [31:0]      result_r;         // ALU最终结果寄存器

// 右移中间流水级寄存器（按位逐级移位，每级移1/2/4/8/16位）
reg [31:16]     shift_right_fill_r; // 算术右移时高位填充值（全1或全0）
reg [31:0]      shift_right_1_r;    // 移位1位后的中间值
reg [31:0]      shift_right_2_r;    // 移位2位后的中间值
reg [31:0]      shift_right_4_r;    // 移位4位后的中间值
reg [31:0]      shift_right_8_r;    // 移位8位后的中间值

// 左移中间流水级寄存器
reg [31:0]      shift_left_1_r;   // 左移1位后的中间值
reg [31:0]      shift_left_2_r;   // 左移2位后的中间值
reg [31:0]      shift_left_4_r;   // 左移4位后的中间值
reg [31:0]      shift_left_8_r;   // 左移8位后的中间值

wire [31:0]     sub_res_w = alu_a_i - alu_b_i; // 预计算减法结果，用于SUB和有符号比较

//-----------------------------------------------------------------
// ALU（组合逻辑主体：根据 alu_op_i 选择执行对应运算）
// 移位实现：桶形移位器，按 alu_b_i[4:0] 的各位依次移1/2/4/8/16位
//-----------------------------------------------------------------
always @ (alu_op_i or alu_a_i or alu_b_i or sub_res_w)
begin
    // 初始化各移位中间值为0，防止锁存
    shift_right_fill_r = 16'b0;
    shift_right_1_r = 32'b0;
    shift_right_2_r = 32'b0;
    shift_right_4_r = 32'b0;
    shift_right_8_r = 32'b0;

    shift_left_1_r = 32'b0;
    shift_left_2_r = 32'b0;
    shift_left_4_r = 32'b0;
    shift_left_8_r = 32'b0;

    case (alu_op_i)
       //----------------------------------------------
       // Shift Left（逻辑左移，SLL/SLLI）
       // 桶形左移：依据移位量各位依次左移1/2/4/8/16位
       //----------------------------------------------   
       `ALU_SHIFTL :
       begin
            // bit[0]=1 则左移1位，低位补0
            if (alu_b_i[0] == 1'b1)
                shift_left_1_r = {alu_a_i[30:0],1'b0};
            else
                shift_left_1_r = alu_a_i;

            // bit[1]=1 则再左移2位，低位补00
            if (alu_b_i[1] == 1'b1)
                shift_left_2_r = {shift_left_1_r[29:0],2'b00};
            else
                shift_left_2_r = shift_left_1_r;

            // bit[2]=1 则再左移4位，低位补0000
            if (alu_b_i[2] == 1'b1)
                shift_left_4_r = {shift_left_2_r[27:0],4'b0000};
            else
                shift_left_4_r = shift_left_2_r;

            // bit[3]=1 则再左移8位，低位补00000000
            if (alu_b_i[3] == 1'b1)
                shift_left_8_r = {shift_left_4_r[23:0],8'b00000000};
            else
                shift_left_8_r = shift_left_4_r;

            // bit[4]=1 则再左移16位，低位补0x0000
            if (alu_b_i[4] == 1'b1)
                result_r = {shift_left_8_r[15:0],16'b0000000000000000};
            else
                result_r = shift_left_8_r;
       end
       //----------------------------------------------
       // Shift Right（逻辑右移SRL/SRLI 或 算术右移SRA/SRAI）
       // 算术右移时高位填充符号位（alu_a_i[31]）
       //----------------------------------------------
       `ALU_SHIFTR, `ALU_SHIFTR_ARITH:
       begin
            // 算术右移：若操作数最高位为1（负数），填充位全为1；否则全为0
            if (alu_a_i[31] == 1'b1 && alu_op_i == `ALU_SHIFTR_ARITH)
                shift_right_fill_r = 16'b1111111111111111;
            else
                shift_right_fill_r = 16'b0000000000000000;

            // bit[0]=1 则右移1位，高位填充符号扩展
            if (alu_b_i[0] == 1'b1)
                shift_right_1_r = {shift_right_fill_r[31], alu_a_i[31:1]};
            else
                shift_right_1_r = alu_a_i;

            // bit[1]=1 则再右移2位
            if (alu_b_i[1] == 1'b1)
                shift_right_2_r = {shift_right_fill_r[31:30], shift_right_1_r[31:2]};
            else
                shift_right_2_r = shift_right_1_r;

            // bit[2]=1 则再右移4位
            if (alu_b_i[2] == 1'b1)
                shift_right_4_r = {shift_right_fill_r[31:28], shift_right_2_r[31:4]};
            else
                shift_right_4_r = shift_right_2_r;

            // bit[3]=1 则再右移8位
            if (alu_b_i[3] == 1'b1)
                shift_right_8_r = {shift_right_fill_r[31:24], shift_right_4_r[31:8]};
            else
                shift_right_8_r = shift_right_4_r;

            // bit[4]=1 则再右移16位，完成全部移位
            if (alu_b_i[4] == 1'b1)
                result_r = {shift_right_fill_r[31:16], shift_right_8_r[31:16]};
            else
                result_r = shift_right_8_r;
       end       
       //----------------------------------------------
       // Arithmetic（算术运算）
       //----------------------------------------------
       `ALU_ADD : // 加法（ADD/ADDI/LUI+偏移/AUIPC等）
       begin
            result_r      = (alu_a_i + alu_b_i);
       end
       `ALU_SUB : // 减法（SUB），结果来自预计算的 sub_res_w
       begin
            result_r      = sub_res_w;
       end
       //----------------------------------------------
       // Logical（逻辑运算）
       //----------------------------------------------       
       `ALU_AND : // 按位与（AND/ANDI）
       begin
            result_r      = (alu_a_i & alu_b_i);
       end
       `ALU_OR  : // 按位或（OR/ORI）
       begin
            result_r      = (alu_a_i | alu_b_i);
       end
       `ALU_XOR : // 按位异或（XOR/XORI）
       begin
            result_r      = (alu_a_i ^ alu_b_i);
       end
       //----------------------------------------------
       // Comparision（比较运算，结果为0或1）
       //----------------------------------------------
       `ALU_LESS_THAN : // 无符号小于（SLTU/SLTIU）：a < b（无符号）时结果为1
       begin
            result_r      = (alu_a_i < alu_b_i) ? 32'h1 : 32'h0;
       end
       `ALU_LESS_THAN_SIGNED : // 有符号小于（SLT/SLTI）：先判断符号位，再用减法结果符号位决定
       begin
            if (alu_a_i[31] != alu_b_i[31])
                // 符号位不同：a为负（MSB=1）则a < b
                result_r  = alu_a_i[31] ? 32'h1 : 32'h0;
            else
                // 符号位相同：由减法结果的符号位判断（sub < 0 则 a < b）
                result_r  = sub_res_w[31] ? 32'h1 : 32'h0;            
       end       
       default  : // 默认：直通操作数A（ALU_NONE 时使用）
       begin
            result_r      = alu_a_i;
       end
    endcase
end

// 将组合逻辑结果连接到输出端口
assign alu_p_o    = result_r;

endmodule
