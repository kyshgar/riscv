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
 * 模块功能概述：riscv_pipe_ctrl —— RISC-V 流水线控制器
 *
 * 本模块是流水线的控制核心，负责管理指令从发射（Issue）经 E1、E2 到写回（WB）
 * 各阶段的状态追踪，主要功能包括：
 *   1. E1/E2/WB 流水级状态寄存器追踪：保存每级的指令有效标志、控制信息、
 *      PC、操作码、操作数及异常编码；
 *   2. RAW 冒险检测（通过 scoreboard / stall 信号）：
 *      - 除法（DIV）未完成时 stall；
 *      - 加载/存储（LSU）E2 响应未到时 stall；
 *   3. 数据前递（Bypass/Forward）MUX：
 *      - SUPPORT_LOAD_BYPASS：E2 级加载结果直接旁路到 result_e2_r；
 *      - SUPPORT_MUL_BYPASS ：E2 级乘法结果直接旁路到 result_e2_r；
 *   4. Squash（流水线冲刷）逻辑：
 *      - 检测 E2 级异常（内存故障、对齐错误、中断等），
 *        拉高 squash_e1_e2_o 使 E1/E2 级的指令无效化；
 *   5. 写回（WB）级提交：将 E2 结果锁存后输出到寄存器堆写口及 CSR 写口；
 *   6. 异常上报：通过 exception_wb_o 将最终确定的异常类型传递给 CSR/陷阱处理器。
 */
module riscv_pipe_ctrl
//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
#(
     parameter SUPPORT_LOAD_BYPASS = 1 // 使能加载结果在 E2 级直接旁路（减少加载-使用延迟）
    ,parameter SUPPORT_MUL_BYPASS  = 1 // 使能乘法结果在 E2 级直接旁路（减少乘法-使用延迟）
)
//-----------------------------------------------------------------
// Ports
//-----------------------------------------------------------------
(
     input           clk_i  // 时钟信号
    ,input           rst_i  // 异步复位（高有效）

    // Issue —— 发射级输入信号
    ,input           issue_valid_i         // 本周期有指令被发射
    ,input           issue_accept_i        // 本级接受发射（未被 stall/squash）
    ,input           issue_stall_i         // 流水线暂停（整体 stall）
    ,input           issue_lsu_i           // 发射的指令是 LSU 操作
    ,input           issue_csr_i           // 发射的指令是 CSR 操作
    ,input           issue_div_i           // 发射的指令是除法
    ,input           issue_mul_i           // 发射的指令是乘法
    ,input           issue_branch_i        // 发射的指令是分支/跳转
    ,input           issue_rd_valid_i      // 指令有目标寄存器写结果
    ,input  [4:0]    issue_rd_i            // 目标寄存器索引
    ,input  [5:0]    issue_exception_i     // 发射级检测到的异常（译码/取指异常）
    ,input           take_interrupt_i      // 有中断待处理，本指令替换为中断入口
    ,input           issue_branch_taken_i  // 发射时分支已确定跳转
    ,input [31:0]    issue_branch_target_i // 发射时的分支目标地址
    ,input [31:0]    issue_pc_i            // 发射指令的 PC
    ,input [31:0]    issue_opcode_i        // 发射指令的原始指令字
    ,input [31:0]    issue_operand_ra_i    // 发射时的 rs1 操作数（含前递）
    ,input [31:0]    issue_operand_rb_i    // 发射时的 rs2 操作数（含前递）

    // Execution stage 1: ALU result —— E1 级 ALU 组合逻辑结果
    ,input [31:0]    alu_result_e1_i       // ALU 在 E1 级的计算结果

    // Execution stage 1: CSR read result / early exceptions
    ,input [ 31:0]   csr_result_value_e1_i     // CSR 读取值
    ,input           csr_result_write_e1_i     // CSR 本次操作需要写回
    ,input [ 31:0]   csr_result_wdata_e1_i     // CSR 写入数据
    ,input [  5:0]   csr_result_exception_e1_i // CSR 操作产生的异常（如非法CSR访问）

    // Execution stage 1 outputs —— E1 级输出（供 LSU/MUL/DIV 等使用）
    ,output          load_e1_o      // E1 级有加载指令
    ,output          store_e1_o     // E1 级有存储指令
    ,output          mul_e1_o       // E1 级有乘法指令
    ,output          branch_e1_o    // E1 级有分支指令
    ,output [  4:0]  rd_e1_o        // E1 级目标寄存器索引（无效时为0）
    ,output [31:0]   pc_e1_o        // E1 级指令 PC
    ,output [31:0]   opcode_e1_o    // E1 级指令原始编码
    ,output [31:0]   operand_ra_e1_o // E1 级 rs1 操作数
    ,output [31:0]   operand_rb_e1_o // E1 级 rs2 操作数

    // Execution stage 2: Other results —— E2 级来自 LSU/MUL 的结果
    ,input           mem_complete_i      // LSU 在 E2 级已完成（响应有效）
    ,input [31:0]    mem_result_e2_i     // E2 级内存读数据（加载结果）
    ,input  [5:0]    mem_exception_e2_i  // E2 级内存访问异常
    ,input [31:0]    mul_result_e2_i     // E2 级乘法结果

    // Execution stage 2 outputs —— E2 级输出
    ,output          load_e2_o     // E2 级有加载指令
    ,output          mul_e2_o      // E2 级有乘法指令
    ,output [  4:0]  rd_e2_o       // E2 级目标寄存器索引（含 valid 掩码）
    ,output [31:0]   result_e2_o   // E2 级结果（含旁路后选择）

    // Out of pipe: Divide Result —— 流水线外的除法结果
    ,input           div_complete_i  // 除法单元完成信号
    ,input  [31:0]   div_result_i    // 除法结果

    // Commit / Writeback —— 写回级提交信号
    ,output          valid_wb_o         // 写回级指令有效（可写寄存器堆）
    ,output          csr_wb_o           // 写回级有 CSR 操作
    ,output [  4:0]  rd_wb_o            // 写回目标寄存器索引
    ,output [31:0]   result_wb_o        // 写回结果值
    ,output [31:0]   pc_wb_o            // 写回指令 PC
    ,output [31:0]   opcode_wb_o        // 写回指令原始编码
    ,output [31:0]   operand_ra_wb_o    // 写回级 rs1 操作数
    ,output [31:0]   operand_rb_wb_o    // 写回级 rs2 操作数
    ,output [5:0]    exception_wb_o     // 写回级最终异常编码
    ,output          csr_write_wb_o     // 写回级 CSR 写使能
    ,output [11:0]   csr_waddr_wb_o     // 写回级 CSR 写地址（指令[31:20]）
    ,output [31:0]   csr_wdata_wb_o     // 写回级 CSR 写数据

    ,output          stall_o        // 流水线暂停请求（DIV/LSU 未完成）
    ,output          squash_e1_e2_o // E1/E2 冲刷信号（异常/中断导致）
    ,input           squash_e1_e2_i // 外部要求 E1/E2 冲刷（如分支错误预测）
    ,input           squash_wb_i    // 外部要求 WB 级冲刷
);

//-------------------------------------------------------------
// Includes
//-------------------------------------------------------------
`include "riscv_defs.v"

wire squash_e1_e2_w;
// 分支目标未对齐检测：JALR/JAL 目标地址 [1:0] 非零则产生取指未对齐异常
wire branch_misaligned_w = (issue_branch_taken_i && issue_branch_target_i[1:0] != 2'b0);

//-------------------------------------------------------------
// E1 / Address —— 执行第一级（地址/ALU计算级）
// PCINFO：流水线控制信息位域定义
// 每个位对应指令类型或完成状态，一起存入 ctrl_e1_q 等控制寄存器
//------------------------------------------------------------- 
`define PCINFO_W     10          // 控制信息总位宽
`define PCINFO_ALU       0       // 位0：ALU 类指令（非LSU/CSR/DIV/MUL）
`define PCINFO_LOAD      1       // 位1：加载指令（LSU且有rd写回）
`define PCINFO_STORE     2       // 位2：存储指令（LSU且无rd写回）
`define PCINFO_CSR       3       // 位3：CSR 类指令
`define PCINFO_DIV       4       // 位4：除法指令
`define PCINFO_MUL       5       // 位5：乘法指令
`define PCINFO_BRANCH    6       // 位6：分支/跳转指令
`define PCINFO_RD_VALID  7       // 位7：本指令写目标寄存器（rd有效）
`define PCINFO_INTR      8       // 位8：本指令是中断响应（替换为陷阱入口）
`define PCINFO_COMPLETE  9       // 位9：本指令到达此级（调试/追踪用）

`define RD_IDX_R    11:7         // 指令字中 rd 字段的位置

// E1 级流水线寄存器
reg                     valid_e1_q;         // E1 级指令有效
reg [`PCINFO_W-1:0]     ctrl_e1_q;          // E1 级控制信息位域
reg [31:0]              pc_e1_q;            // E1 级指令 PC
reg [31:0]              npc_e1_q;           // E1 级下一 PC（用于异常返回地址）
reg [31:0]              opcode_e1_q;        // E1 级指令原始编码
reg [31:0]              operand_ra_e1_q;    // E1 级 rs1 操作数
reg [31:0]              operand_rb_e1_q;    // E1 级 rs2 操作数
reg [`EXCEPTION_W-1:0]  exception_e1_q;     // E1 级异常编码

// 时序逻辑：E1 级寄存器更新
// - 复位：所有清零
// - issue_stall_i：整体暂停，E1 保持不变
// - 有效发射且无 squash：将发射级信息锁入 E1
// - 否则（squash 或无有效指令）：清零 E1（插入气泡）
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    valid_e1_q      <= 1'b0;
    ctrl_e1_q       <= `PCINFO_W'b0;
    pc_e1_q         <= 32'b0;
    npc_e1_q        <= 32'b0;
    opcode_e1_q     <= 32'b0;
    operand_ra_e1_q <= 32'b0;
    operand_rb_e1_q <= 32'b0;
    exception_e1_q  <= `EXCEPTION_W'b0;
end
// Stall - no change in E1 state
else if (issue_stall_i)
    ;
else if ((issue_valid_i && issue_accept_i) && ~(squash_e1_e2_o || squash_e1_e2_i))
begin
    valid_e1_q                  <= 1'b1;
    ctrl_e1_q[`PCINFO_ALU]      <= ~(issue_lsu_i | issue_csr_i | issue_div_i | issue_mul_i);
    ctrl_e1_q[`PCINFO_LOAD]     <= issue_lsu_i &  issue_rd_valid_i & ~take_interrupt_i; // TODO: Check
    ctrl_e1_q[`PCINFO_STORE]    <= issue_lsu_i & ~issue_rd_valid_i & ~take_interrupt_i;
    ctrl_e1_q[`PCINFO_CSR]      <= issue_csr_i & ~take_interrupt_i;
    ctrl_e1_q[`PCINFO_DIV]      <= issue_div_i & ~take_interrupt_i;
    ctrl_e1_q[`PCINFO_MUL]      <= issue_mul_i & ~take_interrupt_i;
    ctrl_e1_q[`PCINFO_BRANCH]   <= issue_branch_i & ~take_interrupt_i;
    ctrl_e1_q[`PCINFO_RD_VALID] <= issue_rd_valid_i & ~take_interrupt_i;
    ctrl_e1_q[`PCINFO_INTR]     <= take_interrupt_i;
    ctrl_e1_q[`PCINFO_COMPLETE] <= 1'b1;

    pc_e1_q         <= issue_pc_i;
    npc_e1_q        <= issue_branch_taken_i ? issue_branch_target_i : issue_pc_i + 32'd4;
    opcode_e1_q     <= issue_opcode_i;
    operand_ra_e1_q <= issue_operand_ra_i;
    operand_rb_e1_q <= issue_operand_rb_i;
    // 优先使用发射级已检测到的异常；若无，检查分支目标是否未对齐
    exception_e1_q  <= (|issue_exception_i) ? issue_exception_i : 
                       branch_misaligned_w  ? `EXCEPTION_MISALIGNED_FETCH : `EXCEPTION_W'b0;
end
// No valid instruction (or pipeline flush event)
else
begin
    valid_e1_q      <= 1'b0;
    ctrl_e1_q       <= `PCINFO_W'b0;
    pc_e1_q         <= 32'b0;
    npc_e1_q        <= 32'b0;
    opcode_e1_q     <= 32'b0;
    operand_ra_e1_q <= 32'b0;
    operand_rb_e1_q <= 32'b0;
    exception_e1_q  <= `EXCEPTION_W'b0;
end

// E1 级控制信号解码输出（组合，直接从 ctrl_e1_q 提取各位）
wire   alu_e1_w        = ctrl_e1_q[`PCINFO_ALU];   // E1 有 ALU 指令
assign load_e1_o       = ctrl_e1_q[`PCINFO_LOAD];  // E1 有加载
assign store_e1_o      = ctrl_e1_q[`PCINFO_STORE]; // E1 有存储
wire   csr_e1_w        = ctrl_e1_q[`PCINFO_CSR];   // E1 有 CSR
wire   div_e1_w        = ctrl_e1_q[`PCINFO_DIV];   // E1 有除法
assign mul_e1_o        = ctrl_e1_q[`PCINFO_MUL];   // E1 有乘法
assign branch_e1_o     = ctrl_e1_q[`PCINFO_BRANCH];// E1 有分支
// rd 索引：仅当 RD_VALID 有效时才输出真实索引，否则为 0（防止错误写寄存器堆）
assign rd_e1_o         = {5{ctrl_e1_q[`PCINFO_RD_VALID]}} & opcode_e1_q[`RD_IDX_R];
assign pc_e1_o         = pc_e1_q;
assign opcode_e1_o     = opcode_e1_q;
assign operand_ra_e1_o = operand_ra_e1_q;
assign operand_rb_e1_o = operand_rb_e1_q;

//-------------------------------------------------------------
// E2 / Mem result —— 执行第二级（内存响应/乘法结果接收级）
//
// E2 级接收 LSU/MUL/DIV 的结果，进行异常检测，并准备写回数据。
// 若 E1 存在异常，E2 直接传递异常而不传递指令有效标志。
//------------------------------------------------------------- 
// E2 级流水线寄存器
reg                     valid_e2_q;         // E2 级指令有效
reg [`PCINFO_W-1:0]     ctrl_e2_q;          // E2 级控制信息位域
reg                     csr_wr_e2_q;        // E2 级 CSR 写使能
reg [31:0]              csr_wdata_e2_q;     // E2 级 CSR 写数据
reg [31:0]              result_e2_q;        // E2 级结果（ALU/CSR/DIV）
reg [31:0]              pc_e2_q;            // E2 级指令 PC
reg [31:0]              npc_e2_q;           // E2 级下一 PC
reg [31:0]              opcode_e2_q;        // E2 级指令编码
reg [31:0]              operand_ra_e2_q;    // E2 级 rs1 操作数
reg [31:0]              operand_rb_e2_q;    // E2 级 rs2 操作数
reg [`EXCEPTION_W-1:0]  exception_e2_q;     // E2 级异常编码

// 时序逻辑：E2 级寄存器更新
// - 复位：清零
// - issue_stall_i：暂停，E2 保持不变
// - squash_e1_e2_o/i：冲刷，E2 插入气泡
// - 正常：将 E1 结果锁入 E2
//   * 结果选择优先级：DIV > CSR > ALU
//   * 异常优先级：中断 > E1异常（取指/译码）> CSR异常
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    valid_e2_q      <= 1'b0;
    ctrl_e2_q       <= `PCINFO_W'b0;
    csr_wr_e2_q     <= 1'b0;
    csr_wdata_e2_q  <= 32'b0;
    pc_e2_q         <= 32'b0;
    npc_e2_q        <= 32'b0;
    opcode_e2_q     <= 32'b0;
    operand_ra_e2_q <= 32'b0;
    operand_rb_e2_q <= 32'b0;
    result_e2_q     <= 32'b0;
    exception_e2_q  <= `EXCEPTION_W'b0;
end
// Stall - no change in E2 state
else if (issue_stall_i)
    ;
// Pipeline flush
else if (squash_e1_e2_o || squash_e1_e2_i)
begin
    valid_e2_q      <= 1'b0;
    ctrl_e2_q       <= `PCINFO_W'b0;
    csr_wr_e2_q     <= 1'b0;
    csr_wdata_e2_q  <= 32'b0;
    pc_e2_q         <= 32'b0;
    npc_e2_q        <= 32'b0;
    opcode_e2_q     <= 32'b0;
    operand_ra_e2_q <= 32'b0;
    operand_rb_e2_q <= 32'b0;
    result_e2_q     <= 32'b0;
    exception_e2_q  <= `EXCEPTION_W'b0;
end
// Normal pipeline advance
else
begin
    valid_e2_q      <= valid_e1_q;
    ctrl_e2_q       <= ctrl_e1_q;
    csr_wr_e2_q     <= csr_result_write_e1_i;
    csr_wdata_e2_q  <= csr_result_wdata_e1_i;
    pc_e2_q         <= pc_e1_q;
    npc_e2_q        <= npc_e1_q;
    opcode_e2_q     <= opcode_e1_q;
    operand_ra_e2_q <= operand_ra_e1_q;
    operand_rb_e2_q <= operand_rb_e1_q;

    // Launch interrupt
    if (ctrl_e1_q[`PCINFO_INTR])
        exception_e2_q  <= `EXCEPTION_INTERRUPT;
    // If frontend reports bad instruction, ignore later CSR errors...
    else if (|exception_e1_q)
    begin
        valid_e2_q      <= 1'b0;
        exception_e2_q  <= exception_e1_q;
    end
    else
        exception_e2_q  <= csr_result_exception_e1_i;

    // E2 结果选择：DIV 完成优先，其次 CSR 读，最后 ALU
    if (ctrl_e1_q[`PCINFO_DIV])
        result_e2_q <= div_result_i; 
    else if (ctrl_e1_q[`PCINFO_CSR])
        result_e2_q <= csr_result_value_e1_i;
    else
        result_e2_q <= alu_result_e1_i;
end

reg [31:0] result_e2_r; // E2 级经旁路选择后的最终结果（组合逻辑）

// valid_e2_w：E2 指令有效且流水线未暂停
wire valid_e2_w      = valid_e2_q & ~issue_stall_i;

// 组合逻辑：E2 级数据旁路 MUX（前递）
// SUPPORT_LOAD_BYPASS：若 E2 有加载/存储指令，用内存返回数据替换锁存的 ALU 结果
// SUPPORT_MUL_BYPASS ：若 E2 有乘法指令，用乘法器结果替换锁存的 ALU 结果
// 默认：使用 E2 锁存的结果（ALU/CSR/DIV）
always @ *
begin
    // Default: ALU result
    result_e2_r = result_e2_q;

    if (SUPPORT_LOAD_BYPASS && valid_e2_w && (ctrl_e2_q[`PCINFO_LOAD] || ctrl_e2_q[`PCINFO_STORE]))
        result_e2_r = mem_result_e2_i;
    else if (SUPPORT_MUL_BYPASS && valid_e2_w && ctrl_e2_q[`PCINFO_MUL])
        result_e2_r = mul_result_e2_i;
end

wire   load_store_e2_w = ctrl_e2_q[`PCINFO_LOAD] | ctrl_e2_q[`PCINFO_STORE]; // E2 有 LSU
assign load_e2_o       = ctrl_e2_q[`PCINFO_LOAD];   // E2 加载有效
assign mul_e2_o        = ctrl_e2_q[`PCINFO_MUL];    // E2 乘法有效
// rd_e2_o：stall 时不输出有效 rd（防止 scoreboard 误判已完成）
assign rd_e2_o         = {5{(valid_e2_w && ctrl_e2_q[`PCINFO_RD_VALID] && ~stall_o)}} & opcode_e2_q[`RD_IDX_R];
assign result_e2_o     = result_e2_r;

// RAW 冒险检测 / 流水线暂停
// stall_o 条件：
//   1. E1 有除法且除法未完成（div_complete_i=0）
//   2. E2 有加载或存储且内存响应未到（mem_complete_i=0）
// Load store result not ready when reaching E2
assign stall_o         = (ctrl_e1_q[`PCINFO_DIV] && ~div_complete_i) || ((ctrl_e2_q[`PCINFO_LOAD] | ctrl_e2_q[`PCINFO_STORE]) & ~mem_complete_i);

// 异常仲裁：E2 级来自内存的异常优先于锁存的 E2 异常
reg [`EXCEPTION_W-1:0] exception_e2_r;
always @ *
begin
    if (valid_e2_q && (ctrl_e2_q[`PCINFO_LOAD] || ctrl_e2_q[`PCINFO_STORE]) && mem_complete_i)
        exception_e2_r = mem_exception_e2_i;
    else
        exception_e2_r = exception_e2_q;
end

// squash_e1_e2_w：E2 级有任意异常时，拉高冲刷信号，E1 级气泡化
assign squash_e1_e2_w = |exception_e2_r;

// squash_e1_e2_q：将组合冲刷信号打一拍，确保下一周期 E1 也被冲刷
reg squash_e1_e2_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    squash_e1_e2_q <= 1'b0;
else if (~issue_stall_i)
    squash_e1_e2_q <= squash_e1_e2_w;

// squash_e1_e2_o：当拍或打拍冲刷信号，OR 保证至少连续两拍有效
assign squash_e1_e2_o = squash_e1_e2_w | squash_e1_e2_q;

//-------------------------------------------------------------
// Writeback / Commit —— 写回提交级
//
// WB 级是流水线最后一级，负责将结果提交到寄存器堆和 CSR。
// 若 E2 产生了内存异常，则此级的 valid 被清零（指令不写寄存器堆），
// 但仍通过 exception_wb_o 上报异常供陷阱处理器使用。
//------------------------------------------------------------- 
// WB 级流水线寄存器
reg                     valid_wb_q;         // WB 级指令有效（可写寄存器堆）
reg [`PCINFO_W-1:0]     ctrl_wb_q;          // WB 级控制信息位域
reg                     csr_wr_wb_q;        // WB 级 CSR 写使能
reg [31:0]              csr_wdata_wb_q;     // WB 级 CSR 写数据
reg [31:0]              result_wb_q;        // WB 级写回结果
reg [31:0]              pc_wb_q;            // WB 级指令 PC
reg [31:0]              npc_wb_q;           // WB 级下一 PC
reg [31:0]              opcode_wb_q;        // WB 级指令编码
reg [31:0]              operand_ra_wb_q;    // WB 级 rs1 操作数
reg [31:0]              operand_rb_wb_q;    // WB 级 rs2 操作数
reg [`EXCEPTION_W-1:0]  exception_wb_q;     // WB 级最终异常编码

// 时序逻辑：WB 级寄存器更新
// - 复位/squash_wb_i：清零（外部陷阱处理器要求冲刷）
// - issue_stall_i：暂停，WB 保持不变
// - 正常：将 E2 结果锁入 WB
//   * valid_wb_q：内存异常时清零（指令不写寄存器堆，但异常仍上报）
//   * result：优先选内存结果（加载）或乘法结果，再取 E2 结果
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    valid_wb_q      <= 1'b0;
    ctrl_wb_q       <= `PCINFO_W'b0;
    csr_wr_wb_q     <= 1'b0;
    csr_wdata_wb_q  <= 32'b0;
    pc_wb_q         <= 32'b0;
    npc_wb_q        <= 32'b0;
    opcode_wb_q     <= 32'b0;
    operand_ra_wb_q <= 32'b0;
    operand_rb_wb_q <= 32'b0;
    result_wb_q     <= 32'b0;
    exception_wb_q  <= `EXCEPTION_W'b0;
end
// Stall - no change in WB state
else if (issue_stall_i)
    ;
else if (squash_wb_i)
begin
    valid_wb_q      <= 1'b0;
    ctrl_wb_q       <= `PCINFO_W'b0;
    csr_wr_wb_q     <= 1'b0;
    csr_wdata_wb_q  <= 32'b0;
    pc_wb_q         <= 32'b0;
    npc_wb_q        <= 32'b0;
    opcode_wb_q     <= 32'b0;
    operand_ra_wb_q <= 32'b0;
    operand_rb_wb_q <= 32'b0;
    result_wb_q     <= 32'b0;
    exception_wb_q  <= `EXCEPTION_W'b0;
end
else
begin
    // 内存异常时，清零 valid（指令不提交），异常仍传递
    // Squash instruction valid on memory faults
    case (exception_e2_r)
    `EXCEPTION_MISALIGNED_LOAD,
    `EXCEPTION_FAULT_LOAD,
    `EXCEPTION_MISALIGNED_STORE,
    `EXCEPTION_FAULT_STORE,
    `EXCEPTION_PAGE_FAULT_LOAD,
    `EXCEPTION_PAGE_FAULT_STORE:
        valid_wb_q      <= 1'b0;
    default:
        valid_wb_q      <= valid_e2_q;
    endcase

    csr_wr_wb_q     <= csr_wr_e2_q;  // TODO: Fault disable???
    csr_wdata_wb_q  <= csr_wdata_e2_q;

    // 若 E2 有异常，清除 RD_VALID 位（防止写寄存器堆）
    // Exception - squash writeback
    if (|exception_e2_r)
        ctrl_wb_q       <= ctrl_e2_q & ~(1 << `PCINFO_RD_VALID);
    else
        ctrl_wb_q       <= ctrl_e2_q;

    pc_wb_q         <= pc_e2_q;
    npc_wb_q        <= npc_e2_q;
    opcode_wb_q     <= opcode_e2_q;
    operand_ra_wb_q <= operand_ra_e2_q;
    operand_rb_wb_q <= operand_rb_e2_q;
    exception_wb_q  <= exception_e2_r;

    // WB 结果选择：加载/存储结果 > 乘法结果 > E2 锁存结果
    if (valid_e2_w && (ctrl_e2_q[`PCINFO_LOAD] || ctrl_e2_q[`PCINFO_STORE]))
        result_wb_q <= mem_result_e2_i;
    else if (valid_e2_w && ctrl_e2_q[`PCINFO_MUL])
        result_wb_q <= mul_result_e2_i;
    else
        result_wb_q <= result_e2_q;
end

// Instruction completion (for debug)
// complete_wb_w：WB 级有指令完成（PCINFO_COMPLETE 置位），用于调试/追踪
wire complete_wb_w     = ctrl_wb_q[`PCINFO_COMPLETE] & ~issue_stall_i;

// WB 级输出：驱动寄存器堆写端口和 CSR 写端口
assign valid_wb_o      = valid_wb_q & ~issue_stall_i;
assign csr_wb_o        = ctrl_wb_q[`PCINFO_CSR] & ~issue_stall_i; // TODO: Fault disable???
// rd_wb_o：同时满足 valid_wb_o、RD_VALID 且无 stall 时才输出有效 rd 索引
assign rd_wb_o         = {5{(valid_wb_o && ctrl_wb_q[`PCINFO_RD_VALID] && ~stall_o)}} & opcode_wb_q[`RD_IDX_R];
assign result_wb_o     = result_wb_q;
assign pc_wb_o         = pc_wb_q;
assign opcode_wb_o     = opcode_wb_q;
assign operand_ra_wb_o = operand_ra_wb_q;
assign operand_rb_wb_o = operand_rb_wb_q;

// 异常和 CSR 写端口
assign exception_wb_o  = exception_wb_q;

assign csr_write_wb_o  = csr_wr_wb_q;
assign csr_waddr_wb_o  = opcode_wb_q[31:20]; // CSR 地址编码在指令字 [31:20]
assign csr_wdata_wb_o  = csr_wdata_wb_q;

`ifdef verilator
riscv_trace_sim
u_trace_d
(
     .valid_i(issue_valid_i)
    ,.pc_i(issue_pc_i)
    ,.opcode_i(issue_opcode_i)
);

riscv_trace_sim
u_trace_wb
(
     .valid_i(valid_wb_o)
    ,.pc_i(pc_wb_o)
    ,.opcode_i(opcode_wb_o)
);
`endif

endmodule