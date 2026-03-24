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
// 模块: riscv_fetch
// 功能: 取指单元（Instruction Fetch Unit）
//   - 维护并更新程序计数器（PC）
//   - 向指令缓存（ICache）发送取指请求
//   - 通过 Skid Buffer 吸收下游背压（back-pressure）
//   - 在分支/跳转发生时冲刷流水线，丢弃已取的旧指令
//   - 支持 MMU 页错误上报
// 流水线位置: 第一级 —— 取指（IF）
// ============================================================
module riscv_fetch
//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
#(
     parameter SUPPORT_MMU      = 1    // 是否支持 MMU 地址翻译（1=支持，0=不支持）
)
//-----------------------------------------------------------------
// Ports
//-----------------------------------------------------------------
(
    // Inputs
     input           clk_i              // 系统时钟
    ,input           rst_i              // 同步复位（高有效）
    ,input           fetch_accept_i     // 下游译码级就绪，可接收取指结果
    ,input           icache_accept_i    // ICache 已接受本次取指请求
    ,input           icache_valid_i     // ICache 返回数据有效
    ,input           icache_error_i     // ICache 访问出错（总线错误等）
    ,input  [ 31:0]  icache_inst_i      // ICache 返回的指令数据
    ,input           icache_page_fault_i// ICache 产生页错误（MMU）
    ,input           fetch_invalidate_i // 强制刷新 ICache（fence.i 等指令触发）
    ,input           branch_request_i   // 分支/跳转请求（来自执行级）
    ,input  [ 31:0]  branch_pc_i        // 分支目标 PC
    ,input  [  1:0]  branch_priv_i      // 分支后的特权级别

    // Outputs
    ,output          fetch_valid_o      // 取指结果有效，可被下游消费
    ,output [ 31:0]  fetch_instr_o      // 取回的指令字
    ,output [ 31:0]  fetch_pc_o         // 取回指令对应的 PC
    ,output          fetch_fault_fetch_o// 取指总线错误标志
    ,output          fetch_fault_page_o // 取指页错误标志（MMU）
    ,output          icache_rd_o        // 向 ICache 发出读请求
    ,output          icache_flush_o     // 刷新 ICache（fence.i）
    ,output          icache_invalidate_o// 使 ICache 无效（当前恒为 0）
    ,output [ 31:0]  icache_pc_o        // 向 ICache 送出的取指地址（4 字节对齐）
    ,output [  1:0]  icache_priv_o      // 向 ICache 送出的特权级别（用于 MMU）
    ,output          squash_decode_o    // 冲刷信号：通知译码级丢弃当前指令
);



//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

//-------------------------------------------------------------
// Registers / Wires
//-------------------------------------------------------------
reg         active_q;       // 取指激活标志：收到第一次有效分支后置 1，使能后续取指

wire        icache_busy_w;
// stall_w：任意一个条件导致流水线停顿：下游不接受 / ICache 繁忙 / ICache 未接受请求
wire        stall_w       = !fetch_accept_i || icache_busy_w || !icache_accept_i;

//-------------------------------------------------------------
// Buffered branch（分支请求缓存）
// 当分支请求到来但当前无法立即发出取指（stall 状态）时，
// 将目标 PC 和特权级锁存，待流水线就绪后再使用
//-------------------------------------------------------------
reg         branch_q;       // 已缓存的分支请求标志
reg [31:0]  branch_pc_q;    // 缓存的分支目标 PC
reg [1:0]   branch_priv_q;  // 缓存的分支目标特权级

// 分支缓存寄存器更新逻辑
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    branch_q       <= 1'b0;
    branch_pc_q    <= 32'b0;
    branch_priv_q  <= `PRIV_MACHINE;
end
else if (branch_request_i)
begin
    // 收到新的分支请求，锁存目标地址与特权级
    branch_q       <= 1'b1;
    branch_pc_q    <= branch_pc_i;
    branch_priv_q  <= branch_priv_i;
end
else if (icache_rd_o && icache_accept_i)
begin
    // ICache 已接受本次取指请求，缓存的分支已被消费，清除标志
    branch_q       <= 1'b0;
    branch_pc_q    <= 32'b0;
end

// 将缓存的分支信息以 wire 形式输出，供 PC 更新逻辑使用
wire        branch_w      = branch_q;
wire [31:0] branch_pc_w   = branch_pc_q;
wire [1:0]  branch_priv_w = branch_priv_q;

// 只要收到分支请求（即使尚未消费），立即通知译码级冲刷
assign squash_decode_o    = branch_request_i;

//-------------------------------------------------------------
// Active flag（取指激活标志）
// 上电复位后 active_q=0，阻止 ICache 无效请求；
// 当第一次分支请求成功发出（branch_w && !stall_w）时置 1，
// 此后持续保持有效，允许顺序取指。
//-------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    active_q    <= 1'b0;
else if (branch_w && ~stall_w)
    active_q    <= 1'b1;

//-------------------------------------------------------------
// Stall flag（停顿状态寄存器）
// 将组合信号 stall_w 寄存为时序信号 stall_q，
// 供下游逻辑在下一拍判断是否处于停顿状态
//-------------------------------------------------------------
reg stall_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    stall_q    <= 1'b0;
else
    stall_q    <= stall_w;

//-------------------------------------------------------------
// Request tracking（ICache 请求状态跟踪）
// icache_fetch_q   : 记录是否已向 ICache 发出请求但尚未返回结果
// icache_invalidate_q: 记录 invalidate 请求未被接受、需重试
//-------------------------------------------------------------
reg icache_fetch_q;       // ICache 请求在途标志（请求已发出但 valid 未返回）
reg icache_invalidate_q;  // invalidate 重试标志

// ICACHE fetch tracking
// 请求发出时置 1，返回结果（icache_valid_i）后清 0
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    icache_fetch_q <= 1'b0;
else if (icache_rd_o && icache_accept_i)
    icache_fetch_q <= 1'b1;
else if (icache_valid_i)
    icache_fetch_q <= 1'b0;

// invalidate 未被接受时锁存，待下一周期重试；接受后清除
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    icache_invalidate_q <= 1'b0;
else if (icache_invalidate_o && !icache_accept_i)
    icache_invalidate_q <= 1'b1;
else
    icache_invalidate_q <= 1'b0;

//-------------------------------------------------------------
// PC（程序计数器）
// pc_f_q : 待发往 ICache 的取指 PC（Fetch 阶段当前 PC）
// pc_d_q : 已被 ICache 接受的最近一次取指 PC（用于构造输出 fetch_pc_o）
//-------------------------------------------------------------
reg [31:0]  pc_f_q;   // 取指 PC 寄存器
reg [31:0]  pc_d_q;   // 最近一次已发出请求的 PC（用于响应阶段）

wire [31:0] icache_pc_w;       // 当前送往 ICache 的 PC（直接来自 pc_f_q）
wire [1:0]  icache_priv_w;     // 当前送往 ICache 的特权级
wire        fetch_resp_drop_w; // 当有未消费分支时，需丢弃当前 ICache 响应

// PC 更新逻辑：
//   优先级1：复位 -> PC=0
//   优先级2：分支请求 && 未停顿 -> 跳转到分支目标
//   优先级3：正常取指 && 未停顿 -> PC+4（顺序执行）
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    pc_f_q  <= 32'b0;
// Branch request
else if (branch_w && ~stall_w)
    pc_f_q  <= branch_pc_w;
// NPC
else if (!stall_w)
    pc_f_q  <= {icache_pc_w[31:2],2'b0} + 32'd4;

reg [1:0] priv_f_q;   // 当前特权级寄存器（跟随分支更新）
reg       branch_d_q; // 分支请求的延迟标志（表示上一拍已发出分支取指请求）

// 特权级跟随分支更新，其余时刻保持不变
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    priv_f_q  <= `PRIV_MACHINE;
// Branch request
else if (branch_w && ~stall_w)
    priv_f_q  <= branch_priv_w;

// branch_d_q：在分支取指发出后的下一拍标记为 1，
// 表示此时 ICache 仍可能返回旧地址的响应，需要丢弃
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    branch_d_q  <= 1'b0;
// Branch request
else if (branch_w && ~stall_w)
    branch_d_q  <= 1'b1;
// NPC
else if (!stall_w)
    branch_d_q  <= 1'b0;

// 将 pc_f_q / priv_f_q 直接映射到 ICache 接口
assign icache_pc_w       = pc_f_q;
assign icache_priv_w     = priv_f_q;
// 只要存在待处理的分支（branch_w）或已发出分支请求（branch_d_q），
// 就丢弃当前 ICache 返回的响应，避免旧指令污染流水线
assign fetch_resp_drop_w = branch_w | branch_d_q;

// Last fetch address（记录最近一次 ICache 接受的取指地址，作为响应 PC 基准）
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    pc_d_q <= 32'b0;
else if (icache_rd_o && icache_accept_i)
    pc_d_q <= icache_pc_w;

//-------------------------------------------------------------
// Outputs（输出信号生成）
//-------------------------------------------------------------
// 仅当模块激活、下游就绪且 ICache 不忙时才发出读请求
assign icache_rd_o         = active_q & fetch_accept_i & !icache_busy_w;
// 取指地址低 2 位强制为 0（4 字节对齐）
assign icache_pc_o         = {icache_pc_w[31:2],2'b0};
assign icache_priv_o       = icache_priv_w;
// fence.i 触发的失效请求，或之前未被接受的 invalidate 重试
assign icache_flush_o      = fetch_invalidate_i | icache_invalidate_q;
// 当前不产生 invalidate 请求（预留信号，恒为 0）
assign icache_invalidate_o = 1'b0;

// ICache 繁忙：已发出请求但尚未收到有效响应
assign icache_busy_w       =  icache_fetch_q && !icache_valid_i;

//-------------------------------------------------------------
// Response Buffer（Skid Buffer：响应缓冲 / 反压缓冲）
// 工作原理：
//   当取指结果有效（fetch_valid_o=1）但下游不接受（fetch_accept_i=0）时，
//   将当前输出锁入 skid_buffer_q，并置 skid_valid_q=1；
//   下一拍即使 ICache 没有新的返回，也可从 skid buffer 重新输出，
//   从而保证数据不丢失。
// skid_buffer_q 位域：[65]=page_fault, [64]=fetch_fault, [63:32]=PC, [31:0]=指令
//-------------------------------------------------------------
reg [65:0]  skid_buffer_q;  // Skid Buffer 数据寄存器（66 位：page_fault+fault+PC+指令）
reg         skid_valid_q;   // Skid Buffer 有效标志

// Skid Buffer 更新逻辑
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    skid_buffer_q  <= 66'b0;
    skid_valid_q   <= 1'b0;
end 
// Instruction output back-pressured - hold in skid buffer
else if (fetch_valid_o && !fetch_accept_i)
begin
    // 下游反压：将当前输出保存至 Skid Buffer
    skid_valid_q  <= 1'b1;
    skid_buffer_q <= {fetch_fault_page_o, fetch_fault_fetch_o, fetch_pc_o, fetch_instr_o};
end
else
begin
    // 正常或已被消费：清空 Skid Buffer
    skid_valid_q  <= 1'b0;
    skid_buffer_q <= 66'b0;
end

// 取指结果有效：ICache 返回数据或 Skid Buffer 有数据，且不需要丢弃响应
assign fetch_valid_o       = (icache_valid_i || skid_valid_q) & !fetch_resp_drop_w;
// 优先输出 Skid Buffer 中缓存的数据，保证背压时数据不丢失
assign fetch_pc_o          = skid_valid_q ? skid_buffer_q[63:32] : {pc_d_q[31:2],2'b0};
assign fetch_instr_o       = skid_valid_q ? skid_buffer_q[31:0]  : icache_inst_i;

// Faults（错误信号：同样优先从 Skid Buffer 输出）
assign fetch_fault_fetch_o = skid_valid_q ? skid_buffer_q[64] : icache_error_i;
assign fetch_fault_page_o  = skid_valid_q ? skid_buffer_q[65] : icache_page_fault_i;



endmodule
