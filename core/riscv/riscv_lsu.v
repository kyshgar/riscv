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
 * 模块功能概述：riscv_lsu —— RISC-V 加载/存储单元（LSU）
 *
 * 本模块负责 RISC-V 流水线中所有内存访问操作，主要功能包括：
 *   1. 地址计算：rs1 + 符号扩展立即数，区分加载（I 型）与存储（S 型）格式；
 *   2. 对齐检测：检查字（LW/SW）按 4 字节、半字（LH/SH）按 2 字节对齐，
 *                未对齐访问产生对齐异常（misaligned exception）；
 *   3. 字节使能生成：按地址低两位将写数据放到正确字节通道（SB/SH/SW）；
 *   4. 未完成请求追踪：防止对同一地址发出多次重叠请求（pending 机制）；
 *   5. 加载数据对齐与符号扩展：从32位总线数据中按字节/半字提取，并可选符号扩展；
 *   6. DCache 管理：通过 CSR 写操作触发 flush/writeback/invalidate 命令；
 *   7. 异常上报：对齐错误、总线错误、页面错误通过 writeback_exception_o 上报。
 */
module riscv_lsu
//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
#(
     parameter MEM_CACHE_ADDR_MIN = 32'h80000000 // 可缓存地址区间下限
    ,parameter MEM_CACHE_ADDR_MAX = 32'h8fffffff // 可缓存地址区间上限
)
//-----------------------------------------------------------------
// Ports
//-----------------------------------------------------------------
(
    // Inputs
     input           clk_i              // 时钟信号
    ,input           rst_i              // 异步复位（高有效）
    ,input           opcode_valid_i     // 当前指令有效
    ,input  [ 31:0]  opcode_opcode_i    // 32 位原始指令字
    ,input  [ 31:0]  opcode_pc_i        // 当前指令 PC
    ,input           opcode_invalid_i   // 指令非法标志
    ,input  [  4:0]  opcode_rd_idx_i    // 目的寄存器索引 rd
    ,input  [  4:0]  opcode_ra_idx_i    // 源寄存器索引 rs1
    ,input  [  4:0]  opcode_rb_idx_i    // 源寄存器索引 rs2（存储数据来源）
    ,input  [ 31:0]  opcode_ra_operand_i // rs1 操作数（基地址）
    ,input  [ 31:0]  opcode_rb_operand_i // rs2 操作数（存储数据）
    ,input  [ 31:0]  mem_data_rd_i      // 从内存/DCache 返回的读数据
    ,input           mem_accept_i       // 内存接口接受本次请求（握手信号）
    ,input           mem_ack_i          // 内存响应有效（数据/完成应答）
    ,input           mem_error_i        // 内存访问错误（总线错误）
    ,input  [ 10:0]  mem_resp_tag_i     // 内存响应标签（用于乱序响应匹配）
    ,input           mem_load_fault_i   // 加载页面异常标志
    ,input           mem_store_fault_i  // 存储页面异常标志

    // Outputs
    ,output [ 31:0]  mem_addr_o         // 发送给内存的字对齐地址（低2位置0）
    ,output [ 31:0]  mem_data_wr_o      // 发送给内存的写数据（字节已对齐）
    ,output          mem_rd_o           // 读请求使能
    ,output [  3:0]  mem_wr_o           // 写字节使能（4 位，对应4个字节通道）
    ,output          mem_cacheable_o    // 本次访问是否命中可缓存地址区间
    ,output [ 10:0]  mem_req_tag_o      // 请求标签（本实现固定为 0）
    ,output          mem_invalidate_o   // DCache 无效化命令
    ,output          mem_writeback_o    // DCache 写回命令
    ,output          mem_flush_o        // DCache 刷新（写回+无效化）命令
    ,output          writeback_valid_o  // 写回结果有效（ack 或对齐异常虚假 ack）
    ,output [ 31:0]  writeback_value_o  // 写回寄存器的加载结果或异常地址
    ,output [  5:0]  writeback_exception_o // 异常编码（见 riscv_defs.v）
    ,output          stall_o            // 流水线暂停请求（等待内存响应或对齐检测）
);



//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

//-----------------------------------------------------------------
// Registers / Wires —— 流水线寄存器与内部信号
//-----------------------------------------------------------------
reg [ 31:0]  mem_addr_q;       // E1 级锁存的访存地址
reg [ 31:0]  mem_data_wr_q;    // E1 级锁存的写数据（字节已对齐）
reg          mem_rd_q;         // E1 级锁存的读请求使能
reg [  3:0]  mem_wr_q;         // E1 级锁存的写字节使能
reg          mem_cacheable_q;  // E1 级锁存的可缓存标志
reg          mem_invalidate_q; // E1 级锁存的 DCache 无效化命令
reg          mem_writeback_q;  // E1 级锁存的 DCache 写回命令
reg          mem_flush_q;      // E1 级锁存的 DCache 刷新命令
reg          mem_unaligned_e1_q; // E1 级检测到的未对齐访问标志
reg          mem_unaligned_e2_q; // E2 级的未对齐访问标志（产生虚假 ack）

// 加载请求附属信息（传递给响应处理逻辑）
reg          mem_load_q;  // 当前请求是否为加载指令
reg          mem_xb_q;    // 当前请求是字节（byte）粒度
reg          mem_xh_q;    // 当前请求是半字（halfword）粒度
reg          mem_ls_q;    // 当前加载是有符号扩展（load signed）

//-----------------------------------------------------------------
// Outstanding Access Tracking —— 未完成请求追踪
// 记录是否有尚未收到响应的内存请求，防止流水线提前推进
//-----------------------------------------------------------------
// pending_lsu_e2_q：置 1 表示有一个请求已发出但尚未收到 ack
reg pending_lsu_e2_q;

// issue_lsu_e1_w：本周期成功发出了一次内存请求（读/写/flush/wb/inv 且被接受）
wire issue_lsu_e1_w    = (mem_rd_o || (|mem_wr_o) || mem_writeback_o || mem_invalidate_o || mem_flush_o) && mem_accept_i;
// complete_ok_e2_w：本周期收到正常 ack（无错误）
wire complete_ok_e2_w  = mem_ack_i & ~mem_error_i;
// complete_err_e2_w：本周期收到错误 ack（总线错误）
wire complete_err_e2_w = mem_ack_i & mem_error_i;

// 时序逻辑：维护 pending 状态位
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    pending_lsu_e2_q <= 1'b0;
else if (issue_lsu_e1_w)
    pending_lsu_e2_q <= 1'b1;
else if (complete_ok_e2_w || complete_err_e2_w)
    pending_lsu_e2_q <= 1'b0;

// Delay next instruction if outstanding response is late
// delay_lsu_e2_w：有未完成请求且本周期还未收到 ack，需暂停发新请求
wire delay_lsu_e2_w = pending_lsu_e2_q && !complete_ok_e2_w;

//-----------------------------------------------------------------
// Dummy Ack (unaligned access /E2) —— 未对齐访问虚假应答
// 未对齐访问不会发出真实内存请求，因此在 E2 级产生一个虚假 ack
// 以触发异常处理流程（写回异常地址，stall 解除）
//-----------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    mem_unaligned_e2_q <= 1'b0;
else
    mem_unaligned_e2_q <= mem_unaligned_e1_q & ~delay_lsu_e2_w;

//-----------------------------------------------------------------
// Opcode decode —— 指令类型解码
// 通过掩码匹配判断当前指令是哪种加载/存储操作
//-----------------------------------------------------------------

// 任意加载指令（LB/LH/LW/LBU/LHU/LWU）
wire load_inst_w = (((opcode_opcode_i & `INST_LB_MASK) == `INST_LB)  || 
                    ((opcode_opcode_i & `INST_LH_MASK) == `INST_LH)  || 
                    ((opcode_opcode_i & `INST_LW_MASK) == `INST_LW)  || 
                    ((opcode_opcode_i & `INST_LBU_MASK) == `INST_LBU) || 
                    ((opcode_opcode_i & `INST_LHU_MASK) == `INST_LHU) || 
                    ((opcode_opcode_i & `INST_LWU_MASK) == `INST_LWU));

// 有符号加载指令（LB/LH/LW，结果需符号扩展）
wire load_signed_inst_w = (((opcode_opcode_i & `INST_LB_MASK) == `INST_LB)  || 
                           ((opcode_opcode_i & `INST_LH_MASK) == `INST_LH)  || 
                           ((opcode_opcode_i & `INST_LW_MASK) == `INST_LW));

// 任意存储指令（SB/SH/SW）
wire store_inst_w = (((opcode_opcode_i & `INST_SB_MASK) == `INST_SB)  || 
                     ((opcode_opcode_i & `INST_SH_MASK) == `INST_SH)  || 
                     ((opcode_opcode_i & `INST_SW_MASK) == `INST_SW));

// 字节粒度请求：LB/LBU 或 SB
wire req_lb_w = ((opcode_opcode_i & `INST_LB_MASK) == `INST_LB) || ((opcode_opcode_i & `INST_LBU_MASK) == `INST_LBU);
// 半字粒度请求：LH/LHU 或 SH
wire req_lh_w = ((opcode_opcode_i & `INST_LH_MASK) == `INST_LH) || ((opcode_opcode_i & `INST_LHU_MASK) == `INST_LHU);
// 字粒度请求：LW/LWU 或 SW
wire req_lw_w = ((opcode_opcode_i & `INST_LW_MASK) == `INST_LW) || ((opcode_opcode_i & `INST_LWU_MASK) == `INST_LWU);
wire req_sb_w = ((opcode_opcode_i & `INST_LB_MASK) == `INST_SB);
wire req_sh_w = ((opcode_opcode_i & `INST_LH_MASK) == `INST_SH);
wire req_sw_w = ((opcode_opcode_i & `INST_LW_MASK) == `INST_SW);

// 字访问（SW/LW/LWU）：需检测4字节对齐
wire req_sw_lw_w = ((opcode_opcode_i & `INST_SW_MASK) == `INST_SW) || ((opcode_opcode_i & `INST_LW_MASK) == `INST_LW) || ((opcode_opcode_i & `INST_LWU_MASK) == `INST_LWU);
// 半字访问（SH/LH/LHU）：需检测2字节对齐
wire req_sh_lh_w = ((opcode_opcode_i & `INST_SH_MASK) == `INST_SH) || ((opcode_opcode_i & `INST_LH_MASK) == `INST_LH) || ((opcode_opcode_i & `INST_LHU_MASK) == `INST_LHU);

// 计算所用的组合逻辑中间变量
reg [31:0]  mem_addr_r;      // 计算得到的访存地址
reg         mem_unaligned_r; // 当前请求是否未对齐
reg [31:0]  mem_data_r;      // 按字节通道排好的写数据
reg         mem_rd_r;        // 读请求使能（组合）
reg [3:0]   mem_wr_r;        // 写字节使能（组合）

// 组合逻辑：地址计算 + 对齐检测 + 字节使能生成
// 地址计算：
//   - CSRRW（DCache管理）: 地址直接来自 rs1
//   - 加载指令：rs1 + sign_extend(opcode[31:20])  （I 型立即数格式）
//   - 存储指令：rs1 + sign_extend({opcode[31:25], opcode[11:7]}) （S 型立即数格式）
// 对齐检测：
//   - 字（SW/LW）：addr[1:0] != 0 则未对齐
//   - 半字（SH/LH）：addr[0] != 0 则未对齐
// 字节使能生成（按 addr[1:0] 路由写数据到正确字节通道）：
//   - SW：4'hF，数据填满32位
//   - SH：addr[1]=1 → 高16位，否则 → 低16位
//   - SB：按 addr[1:0] 将字节放到对应位置（byte0~byte3）

always @ *
begin
    mem_addr_r      = 32'b0;
    mem_data_r      = 32'b0;
    mem_unaligned_r = 1'b0;
    mem_wr_r        = 4'b0;
    mem_rd_r        = 1'b0;

    if (opcode_valid_i && ((opcode_opcode_i & `INST_CSRRW_MASK) == `INST_CSRRW))
        mem_addr_r = opcode_ra_operand_i;
    else if (opcode_valid_i && load_inst_w)
        mem_addr_r = opcode_ra_operand_i + {{20{opcode_opcode_i[31]}}, opcode_opcode_i[31:20]};
    else
        mem_addr_r = opcode_ra_operand_i + {{20{opcode_opcode_i[31]}}, opcode_opcode_i[31:25], opcode_opcode_i[11:7]};

    if (opcode_valid_i && req_sw_lw_w)
        mem_unaligned_r = (mem_addr_r[1:0] != 2'b0);
    else if (opcode_valid_i && req_sh_lh_w)
        mem_unaligned_r = mem_addr_r[0];

    mem_rd_r = (opcode_valid_i && load_inst_w && !mem_unaligned_r);

    if (opcode_valid_i && ((opcode_opcode_i & `INST_SW_MASK) == `INST_SW) && !mem_unaligned_r)
    begin
        mem_data_r  = opcode_rb_operand_i;
        mem_wr_r    = 4'hF;
    end
    else if (opcode_valid_i && ((opcode_opcode_i & `INST_SH_MASK) == `INST_SH) && !mem_unaligned_r)
    begin
        case (mem_addr_r[1:0])
        2'h2 :
        begin
            mem_data_r  = {opcode_rb_operand_i[15:0],16'h0000};
            mem_wr_r    = 4'b1100;
        end
        default :
        begin
            mem_data_r  = {16'h0000,opcode_rb_operand_i[15:0]};
            mem_wr_r    = 4'b0011;
        end
        endcase
    end
    else if (opcode_valid_i && ((opcode_opcode_i & `INST_SB_MASK) == `INST_SB))
    begin
        case (mem_addr_r[1:0])
        2'h3 :
        begin
            mem_data_r  = {opcode_rb_operand_i[7:0],24'h000000};
            mem_wr_r    = 4'b1000;
        end
        2'h2 :
        begin
            mem_data_r  = {{8'h00,opcode_rb_operand_i[7:0]},16'h0000};
            mem_wr_r    = 4'b0100;
        end
        2'h1 :
        begin
            mem_data_r  = {{16'h0000,opcode_rb_operand_i[7:0]},8'h00};
            mem_wr_r    = 4'b0010;
        end
        2'h0 :
        begin
            mem_data_r  = {24'h000000,opcode_rb_operand_i[7:0]};
            mem_wr_r    = 4'b0001;
        end
        default :
        ;
        endcase
    end
    else
        mem_wr_r    = 4'b0;
end

// DCache 管理命令解码：通过 CSRRW 指令写特定 CSR 地址触发
// CSR_DFLUSH      : 将 DCache 所有脏行写回内存并无效化（flush = writeback + invalidate）
wire dcache_flush_w      = ((opcode_opcode_i & `INST_CSRRW_MASK) == `INST_CSRRW) && (opcode_opcode_i[31:20] == `CSR_DFLUSH);
// CSR_DWRITEBACK  : 将 DCache 所有脏行写回内存（保持有效状态）
wire dcache_writeback_w  = ((opcode_opcode_i & `INST_CSRRW_MASK) == `INST_CSRRW) && (opcode_opcode_i[31:20] == `CSR_DWRITEBACK);
// CSR_DINVALIDATE : 无效化 DCache 所有行（不写回，直接丢弃）
wire dcache_invalidate_w = ((opcode_opcode_i & `INST_CSRRW_MASK) == `INST_CSRRW) && (opcode_opcode_i[31:20] == `CSR_DINVALIDATE);

//-----------------------------------------------------------------
// Sequential —— E1 级流水线寄存器时序逻辑
// 在时钟上升沿将组合逻辑计算结果锁存到 E1 级寄存器
// 优先级（由高到低）：
//   1. 复位：所有信号清零
//   2. 内存错误或对齐异常（complete_err/unaligned_e2）：清零（squash 后续请求）
//   3. E1 级请求已发出但 E2 响应还未到（delay_lsu_e2_w）：保持不变（等待响应）
//   4. 上游接口暂停（mem_accept_i=0）：保持不变（重试）
//   5. 正常流：锁存本周期计算结果，准备在下一拍驱动内存接口
//-----------------------------------------------------------------

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    mem_addr_q         <= 32'b0;
    mem_data_wr_q      <= 32'b0;
    mem_rd_q           <= 1'b0;
    mem_wr_q           <= 4'b0;
    mem_cacheable_q    <= 1'b0;
    mem_invalidate_q   <= 1'b0;
    mem_writeback_q    <= 1'b0;
    mem_flush_q        <= 1'b0;
    mem_unaligned_e1_q <= 1'b0;
    mem_load_q         <= 1'b0;
    mem_xb_q           <= 1'b0;
    mem_xh_q           <= 1'b0;
    mem_ls_q           <= 1'b0;
end
// Memory access fault - squash next operation (exception coming...)
else if (complete_err_e2_w || mem_unaligned_e2_q)
begin
    mem_addr_q         <= 32'b0;
    mem_data_wr_q      <= 32'b0;
    mem_rd_q           <= 1'b0;
    mem_wr_q           <= 4'b0;
    mem_cacheable_q    <= 1'b0;
    mem_invalidate_q   <= 1'b0;
    mem_writeback_q    <= 1'b0;
    mem_flush_q        <= 1'b0;
    mem_unaligned_e1_q <= 1'b0;
    mem_load_q         <= 1'b0;
    mem_xb_q           <= 1'b0;
    mem_xh_q           <= 1'b0;
    mem_ls_q           <= 1'b0;
end
else if ((mem_rd_q || (|mem_wr_q) || mem_unaligned_e1_q) && delay_lsu_e2_w)
    ;
else if (!((mem_writeback_o || mem_invalidate_o || mem_flush_o || mem_rd_o || mem_wr_o != 4'b0) && !mem_accept_i))
begin
    mem_addr_q         <= 32'b0;
    mem_data_wr_q      <= mem_data_r;
    mem_rd_q           <= mem_rd_r;
    mem_wr_q           <= mem_wr_r;
    mem_cacheable_q    <= 1'b0;
    mem_invalidate_q   <= 1'b0;
    mem_writeback_q    <= 1'b0;
    mem_flush_q        <= 1'b0;
    mem_unaligned_e1_q <= mem_unaligned_r;
    mem_load_q         <= opcode_valid_i && load_inst_w;
    mem_xb_q           <= req_lb_w | req_sb_w;
    mem_xh_q           <= req_lh_w | req_sh_w;
    mem_ls_q           <= load_signed_inst_w;

/* verilator lint_off UNSIGNED */
/* verilator lint_off CMPCONST */
    mem_cacheable_q  <= (mem_addr_r >= MEM_CACHE_ADDR_MIN && mem_addr_r <= MEM_CACHE_ADDR_MAX) ||
                        (opcode_valid_i && (dcache_invalidate_w || dcache_writeback_w || dcache_flush_w));
/* verilator lint_on CMPCONST */
/* verilator lint_on UNSIGNED */

    mem_invalidate_q <= opcode_valid_i & dcache_invalidate_w;
    mem_writeback_q  <= opcode_valid_i & dcache_writeback_w;
    mem_flush_q      <= opcode_valid_i & dcache_flush_w;
    mem_addr_q       <= mem_addr_r;
end

// 内存接口输出：将锁存的 E1 请求驱动到内存总线
// mem_addr_o 低两位强制清零（字对齐），字节路由由 mem_wr_o 完成
assign mem_addr_o       = {mem_addr_q[31:2], 2'b0};
assign mem_data_wr_o    = mem_data_wr_q;
// delay_lsu_e2_w 有效时暂停新请求（等待上一个响应）
assign mem_rd_o         = mem_rd_q & ~delay_lsu_e2_w;
assign mem_wr_o         = mem_wr_q & ~{4{delay_lsu_e2_w}};
assign mem_cacheable_o  = mem_cacheable_q;
assign mem_req_tag_o    = 11'b0;   // 本实现无乱序访问，tag 固定为 0
assign mem_invalidate_o = mem_invalidate_q;
assign mem_writeback_o  = mem_writeback_q;
assign mem_flush_o      = mem_flush_q;

// Stall upstream if cache is busy
// 暂停上游流水线的条件：
//   - 内存接口有请求但未被接受（mem_accept_i=0）
//   - 等待上一笔未完成的响应（delay_lsu_e2_w）
//   - E1 检测到未对齐访问（需等虚假 ack）
assign stall_o          = ((mem_writeback_o || mem_invalidate_o || mem_flush_o || mem_rd_o || mem_wr_o != 4'b0) && !mem_accept_i) || delay_lsu_e2_w || mem_unaligned_e1_q;

// FIFO 响应信息追踪信号：从 FIFO 弹出的请求属性
wire        resp_load_w;   // 响应对应的是加载请求
wire [31:0] resp_addr_w;   // 响应对应的访存地址（用于异常报告及字节路由）
wire        resp_byte_w;   // 响应是字节粒度
wire        resp_half_w;   // 响应是半字粒度
wire        resp_signed_w; // 响应需符号扩展

// riscv_lsu_fifo：请求信息跟踪 FIFO
// 在发出内存请求时将该请求的元信息（地址、粒度、加载标志等）压入 FIFO
// 在收到 ack 或虚假 ack 时弹出，确保响应处理与请求对应
riscv_lsu_fifo
#(
     .WIDTH(36)
    ,.DEPTH(2)
    ,.ADDR_W(1)
)
u_lsu_request
(
     .clk_i(clk_i)
    ,.rst_i(rst_i)

    ,.push_i(((mem_rd_o || (|mem_wr_o) || mem_writeback_o || mem_invalidate_o || mem_flush_o) && mem_accept_i) || (mem_unaligned_e1_q && ~delay_lsu_e2_w))
    ,.data_in_i({mem_addr_q, mem_ls_q, mem_xh_q, mem_xb_q, mem_load_q})
    ,.accept_o()

    ,.valid_o()
    ,.data_out_o({resp_addr_w, resp_signed_w, resp_half_w, resp_byte_w, resp_load_w})
    ,.pop_i(mem_ack_i || mem_unaligned_e2_q)
);

//-----------------------------------------------------------------
// Load response —— 加载数据对齐与符号扩展
// 根据请求时记录的粒度（字节/半字/字）和地址低位，从32位总线数据
// 中提取正确字节通道，并根据 load_signed 标志决定是否做符号扩展
//-----------------------------------------------------------------
reg [1:0]  addr_lsb_r;    // 请求地址的低2位（用于字节通道选择）
reg        load_byte_r;   // 本次响应是字节粒度
reg        load_half_r;   // 本次响应是半字粒度
reg        load_signed_r; // 本次响应需符号扩展
reg [31:0] wb_result_r;   // 最终写回结果（对齐+扩展后）

// 组合逻辑：加载数据提取与扩展
// 错误情况：将错误地址（resp_addr_w）作为写回值传递给异常处理
// 字节加载：从4个字节通道按 addr[1:0] 选取，有符号时扩展 bit7
// 半字加载：按 addr[1] 选取高/低16位，有符号时扩展 bit15
// 字加载：直接透传32位数据
always @ *
begin
    wb_result_r   = 32'b0;

    // Tag associated with load
    addr_lsb_r    = resp_addr_w[1:0];
    load_byte_r   = resp_byte_w;
    load_half_r   = resp_half_w;
    load_signed_r = resp_signed_w;

    // Access fault - pass badaddr on writeback result bus
    if ((mem_ack_i && mem_error_i) || mem_unaligned_e2_q)
        wb_result_r = resp_addr_w;
    // Handle responses
    else if (mem_ack_i && resp_load_w)
    begin
        if (load_byte_r)
        begin
            case (addr_lsb_r[1:0])
            2'h3: wb_result_r = {24'b0, mem_data_rd_i[31:24]};
            2'h2: wb_result_r = {24'b0, mem_data_rd_i[23:16]};
            2'h1: wb_result_r = {24'b0, mem_data_rd_i[15:8]};
            2'h0: wb_result_r = {24'b0, mem_data_rd_i[7:0]};
            endcase

            if (load_signed_r && wb_result_r[7])
                wb_result_r = {24'hFFFFFF, wb_result_r[7:0]};
        end
        else if (load_half_r)
        begin
            if (addr_lsb_r[1])
                wb_result_r = {16'b0, mem_data_rd_i[31:16]};
            else
                wb_result_r = {16'b0, mem_data_rd_i[15:0]};

            if (load_signed_r && wb_result_r[15])
                wb_result_r = {16'hFFFF, wb_result_r[15:0]};
        end
        else
            wb_result_r = mem_data_rd_i;
    end
end

// writeback_valid_o：内存 ack 或对齐异常虚假 ack 时写回有效
assign writeback_valid_o    = mem_ack_i | mem_unaligned_e2_q;
// writeback_value_o：正常时为加载结果，异常时为错误地址
assign writeback_value_o    = wb_result_r;

// 各类访存异常标志解码
wire fault_load_align_w     = mem_unaligned_e2_q & resp_load_w;   // 加载未对齐
wire fault_store_align_w    = mem_unaligned_e2_q & ~resp_load_w;  // 存储未对齐
wire fault_load_bus_w       = mem_error_i &&  resp_load_w;         // 加载总线错误
wire fault_store_bus_w      = mem_error_i && ~resp_load_w;         // 存储总线错误
wire fault_load_page_w      = mem_error_i && mem_load_fault_i;     // 加载页面错误
wire fault_store_page_w     = mem_error_i && mem_store_fault_i;    // 存储页面错误

// 异常编码优先级编码器：按优先级选择最高优先级的异常类型上报
assign writeback_exception_o         = fault_load_align_w  ? `EXCEPTION_MISALIGNED_LOAD:
                                       fault_store_align_w ? `EXCEPTION_MISALIGNED_STORE:
                                       fault_load_page_w   ? `EXCEPTION_PAGE_FAULT_LOAD:
                                       fault_store_page_w  ? `EXCEPTION_PAGE_FAULT_STORE:
                                       fault_load_bus_w    ? `EXCEPTION_FAULT_LOAD:
                                       fault_store_bus_w   ? `EXCEPTION_FAULT_STORE:
                                       `EXCEPTION_W'b0;

endmodule 

/*
 * 模块功能概述：riscv_lsu_fifo —— LSU 请求信息追踪 FIFO
 *
 * 这是一个参数化的同步 FIFO，用于在 LSU 内部追踪已发出但尚未收到响应的
 * 内存请求的元信息（地址、粒度、加载/存储标志等）。
 * 当请求被发出（push_i）时，将元信息入队；
 * 当内存返回 ack 或产生对齐异常虚假 ack（pop_i）时，出队。
 * 由此保证响应处理时能正确对应原始请求的粒度和地址。
 */
module riscv_lsu_fifo
//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
#(
    parameter WIDTH   = 8,  // 数据位宽
    parameter DEPTH   = 4,  // FIFO 深度（条目数）
    parameter ADDR_W  = 2   // 地址指针宽度，满足 2^ADDR_W >= DEPTH
)
//-----------------------------------------------------------------
// Ports
//-----------------------------------------------------------------
(
    // Inputs
     input               clk_i        // 时钟信号
    ,input               rst_i        // 异步复位（高有效）
    ,input  [WIDTH-1:0]  data_in_i    // 入队数据
    ,input               push_i       // 入队使能（配合 accept_o 握手）
    ,input               pop_i        // 出队使能（配合 valid_o 握手）

    // Outputs
    ,output [WIDTH-1:0]  data_out_o   // 队头数据（组合输出）
    ,output              accept_o     // FIFO 未满，可接受新入队
    ,output              valid_o      // FIFO 非空，队头数据有效
);

//-----------------------------------------------------------------
// Local Params
//-----------------------------------------------------------------
localparam COUNT_W = ADDR_W + 1; // 计数器宽度，多1位以区分满/空

//-----------------------------------------------------------------
// Registers —— FIFO 内部存储与指针
//-----------------------------------------------------------------
reg [WIDTH-1:0]   ram_q[DEPTH-1:0]; // FIFO 存储数组
reg [ADDR_W-1:0]  rd_ptr_q;         // 读指针（出队位置）
reg [ADDR_W-1:0]  wr_ptr_q;         // 写指针（入队位置）
reg [COUNT_W-1:0] count_q;          // 当前 FIFO 中的条目数

integer i;

//-----------------------------------------------------------------
// Sequential —— FIFO 入队/出队时序逻辑
// 支持同一周期同时 push 和 pop（计数不变）
//-----------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    count_q   <= {(COUNT_W) {1'b0}};
    rd_ptr_q  <= {(ADDR_W) {1'b0}};
    wr_ptr_q  <= {(ADDR_W) {1'b0}};

    for (i=0;i<DEPTH;i=i+1)
    begin
        ram_q[i] <= {(WIDTH) {1'b0}};
    end
end
else
begin
    // Push
    if (push_i & accept_o)
    begin
        ram_q[wr_ptr_q] <= data_in_i;
        wr_ptr_q        <= wr_ptr_q + 1;
    end

    // Pop
    if (pop_i & valid_o)
        rd_ptr_q      <= rd_ptr_q + 1;

    // Count up
    if ((push_i & accept_o) & ~(pop_i & valid_o))
        count_q <= count_q + 1;
    // Count down
    else if (~(push_i & accept_o) & (pop_i & valid_o))
        count_q <= count_q - 1;
end

//-------------------------------------------------------------------
// Combinatorial —— 状态输出
//-------------------------------------------------------------------
/* verilator lint_off WIDTH */
assign valid_o       = (count_q != 0);    // FIFO 非空
assign accept_o      = (count_q != DEPTH); // FIFO 未满
/* verilator lint_on WIDTH */

assign data_out_o    = ram_q[rd_ptr_q]; // 队头数据



endmodule
