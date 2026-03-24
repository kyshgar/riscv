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

// ================================================================
// 模块功能概述：RISC-V CPU 顶层模块
//   本模块是整个RISC-V处理器的顶层连接模块，负责将以下子模块
//   通过内部线网（wire）互连在一起：
//     - riscv_fetch    : 取指单元，从ICache取指令并做分支预测
//     - riscv_decode   : 译码单元，解析指令类型（ALU/LSU/MUL/DIV/CSR/Branch）
//     - riscv_issue    : 发射单元，读寄存器堆、数据前递、冒险检测、指令分发
//     - riscv_exec     : 执行单元（ALU），执行整数运算和分支
//     - riscv_lsu      : 访存单元，处理Load/Store
//     - riscv_csr      : CSR寄存器单元，处理特权指令和中断
//     - riscv_multiplier: 乘法器（M扩展）
//     - riscv_divider  : 除法器（M扩展）
//     - riscv_mmu      : 内存管理单元适配层（可选，支持Sv32页表翻译）
//   顶层本身不含时序逻辑，所有状态均在各子模块中维护。
// ================================================================
module riscv_core
//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
#(
     parameter SUPPORT_MULDIV   = 1  // 是否支持乘除法扩展（M扩展），1=支持，0=不支持
    ,parameter SUPPORT_SUPER    = 0  // 是否支持Supervisor特权级，1=支持，0=仅支持Machine级
    ,parameter SUPPORT_MMU      = 0  // 是否启用MMU（内存管理单元），1=启用Sv32页表翻译，0=物理地址直通
    ,parameter SUPPORT_LOAD_BYPASS = 1  // 是否支持Load结果直接旁路到后续指令，减少流水线停顿
    ,parameter SUPPORT_MUL_BYPASS = 1   // 是否支持乘法结果直接旁路到后续指令，减少流水线停顿
    ,parameter SUPPORT_REGFILE_XILINX = 0  // 针对Xilinx FPGA优化的寄存器堆实现，1=使用BRAM，0=使用LUT
    ,parameter EXTRA_DECODE_STAGE = 0   // 是否增加额外的译码流水线级，用于提高时序但增加延迟
    ,parameter MEM_CACHE_ADDR_MIN = 32'h80000000  // 可缓存内存区域的起始地址（含），用于判断访存是否可缓存
    ,parameter MEM_CACHE_ADDR_MAX = 32'h8fffffff  // 可缓存内存区域的结束地址（含），用于判断访存是否可缓存
)
//-----------------------------------------------------------------
// Ports
//-----------------------------------------------------------------
(
    // Inputs
     input           clk_i            // 系统时钟，上升沿触发
    ,input           rst_i            // 异步复位，高电平有效
    ,input  [ 31:0]  mem_d_data_rd_i  // DCache返回的读数据（32位）
    ,input           mem_d_accept_i   // DCache接受请求的握手信号
    ,input           mem_d_ack_i      // DCache完成请求的应答信号
    ,input           mem_d_error_i    // DCache总线错误标志
    ,input  [ 10:0]  mem_d_resp_tag_i // DCache响应标签，用于匹配请求
    ,input           mem_i_accept_i   // ICache接受取指请求的握手信号
    ,input           mem_i_valid_i    // ICache返回数据有效信号
    ,input           mem_i_error_i    // ICache总线错误标志
    ,input  [ 31:0]  mem_i_inst_i     // ICache返回的指令数据（32位）
    ,input           intr_i           // 外部中断输入信号，高电平有效
    ,input  [ 31:0]  reset_vector_i   // 复位后的程序起始地址（复位向量）
    ,input  [ 31:0]  cpu_id_i         // CPU唯一标识号，映射到mhartid CSR

    // Outputs
    ,output [ 31:0]  mem_d_addr_o      // DCache访问地址（32位）
    ,output [ 31:0]  mem_d_data_wr_o   // DCache写数据（32位）
    ,output          mem_d_rd_o        // DCache读使能
    ,output [  3:0]  mem_d_wr_o        // DCache字节写使能（4位，每位对应一个字节）
    ,output          mem_d_cacheable_o // 当前DCache访问地址是否可缓存
    ,output [ 10:0]  mem_d_req_tag_o   // DCache请求标签，用于匹配响应
    ,output          mem_d_invalidate_o// DCache缓存行无效化请求
    ,output          mem_d_writeback_o // DCache缓存行写回请求
    ,output          mem_d_flush_o     // DCache全部写回并无效化请求
    ,output          mem_i_rd_o        // ICache取指使能
    ,output          mem_i_flush_o     // ICache冲刷请求（ifence指令触发）
    ,output          mem_i_invalidate_o// ICache无效化请求
    ,output [ 31:0]  mem_i_pc_o        // 向ICache发出的取指地址（PC值）
);

// ---------------------------------------------------------------
// 内部线网信号声明：连接各子模块之间的数据通路和控制信号
// ---------------------------------------------------------------
wire           mmu_lsu_writeback_w;
wire  [  1:0]  fetch_in_priv_w;
wire  [  4:0]  mul_opcode_rd_idx_w;
wire           mmu_flush_w;
wire  [ 31:0]  lsu_opcode_pc_w;
wire           fetch_accept_w;
wire  [  4:0]  csr_opcode_rd_idx_w;
wire  [ 31:0]  branch_exec_source_w;
wire  [ 31:0]  csr_opcode_rb_operand_w;
wire  [ 31:0]  writeback_div_value_w;
wire           csr_opcode_valid_w;
wire           branch_csr_request_w;
wire  [ 31:0]  mmu_ifetch_inst_w;
wire  [ 31:0]  opcode_pc_w;
wire  [  4:0]  opcode_rb_idx_w;
wire           mmu_lsu_error_w;
wire           mul_opcode_valid_w;
wire           mmu_mxr_w;
wire  [  1:0]  branch_d_exec_priv_w;
wire           mmu_ifetch_valid_w;
wire           csr_opcode_invalid_w;
wire  [  5:0]  csr_writeback_exception_w;
wire           fetch_instr_mul_w;
wire           branch_exec_is_ret_w;
wire  [ 31:0]  csr_writeback_exception_addr_w;
wire  [  3:0]  mmu_lsu_wr_w;
wire           fetch_in_fault_w;
wire           branch_request_w;
wire  [ 31:0]  csr_opcode_pc_w;
wire           writeback_mem_valid_w;
wire  [  5:0]  csr_result_e1_exception_w;
wire  [ 31:0]  branch_csr_pc_w;
wire  [ 31:0]  mmu_lsu_data_wr_w;
wire           fetch_fault_page_w;
wire  [ 10:0]  mmu_lsu_resp_tag_w;
wire  [ 10:0]  mmu_lsu_req_tag_w;
wire  [ 31:0]  opcode_ra_operand_w;
wire           squash_decode_w;
wire           fetch_dec_fault_page_w;
wire  [ 31:0]  mul_opcode_opcode_w;
wire           exec_hold_w;
wire           fetch_instr_invalid_w;
wire  [ 31:0]  branch_pc_w;
wire  [  4:0]  mul_opcode_ra_idx_w;
wire  [  4:0]  csr_opcode_rb_idx_w;
wire           lsu_stall_w;
wire           branch_exec_is_not_taken_w;
wire  [ 31:0]  branch_exec_pc_w;
wire  [ 31:0]  opcode_opcode_w;
wire  [ 31:0]  mul_opcode_pc_w;
wire           branch_d_exec_request_w;
wire  [ 31:0]  mul_opcode_ra_operand_w;
wire           branch_exec_is_taken_w;
wire           fetch_dec_fault_fetch_w;
wire           fetch_dec_valid_w;
wire           fetch_fault_fetch_w;
wire           lsu_opcode_invalid_w;
wire  [ 31:0]  mmu_lsu_addr_w;
wire           mul_hold_w;
wire           mmu_ifetch_accept_w;
wire           mmu_lsu_ack_w;
wire  [ 31:0]  fetch_pc_w;
wire           mmu_ifetch_invalidate_w;
wire  [ 31:0]  mul_opcode_rb_operand_w;
wire  [  1:0]  branch_csr_priv_w;
wire           branch_exec_request_w;
wire  [ 31:0]  lsu_opcode_ra_operand_w;
wire           div_opcode_valid_w;
wire  [  1:0]  branch_priv_w;
wire           mmu_lsu_rd_w;
wire  [ 31:0]  fetch_dec_pc_w;
wire           interrupt_inhibit_w;
wire           mmu_ifetch_error_w;
wire  [  5:0]  writeback_mem_exception_w;
wire           fetch_instr_lsu_w;
wire  [  1:0]  mmu_priv_d_w;
wire  [  4:0]  opcode_ra_idx_w;
wire  [ 31:0]  csr_opcode_ra_operand_w;
wire  [ 31:0]  writeback_mem_value_w;
wire           writeback_div_valid_w;
wire  [  4:0]  mul_opcode_rb_idx_w;
wire           opcode_invalid_w;
wire           fetch_instr_branch_w;
wire  [ 31:0]  mmu_ifetch_pc_w;
wire           mmu_ifetch_rd_w;
wire           mmu_ifetch_flush_w;
wire  [  4:0]  lsu_opcode_rd_idx_w;
wire  [ 31:0]  lsu_opcode_opcode_w;
wire           mmu_load_fault_w;
wire  [ 31:0]  mmu_satp_w;
wire  [ 31:0]  csr_result_e1_wdata_w;
wire  [ 31:0]  opcode_rb_operand_w;
wire           mmu_lsu_invalidate_w;
wire           fetch_dec_accept_w;
wire  [  4:0]  csr_opcode_ra_idx_w;
wire           ifence_w;
wire           fetch_instr_exec_w;
wire  [  4:0]  opcode_rd_idx_w;
wire  [ 31:0]  csr_writeback_wdata_w;
wire           csr_writeback_write_w;
wire           take_interrupt_w;
wire  [ 31:0]  csr_result_e1_value_w;
wire  [ 31:0]  branch_d_exec_pc_w;
wire           fetch_valid_w;
wire  [ 11:0]  csr_writeback_waddr_w;
wire           branch_exec_is_jmp_w;
wire           mmu_lsu_cacheable_w;
wire           fetch_instr_csr_w;
wire           lsu_opcode_valid_w;
wire  [ 31:0]  fetch_dec_instr_w;
wire           csr_result_e1_write_w;
wire  [ 31:0]  csr_opcode_opcode_w;
wire           fetch_instr_div_w;
wire  [ 31:0]  fetch_instr_w;
wire           mul_opcode_invalid_w;
wire           fetch_instr_rd_valid_w;
wire  [ 31:0]  mmu_lsu_data_rd_w;
wire           exec_opcode_valid_w;
wire  [ 31:0]  writeback_mul_value_w;
wire           mmu_lsu_flush_w;
wire  [  4:0]  lsu_opcode_rb_idx_w;
wire           mmu_lsu_accept_w;
wire  [ 31:0]  lsu_opcode_rb_operand_w;
wire           mmu_sum_w;
wire  [ 31:0]  writeback_exec_value_w;
wire  [  4:0]  lsu_opcode_ra_idx_w;
wire  [ 31:0]  csr_writeback_exception_pc_w;
wire           mmu_store_fault_w;
wire           branch_exec_is_call_w;


// ---------------------------------------------------------------
// 执行单元：ALU整数运算、分支计算
// 接收发射级提供的指令、操作数，输出分支请求和写回值
// ---------------------------------------------------------------
riscv_exec
u_exec
(
    // Inputs
     .clk_i(clk_i)
    ,.rst_i(rst_i)
    ,.opcode_valid_i(exec_opcode_valid_w)
    ,.opcode_opcode_i(opcode_opcode_w)
    ,.opcode_pc_i(opcode_pc_w)
    ,.opcode_invalid_i(opcode_invalid_w)
    ,.opcode_rd_idx_i(opcode_rd_idx_w)
    ,.opcode_ra_idx_i(opcode_ra_idx_w)
    ,.opcode_rb_idx_i(opcode_rb_idx_w)
    ,.opcode_ra_operand_i(opcode_ra_operand_w)
    ,.opcode_rb_operand_i(opcode_rb_operand_w)
    ,.hold_i(exec_hold_w)

    // Outputs
    ,.branch_request_o(branch_exec_request_w)
    ,.branch_is_taken_o(branch_exec_is_taken_w)
    ,.branch_is_not_taken_o(branch_exec_is_not_taken_w)
    ,.branch_source_o(branch_exec_source_w)
    ,.branch_is_call_o(branch_exec_is_call_w)
    ,.branch_is_ret_o(branch_exec_is_ret_w)
    ,.branch_is_jmp_o(branch_exec_is_jmp_w)
    ,.branch_pc_o(branch_exec_pc_w)
    ,.branch_d_request_o(branch_d_exec_request_w)
    ,.branch_d_pc_o(branch_d_exec_pc_w)
    ,.branch_d_priv_o(branch_d_exec_priv_w)
    ,.writeback_value_o(writeback_exec_value_w)
);


// ---------------------------------------------------------------
// 译码单元：解析取回的指令，标记指令类型（执行/访存/分支/乘除/CSR）
// 支持可选的额外流水线级（EXTRA_DECODE_STAGE）以改善时序
// ---------------------------------------------------------------
riscv_decode
#(
     .EXTRA_DECODE_STAGE(EXTRA_DECODE_STAGE)
    ,.SUPPORT_MULDIV(SUPPORT_MULDIV)
)
u_decode
(
    // Inputs
     .clk_i(clk_i)
    ,.rst_i(rst_i)
    ,.fetch_in_valid_i(fetch_dec_valid_w)
    ,.fetch_in_instr_i(fetch_dec_instr_w)
    ,.fetch_in_pc_i(fetch_dec_pc_w)
    ,.fetch_in_fault_fetch_i(fetch_dec_fault_fetch_w)
    ,.fetch_in_fault_page_i(fetch_dec_fault_page_w)
    ,.fetch_out_accept_i(fetch_accept_w)
    ,.squash_decode_i(squash_decode_w)

    // Outputs
    ,.fetch_in_accept_o(fetch_dec_accept_w)
    ,.fetch_out_valid_o(fetch_valid_w)
    ,.fetch_out_instr_o(fetch_instr_w)
    ,.fetch_out_pc_o(fetch_pc_w)
    ,.fetch_out_fault_fetch_o(fetch_fault_fetch_w)
    ,.fetch_out_fault_page_o(fetch_fault_page_w)
    ,.fetch_out_instr_exec_o(fetch_instr_exec_w)
    ,.fetch_out_instr_lsu_o(fetch_instr_lsu_w)
    ,.fetch_out_instr_branch_o(fetch_instr_branch_w)
    ,.fetch_out_instr_mul_o(fetch_instr_mul_w)
    ,.fetch_out_instr_div_o(fetch_instr_div_w)
    ,.fetch_out_instr_csr_o(fetch_instr_csr_w)
    ,.fetch_out_instr_rd_valid_o(fetch_instr_rd_valid_w)
    ,.fetch_out_instr_invalid_o(fetch_instr_invalid_w)
);


// ---------------------------------------------------------------
// MMU适配层：当SUPPORT_MMU=0时物理地址直通；=1时进行Sv32页表翻译
// 同时负责ICache和DCache的地址翻译和可缓存性判定
// ---------------------------------------------------------------
riscv_mmu
#(
     .MEM_CACHE_ADDR_MAX(MEM_CACHE_ADDR_MAX)
    ,.SUPPORT_MMU(SUPPORT_MMU)
    ,.MEM_CACHE_ADDR_MIN(MEM_CACHE_ADDR_MIN)
)
u_mmu
(
    // Inputs
     .clk_i(clk_i)
    ,.rst_i(rst_i)
    ,.priv_d_i(mmu_priv_d_w)
    ,.sum_i(mmu_sum_w)
    ,.mxr_i(mmu_mxr_w)
    ,.flush_i(mmu_flush_w)
    ,.satp_i(mmu_satp_w)
    ,.fetch_in_rd_i(mmu_ifetch_rd_w)
    ,.fetch_in_flush_i(mmu_ifetch_flush_w)
    ,.fetch_in_invalidate_i(mmu_ifetch_invalidate_w)
    ,.fetch_in_pc_i(mmu_ifetch_pc_w)
    ,.fetch_in_priv_i(fetch_in_priv_w)
    ,.fetch_out_accept_i(mem_i_accept_i)
    ,.fetch_out_valid_i(mem_i_valid_i)
    ,.fetch_out_error_i(mem_i_error_i)
    ,.fetch_out_inst_i(mem_i_inst_i)
    ,.lsu_in_addr_i(mmu_lsu_addr_w)
    ,.lsu_in_data_wr_i(mmu_lsu_data_wr_w)
    ,.lsu_in_rd_i(mmu_lsu_rd_w)
    ,.lsu_in_wr_i(mmu_lsu_wr_w)
    ,.lsu_in_cacheable_i(mmu_lsu_cacheable_w)
    ,.lsu_in_req_tag_i(mmu_lsu_req_tag_w)
    ,.lsu_in_invalidate_i(mmu_lsu_invalidate_w)
    ,.lsu_in_writeback_i(mmu_lsu_writeback_w)
    ,.lsu_in_flush_i(mmu_lsu_flush_w)
    ,.lsu_out_data_rd_i(mem_d_data_rd_i)
    ,.lsu_out_accept_i(mem_d_accept_i)
    ,.lsu_out_ack_i(mem_d_ack_i)
    ,.lsu_out_error_i(mem_d_error_i)
    ,.lsu_out_resp_tag_i(mem_d_resp_tag_i)

    // Outputs
    ,.fetch_in_accept_o(mmu_ifetch_accept_w)
    ,.fetch_in_valid_o(mmu_ifetch_valid_w)
    ,.fetch_in_error_o(mmu_ifetch_error_w)
    ,.fetch_in_inst_o(mmu_ifetch_inst_w)
    ,.fetch_out_rd_o(mem_i_rd_o)
    ,.fetch_out_flush_o(mem_i_flush_o)
    ,.fetch_out_invalidate_o(mem_i_invalidate_o)
    ,.fetch_out_pc_o(mem_i_pc_o)
    ,.fetch_in_fault_o(fetch_in_fault_w)
    ,.lsu_in_data_rd_o(mmu_lsu_data_rd_w)
    ,.lsu_in_accept_o(mmu_lsu_accept_w)
    ,.lsu_in_ack_o(mmu_lsu_ack_w)
    ,.lsu_in_error_o(mmu_lsu_error_w)
    ,.lsu_in_resp_tag_o(mmu_lsu_resp_tag_w)
    ,.lsu_out_addr_o(mem_d_addr_o)
    ,.lsu_out_data_wr_o(mem_d_data_wr_o)
    ,.lsu_out_rd_o(mem_d_rd_o)
    ,.lsu_out_wr_o(mem_d_wr_o)
    ,.lsu_out_cacheable_o(mem_d_cacheable_o)
    ,.lsu_out_req_tag_o(mem_d_req_tag_o)
    ,.lsu_out_invalidate_o(mem_d_invalidate_o)
    ,.lsu_out_writeback_o(mem_d_writeback_o)
    ,.lsu_out_flush_o(mem_d_flush_o)
    ,.lsu_in_load_fault_o(mmu_load_fault_w)
    ,.lsu_in_store_fault_o(mmu_store_fault_w)
);


// ---------------------------------------------------------------
// 访存单元：处理Load/Store指令，管理内存请求和写回
// ---------------------------------------------------------------
riscv_lsu
#(
     .MEM_CACHE_ADDR_MAX(MEM_CACHE_ADDR_MAX)
    ,.MEM_CACHE_ADDR_MIN(MEM_CACHE_ADDR_MIN)
)
u_lsu
(
    // Inputs
     .clk_i(clk_i)
    ,.rst_i(rst_i)
    ,.opcode_valid_i(lsu_opcode_valid_w)
    ,.opcode_opcode_i(lsu_opcode_opcode_w)
    ,.opcode_pc_i(lsu_opcode_pc_w)
    ,.opcode_invalid_i(lsu_opcode_invalid_w)
    ,.opcode_rd_idx_i(lsu_opcode_rd_idx_w)
    ,.opcode_ra_idx_i(lsu_opcode_ra_idx_w)
    ,.opcode_rb_idx_i(lsu_opcode_rb_idx_w)
    ,.opcode_ra_operand_i(lsu_opcode_ra_operand_w)
    ,.opcode_rb_operand_i(lsu_opcode_rb_operand_w)
    ,.mem_data_rd_i(mmu_lsu_data_rd_w)
    ,.mem_accept_i(mmu_lsu_accept_w)
    ,.mem_ack_i(mmu_lsu_ack_w)
    ,.mem_error_i(mmu_lsu_error_w)
    ,.mem_resp_tag_i(mmu_lsu_resp_tag_w)
    ,.mem_load_fault_i(mmu_load_fault_w)
    ,.mem_store_fault_i(mmu_store_fault_w)

    // Outputs
    ,.mem_addr_o(mmu_lsu_addr_w)
    ,.mem_data_wr_o(mmu_lsu_data_wr_w)
    ,.mem_rd_o(mmu_lsu_rd_w)
    ,.mem_wr_o(mmu_lsu_wr_w)
    ,.mem_cacheable_o(mmu_lsu_cacheable_w)
    ,.mem_req_tag_o(mmu_lsu_req_tag_w)
    ,.mem_invalidate_o(mmu_lsu_invalidate_w)
    ,.mem_writeback_o(mmu_lsu_writeback_w)
    ,.mem_flush_o(mmu_lsu_flush_w)
    ,.writeback_valid_o(writeback_mem_valid_w)
    ,.writeback_value_o(writeback_mem_value_w)
    ,.writeback_exception_o(writeback_mem_exception_w)
    ,.stall_o(lsu_stall_w)
);


// ---------------------------------------------------------------
// CSR单元：处理特权指令（ecall/ebreak/xret）、中断、异常
// 输出MMU控制信号（satp、priv、sum、mxr）和分支请求
// ---------------------------------------------------------------
riscv_csr
#(
     .SUPPORT_SUPER(SUPPORT_SUPER)
    ,.SUPPORT_MULDIV(SUPPORT_MULDIV)
)
u_csr
(
    // Inputs
     .clk_i(clk_i)
    ,.rst_i(rst_i)
    ,.intr_i(intr_i)
    ,.opcode_valid_i(csr_opcode_valid_w)
    ,.opcode_opcode_i(csr_opcode_opcode_w)
    ,.opcode_pc_i(csr_opcode_pc_w)
    ,.opcode_invalid_i(csr_opcode_invalid_w)
    ,.opcode_rd_idx_i(csr_opcode_rd_idx_w)
    ,.opcode_ra_idx_i(csr_opcode_ra_idx_w)
    ,.opcode_rb_idx_i(csr_opcode_rb_idx_w)
    ,.opcode_ra_operand_i(csr_opcode_ra_operand_w)
    ,.opcode_rb_operand_i(csr_opcode_rb_operand_w)
    ,.csr_writeback_write_i(csr_writeback_write_w)
    ,.csr_writeback_waddr_i(csr_writeback_waddr_w)
    ,.csr_writeback_wdata_i(csr_writeback_wdata_w)
    ,.csr_writeback_exception_i(csr_writeback_exception_w)
    ,.csr_writeback_exception_pc_i(csr_writeback_exception_pc_w)
    ,.csr_writeback_exception_addr_i(csr_writeback_exception_addr_w)
    ,.cpu_id_i(cpu_id_i)
    ,.reset_vector_i(reset_vector_i)
    ,.interrupt_inhibit_i(interrupt_inhibit_w)

    // Outputs
    ,.csr_result_e1_value_o(csr_result_e1_value_w)
    ,.csr_result_e1_write_o(csr_result_e1_write_w)
    ,.csr_result_e1_wdata_o(csr_result_e1_wdata_w)
    ,.csr_result_e1_exception_o(csr_result_e1_exception_w)
    ,.branch_csr_request_o(branch_csr_request_w)
    ,.branch_csr_pc_o(branch_csr_pc_w)
    ,.branch_csr_priv_o(branch_csr_priv_w)
    ,.take_interrupt_o(take_interrupt_w)
    ,.ifence_o(ifence_w)
    ,.mmu_priv_d_o(mmu_priv_d_w)
    ,.mmu_sum_o(mmu_sum_w)
    ,.mmu_mxr_o(mmu_mxr_w)
    ,.mmu_flush_o(mmu_flush_w)
    ,.mmu_satp_o(mmu_satp_w)
);


// ---------------------------------------------------------------
// 乘法器：处理MUL/MULH/MULHSU/MULHU指令（M扩展）
// ---------------------------------------------------------------
riscv_multiplier
u_mul
(
    // Inputs
     .clk_i(clk_i)
    ,.rst_i(rst_i)
    ,.opcode_valid_i(mul_opcode_valid_w)
    ,.opcode_opcode_i(mul_opcode_opcode_w)
    ,.opcode_pc_i(mul_opcode_pc_w)
    ,.opcode_invalid_i(mul_opcode_invalid_w)
    ,.opcode_rd_idx_i(mul_opcode_rd_idx_w)
    ,.opcode_ra_idx_i(mul_opcode_ra_idx_w)
    ,.opcode_rb_idx_i(mul_opcode_rb_idx_w)
    ,.opcode_ra_operand_i(mul_opcode_ra_operand_w)
    ,.opcode_rb_operand_i(mul_opcode_rb_operand_w)
    ,.hold_i(mul_hold_w)

    // Outputs
    ,.writeback_value_o(writeback_mul_value_w)
);


// ---------------------------------------------------------------
// 除法器：处理DIV/DIVU/REM/REMU指令（M扩展），多周期完成
// ---------------------------------------------------------------
riscv_divider
u_div
(
    // Inputs
     .clk_i(clk_i)
    ,.rst_i(rst_i)
    ,.opcode_valid_i(div_opcode_valid_w)
    ,.opcode_opcode_i(opcode_opcode_w)
    ,.opcode_pc_i(opcode_pc_w)
    ,.opcode_invalid_i(opcode_invalid_w)
    ,.opcode_rd_idx_i(opcode_rd_idx_w)
    ,.opcode_ra_idx_i(opcode_ra_idx_w)
    ,.opcode_rb_idx_i(opcode_rb_idx_w)
    ,.opcode_ra_operand_i(opcode_ra_operand_w)
    ,.opcode_rb_operand_i(opcode_rb_operand_w)

    // Outputs
    ,.writeback_valid_o(writeback_div_valid_w)
    ,.writeback_value_o(writeback_div_value_w)
);


// ---------------------------------------------------------------
// 发射单元：读寄存器堆、数据前递/旁路、RAW冒险检测
// 负责将指令分发到执行/访存/乘法/除法/CSR各执行单元
// ---------------------------------------------------------------
riscv_issue
#(
     .SUPPORT_REGFILE_XILINX(SUPPORT_REGFILE_XILINX)
    ,.SUPPORT_LOAD_BYPASS(SUPPORT_LOAD_BYPASS)
    ,.SUPPORT_MULDIV(SUPPORT_MULDIV)
    ,.SUPPORT_MUL_BYPASS(SUPPORT_MUL_BYPASS)
    ,.SUPPORT_DUAL_ISSUE(1)
)
u_issue
(
    // Inputs
     .clk_i(clk_i)
    ,.rst_i(rst_i)
    ,.fetch_valid_i(fetch_valid_w)
    ,.fetch_instr_i(fetch_instr_w)
    ,.fetch_pc_i(fetch_pc_w)
    ,.fetch_fault_fetch_i(fetch_fault_fetch_w)
    ,.fetch_fault_page_i(fetch_fault_page_w)
    ,.fetch_instr_exec_i(fetch_instr_exec_w)
    ,.fetch_instr_lsu_i(fetch_instr_lsu_w)
    ,.fetch_instr_branch_i(fetch_instr_branch_w)
    ,.fetch_instr_mul_i(fetch_instr_mul_w)
    ,.fetch_instr_div_i(fetch_instr_div_w)
    ,.fetch_instr_csr_i(fetch_instr_csr_w)
    ,.fetch_instr_rd_valid_i(fetch_instr_rd_valid_w)
    ,.fetch_instr_invalid_i(fetch_instr_invalid_w)
    ,.branch_exec_request_i(branch_exec_request_w)
    ,.branch_exec_is_taken_i(branch_exec_is_taken_w)
    ,.branch_exec_is_not_taken_i(branch_exec_is_not_taken_w)
    ,.branch_exec_source_i(branch_exec_source_w)
    ,.branch_exec_is_call_i(branch_exec_is_call_w)
    ,.branch_exec_is_ret_i(branch_exec_is_ret_w)
    ,.branch_exec_is_jmp_i(branch_exec_is_jmp_w)
    ,.branch_exec_pc_i(branch_exec_pc_w)
    ,.branch_d_exec_request_i(branch_d_exec_request_w)
    ,.branch_d_exec_pc_i(branch_d_exec_pc_w)
    ,.branch_d_exec_priv_i(branch_d_exec_priv_w)
    ,.branch_csr_request_i(branch_csr_request_w)
    ,.branch_csr_pc_i(branch_csr_pc_w)
    ,.branch_csr_priv_i(branch_csr_priv_w)
    ,.writeback_exec_value_i(writeback_exec_value_w)
    ,.writeback_mem_valid_i(writeback_mem_valid_w)
    ,.writeback_mem_value_i(writeback_mem_value_w)
    ,.writeback_mem_exception_i(writeback_mem_exception_w)
    ,.writeback_mul_value_i(writeback_mul_value_w)
    ,.writeback_div_valid_i(writeback_div_valid_w)
    ,.writeback_div_value_i(writeback_div_value_w)
    ,.csr_result_e1_value_i(csr_result_e1_value_w)
    ,.csr_result_e1_write_i(csr_result_e1_write_w)
    ,.csr_result_e1_wdata_i(csr_result_e1_wdata_w)
    ,.csr_result_e1_exception_i(csr_result_e1_exception_w)
    ,.lsu_stall_i(lsu_stall_w)
    ,.take_interrupt_i(take_interrupt_w)

    // Outputs
    ,.fetch_accept_o(fetch_accept_w)
    ,.branch_request_o(branch_request_w)
    ,.branch_pc_o(branch_pc_w)
    ,.branch_priv_o(branch_priv_w)
    ,.exec_opcode_valid_o(exec_opcode_valid_w)
    ,.lsu_opcode_valid_o(lsu_opcode_valid_w)
    ,.csr_opcode_valid_o(csr_opcode_valid_w)
    ,.mul_opcode_valid_o(mul_opcode_valid_w)
    ,.div_opcode_valid_o(div_opcode_valid_w)
    ,.opcode_opcode_o(opcode_opcode_w)
    ,.opcode_pc_o(opcode_pc_w)
    ,.opcode_invalid_o(opcode_invalid_w)
    ,.opcode_rd_idx_o(opcode_rd_idx_w)
    ,.opcode_ra_idx_o(opcode_ra_idx_w)
    ,.opcode_rb_idx_o(opcode_rb_idx_w)
    ,.opcode_ra_operand_o(opcode_ra_operand_w)
    ,.opcode_rb_operand_o(opcode_rb_operand_w)
    ,.lsu_opcode_opcode_o(lsu_opcode_opcode_w)
    ,.lsu_opcode_pc_o(lsu_opcode_pc_w)
    ,.lsu_opcode_invalid_o(lsu_opcode_invalid_w)
    ,.lsu_opcode_rd_idx_o(lsu_opcode_rd_idx_w)
    ,.lsu_opcode_ra_idx_o(lsu_opcode_ra_idx_w)
    ,.lsu_opcode_rb_idx_o(lsu_opcode_rb_idx_w)
    ,.lsu_opcode_ra_operand_o(lsu_opcode_ra_operand_w)
    ,.lsu_opcode_rb_operand_o(lsu_opcode_rb_operand_w)
    ,.mul_opcode_opcode_o(mul_opcode_opcode_w)
    ,.mul_opcode_pc_o(mul_opcode_pc_w)
    ,.mul_opcode_invalid_o(mul_opcode_invalid_w)
    ,.mul_opcode_rd_idx_o(mul_opcode_rd_idx_w)
    ,.mul_opcode_ra_idx_o(mul_opcode_ra_idx_w)
    ,.mul_opcode_rb_idx_o(mul_opcode_rb_idx_w)
    ,.mul_opcode_ra_operand_o(mul_opcode_ra_operand_w)
    ,.mul_opcode_rb_operand_o(mul_opcode_rb_operand_w)
    ,.csr_opcode_opcode_o(csr_opcode_opcode_w)
    ,.csr_opcode_pc_o(csr_opcode_pc_w)
    ,.csr_opcode_invalid_o(csr_opcode_invalid_w)
    ,.csr_opcode_rd_idx_o(csr_opcode_rd_idx_w)
    ,.csr_opcode_ra_idx_o(csr_opcode_ra_idx_w)
    ,.csr_opcode_rb_idx_o(csr_opcode_rb_idx_w)
    ,.csr_opcode_ra_operand_o(csr_opcode_ra_operand_w)
    ,.csr_opcode_rb_operand_o(csr_opcode_rb_operand_w)
    ,.csr_writeback_write_o(csr_writeback_write_w)
    ,.csr_writeback_waddr_o(csr_writeback_waddr_w)
    ,.csr_writeback_wdata_o(csr_writeback_wdata_w)
    ,.csr_writeback_exception_o(csr_writeback_exception_w)
    ,.csr_writeback_exception_pc_o(csr_writeback_exception_pc_w)
    ,.csr_writeback_exception_addr_o(csr_writeback_exception_addr_w)
    ,.exec_hold_o(exec_hold_w)
    ,.mul_hold_o(mul_hold_w)
    ,.interrupt_inhibit_o(interrupt_inhibit_w)
);


// ---------------------------------------------------------------
// 取指单元：向ICache发送取指请求，处理分支跳转和指令缺页异常
// ---------------------------------------------------------------
riscv_fetch
#(
     .SUPPORT_MMU(SUPPORT_MMU)
)
u_fetch
(
    // Inputs
     .clk_i(clk_i)
    ,.rst_i(rst_i)
    ,.fetch_accept_i(fetch_dec_accept_w)
    ,.icache_accept_i(mmu_ifetch_accept_w)
    ,.icache_valid_i(mmu_ifetch_valid_w)
    ,.icache_error_i(mmu_ifetch_error_w)
    ,.icache_inst_i(mmu_ifetch_inst_w)
    ,.icache_page_fault_i(fetch_in_fault_w)
    ,.fetch_invalidate_i(ifence_w)
    ,.branch_request_i(branch_request_w)
    ,.branch_pc_i(branch_pc_w)
    ,.branch_priv_i(branch_priv_w)

    // Outputs
    ,.fetch_valid_o(fetch_dec_valid_w)
    ,.fetch_instr_o(fetch_dec_instr_w)
    ,.fetch_pc_o(fetch_dec_pc_w)
    ,.fetch_fault_fetch_o(fetch_dec_fault_fetch_w)
    ,.fetch_fault_page_o(fetch_dec_fault_page_w)
    ,.icache_rd_o(mmu_ifetch_rd_w)
    ,.icache_flush_o(mmu_ifetch_flush_w)
    ,.icache_invalidate_o(mmu_ifetch_invalidate_w)
    ,.icache_pc_o(mmu_ifetch_pc_w)
    ,.icache_priv_o(fetch_in_priv_w)
    ,.squash_decode_o(squash_decode_w)
);



endmodule
