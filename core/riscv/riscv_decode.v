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
// 模块: riscv_decode
// 功能: 译码级包装模块（Decode Stage Wrapper）
//   - 根据参数 EXTRA_DECODE_STAGE 选择两种实现路径：
//       0（直通模式）：取指结果直接送入 riscv_decoder 进行组合译码，
//                      无额外流水寄存器，延迟最小但关键路径可能较长；
//       1（流水模式）：在取指结果和 riscv_decoder 之间插入一级
//                      流水寄存器（buffer_q），改善时序/提升频率。
//   - 内部例化 riscv_decoder 完成指令类型分类
//   - 支持分支冲刷（squash_decode_i 清除流水级寄存器）
// 流水线位置: 第二级 —— 译码（ID）
// ============================================================
module riscv_decode
//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
#(
     parameter SUPPORT_MULDIV   = 1    // 是否支持乘除法指令（M 扩展）
    ,parameter EXTRA_DECODE_STAGE = 0  // 是否插入额外译码流水级（1=插入，改善时序；0=直通）
)
//-----------------------------------------------------------------
// Ports
//-----------------------------------------------------------------
(
    // Inputs
     input           clk_i                  // 系统时钟
    ,input           rst_i                  // 同步复位（高有效）
    ,input           fetch_in_valid_i       // 取指结果有效
    ,input  [ 31:0]  fetch_in_instr_i       // 取回的指令字
    ,input  [ 31:0]  fetch_in_pc_i          // 取回指令的 PC
    ,input           fetch_in_fault_fetch_i // 取指总线错误标志
    ,input           fetch_in_fault_page_i  // 取指页错误标志
    ,input           fetch_out_accept_i     // 下游（执行级）可以接收译码结果
    ,input           squash_decode_i        // 冲刷信号：清空流水级寄存器（分支冲刷）

    // Outputs
    ,output          fetch_in_accept_o          // 本级可以接收新的取指结果
    ,output          fetch_out_valid_o           // 译码结果有效
    ,output [ 31:0]  fetch_out_instr_o           // 译码后输出的指令字
    ,output [ 31:0]  fetch_out_pc_o              // 译码后输出的 PC
    ,output          fetch_out_fault_fetch_o     // 输出的取指总线错误标志
    ,output          fetch_out_fault_page_o      // 输出的取指页错误标志
    ,output          fetch_out_instr_exec_o      // 指令类型：整数运算（exec）
    ,output          fetch_out_instr_lsu_o       // 指令类型：访存（lsu）
    ,output          fetch_out_instr_branch_o    // 指令类型：分支/跳转（branch）
    ,output          fetch_out_instr_mul_o       // 指令类型：乘法（mul）
    ,output          fetch_out_instr_div_o       // 指令类型：除法（div）
    ,output          fetch_out_instr_csr_o       // 指令类型：CSR/系统指令（csr）
    ,output          fetch_out_instr_rd_valid_o  // 目标寄存器 rd 是否有效（有写回）
    ,output          fetch_out_instr_invalid_o   // 非法指令标志
);



// 乘除法使能信号：由参数 SUPPORT_MULDIV 决定
wire        enable_muldiv_w     = SUPPORT_MULDIV;

//-----------------------------------------------------------------
// Extra decode stage (to improve cycle time)
// 额外流水级（改善时序）
// 当 EXTRA_DECODE_STAGE=1 时：取指结果先存入流水寄存器 buffer_q，
// 再送入 riscv_decoder，打断组合逻辑关键路径，提升工作频率。
//-----------------------------------------------------------------
generate
if (EXTRA_DECODE_STAGE)
begin
    // 有取指错误时，将指令字清零，防止非法指令码进入译码器
    wire [31:0] fetch_in_instr_w = (fetch_in_fault_page_i | fetch_in_fault_fetch_i) ? 32'b0 : fetch_in_instr_i;
    // buffer_q 位域: [66]=valid, [65]=page_fault, [64]=fetch_fault,
    //               [63:32]=instr, [31:0]=pc
    reg [66:0]  buffer_q;  // 额外流水级寄存器（67 位）

    // 流水级寄存器更新逻辑：
    //   冲刷时清零；下游接受或本级为空时，从取指级采样新数据
    always @(posedge clk_i or posedge rst_i)
    if (rst_i)
        buffer_q <= 67'b0;
    else if (squash_decode_i)
        // 分支冲刷：清空流水寄存器，使本级无效
        buffer_q <= 67'b0;
    else if (fetch_out_accept_i || !fetch_out_valid_o)
        // 下游已接受或本级空闲：锁入新数据
        buffer_q <= {fetch_in_valid_i, fetch_in_fault_page_i, fetch_in_fault_fetch_i, fetch_in_instr_w, fetch_in_pc_i};

    // 将流水寄存器内容拆分并输出到下游
    assign {fetch_out_valid_o,
            fetch_out_fault_page_o,
            fetch_out_fault_fetch_o,
            fetch_out_instr_o,
            fetch_out_pc_o} = buffer_q;

    // 例化 riscv_decoder：对已缓冲的指令进行组合译码，输出指令类型标志
    riscv_decoder
    u_dec
    (
         .valid_i(fetch_out_valid_o)
        ,.fetch_fault_i(fetch_out_fault_page_o | fetch_out_fault_fetch_o)
        ,.enable_muldiv_i(enable_muldiv_w)
        ,.opcode_i(fetch_out_instr_o)

        ,.invalid_o(fetch_out_instr_invalid_o)
        ,.exec_o(fetch_out_instr_exec_o)
        ,.lsu_o(fetch_out_instr_lsu_o)
        ,.branch_o(fetch_out_instr_branch_o)
        ,.mul_o(fetch_out_instr_mul_o)
        ,.div_o(fetch_out_instr_div_o)
        ,.csr_o(fetch_out_instr_csr_o)
        ,.rd_valid_o(fetch_out_instr_rd_valid_o)
    );

    // 本级背压逻辑：直接传递下游的接受信号（本级寄存器满足单拍接受）
    assign fetch_in_accept_o        = fetch_out_accept_i;
end
//-----------------------------------------------------------------
// Straight through decode（直通译码，无额外流水寄存器）
// 当 EXTRA_DECODE_STAGE=0 时：取指数据直接驱动输出和 riscv_decoder，
// 无额外延迟，但组合路径较长。
//-----------------------------------------------------------------
else
begin
    // 有取指错误时，将指令字清零，避免非法指令码干扰译码器
    wire [31:0] fetch_in_instr_w = (fetch_in_fault_page_i | fetch_in_fault_fetch_i) ? 32'b0 : fetch_in_instr_i;

    // 直通模式：取指结果直接送入 riscv_decoder 进行组合译码
    riscv_decoder
    u_dec
    (
         .valid_i(fetch_in_valid_i)
        ,.fetch_fault_i(fetch_in_fault_fetch_i | fetch_in_fault_page_i)
        ,.enable_muldiv_i(enable_muldiv_w)
        ,.opcode_i(fetch_out_instr_o)

        ,.invalid_o(fetch_out_instr_invalid_o)
        ,.exec_o(fetch_out_instr_exec_o)
        ,.lsu_o(fetch_out_instr_lsu_o)
        ,.branch_o(fetch_out_instr_branch_o)
        ,.mul_o(fetch_out_instr_mul_o)
        ,.div_o(fetch_out_instr_div_o)
        ,.csr_o(fetch_out_instr_csr_o)
        ,.rd_valid_o(fetch_out_instr_rd_valid_o)
    );

    // Outputs（直接将取指信号透传为译码输出）
    assign fetch_out_valid_o        = fetch_in_valid_i;
    assign fetch_out_pc_o           = fetch_in_pc_i;
    assign fetch_out_instr_o        = fetch_in_instr_w;
    assign fetch_out_fault_page_o   = fetch_in_fault_page_i;
    assign fetch_out_fault_fetch_o  = fetch_in_fault_fetch_i;

    // 直通模式下，本级不引入额外背压，直接透传下游接受信号
    assign fetch_in_accept_o        = fetch_out_accept_i;
end
endgenerate


endmodule
