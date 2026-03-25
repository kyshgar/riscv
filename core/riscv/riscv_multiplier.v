// ============================================================
// 模块: riscv_multiplier
// 功能: RV32M 乘法器，支持以下四种乘法指令：
//   MUL    - 有符号×有符号，取乘积低32位
//   MULH   - 有符号×有符号，取乘积高32位
//   MULHSU - 有符号×无符号，取乘积高32位
//   MULHU  - 无符号×无符号，取乘积高32位
// 实现：2或3级流水线（由 MULT_STAGES 参数控制）
//   E1级：操作数符号扩展与锁存
//   E2级：65位乘法结果计算与结果选择
//   E3级（可选）：额外一拍寄存，提高时序裕量
// ============================================================
module riscv_multiplier
(
    // Inputs
     input           clk_i              // 系统时钟
    ,input           rst_i              // 同步复位（高有效）
    ,input           opcode_valid_i     // 当前指令有效标志
    ,input  [ 31:0]  opcode_opcode_i    // 指令编码（用于区分MUL/MULH/MULHSU/MULHU）
    ,input  [ 31:0]  opcode_pc_i        // 当前指令PC（本模块未使用，保持接口完整）
    ,input           opcode_invalid_i   // 指令非法标志（本模块未使用）
    ,input  [  4:0]  opcode_rd_idx_i    // 目标寄存器索引（本模块未使用）
    ,input  [  4:0]  opcode_ra_idx_i    // 源寄存器A索引（本模块未使用）
    ,input  [  4:0]  opcode_rb_idx_i    // 源寄存器B索引（本模块未使用）
    ,input  [ 31:0]  opcode_ra_operand_i // 源操作数A（rs1）
    ,input  [ 31:0]  opcode_rb_operand_i // 源操作数B（rs2）
    ,input           hold_i             // 流水线暂停信号，高有效时冻结所有流水级

    // Outputs
    ,output [ 31:0]  writeback_value_o  // 乘法结果写回值（低32位或高32位）
);



//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

localparam MULT_STAGES = 2; // 流水线级数：2或3，值为3时增加额外寄存级以提高频率

//-------------------------------------------------------------
// Registers / Wires（流水线各级寄存器）
//-------------------------------------------------------------
reg  [31:0]  result_e2_q;    // E2级结果寄存器（乘法输出后的第一个锁存）
reg  [31:0]  result_e3_q;    // E3级结果寄存器（可选的第三级流水寄存，用于3级流水模式）

reg [32:0]   operand_a_e1_q; // E1级操作数A锁存（33位，含符号扩展位）
reg [32:0]   operand_b_e1_q; // E1级操作数B锁存（33位，含符号扩展位）
reg          mulhi_sel_e1_q; // E1级高32位选择标志：1=取高32位(MULH/MULHSU/MULHU)，0=取低32位(MUL)

//-------------------------------------------------------------
// Multiplier（乘法器主体逻辑）
//-------------------------------------------------------------
wire [64:0]  mult_result_w;  // 65位全精度乘法结果（含符号扩展溢出位）
reg  [32:0]  operand_b_r;   // 组合逻辑：当前周期操作数B（带符号扩展，33位）
reg  [32:0]  operand_a_r;   // 组合逻辑：当前周期操作数A（带符号扩展，33位）
reg  [31:0]  result_r;      // 组合逻辑：从64位乘积中选取高/低32位

// 乘法指令检测：只要是MUL/MULH/MULHSU/MULHU中的任意一种则为高
wire mult_inst_w    = ((opcode_opcode_i & `INST_MUL_MASK) == `INST_MUL)        || 
                      ((opcode_opcode_i & `INST_MULH_MASK) == `INST_MULH)      ||
                      ((opcode_opcode_i & `INST_MULHSU_MASK) == `INST_MULHSU)  ||
                      ((opcode_opcode_i & `INST_MULHU_MASK) == `INST_MULHU);

// 操作数A符号扩展：
// MULHSU/MULH：rs1视为有符号数，扩展其符号位（第32位=rs1[31]）
// MUL/MULHU：  rs1视为无符号数，扩展0
always @ *
begin
    if ((opcode_opcode_i & `INST_MULHSU_MASK) == `INST_MULHSU)
        operand_a_r = {opcode_ra_operand_i[31], opcode_ra_operand_i[31:0]};
    else if ((opcode_opcode_i & `INST_MULH_MASK) == `INST_MULH)
        operand_a_r = {opcode_ra_operand_i[31], opcode_ra_operand_i[31:0]};
    else // MULHU || MUL
        operand_a_r = {1'b0, opcode_ra_operand_i[31:0]};
end

// 操作数B符号扩展：
// MULH：rs2视为有符号数，符号扩展
// MULHSU/MULHU/MUL：rs2视为无符号数，高位补0
always @ *
begin
    if ((opcode_opcode_i & `INST_MULHSU_MASK) == `INST_MULHSU)
        operand_b_r = {1'b0, opcode_rb_operand_i[31:0]};
    else if ((opcode_opcode_i & `INST_MULH_MASK) == `INST_MULH)
        operand_b_r = {opcode_rb_operand_i[31], opcode_rb_operand_i[31:0]};
    else // MULHU || MUL
        operand_b_r = {1'b0, opcode_rb_operand_i[31:0]};
end


// E1级流水线寄存器：锁存操作数及高/低位选择标志
// hold_i=1时保持当前值（流水线暂停）
// 无乘法指令时清零，防止无效运算传播
always @(posedge clk_i or posedge rst_i)
if (rst_i)
begin
    operand_a_e1_q <= 33'b0;
    operand_b_e1_q <= 33'b0;
    mulhi_sel_e1_q <= 1'b0;
end
else if (hold_i)
    ;
else if (opcode_valid_i && mult_inst_w)
begin
    operand_a_e1_q <= operand_a_r;
    operand_b_e1_q <= operand_b_r;
    // MUL取低32位(sel=0)，MULH/MULHSU/MULHU取高32位(sel=1)
    mulhi_sel_e1_q <= ~((opcode_opcode_i & `INST_MUL_MASK) == `INST_MUL);
end
else
begin
    operand_a_e1_q <= 33'b0;
    operand_b_e1_q <= 33'b0;
    mulhi_sel_e1_q <= 1'b0;
end

// 65位符号扩展乘法：将33位操作数各符号扩展至65位后相乘，得到65位积
assign mult_result_w = {{ 32 {operand_a_e1_q[32]}}, operand_a_e1_q}*{{ 32 {operand_b_e1_q[32]}}, operand_b_e1_q};

// 根据 mulhi_sel_e1_q 选取高32位或低32位
always @ *
begin
    result_r = mulhi_sel_e1_q ? mult_result_w[63:32] : mult_result_w[31:0];
end

// E2级流水线寄存器：将乘法结果打一拍
always @(posedge clk_i or posedge rst_i)
if (rst_i)
    result_e2_q <= 32'b0;
else if (~hold_i)
    result_e2_q <= result_r;

// E3级流水线寄存器（仅在 MULT_STAGES=3 时有效，增加额外一级流水）
always @(posedge clk_i or posedge rst_i)
if (rst_i)
    result_e3_q <= 32'b0;
else if (~hold_i)
    result_e3_q <= result_e2_q;

// 根据流水线级数选择最终写回结果
assign writeback_value_o  = (MULT_STAGES == 3) ? result_e3_q : result_e2_q;


endmodule
