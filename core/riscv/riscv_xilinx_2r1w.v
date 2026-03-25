// ============================================================
// 模块: riscv_xilinx_2r1w
// 功能: Xilinx FPGA 专用寄存器堆（2个异步读端口 + 1个同步写端口）
//       使用 Xilinx 原语 RAM16X1D 实现，每个原语存储16×1bit
//       32个寄存器（x0-x31）分两组（0-15 和 16-31），
//       每组各32位宽，共需 2×32×2 = 128 个 RAM16X1D 实例
//       读操作：组合逻辑异步读（零延迟）
//       写操作：时钟上升沿同步写
//       特殊处理：x0（地址0）永远读出0（硬件零寄存器）
// ============================================================
//-----------------------------------------------------------------
// Module - Xilinx register file (2 async read, 1 write port)
//-----------------------------------------------------------------
module riscv_xilinx_2r1w
(
    // Inputs
     input           clk_i       // 系统时钟（写端口使用）
    ,input           rst_i       // 复位（本模块未使用，接口保留）
    ,input  [  4:0]  rd0_i       // 写端口目标寄存器索引（x0-x31）
    ,input  [ 31:0]  rd0_value_i // 写端口数据输入
    ,input  [  4:0]  ra_i        // 读端口A寄存器索引（rs1，x0-x31）
    ,input  [  4:0]  rb_i        // 读端口B寄存器索引（rs2，x0-x31）

    // Outputs
    ,output [ 31:0]  ra_value_o  // 读端口A数据输出（rs1值）
    ,output [ 31:0]  rb_value_o  // 读端口B数据输出（rs2值）
);


//-----------------------------------------------------------------
// Registers / Wires（内部信号声明）
//-----------------------------------------------------------------
wire [31:0]     reg_rs1_w;      // 读端口A最终数据（组合逻辑选择 bank A 或 bank B）
wire [31:0]     reg_rs2_w;      // 读端口B最终数据
wire [31:0]     rs1_0_15_w;     // Bank A（寄存器 0-15）的 rs1 读出数据
wire [31:0]     rs1_16_31_w;    // Bank B（寄存器 16-31）的 rs1 读出数据
wire [31:0]     rs2_0_15_w;     // Bank A 的 rs2 读出数据
wire [31:0]     rs2_16_31_w;    // Bank B 的 rs2 读出数据
wire            write_enable_w; // 全局写使能（x0不可写，强制禁止）
wire            write_banka_w;  // Bank A 写使能（目标寄存器在 0-15 时有效）
wire            write_bankb_w;  // Bank B 写使能（目标寄存器在 16-31 时有效）

//-----------------------------------------------------------------
// Register File (using RAM16X1D)
// RAM16X1D：Xilinx LUT 实现的 16×1bit 双端口同步写/异步读 RAM
//   WCLK: 写时钟   WE: 写使能   A[3:0]: 写地址   D: 写数据
//   DPRA[3:0]: 读地址（双端口独立读地址）  DPO: 读数据输出
//-----------------------------------------------------------------
genvar i;

// Registers 0 - 15（Bank A：rd0_i[3:0]为地址，rd0_i[4]=0时写入此组）
generate
for (i=0;i<32;i=i+1)
begin : reg_loop1
    // 每位宽度实例化两个 RAM16X1D：一个用于 rs1 读端口，一个用于 rs2 读端口
    RAM16X1D reg_bit1a(.WCLK(clk_i), .WE(write_banka_w), .A0(rd0_i[0]), .A1(rd0_i[1]), .A2(rd0_i[2]), .A3(rd0_i[3]), .D(rd0_value_i[i]), .DPRA0(ra_i[0]), .DPRA1(ra_i[1]), .DPRA2(ra_i[2]), .DPRA3(ra_i[3]), .DPO(rs1_0_15_w[i]), .SPO(/* open */));
    RAM16X1D reg_bit2a(.WCLK(clk_i), .WE(write_banka_w), .A0(rd0_i[0]), .A1(rd0_i[1]), .A2(rd0_i[2]), .A3(rd0_i[3]), .D(rd0_value_i[i]), .DPRA0(rb_i[0]), .DPRA1(rb_i[1]), .DPRA2(rb_i[2]), .DPRA3(rb_i[3]), .DPO(rs2_0_15_w[i]), .SPO(/* open */));
end
endgenerate

// Registers 16 - 31（Bank B：rd0_i[3:0]为地址，rd0_i[4]=1时写入此组）
generate
for (i=0;i<32;i=i+1)
begin : reg_loop2
    RAM16X1D reg_bit1b(.WCLK(clk_i), .WE(write_bankb_w), .A0(rd0_i[0]), .A1(rd0_i[1]), .A2(rd0_i[2]), .A3(rd0_i[3]), .D(rd0_value_i[i]), .DPRA0(ra_i[0]), .DPRA1(ra_i[1]), .DPRA2(ra_i[2]), .DPRA3(ra_i[3]), .DPO(rs1_16_31_w[i]), .SPO(/* open */));
    RAM16X1D reg_bit2b(.WCLK(clk_i), .WE(write_bankb_w), .A0(rd0_i[0]), .A1(rd0_i[1]), .A2(rd0_i[2]), .A3(rd0_i[3]), .D(rd0_value_i[i]), .DPRA0(rb_i[0]), .DPRA1(rb_i[1]), .DPRA2(rb_i[2]), .DPRA3(rb_i[3]), .DPO(rs2_16_31_w[i]), .SPO(/* open */));
end
endgenerate

//-----------------------------------------------------------------
// Combinatorial Assignments（组合逻辑：Bank选择与写使能控制）
//-----------------------------------------------------------------
// 根据地址第4位（bit[4]）选择从哪个Bank读取：0=寄存器0-15，1=寄存器16-31
assign reg_rs1_w       = (ra_i[4] == 1'b0) ? rs1_0_15_w : rs1_16_31_w;
assign reg_rs2_w       = (rb_i[4] == 1'b0) ? rs2_0_15_w : rs2_16_31_w;

// x0（地址0）硬件为零寄存器，不允许写入
assign write_enable_w = (rd0_i != 5'b00000);

// 按地址最高位分配写使能到对应Bank
assign write_banka_w  = (write_enable_w & (~rd0_i[4])); // rd0_i[4]=0 写 Bank A（0-15）
assign write_bankb_w  = (write_enable_w & rd0_i[4]);    // rd0_i[4]=1 写 Bank B（16-31）

reg [31:0] ra_value_r; // 读端口A最终输出（含x0零处理）
reg [31:0] rb_value_r; // 读端口B最终输出（含x0零处理）

// 寄存器读端口：x0始终返回0（RISC-V规范：x0硬连线为0）
always @ *
begin
    if (ra_i == 5'b00000)
        ra_value_r = 32'h00000000; // x0永远为0
    else
        ra_value_r = reg_rs1_w;

    if (rb_i == 5'b00000)
        rb_value_r = 32'h00000000; // x0永远为0
    else
        rb_value_r = reg_rs2_w;
end

assign ra_value_o = ra_value_r;
assign rb_value_o = rb_value_r;

endmodule

//-------------------------------------------------------------
// RAM16X1D: Verilator target RAM16X1D model
// 说明：这是 Xilinx RAM16X1D 原语的 Verilator 仿真模型
//       仅在 verilator 仿真时编译，实际 FPGA 综合使用厂商原语
//       RAM16X1D 是一个 16×1bit 的双端口 LUT RAM：
//         - 写端口：WCLK 时钟上升沿、WE 使能、A[3:0] 地址、D 数据
//         - 读端口（SPO）：与写地址相同，同步读
//         - 读端口（DPO）：DPRA[3:0] 独立地址，异步读
//-------------------------------------------------------------
`ifdef verilator
module RAM16X1D (DPO, SPO, A0, A1, A2, A3, D, DPRA0, DPRA1, DPRA2, DPRA3, WCLK, WE);

    parameter INIT = 16'h0000; // RAM初始化值（仿真时所有位初始为0）

    output DPO, SPO; // DPO：双端口异步读输出；SPO：单端口（写地址）同步读输出

    input  A0, A1, A2, A3, D, DPRA0, DPRA1, DPRA2, DPRA3, WCLK, WE;
    // A[3:0]：写端口地址   D：写数据   WCLK：写时钟   WE：写使能
    // DPRA[3:0]：双端口读地址（独立于写地址）

    reg  [15:0] mem; // 16×1bit 存储阵列
    wire [3:0] adr;  // 写地址组合

    assign adr = {A3, A2, A1, A0};                        // 拼合写端口4位地址
    assign SPO = mem[adr];                                  // 单端口读：与写地址相同
    assign DPO = mem[{DPRA3, DPRA2, DPRA1, DPRA0}];       // 双端口异步读

    initial 
        mem = INIT; // 仿真初始化

    always @(posedge WCLK) 
        if (WE == 1'b1)
            mem[adr] <= D; // 写时钟上升沿且写使能时写入数据

endmodule
`endif
