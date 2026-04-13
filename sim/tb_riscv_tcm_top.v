 `timescale 1ns / 1ps

module tb_riscv_tcm_top;

// ============================================================
// 参数
// ============================================================
parameter CLK_PERIOD = 20;   // 50 MHz

// ============================================================
// 信号声明
// ============================================================
reg         clk;
reg         rst;
reg         rst_cpu;
reg  [31:0] intr;

// AXI Master (CPU → 外设) — 本 TB 中悬空/给默认响应
wire        axi_i_awvalid;
wire [31:0] axi_i_awaddr;
reg         axi_i_awready;
wire        axi_i_wvalid;
wire [31:0] axi_i_wdata;
wire [3:0]  axi_i_wstrb;
reg         axi_i_wready;
reg         axi_i_bvalid;
reg  [1:0]  axi_i_bresp;
wire        axi_i_bready;
wire        axi_i_arvalid;
wire [31:0] axi_i_araddr;
reg         axi_i_arready;
reg         axi_i_rvalid;
reg  [31:0] axi_i_rdata;
reg  [1:0]  axi_i_rresp;
wire        axi_i_rready;

// AXI Slave (外部 → TCM) — 用于加载程序
reg         axi_t_awvalid;
reg  [31:0] axi_t_awaddr;
reg  [3:0]  axi_t_awid;
reg  [7:0]  axi_t_awlen;
reg  [1:0]  axi_t_awburst;
wire        axi_t_awready;
reg         axi_t_wvalid;
reg  [31:0] axi_t_wdata;
reg  [3:0]  axi_t_wstrb;
reg         axi_t_wlast;
wire        axi_t_wready;
wire        axi_t_bvalid;
wire [1:0]  axi_t_bresp;
wire [3:0]  axi_t_bid;
reg         axi_t_bready;
reg         axi_t_arvalid;
reg  [31:0] axi_t_araddr;
reg  [3:0]  axi_t_arid;
reg  [7:0]  axi_t_arlen;
reg  [1:0]  axi_t_arburst;
wire        axi_t_arready;
wire        axi_t_rvalid;
wire [31:0] axi_t_rdata;
wire [1:0]  axi_t_rresp;
wire [3:0]  axi_t_rid;
wire        axi_t_rlast;
reg         axi_t_rready;

// ============================================================
// 时钟产生
// ============================================================
initial clk = 0;
always #(CLK_PERIOD/2) clk = ~clk;

// ============================================================
// DUT 例化
// ============================================================
riscv_tcm_top u_dut (
    .clk_i              (clk),
    .rst_i              (rst),
    .rst_cpu_i          (rst_cpu),
    .axi_i_awvalid_o    (axi_i_awvalid),
    .axi_i_awaddr_o     (axi_i_awaddr),
    .axi_i_awready_i    (axi_i_awready),
    .axi_i_wvalid_o     (axi_i_wvalid),
    .axi_i_wdata_o      (axi_i_wdata),
    .axi_i_wstrb_o      (axi_i_wstrb),
    .axi_i_wready_i     (axi_i_wready),
    .axi_i_bvalid_i     (axi_i_bvalid),
    .axi_i_bresp_i      (axi_i_bresp),
    .axi_i_bready_o     (axi_i_bready),
    .axi_i_arvalid_o    (axi_i_arvalid),
    .axi_i_araddr_o     (axi_i_araddr),
    .axi_i_arready_i    (axi_i_arready),
    .axi_i_rvalid_i     (axi_i_rvalid),
    .axi_i_rdata_i      (axi_i_rdata),
    .axi_i_rresp_i      (axi_i_rresp),
    .axi_i_rready_o     (axi_i_rready),
    .axi_t_awvalid_i    (axi_t_awvalid),
    .axi_t_awaddr_i     (axi_t_awaddr),
    .axi_t_awid_i       (axi_t_awid),
    .axi_t_awlen_i      (axi_t_awlen),
    .axi_t_awburst_i    (axi_t_awburst),
    .axi_t_awready_o    (axi_t_awready),
    .axi_t_wvalid_i     (axi_t_wvalid),
    .axi_t_wdata_i      (axi_t_wdata),
    .axi_t_wstrb_i      (axi_t_wstrb),
    .axi_t_wlast_i      (axi_t_wlast),
    .axi_t_wready_o     (axi_t_wready),
    .axi_t_bvalid_o     (axi_t_bvalid),
    .axi_t_bresp_o      (axi_t_bresp),
    .axi_t_bid_o        (axi_t_bid),
    .axi_t_bready_i     (axi_t_bready),
    .axi_t_arvalid_i    (axi_t_arvalid),
    .axi_t_araddr_i     (axi_t_araddr),
    .axi_t_arid_i       (axi_t_arid),
    .axi_t_arlen_i      (axi_t_arlen),
    .axi_t_arburst_i    (axi_t_arburst),
    .axi_t_arready_o    (axi_t_arready),
    .axi_t_rvalid_o     (axi_t_rvalid),
    .axi_t_rdata_o      (axi_t_rdata),
    .axi_t_rresp_o      (axi_t_rresp),
    .axi_t_rid_o        (axi_t_rid),
    .axi_t_rlast_o      (axi_t_rlast),
    .axi_t_rready_i     (axi_t_rready),
    .intr_i              (intr)
);

// ============================================================
// AXI Master 端简易应答（防止 CPU 外部访问挂死）
// ============================================================
always @(posedge clk) begin
    if (rst) begin
        axi_i_awready <= 1'b0;
        axi_i_wready  <= 1'b0;
        axi_i_bvalid  <= 1'b0;
        axi_i_bresp   <= 2'b00;
        axi_i_arready <= 1'b0;
        axi_i_rvalid  <= 1'b0;
        axi_i_rdata   <= 32'h0;
        axi_i_rresp   <= 2'b00;
    end else begin
        // 写通道：始终接受
        axi_i_awready <= 1'b1;
        axi_i_wready  <= 1'b1;
        // 写响应：跟随写数据
        axi_i_bvalid  <= axi_i_wvalid & axi_i_wready;
        axi_i_bresp   <= 2'b00; // OKAY
        // 读通道：始终接受，返回 0
        axi_i_arready <= 1'b1;
        axi_i_rvalid  <= axi_i_arvalid & axi_i_arready;
        axi_i_rdata   <= 32'hDEADBEEF;
        axi_i_rresp   <= 2'b00; // OKAY
    end
end

// ============================================================
// 测试主程序
// ============================================================
integer i;
reg [31:0] test_program [0:15]; // 最多 16 条指令

initial begin
    // ------- 初始化所有信号 -------
    rst         = 1;
    rst_cpu     = 1;
    intr        = 32'h0;
    axi_t_awvalid = 0;
    axi_t_awaddr  = 0;
    axi_t_awid    = 0;
    axi_t_awlen   = 0;
    axi_t_awburst = 0;
    axi_t_wvalid  = 0;
    axi_t_wdata   = 0;
    axi_t_wstrb   = 0;
    axi_t_wlast   = 0;
    axi_t_bready  = 1;
    axi_t_arvalid = 0;
    axi_t_araddr  = 0;
    axi_t_arid    = 0;
    axi_t_arlen   = 0;
    axi_t_arburst = 0;
    axi_t_rready  = 1;

    // ------- 测试程序（机器码） -------
    // 综合测试：算术、逻辑、移位、比较、分支、存储/加载
    //
    // [0]  0x2000: addi x1, x0, 10       x1 = 10
    // [1]  0x2004: addi x2, x0, 20       x2 = 20
    // [2]  0x2008: add  x3, x1, x2       x3 = 30
    // [3]  0x200C: sub  x4, x2, x1       x4 = 10
    // [4]  0x2010: and  x5, x3, x4       x5 = 30 & 10 = 10
    // [5]  0x2014: or   x6, x3, x4       x6 = 30 | 10 = 30
    // [6]  0x2018: xor  x7, x3, x4       x7 = 30 ^ 10 = 20
    // [7]  0x201C: slli x8, x1, 3        x8 = 10 << 3 = 80
    // [8]  0x2020: srli x9, x8, 1        x9 = 80 >> 1 = 40
    // [9]  0x2024: slti x10, x1, 15      x10 = (10<15) = 1
    // [10] 0x2028: slti x11, x1, 5       x11 = (10<5)  = 0
    // [11] 0x202C: bne  x10, x0, +8      if x10!=0, jump to 0x2034 (skip [12])
    // [12] 0x2030: addi x12, x0, 0xFF    *** 应被跳过，x12 不应 = 255 ***
    // [13] 0x2034: sw   x3, 0(x2)        mem[20] = 30
    // [14] 0x2038: lw   x12, 0(x2)       x12 = mem[20] = 30
    // [15] 0x203C: jal  x0, 0            无限循环

    test_program[0]  = 32'h00A00093; // addi x1, x0, 10
    test_program[1]  = 32'h01400113; // addi x2, x0, 20
    test_program[2]  = 32'h002081B3; // add  x3, x1, x2
    test_program[3]  = 32'h40110233; // sub  x4, x2, x1
    test_program[4]  = 32'h0041F2B3; // and  x5, x3, x4
    test_program[5]  = 32'h0041E333; // or   x6, x3, x4
    test_program[6]  = 32'h0041C3B3; // xor  x7, x3, x4
    test_program[7]  = 32'h00309413; // slli x8, x1, 3
    test_program[8]  = 32'h00145493; // srli x9, x8, 1
    test_program[9]  = 32'h00F0A513; // slti x10, x1, 15   (10<15 → x10=1)
    test_program[10] = 32'h0050A593; // slti x11, x1, 5    (10<5  → x11=0)
    test_program[11] = 32'h00051463; // bne  x10, x0, +8
    test_program[12] = 32'h0FF00613; // addi x12, x0, 255 (should be skipped)
    test_program[13] = 32'h00312023; // sw   x3, 0(x2)
    test_program[14] = 32'h00012603; // lw   x12, 0(x2)
    test_program[15] = 32'h0000006F; // jal  x0, 0 (infinite loop)

    // ------- 复位阶段 -------
    #(CLK_PERIOD * 10);
    rst = 0;         // 释放系统复位（TCM 可写入）
    #(CLK_PERIOD * 5);

    // ------- 通过 AXI Slave 写入程序到 TCM -------
    // BOOT_VECTOR = 0x2000，程序必须写到该地址
    // 单拍写入（awlen=0），每次写 4 字节
    for (i = 0; i < 16; i = i + 1) begin
        axi_write(32'h0000_2000 + i * 4, test_program[i]);
    end

    $display("[TB] Program loaded into TCM");

    // ------- 释放 CPU 复位，开始执行 -------
    #(CLK_PERIOD * 5);
    rst_cpu = 0;
    $display("[TB] CPU reset released, execution starts");

    // ------- 运行一段时间观察波形 -------
    #(CLK_PERIOD * 500);

    // ------- 检查结果 -------
    $display("======== 仿真结果 ========");
    $display("[TB] x1  = %0d (expected 10)",  u_dut.u_core.u_issue.u_regfile.REGFILE.reg_r1_q);
    $display("[TB] x2  = %0d (expected 20)",  u_dut.u_core.u_issue.u_regfile.REGFILE.reg_r2_q);
    $display("[TB] x3  = %0d (expected 30)",  u_dut.u_core.u_issue.u_regfile.REGFILE.reg_r3_q);
    $display("[TB] x4  = %0d (expected 10)",  u_dut.u_core.u_issue.u_regfile.REGFILE.reg_r4_q);
    $display("[TB] x5  = %0d (expected 10)",  u_dut.u_core.u_issue.u_regfile.REGFILE.reg_r5_q);
    $display("[TB] x6  = %0d (expected 30)",  u_dut.u_core.u_issue.u_regfile.REGFILE.reg_r6_q);
    $display("[TB] x7  = %0d (expected 20)",  u_dut.u_core.u_issue.u_regfile.REGFILE.reg_r7_q);
    $display("[TB] x8  = %0d (expected 80)",  u_dut.u_core.u_issue.u_regfile.REGFILE.reg_r8_q);
    $display("[TB] x9  = %0d (expected 40)",  u_dut.u_core.u_issue.u_regfile.REGFILE.reg_r9_q);
    $display("[TB] x10 = %0d (expected 1)",   u_dut.u_core.u_issue.u_regfile.REGFILE.reg_r10_q);
    $display("[TB] x11 = %0d (expected 0)",   u_dut.u_core.u_issue.u_regfile.REGFILE.reg_r11_q);
    $display("[TB] x12 = %0d (expected 30)",  u_dut.u_core.u_issue.u_regfile.REGFILE.reg_r12_q);
    $display("==========================");

    $display("[TB] Simulation finished");
    $finish;
end

// ============================================================
// AXI 单拍写入任务（通过 axi_t 向 TCM 写数据）
// ============================================================
task axi_write(input [31:0] addr, input [31:0] data);
begin
    @(posedge clk);
    // 写地址通道
    axi_t_awvalid = 1;
    axi_t_awaddr  = addr;
    axi_t_awid    = 4'h0;
    axi_t_awlen   = 8'h00;  // 单拍
    axi_t_awburst = 2'b01;  // INCR
    // 写数据通道
    axi_t_wvalid  = 1;
    axi_t_wdata   = data;
    axi_t_wstrb   = 4'hF;
    axi_t_wlast   = 1;

    // 等待两个通道都握手
    @(posedge clk);
    while (!axi_t_awready) @(posedge clk);
    axi_t_awvalid = 0;
    while (!axi_t_wready) @(posedge clk);
    axi_t_wvalid  = 0;
    axi_t_wlast   = 0;

    // 等待写响应
    while (!axi_t_bvalid) @(posedge clk);
    @(posedge clk);
end
endtask

// ============================================================
// 波形输出（VCD，用于非 Vivado 仿真器查看）
// ============================================================
initial begin
    $dumpfile("riscv_tcm_sim.vcd");
    $dumpvars(0, tb_riscv_tcm_top);
end

endmodule
