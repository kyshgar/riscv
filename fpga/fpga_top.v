//-----------------------------------------------------------------
// FPGA Top Level - RISC-V TCM CPU + GPIO
//-----------------------------------------------------------------
// 板子信息：
//   时钟：SIT9102 200MHz 差分晶振 → CLKP (LVCMOS18)
//   按键：按下=0，松开=1（低有效）
//   LED ：高电平点亮
//   拨码：上=0（断电），下=1（通电）
//
// 地址映射：
//   0x00000000 ~ 0x0000FFFF : TCM (64KB)
//   0x10000000              : GPIO_OUT (写→LED)
//   0x10000004              : GPIO_IN  (读→拨码开关)
//-----------------------------------------------------------------
module fpga_top
(
    // 时钟（200MHz 晶振，仅使用 P 端）
    input  wire CLKP,

    // 按键复位（KEY1，按下=0）
    input  wire KEY1,

    // LED 输出（高电平点亮）
    output wire LED1,
    output wire LED2,
    output wire LED3,
    output wire LED4,
    output wire LED5,
    output wire LED6,
    output wire LED7,
    output wire LED8,

    // 拨码开关输入（下拨=1）
    input  wire SW1,
    input  wire SW2,
    input  wire SW3,
    input  wire SW4,
    input  wire SW5,
    input  wire SW6,
    input  wire SW7,
    input  wire SW8
);

// ============================================================
// PLL: 200MHz → 50MHz
// ============================================================
// Xilinx 7 系列 PLLE2_BASE 原语
// VCO = 200MHz × 5 = 1000MHz (范围 800~1600MHz)
// CLKOUT0 = 1000MHz / 20 = 50MHz

wire clk_ibuf;
wire clk_50m_unbuf;
wire clk_50m;
wire pll_locked;
wire pll_feedback;

IBUF u_clk_ibuf (
    .I  (CLKP),
    .O  (clk_ibuf)
);

PLLE2_BASE #(
    .CLKIN1_PERIOD  (5.000),    // 200 MHz → 5ns
    .CLKFBOUT_MULT  (5),        // VCO = 200 × 5 = 1000 MHz
    .DIVCLK_DIVIDE  (1),
    .CLKOUT0_DIVIDE (20)        // 1000 / 20 = 50 MHz
) u_pll (
    .CLKIN1     (clk_ibuf),
    .CLKFBOUT   (pll_feedback),
    .CLKFBIN    (pll_feedback),
    .CLKOUT0    (clk_50m_unbuf),
    .CLKOUT1    (),
    .CLKOUT2    (),
    .CLKOUT3    (),
    .CLKOUT4    (),
    .CLKOUT5    (),
    .LOCKED     (pll_locked),
    .PWRDWN     (1'b0),
    .RST        (1'b0)
);

BUFG u_clk_bufg (
    .I  (clk_50m_unbuf),
    .O  (clk_50m)
);

wire clk = clk_50m;

// ============================================================
// 复位逻辑
// ============================================================
// KEY1: 按下=0（有效低），松开=1
// rst 有效条件：按键按下 OR PLL 未锁定
wire rst_btn = ~KEY1 | ~pll_locked;

// 复位同步器（防亚稳态）
reg [3:0] rst_sync_q;
always @(posedge clk or posedge rst_btn) begin
    if (rst_btn)
        rst_sync_q <= 4'hF;
    else
        rst_sync_q <= {rst_sync_q[2:0], 1'b0};
end
wire rst = rst_sync_q[3];  // 同步后的系统复位

// CPU 复位：系统复位释放后延迟一段时间再释放
reg [7:0] cpu_rst_cnt_q;
reg       rst_cpu_q;

always @(posedge clk) begin
    if (rst) begin
        cpu_rst_cnt_q <= 8'h0;
        rst_cpu_q     <= 1'b1;
    end else if (cpu_rst_cnt_q < 8'hFF) begin
        cpu_rst_cnt_q <= cpu_rst_cnt_q + 1'b1;
    end else begin
        rst_cpu_q <= 1'b0;
    end
end

// ============================================================
// GPIO 信号
// ============================================================
wire [31:0] gpio_out_w;
wire [31:0] gpio_in_w;

// LED: 高电平点亮，直接连接
assign {LED8, LED7, LED6, LED5, LED4, LED3, LED2, LED1} = gpio_out_w[7:0];

// 拨码开关: 下拨=1，直接连接
assign gpio_in_w = {24'h0, SW8, SW7, SW6, SW5, SW4, SW3, SW2, SW1};

// ============================================================
// CPU AXI Master 信号
// ============================================================
wire        axi_m_awvalid;
wire [31:0] axi_m_awaddr;
wire        axi_m_awready;
wire        axi_m_wvalid;
wire [31:0] axi_m_wdata;
wire [3:0]  axi_m_wstrb;
wire        axi_m_wready;
wire        axi_m_bvalid;
wire [1:0]  axi_m_bresp;
wire        axi_m_bready;
wire        axi_m_arvalid;
wire [31:0] axi_m_araddr;
wire        axi_m_arready;
wire        axi_m_rvalid;
wire [31:0] axi_m_rdata;
wire [1:0]  axi_m_rresp;
wire        axi_m_rready;

// ============================================================
// CPU 例化
// ============================================================
riscv_tcm_top
#(
    .BOOT_VECTOR    (32'h0000_2000),
    .TCM_MEM_BASE   (32'h0000_0000)
)
u_cpu
(
    .clk_i              (clk),
    .rst_i              (rst),
    .rst_cpu_i          (rst_cpu_q),

    // AXI Master（CPU → 外设）→ 连接 GPIO
    .axi_i_awvalid_o    (axi_m_awvalid),
    .axi_i_awaddr_o     (axi_m_awaddr),
    .axi_i_awready_i    (axi_m_awready),
    .axi_i_wvalid_o     (axi_m_wvalid),
    .axi_i_wdata_o      (axi_m_wdata),
    .axi_i_wstrb_o      (axi_m_wstrb),
    .axi_i_wready_i     (axi_m_wready),
    .axi_i_bvalid_i     (axi_m_bvalid),
    .axi_i_bresp_i      (axi_m_bresp),
    .axi_i_bready_o     (axi_m_bready),
    .axi_i_arvalid_o    (axi_m_arvalid),
    .axi_i_araddr_o     (axi_m_araddr),
    .axi_i_arready_i    (axi_m_arready),
    .axi_i_rvalid_i     (axi_m_rvalid),
    .axi_i_rdata_i      (axi_m_rdata),
    .axi_i_rresp_i      (axi_m_rresp),
    .axi_i_rready_o     (axi_m_rready),

    // AXI Slave（外部 → TCM）— 上板不需要外部加载，悬空
    .axi_t_awvalid_i    (1'b0),
    .axi_t_awaddr_i     (32'h0),
    .axi_t_awid_i       (4'h0),
    .axi_t_awlen_i      (8'h0),
    .axi_t_awburst_i    (2'b00),
    .axi_t_wvalid_i     (1'b0),
    .axi_t_wdata_i      (32'h0),
    .axi_t_wstrb_i      (4'h0),
    .axi_t_wlast_i      (1'b0),
    .axi_t_bready_i     (1'b1),
    .axi_t_arvalid_i    (1'b0),
    .axi_t_araddr_i     (32'h0),
    .axi_t_arid_i       (4'h0),
    .axi_t_arlen_i      (8'h0),
    .axi_t_arburst_i    (2'b00),
    .axi_t_rready_i     (1'b1),

    // 中断 — 不使用
    .intr_i             (32'h0)
);

// ============================================================
// AXI GPIO 外设
// ============================================================
axi_gpio u_gpio
(
    .clk_i      (clk),
    .rst_i      (rst),

    .awvalid_i  (axi_m_awvalid),
    .awaddr_i   (axi_m_awaddr),
    .awready_o  (axi_m_awready),
    .wvalid_i   (axi_m_wvalid),
    .wdata_i    (axi_m_wdata),
    .wstrb_i    (axi_m_wstrb),
    .wready_o   (axi_m_wready),
    .bvalid_o   (axi_m_bvalid),
    .bresp_o    (axi_m_bresp),
    .bready_i   (axi_m_bready),
    .arvalid_i  (axi_m_arvalid),
    .araddr_i   (axi_m_araddr),
    .arready_o  (axi_m_arready),
    .rvalid_o   (axi_m_rvalid),
    .rdata_o    (axi_m_rdata),
    .rresp_o    (axi_m_rresp),
    .rready_i   (axi_m_rready),

    .gpio_out_o (gpio_out_w),
    .gpio_in_i  (gpio_in_w)
);

endmodule
