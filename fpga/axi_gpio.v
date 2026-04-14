//-----------------------------------------------------------------
// AXI GPIO - 简单 AXI4-Lite GPIO 外设
//-----------------------------------------------------------------
// 寄存器映射（基于 bit[2] 区分）：
//   offset 0x0 : GPIO_OUT (R/W) — 写入值驱动 LED
//   offset 0x4 : GPIO_IN  (R)   — 读取拨码开关状态
//
// 注意：本模块假设 AXI Master 的 bready/rready 始终为 1
//       （riscv_tcm_top 的 dport_axi 确实如此）
//-----------------------------------------------------------------
module axi_gpio
(
    input  wire        clk_i,
    input  wire        rst_i,

    // AXI Slave 接口
    input  wire        awvalid_i,
    input  wire [31:0] awaddr_i,
    output wire        awready_o,

    input  wire        wvalid_i,
    input  wire [31:0] wdata_i,
    input  wire [3:0]  wstrb_i,
    output wire        wready_o,

    output reg         bvalid_o,
    output wire [1:0]  bresp_o,
    input  wire        bready_i,

    input  wire        arvalid_i,
    input  wire [31:0] araddr_i,
    output wire        arready_o,

    output reg         rvalid_o,
    output reg  [31:0] rdata_o,
    output wire [1:0]  rresp_o,
    input  wire        rready_i,

    // GPIO 端口
    output reg  [31:0] gpio_out_o,
    input  wire [31:0] gpio_in_i
);

// ============================================================
// 写通道
// ============================================================
// dport_axi 同时发送 AW 和 W，且 bready 始终为 1
// 所以可以简化为：收到写请求 → 锁存数据 → 下一拍给 bvalid
assign awready_o = 1'b1;
assign wready_o  = 1'b1;
assign bresp_o   = 2'b00; // OKAY

// 捕获写数据到 GPIO 输出寄存器
always @(posedge clk_i) begin
    if (rst_i) begin
        gpio_out_o <= 32'h0;
    end else if (awvalid_i && wvalid_i) begin
        // 按字节写使能更新
        if (wstrb_i[0]) gpio_out_o[ 7: 0] <= wdata_i[ 7: 0];
        if (wstrb_i[1]) gpio_out_o[15: 8] <= wdata_i[15: 8];
        if (wstrb_i[2]) gpio_out_o[23:16] <= wdata_i[23:16];
        if (wstrb_i[3]) gpio_out_o[31:24] <= wdata_i[31:24];
    end
end

// 写响应（1 拍延迟）
always @(posedge clk_i) begin
    if (rst_i)
        bvalid_o <= 1'b0;
    else
        bvalid_o <= awvalid_i && wvalid_i;
end

// ============================================================
// 读通道
// ============================================================
assign arready_o = 1'b1;
assign rresp_o   = 2'b00; // OKAY

always @(posedge clk_i) begin
    if (rst_i) begin
        rvalid_o <= 1'b0;
        rdata_o  <= 32'h0;
    end else begin
        rvalid_o <= arvalid_i;
        // addr bit[2] 区分寄存器：0=GPIO_OUT, 4=GPIO_IN
        if (arvalid_i)
            rdata_o <= araddr_i[2] ? gpio_in_i : gpio_out_o;
    end
end

endmodule
