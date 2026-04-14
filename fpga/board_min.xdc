# ==========================================
# RISC-V FPGA 最小约束文件
# 板载晶振：SIT9102 200MHz
# ==========================================

# ---- 时钟 (200MHz) ----
set_property -dict {PACKAGE_PIN AD12 IOSTANDARD LVCMOS18} [get_ports CLKP]
create_clock -period 5.000 -name sys_clk [get_ports CLKP]

# ---- PLL 输出时钟约束（50MHz，自动推导） ----
# Vivado 会根据 PLLE2_BASE 参数自动生成 generated clock
# 如果需要手动指定：
# create_generated_clock -name clk_50m -source [get_ports CLKP] -divide_by 4 [get_pins u_pll/CLKOUT0]

# ---- 复位按键 (KEY1，按下=0) ----
set_property -dict {PACKAGE_PIN D11 IOSTANDARD LVCMOS18} [get_ports KEY1]

# ---- LED 输出 (LED1-LED8，高电平点亮) ----
set_property -dict {PACKAGE_PIN G24 IOSTANDARD LVCMOS18} [get_ports LED1]
set_property -dict {PACKAGE_PIN E24 IOSTANDARD LVCMOS18} [get_ports LED2]
set_property -dict {PACKAGE_PIN C24 IOSTANDARD LVCMOS18} [get_ports LED3]
set_property -dict {PACKAGE_PIN E25 IOSTANDARD LVCMOS18} [get_ports LED4]
set_property -dict {PACKAGE_PIN C26 IOSTANDARD LVCMOS18} [get_ports LED5]
set_property -dict {PACKAGE_PIN F26 IOSTANDARD LVCMOS18} [get_ports LED6]
set_property -dict {PACKAGE_PIN G25 IOSTANDARD LVCMOS18} [get_ports LED7]
set_property -dict {PACKAGE_PIN E29 IOSTANDARD LVCMOS18} [get_ports LED8]

# ---- 拨码开关 (SW1-SW8，下拨=1) ----
set_property -dict {PACKAGE_PIN T21 IOSTANDARD LVCMOS18} [get_ports SW1]
set_property -dict {PACKAGE_PIN U22 IOSTANDARD LVCMOS18} [get_ports SW2]
set_property -dict {PACKAGE_PIN T22 IOSTANDARD LVCMOS18} [get_ports SW3]
set_property -dict {PACKAGE_PIN W23 IOSTANDARD LVCMOS18} [get_ports SW4]
set_property -dict {PACKAGE_PIN T23 IOSTANDARD LVCMOS18} [get_ports SW5]
set_property -dict {PACKAGE_PIN P26 IOSTANDARD LVCMOS18} [get_ports SW6]
set_property -dict {PACKAGE_PIN P27 IOSTANDARD LVCMOS18} [get_ports SW7]
set_property -dict {PACKAGE_PIN P28 IOSTANDARD LVCMOS18} [get_ports SW8]

# ---- 异步输入的时序例外 ----
set_false_path -from [get_ports KEY1]
set_false_path -from [get_ports SW*]
