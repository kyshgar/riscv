# RISC-V CPU Core 工程架构全景分析

> 本文档对 `kyshgar/riscv` 工程进行全面的模块划分与功能解析，涵盖 CPU 核心、SoC 封装、ISA 模拟器等全部组件。

---

## 一、目录总览

```
kyshgar/riscv/
├── core/riscv/            ← 🔴 CPU 核心（Verilog）—— 工程的灵魂
├── top_tcm_wrapper/       ← 🟠 TCM（紧耦合存储器）封装顶层
├── top_tcm_axi/           ← 🟡 TCM + AXI 总线 SoC 顶层 + testbench
├── top_cache_axi/         ← 🟡 Cache + AXI 总线 SoC 顶层 + testbench
├── isa_sim/               ← 🔵 C++ ISA 指令集模拟器（软件）
├── doc/                   ← 📄 文档
├── LICENSE / README.md
```

**语言构成**：

| 语言 | 占比 |
|------|------|
| Verilog | 68.5% |
| C++ | 27.6% |
| C | 3.3% |
| Makefile | 0.6% |

---

## 二、各模块详解

### 模块 A：CPU 核心 `core/riscv/`

这是一个 **5 级流水线、支持 RV32IM + Supervisor 扩展** 的 RISC-V CPU。内部结构如下：

```
┌────────────────────────────────────────────────────────────────────┐
│                        riscv_core.v (顶层例化)                      │
│                                                                    │
│  ┌──────────┐  ┌───────────┐  ┌───────────┐  ┌────────────────┐   │
│  │ u_fetch  │→│ u_decode   │→│ u_issue    │→│ 5条执行通道     │   │
│  │ 取指单元 │  │ 译码单元   │  │ 发射+寄存器│  │(见下方)         │   │
│  └──────────┘  └───────────┘  └───────────┘  └────────────────┘   │
│       ↕              ↕                              ↕              │
│  ┌──────────┐  ┌───────────┐         ┌──────────────────────┐     │
│  │ u_mmu    │  │u_decoder  │         │   u_pipe_ctrl        │     │
│  │ 内存管理 │  │(纯组合逻辑)│         │   流水线控制/旁路    │     │
│  └──────────┘  └───────────┘         └──────────────────────┘     │
└────────────────────────────────────────────────────────────────────┘
```

**18 个 Verilog 文件，按功能分组：**

| # | 文件 | 功能 | 流水级 |
|---|------|------|--------|
| 1 | `riscv_core.v` | **顶层连线**：例化所有子模块、参数传递 | — |
| 2 | `riscv_defs.v` | **全局宏定义**：指令编码、CSR 地址、异常码、ALU 操作码 | — |
| 3 | `riscv_fetch.v` | **取指单元**：PC 管理、分支缓冲、ICache 握手、Skid Buffer | IF |
| 4 | `riscv_decode.v` | **译码流水外壳**：可选的额外寄存器级（参数控制） | IF/ID |
| 5 | `riscv_decoder.v` | **译码器核心**：纯组合逻辑，指令→执行通道分类 | ID |
| 6 | `riscv_issue.v` | **发射单元**：记分板(scoreboard)、冒险检测(RAW)、指令派发 | ID/IS |
| 7 | `riscv_pipe_ctrl.v` | **流水线控制**：旁路(bypass)、stall、squash、结果回写仲裁 | 全局 |
| 8 | `riscv_regfile.v` | **通用寄存器堆**：32×32bit，2读1写端口 | ID |
| 9 | `riscv_xilinx_2r1w.v` | **Xilinx 专用寄存器堆**：用分布式 RAM 实现（可选） | ID |
| 10 | `riscv_exec.v` | **执行单元①**：ALU运算 + 分支判断 + 分支目标计算 | EX |
| 11 | `riscv_alu.v` | **ALU 核心**：加减、逻辑、移位、比较（桶形移位器） | EX |
| 12 | `riscv_lsu.v` | **执行单元②**：Load/Store 单元，地址计算、字节对齐、DCache 握手 | EX/MEM |
| 13 | `riscv_multiplier.v` | **执行单元③**：2级流水乘法器（MUL/MULH/MULHSU/MULHU） | EX |
| 14 | `riscv_divider.v` | **执行单元④**：迭代除法器（DIV/DIVU/REM/REMU，2~34周期） | EX (多周期) |
| 15 | `riscv_csr.v` | **执行单元⑤**：CSR 读写/ECALL/EBREAK/MRET/中断入口 | EX |
| 16 | `riscv_csr_regfile.v` | **CSR 寄存器堆**：mstatus/mtvec/mepc/mcause/mtimecmp 等 | WB |
| 17 | `riscv_mmu.v` | **MMU**：Sv32 页表遍历、ITLB/DTLB（各1项） | IF/MEM |
| 18 | `riscv_trace_sim.v` | **仿真追踪**：反汇编打印指令执行记录（仅仿真用） | — |

---

### 模块 B：TCM 封装 `top_tcm_wrapper/`

| 文件 | 功能 |
|------|------|
| `riscv_tcm_wrapper.v` | **SoC 顶层封装**：将 `riscv_core` 与 TCM 存储器和 AXI 总线桥连接 |
| `tcm_mem.v` | **TCM 存储控制器**：将 ICache/DCache 接口映射到 SRAM |
| `tcm_mem_ram.v` | **SRAM 基本单元**：可综合的双端口 RAM |
| `tcm_mem_pmem.v` | **分体存储器**：将大 RAM 切分为多个小 bank（适应 FPGA BRAM） |
| `dport_mux.v` | **数据端口复用器**：仲裁 TCM 和外设访问 |
| `dport_axi.v` | **AXI 桥接器**：将简单存储接口转换为 AXI4 主接口（访问外设） |

**这套组合的意义**：CPU 核心使用简单的 `rd/wr/addr/data` 接口，而外部总线一般是 AXI。这一层做"翻译"。

---

### 模块 C：SoC 顶层变体 `top_cache_axi/` 和 `top_tcm_axi/`

| 目录 | 区别 |
|------|------|
| `top_tcm_axi/` | **TCM 模式**：指令和数据都在片上 SRAM 中（低延迟、确定性），外设走 AXI |
| `top_cache_axi/` | **Cache 模式**：指令和数据通过 ICache/DCache 从外部 AXI 存储器读取 |

两者各自包含 `src_v/`（源码）和 `tb/`（testbench），分别提供了不同存储架构的完整 SoC 集成方案。

---

### 模块 D：ISA 模拟器 `isa_sim/`

| 文件 | 功能 |
|------|------|
| `riscv.cpp/.h` | **CPU 模拟核心**：C++ 实现的 RV32IM 指令集解释器 |
| `riscv_isa.h` | 指令编码定义（与 `riscv_defs.v` 对应的 C 版本） |
| `riscv_inst_dump.cpp/.h` | 反汇编器，将机器码转为可读汇编 |
| `cosim_api.cpp/.h` | **协同仿真接口**：与 RTL 仿真 (Verilator) 对比验证 |
| `elf_load.cpp/.h` | ELF 文件加载器 |
| `memory.h` | 存储器模型 |
| `main.cpp` / `riscv_main.cpp` | 入口程序 |

**用途**：这是一个 **golden reference model**，用来与 RTL 仿真逐拍比对，确保硬件实现的正确性。

---

## 三、全局数据流图

```
                        ┌─────────────────────────────────────────────┐
                        │              riscv_tcm_wrapper.v            │
                        │  ┌─────────┐  ┌──────────┐  ┌───────────┐  │
         AXI 外设 ◄────►│  │dport_axi│◄►│dport_mux │◄►│  tcm_mem  │  │
                        │  └─────────┘  └──────────┘  └───────────┘  │
                        │                    ↕               ↕        │
                        │  ┌──────────────────────────────────────┐   │
                        │  │           riscv_core.v               │   │
                        │  │                                      │   │
                        │  │  [fetch]→[mmu]→[decode]→[issue]      │   │
                        │  │     ↑                      ↓         │   │
                        │  │     └──branch──[exec/alu]  ↓         │   │
                        │  │                [lsu]────►dcache      │   │
                        │  │                [mul]                  │   │
                        │  │                [div]                  │   │
                        │  │                [csr]──►中断/异常      │   │
                        │  │                  ↓                    │   │
                        │  │             [pipe_ctrl]──►regfile     │   │
                        │  └──────────────────────────────────────┘   │
                        └─────────────────────────────────────────────┘
```

---

## 四、可裁剪分析

按"对功能正确性的影响"分为三级。

### 🔴 绝对不能去除

| 模块/文件 | 原因 |
|-----------|------|
| `riscv_core.v` | 顶层，去掉就没 CPU 了 |
| `riscv_defs.v` | 所有文件都 include 它 |
| `riscv_fetch.v` | 取指是流水线第一步 |
| `riscv_decode.v` + `riscv_decoder.v` | 不译码就不知道执行什么 |
| `riscv_issue.v` | 发射+寄存器读取+冒险检测 |
| `riscv_pipe_ctrl.v` | 流水线控制核心（旁路/stall/squash） |
| `riscv_regfile.v` | x0~x31 通用寄存器 |
| `riscv_exec.v` + `riscv_alu.v` | ALU + 分支计算，RV32I 基本功能 |
| `riscv_lsu.v` | Load/Store，没它无法访存 |
| `riscv_csr.v` + `riscv_csr_regfile.v` | 中断/异常/系统调用的唯一通道 |
| `top_tcm_wrapper/` 下全部文件 | SoC 集成必需（如果用 TCM 模式） |

### 🟡 可以通过参数关闭（但文件保留）

| 模块/参数 | 如何关闭 | 效果 |
|-----------|---------|------|
| **MMU** `riscv_mmu.v` | `SUPPORT_MMU = 0` | 约 20KB 的页表遍历逻辑变为纯线连(passthrough)。裸机/RTOS 不需要虚拟内存，设为 0 可大幅节省面积 |
| **Supervisor 模式** | `SUPPORT_SUPER = 0`（在 `riscv_csr.v`） | 去掉 S 模式 CSR 寄存器(sstatus/stvec/sepc 等)，裸机只跑 M 模式足够 |
| **乘法器** `riscv_multiplier.v` | `SUPPORT_MULDIV = 0` | MUL/MULH/MULHSU/MULHU 4条指令变为非法。⚠️ 如果你的软件用了乘法指令，不能关 |
| **除法器** `riscv_divider.v` | 同上，`SUPPORT_MULDIV = 0` 会同时关闭 | DIV/DIVU/REM/REMU 4条指令变为非法。除法器面积大，但 GCC 默认会生成 div 指令 |
| **Xilinx 寄存器堆** `riscv_xilinx_2r1w.v` | `SUPPORT_REGFILE_XILINX = 0` | 改用通用 Verilog 寄存器堆。如果目标不是 Xilinx FPGA，设为 0 |
| **额外译码级** | `EXTRA_DECODE_STAGE = 0` | 少一级流水线延迟 |
| **Load 旁路** | `SUPPORT_LOAD_BYPASS = 0/1` | 控制是否允许 Load 结果直接旁路到下一条指令 |
| **乘法旁路** | `SUPPORT_MUL_BYPASS = 0/1` | 控制乘法结果旁路 |

### 🟢 可以安全删除（不影响综合/功能）

| 模块/文件 | 原因 |
|-----------|------|
| `riscv_trace_sim.v` | 仅用于 Verilator 仿真时打印指令追踪日志。综合时 EDA 工具会忽略 |
| `isa_sim/` 整个目录 | 纯 C++ 软件模拟器，与硬件综合无关。仅在你做 co-simulation 验证时有用 |
| `top_cache_axi/`（如果用 TCM 模式） | 两套 SoC 顶层二选一 |
| `top_tcm_axi/`（如果用 Cache 模式） | 同上，反过来 |
| `doc/` | 文档目录，不参与综合 |

---

## 五、推荐的最小化配置

如果目标是 **裸机/RTOS (M-mode only)** + **节省 FPGA 面积**，推荐的参数配置：

```verilog
// 在 riscv_core.v 实例化时设置：
.SUPPORT_MMU(0),           // 关闭 → 省掉 MMU 页表遍历 + 2个TLB
.SUPPORT_SUPER(0),         // 关闭 → 省掉 S-mode CSR
.SUPPORT_MULDIV(1),        // 保留 → RT-Thread/矩阵运算需要 mul/div
.SUPPORT_LOAD_BYPASS(1),   // 保留 → 提升 IPC
.SUPPORT_MUL_BYPASS(1),    // 保留 → 提升 IPC
.SUPPORT_REGFILE_XILINX(0),// 关闭 → 非 Xilinx 平台
.EXTRA_DECODE_STAGE(0),    // 关闭 → 减少分支惩罚
```

**面积估算影响**：

- 关 MMU → 省约 15-20% LUT
- 关 SUPER → 省约 5% LUT（CSR 寄存器减少）
- 总面积预估可从约 8000 LUT 降至约 6000 LUT

---

## 六、总结表

| 模块 | 角色 | 能否裁剪 | 裁剪方式 |
|------|------|---------|---------|
| `core/riscv/` (18文件) | CPU 核心 | 文件不能删，参数可调 | 参数 `SUPPORT_MMU/SUPER/MULDIV` |
| `top_tcm_wrapper/` (6文件) | TCM SoC 封装 | 必须保留（TCM 模式） | — |
| `top_cache_axi/` | Cache SoC 变体 | 🟢 可删（若用 TCM） | 整目录删除 |
| `top_tcm_axi/` | TCM SoC 变体 | 🟢 可删（若用 Cache） | 整目录删除 |
| `isa_sim/` (C++模拟器) | 软件验证工具 | 🟢 可删 | 整目录删除 |
| `riscv_trace_sim.v` | 仿真打印 | 🟢 可删 | 直接删文件 |
| `riscv_xilinx_2r1w.v` | Xilinx 专用 RF | 🟢 参数关闭 | `SUPPORT_REGFILE_XILINX=0` |
| `riscv_mmu.v` | 虚拟内存 | 🟡 参数关闭 | `SUPPORT_MMU=0` |
| `riscv_multiplier.v` | 乘法器 | ⚠️ 看软件需求 | `SUPPORT_MULDIV=0`（慎用） |
| `riscv_divider.v` | 除法器 | ⚠️ 看软件需求 | 同上 |

---

*文档生成时间：2026-03-24*