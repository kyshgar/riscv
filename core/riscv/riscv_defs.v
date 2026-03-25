//--------------------------------------------------------------------
// ALU Operations（ALU操作码，4位编码，选择ALU执行的运算类型）
//--------------------------------------------------------------------
`define ALU_NONE                                4'b0000 // 无操作，直通操作数A
`define ALU_SHIFTL                              4'b0001 // 逻辑左移（SLL/SLLI）
`define ALU_SHIFTR                              4'b0010 // 逻辑右移（SRL/SRLI），高位补0
`define ALU_SHIFTR_ARITH                        4'b0011 // 算术右移（SRA/SRAI），高位补符号位
`define ALU_ADD                                 4'b0100 // 加法（ADD/ADDI）
`define ALU_SUB                                 4'b0110 // 减法（SUB）
`define ALU_AND                                 4'b0111 // 按位与（AND/ANDI）
`define ALU_OR                                  4'b1000 // 按位或（OR/ORI）
`define ALU_XOR                                 4'b1001 // 按位异或（XOR/XORI）
`define ALU_LESS_THAN                           4'b1010 // 无符号小于比较（SLTU/SLTIU）
`define ALU_LESS_THAN_SIGNED                    4'b1011 // 有符号小于比较（SLT/SLTI）

//--------------------------------------------------------------------
// Instructions Masks（RV32I/M 指令编码与掩码，用于指令译码匹配）
// 命名规则：INST_XXX 为指令特征码，INST_XXX_MASK 为对应掩码
// 匹配方式：(opcode & MASK) == INST 即可识别该指令
//--------------------------------------------------------------------
// andi（立即数与操作，I型，funct3=111）
`define INST_ANDI 32'h7013
`define INST_ANDI_MASK 32'h707f

// addi（立即数加法，I型，funct3=000）
`define INST_ADDI 32'h13
`define INST_ADDI_MASK 32'h707f

// slti（立即数有符号小于置位，I型，funct3=010）
`define INST_SLTI_MASK 32'h707f

// sltiu（立即数无符号小于置位，I型，funct3=011）
`define INST_SLTIU 32'h3013
`define INST_SLTIU_MASK 32'h707f

// ori（立即数或操作，I型，funct3=110）
`define INST_ORI 32'h6013
`define INST_ORI_MASK 32'h707f

// xori（立即数异或操作，I型，funct3=100）
`define INST_XORI 32'h4013
`define INST_XORI_MASK 32'h707f

// slli（立即数逻辑左移，I型，funct3=001，funct7=0x00）
`define INST_SLLI 32'h1013
`define INST_SLLI_MASK 32'hfc00707f

// srli（立即数逻辑右移，I型，funct3=101，funct7=0x00）
`define INST_SRLI 32'h5013
`define INST_SRLI_MASK 32'hfc00707f

// srai（立即数算术右移，I型，funct3=101，funct7=0x20）
`define INST_SRAI 32'h40005013
`define INST_SRAI_MASK 32'hfc00707f

// lui（加载高位立即数，U型，将20位立即数加载到rd高20位）
`define INST_LUI 32'h37
`define INST_LUI_MASK 32'h7f

// auipc（PC加高位立即数，U型，rd = PC + (imm<<12)）
`define INST_AUIPC 32'h17
`define INST_AUIPC_MASK 32'h7f

// add（寄存器加法，R型，funct3=000，funct7=0x00）
`define INST_ADD 32'h33
`define INST_ADD_MASK 32'hfe00707f

// sub（寄存器减法，R型，funct3=000，funct7=0x20）
`define INST_SUB 32'h40000033
`define INST_SUB_MASK 32'hfe00707f

// slt（有符号小于置位，R型，funct3=010）
`define INST_SLT 32'h2033
`define INST_SLT_MASK 32'hfe00707f

// sltu（无符号小于置位，R型，funct3=011）
`define INST_SLTU 32'h3033
`define INST_SLTU_MASK 32'hfe00707f

// xor（寄存器异或，R型，funct3=100）
`define INST_XOR 32'h4033
`define INST_XOR_MASK 32'hfe00707f

// or（寄存器或，R型，funct3=110）
`define INST_OR 32'h6033
`define INST_OR_MASK 32'hfe00707f

// and（寄存器与，R型，funct3=111）
`define INST_AND 32'h7033
`define INST_AND_MASK 32'hfe00707f

// sll（寄存器逻辑左移，R型，funct3=001）
`define INST_SLL 32'h1033
`define INST_SLL_MASK 32'hfe00707f

// srl（寄存器逻辑右移，R型，funct3=101，funct7=0x00）
`define INST_SRL 32'h5033
`define INST_SRL_MASK 32'hfe00707f

// sra（寄存器算术右移，R型，funct3=101，funct7=0x20）
`define INST_SRA 32'h40005033
`define INST_SRA_MASK 32'hfe00707f

// jal（无条件跳转，J型，rd=PC+4，PC=PC+imm）
`define INST_JAL 32'h6f
`define INST_JAL_MASK 32'h7f

// jalr（间接跳转，I型，rd=PC+4，PC=(rs1+imm)&~1）
`define INST_JALR 32'h67
`define INST_JALR_MASK 32'h707f

// beq（相等则跳转，B型，funct3=000）
`define INST_BEQ 32'h63
`define INST_BEQ_MASK 32'h707f

// bne（不等则跳转，B型，funct3=001）
`define INST_BNE 32'h1063
`define INST_BNE_MASK 32'h707f

// blt（有符号小于则跳转，B型，funct3=100）
`define INST_BLT 32'h4063
`define INST_BLT_MASK 32'h707f

// bge（有符号大于等于则跳转，B型，funct3=101）
`define INST_BGE 32'h5063
`define INST_BGE_MASK 32'h707f

// bltu（无符号小于则跳转，B型，funct3=110）
`define INST_BLTU 32'h6063
`define INST_BLTU_MASK 32'h707f

// bgeu（无符号大于等于则跳转，B型，funct3=111）
`define INST_BGEU 32'h7063
`define INST_BGEU_MASK 32'h707f

// lb（加载有符号字节，I型，funct3=000，符号扩展到32位）
`define INST_LB 32'h3
`define INST_LB_MASK 32'h707f

// lh（加载有符号半字，I型，funct3=001，符号扩展到32位）
`define INST_LH 32'h1003
`define INST_LH_MASK 32'h707f

// lw（加载字，I型，funct3=010）
`define INST_LW 32'h2003
`define INST_LW_MASK 32'h707f

// lbu（加载无符号字节，I型，funct3=100，零扩展到32位）
`define INST_LBU 32'h4003
`define INST_LBU_MASK 32'h707f

// lhu（加载无符号半字，I型，funct3=101，零扩展到32位）
`define INST_LHU 32'h5003
`define INST_LHU_MASK 32'h707f

// lwu（加载无符号字，I型，funct3=110，零扩展到32位）
`define INST_LWU 32'h6003
`define INST_LWU_MASK 32'h707f

// sb（存储字节，S型，funct3=000，存储rs2[7:0]）
`define INST_SB 32'h23
`define INST_SB_MASK 32'h707f

// sh（存储半字，S型，funct3=001，存储rs2[15:0]）
`define INST_SH 32'h1023
`define INST_SH_MASK 32'h707f

// sw（存储字，S型，funct3=010，存储rs2[31:0]）
`define INST_SW 32'h2023
`define INST_SW_MASK 32'h707f

// ecall（环境调用，系统调用入口，触发异常进入特权模式）
`define INST_ECALL 32'h73
`define INST_ECALL_MASK 32'hffffffff

// ebreak（调试断点，触发断点异常）
`define INST_EBREAK 32'h100073
`define INST_EBREAK_MASK 32'hffffffff

// eret（异常返回，从中断/异常处理程序返回）
`define INST_ERET 32'h200073
`define INST_ERET_MASK 32'hcfffffff

// csrrw（CSR读写，将rs1写入CSR，旧值写回rd）
`define INST_CSRRW 32'h1073
`define INST_CSRRW_MASK 32'h707f

// csrrs（CSR读置位，将CSR与rs1按位或后写回CSR，旧值写回rd）
`define INST_CSRRS 32'h2073
`define INST_CSRRS_MASK 32'h707f

// csrrc（CSR读清位，将CSR与rs1按位与非后写回CSR，旧值写回rd）
`define INST_CSRRC 32'h3073
`define INST_CSRRC_MASK 32'h707f

// csrrwi（CSR立即数读写，使用零扩展的5位立即数写CSR）
`define INST_CSRRWI 32'h5073
`define INST_CSRRWI_MASK 32'h707f

// csrrsi（CSR立即数读置位，使用5位立即数置位CSR相应位）
`define INST_CSRRSI 32'h6073
`define INST_CSRRSI_MASK 32'h707f

// csrrci（CSR立即数读清位，使用5位立即数清除CSR相应位）
`define INST_CSRRCI 32'h7073
`define INST_CSRRCI_MASK 32'h707f

// mul（有符号乘法，RV32M，取乘积低32位）
`define INST_MUL 32'h2000033
`define INST_MUL_MASK 32'hfe00707f

// mulh（有符号×有符号乘法，RV32M，取乘积高32位）
`define INST_MULH 32'h2001033
`define INST_MULH_MASK 32'hfe00707f

// mulhsu（有符号×无符号乘法，RV32M，取乘积高32位）
`define INST_MULHSU 32'h2002033
`define INST_MULHSU_MASK 32'hfe00707f

// mulhu（无符号×无符号乘法，RV32M，取乘积高32位）
`define INST_MULHU 32'h2003033
`define INST_MULHU_MASK 32'hfe00707f

// div（有符号整数除法，RV32M，结果取商）
`define INST_DIV 32'h2004033
`define INST_DIV_MASK 32'hfe00707f

// divu（无符号整数除法，RV32M，结果取商）
`define INST_DIVU 32'h2005033
`define INST_DIVU_MASK 32'hfe00707f

// rem（有符号取余，RV32M，余数符号与被除数相同）
`define INST_REM 32'h2006033
`define INST_REM_MASK 32'hfe00707f

// remu（无符号取余，RV32M）
`define INST_REMU 32'h2007033
`define INST_REMU_MASK 32'hfe00707f

// wfi（等待中断，处理器进入低功耗等待状态直到中断发生）
`define INST_WFI 32'h10500073
`define INST_WFI_MASK 32'hffff8fff

// fence（内存屏障，确保fence前的内存访问对fence后的访问可见）
`define INST_FENCE 32'hf
`define INST_FENCE_MASK 32'h707f

// sfence（TLB刷新屏障，确保页表更新对后续访问可见）
`define INST_SFENCE 32'h12000073
`define INST_SFENCE_MASK 32'hfe007fff

// fence.i（指令缓存刷新屏障，确保数据写入对后续指令取指可见）
`define INST_IFENCE 32'h100f
`define INST_IFENCE_MASK 32'h707f

//--------------------------------------------------------------------
// Privilege levels（特权级别定义，2位编码）
//--------------------------------------------------------------------
`define PRIV_USER         2'd0  // 用户模式（U模式），权限最低
`define PRIV_SUPER        2'd1  // 监督者模式（S模式），操作系统内核
`define PRIV_MACHINE      2'd3  // 机器模式（M模式），权限最高，固件/运行时

//--------------------------------------------------------------------
// IRQ Numbers（中断号定义，对应 mip/mie CSR 寄存器的位索引）
//--------------------------------------------------------------------
`define IRQ_S_SOFT   1  // 监督者软件中断（S模式软件中断）
`define IRQ_M_SOFT   3  // 机器软件中断（M模式软件中断，MSIP）
`define IRQ_S_TIMER  5  // 监督者定时器中断
`define IRQ_M_TIMER  7  // 机器定时器中断（MTIP，来自 mtime≥mtimecmp）
`define IRQ_S_EXT    9  // 监督者外部中断
`define IRQ_M_EXT    11 // 机器外部中断（MEIP，来自外部中断控制器PLIC）
`define IRQ_MIN      (`IRQ_S_SOFT)              // 最小中断号
`define IRQ_MAX      (`IRQ_M_EXT + 1)           // 最大中断号+1（用于范围判断）
`define IRQ_MASK     ((1 << `IRQ_M_EXT)   | (1 << `IRQ_S_EXT)   |                       (1 << `IRQ_M_TIMER) | (1 << `IRQ_S_TIMER) |                       (1 << `IRQ_M_SOFT)  | (1 << `IRQ_S_SOFT)) // 所有有效中断位掩码

// mip/mie 寄存器中各中断位的位索引别名
`define SR_IP_MSIP_R      `IRQ_M_SOFT   // 机器软件中断挂起位
`define SR_IP_MTIP_R      `IRQ_M_TIMER  // 机器定时器中断挂起位
`define SR_IP_MEIP_R      `IRQ_M_EXT    // 机器外部中断挂起位
`define SR_IP_SSIP_R      `IRQ_S_SOFT   // 监督者软件中断挂起位
`define SR_IP_STIP_R      `IRQ_S_TIMER  // 监督者定时器中断挂起位
`define SR_IP_SEIP_R      `IRQ_S_EXT    // 监督者外部中断挂起位

//--------------------------------------------------------------------
// CSR Registers - Simulation control（仿真控制用CSR，非标准RISC-V规范）
//--------------------------------------------------------------------
`define CSR_DSCRATCH       12'h7b2 // 调试暂存寄存器
`define CSR_SIM_CTRL       12'h8b2 // 仿真控制寄存器（仿真环境专用）
`define CSR_SIM_CTRL_MASK  32'hFFFFFFFF
    `define CSR_SIM_CTRL_EXIT (0 << 24) // 仿真退出命令（低24位为退出码）
    `define CSR_SIM_CTRL_PUTC (1 << 24) // 仿真字符输出命令（低8位为字符）

//--------------------------------------------------------------------
// CSR Registers（机器模式 CSR 寄存器地址和访问掩码）
//--------------------------------------------------------------------
`define CSR_MSTATUS       12'h300 // 机器状态寄存器（中断使能/特权级栈）
`define CSR_MSTATUS_MASK  32'hFFFFFFFF
`define CSR_MISA          12'h301 // ISA扩展寄存器（报告硬件支持的ISA特性）
`define CSR_MISA_MASK     32'hFFFFFFFF
    `define MISA_RV32     32'h40000000 // RV32（32位基础ISA）
    `define MISA_RVI      32'h00000100 // 'I'扩展：整数基础指令集
    `define MISA_RVE      32'h00000010 // 'E'扩展：嵌入式（16个寄存器）
    `define MISA_RVM      32'h00001000 // 'M'扩展：整数乘除法
    `define MISA_RVA      32'h00000001 // 'A'扩展：原子操作
    `define MISA_RVF      32'h00000020 // 'F'扩展：单精度浮点
    `define MISA_RVD      32'h00000008 // 'D'扩展：双精度浮点
    `define MISA_RVC      32'h00000004 // 'C'扩展：压缩指令
    `define MISA_RVS      32'h00040000 // 'S'扩展：监督者模式
    `define MISA_RVU      32'h00100000 // 'U'扩展：用户模式
`define CSR_MEDELEG       12'h302 // 机器异常委托寄存器（委托给S模式处理）
`define CSR_MEDELEG_MASK  32'h0000FFFF
`define CSR_MIDELEG       12'h303 // 机器中断委托寄存器（委托给S模式处理）
`define CSR_MIDELEG_MASK  32'h0000FFFF
`define CSR_MIE           12'h304 // 机器中断使能寄存器
`define CSR_MIE_MASK      `IRQ_MASK
`define CSR_MTVEC         12'h305 // 机器异常向量表基地址寄存器
`define CSR_MTVEC_MASK    32'hFFFFFFFF
`define CSR_MSCRATCH      12'h340 // 机器模式暂存寄存器（供M模式异常处理程序使用）
`define CSR_MSCRATCH_MASK 32'hFFFFFFFF
`define CSR_MEPC          12'h341 // 机器异常程序计数器（保存发生异常时的PC）
`define CSR_MEPC_MASK     32'hFFFFFFFF
`define CSR_MCAUSE        12'h342 // 机器异常原因寄存器（bit31=中断标志，低位=原因码）
`define CSR_MCAUSE_MASK   32'h8000000F
`define CSR_MTVAL         12'h343 // 机器异常值寄存器（保存导致异常的地址或指令）
`define CSR_MTVAL_MASK    32'hFFFFFFFF
`define CSR_MIP           12'h344 // 机器中断挂起寄存器（只读，反映当前挂起的中断）
`define CSR_MIP_MASK      `IRQ_MASK
`define CSR_MCYCLE        12'hc00 // 机器周期计数器低32位（自复位以来的时钟周期数）
`define CSR_MCYCLE_MASK   32'hFFFFFFFF
`define CSR_MTIME         12'hc01 // 机器时间计数器低32位（实时时钟）
`define CSR_MTIME_MASK    32'hFFFFFFFF
`define CSR_MTIMEH        12'hc81 // 机器时间计数器高32位
`define CSR_MTIMEH_MASK   32'hFFFFFFFF
`define CSR_MHARTID       12'hF14 // 硬件线程ID（Hart ID，标识当前处理器核）
`define CSR_MHARTID_MASK  32'hFFFFFFFF

// Non-std（非标准扩展：定时器比较寄存器）
`define CSR_MTIMECMP        12'h7c0 // 定时器比较值（mtime >= mtimecmp时触发定时器中断）
`define CSR_MTIMECMP_MASK   32'hFFFFFFFF

//-----------------------------------------------------------------
// CSR Registers - Supervisor（监督者模式 S 级 CSR 寄存器）
//-----------------------------------------------------------------
`define CSR_SSTATUS       12'h100 // 监督者状态寄存器（mstatus的S模式视图子集）
`define CSR_SSTATUS_MASK  `SR_SMODE_MASK
`define CSR_SIE           12'h104 // 监督者中断使能寄存器（仅S/U模式中断位）
`define CSR_SIE_MASK      ((1 << `IRQ_S_EXT) | (1 << `IRQ_S_TIMER) | (1 << `IRQ_S_SOFT))
`define CSR_STVEC         12'h105 // 监督者异常向量表基地址
`define CSR_STVEC_MASK    32'hFFFFFFFF
`define CSR_SSCRATCH      12'h140 // 监督者模式暂存寄存器
`define CSR_SSCRATCH_MASK 32'hFFFFFFFF
`define CSR_SEPC          12'h141 // 监督者异常程序计数器
`define CSR_SEPC_MASK     32'hFFFFFFFF
`define CSR_SCAUSE        12'h142 // 监督者异常原因寄存器
`define CSR_SCAUSE_MASK   32'h8000000F
`define CSR_STVAL         12'h143 // 监督者异常值寄存器
`define CSR_STVAL_MASK    32'hFFFFFFFF
`define CSR_SIP           12'h144 // 监督者中断挂起寄存器
`define CSR_SIP_MASK      ((1 << `IRQ_S_EXT) | (1 << `IRQ_S_TIMER) | (1 << `IRQ_S_SOFT))
`define CSR_SATP          12'h180 // 监督者地址转换和保护寄存器（页表基地址+ASID+模式）
`define CSR_SATP_MASK     32'hFFFFFFFF

//--------------------------------------------------------------------
// CSR Registers - DCACHE control（数据缓存控制CSR，复用 pmpcfg0-2 地址）
//--------------------------------------------------------------------
`define CSR_DFLUSH            12'h3a0 // D-Cache 全部刷写（flush）控制，地址同 pmpcfg0
`define CSR_DFLUSH_MASK       32'hFFFFFFFF
`define CSR_DWRITEBACK        12'h3a1 // D-Cache 脏行写回控制，地址同 pmpcfg1
`define CSR_DWRITEBACK_MASK   32'hFFFFFFFF
`define CSR_DINVALIDATE       12'h3a2 // D-Cache 无效化控制，地址同 pmpcfg2
`define CSR_DINVALIDATE_MASK  32'hFFFFFFFF

//--------------------------------------------------------------------
// Status Register（mstatus 寄存器各字段的位定义）
// mstatus 是机器模式状态寄存器，控制中断使能和特权级切换
//--------------------------------------------------------------------
`define SR_UIE         (1 << 0)  // 用户模式中断使能位（UIE）
`define SR_UIE_R       0         // UIE 位索引
`define SR_SIE         (1 << 1)  // 监督者模式中断使能位（SIE）
`define SR_SIE_R       1         // SIE 位索引
`define SR_MIE         (1 << 3)  // 机器模式中断使能位（MIE）
`define SR_MIE_R       3         // MIE 位索引
`define SR_UPIE        (1 << 4)  // 进入U模式前的中断使能备份（UPIE）
`define SR_UPIE_R      4         // UPIE 位索引
`define SR_SPIE        (1 << 5)  // 进入S模式前的中断使能备份（SPIE）
`define SR_SPIE_R      5         // SPIE 位索引
`define SR_MPIE        (1 << 7)  // 进入M模式前的中断使能备份（MPIE）
`define SR_MPIE_R      7         // MPIE 位索引
`define SR_SPP         (1 << 8)  // 进入S模式前的特权级备份（SPP），0=U，1=S
`define SR_SPP_R       8         // SPP 位索引

// MPP 字段：进入M模式前的特权级备份（2位）
`define SR_MPP_SHIFT   11        // MPP 字段起始位
`define SR_MPP_MASK    2'h3      // MPP 字段掩码
`define SR_MPP_R       12:11     // MPP 字段位范围
`define SR_MPP_U       `PRIV_USER    // MPP=00：陷入前为U模式
`define SR_MPP_S       `PRIV_SUPER   // MPP=01：陷入前为S模式
`define SR_MPP_M       `PRIV_MACHINE // MPP=11：陷入前为M模式

`define SR_SUM_R        18       // SUM 位索引：S模式可访问U模式内存时置1
`define SR_SUM          (1 << `SR_SUM_R)

`define SR_MPRV_R       17       // MPRV 位索引：修改特权级（影响数据内存访问特权级）
`define SR_MPRV         (1 << `SR_MPRV_R)

`define SR_MXR_R        19       // MXR 位索引：使可执行页变为可读（用于加载指令页表项）
`define SR_MXR          (1 << `SR_MXR_R)

// S模式可见的 mstatus 字段掩码（sstatus 是 mstatus 的受限视图）
`define SR_SMODE_MASK   (`SR_UIE | `SR_SIE | `SR_UPIE | `SR_SPIE | `SR_SPP | `SR_SUM)

//--------------------------------------------------------------------
// SATP definitions（satp CSR 寄存器字段定义，控制页表基地址和模式）
//--------------------------------------------------------------------
`define SATP_PPN_R        19:0  // 页表根物理页号（PPN），物理基地址的高位 // TODO: Should be 21??
`define SATP_ASID_R       30:22 // 地址空间标识符（ASID），用于TLB区分进程
`define SATP_MODE_R       31    // 地址转换模式：0=Bare（无转换），1=SV32（32位虚拟内存）

//--------------------------------------------------------------------
// MMU Defs (SV32)（SV32 分页机制常量定义，用于两级页表地址转换）
//--------------------------------------------------------------------
`define MMU_LEVELS        2                       // 页表层级数（SV32 两级：VPN[1] 和 VPN[0]）
`define MMU_PTIDXBITS     10                      // 每级页表索引位数（10位，每级1024个页表项）
`define MMU_PTESIZE       4                       // 页表项大小（4字节）
`define MMU_PGSHIFT       (`MMU_PTIDXBITS + 2)    // 页内偏移位数（12位 = 4KB页）
`define MMU_PGSIZE        (1 << `MMU_PGSHIFT)     // 页大小（4096字节）
`define MMU_VPN_BITS      (`MMU_PTIDXBITS * `MMU_LEVELS) // 虚拟页号总位数（20位）
`define MMU_PPN_BITS      (32 - `MMU_PGSHIFT)     // 物理页号位数（20位）
`define MMU_VA_BITS       (`MMU_VPN_BITS + `MMU_PGSHIFT)  // 虚拟地址总位数（32位）

// 页表项（PTE）标志位定义（SV32 规范）
`define PAGE_PRESENT      0  // V（Valid）：页表项有效位
`define PAGE_READ         1  // R（Read）：页可读
`define PAGE_WRITE        2  // W（Write）：页可写
`define PAGE_EXEC         3  // X（Execute）：页可执行
`define PAGE_USER         4  // U（User）：页可被用户模式访问
`define PAGE_GLOBAL       5  // G（Global）：全局映射（所有ASID共享）
`define PAGE_ACCESSED     6  // A（Accessed）：页已被访问（硬件置位）
`define PAGE_DIRTY        7  // D（Dirty）：页已被写（硬件置位）
`define PAGE_SOFT         9:8 // RSW：软件保留位（操作系统自定义使用）

`define PAGE_FLAGS       10'h3FF // 全部标志位掩码（PTE低10位）

`define PAGE_PFN_SHIFT   10   // PTE 中物理页号字段起始位
`define PAGE_SIZE        4096 // 页大小（字节）

//--------------------------------------------------------------------
// Exception Causes（内部异常类型编码，6位宽，用于流水线内部传递异常信息）
// bit[5:4]为类型掩码：0x1x=异常，0x2x=中断，0x3x=ERET返回
//--------------------------------------------------------------------
`define EXCEPTION_W                        6    // 异常码位宽
`define EXCEPTION_MISALIGNED_FETCH         6'h10 // 取指地址未对齐
`define EXCEPTION_FAULT_FETCH              6'h11 // 取指访问错误
`define EXCEPTION_ILLEGAL_INSTRUCTION      6'h12 // 非法指令
`define EXCEPTION_BREAKPOINT               6'h13 // 断点（ebreak）
`define EXCEPTION_MISALIGNED_LOAD          6'h14 // 加载地址未对齐
`define EXCEPTION_FAULT_LOAD               6'h15 // 加载访问错误
`define EXCEPTION_MISALIGNED_STORE         6'h16 // 存储地址未对齐
`define EXCEPTION_FAULT_STORE              6'h17 // 存储访问错误
`define EXCEPTION_ECALL                    6'h18 // 环境调用（ecall）
`define EXCEPTION_ECALL_U                  6'h18 // U模式环境调用
`define EXCEPTION_ECALL_S                  6'h19 // S模式环境调用
`define EXCEPTION_ECALL_H                  6'h1a // H模式环境调用（预留）
`define EXCEPTION_ECALL_M                  6'h1b // M模式环境调用
`define EXCEPTION_PAGE_FAULT_INST          6'h1c // 指令页错误
`define EXCEPTION_PAGE_FAULT_LOAD          6'h1d // 加载页错误
`define EXCEPTION_PAGE_FAULT_STORE         6'h1f // 存储页错误
`define EXCEPTION_EXCEPTION                6'h10 // 通用异常标志（bit[5:4]=01）
`define EXCEPTION_INTERRUPT                6'h20 // 中断标志（bit[5:4]=10）
`define EXCEPTION_ERET_U                   6'h30 // U模式ERET返回（bit[5:4]=11）
`define EXCEPTION_ERET_S                   6'h31 // S模式ERET返回
`define EXCEPTION_ERET_H                   6'h32 // H模式ERET返回
`define EXCEPTION_ERET_M                   6'h33 // M模式ERET返回
`define EXCEPTION_FENCE                    6'h34 // FENCE指令（需要流水线冲刷）
`define EXCEPTION_TYPE_MASK                6'h30 // 异常类型掩码（高2位）
`define EXCEPTION_SUBTYPE_R                3:0   // 异常子类型位域

// mcause 寄存器编码（RISC-V规范定义的标准异常原因码）
`define MCAUSE_INT                      31        // mcause 中断标志位（bit31=1表示中断）
`define MCAUSE_MISALIGNED_FETCH         ((0 << `MCAUSE_INT) | 0)  // 取指未对齐
`define MCAUSE_FAULT_FETCH              ((0 << `MCAUSE_INT) | 1)  // 取指访问错误
`define MCAUSE_ILLEGAL_INSTRUCTION      ((0 << `MCAUSE_INT) | 2)  // 非法指令
`define MCAUSE_BREAKPOINT               ((0 << `MCAUSE_INT) | 3)  // 断点
`define MCAUSE_MISALIGNED_LOAD          ((0 << `MCAUSE_INT) | 4)  // 加载未对齐
`define MCAUSE_FAULT_LOAD               ((0 << `MCAUSE_INT) | 5)  // 加载访问错误
`define MCAUSE_MISALIGNED_STORE         ((0 << `MCAUSE_INT) | 6)  // 存储未对齐
`define MCAUSE_FAULT_STORE              ((0 << `MCAUSE_INT) | 7)  // 存储访问错误
`define MCAUSE_ECALL_U                  ((0 << `MCAUSE_INT) | 8)  // U模式 ecall
`define MCAUSE_ECALL_S                  ((0 << `MCAUSE_INT) | 9)  // S模式 ecall
`define MCAUSE_ECALL_H                  ((0 << `MCAUSE_INT) | 10) // H模式 ecall
`define MCAUSE_ECALL_M                  ((0 << `MCAUSE_INT) | 11) // M模式 ecall
`define MCAUSE_PAGE_FAULT_INST          ((0 << `MCAUSE_INT) | 12) // 指令页错误
`define MCAUSE_PAGE_FAULT_LOAD          ((0 << `MCAUSE_INT) | 13) // 加载页错误
`define MCAUSE_PAGE_FAULT_STORE         ((0 << `MCAUSE_INT) | 15) // 存储页错误
`define MCAUSE_INTERRUPT                (1 << `MCAUSE_INT)         // 中断标志（bit31=1）

//--------------------------------------------------------------------
// Debug（调试接口寄存器编号定义，用于外部调试器访问内部状态）
//--------------------------------------------------------------------
`define RISCV_REGNO_FIRST   13'd0    // 寄存器编号起始值
`define RISCV_REGNO_GPR0    13'd0    // 通用寄存器 x0 的编号
`define RISCV_REGNO_GPR31   13'd31   // 通用寄存器 x31 的编号
`define RISCV_REGNO_PC      13'd32   // PC 的调试寄存器编号
`define RISCV_REGNO_CSR0    13'd65   // CSR 寄存器起始编号（CSR地址0对应此编号）
`define RISCV_REGNO_CSR4095 (`RISCV_REGNO_CSR0 +  13'd4095) // CSR 寄存器最大编号
`define RISCV_REGNO_PRIV    13'd4161 // 当前特权级的调试寄存器编号
