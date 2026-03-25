// ================================================================
// 模块功能概述：MMU内存管理单元适配层
//   本模块根据 SUPPORT_MMU 参数选择两种工作模式：
//   1. SUPPORT_MMU=0（直通模式）：ICache和DCache地址不做翻译，
//      物理地址直接透传，不产生页表错误。
//   2. SUPPORT_MMU=1（Sv32翻译模式）：实现RISC-V Sv32虚拟地址翻译：
//      - 维护指令TLB（ITLB）和数据TLB（DTLB），各1项
//      - TLB未命中时启动页表遍历（Page Table Walker, PTW）
//      - PTW通过LSU数据端口读取页表项（PTE），最多两级（4MB超页/4KB页）
//      - 根据PTE标志位进行权限检查（可执行/可读/可写/用户页）
//      - 负责ICache（取指侧）和DCache（访存侧）的虚实地址翻译
//      - 判断物理地址是否在可缓存范围 [MEM_CACHE_ADDR_MIN, MEM_CACHE_ADDR_MAX]
//      - 翻译请求与CPU LSU请求通过src_mmu_w信号进行多路复用
// ================================================================
module riscv_mmu
//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
#(
     parameter MEM_CACHE_ADDR_MIN = 32'h80000000  // 可缓存内存区域起始地址（含），物理地址在此范围内才标记为cacheable
    ,parameter MEM_CACHE_ADDR_MAX = 32'h8fffffff  // 可缓存内存区域结束地址（含）
    ,parameter SUPPORT_MMU      = 1               // 是否启用MMU：1=Sv32地址翻译，0=物理地址直通
)
//-----------------------------------------------------------------
// Ports
//-----------------------------------------------------------------
(
    // Inputs
     input           clk_i                // 系统时钟，上升沿触发
    ,input           rst_i                // 异步复位，高电平有效
    ,input  [  1:0]  priv_d_i             // 数据访问特权级（00=用户,01=Supervisor,11=Machine）
    ,input           sum_i                // Supervisor模式是否允许访问用户页（sstatus.SUM位）
    ,input           mxr_i               // 是否允许通过可执行页做数据读取（sstatus.MXR位）
    ,input           flush_i             // TLB冲刷请求（sfence.vma指令触发），使所有TLB项失效
    ,input  [ 31:0]  satp_i              // satp CSR值：[31]=虚拟化使能，[21:0]=页表根物理页号（PPN）
    ,input           fetch_in_rd_i       // 取指请求使能（来自CPU取指单元）
    ,input           fetch_in_flush_i    // 取指冲刷请求
    ,input           fetch_in_invalidate_i // 取指无效化请求
    ,input  [ 31:0]  fetch_in_pc_i       // CPU侧取指虚拟地址（PC）
    ,input  [  1:0]  fetch_in_priv_i     // 取指特权级
    ,input           fetch_out_accept_i  // ICache接受请求（背压信号）
    ,input           fetch_out_valid_i   // ICache返回数据有效
    ,input           fetch_out_error_i   // ICache访问错误
    ,input  [ 31:0]  fetch_out_inst_i    // ICache返回的指令数据
    ,input  [ 31:0]  lsu_in_addr_i       // CPU侧LSU虚拟地址
    ,input  [ 31:0]  lsu_in_data_wr_i    // CPU侧LSU写数据
    ,input           lsu_in_rd_i         // CPU侧LSU读使能
    ,input  [  3:0]  lsu_in_wr_i         // CPU侧LSU字节写使能
    ,input           lsu_in_cacheable_i  // CPU侧LSU可缓存标志（直通模式使用）
    ,input  [ 10:0]  lsu_in_req_tag_i    // CPU侧LSU请求标签
    ,input           lsu_in_invalidate_i // CPU侧LSU无效化请求
    ,input           lsu_in_writeback_i  // CPU侧LSU写回请求
    ,input           lsu_in_flush_i      // CPU侧LSU冲刷请求
    ,input  [ 31:0]  lsu_out_data_rd_i   // DCache返回的读数据（包含PTE数据）
    ,input           lsu_out_accept_i    // DCache接受请求的握手信号
    ,input           lsu_out_ack_i       // DCache完成请求的应答
    ,input           lsu_out_error_i     // DCache总线错误
    ,input  [ 10:0]  lsu_out_resp_tag_i  // DCache响应标签（[9:7]==3'b111时为MMU PTE响应）

    // Outputs
    ,output          fetch_in_accept_o   // CPU取指请求被接受（握手确认）
    ,output          fetch_in_valid_o    // 向CPU取指单元返回指令有效
    ,output          fetch_in_error_o    // 向CPU取指单元报告访问错误
    ,output [ 31:0]  fetch_in_inst_o     // 向CPU取指单元返回的指令
    ,output          fetch_out_rd_o      // 向ICache发出的取指使能（翻译后的物理地址读请求）
    ,output          fetch_out_flush_o   // 向ICache发出的冲刷请求
    ,output          fetch_out_invalidate_o // 向ICache发出的无效化请求
    ,output [ 31:0]  fetch_out_pc_o      // 向ICache发出的取指物理地址
    ,output          fetch_in_fault_o    // 取指页访问错误（页表权限不足）
    ,output [ 31:0]  lsu_in_data_rd_o    // 向CPU LSU返回的读数据
    ,output          lsu_in_accept_o     // CPU侧LSU请求被接受（握手确认）
    ,output          lsu_in_ack_o        // CPU侧LSU完成应答
    ,output          lsu_in_error_o      // CPU侧LSU访问错误
    ,output [ 10:0]  lsu_in_resp_tag_o   // CPU侧LSU响应标签
    ,output [ 31:0]  lsu_out_addr_o      // 向DCache发出的物理地址（翻译后）
    ,output [ 31:0]  lsu_out_data_wr_o   // 向DCache发出的写数据
    ,output          lsu_out_rd_o        // 向DCache发出的读使能
    ,output [  3:0]  lsu_out_wr_o        // 向DCache发出的字节写使能
    ,output          lsu_out_cacheable_o // 向DCache标记是否可缓存
    ,output [ 10:0]  lsu_out_req_tag_o   // 向DCache发出的请求标签（[9:7]=3'b111时为MMU请求）
    ,output          lsu_out_invalidate_o// 向DCache发出的无效化请求
    ,output          lsu_out_writeback_o // 向DCache发出的写回请求
    ,output          lsu_out_flush_o     // 向DCache发出的冲刷请求
    ,output          lsu_in_load_fault_o // 加载访问页表错误信号
    ,output          lsu_in_store_fault_o// 存储访问页表错误信号
);



//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

//-----------------------------------------------------------------
// Local defs
//-----------------------------------------------------------------
localparam  STATE_W            = 2; // 状态机位宽
localparam  STATE_IDLE         = 0; // 空闲状态：等待TLB未命中
localparam  STATE_LEVEL_FIRST  = 1; // 第一级页表遍历（读取一级PTE，可能是4MB超页或指向二级页表）
localparam  STATE_LEVEL_SECOND = 2; // 第二级页表遍历（读取二级PTE，4KB普通页）
localparam  STATE_UPDATE       = 3; // TLB更新状态：将新PTE写入ITLB或DTLB

//-----------------------------------------------------------------
// Basic MMU support
//-----------------------------------------------------------------
generate
if (SUPPORT_MMU)
begin

    //-----------------------------------------------------------------
    // Registers
    //-----------------------------------------------------------------
    // PTW（页表遍历器）状态机寄存器
    // resp_mmu_w：通过响应标签[9:7]==3'b111判断此次DCache响应是否为PTW页表读取
    reg [STATE_W-1:0] state_q;
    wire              idle_w = (state_q == STATE_IDLE);

    // Magic combo used only by MMU
    wire        resp_mmu_w   = (lsu_out_resp_tag_i[9:7] == 3'b111);
    wire        resp_valid_w = resp_mmu_w & lsu_out_ack_i;
    wire        resp_error_w = resp_mmu_w & lsu_out_error_i;
    wire [31:0] resp_data_w  = lsu_out_data_rd_i;

    wire        cpu_accept_w;

    //-----------------------------------------------------------------
    // Load / Store
    //-----------------------------------------------------------------
    // load_q/store_q：当CPU发出Load/Store请求但尚未被接受时，保持请求信号
    // 防止请求在地址翻译等待期间丢失
    reg       load_q;
    reg [3:0] store_q;

    always @ (posedge clk_i or posedge rst_i)
    if (rst_i)
        load_q <= 1'b0;
    else if (lsu_in_rd_i)
        load_q <= ~lsu_in_accept_o;

    always @ (posedge clk_i or posedge rst_i)
    if (rst_i)
        store_q <= 4'b0;
    else if (|lsu_in_wr_i)
        store_q <= lsu_in_accept_o ? 4'b0 : lsu_in_wr_i;

    wire       load_w  = lsu_in_rd_i | load_q;
    wire [3:0] store_w = lsu_in_wr_i | store_q;

    reg [31:0] lsu_in_addr_q;

    always @ (posedge clk_i or posedge rst_i)
    if (rst_i)
        lsu_in_addr_q <= 32'b0;
    else if (load_w || (|store_w))
        lsu_in_addr_q <= lsu_in_addr_i;

    wire [31:0] lsu_addr_w = (load_w || (|store_w)) ? lsu_in_addr_i : lsu_in_addr_q;

    //-----------------------------------------------------------------
    // Page table walker
    //-----------------------------------------------------------------
    // Sv32地址结构：VPN[1]=VA[31:22]（10位），VPN[0]=VA[21:12]（10位），页内偏移=VA[11:0]
    // ptbr_w：页表根地址 = satp[21:0] << 12（页大小对齐）
    // ifetch_vm_w：指令侧是否启用虚拟化（非Machine模式下启用）
    // dfetch_vm_w：数据侧是否启用虚拟化（satp.MODE=1且非Machine模式）
    wire        itlb_hit_w;
    wire        dtlb_hit_w;

    reg         dtlb_req_q;

    // Global enable
    wire        vm_enable_w = satp_i[`SATP_MODE_R];
    wire [31:0] ptbr_w      = {satp_i[`SATP_PPN_R], 12'b0};

    wire        ifetch_vm_w = (fetch_in_priv_i != `PRIV_MACHINE);
    wire        dfetch_vm_w = (priv_d_i != `PRIV_MACHINE);

    wire        supervisor_i_w = (fetch_in_priv_i == `PRIV_SUPER);
    wire        supervisor_d_w = (priv_d_i == `PRIV_SUPER);

    wire        vm_i_enable_w = (ifetch_vm_w);
    wire        vm_d_enable_w = (vm_enable_w & dfetch_vm_w);

    // TLB entry does not match request address
    wire        itlb_miss_w = fetch_in_rd_i & vm_i_enable_w & ~itlb_hit_w;
    wire        dtlb_miss_w = (load_w || (|store_w)) & vm_d_enable_w & ~dtlb_hit_w;

    // Data miss is higher priority than instruction...
    wire [31:0] request_addr_w = idle_w ? 
                                (dtlb_miss_w ? lsu_addr_w : fetch_in_pc_i) :
                                 dtlb_req_q ? lsu_addr_w : fetch_in_pc_i;

    reg [31:0]  pte_addr_q;
    reg [31:0]  pte_entry_q;
    reg [31:0]  virt_addr_q;

    wire [31:0] pte_ppn_w   = {`PAGE_PFN_SHIFT'b0, resp_data_w[31:`PAGE_PFN_SHIFT]};
    wire [9:0]  pte_flags_w = resp_data_w[9:0];

    always @ (posedge clk_i or posedge rst_i)
    if (rst_i)
    begin
        pte_addr_q  <= 32'b0;
        pte_entry_q <= 32'b0;
        virt_addr_q <= 32'b0;
        dtlb_req_q  <= 1'b0;
        state_q     <= STATE_IDLE;
    end
    else
    begin
        // TLB miss, walk page table
        if (state_q == STATE_IDLE && (itlb_miss_w || dtlb_miss_w))
        begin
            pte_addr_q  <= ptbr_w + {20'b0, request_addr_w[31:22], 2'b0};
            virt_addr_q <= request_addr_w;
            dtlb_req_q  <= dtlb_miss_w;

            state_q     <= STATE_LEVEL_FIRST;
        end
        // First level (4MB superpage)
        else if (state_q == STATE_LEVEL_FIRST && resp_valid_w)
        begin
            // Error or page not present
            if (resp_error_w || !resp_data_w[`PAGE_PRESENT])
            begin
                pte_entry_q <= 32'b0;
                state_q     <= STATE_UPDATE;
            end
            // Valid entry, but another level to fetch
            else if (!(resp_data_w[`PAGE_READ] || resp_data_w[`PAGE_WRITE] || resp_data_w[`PAGE_EXEC]))
            begin
                pte_addr_q  <= {resp_data_w[29:10], 12'b0} + {20'b0, request_addr_w[21:12], 2'b0};
                state_q     <= STATE_LEVEL_SECOND;
            end
            // Valid entry, actual valid PTE
            else
            begin
                pte_entry_q <= ((pte_ppn_w | {22'b0, request_addr_w[21:12]}) << `MMU_PGSHIFT) | {22'b0, pte_flags_w};
                state_q     <= STATE_UPDATE;
            end
        end
        // Second level (4KB page)
        else if (state_q == STATE_LEVEL_SECOND && resp_valid_w)
        begin
            // Valid entry, final level
            if (resp_data_w[`PAGE_PRESENT])
            begin
                pte_entry_q <= (pte_ppn_w << `MMU_PGSHIFT) | {22'b0, pte_flags_w};
                state_q     <= STATE_UPDATE;
            end
            // Page fault
            else
            begin
                pte_entry_q <= 32'b0;
                state_q     <= STATE_UPDATE;
            end
        end
        else if (state_q == STATE_UPDATE)
        begin
            state_q    <= STATE_IDLE;
        end
    end

    //-----------------------------------------------------------------
    // IMMU TLB
    //-----------------------------------------------------------------
    // 指令TLB（ITLB）：单项直接映射
    // itlb_valid_q：TLB项有效位
    // itlb_va_addr_q：TLB保存的虚拟地址页号（VA[31:12]）
    // itlb_entry_q：TLB保存的PTE（包含物理页号和权限标志）
    // itlb_hit_w：TLB命中条件：有效且虚拟页号匹配
    reg         itlb_valid_q;
    reg [31:12] itlb_va_addr_q;
    reg [31:0]  itlb_entry_q;

    always @ (posedge clk_i or posedge rst_i)
    if (rst_i)
        itlb_valid_q <= 1'b0;
    else if (flush_i)
        itlb_valid_q <= 1'b0;
    else if (state_q == STATE_UPDATE && !dtlb_req_q)
        itlb_valid_q <= (itlb_va_addr_q == fetch_in_pc_i[31:12]); // Fetch TLB still matches incoming request
    else if (state_q != STATE_IDLE && !dtlb_req_q)
        itlb_valid_q <= 1'b0;

    always @ (posedge clk_i or posedge rst_i)
    if (rst_i)
    begin
        itlb_va_addr_q <= 20'b0;
        itlb_entry_q   <= 32'b0;
    end
    else if (state_q == STATE_UPDATE && !dtlb_req_q)
    begin
        itlb_va_addr_q <= virt_addr_q[31:12];
        itlb_entry_q   <= pte_entry_q;
    end

    // TLB address matched (even on page fault)
    assign itlb_hit_w   = fetch_in_rd_i & itlb_valid_q & (itlb_va_addr_q == fetch_in_pc_i[31:12]);

    reg pc_fault_r;
    always @ *
    begin
        pc_fault_r = 1'b0;

        // 指令页访问权限检查：
        // - Supervisor模式：不允许访问用户页；需要PAGE_EXEC权限
        // - User模式：需要PAGE_EXEC且PAGE_USER权限
        if (vm_i_enable_w && itlb_hit_w)
        begin
            // Supervisor mode
            if (supervisor_i_w)
            begin
                // User page, supervisor cannot execute
                if (itlb_entry_q[`PAGE_USER])
                    pc_fault_r = 1'b1;
                // Check exec permissions
                else
                    pc_fault_r = ~itlb_entry_q[`PAGE_EXEC];
            end
            // User mode
            else
                pc_fault_r = (~itlb_entry_q[`PAGE_EXEC]) | (~itlb_entry_q[`PAGE_USER]);
        end
    end

    reg pc_fault_q;

    always @ (posedge clk_i or posedge rst_i)
    if (rst_i)
        pc_fault_q <= 1'b0;
    else
        pc_fault_q <= pc_fault_r;

    assign fetch_out_rd_o         = (~vm_i_enable_w & fetch_in_rd_i) || (itlb_hit_w & ~pc_fault_r);
    assign fetch_out_pc_o         = vm_i_enable_w ? {itlb_entry_q[31:12], fetch_in_pc_i[11:0]} : fetch_in_pc_i;
    assign fetch_out_flush_o      = fetch_in_flush_i;
    assign fetch_out_invalidate_o = fetch_in_invalidate_i; // TODO: ...

    assign fetch_in_accept_o      = (~vm_i_enable_w & fetch_out_accept_i) | (vm_i_enable_w & itlb_hit_w & fetch_out_accept_i) | pc_fault_r;
    assign fetch_in_valid_o       = fetch_out_valid_i | pc_fault_q;
    assign fetch_in_error_o       = fetch_out_valid_i & fetch_out_error_i;
    assign fetch_in_fault_o       = pc_fault_q;
    assign fetch_in_inst_o        = fetch_out_inst_i;

    //-----------------------------------------------------------------
    // DMMU TLB
    //-----------------------------------------------------------------
    // 数据TLB（DTLB）：单项直接映射
    // dtlb_valid_q：TLB项有效位
    // dtlb_va_addr_q：TLB保存的虚拟地址页号
    // dtlb_entry_q：TLB保存的PTE
    reg         dtlb_valid_q;
    reg [31:12] dtlb_va_addr_q;
    reg [31:0]  dtlb_entry_q;

    always @ (posedge clk_i or posedge rst_i)
    if (rst_i)
        dtlb_valid_q <= 1'b0;
    else if (flush_i)
        dtlb_valid_q <= 1'b0;
    else if (state_q == STATE_UPDATE && dtlb_req_q)
        dtlb_valid_q <= 1'b1;

    always @ (posedge clk_i or posedge rst_i)
    if (rst_i)
    begin
        dtlb_va_addr_q <= 20'b0;
        dtlb_entry_q   <= 32'b0;
    end
    else if (state_q == STATE_UPDATE && dtlb_req_q)
    begin
        dtlb_va_addr_q <= virt_addr_q[31:12];
        dtlb_entry_q   <= pte_entry_q;
    end

    // TLB address matched (even on page fault)
    assign dtlb_hit_w   = dtlb_valid_q & (dtlb_va_addr_q == lsu_addr_w[31:12]);

    reg load_fault_r;
    always @ *
    begin
        load_fault_r = 1'b0;

        // 数据加载页访问权限检查：
        // - Supervisor模式：不允许访问用户页（除非sum_i=1）；需要READ或（mxr_i && EXEC）权限
        // - User模式：需要READ且USER权限
        if (vm_d_enable_w && load_w && dtlb_hit_w)
        begin
            // Supervisor mode
            if (supervisor_d_w)
            begin
                // User page, supervisor user mode not enabled
                if (dtlb_entry_q[`PAGE_USER] && !sum_i)
                    load_fault_r = 1'b1;
                // Check exec permissions
                else
                    load_fault_r = ~(dtlb_entry_q[`PAGE_READ] | (mxr_i & dtlb_entry_q[`PAGE_EXEC]));
            end
            // User mode
            else
                load_fault_r = (~dtlb_entry_q[`PAGE_READ]) | (~dtlb_entry_q[`PAGE_USER]);
        end
    end

    reg store_fault_r;
    always @ *
    begin
        store_fault_r = 1'b0;

        // 数据存储页访问权限检查：
        // - Supervisor模式：不允许访问用户页（除非sum_i=1）；需要READ且WRITE权限
        // - User模式：需要READ且WRITE且USER权限
        if (vm_d_enable_w && (|store_w) && dtlb_hit_w)
        begin
            // Supervisor mode
            if (supervisor_d_w)
            begin
                // User page, supervisor user mode not enabled
                if (dtlb_entry_q[`PAGE_USER] && !sum_i)
                    store_fault_r = 1'b1;
                // Check exec permissions
                else
                    store_fault_r = (~dtlb_entry_q[`PAGE_READ]) | (~dtlb_entry_q[`PAGE_WRITE]);
            end
            // User mode
            else
                store_fault_r = (~dtlb_entry_q[`PAGE_READ]) | (~dtlb_entry_q[`PAGE_WRITE]) | (~dtlb_entry_q[`PAGE_USER]);
        end
    end

    reg store_fault_q;
    reg load_fault_q;

    always @ (posedge clk_i or posedge rst_i)
    if (rst_i)
        store_fault_q <= 1'b0;
    else
        store_fault_q <= store_fault_r;

    always @ (posedge clk_i or posedge rst_i)
    if (rst_i)
        load_fault_q <= 1'b0;
    else
        load_fault_q <= load_fault_r;   

    // 地址翻译：将虚拟地址翻译为物理地址
    // vm_d_enable_w=1时：使用DTLB中的物理页号（PA[31:12]）与页内偏移（VA[11:0]）拼接
    // lsu_out_cacheable_r：根据翻译后的物理地址判断是否在可缓存地址范围内
    //   （invalidate/writeback/flush操作始终标记为可缓存）
    wire        lsu_out_rd_w         = vm_d_enable_w ? (load_w  & dtlb_hit_w & ~load_fault_r)       : lsu_in_rd_i;
    wire [3:0]  lsu_out_wr_w         = vm_d_enable_w ? (store_w & {4{dtlb_hit_w & ~store_fault_r}}) : lsu_in_wr_i;
    wire [31:0] lsu_out_addr_w       = vm_d_enable_w ? {dtlb_entry_q[31:12], lsu_addr_w[11:0]}      : lsu_addr_w;
    wire [31:0] lsu_out_data_wr_w    = lsu_in_data_wr_i;

    wire        lsu_out_invalidate_w = lsu_in_invalidate_i;
    wire        lsu_out_writeback_w  = lsu_in_writeback_i;

    reg         lsu_out_cacheable_r;
    always @ *
    begin
/* verilator lint_off UNSIGNED */
/* verilator lint_off CMPCONST */
        if (lsu_in_invalidate_i || lsu_in_writeback_i || lsu_in_flush_i)
            lsu_out_cacheable_r = 1'b1;
        else
            lsu_out_cacheable_r = (lsu_out_addr_w >= MEM_CACHE_ADDR_MIN && lsu_out_addr_w <= MEM_CACHE_ADDR_MAX);
/* verilator lint_on CMPCONST */
/* verilator lint_on UNSIGNED */
    end

    wire [10:0] lsu_out_req_tag_w    = lsu_in_req_tag_i;
    wire        lsu_out_flush_w      = lsu_in_flush_i;

    assign lsu_in_ack_o         = (lsu_out_ack_i & ~resp_mmu_w) | store_fault_q | load_fault_q;
    assign lsu_in_resp_tag_o    = lsu_out_resp_tag_i;
    assign lsu_in_error_o       = (lsu_out_error_i & ~resp_mmu_w) | store_fault_q | load_fault_q;
    assign lsu_in_data_rd_o     = lsu_out_data_rd_i;
    assign lsu_in_store_fault_o = store_fault_q;
    assign lsu_in_load_fault_o  = load_fault_q;

    assign lsu_in_accept_o      = (~vm_d_enable_w & cpu_accept_w) | (vm_d_enable_w & dtlb_hit_w & cpu_accept_w) | store_fault_r | load_fault_r;

    //-----------------------------------------------------------------
    // PTE Fetch Port
    //-----------------------------------------------------------------
    // mem_req_q：PTW需要通过DCache端口读取PTE时的请求信号
    // src_mmu_w：当前DCache端口请求来源：1=MMU PTW请求，0=CPU LSU请求
    // read_hold_q：DCache请求已发出但未被接受时锁存请求来源
    // 多路选择：PTW请求使用特殊标签[9:7]=3'b111，以便区分响应来源
    //   PTW请求地址=pte_addr_q（页表项地址），数据写使能为0（只读）
    reg mem_req_q;
    wire mmu_accept_w;

    always @ (posedge clk_i or posedge rst_i)
    if (rst_i)
        mem_req_q <= 1'b0;
    else if (state_q == STATE_IDLE && (itlb_miss_w || dtlb_miss_w))
        mem_req_q <= 1'b1;
    else if (state_q == STATE_LEVEL_FIRST && resp_valid_w && !resp_error_w && resp_data_w[`PAGE_PRESENT] && (!(resp_data_w[`PAGE_READ] || resp_data_w[`PAGE_WRITE] || resp_data_w[`PAGE_EXEC])))
        mem_req_q <= 1'b1;    
    else if (mmu_accept_w)
        mem_req_q <= 1'b0;

    //-----------------------------------------------------------------
    // Request Muxing
    //-----------------------------------------------------------------
    reg  read_hold_q;
    reg  src_mmu_q;
    wire src_mmu_w = read_hold_q ? src_mmu_q : mem_req_q;

    always @ (posedge clk_i or posedge rst_i)
    if (rst_i)
    begin
        read_hold_q  <= 1'b0;
        src_mmu_q    <= 1'b0;
    end
    else if ((lsu_out_rd_o || (|lsu_out_wr_o)) && !lsu_out_accept_i)
    begin
        read_hold_q  <= 1'b1;
        src_mmu_q    <= src_mmu_w;
    end
    else if (lsu_out_accept_i)
        read_hold_q  <= 1'b0;

    assign mmu_accept_w         = src_mmu_w  & lsu_out_accept_i;
    assign cpu_accept_w         = ~src_mmu_w & lsu_out_accept_i;

    assign lsu_out_rd_o         = src_mmu_w ? mem_req_q  : lsu_out_rd_w;
    assign lsu_out_wr_o         = src_mmu_w ? 4'b0       : lsu_out_wr_w;
    assign lsu_out_addr_o       = src_mmu_w ? pte_addr_q : lsu_out_addr_w;
    assign lsu_out_data_wr_o    = lsu_out_data_wr_w;

    assign lsu_out_invalidate_o = src_mmu_w ? 1'b0 : lsu_out_invalidate_w;
    assign lsu_out_writeback_o  = src_mmu_w ? 1'b0 : lsu_out_writeback_w;
    assign lsu_out_cacheable_o  = src_mmu_w ? 1'b1 : lsu_out_cacheable_r;
    assign lsu_out_req_tag_o    = src_mmu_w ? {1'b0, 3'b111, 7'b0} : lsu_out_req_tag_w;
    assign lsu_out_flush_o      = src_mmu_w ? 1'b0 : lsu_out_flush_w;

end
//-----------------------------------------------------------------
// No MMU support
//-----------------------------------------------------------------
// ---------------------------------------------------------------
// SUPPORT_MMU=0 直通模式：
// 所有信号直接透传，不进行地址翻译，无页表错误
// ICache侧：fetch_in_* <-> fetch_out_* 直接连接
// DCache侧：lsu_in_*   <-> lsu_out_*   直接连接
// ---------------------------------------------------------------
else
begin
    assign fetch_out_rd_o         = fetch_in_rd_i;
    assign fetch_out_pc_o         = fetch_in_pc_i;
    assign fetch_out_flush_o      = fetch_in_flush_i;
    assign fetch_out_invalidate_o = fetch_in_invalidate_i;
    assign fetch_in_accept_o      = fetch_out_accept_i;
    assign fetch_in_valid_o       = fetch_out_valid_i;
    assign fetch_in_error_o       = fetch_out_error_i;
    assign fetch_in_fault_o       = 1'b0;
    assign fetch_in_inst_o        = fetch_out_inst_i;

    assign lsu_out_rd_o           = lsu_in_rd_i;
    assign lsu_out_wr_o           = lsu_in_wr_i;
    assign lsu_out_addr_o         = lsu_in_addr_i;
    assign lsu_out_data_wr_o      = lsu_in_data_wr_i;
    assign lsu_out_invalidate_o   = lsu_in_invalidate_i;
    assign lsu_out_writeback_o    = lsu_in_writeback_i;
    assign lsu_out_cacheable_o    = lsu_in_cacheable_i;
    assign lsu_out_req_tag_o      = lsu_in_req_tag_i;
    assign lsu_out_flush_o        = lsu_in_flush_i;
    
    assign lsu_in_ack_o           = lsu_out_ack_i;
    assign lsu_in_resp_tag_o      = lsu_out_resp_tag_i;
    assign lsu_in_error_o         = lsu_out_error_i;
    assign lsu_in_data_rd_o       = lsu_out_data_rd_i;
    assign lsu_in_store_fault_o   = 1'b0;
    assign lsu_in_load_fault_o    = 1'b0;

    assign lsu_in_accept_o        = lsu_out_accept_i;
end
endgenerate

endmodule
