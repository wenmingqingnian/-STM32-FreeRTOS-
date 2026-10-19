/*
 * Boot 决策主流程
 * + 镜像校验与无栈跳转（Task 9，设计文档 §5.5）
 *
 * 决策优先级（复位后只评一次，运行中不再改道）：
 *   1. 恢复键（KEY1/PB1）复位时保持按下——用户显式恢复意图：
 *      强制进入升级模式，无握手超时回退；
 *   2. BKP_DR2 升级请求（APP 侧 Task 10 写入）——
 *      a. 旧 APP 有效（元数据 CONTINUE）：升级模式，握手超时清请求后
 *         回退旧 APP；
 *      b. 旧 APP 不可信：升级模式，无回退（与 3b 同）；
 *   3. 无任何请求——
 *      a. 元数据 CONTINUE：直接跳 APP（Task 9 无栈跳转）；
 *      b. 元数据 REJECT（RECEIVING/无记录/歧义/字段非法）：强制停留
 *         升级模式，直到新镜像 VALID 并复位。
 *
 * 回退安全性（设计 §5.1"一旦 RECEIVING 已提交，不再退回"）：
 *   握手超时处不直接跳，而是重读双页元数据——RECEIVING 已提交时
 *   MetaDecision=REJECT，回退分支自然不触发；declare-before-erase
 *   保证擦 APP 之前 RECEIVING 必已落盘。
 *
 * 禁止事项（沿计划总体规则）：
 *   - 只初始化 Boot 必需外设（备份域/恢复键、SysTick、Flash/CRC、
 *     USART1+DMA1 CH5），不碰 LCD/电机/FreeRTOS；
 *   - 不写死分区地址（一律经 CONFIG/memory_layout.h 的各模块接口）；
 *   - 不引用 printf/半主机路径。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#include "boot_minimal.h"
#include "boot_request.h"
#include "boot_uart_dma.h"
#include "boot_update.h"
#include "boot_image.h"
#include "iap_meta.h"

/* stm32f10x.h（经 boot_minimal.h 链进入）与 memory_layout.h 都定义
 * FLASH_BASE 且同值不同形：先取消 SPL 版本，让布局头成为唯一权威定义，
 * 避免 ARMCC 宏重定义告警。两者恒等 0x08000000 由
 * tools/check_memory_layout.py 强制（与 boot_update.c 同一处理）。 */
#undef FLASH_BASE
#include "memory_layout.h"

static volatile Boot_Phase_t s_bootPhase = BOOT_PHASE_INIT;

/* boot_jump.s（Task 9）：无栈尾跳。R0 = APP 初始 MSP，R1 = Reset
 * Handler；写 MSP 后零栈操作（静态审查证明），本函数永不返回。 */
extern void Boot_JumpTail(u32 msp, u32 resetHandler);

/* 目标侧只读后端：内存映射直读半字（与 boot_update.c tio_read_halfword
 * 同语义；Boot_ImageValidate 的依赖注入接口，见 boot_image.h） */
static u16 jio_read_halfword(void *self, u32 addr)
{
    (void)self;
    return *(volatile u16 *)addr;
}

/* 校验配置：Flash 常量来自 memory_layout.h，SRAM 边界为芯片事实
 *（0x20000000 基址 + board_profile.h 容量，上界含 0x20005000）。
 *
 * 双槽改造：appBase 不再固化为 APP_BASE，而是在跳转前由
 * Boot_Update_ActiveSlot() 返回的活动槽填入（见 Boot_JumpToApp）。 */
static Boot_ImageConfig_t s_imgCfg =
{
    SLOT_A_BASE,                           /* appBase（运行期改写为活动槽） */
    SLOT_CAPACITY,                         /* appCapacity（两槽等长） */
    0x20000000UL,                          /* sramBase（Cortex-M3 SRAM 基址） */
    0x20000000UL + BOARD_SRAM_SIZE_BYTES   /* sramLimit（SRAM 顶，含） */
};

/*
 * 无栈跳转
 *
 * 三段式：
 *   1. 元数据决策 + 整镜像校验链（Boot_ImageValidate：MSP/PC/size/
 *      base/CRC）——任何一级失败本函数【返回】，调用点自然落入
 *      升级模式（正常启动分支的 FORCED 兜底 / 回退分支的继续等待）；
 *   2. C 侧 NVIC/SCB 清理：关中断 -> 停 Boot 自有中断源（SysTick/
 *      USART1/DMA1_CH5）-> 关闭并清除全部 NVIC 外部中断 -> 清
 *      PendSV/SysTick pending -> VTOR 指向 APP 向量表 -> DSB/ISB；
 *   3. Boot_JumpTail 尾跳（boot_jump.s）：清 BASEPRI/FAULTMASK ->
 *      切回 privileged MSP -> 写 APP MSP -> CPSIE i -> BX，不返回。
 *
 * 为什么校验在本函数而不在 main 决策处：跳转前一刻重读元数据，
 * 与决策点之间不存在擦写，双保险且失败路径统一。
 */
void Boot_JumpToApp(void)
{
    IAP_MetaRecord_t rec;
    Boot_ImageIo_t io;
    u32 slotBase;
    u32 msp;
    u32 pc;

    /*---- 1. 校验链（运行期活动槽）----*/
    if (Boot_Update_MetaDecision(&rec) != IAP_BOOT_CONTINUE)
    {
        return;   /* 元数据不再可信：停留升级模式 */
    }

    /* 活动槽 = 最新 VALID 记录指向的槽（Slot A 或 Slot B）。本处与
     * 上一行元数据决策读的是同一份记录、其间无擦写，故一致。 */
    slotBase = Boot_Update_ActiveSlot();
    if (slotBase == 0u)
    {
        return;   /* 理论不可达（CONTINUE 已保证槽合法）：保守返回 */
    }
    s_imgCfg.appBase = slotBase;   /* 校验链 BASE/向量/CRC 全按活动槽 */

    io.self = NULL;
    io.read_halfword = jio_read_halfword;
    if (Boot_ImageValidate(&rec, &io, &s_imgCfg) != BOOT_IMAGE_OK)
    {
        return;   /* 向量/CRC 非法：停留升级模式 */
    }

    /*---- 2. NVIC/SCB 清理（设计 §5.5 跳转前清单） ----*/
    /* 2.1 关全局中断：清理序列原子化（PRIMASK=1）*/
    __disable_irq();

    /* 2.2 停 Boot 自有中断源：SysTick 1ms 时基、USART1、RX DMA
     *（PWR/BKP 时钟保持——APP 侧自行决定是否复用备份域）*/
    SysTick->CTRL = 0u;
    DMA1_Channel5->CCR = 0u;
    USART1->CR1 = 0u;

    /* 2.3 关闭并清除全部 NVIC 外部中断（中容量 F103 共 43 路，
     *     ICER/ICPR 各写两字覆盖；未实现位写入被忽略）*/
    NVIC->ICER[0] = 0xFFFFFFFFu;
    NVIC->ICER[1] = 0xFFFFFFFFu;
    NVIC->ICPR[0] = 0xFFFFFFFFu;
    NVIC->ICPR[1] = 0xFFFFFFFFu;

    /* 2.4 清 PendSV / SysTick pending（ICSR 相应位写 1 清除）*/
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;

    /* 2.5 VTOR 指向活动槽向量表：CPSIE 之后到 APP SystemInit 再按
     *     本镜像编译期 VECT_TAB_OFFSET 重设 VTOR 之前的窗口内，
     *     任何异常入口取的都是正确槽的向量。
     *     【CPSIE 前的异常遮蔽】：2.1 起 PRIMASK=1，直到尾跳内
     *     CPSIE i 为止不响应任何（可屏蔽）异常，故不存在"VTOR 与
     *     向量表不匹配"的窗口；NMI/HardFault 由 APP 向量表首项
     *     兜底（尾跳后 VTOR 已就位）。 */
    SCB->VTOR = slotBase;

    /* 2.6 屏障：清 pending 先于屏障，保证跳转前清理全部生效 */
    __DSB();
    __ISB();

    /*---- 3. 取已校验向量，进入无栈尾跳（不返回） ----*/
    s_bootPhase = BOOT_PHASE_JUMP;
    msp = *(volatile u32 *)slotBase;
    pc = *(volatile u32 *)(slotBase + 4u);
    Boot_JumpTail(msp, pc);
}

Boot_Phase_t Boot_GetPhase(void)
{
    return s_bootPhase;
}

/*
 * Boot 主入口（裸机）：初始化 -> 一次性启动决策 -> 常驻循环。
 *
 * 初始化顺序约束：
 *   1. BootRequest_Init 最先——备份域时钟/权限必须在任何 BKP 读取之前，
 *      且不得重置备份域（否则 BKP_DR2 请求丢失，设计 §5.1）；
 *   2. Boot_Update_Init 建立 SysTick 1 ms 时基与 Flash/协议上下文；
 *   3. Boot_UartDma_Init 最后开 USART1/DMA 接收——初始化期间到达的
 *      线路噪声不入消费窗口（SessionBegin 还会再排空一次）。
 */
int main(void)
{
    IAP_BootAction_t action;
    Boot_UpdateEntry_t entry;
    u8 fallbackAllowed = 0u;

    BootRequest_Init();
    Boot_Update_Init();
    Boot_UartDma_Init();

    /*---- 一次性启动决策（优先级：恢复键 > BKP 请求 > 元数据） ----*/
    action = Boot_Update_MetaDecision(NULL);

    if (BootRequest_RecoveryKeyPressed() != 0u)
    {
        entry = BOOT_UPDATE_ENTRY_RECOVERY_KEY;
    }
    else if (BootRequest_IsRequested() != 0u)
    {
        if (action == IAP_BOOT_CONTINUE)
        {
            entry = BOOT_UPDATE_ENTRY_BKP_REQUEST;
            fallbackAllowed = 1u;
        }
        else
        {
            /* 有请求但旧 APP 不可信：停留升级模式直到新镜像 VALID */
            entry = BOOT_UPDATE_ENTRY_FORCED;
        }
    }
    else
    {
        if (action == IAP_BOOT_CONTINUE)
        {
            /* 无请求且镜像有效：正常启动（校验失败则返回落入升级模式） */
            Boot_JumpToApp();
        }
        entry = BOOT_UPDATE_ENTRY_FORCED;
    }

    /*---- 升级会话常驻循环 ----*/
    Boot_Update_SessionBegin(entry);
    s_bootPhase = BOOT_PHASE_WAIT_FRAME;

    for (;;)
    {
        /* 硬件 IWDG（option bytes 启用时）安全喂狗：只刷新计数，不启动 */
        IWDG->KR = 0xAAAAu;

        if (Boot_Update_SessionPoll() == BOOT_UPDATE_RUN_HANDSHAKE_TIMEOUT)
        {
            /* 握手超时仅 BKP 入口可到（引擎只对该入口开窗且已重启窗口）。
             * 回退前重读元数据：RECEIVING 已提交则 REJECT，不回退。 */
            if ((fallbackAllowed != 0u) &&
                (Boot_Update_MetaDecision(NULL) == IAP_BOOT_CONTINUE))
            {
                /* 清 BKP 请求：防止回退后下次复位再次落入升级模式 */
                BootRequest_Clear();
                Boot_JumpToApp();
            }
            /* 镜像不可信：继续等待（BKP 入口窗口已由引擎重启） */
        }
    }
}
