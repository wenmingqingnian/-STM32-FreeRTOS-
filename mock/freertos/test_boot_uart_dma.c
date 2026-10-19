// SPDX-License-Identifier: GPL-2.0
/*
 * Host-side mock test for the Boot UART DMA ring receiver

 *
 * 被测：BOOT/boot_uart_dma.c（**未修改的生产源码**）。
 * 本套件补齐了这条链此前**唯一没有 host 单测**的模块。
 *
 * 覆盖（按"为什么值得测"排序）：
 *   1. 配置契约：DMA1_Channel5 必须是**循环模式**、缓冲 = 环容量 1024 B、
 *      两条 NVIC（USART1 / DMA1_Channel5）都配好。循环模式是"无需 ISR 重装
 *      NDTR"这条设计理由的物质前提，配错就无声退化成 Normal 模式。
 *   2. 单圈推进 + 单段跨度：Pending() 的 span0/len0 与 Consume() 的推进。
 *   3. **跨环尾两段拼接的字节顺序**：由 mock 把 DMA_MemoryBaseAddr 捕获后
 *      直接向环写"已被搬进来的字节"，再用 span0+span1 读回比对位置模式，
 *      证明两段拼接与写入顺序完全一致。
 *   4. overrun 粘滞语义 + Pending() 饱和 + ResetRx() 排空。
 *   5. ⚠️ **已知局限固化**：位置回跳法只能分辨一圈——整圈被漏、多圈少计、
 *      且**都不置 overrun（静默）**。把这三条写成断言，而不是留在注释里。
 *   6. **逐页轮询不变量（P0 的锁）**：模拟"擦 120 页、每页窗口 460 B、每页
 *      轮询一次"，断言位次推算精确、零 overrun；并用反证案例（把整个槽区
 *      擦除当成一个窗口）断言回跳法会静默少计——这就是 boot_update.c 的
 *      tio_erase_page 里那句 Boot_UartDma_Poll() 不可删的原因。
 *   7. ISR 契约：只清标志 + 计数，且**不推进位置/不置溢出/不消费数据**。
 *（"读 SR 再读 DR"的 F1 清序列无法被 mock 观测，不造假断言，见 s7 注释。）
 *   8. TX 契约：轮询发送逐字节等 TXE、帧尾等 TC。
 *
 * 不覆盖（诚实边界）：
 *   - 真机 DMA 行为本身（NDTR 硬件递减/重装、APB 门控、擦写期间总线停摆）
 *     只能上板验证；本套件的 DMA 是行为模型。
 *   - "读 SR 再读 DR"的清标志序列（成员读不可拦截）→ 由源码审查保证。
 *   - TC 中断与位置推导的一致性（s_tcCount 无 getter，见 P1 待办）。
 *   - 擦页路径本体：本套件断言的是"逐页轮询窗口 ≤ 460 B 时回跳法成立"，
 *     而"boot_update.c 真的每页都调了 Poll"属 boot_update.c 的契约，
 *     该文件无 host 单测（依赖 SPL/ARMCC），只能靠代码审查 + Keil 编译。
 *
 * 链接集证明：本套件**不链接任何 FreeRTOS / SPL 源**，只链 boot_uart_dma.c，
 * 故"ISR 不调 FreeRTOS API / 不 printf"由链接集本身成立。
 */

#include <stdio.h>
#include <string.h>
#include "boot_uart_dma.h"

/* ISR 入口：生产侧在 stm32f10x_it.h 里声明，本套件不链真机中断头，
 * 故在此显式声明（否则隐式声明会与 -Wall 冲突）。 */
void USART1_IRQHandler(void);
void DMA1_Channel5_IRQHandler(void);

static int g_failed = 0;
static int g_total = 0;

#define CHECK_DESC(desc, cond) do {                        \
    g_total++;                                             \
    if (cond) { printf("  ok %s\n", desc); }               \
    else      { g_failed++; printf("  FAIL %s\n", desc); } \
} while (0)

#define CHECK_EQ_DESC(desc, got, want) do {                \
    g_total++;                                             \
    if ((unsigned long)(got) == (unsigned long)(want))     \
        printf("  ok %s\n", desc);                         \
    else { g_failed++;                                     \
        printf("  FAIL %s: got %lu, want %lu\n",           \
               desc, (unsigned long)(got),                 \
               (unsigned long)(want)); }                   \
} while (0)

/*===========================================================================
 * mock 实例（声明在 stub/stm32f10x.h）
 *===========================================================================*/
USART_TypeDef USART1_MOCK_INSTANCE;
DMA_Channel_TypeDef DMA1_Channel5_MOCK_INSTANCE;
GPIO_TypeDef GPIOA_MOCK_INSTANCE;   /* boot_uart_dma.c 只用到 GPIOA */

/* 脚本化状态：测试体可随时改写，用来驱动 ISR 与发送路径 */
static ITStatus    g_usart_it_idle;      /* USART_GetITStatus(IDLE) 的返回值 */
static FlagStatus  g_usart_err_pending;  /* ORE/NE/FE/PE 任一挂起 */
static FlagStatus  g_usart_txe = SET;
static FlagStatus  g_usart_tc  = SET;
static ITStatus    g_dma_tc;
static ITStatus    g_dma_te;

/* 观测计数 */
static int g_nvic_inits;
static int g_usart_tx_cnt;
static int g_dma_clear_cnt;

/*===========================================================================
 * SPL mock 体（最小：只让未修改的生产代码能链接与观测）
 *===========================================================================*/
void RCC_APB2PeriphClockCmd(u32 periph, FunctionalState state)
{
    (void)periph; (void)state;
}

void RCC_AHBPeriphClockCmd(u32 periph, FunctionalState state)
{
    (void)periph; (void)state;
}

void GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    (void)port; (void)init;
}

void NVIC_Init(NVIC_InitTypeDef *init)
{
    (void)init;
    g_nvic_inits++;
}

void USART_Init(USART_TypeDef *usart, USART_InitTypeDef *init)
{
    (void)usart; (void)init;
}

void USART_DMACmd(USART_TypeDef *usart, u16 req, FunctionalState state)
{
    (void)usart; (void)req; (void)state;
}

void USART_ITConfig(USART_TypeDef *usart, u16 it, FunctionalState state)
{
    (void)usart; (void)it; (void)state;
}

void USART_Cmd(USART_TypeDef *usart, FunctionalState state)
{
    (void)usart; (void)state;
}

void USART_SendData(USART_TypeDef *usart, u16 data)
{
    (void)usart; (void)data;
    g_usart_tx_cnt++;
}

ITStatus USART_GetITStatus(USART_TypeDef *usart, u16 it)
{
    (void)usart;
    return (it == USART_IT_IDLE) ? g_usart_it_idle : RESET;
}

FlagStatus USART_GetFlagStatus(USART_TypeDef *usart, u16 flag)
{
    (void)usart;
    switch (flag) {
    case USART_FLAG_TXE: return g_usart_txe;
    case USART_FLAG_TC:  return g_usart_tc;
    default:
        /* 错误标志：由 g_usart_err_pending 统一驱动，ISR 里是"任一挂起即计数" */
        return g_usart_err_pending;
    }
}

void DMA_DeInit(DMA_Channel_TypeDef *ch)
{
    memset(ch, 0, sizeof(*ch));
}

void DMA_Init(DMA_Channel_TypeDef *ch, DMA_InitTypeDef *init)
{
    ch->CPAR  = init->DMA_PeripheralBaseAddr;
    ch->CMAR  = init->DMA_MemoryBaseAddr;   /* ⚠️ 生产代码把它截断成 u32，见下 */
    ch->CCR   = init->DMA_Mode;
    ch->CNDTR = (u16)init->DMA_BufferSize;  /* 真机：使能时 NDTR 载入缓冲长度 */
}

void DMA_ITConfig(DMA_Channel_TypeDef *ch, u32 it, FunctionalState state)
{
    (void)ch; (void)it; (void)state;
}

void DMA_Cmd(DMA_Channel_TypeDef *ch, FunctionalState state)
{
    (void)ch; (void)state;
}

u16 DMA_GetCurrDataCounter(DMA_Channel_TypeDef *ch)
{
    return ch->CNDTR;
}

ITStatus DMA_GetITStatus(u32 dma_it)
{
    if (dma_it == DMA1_IT_TC5) { return g_dma_tc; }
    if (dma_it == DMA1_IT_TE5) { return g_dma_te; }
    return RESET;
}

void DMA_ClearITPendingBit(u32 dma_it)
{
    (void)dma_it;
    g_dma_clear_cnt++;
}

/*===========================================================================
 * 生产端模拟（"DMA 把字节搬进环"）
 *
 * 逐字节内容 = pat(绝对序号)，因此**不需要影子缓冲**：任何一段跨度都能
 * 用绝对序号重新算出期望值，从而验证跨环尾拼接的顺序正确性。
 *===========================================================================*/
#define RING  ((u32)BOOT_UART_DMA_RING_SIZE)

static u32 g_written;   /* 已"被 DMA 搬进环"的字节数（真值，含被覆盖的） */
static u8 *g_ring;      /* 环基址，见 ring_probe() */

static u8 pat(u32 abs)
{
    return (u8)((abs * 37u + 11u) & 0xFFu);
}

/*
 * 环基址获取：不能读 DMA1_Channel5->CMAR —— 生产代码写的是
 * (uint32_t)s_ring，64 位宿主上已被截断。改用公开 API：
 * Pending() 在**空环**时返回 *span0 = s_ring（boot_uart_dma.c 的 pending==0 分支），
 * 这就是精确的环基址，且不依赖宿主字长。
 */
static u8 *ring_probe(void)
{
    const u8 *s0 = NULL;
    const u8 *s1 = NULL;
    u16 l0 = 0u;
    u16 l1 = 0u;

    (void)Boot_UartDma_Pending(&s0, &l0, &s1, &l1);
    return (u8 *)s0;
}

/* 搬运 n 字节进入环；CNDTR 按循环模式语义更新（到 0 由硬件立即重装） */
static void dma_push(u16 n)
{
    u16 i;

    for (i = 0u; i < n; i++) {
        g_ring[g_written % RING] = pat(g_written);
        g_written++;
    }
    DMA1_Channel5->CNDTR = (u16)(RING - (g_written % RING));
    if (DMA1_Channel5->CNDTR == 0u) {
        DMA1_Channel5->CNDTR = (u16)RING;   /* 观察者看到的是重装后的值 */
    }
}

/* 一次"到达 + 轮询"：模拟生产代码里间隔足够小的采样密度 */
static void dma_push_poll(u16 n)
{
    dma_push(n);
    Boot_UartDma_Poll();
}

/* 校验 [s0|l0] + [s1|l1] 拼接后的内容 == 从 absFrom 起的位置模式 */
static int content_ok(const u8 *s0, u16 l0, const u8 *s1, u16 l1, u32 absFrom)
{
    u16 i;
    u32 k = absFrom;

    for (i = 0u; i < l0; i++) {
        if (s0[i] != pat(k)) { return 0; }
        k++;
    }
    for (i = 0u; i < l1; i++) {
        if (s1[i] != pat(k)) { return 0; }
        k++;
    }
    return 1;
}

/* 每个场景开始前复位：Init 会把 prodAbs/consAbs/overrun 全部归零 */
static void scenario_begin(void)
{
    g_written = 0u;
    Boot_UartDma_Init();
    g_ring = ring_probe();
}

static u16 pending_now(const u8 **s0, u16 *l0, const u8 **s1, u16 *l1)
{
    return Boot_UartDma_Pending(s0, l0, s1, l1);
}

/*===========================================================================
 * 1. 配置契约（循环模式是"无需 ISR 重装 NDTR"的物质前提）
 *===========================================================================*/
static void s1_config_contract(void)
{
    const u8 *s0 = NULL;
    const u8 *s1 = NULL;
    u16 l0 = 0u;
    u16 l1 = 0u;

    printf("[s1] 配置契约\n");
    g_nvic_inits = 0;
    scenario_begin();

    CHECK_DESC("环基址可用（Pending 空环返回 s_ring）", g_ring != NULL);
    CHECK_EQ_DESC("环容量 = 1024 B", RING, 1024u);
    CHECK_DESC("DMA 配成循环模式（无需 ISR 重装 NDTR）",
               DMA1_Channel5->CCR == DMA_Mode_Circular);
    CHECK_EQ_DESC("DMA 缓冲长度 = 环容量", DMA1_Channel5->CNDTR, RING);
    CHECK_DESC("DMA 目标基址已写入", DMA1_Channel5->CMAR != 0u);
    CHECK_EQ_DESC("USART1 / DMA1_Channel5 两条 NVIC 都配置", g_nvic_inits, 2);

#if UINTPTR_MAX > 0xFFFFFFFFu
    printf("  -- 64 位宿主：DMA_MemoryBaseAddr 被生产代码截断为 u32，"
           "故基址一致性不在此断言（环基址改由 ring_probe() 取）\n");
#else
    CHECK_EQ_DESC("DMA 目标基址 == 环基址",
                  (unsigned long)DMA1_Channel5->CMAR,
                  (unsigned long)(uintptr_t)g_ring);
#endif

    CHECK_EQ_DESC("初始待消费 = 0", pending_now(&s0, &l0, &s1, &l1), 0u);
    CHECK_DESC("初始 overrun = 0", Boot_UartDma_Overrun() == 0u);
    CHECK_EQ_DESC("初始 IDLE 计数 = 0", Boot_UartDma_GetIdleCount(), 0u);
    CHECK_EQ_DESC("初始错误计数 = 0", Boot_UartDma_GetErrorCount(), 0u);
}

/*===========================================================================
 * 2. 单圈推进 + 单段跨度
 *===========================================================================*/
static void s2_single_span(void)
{
    const u8 *s0 = NULL;
    const u8 *s1 = NULL;
    u16 l0 = 0u;
    u16 l1 = 0u;

    printf("[s2] 单圈推进与单段跨度\n");
    scenario_begin();

    dma_push_poll(100u);
    CHECK_EQ_DESC("待消费 = 100", pending_now(&s0, &l0, &s1, &l1), 100u);
    CHECK_DESC("span0 起于环首（消费者位置 0）", s0 == g_ring);
    CHECK_EQ_DESC("len0 = 100", l0, 100u);
    CHECK_DESC("不跨环尾时 span1 = NULL", s1 == NULL);
    CHECK_EQ_DESC("跨环尾时 len1 = 0", l1, 0u);
    CHECK_DESC("内容与写入顺序一致", content_ok(s0, l0, s1, l1, 0u));

    Boot_UartDma_Consume(100u);
    CHECK_EQ_DESC("消费后待消费 = 0", pending_now(&s0, &l0, &s1, &l1), 0u);
    CHECK_DESC("未触发 overrun", Boot_UartDma_Overrun() == 0u);

    dma_push_poll(200u);
    CHECK_EQ_DESC("再到达 200 后待消费 = 200", pending_now(&s0, &l0, &s1, &l1), 200u);
    CHECK_DESC("span0 起于环内偏移 100", s0 == &g_ring[100u]);
    CHECK_DESC("内容按绝对序号连续", content_ok(s0, l0, s1, l1, 100u));
    Boot_UartDma_Consume(200u);
}

/*===========================================================================
 * 3. 跨环尾两段拼接（本套件的核心价值之一）
 *===========================================================================*/
static void s3_two_spans(void)
{
    const u8 *s0 = NULL;
    const u8 *s1 = NULL;
    u16 l0 = 0u;
    u16 l1 = 0u;

    printf("[s3] 跨环尾拆两段与字节顺序\n");
    scenario_begin();

    /* 消费者停在 1000，生产者继续写 100 字节 -> 环内位置 1000..1023 与 0..75 */
    dma_push_poll(1000u);
    CHECK_EQ_DESC("先积满 1000", pending_now(&s0, &l0, &s1, &l1), 1000u);
    Boot_UartDma_Consume(1000u);

    dma_push_poll(100u);
    CHECK_EQ_DESC("跨环尾待消费 = 100", pending_now(&s0, &l0, &s1, &l1), 100u);
    CHECK_EQ_DESC("第一段长度 = 24（到环尾）", l0, 24u);
    CHECK_EQ_DESC("第二段长度 = 76（绕回环首）", l1, 76u);
    CHECK_DESC("span0 指向环内 1000", s0 == &g_ring[1000u]);
    CHECK_DESC("span1 指向环首", s1 == g_ring);
    CHECK_DESC("两段拼接后与写入顺序完全一致",
               content_ok(s0, l0, s1, l1, 1000u));

    Boot_UartDma_Consume(100u);
    CHECK_EQ_DESC("消费后待消费 = 0", pending_now(&s0, &l0, &s1, &l1), 0u);
    CHECK_DESC("跨环尾不触发 overrun", Boot_UartDma_Overrun() == 0u);
}

/*===========================================================================
 * 4. overrun 粘滞 + Pending 饱和 + ResetRx 排空
 *===========================================================================*/
static void s4_overrun(void)
{
    const u8 *s0 = NULL;
    const u8 *s1 = NULL;
    u16 l0 = 0u;
    u16 l1 = 0u;

    printf("[s4] overrun 粘滞与排空\n");
    scenario_begin();

    /* 每步 <= 460 B，保证位置回跳法可分辨（真机逐页轮询的窗口） */
    dma_push_poll(460u); dma_push_poll(460u); dma_push_poll(80u);
    CHECK_EQ_DESC("累计 1000 字节精确入账", pending_now(&s0, &l0, &s1, &l1), 1000u);
    Boot_UartDma_Consume(1000u);
    CHECK_DESC("消费后无 overrun", Boot_UartDma_Overrun() == 0u);

    /* 生产者套圈：待消费 1030 > 1024 */
    dma_push_poll(460u); dma_push_poll(460u); dma_push_poll(110u);
    CHECK_DESC("套圈后置粘滞 overrun", Boot_UartDma_Overrun() == 1u);
    CHECK_EQ_DESC("Pending 饱和为整环", pending_now(&s0, &l0, &s1, &l1), RING);

    Boot_UartDma_Consume(500u);
    CHECK_DESC("消费部分后 overrun 仍为 1（粘滞）", Boot_UartDma_Overrun() == 1u);

    Boot_UartDma_ResetRx();
    CHECK_DESC("ResetRx 清 overrun", Boot_UartDma_Overrun() == 0u);
    CHECK_EQ_DESC("ResetRx 后待消费 = 0（排空）",
                  pending_now(&s0, &l0, &s1, &l1), 0u);
}

/*===========================================================================
 * 5. ⚠️ 已知局限固化：回跳法只能分辨一圈，且失效时**不置 overrun**
 *
 * 这三条断言是"必须逐页轮询"的论据，也是 P0 修复的回归保护：
 * 一旦有人删掉 boot_update.c 里的逐页 Poll，s6 的不变量就没人守了。
 *===========================================================================*/
static void s5_known_limits(void)
{
    const u8 *s0 = NULL;
    const u8 *s1 = NULL;
    u16 l0 = 0u;
    u16 l1 = 0u;

    printf("[s5] 已知局限（静默少计/漏计）\n");

    /* 5A. 恰好整圈：环内位置回到上次快照 -> 判为"无变化"（等号歧义） */
    scenario_begin();
    dma_push_poll(1000u);                  /* 先落一个非零快照（s_prodPos = 1000） */
    Boot_UartDma_Consume(1000u);           /* 消费者追上，环内位置也到 1000 */
    dma_push(RING);                        /* 单次窗口恰好一整圈 -> 回到 1000 */
    Boot_UartDma_Poll();
    CHECK_EQ_DESC("5A 恰好整圈被漏：待消费仍为 0",
                  pending_now(&s0, &l0, &s1, &l1), 0u);
    CHECK_DESC("5A 且未置 overrun（静默）", Boot_UartDma_Overrun() == 0u);

    /* 5B. 多圈：只记"末圈余数"，其余静默丢弃 */
    scenario_begin();
    dma_push(2500u);                       /* 2500 = 2*1024 + 452 */
    Boot_UartDma_Poll();
    CHECK_EQ_DESC("5B 多圈只记余数 452", pending_now(&s0, &l0, &s1, &l1), 452u);
    CHECK_DESC("5B 且未置 overrun（静默少计 2048）",
               Boot_UartDma_Overrun() == 0u);

    /* 5C. 刚过一圈多 1 字节：最恶劣——只记 1，1024 字节静默蒸发 */
    scenario_begin();
    dma_push(1025u);
    Boot_UartDma_Poll();
    CHECK_EQ_DESC("5C 一圈零 1 字节只记 1", pending_now(&s0, &l0, &s1, &l1), 1u);
    CHECK_DESC("5C 且未置 overrun（静默）", Boot_UartDma_Overrun() == 0u);
}

/*===========================================================================
 * 6. 逐页轮询不变量（P0 的锁）
 *
 * 真机参数：页擦除 20-40 ms（boot_update.c:66 注释），115200-8N1 = 11.52 B/ms
 *           -> 一页窗口最多约 460 B < 2048 B（2 KiB 页下更宽裕）。
 * 断言：在该密度下位次推算精确、零 overrun、字节顺序正确。
 *===========================================================================*/
#define PAGE_WINDOW_BYTES   460u
#define SLOT_PAGES          59u     /* 121856 B / FLASH_PAGE_SIZE(2048) = 59.5 -> 59，向下取整 */

static void s6_pagewise_invariant(void)
{
    const u8 *s0 = NULL;
    const u8 *s1 = NULL;
    u16 l0 = 0u;
    u16 l1 = 0u;
    u32 i;
    u32 total = 0u;
    int  order_ok = 1;
    int  overrun_seen = 0;

    printf("[s6] 逐页轮询不变量（P0 的锁）\n");
    scenario_begin();

    CHECK_DESC("单页窗口 < 整环，故单圈即可分辨",
               PAGE_WINDOW_BYTES < RING);

    for (i = 0u; i < SLOT_PAGES; i++) {
        dma_push_poll((u16)PAGE_WINDOW_BYTES);
        if (pending_now(&s0, &l0, &s1, &l1) != PAGE_WINDOW_BYTES) { order_ok = 0; }
        if (Boot_UartDma_Overrun() != 0u) { overrun_seen = 1; }
        if (!content_ok(s0, l0, s1, l1, total)) { order_ok = 0; }
        Boot_UartDma_Consume(PAGE_WINDOW_BYTES);
        total += PAGE_WINDOW_BYTES;
    }

    CHECK_DESC("59 次逐页轮询：每次待消费都精确 = 460", order_ok != 0);
    CHECK_DESC("59 次逐页轮询：累计无 overrun", overrun_seen == 0);
    CHECK_EQ_DESC("累计消费 = 59 * 460", total, SLOT_PAGES * PAGE_WINDOW_BYTES);
    CHECK_EQ_DESC("收尾待消费 = 0", pending_now(&s0, &l0, &s1, &l1), 0u);

    /* 反证：若把整个槽区擦除当成一个窗口（不逐页轮询），回跳法静默少计。
     * 27140 B 是"59 页 × 一页窗口 460 B"的总量；单窗口下只记 27140 % 2048 = 516。
     * 这就是 boot_update.c 的 tio_erase_page 里那句 Poll 不可删的原因。 */
    scenario_begin();
    dma_push(27140u);
    Boot_UartDma_Poll();
    CHECK_EQ_DESC("反证：整槽区单窗口下只记下末圈余数 516",
                  pending_now(&s0, &l0, &s1, &l1), 516u);
    CHECK_DESC("反证：且仍未置 overrun（静默丢失 26624 字节）",
               Boot_UartDma_Overrun() == 0u);
}

/*===========================================================================
 * 7. ISR 契约：只清标志 + 计数，不解析协议
 *===========================================================================*/
static void s7_isr_contract(void)
{
    const u8 *s0 = NULL;
    const u8 *s1 = NULL;
    u16 l0 = 0u;
    u16 l1 = 0u;

    printf("[s7] ISR 契约（只清标志与计数）\n");

    scenario_begin();
    g_usart_it_idle = RESET;
    g_usart_err_pending = RESET;
    g_dma_tc = RESET;
    g_dma_te = RESET;
    g_dma_clear_cnt = 0;

    /* 7.1 无标志：计数不变 */
    USART1_IRQHandler();
    CHECK_EQ_DESC("无标志时 IDLE 计数不变", Boot_UartDma_GetIdleCount(), 0u);

    /* 7.2 IDLE：只计数，不消费、不推进位置、不置溢出
     *
     * 注：源码里"读 SR 再读 DR"是 F1 清 IDLE 标志的参考手册序列，但 SR/DR 是
     * 普通寄存器成员，成员读不是函数调用，mock 无法观测读取次数 —— 该序列
     * 由源码审查保证，本套件【不】为它造一个假断言。
     * 能观测且更有意义的是"ISR 不做别的事"：位置与溢出标志均不得变化。 */
    dma_push_poll(200u);
    CHECK_EQ_DESC("ISR 前待消费 = 200", pending_now(&s0, &l0, &s1, &l1), 200u);

    g_usart_it_idle = SET;
    USART1_IRQHandler();
    CHECK_EQ_DESC("IDLE 计数 +1", Boot_UartDma_GetIdleCount(), 1u);
    CHECK_EQ_DESC("IDLE 中断不推进待消费位置",
                  pending_now(&s0, &l0, &s1, &l1), 200u);
    CHECK_DESC("IDLE 中断不置 overrun", Boot_UartDma_Overrun() == 0u);

    /* 7.3 接收错误：计数走 GetErrorCount（ORE/NE/FE/PE 任一挂起） */
    g_usart_it_idle = RESET;
    g_usart_err_pending = SET;
    USART1_IRQHandler();
    CHECK_EQ_DESC("错误计数 +1", Boot_UartDma_GetErrorCount(), 1u);
    CHECK_EQ_DESC("IDLE 计数不受影响", Boot_UartDma_GetIdleCount(), 1u);

    /* 7.4 DMA TC：只清标志（位置由轮询推导，不在这里推进） */
    g_dma_tc = SET;
    DMA1_Channel5_IRQHandler();
    CHECK_EQ_DESC("TC 清了 1 次标志位", g_dma_clear_cnt, 1);
    CHECK_EQ_DESC("TC 不改变错误计数", Boot_UartDma_GetErrorCount(), 1u);
    CHECK_EQ_DESC("TC 中断不推进待消费位置",
                  pending_now(&s0, &l0, &s1, &l1), 200u);

    /* 7.5 DMA TE：清标志并计入错误 */
    g_dma_tc = RESET;
    g_dma_te = SET;
    DMA1_Channel5_IRQHandler();
    CHECK_EQ_DESC("TE 也清标志", g_dma_clear_cnt, 2);
    CHECK_EQ_DESC("TE 计入错误计数", Boot_UartDma_GetErrorCount(), 2u);

    Boot_UartDma_Consume(200u);
}

/*===========================================================================
 * 8. TX 契约：轮询发送（逐字节等 TXE，帧尾等 TC）
 *===========================================================================*/
static void s8_tx_contract(void)
{
    static const u8 frame[4] = { 0x49u, 0x41u, 0x50u, 0x31u };   /* "IAP1" */
    static const u8 idle[1] = { 0x00u };

    printf("[s8] TX 契约\n");
    scenario_begin();

    g_usart_txe = SET;
    g_usart_tc  = SET;
    g_usart_tx_cnt = 0;

    Boot_UartDma_Send(frame, 4u);
    CHECK_EQ_DESC("4 字节全部推入 DR", g_usart_tx_cnt, 4);

    /* len=0 只等 TC 清空移位寄存器，不写 DR */
    Boot_UartDma_Send(idle, 0u);
    CHECK_EQ_DESC("len=0 不写 DR（只等 TC）", g_usart_tx_cnt, 4);

    /* 发送期间接收不受影响：DMA 独立搬运（此处断言环仍可正常收货） */
    dma_push_poll(300u);
    {
        const u8 *s0 = NULL; const u8 *s1 = NULL; u16 l0 = 0u; u16 l1 = 0u;
        CHECK_EQ_DESC("阻塞发送后环仍可收货", pending_now(&s0, &l0, &s1, &l1), 300u);
        Boot_UartDma_Consume(300u);
    }
}

/*===========================================================================
 * main
 *===========================================================================*/
int main(void)
{
    s1_config_contract();
    s2_single_span();
    s3_two_spans();
    s4_overrun();
    s5_known_limits();
    s6_pagewise_invariant();
    s7_isr_contract();
    s8_tx_contract();

    printf("boot-uart-dma: %d checks, %d failed\n", g_total, g_failed);
    if (g_failed == 0) {
        printf("boot-uart-dma: all cases passed\n");
        return 0;
    }
    printf("boot-uart-dma: some cases FAILED\n");
    return 1;
}
