/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Host-side mock test for the EXTI-only STOP backend and the shared
 * after-STOP clock recovery.
 *
 * Production sources under test are compiled UNMODIFIED from the project
 * tree (BSP\LOWPOWER\exti_stop_backend.c + BSP\LOWPOWER\
 * system_clock_resume.c); sys.h comes from stub/.
 *
 * Link-set evidence baked into this suite:
 *   - exti_stop_backend.c is linked WITHOUT rtc_timebase.c and without any
 *     FreeRTOS source: the "EXTI-only path never sets an RTC Alarm" and
 *     "never calls vTaskStepTick()" claims would break the link or trip the
 *     counters below if violated.
 *   - The ExtiStop_Io_t interface itself has no alarm callback: the RTC
 *     alarm contract is enforced at compile time, not by convention.
 *
 * Covers:
 *   - backend re-checks the decision layer's confirmed inputs
 *     (eNoTasksWaitingTimeout + Watchdog_RequiresPeriodicService()==false)
 *     and refuses with zero IO side effects otherwise
 *   - enter sequence ordering: SysTick stop -> peripheral quiesce -> key EXTI
 *     wake config -> clear stale key pending (sleep race) -> PWR STOP mode
 *     (CWUF clear, PDDS=0 LPDS=1) -> DSB/ISB/WFI (design §6.6 enter 1/4/5/6/7)
 *   - exit sequence ordering: read EXTI pending (wake source) -> 72 MHz
 *     clock restore (full HSE/PLL/SWS chain) -> peripheral resume ->
 *     SysTick resume; vTaskStepTick stays at 0 (tick frozen by design §6.3)
 *   - clock-resume failure propagates: peripherals/SysTick are NOT resumed
 *     under HSI 8 MHz (fail-safe is the caller's job, design §6.6)
 *   - wake source classification: key line / other line / none
 *   - SystemClock_RestoreAfterStop: success ordering, HSE/PLL/SWS bounded
 *     timeouts (no later op after a failure), off-by-one timeout boundary
 */
#include <stdio.h>
#include <string.h>
#include "exti_stop_backend.h"

static int g_failed = 0;
static int g_total = 0;

#define CHECK_DESC(desc, cond) do {                          \
    g_total++;                                                \
    if (cond) { printf("  ok %s\n", desc); }                  \
    else      { g_failed++; printf("  FAIL %s\n", desc); }    \
} while (0)

#define CHECK_EQ_DESC(desc, got, want) do {                  \
    g_total++;                                                \
    if ((long)(got) == (long)(want))                          \
        printf("  ok %s\n", desc);                            \
    else { g_failed++;                                        \
        printf("  FAIL %s: got %ld, want %ld\n",              \
               desc, (long)(got), (long)(want)); }            \
} while (0)

/*-----------------------------------------------------------
 * FreeRTOS 桩：vTaskStepTick 禁止调用
 * 若 exti_stop_backend.o 引用该符号，会链接到本桩并计数 -> FAIL
 *----------------------------------------------------------*/
static u32 g_vtask_step_tick_calls = 0;
void vTaskStepTick(u32 ticks)
{
    (void)ticks;
    g_vtask_step_tick_calls++;
}

/*-----------------------------------------------------------
 * 脚本化 IO：EXTI/PWR/SysTick/外设 + 时钟恢复共享一个序号账本
 *----------------------------------------------------------*/
typedef enum {
    OP_SYSTICK_STOP = 1,
    OP_PERIPH_QUIESCE,
    OP_KEY_EXTI_CONFIG,
    OP_CLEAR_KEY_PENDING,
    OP_PWR_STOP_ENTER,
    OP_DSB_ISB_WFI,
    OP_READ_EXTI_PENDING,
    OP_PERIPH_RESUME,
    OP_SYSTICK_RESUME,

    OP_CLK_HSE_ON = 100,
    OP_CLK_FLASH_LATENCY,
    OP_CLK_BUS_PRESCALERS,
    OP_CLK_PLL_ENABLE,
    OP_CLK_SELECT_SYSCLK,
    OP_CLK_CORE_UPDATE,
    OP_CLK_WAIT_POLL
} FakeOp_t;

#define FAKE_ORDER_CAP  128

typedef struct {
    u32 order[FAKE_ORDER_CAP];
    u32 order_len;
    u32 count[200];
    u32 exti_pending;           /* read_exti_pending 返回值脚本 */
    /* 时钟脚本：第 N 次查询起返回 1；0 = 永不 ready */
    u32 hse_ready_at_call;
    u32 hse_ready_calls;
    u32 pll_ready_at_call;
    u32 pll_ready_calls;
    u32 sysclk_pll_at_call;
    u32 sysclk_is_pll_calls;
} FakeStop_t;

static void rec(FakeStop_t *f, u32 op)
{
    if (f->order_len < FAKE_ORDER_CAP) {
        f->order[f->order_len++] = op;
    }
    if (op < 200u) {
        f->count[op]++;
    }
}

static int idx_of(const FakeStop_t *f, u32 from, u32 op)
{
    u32 i;
    for (i = from; i < f->order_len; i++) {
        if (f->order[i] == op) {
            return (int)i;
        }
    }
    return -1;
}

static int idx_is(const FakeStop_t *f, u32 from, u32 op, int expect)
{
    return (idx_of(f, from, op) == expect);
}

static int a_before_b(const FakeStop_t *f, u32 from, u32 a, u32 b)
{
    int ia = idx_of(f, from, a);
    int ib = idx_of(f, from, b);
    return (ia >= 0) && (ib >= 0) && (ia < ib);
}

/*--------------------------- EXTI/PWR/外设 IO ---------------------------*/

static void fake_systick_stop(void *self) { rec((FakeStop_t *)self, OP_SYSTICK_STOP); }
static void fake_periph_quiesce(void *self) { rec((FakeStop_t *)self, OP_PERIPH_QUIESCE); }
static void fake_key_exti_config(void *self) { rec((FakeStop_t *)self, OP_KEY_EXTI_CONFIG); }
static void fake_clear_key_pending(void *self) { rec((FakeStop_t *)self, OP_CLEAR_KEY_PENDING); }
static void fake_pwr_stop_enter(void *self) { rec((FakeStop_t *)self, OP_PWR_STOP_ENTER); }
static void fake_dsb_isb_wfi(void *self) { rec((FakeStop_t *)self, OP_DSB_ISB_WFI); }

static u32 fake_read_exti_pending(void *self)
{
    FakeStop_t *f = (FakeStop_t *)self;
    rec(f, OP_READ_EXTI_PENDING);
    return f->exti_pending;
}

static void fake_periph_resume(void *self) { rec((FakeStop_t *)self, OP_PERIPH_RESUME); }
static void fake_systick_resume(void *self) { rec((FakeStop_t *)self, OP_SYSTICK_RESUME); }

/*--------------------------- 时钟恢复 IO ---------------------------*/

static void fake_clk_hse_on(void *self) { rec((FakeStop_t *)self, OP_CLK_HSE_ON); }

static u8 fake_clk_hse_ready(void *self)
{
    FakeStop_t *f = (FakeStop_t *)self;
    f->hse_ready_calls++;
    if (f->hse_ready_at_call == 0u) {
        return 0u;
    }
    return (f->hse_ready_calls >= f->hse_ready_at_call) ? 1u : 0u;
}

static void fake_clk_flash_latency(void *self) { rec((FakeStop_t *)self, OP_CLK_FLASH_LATENCY); }
static void fake_clk_bus_prescalers(void *self) { rec((FakeStop_t *)self, OP_CLK_BUS_PRESCALERS); }
static void fake_clk_pll_enable(void *self) { rec((FakeStop_t *)self, OP_CLK_PLL_ENABLE); }

static u8 fake_clk_pll_ready(void *self)
{
    FakeStop_t *f = (FakeStop_t *)self;
    f->pll_ready_calls++;
    if (f->pll_ready_at_call == 0u) {
        return 0u;
    }
    return (f->pll_ready_calls >= f->pll_ready_at_call) ? 1u : 0u;
}

static void fake_clk_select_sysclk(void *self) { rec((FakeStop_t *)self, OP_CLK_SELECT_SYSCLK); }

static u8 fake_clk_sysclk_is_pll(void *self)
{
    FakeStop_t *f = (FakeStop_t *)self;
    f->sysclk_is_pll_calls++;
    if (f->sysclk_pll_at_call == 0u) {
        return 0u;
    }
    return (f->sysclk_is_pll_calls >= f->sysclk_pll_at_call) ? 1u : 0u;
}

static void fake_clk_core_update(void *self) { rec((FakeStop_t *)self, OP_CLK_CORE_UPDATE); }
static void fake_clk_wait_poll(void *self) { rec((FakeStop_t *)self, OP_CLK_WAIT_POLL); }

static const ExtiStop_Io_t g_stop_io = {
    NULL,
    fake_systick_stop,
    fake_periph_quiesce,
    fake_key_exti_config,
    fake_clear_key_pending,
    fake_read_exti_pending,
    fake_pwr_stop_enter,
    fake_dsb_isb_wfi,
    fake_periph_resume,
    fake_systick_resume
};

static const SystemClockResume_Io_t g_clk_io = {
    NULL,
    fake_clk_hse_on,
    fake_clk_hse_ready,
    fake_clk_flash_latency,
    fake_clk_bus_prescalers,
    fake_clk_pll_enable,
    fake_clk_pll_ready,
    fake_clk_select_sysclk,
    fake_clk_sysclk_is_pll,
    fake_clk_core_update,
    fake_clk_wait_poll
};

#define KEY_LINE_MASK   (1uL << 1)              /* PB1 -> EXTI line 1（设计 §7） */

/*--------------------------- EXTI 后端场景 ---------------------------*/

/* s0: NULL 参数防御 */
static void s0_param_defense(void)
{
    ExtiStopBackend_t bk;

    printf("s0: NULL parameter defense\n");

    CHECK_EQ_DESC("Init(NULL bk) -> ERR_PARAM",
                  ExtiStop_Init(NULL, &g_stop_io, &g_clk_io, KEY_LINE_MASK),
                  EXTI_STOP_ERR_PARAM);
    CHECK_EQ_DESC("Init(NULL io) -> ERR_PARAM",
                  ExtiStop_Init(&bk, NULL, &g_clk_io, KEY_LINE_MASK),
                  EXTI_STOP_ERR_PARAM);
    CHECK_EQ_DESC("Init(NULL clk_io) -> ERR_PARAM",
                  ExtiStop_Init(&bk, &g_stop_io, NULL, KEY_LINE_MASK),
                  EXTI_STOP_ERR_PARAM);
    CHECK_EQ_DESC("Enter(NULL bk) -> ERR_PARAM",
                  ExtiStop_Enter(NULL, 1u, 0u), EXTI_STOP_ERR_PARAM);
    CHECK_EQ_DESC("Exit(NULL bk) -> ERR_PARAM",
                  ExtiStop_Exit(NULL), EXTI_STOP_ERR_PARAM);
    CHECK_EQ_DESC("GetWakeSource(NULL) -> WAKE_NONE",
                  ExtiStop_GetWakeSource(NULL), EXTI_STOP_WAKE_NONE);
}

/* s1: 决策层前提的后端防御再校验（零 IO 副作用） */
static void s1_precondition_defense(void)
{
    ExtiStopBackend_t bk;
    FakeStop_t f;
    ExtiStop_Io_t io = g_stop_io;
    SystemClockResume_Io_t clk = g_clk_io;

    memset(&bk, 0, sizeof(bk));
    memset(&f, 0, sizeof(f));
    io.self = &f;                               /* 脚本账本绑定为 IO self */
    clk.self = &f;
    (void)ExtiStop_Init(&bk, &io, &clk, KEY_LINE_MASK);

    printf("s1: backend re-checks decision preconditions\n");

    /* 有任务等待超时 -> 拒绝 */
    CHECK_EQ_DESC("tasks waiting timeout -> ERR_PRECONDITION",
                  ExtiStop_Enter(&bk, 0u, 0u), EXTI_STOP_ERR_PRECONDITION);
    /* IWDG 需要周期服务 -> 拒绝 */
    CHECK_EQ_DESC("IWDG requires service -> ERR_PRECONDITION",
                  ExtiStop_Enter(&bk, 1u, 1u), EXTI_STOP_ERR_PRECONDITION);
    /* 两者都不满足 -> 拒绝 */
    CHECK_EQ_DESC("both violated -> ERR_PRECONDITION",
                  ExtiStop_Enter(&bk, 0u, 1u), EXTI_STOP_ERR_PRECONDITION);

    /* 拒绝路径零 IO 副作用（未触碰 SysTick/外设/EXTI/PWR） */
    CHECK_EQ_DESC("precondition refusal has zero IO side effects",
                  f.order_len, 0u);

    /* 拒绝的 Enter 不算进入 STOP：Exit 仍报状态错误 */
    CHECK_EQ_DESC("Exit after refused Enter -> ERR_STATE",
                  ExtiStop_Exit(&bk), EXTI_STOP_ERR_STATE);
}

/* s2: 进入序列（设计 §6.6 EXTI-only 进入顺序） */
static void s2_enter_sequence(void)
{
    ExtiStopBackend_t bk;
    FakeStop_t f;
    ExtiStop_Io_t io = g_stop_io;
    SystemClockResume_Io_t clk = g_clk_io;

    printf("s2: enter sequence ordering (design 6.6)\n");

    memset(&bk, 0, sizeof(bk));
    memset(&f, 0, sizeof(f));
    f.hse_ready_at_call = 1u;
    f.pll_ready_at_call = 1u;
    f.sysclk_pll_at_call = 1u;
    io.self = &f;
    clk.self = &f;

    CHECK_EQ_DESC("Init -> OK",
                  ExtiStop_Init(&bk, &io, &clk, KEY_LINE_MASK),
                  EXTI_STOP_OK);
    CHECK_EQ_DESC("Enter with confirmed preconditions -> OK",
                  ExtiStop_Enter(&bk, 1u, 0u), EXTI_STOP_OK);

    /* 顺序：停 SysTick -> 外设静默 -> 按键 EXTI -> 清 pending -> PWR STOP -> WFI */
    CHECK_DESC("enter order: systick_stop/periph_quiesce/key_exti/clear_pending/pwr/wfi",
               idx_is(&f, 0u, OP_SYSTICK_STOP, 0) &&
               idx_is(&f, 0u, OP_PERIPH_QUIESCE, 1) &&
               idx_is(&f, 0u, OP_KEY_EXTI_CONFIG, 2) &&
               idx_is(&f, 0u, OP_CLEAR_KEY_PENDING, 3) &&
               idx_is(&f, 0u, OP_PWR_STOP_ENTER, 4) &&
               idx_is(&f, 0u, OP_DSB_ISB_WFI, 5));
    CHECK_EQ_DESC("enter is exactly 6 ops", f.order_len, 6u);

    /* 睡前清旧 pending 防止 WFI 立即返回（sleep race，设计 §6.6 进入 6） */
    CHECK_DESC("clear_key_pending happens before WFI",
               a_before_b(&f, 0u, OP_CLEAR_KEY_PENDING, OP_DSB_ISB_WFI));

    /* Enter 返回（=WFI 唤醒）后不得有任何退出侧动作（Exit 显式触发） */
    CHECK_EQ_DESC("no exit-side ops right after Enter",
                  f.count[OP_READ_EXTI_PENDING] == 0u &&
                  f.count[OP_PERIPH_RESUME] == 0u &&
                  f.count[OP_SYSTICK_RESUME] == 0u &&
                  f.count[OP_CLK_HSE_ON] == 0u, 1);

    /* 不调用 vTaskStepTick（tick 冻结 */
    CHECK_EQ_DESC("vTaskStepTick never called on enter path",
                  g_vtask_step_tick_calls, 0u);

    /* 唤醒源默认 NONE（Exit 前未读取） */
    CHECK_EQ_DESC("wake source default NONE before Exit",
                  ExtiStop_GetWakeSource(&bk), EXTI_STOP_WAKE_NONE);

    /* 重复 Enter（未 Exit）-> 状态错误且零新副作用 */
    CHECK_EQ_DESC("re-Enter without Exit -> ERR_STATE",
                  ExtiStop_Enter(&bk, 1u, 0u), EXTI_STOP_ERR_STATE);
    CHECK_EQ_DESC("re-Enter adds no IO ops", f.order_len, 6u);

    /* 本场景收尾：完整退出，保持后端状态干净 */
    f.exti_pending = KEY_LINE_MASK;
    CHECK_EQ_DESC("cleanup Exit -> OK", ExtiStop_Exit(&bk), EXTI_STOP_OK);
}

/* s3: 退出序列 + tick 冻结（key 唤醒） */
static void s3_exit_sequence(void)
{
    ExtiStopBackend_t bk;
    FakeStop_t f;
    ExtiStop_Io_t io = g_stop_io;
    SystemClockResume_Io_t clk = g_clk_io;

    printf("s3: exit sequence ordering + frozen tick\n");

    memset(&bk, 0, sizeof(bk));
    memset(&f, 0, sizeof(f));
    f.hse_ready_at_call = 1u;
    f.pll_ready_at_call = 1u;
    f.sysclk_pll_at_call = 1u;
    f.exti_pending = KEY_LINE_MASK;
    io.self = &f;
    clk.self = &f;

    (void)ExtiStop_Init(&bk, &io, &clk, KEY_LINE_MASK);
    (void)ExtiStop_Enter(&bk, 1u, 0u);
    f.order_len = 0u;                           /* 只审计退出序列 */
    memset(f.count, 0, sizeof(f.count));

    CHECK_EQ_DESC("Exit with key pending -> OK",
                  ExtiStop_Exit(&bk), EXTI_STOP_OK);
    CHECK_EQ_DESC("wake source is KEY",
                  ExtiStop_GetWakeSource(&bk), EXTI_STOP_WAKE_KEY);

    /* 顺序：读 pending -> 时钟恢复完整链 -> 外设恢复 -> SysTick 恢复 */
    CHECK_DESC("exit: read_exti_pending runs first",
               idx_is(&f, 0u, OP_READ_EXTI_PENDING, 0));
    CHECK_DESC("exit: clock HSE on before flash latency",
               a_before_b(&f, 0u, OP_CLK_HSE_ON, OP_CLK_FLASH_LATENCY));
    CHECK_DESC("exit: flash latency before bus prescalers",
               a_before_b(&f, 0u, OP_CLK_FLASH_LATENCY, OP_CLK_BUS_PRESCALERS));
    CHECK_DESC("exit: bus prescalers before PLL enable",
               a_before_b(&f, 0u, OP_CLK_BUS_PRESCALERS, OP_CLK_PLL_ENABLE));
    CHECK_DESC("exit: PLL enable before SYSCLK select",
               a_before_b(&f, 0u, OP_CLK_PLL_ENABLE, OP_CLK_SELECT_SYSCLK));
    CHECK_DESC("exit: SYSCLK select before core clock update",
               a_before_b(&f, 0u, OP_CLK_SELECT_SYSCLK, OP_CLK_CORE_UPDATE));
    CHECK_DESC("exit: clock chain before peripheral resume",
               a_before_b(&f, 0u, OP_CLK_CORE_UPDATE, OP_PERIPH_RESUME));
    CHECK_DESC("exit: peripheral resume before SysTick resume",
               a_before_b(&f, 0u, OP_PERIPH_RESUME, OP_SYSTICK_RESUME));

    /* SystemCoreClockUpdate 恰好一次（设计 §6.6 退出 2/3） */
    CHECK_EQ_DESC("SystemCoreClockUpdate called once",
                  f.count[OP_CLK_CORE_UPDATE], 1u);

    /* vTaskStepTick 禁止：EXTI-only 路径 tick 冻结不补偿（设计 §6.3） */
    CHECK_EQ_DESC("vTaskStepTick never called (tick frozen)",
                  g_vtask_step_tick_calls, 0u);

    /* Exit 一次性：再次调用报状态错误 */
    CHECK_EQ_DESC("second Exit -> ERR_STATE", ExtiStop_Exit(&bk),
                  EXTI_STOP_ERR_STATE);
}

/* s4: 唤醒源分类 */
static void s4_wake_source(void)
{
    ExtiStopBackend_t bk;
    FakeStop_t f;
    ExtiStop_Io_t io = g_stop_io;
    SystemClockResume_Io_t clk = g_clk_io;

    printf("s4: wake source classification\n");

    io.self = &f;
    clk.self = &f;

    /* 其他 EXTI 线唤醒 */
    memset(&bk, 0, sizeof(bk));
    memset(&f, 0, sizeof(f));
    f.hse_ready_at_call = 1u;
    f.pll_ready_at_call = 1u;
    f.sysclk_pll_at_call = 1u;
    f.exti_pending = (1uL << 5);               /* 非 key 线 */
    (void)ExtiStop_Init(&bk, &io, &clk, KEY_LINE_MASK);
    (void)ExtiStop_Enter(&bk, 1u, 0u);
    CHECK_EQ_DESC("Exit with other-line pending -> OK",
                  ExtiStop_Exit(&bk), EXTI_STOP_OK);
    CHECK_EQ_DESC("wake source is OTHER",
                  ExtiStop_GetWakeSource(&bk), EXTI_STOP_WAKE_OTHER);

    /* key 位 + 其他位同时 pending -> KEY 优先归类 */
    memset(&bk, 0, sizeof(bk));
    memset(&f, 0, sizeof(f));
    f.hse_ready_at_call = 1u;
    f.pll_ready_at_call = 1u;
    f.sysclk_pll_at_call = 1u;
    f.exti_pending = (1uL << 5) | KEY_LINE_MASK;
    (void)ExtiStop_Init(&bk, &io, &clk, KEY_LINE_MASK);
    (void)ExtiStop_Enter(&bk, 1u, 0u);
    (void)ExtiStop_Exit(&bk);
    CHECK_EQ_DESC("key + other pending -> classified as KEY",
                  ExtiStop_GetWakeSource(&bk), EXTI_STOP_WAKE_KEY);

    /* 无 pending（调试器/非 EXTI 唤醒）-> NONE */
    memset(&bk, 0, sizeof(bk));
    memset(&f, 0, sizeof(f));
    f.hse_ready_at_call = 1u;
    f.pll_ready_at_call = 1u;
    f.sysclk_pll_at_call = 1u;
    f.exti_pending = 0u;
    (void)ExtiStop_Init(&bk, &io, &clk, KEY_LINE_MASK);
    (void)ExtiStop_Enter(&bk, 1u, 0u);
    CHECK_EQ_DESC("Exit with no pending -> OK",
                  ExtiStop_Exit(&bk), EXTI_STOP_OK);
    CHECK_EQ_DESC("wake source is NONE",
                  ExtiStop_GetWakeSource(&bk), EXTI_STOP_WAKE_NONE);
}

/* s5: 时钟恢复失败传播（HSI 下不得恢复外设/SysTick） */
static void s5_clock_resume_failure(void)
{
    ExtiStopBackend_t bk;
    FakeStop_t f;
    ExtiStop_Io_t io = g_stop_io;
    SystemClockResume_Io_t clk = g_clk_io;

    printf("s5: clock resume failure propagation\n");

    memset(&bk, 0, sizeof(bk));
    memset(&f, 0, sizeof(f));
    f.hse_ready_at_call = 0u;                   /* HSE 永不就绪 */
    f.exti_pending = KEY_LINE_MASK;
    io.self = &f;
    clk.self = &f;
    (void)ExtiStop_Init(&bk, &io, &clk, KEY_LINE_MASK);
    (void)ExtiStop_Enter(&bk, 1u, 0u);
    f.order_len = 0u;
    memset(f.count, 0, sizeof(f.count));

    CHECK_EQ_DESC("Exit with dead HSE -> ERR_CLOCK_RESUME",
                  ExtiStop_Exit(&bk), EXTI_STOP_ERR_CLOCK_RESUME);

    /* 失败后：唤醒源已记录，外设/SysTick 不恢复（设计 §6.6 fail-safe） */
    CHECK_EQ_DESC("wake source still recorded (KEY)",
                  ExtiStop_GetWakeSource(&bk), EXTI_STOP_WAKE_KEY);
    CHECK_EQ_DESC("peripherals NOT resumed under HSI",
                  f.count[OP_PERIPH_RESUME], 0u);
    CHECK_EQ_DESC("SysTick NOT resumed under HSI",
                  f.count[OP_SYSTICK_RESUME], 0u);

    /* 失败的 Exit 也是一次性的 */
    CHECK_EQ_DESC("Exit after failed Exit -> ERR_STATE",
                  ExtiStop_Exit(&bk), EXTI_STOP_ERR_STATE);
}

/*--------------------------- SystemClock_RestoreAfterStop 场景 ---------------------------*/

/* s6: 成功序列与 NULL 防御 */
static void s6_clk_resume_success(void)
{
    FakeStop_t f;
    SystemClockResume_Io_t clk = g_clk_io;

    printf("s6: SystemClock_RestoreAfterStop success order\n");

    CHECK_EQ_DESC("NULL io -> ERR_PARAM",
                  SystemClock_RestoreAfterStop(NULL), CLK_RESUME_ERR_PARAM);

    memset(&f, 0, sizeof(f));
    f.hse_ready_at_call = 3u;                   /* HSE 第 3 次查询就绪 */
    f.pll_ready_at_call = 1u;
    f.sysclk_pll_at_call = 1u;
    clk.self = &f;

    CHECK_EQ_DESC("restore -> OK",
                  SystemClock_RestoreAfterStop(&clk), CLK_RESUME_OK);

    /* 顺序：HSE on -> Flash latency -> 总线分频 -> PLL 使能 -> 选源 -> 更新
     *（HSE 前 2 轮未就绪各插一次 wait_poll，其余段立即就绪） */
    CHECK_DESC("restore order: hse_on/flash/bus/pll/select/update",
               idx_is(&f, 0u, OP_CLK_HSE_ON, 0) &&
               idx_is(&f, 0u, OP_CLK_FLASH_LATENCY, 3) &&
               idx_is(&f, 0u, OP_CLK_BUS_PRESCALERS, 4) &&
               idx_is(&f, 0u, OP_CLK_PLL_ENABLE, 5) &&
               idx_is(&f, 0u, OP_CLK_SELECT_SYSCLK, 6) &&
               idx_is(&f, 0u, OP_CLK_CORE_UPDATE, 7));
    CHECK_EQ_DESC("order includes 2 HSE wait_polls (8 ops total)",
                  f.order_len, 8u);
    CHECK_EQ_DESC("HSE polled 3 times", f.hse_ready_calls, 3u);
    CHECK_EQ_DESC("wait_poll count 2 (HSE settle)", f.count[OP_CLK_WAIT_POLL], 2u);
    CHECK_EQ_DESC("PLL polled once", f.pll_ready_calls, 1u);
    CHECK_EQ_DESC("SWS polled once", f.sysclk_is_pll_calls, 1u);
}

/* s7: 三种超时（后续操作全部不执行） */
static void s7_clk_resume_timeouts(void)
{
    FakeStop_t f;
    SystemClockResume_Io_t clk = g_clk_io;

    printf("s7: clock resume timeout fail-safes\n");

    clk.self = &f;

    /* HSE 超时：Flash/总线/PLL/选源/更新都不执行 */
    memset(&f, 0, sizeof(f));
    f.hse_ready_at_call = 0u;
    f.pll_ready_at_call = 1u;
    f.sysclk_pll_at_call = 1u;
    CHECK_EQ_DESC("HSE never ready -> ERR_HSE_TIMEOUT",
                  SystemClock_RestoreAfterStop(&clk),
                  CLK_RESUME_ERR_HSE_TIMEOUT);
    CHECK_EQ_DESC("no flash/bus/pll/select/update after HSE timeout",
                  f.count[OP_CLK_FLASH_LATENCY] == 0u &&
                  f.count[OP_CLK_BUS_PRESCALERS] == 0u &&
                  f.count[OP_CLK_PLL_ENABLE] == 0u &&
                  f.count[OP_CLK_SELECT_SYSCLK] == 0u &&
                  f.count[OP_CLK_CORE_UPDATE] == 0u, 1);
    CHECK_EQ_DESC("HSE wait bounded to timeout loops",
                  f.count[OP_CLK_WAIT_POLL], CLK_RESUME_TIMEOUT_LOOPS);

    /* PLL 超时：选源/更新不执行 */
    memset(&f, 0, sizeof(f));
    f.hse_ready_at_call = 1u;
    f.pll_ready_at_call = 0u;
    f.sysclk_pll_at_call = 1u;
    CHECK_EQ_DESC("PLL never ready -> ERR_PLL_TIMEOUT",
                  SystemClock_RestoreAfterStop(&clk),
                  CLK_RESUME_ERR_PLL_TIMEOUT);
    CHECK_EQ_DESC("flash/bus/pll executed before PLL timeout",
                  f.count[OP_CLK_FLASH_LATENCY] == 1u &&
                  f.count[OP_CLK_BUS_PRESCALERS] == 1u &&
                  f.count[OP_CLK_PLL_ENABLE] == 1u, 1);
    CHECK_EQ_DESC("no select/update after PLL timeout",
                  f.count[OP_CLK_SELECT_SYSCLK] == 0u &&
                  f.count[OP_CLK_CORE_UPDATE] == 0u, 1);
    CHECK_EQ_DESC("PLL wait bounded to timeout loops",
                  f.count[OP_CLK_WAIT_POLL], CLK_RESUME_TIMEOUT_LOOPS);

    /* SWS 切换超时：更新不执行 */
    memset(&f, 0, sizeof(f));
    f.hse_ready_at_call = 1u;
    f.pll_ready_at_call = 1u;
    f.sysclk_pll_at_call = 0u;
    CHECK_EQ_DESC("SWS never PLL -> ERR_SWITCH_TIMEOUT",
                  SystemClock_RestoreAfterStop(&clk),
                  CLK_RESUME_ERR_SWITCH_TIMEOUT);
    CHECK_EQ_DESC("select executed but no update after switch timeout",
                  f.count[OP_CLK_SELECT_SYSCLK] == 1u &&
                  f.count[OP_CLK_CORE_UPDATE] == 0u, 1);
    CHECK_EQ_DESC("SWS wait bounded to timeout loops",
                  f.count[OP_CLK_WAIT_POLL], CLK_RESUME_TIMEOUT_LOOPS);
}

/* s8: 超时边界（off-by-one） */
static void s8_clk_resume_boundary(void)
{
    FakeStop_t f;
    SystemClockResume_Io_t clk = g_clk_io;

    printf("s8: clock resume timeout boundary\n");

    clk.self = &f;

    /* HSE 恰好在第 TIMEOUT 次查询就绪 -> 成功 */
    memset(&f, 0, sizeof(f));
    f.hse_ready_at_call = CLK_RESUME_TIMEOUT_LOOPS;
    f.pll_ready_at_call = 1u;
    f.sysclk_pll_at_call = 1u;
    CHECK_EQ_DESC("HSE ready at last loop -> OK",
                  SystemClock_RestoreAfterStop(&clk), CLK_RESUME_OK);

    /* 第 TIMEOUT+1 次才就绪 -> 超时 */
    memset(&f, 0, sizeof(f));
    f.hse_ready_at_call = CLK_RESUME_TIMEOUT_LOOPS + 1u;
    f.pll_ready_at_call = 1u;
    f.sysclk_pll_at_call = 1u;
    CHECK_EQ_DESC("HSE one loop too late -> ERR_HSE_TIMEOUT",
                  SystemClock_RestoreAfterStop(&clk),
                  CLK_RESUME_ERR_HSE_TIMEOUT);

    /* PLL 恰好在第 TIMEOUT 次查询就绪 -> 成功 */
    memset(&f, 0, sizeof(f));
    f.hse_ready_at_call = 1u;
    f.pll_ready_at_call = CLK_RESUME_TIMEOUT_LOOPS;
    f.sysclk_pll_at_call = 1u;
    CHECK_EQ_DESC("PLL ready at last loop -> OK",
                  SystemClock_RestoreAfterStop(&clk), CLK_RESUME_OK);
}

int main(void)
{
    printf("exti-stop-backend + clock-resume mock\n");
    s0_param_defense();
    s1_precondition_defense();
    s2_enter_sequence();
    s3_exit_sequence();
    s4_wake_source();
    s5_clock_resume_failure();
    s6_clk_resume_success();
    s7_clk_resume_timeouts();
    s8_clk_resume_boundary();

    /* 全程禁调 vTaskStepTick（链接集+计数双证明） */
    CHECK_EQ_DESC("vTaskStepTick never called across whole suite",
                  g_vtask_step_tick_calls, 0u);

    printf("exti-stop-backend mock: %d checks, %d failed\n",
           g_total, g_failed);
    return (g_failed == 0) ? 0 : 1;
}
