/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Host-side mock test for the board-aware STOP policy decision
 * Covers:
 *
 *   1. Three distinct decisions reachable (LP_SHALLOW_TICKLESS /
 *      LP_STOP_RTC / LP_STOP_EXTI_ONLY) -- path differences are not
 *      hidden behind a single bool
 *   2. Common STOP gate: every stop path requires standby mode +
 *      motor physically off + buzzer/IAP/UART-DMA idle + clock healthy;
 *      each violation alone degrades to shallow tickless
 *   3. LP_STOP_RTC only for BOARD_C8_LSE + RTC ready +
 *      expected_idle_ticks > wake_guard (each condition broken
 *      individually)
 *   4. LP_STOP_EXTI_ONLY only for eNoTasksWaitingTimeout + no IWDG
 *      periodic service + at least one valid key EXTI (each broken
 *      individually -> shallow); RTC path takes precedence when both
 *      paths are eligible (design §6.4 ordering)
 *   5. BOARD_C8_NO_LSE / BOARD_PROFILE_PENDING never return LP_STOP_RTC
 *      even with a maliciously asserted rtc_ready (Task 1 gate re-checked)
 *   6. Watchdog_RequiresPeriodicService: sw IWDG started OR
 *      FLASH->OBR WDG_SW bit clear (hardware IWDG) -> requires service
 *   7. Pure-function contract: identical snapshot -> identical decision,
 *      interleaved different snapshots do not interfere (no hidden state)
 *
 * Production sources under test are compiled UNMODIFIED from the project
 * tree (BSP\LOWPOWER\lowpower_policy.c); sys.h comes from stub/.
 */
#include <stdio.h>
#include <string.h>
#include "lowpower_policy.h"

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
        printf("  ok %s\n", desc);                           \
    else { g_failed++;                                        \
        printf("  FAIL %s: got %ld, want %ld\n",              \
               desc, (long)(got), (long)(want)); }            \
} while (0)

/*--------------------------- snapshot builders ---------------------------*/

/* LSE 板型全绿 RTC 基线：期望 LP_STOP_RTC */
static LpPolicySnapshot_t snap_lse_rtc(void)
{
    LpPolicySnapshot_t s;
    memset(&s, 0, sizeof(s));
    s.board                    = LP_BOARD_C8_LSE;
    s.standby_mode             = 1;
    s.motor_physically_off     = 1;
    s.buzzer_idle              = 1;
    s.iap_idle                 = 1;
    s.uart_dma_idle            = 1;
    s.clock_healthy            = 1;
    s.rtc_ready                = 1;
    s.no_tasks_waiting_timeout = 0;
    s.watchdog_requires_service = 0;
    s.key_exti_available       = 0;
    s.expected_idle_ticks      = 100u;
    return s;
}

/* EXTI-only 基线（NO_LSE 板型）：期望 LP_STOP_EXTI_ONLY */
static LpPolicySnapshot_t snap_exti(void)
{
    LpPolicySnapshot_t s;
    memset(&s, 0, sizeof(s));
    s.board                    = LP_BOARD_C8_NO_LSE;
    s.standby_mode             = 1;
    s.motor_physically_off     = 1;
    s.buzzer_idle              = 1;
    s.iap_idle                 = 1;
    s.uart_dma_idle            = 1;
    s.clock_healthy            = 1;
    s.rtc_ready                = 0;
    s.no_tasks_waiting_timeout = 1;
    s.watchdog_requires_service = 0;
    s.key_exti_available       = 1;
    s.expected_idle_ticks      = 0xFFFFFFFFu;
    return s;
}

/*--------------------------- s0: 纯函数确定性 ---------------------------*/

static void s0_pure_function(void)
{
    LpPolicySnapshot_t a = snap_lse_rtc();
    LpPolicySnapshot_t b = snap_exti();
    LpDecision_t d1, d2, d3;

    printf("s0: pure function (same snapshot -> same decision, no state)\n");

    /* 同一快照两次调用一致 */
    d1 = LpPolicy_Decide(&a);
    d2 = LpPolicy_Decide(&a);
    CHECK_DESC("same snapshot twice -> identical decision", d1 == d2);

    /* 穿插不同快照互不影响 */
    d3 = LpPolicy_Decide(&b);
    CHECK_DESC("interleaved snapshot does not interfere",
               LpPolicy_Decide(&a) == d1 && d3 != d1);

    /* NULL 快照防御 -> 浅睡 */
    CHECK_EQ_DESC("NULL snapshot -> LP_SHALLOW_TICKLESS",
                  LpPolicy_Decide(NULL), LP_SHALLOW_TICKLESS);
}

/*--------------------------- s1: 三态可达 ---------------------------*/

static void s1_three_decisions(void)
{
    LpPolicySnapshot_t rtc = snap_lse_rtc();
    LpPolicySnapshot_t exti = snap_exti();
    LpPolicySnapshot_t shallow = snap_lse_rtc();

    printf("s1: three distinct decisions reachable\n");

    /* 枚举恰好三态，值域 0..2（不用单一 bool 掩盖路径差异）*/
    CHECK_DESC("enum has exactly 3 members (0/1/2)",
               LP_SHALLOW_TICKLESS == 0 && LP_STOP_RTC == 1 &&
               LP_STOP_EXTI_ONLY == 2);

    CHECK_EQ_DESC("LSE+ready+idle>guard -> LP_STOP_RTC",
                  LpPolicy_Decide(&rtc), LP_STOP_RTC);

    CHECK_EQ_DESC("no-timeout+no-iwdg+key-exti -> LP_STOP_EXTI_ONLY",
                  LpPolicy_Decide(&exti), LP_STOP_EXTI_ONLY);

    shallow.standby_mode = 0;   /* 破坏公共门槛 */
    CHECK_EQ_DESC("gate violation -> LP_SHALLOW_TICKLESS",
                  LpPolicy_Decide(&shallow), LP_SHALLOW_TICKLESS);
}

/*--------------------------- s2: 公共 STOP 门槛 ---------------------------*/

static void s2_common_stop_gate(void)
{
    LpPolicySnapshot_t s = snap_lse_rtc();
    LpDecision_t baseline;

    printf("s2: common STOP gate (all paths share it)\n");

    baseline = LpPolicy_Decide(&s);
    CHECK_EQ_DESC("baseline is LP_STOP_RTC", baseline, LP_STOP_RTC);

    s.standby_mode = 0;
    CHECK_EQ_DESC("not MODE_STANDBY -> shallow",
                  LpPolicy_Decide(&s), LP_SHALLOW_TICKLESS);
    s.standby_mode = 1;

    s.motor_physically_off = 0;
    CHECK_EQ_DESC("motor not physically off -> shallow",
                  LpPolicy_Decide(&s), LP_SHALLOW_TICKLESS);
    s.motor_physically_off = 1;

    s.buzzer_idle = 0;
    CHECK_EQ_DESC("buzzer busy -> shallow",
                  LpPolicy_Decide(&s), LP_SHALLOW_TICKLESS);
    s.buzzer_idle = 1;

    s.iap_idle = 0;
    CHECK_EQ_DESC("IAP busy -> shallow",
                  LpPolicy_Decide(&s), LP_SHALLOW_TICKLESS);
    s.iap_idle = 1;

    s.uart_dma_idle = 0;
    CHECK_EQ_DESC("UART/DMA busy -> shallow",
                  LpPolicy_Decide(&s), LP_SHALLOW_TICKLESS);
    s.uart_dma_idle = 1;

    s.clock_healthy = 0;
    CHECK_EQ_DESC("clock degraded -> shallow",
                  LpPolicy_Decide(&s), LP_SHALLOW_TICKLESS);
    s.clock_healthy = 1;

    /* 同一门槛同样约束 EXTI-only 路径 */
    {
        LpPolicySnapshot_t e = snap_exti();
        e.motor_physically_off = 0;
        CHECK_EQ_DESC("gate violation also blocks EXTI-only path",
                      LpPolicy_Decide(&e), LP_SHALLOW_TICKLESS);
    }
}

/*--------------------------- s3: LP_STOP_RTC 专属条件 ---------------------------*/

static void s3_rtc_path_conditions(void)
{
    LpPolicySnapshot_t s = snap_lse_rtc();

    printf("s3: LP_STOP_RTC preconditions (board / rtc_ready / idle>guard)\n");

    CHECK_EQ_DESC("baseline -> LP_STOP_RTC",
                  LpPolicy_Decide(&s), LP_STOP_RTC);

    s.rtc_ready = 0;
    CHECK_EQ_DESC("RTC not ready -> not LP_STOP_RTC",
                  LpPolicy_Decide(&s) != LP_STOP_RTC, 1);
    s.rtc_ready = 1;

    /* expected_idle == wake_guard：不大于，拒绝 RTC 路径 */
    s.expected_idle_ticks = LP_WAKE_GUARD_TICKS;
    CHECK_EQ_DESC("idle == wake_guard -> not LP_STOP_RTC",
                  LpPolicy_Decide(&s) != LP_STOP_RTC, 1);

    /* expected_idle = guard+1：恰过阈值，允许 */
    s.expected_idle_ticks = LP_WAKE_GUARD_TICKS + 1u;
    CHECK_EQ_DESC("idle == guard+1 -> LP_STOP_RTC",
                  LpPolicy_Decide(&s), LP_STOP_RTC);

    /* expected_idle = portMAX_DELAY（无任务等待超时场景）也允许 RTC 路径 */
    s.expected_idle_ticks = 0xFFFFFFFFu;
    CHECK_EQ_DESC("idle == portMAX_DELAY -> LP_STOP_RTC",
                  LpPolicy_Decide(&s), LP_STOP_RTC);
}

/*--------------------------- s4: LP_STOP_EXTI_ONLY 专属条件 ---------------------------*/

static void s4_exti_path_conditions(void)
{
    LpPolicySnapshot_t s = snap_exti();

    printf("s4: LP_STOP_EXTI_ONLY preconditions (no-timeout/iwdg/key-exti)\n");

    CHECK_EQ_DESC("baseline -> LP_STOP_EXTI_ONLY",
                  LpPolicy_Decide(&s), LP_STOP_EXTI_ONLY);

    s.no_tasks_waiting_timeout = 0;
    CHECK_EQ_DESC("tasks waiting timeout -> shallow",
                  LpPolicy_Decide(&s), LP_SHALLOW_TICKLESS);
    s.no_tasks_waiting_timeout = 1;

    s.watchdog_requires_service = 1;
    CHECK_EQ_DESC("IWDG needs service -> shallow",
                  LpPolicy_Decide(&s), LP_SHALLOW_TICKLESS);
    s.watchdog_requires_service = 0;

    s.key_exti_available = 0;
    CHECK_EQ_DESC("no key EXTI wake source -> shallow",
                  LpPolicy_Decide(&s), LP_SHALLOW_TICKLESS);
    s.key_exti_available = 1;

    /* RTC 优先于 EXTI（设计 §6.4 决策顺序）：两条路径同时满足时走 RTC */
    {
        LpPolicySnapshot_t both = snap_lse_rtc();
        both.no_tasks_waiting_timeout = 1;
        both.watchdog_requires_service = 0;
        both.key_exti_available = 1;
        CHECK_EQ_DESC("RTC path takes precedence over EXTI-only",
                      LpPolicy_Decide(&both), LP_STOP_RTC);
    }
}

/*--------------------------- s5: 板型门禁（NO_LSE / PENDING） ---------------------------*/

static void s5_board_gate(void)
{
    /* 恶意注入：非 LSE 板型但 rtc_ready=1、空闲时间充足——
     * 决策仍不得返回 LP_STOP_RTC（无 LSE 不做 RTC 补偿，
     *；rtc_ready 在这些板型上
     * 属组装错误，纯函数按板型硬门禁拒绝）*/
    LpPolicySnapshot_t no_lse = snap_lse_rtc();
    LpPolicySnapshot_t pending = snap_lse_rtc();

    printf("s5: board gate (NO_LSE / PENDING never take RTC path)\n");

    no_lse.board = LP_BOARD_C8_NO_LSE;
    CHECK_EQ_DESC("NO_LSE board + injected rtc_ready -> not LP_STOP_RTC",
                  LpPolicy_Decide(&no_lse) != LP_STOP_RTC, 1);

    pending.board = LP_BOARD_PENDING;
    CHECK_EQ_DESC("PENDING board + injected rtc_ready -> not LP_STOP_RTC",
                  LpPolicy_Decide(&pending) != LP_STOP_RTC, 1);

    /* PENDING 期间 EXTI-only 路径不依赖 LSE，条件满足仍可走
     *（板型门禁只禁 LSE/RTC 路径，board_profile.h Task 1）*/
    {
        LpPolicySnapshot_t e = snap_exti();
        e.board = LP_BOARD_PENDING;
        CHECK_EQ_DESC("PENDING board may still take EXTI-only path",
                      LpPolicy_Decide(&e), LP_STOP_EXTI_ONLY);
    }
}

/*--------------------------- s6: IWDG 服务需求 ---------------------------*/

static void s6_watchdog_service(void)
{
    printf("s6: Watchdog_RequiresPeriodicService\n");

    /* 软件 IWDG 已启动 -> 必须服务（OBR 无关）*/
    CHECK_EQ_DESC("sw IWDG started + OBR.WDG_SW set -> service",
                  Watchdog_RequiresPeriodicService(1, LP_FLASH_OBR_WDG_SW), 1);
    CHECK_EQ_DESC("sw IWDG started + OBR=0 -> service",
                  Watchdog_RequiresPeriodicService(1, 0u), 1);

    /* 软件 IWDG 未启动 + 硬件 IWDG（WDG_SW 位未置位）-> 必须服务 */
    CHECK_EQ_DESC("sw stopped + OBR=0 (hw IWDG) -> service",
                  Watchdog_RequiresPeriodicService(0, 0u), 1);
    CHECK_EQ_DESC("sw stopped + OBR=0x1FF (WDG_SW clear) -> service",
                  Watchdog_RequiresPeriodicService(0, 0x1FFu), 1);

    /* 软件 IWDG 未启动 + OBR.WDG_SW 置位（软件看门狗未启用）-> 不需要 */
    CHECK_EQ_DESC("sw stopped + OBR.WDG_SW set -> no service",
                  Watchdog_RequiresPeriodicService(0, LP_FLASH_OBR_WDG_SW), 0);
    CHECK_EQ_DESC("sw stopped + OBR=0x10000 + other bits -> no service",
                  Watchdog_RequiresPeriodicService(0, 0x1FFFFu), 0);

    /* 位掩码只看 bit16（其余位不参与判定）*/
    CHECK_EQ_DESC("OBR bit16 is the only discriminator",
                  Watchdog_RequiresPeriodicService(0, 0xFFFEFFFFu), 1);
}

/*--------------------------- s7: 拒绝原因分类 ---------------------------*/
/*：拒绝原因分别计数，不能折叠成单一失败值。
 * 契约：返回"阻止深睡（RTC 或 EXTI-only 任一路径）的第一原因"，
 * 检查顺序与 LpPolicy_Decide 单一来源：
 *   公共门槛（按字段顺序）-> RTC 可达即 NONE -> EXTI 前提（按序）。
 * RTC 不可用只是"降级到 EXTI 候选"，不算拒绝（设计 §6.4 顺序）。 */

static void s7_refuse_reason(void)
{
    LpPolicySnapshot_t s;

    printf("s7: LpPolicy_RefuseReason classification\n");

    /* NONE：两条深睡路径基线 */
    s = snap_lse_rtc();
    CHECK_EQ_DESC("refuse: LSE RTC baseline -> NONE",
                  LpPolicy_RefuseReason(&s), LP_REFUSE_NONE);
    s = snap_exti();
    CHECK_EQ_DESC("refuse: EXTI baseline -> NONE",
                  LpPolicy_RefuseReason(&s), LP_REFUSE_NONE);

    /* 公共门槛单项失败（其余全绿，按检查顺序第一失败项归类）*/
    s = snap_exti(); s.standby_mode = 0;
    CHECK_EQ_DESC("refuse: not standby -> NOT_STANDBY",
                  LpPolicy_RefuseReason(&s), LP_REFUSE_NOT_STANDBY);
    s = snap_exti(); s.motor_physically_off = 0;
    CHECK_EQ_DESC("refuse: motor on -> MOTOR_ON",
                  LpPolicy_RefuseReason(&s), LP_REFUSE_MOTOR_ON);
    s = snap_exti(); s.buzzer_idle = 0;
    CHECK_EQ_DESC("refuse: buzzer busy -> BUZZER_BUSY",
                  LpPolicy_RefuseReason(&s), LP_REFUSE_BUZZER_BUSY);
    s = snap_exti(); s.iap_idle = 0;
    CHECK_EQ_DESC("refuse: iap busy -> IAP_BUSY",
                  LpPolicy_RefuseReason(&s), LP_REFUSE_IAP_BUSY);
    s = snap_exti(); s.uart_dma_idle = 0;
    CHECK_EQ_DESC("refuse: uart busy -> UART_BUSY",
                  LpPolicy_RefuseReason(&s), LP_REFUSE_UART_BUSY);
    s = snap_exti(); s.clock_healthy = 0;
    CHECK_EQ_DESC("refuse: clock unhealthy -> CLOCK_UNHEALTHY",
                  LpPolicy_RefuseReason(&s), LP_REFUSE_CLOCK_UNHEALTHY);

    /* 公共门槛优先于 EXTI 前提：非待机 + 无 EXTI 同时失败 -> 非待机 */
    s = snap_exti(); s.standby_mode = 0; s.key_exti_available = 0;
    CHECK_EQ_DESC("refuse: common gate takes precedence over EXTI preconds",
                  LpPolicy_RefuseReason(&s), LP_REFUSE_NOT_STANDBY);

    /* EXTI 前提单项失败（公共门槛全绿、RTC 不可达）*/
    s = snap_exti(); s.no_tasks_waiting_timeout = 0;
    CHECK_EQ_DESC("refuse: tasks waiting timeout -> TASKS_WAITING",
                  LpPolicy_RefuseReason(&s), LP_REFUSE_TASKS_WAITING);
    s = snap_exti(); s.watchdog_requires_service = 1;
    CHECK_EQ_DESC("refuse: iwdg needs service -> IWDG_SERVICE",
                  LpPolicy_RefuseReason(&s), LP_REFUSE_IWDG_SERVICE);
    s = snap_exti(); s.key_exti_available = 0;
    CHECK_EQ_DESC("refuse: no key exti -> NO_EXTI",
                  LpPolicy_RefuseReason(&s), LP_REFUSE_NO_EXTI);

    /* EXTI 前提内部顺序：有任务等待 + IWDG 需服务 -> 前者优先 */
    s = snap_exti(); s.no_tasks_waiting_timeout = 0;
    s.watchdog_requires_service = 1;
    CHECK_EQ_DESC("refuse: EXTI precondition order: tasks first",
                  LpPolicy_RefuseReason(&s), LP_REFUSE_TASKS_WAITING);

    /* RTC 降级不是拒绝：LSE 板 rtc_ready=0 但 EXTI 全绿 -> NONE */
    s = snap_lse_rtc(); s.rtc_ready = 0;
    s.no_tasks_waiting_timeout = 1; s.key_exti_available = 1;
    CHECK_EQ_DESC("refuse: RTC degraded but EXTI eligible -> NONE",
                  LpPolicy_RefuseReason(&s), LP_REFUSE_NONE);

    /* RTC 不可用 + EXTI 也不可用 -> EXTI 前提失败项（非新原因）*/
    s = snap_lse_rtc(); s.rtc_ready = 0;
    s.no_tasks_waiting_timeout = 0; s.key_exti_available = 1;
    CHECK_EQ_DESC("refuse: RTC down + tasks waiting -> TASKS_WAITING",
                  LpPolicy_RefuseReason(&s), LP_REFUSE_TASKS_WAITING);

    /* LSE 板预期空闲不足（<= guard）+ EXTI 全绿 -> NONE（定时深睡降级
     * 为 EXTI-only 候选，不算拒绝）*/
    s = snap_lse_rtc(); s.expected_idle_ticks = LP_WAKE_GUARD_TICKS;
    s.no_tasks_waiting_timeout = 1; s.key_exti_available = 1;
    CHECK_EQ_DESC("refuse: short idle + EXTI eligible -> NONE",
                  LpPolicy_RefuseReason(&s), LP_REFUSE_NONE);

    /* NULL 防御：无法确认状态，按最保守归类（与 Decide NULL->浅睡一致）*/
    CHECK_EQ_DESC("refuse: NULL snapshot -> NOT_STANDBY (conservative)",
                  LpPolicy_RefuseReason(NULL), LP_REFUSE_NOT_STANDBY);

    /* 枚举完整性：COUNT 覆盖全部原因值（计数字组边界单一来源）*/
    CHECK_EQ_DESC("refuse: LP_REFUSE_COUNT covers all reasons",
                  (int)LP_REFUSE_COUNT, (int)LP_REFUSE_NO_EXTI + 1);
    CHECK_EQ_DESC("refuse: LP_REFUSE_NONE == 0 (index 0 for no refusal)",
                  (int)LP_REFUSE_NONE, 0);

    /* 一致性性质：RefuseReason==NONE <=> Decide 是深睡路径之一 */
    s = snap_lse_rtc();
    CHECK_EQ_DESC("refuse: NONE <=> deep sleep decision (LSE RTC)",
                  LpPolicy_RefuseReason(&s) == LP_REFUSE_NONE,
                  LpPolicy_Decide(&s) != LP_SHALLOW_TICKLESS);
    s = snap_exti(); s.watchdog_requires_service = 1;
    CHECK_EQ_DESC("refuse: refused <=> shallow decision (IWDG)",
                  LpPolicy_RefuseReason(&s) != LP_REFUSE_NONE,
                  LpPolicy_Decide(&s) == LP_SHALLOW_TICKLESS);
    s = snap_exti(); s.buzzer_idle = 0;
    CHECK_EQ_DESC("refuse: refused <=> shallow decision (buzzer)",
                  LpPolicy_RefuseReason(&s) != LP_REFUSE_NONE,
                  LpPolicy_Decide(&s) == LP_SHALLOW_TICKLESS);
}
/*--------------------------- main ---------------------------*/

int main(void)
{
    s0_pure_function();
    s1_three_decisions();
    s2_common_stop_gate();
    s3_rtc_path_conditions();
    s4_exti_path_conditions();
    s5_board_gate();
    s6_watchdog_service();
    s7_refuse_reason();

    printf("lowpower-policy mock: %d checks, %d failed\n",
           g_total, g_failed);
    return (g_failed == 0) ? 0 : 1;
}
