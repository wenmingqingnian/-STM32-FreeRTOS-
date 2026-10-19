// SPDX-License-Identifier: GPL-2.0
/*
 * IAP+STOP：待机阻塞决策 + IWDG 整合
 *
 * 生产源未修改编译进本套件：
 *   - APP_TASK/standby_policy.c      待机阻塞决策纯函数（本 Task 新增）
 *   - APP_TASK/task_health.c         required mask 切换 + 待机等待策略
 *   - BSP/LOWPOWER/lowpower_policy.c IWDG 服务判定 + 三态决策（Task 11）
 *
 * 链接集证明：不链接任何 FreeRTOS 源——"待机阻塞/唤醒广播"由纯函数
 * 决策表 + 事件位常量单一来源锁定；目标侧 xEventGroupWaitBits /
 * xQueueReceive(portMAX_DELAY) 的接线属 app_tasks.c（BUILD_VERIFIED，
 * Keil 0 错误 0 警告）。SpeedCalcTask 已事件驱动（TIM4 信号量无限期
 * take，不动），其"退出待机恢复"由 TIM4 中断链路保证（代码审查项）。
 *
 * Test first 对应：
 *   s1 StateManager 待机只发一次 STOP，之后阻塞到状态事件
 *   s2 Motor 已停时阻塞到命令；运行时仍保持 50ms PID
 *   s3 Sensor/UI/WindSpeed/Diagnostic 待机阻塞，ACTIVE 事件唤醒
 *   s4 Buzzer 队列非空或输出有效时禁止 STOP
 *   s5 Health 待机等待策略（mask 切换细则在 test_task_health.c）
 *   s6 软件 IWDG 关闭但 option byte 硬件 IWDG -> 仍需服务，EXTI-only 拒绝
 *   s7 watchdog budget 静态预算（LSI 上界 60kHz 最短超时 > 监督窗口）
 */
#include <stdio.h>
#include "standby_policy.h"
#include "task_health.h"
#include "watchdog.h"
#include "lowpower_policy.h"

/* ---- task_health.c 引用的 FreeRTOS 符号（host stub 声明于 stub/）----
 * HealthTask 循环不被测试执行，这里只提供链接桩。 */
void vTaskDelayUntil(TickType_t *last, TickType_t period)
{
    (void)last; (void)period;
}
TickType_t xTaskGetTickCount(void)
{
    return 0;
}
EventBits_t xEventGroupWaitBits(EventGroupHandle_t grp, EventBits_t bits,
                                BaseType_t clear, BaseType_t all,
                                TickType_t ticks)
{
    (void)grp; (void)bits; (void)clear; (void)all; (void)ticks;
    return 0;
}

static int g_failed = 0;
static int g_total = 0;

#define CHECK_DESC(desc, cond) do {                     \
    g_total++;                                          \
    if (cond) { printf("  ok %s\n", desc); }            \
    else      { g_failed++; printf("  FAIL %s\n", desc); } \
} while (0)

#define CHECK_EQ_DESC(desc, got, want) do {             \
    g_total++;                                          \
    if ((unsigned long)(got) == (unsigned long)(want))  \
        printf("  ok %s\n", desc);                      \
    else { g_failed++;                                  \
        printf("  FAIL %s: got %lu, want %lu\n",        \
               desc, (unsigned long)(got),              \
               (unsigned long)(want)); }                \
} while (0)

/* 满足公共 STOP 门槛 + EXTI-only 专属条件的基准快照（除 IWDG 外） */
static LpPolicySnapshot_t base_exti_snapshot(void)
{
    LpPolicySnapshot_t s;
    s.board = LP_BOARD_C8_NO_LSE;     /* NO_LSE 路径（实物已冻结 LSE）*/
    s.standby_mode = 1u;
    s.motor_physically_off = 1u;
    s.buzzer_idle = 1u;
    s.iap_idle = 1u;
    s.uart_dma_idle = 1u;
    s.clock_healthy = 1u;
    s.rtc_ready = 0u;                 /* NO_LSE 板型无 RTC */
    s.no_tasks_waiting_timeout = 1u;
    s.watchdog_requires_service = 0u;
    s.key_exti_available = 1u;
    s.expected_idle_ticks = 0u;       /* EXTI-only 不依赖该字段 */
    return s;
}

/* ---- s1 StateManager：待机只发一次 STOP，之后阻塞到状态事件 ---- */
static void test_state_mgr(void)
{
    printf("suite standby-state-mgr\n");

    /* 模式边沿：只在活动<->待机转换的边沿广播（不重复置位）*/
    CHECK_EQ_DESC("active->standby = EDGE_ENTER",
        StandbyMgr_DetectEdge(0u, 1u), STANDBY_EDGE_ENTER);
    CHECK_EQ_DESC("standby->active = EDGE_EXIT",
        StandbyMgr_DetectEdge(1u, 0u), STANDBY_EDGE_EXIT);
    CHECK_EQ_DESC("active->active = EDGE_NONE",
        StandbyMgr_DetectEdge(0u, 0u), STANDBY_EDGE_NONE);
    CHECK_EQ_DESC("standby->standby = EDGE_NONE",
        StandbyMgr_DetectEdge(1u, 1u), STANDBY_EDGE_NONE);

    /* STOP 只发一次：进入待机第一周期发，之后置 stop_sent 不再重发 */
    CHECK_EQ_DESC("standby & !sent -> send STOP (first cycle)",
        StandbyMgr_ShouldSendStop(1u, 0u), 1u);
    CHECK_EQ_DESC("standby & sent -> no STOP resend",
        StandbyMgr_ShouldSendStop(1u, 1u), 0u);
    CHECK_EQ_DESC("active -> never send STOP via standby path",
        StandbyMgr_ShouldSendStop(0u, 0u), 0u);
    CHECK_EQ_DESC("active & sent(stale) -> no STOP",
        StandbyMgr_ShouldSendStop(0u, 1u), 0u);

    /* 阻塞到状态事件：待机稳态（STOP 已发）才阻塞 */
    CHECK_EQ_DESC("standby & sent -> block on state event queue",
        StandbyMgr_ShouldBlockOnEvent(1u, 1u), 1u);
    CHECK_EQ_DESC("standby & !sent -> keep 50ms cycle (STOP pending)",
        StandbyMgr_ShouldBlockOnEvent(1u, 0u), 0u);
    CHECK_EQ_DESC("active -> keep 50ms decision cycle",
        StandbyMgr_ShouldBlockOnEvent(0u, 1u), 0u);
    CHECK_EQ_DESC("active & !sent -> keep 50ms cycle",
        StandbyMgr_ShouldBlockOnEvent(0u, 0u), 0u);
}

/* ---- s2 Motor：已停阻塞到命令；运行保持 50ms PID ---- */
static void test_motor(void)
{
    printf("suite standby-motor\n");

    CHECK_EQ_DESC("stopped & queue empty -> block on command",
        StandbyMotor_ShouldBlock(0u, 1u), 1u);
    CHECK_EQ_DESC("stopped & queue non-empty -> drain queue first",
        StandbyMotor_ShouldBlock(0u, 0u), 0u);
    CHECK_EQ_DESC("running & queue empty -> keep 50ms PID",
        StandbyMotor_ShouldBlock(1u, 1u), 0u);
    CHECK_EQ_DESC("running & queue non-empty -> keep 50ms PID",
        StandbyMotor_ShouldBlock(1u, 0u), 0u);
}

/* ---- s3 周期任务：待机阻塞在事件组，ACTIVE 位唤醒 ---- */
static void test_periodic(void)
{
    printf("suite standby-periodic\n");

    CHECK_EQ_DESC("standby -> block on event group",
        StandbyPeriodic_ShouldBlock(1u), 1u);
    CHECK_EQ_DESC("active -> keep periodic cycle",
        StandbyPeriodic_ShouldBlock(0u), 0u);

    /* 事件位常量单一来源：广播（StateManager）与等待（各任务）共用，
     * ACTIVE 独占 bit0、STANDBY 独占 bit1，两位互不重叠 */
    CHECK_EQ_DESC("ACTIVE bit == 0x01", STANDBY_EVT_ACTIVE_BIT, 0x01u);
    CHECK_EQ_DESC("STANDBY bit == 0x02", STANDBY_EVT_STANDBY_BIT, 0x02u);
    CHECK_DESC("ACTIVE/STANDBY bits disjoint",
        (STANDBY_EVT_ACTIVE_BIT & STANDBY_EVT_STANDBY_BIT) == 0u);

    /* Task 14 接线：周期任务独立唤醒位（Sensor/WindSpeed/UI/Diag 各
     * 独占一位，xClearOnExit 消费清零互不竞争——多等待者共享同一位
     * 时只有首个被解除阻塞者清位，其余不会被唤醒）；六位全互斥 */
    CHECK_EQ_DESC("SENSOR wake bit == 0x04", STANDBY_EVT_SENSOR_BIT, 0x04u);
    CHECK_EQ_DESC("WIND wake bit == 0x08", STANDBY_EVT_WIND_BIT, 0x08u);
    CHECK_EQ_DESC("UI wake bit == 0x10", STANDBY_EVT_UI_BIT, 0x10u);
    CHECK_EQ_DESC("DIAG wake bit == 0x20", STANDBY_EVT_DIAG_BIT, 0x20u);
    CHECK_DESC("all six standby event bits pairwise disjoint (0x3F)",
        (STANDBY_EVT_ACTIVE_BIT | STANDBY_EVT_STANDBY_BIT |
         STANDBY_EVT_SENSOR_BIT | STANDBY_EVT_WIND_BIT |
         STANDBY_EVT_UI_BIT | STANDBY_EVT_DIAG_BIT) == 0x3Fu);
}

/* ---- s4 Buzzer：队列非空或输出有效时禁止 STOP ---- */
static void test_buzzer(void)
{
    printf("suite standby-buzzer\n");

    CHECK_EQ_DESC("queue empty & output off -> STOP permitted",
        StandbyBuzzer_StopPermitted(1u, 0u), 1u);
    CHECK_EQ_DESC("queue non-empty -> STOP denied",
        StandbyBuzzer_StopPermitted(0u, 0u), 0u);
    CHECK_EQ_DESC("output active (beeping) -> STOP denied",
        StandbyBuzzer_StopPermitted(1u, 1u), 0u);
    CHECK_EQ_DESC("queue non-empty & output active -> STOP denied",
        StandbyBuzzer_StopPermitted(0u, 1u), 0u);

    /* 组装一致性：buzzer_idle == StopPermitted(...)（LpPolicy 快照字段）*/
    CHECK_DESC("permitted maps to buzzer_idle=1 for policy snapshot",
        StandbyBuzzer_StopPermitted(1u, 0u) == 1u);
}

/* ---- s5 Health：待机等待策略（IWDG 服务需求决定无限期/周期）---- */
static void test_health_wait(void)
{
    printf("suite standby-health-wait\n");

    /* 待机 + IWDG 无需服务：无限期事件阻塞（eNoTasksWaitingTimeout 前提）*/
    CHECK_EQ_DESC("standby & no IWDG service -> portMAX_DELAY",
        TaskHealth_WaitTicks(1u, 0u), portMAX_DELAY);
    /* 待机 + IWDG 需要服务：保持 2s 周期监督（强制浅睡的内核侧来源）*/
    CHECK_EQ_DESC("standby & IWDG service -> 2s window",
        TaskHealth_WaitTicks(1u, 1u), (TickType_t)TASK_HEALTH_WINDOW_MS);
    /* 活动态：恒 2s 周期窗口 */
    CHECK_EQ_DESC("active & no IWDG service -> 2s window",
        TaskHealth_WaitTicks(0u, 0u), (TickType_t)TASK_HEALTH_WINDOW_MS);
    CHECK_EQ_DESC("active & IWDG service -> 2s window",
        TaskHealth_WaitTicks(0u, 1u), (TickType_t)TASK_HEALTH_WINDOW_MS);

    /* IWDG 服务查询注入：未绑定时保守默认"需要服务"（误判方向安全：
     * 多醒只损失深睡机会，漏醒会在硬件 IWDG 下睡死咬狗）*/
    CHECK_EQ_DESC("IWDG query default (unbound) -> 1 (conservative)",
        TaskHealth_IwdgServiceNow(), 1u);
}

/* ---- s6 软件 IWDG 关 + option byte 硬件 IWDG -> EXTI-only 拒绝 ---- */
static void test_iwdg_exti_only(void)
{
    LpPolicySnapshot_t snap;

    printf("suite standby-iwdg-exti-only\n");

    /* 纯函数：软件关 + 硬件狗（OBR.WDG_SW=0）-> 仍需周期服务 */
    CHECK_EQ_DESC("sw off + hw IWDG (OBR WDG_SW=0) -> service required",
        Watchdog_RequiresPeriodicService(0u, 0u & ~LP_FLASH_OBR_WDG_SW), 1u);
    CHECK_EQ_DESC("sw off + sw watchdog (OBR WDG_SW=1) -> no service",
        Watchdog_RequiresPeriodicService(0u, LP_FLASH_OBR_WDG_SW), 0u);
    CHECK_EQ_DESC("sw on -> service required regardless of OBR",
        Watchdog_RequiresPeriodicService(1u, LP_FLASH_OBR_WDG_SW), 1u);

    /* 决策面：其余 EXTI-only 条件全满足，仅 IWDG 需要服务 -> 拒绝
     *（返回浅睡，绝不无限期 STOP）*/
    snap = base_exti_snapshot();
    snap.watchdog_requires_service = 1u;
    CHECK_EQ_DESC("all EXTI-only gates ok but IWDG needs service -> shallow",
        LpPolicy_Decide(&snap), LP_SHALLOW_TICKLESS);

    /* 对照：IWDG 无需服务时同快照 -> EXTI-only（证明拒绝确实由 IWDG 引起）*/
    snap.watchdog_requires_service = 0u;
    CHECK_EQ_DESC("same snapshot with no IWDG service -> EXTI-only STOP",
        LpPolicy_Decide(&snap), LP_STOP_EXTI_ONLY);

    /* 公共门槛：蜂鸣器忙（s4 组装）-> 浅睡，即使 IWDG 无需服务 */
    snap.buzzer_idle = 0u;
    CHECK_EQ_DESC("buzzer busy -> shallow (STOP gate)",
        LpPolicy_Decide(&snap), LP_SHALLOW_TICKLESS);
}

/* ---- s7 watchdog budget：LSI 上界最短超时 > 监督窗口 ---- */
static void test_watchdog_budget(void)
{
    printf("suite standby-watchdog-budget\n");

    /* 常量计算核对：reload=2500, prescaler=64（watchdog.h 单一来源）*/
    CHECK_EQ_DESC("reload == 2500 (nominal 4s @40kHz)",
        WATCHDOG_RELOAD, 2500uL);
    /* 最短超时 (2500+1)*64/60kHz = 2667ms（向下取整，更保守）*/
    CHECK_EQ_DESC("min timeout (LSI 60kHz) == 2668ms bound check",
        WATCHDOG_TIMEOUT_MIN_MS, 2667uL);
    /* 最长超时 (2500+1)*64/30kHz = 5335ms（向下取整）*/
    CHECK_EQ_DESC("max timeout (LSI 30kHz) == 5335ms",
        WATCHDOG_TIMEOUT_MAX_MS, 5335uL);

    /* 预算不等式：喂狗最大间隔（2s 监督窗口）< 最短超时。
     * 编译期 #error 已锁同一断言（task_health.h），此处运行时复查。*/
    CHECK_DESC("min timeout > supervision window (2000ms)",
        WATCHDOG_TIMEOUT_MIN_MS > TASK_HEALTH_WINDOW_MS);
    /* 裕量量化：667ms >> 事件唤醒/窗口处理耗时（微秒级）*/
    CHECK_DESC("budget margin >= 600ms",
        (WATCHDOG_TIMEOUT_MIN_MS - TASK_HEALTH_WINDOW_MS) >= 600uL);
}

int main(void)
{
    test_state_mgr();
    test_motor();
    test_periodic();
    test_buzzer();
    test_health_wait();
    test_iwdg_exti_only();
    test_watchdog_budget();

    printf("mock result: %d/%d cases passed (Task 14 standby blocking)\n",
           g_total - g_failed, g_total);
    return g_failed ? 1 : 0;
}
