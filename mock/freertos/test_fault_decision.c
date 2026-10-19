// SPDX-License-Identifier: GPL-2.0
/*
 * Host-side mock test for fault-injection pure logic:
 *   1. Motor_StallCheck()        - 堵转判定：门限/窗口/宽限期/锁存/
 *      闭环 vs 开环路径 + 中断复位 + NULL 防御
 *   2. Sensor_HealthCheck()      - 传感器组健康：失败/成功计数边界 +
 *      首次超时/恢复边沿 + 超时保持重发 + NULL 防御
 *   3. Queue_FullPolicy() /
 *      Queue_ResolveFullAction() - 队列满策略：安全关键挤占重试 vs
 *      非关键丢弃计数 + 计数增量语义 + 挤占失败矛盾路径
 *
 * Production source under test (APP_TASK\fault_decision.c) is compiled
 * UNMODIFIED; only sys.h basic types come from stub/sys.h. No FreeRTOS /
 * SPL / hardware dependencies — pure logic is fully isolated.
 *
 * 覆盖的4类异常路径（停止条件第7节）：
 *   堵转 / 传感器超时 / 队列满 / IAP拒绝(已由 step6 mock 覆盖，本文件引用)
 */
#include <stdio.h>
#include <string.h>
#include "fault_decision.h"

/* ---- tiny check framework (same shape as test_task_health.c) ---- */
static int g_failed = 0;
static int g_total = 0;

#define CHECK_DESC(desc, cond) do {                       \
    g_total++;                                            \
    if (cond) { printf("  ok %s\n", desc); }              \
    else      { g_failed++; printf("  FAIL %s\n", desc); } \
} while (0)

#define CHECK_EQ_DESC(desc, got, want) do {               \
    g_total++;                                            \
    if ((unsigned long)(got) == (unsigned long)(want))    \
        printf("  ok %s\n", desc);                        \
    else { g_failed++;                                    \
        printf("  FAIL %s: got %lu, want %lu\n",          \
               desc, (unsigned long)(got),                \
               (unsigned long)(want)); }                  \
} while (0)

/*-----------------------------------------------------------
 * 1. 电机堵转判定
 *----------------------------------------------------------*/
static void test_stall_not_running(void)
{
    StallCheckCtx_t c;
    StallVerdict_t v;

    printf("suite stall-not-running\n");

    /* 电机未运行：即使其他条件满足也不判定 */
    memset(&c, 0, sizeof(c));
    c.motorRunning = 0;
    c.pidActive = 1;
    c.commandedTarget = 200;
    c.speed = 0.0f;
    v = Motor_StallCheck(&c);
    CHECK_EQ_DESC("未运行 -> STALL_NONE", v, STALL_NONE);
    CHECK_EQ_DESC("未运行 stallCounter 不变", c.stallCounter, 0);
}

static void test_stall_latched(void)
{
    StallCheckCtx_t c;
    StallVerdict_t v;

    printf("suite stall-latched\n");

    /* 故障已锁存：拒绝判定，避免反复触发 */
    memset(&c, 0, sizeof(c));
    c.motorRunning = 1;
    c.faultLatched = 1;
    c.pidActive = 1;
    c.commandedTarget = 200;
    c.speed = 0.0f;
    v = Motor_StallCheck(&c);
    CHECK_EQ_DESC("锁存 -> STALL_NONE", v, STALL_NONE);
}

static void test_stall_grace_period(void)
{
    StallCheckCtx_t c;
    StallVerdict_t v;
    int i;

    printf("suite stall-grace-period\n");

    /* 起转宽限期：startGrace 从 1000 每周期递减 50，递减后仍 >0 的周期
     * 不判定（避免正常起动被误判为堵转）。1000/50=20 周期里，前 19 周期
     * startGrace 递减后仍 >0（950..50），全 NONE；第 20 周期 startGrace
     * 50->0，判定开始（与原内联逻辑一致：先递减再判 ==0） */
    memset(&c, 0, sizeof(c));
    c.motorRunning = 1;
    c.pidActive = 1;
    c.commandedTarget = 200;
    c.speed = 0.0f;             /* 起转期间转速为0 */
    c.startGrace = MOTOR_START_GRACE_MS;
    for (i = 0; i < (MOTOR_START_GRACE_MS / FAULT_PERIOD_MS) - 1; i++)
    {
        v = Motor_StallCheck(&c);
        if (v != STALL_NONE) break;
    }
    CHECK_EQ_DESC("宽限保护期(前19周期)全 NONE", v, STALL_NONE);
    CHECK_EQ_DESC("19周期后 startGrace=50", c.startGrace, 50);
    CHECK_EQ_DESC("宽限保护期 stallCounter 仍为0", c.stallCounter, 0);
    /* 第20周期：startGrace 50->0，判定开始（speed=0 构成堵转条件） */
    v = Motor_StallCheck(&c);
    CHECK_EQ_DESC("第20周期判定开始 PENDING", v, STALL_PENDING);
    CHECK_EQ_DESC("判定开始后 startGrace=0", c.startGrace, 0);
    CHECK_EQ_DESC("第20周期 stallCounter=50", c.stallCounter, 50);
}

static void test_stall_closed_loop_confirm(void)
{
    StallCheckCtx_t c;
    StallVerdict_t v;
    int i;

    printf("suite stall-closed-loop-confirm\n");

    /* 闭环：目标200>60门限 且 实际0<60门限，连续500ms(10周期)确认。
     * 前9周期 PENDING，第10周期 CONFIRMED，stallCounter 复位 */
    memset(&c, 0, sizeof(c));
    c.motorRunning = 1;
    c.pidActive = 1;
    c.commandedTarget = 200;
    c.speed = 0.0f;
    for (i = 0; i < (MOTOR_STALL_WINDOW_MS / FAULT_PERIOD_MS) - 1; i++)
    {
        v = Motor_StallCheck(&c);
        if (v != STALL_PENDING) break;
    }
    CHECK_EQ_DESC("窗口未满前 STALL_PENDING", v, STALL_PENDING);
    CHECK_EQ_DESC("PENDING 期间 stallCounter=450", c.stallCounter, 450);
    v = Motor_StallCheck(&c);
    CHECK_EQ_DESC("窗口满 STALL_CONFIRMED", v, STALL_CONFIRMED);
    CHECK_EQ_DESC("确认后 stallCounter 复位", c.stallCounter, 0);
}

static void test_stall_open_loop_confirm(void)
{
    StallCheckCtx_t c;
    StallVerdict_t v;
    int i;

    printf("suite stall-open-loop-confirm\n");

    /* 开环：比较值200>150门限 且 实际0<60，连续500ms确认 */
    memset(&c, 0, sizeof(c));
    c.motorRunning = 1;
    c.pidActive = 0;
    c.openLoopPwm = 200;       /* > MOTOR_STALL_PWM_THRESHOLD(150) */
    c.speed = 0.0f;
    for (i = 0; i < (MOTOR_STALL_WINDOW_MS / FAULT_PERIOD_MS); i++)
    {
        v = Motor_StallCheck(&c);
    }
    CHECK_EQ_DESC("开环确认 STALL_CONFIRMED", v, STALL_CONFIRMED);
}

static void test_stall_condition_interrupt(void)
{
    StallCheckCtx_t c;
    StallVerdict_t v;

    printf("suite stall-condition-interrupt\n");

    /* 条件中途恢复（转速拉起）：stallCounter 必须复位重新计时 */
    memset(&c, 0, sizeof(c));
    c.motorRunning = 1;
    c.pidActive = 1;
    c.commandedTarget = 200;
    c.speed = 0.0f;
    Motor_StallCheck(&c);            /* +50 */
    Motor_StallCheck(&c);            /* +50 -> 100 */
    CHECK_EQ_DESC("累计2周期 stallCounter=100", c.stallCounter, 100);
    c.speed = 100.0f;                /* 转速恢复，超过门限 */
    v = Motor_StallCheck(&c);
    CHECK_EQ_DESC("条件中断 STALL_NONE", v, STALL_NONE);
    CHECK_EQ_DESC("中断后 stallCounter 复位", c.stallCounter, 0);
}

static void test_stall_below_threshold_no_stall(void)
{
    StallCheckCtx_t c;
    StallVerdict_t v;

    printf("suite stall-below-threshold\n");

    /* 闭环目标低于门限（如手动停机下发0）：不构成堵转条件 */
    memset(&c, 0, sizeof(c));
    c.motorRunning = 1;
    c.pidActive = 1;
    c.commandedTarget = 30;          /* < 60 门限 */
    c.speed = 0.0f;
    v = Motor_StallCheck(&c);
    CHECK_EQ_DESC("目标低于门限 STALL_NONE", v, STALL_NONE);
    CHECK_EQ_DESC("无堵转条件 stallCounter=0", c.stallCounter, 0);
}

static void test_stall_null_defense(void)
{
    printf("suite stall-null-defense\n");
    CHECK_EQ_DESC("NULL ctx -> STALL_NONE", Motor_StallCheck(0), STALL_NONE);
}

/*-----------------------------------------------------------
 * 2. 传感器组健康判定
 *----------------------------------------------------------*/
static void test_sensor_normal(void)
{
    SensorHealthCtx_t c;
    SensorHealthEvt_t e;

    printf("suite sensor-normal\n");

    /* 双传感器正常：无事件，failCount 保持0 */
    memset(&c, 0, sizeof(c));
    c.dhtOk = 1;
    c.mq2Ok = 1;
    e = Sensor_HealthCheck(&c);
    CHECK_EQ_DESC("正常 NONE", e, SENSOR_EVT_NONE);
    CHECK_EQ_DESC("failCount=0", c.failCount, 0);
}

static void test_sensor_timeout_first_detect(void)
{
    SensorHealthCtx_t c;
    SensorHealthEvt_t e;
    int i;

    printf("suite sensor-timeout-first-detect\n");

    /* 连续3次失败 -> 第3次首次确认 TIMEOUT（边沿触发）。
     * 注意首确认周期同时触发 HOLD（fault 仍为NONE）：e = TIMEOUT|HOLD */
    memset(&c, 0, sizeof(c));
    c.dhtOk = 0;
    c.mq2Ok = 1;                    /* 任一失败即失败 */
    c.currentFault = FAULT_NONE;
    for (i = 0; i < SENSOR_TIMEOUT_FAILS - 1; i++)
        e = Sensor_HealthCheck(&c);
    CHECK_EQ_DESC("前2次失败 NONE（计数中）", e, SENSOR_EVT_NONE);
    CHECK_EQ_DESC("计数2次 failCount=2", c.failCount, 2);
    CHECK_EQ_DESC("未确认 timeoutActive=0", c.timeoutActive, 0);
    e = Sensor_HealthCheck(&c);
    CHECK_EQ_DESC("第3次 TIMEOUT 位", (e & SENSOR_EVT_TIMEOUT) != 0, 1);
    CHECK_EQ_DESC("第3次 HOLD 位（fault仍NONE）", (e & SENSOR_EVT_TIMEOUT_HOLD) != 0, 1);
    CHECK_EQ_DESC("确认后 timeoutActive=1", c.timeoutActive, 1);
    CHECK_EQ_DESC("确认后 failCount 复位", c.failCount, 0);
}

static void test_sensor_timeout_hold(void)
{
    SensorHealthCtx_t c;
    SensorHealthEvt_t e;

    printf("suite sensor-timeout-hold\n");

    /* 已超时(timeoutActive=1)且 fault 仍为NONE：持续 HOLD 重发 */
    memset(&c, 0, sizeof(c));
    c.dhtOk = 0;
    c.mq2Ok = 0;
    c.currentFault = FAULT_NONE;
    c.timeoutActive = 1;            /* 已进入超时态 */
    e = Sensor_HealthCheck(&c);
    CHECK_EQ_DESC("保持态仅 HOLD 位", e, SENSOR_EVT_TIMEOUT_HOLD);
    CHECK_EQ_DESC("保持态 timeoutActive 不变", c.timeoutActive, 1);
}

static void test_sensor_timeout_hold_suppressed_when_fault_set(void)
{
    SensorHealthCtx_t c;
    SensorHealthEvt_t e;

    printf("suite sensor-hold-suppressed-when-fault-set\n");

    /* StateManager 已将 fault 置 SENSOR_TIMEOUT 后，保持重发被抑制
     *（避免无意义刷屏；降级已生效） */
    memset(&c, 0, sizeof(c));
    c.dhtOk = 0;
    c.mq2Ok = 0;
    c.currentFault = FAULT_SENSOR_TIMEOUT;
    c.timeoutActive = 1;
    e = Sensor_HealthCheck(&c);
    CHECK_EQ_DESC("fault已置 -> NONE（保持重发抑制）", e, SENSOR_EVT_NONE);
}

static void test_sensor_recover(void)
{
    SensorHealthCtx_t c;
    SensorHealthEvt_t e;
    int i;

    printf("suite sensor-recover\n");

    /* 超时后连续3次成功 -> 第3次恢复确认 RECOVERED，timeoutActive 清零 */
    memset(&c, 0, sizeof(c));
    c.dhtOk = 1;
    c.mq2Ok = 1;
    c.currentFault = FAULT_SENSOR_TIMEOUT;  /* fault 尚未清除 */
    c.timeoutActive = 1;
    for (i = 0; i < SENSOR_RECOVER_SUCCESSES - 1; i++)
        e = Sensor_HealthCheck(&c);
    CHECK_EQ_DESC("恢复计数中 NONE", e, SENSOR_EVT_NONE);
    CHECK_EQ_DESC("okCount=2", c.okCount, 2);
    e = Sensor_HealthCheck(&c);
    CHECK_EQ_DESC("第3次成功 RECOVERED", (e & SENSOR_EVT_RECOVERED) != 0, 1);
    CHECK_EQ_DESC("恢复后 timeoutActive=0", c.timeoutActive, 0);
    CHECK_EQ_DESC("恢复后 okCount 复位", c.okCount, 0);
}

static void test_sensor_recover_interrupt(void)
{
    SensorHealthCtx_t c;
    SensorHealthEvt_t e;

    printf("suite sensor-recover-interrupt\n");

    /* 恢复计数中途再次失败：okCount 清零，重新计时 */
    memset(&c, 0, sizeof(c));
    c.dhtOk = 1;
    c.mq2Ok = 1;
    c.currentFault = FAULT_SENSOR_TIMEOUT;
    c.timeoutActive = 1;
    Sensor_HealthCheck(&c);          /* okCount=1 */
    Sensor_HealthCheck(&c);          /* okCount=2 */
    CHECK_EQ_DESC("2次成功 okCount=2", c.okCount, 2);
    c.dhtOk = 0;                     /* 又失败 */
    e = Sensor_HealthCheck(&c);
    CHECK_EQ_DESC("中断 NONE", e, SENSOR_EVT_NONE);
    CHECK_EQ_DESC("中断后 okCount=0", c.okCount, 0);
    CHECK_EQ_DESC("中断后 timeoutActive 仍1", c.timeoutActive, 1);
}

static void test_sensor_null_defense(void)
{
    printf("suite sensor-null-defense\n");
    CHECK_EQ_DESC("NULL ctx -> NONE", Sensor_HealthCheck(0), SENSOR_EVT_NONE);
}

/*-----------------------------------------------------------
 * 3. 队列满策略
 *----------------------------------------------------------*/
static void test_queue_policy(void)
{
    printf("suite queue-policy\n");

    CHECK_EQ_DESC("安全关键 -> EVICT_RETRY",
        Queue_FullPolicy(1), QF_POLICY_EVICT_RETRY);
    CHECK_EQ_DESC("非关键 -> DROP_COUNT",
        Queue_FullPolicy(0), QF_POLICY_DROP_COUNT);
}

static void test_queue_action_delivered(void)
{
    u8 inc = 0xFF;
    QueueFullAction_t a;

    printf("suite queue-action-delivered\n");

    a = Queue_ResolveFullAction(QF_POLICY_DROP_COUNT, 1, 0, &inc);
    CHECK_EQ_DESC("发送成功 DELIVERED", a, QF_ACT_DELIVERED);
    CHECK_EQ_DESC("成功 inc=0", inc, 0);

    /* 策略无关：只要 sendOk=1 即 DELIVERED */
    a = Queue_ResolveFullAction(QF_POLICY_EVICT_RETRY, 1, 0, &inc);
    CHECK_EQ_DESC("EVICT_RETRY 策略下成功也 DELIVERED", a, QF_ACT_DELIVERED);
    CHECK_EQ_DESC("成功 inc=0（重试）", inc, 0);
}

static void test_queue_action_dropped(void)
{
    u8 inc = 0xFF;
    QueueFullAction_t a;

    printf("suite queue-action-dropped\n");

    /* 非关键：发送失败 -> 丢弃+计数 */
    a = Queue_ResolveFullAction(QF_POLICY_DROP_COUNT, 0, 0, &inc);
    CHECK_EQ_DESC("非关键失败 DROPPED", a, QF_ACT_DROPPED);
    CHECK_EQ_DESC("丢弃 inc=1", inc, 1);
}

static void test_queue_action_evict_retry(void)
{
    u8 inc = 0xFF;
    QueueFullAction_t a;

    printf("suite queue-action-evict-retry\n");

    /* 安全关键：发送失败 + 挤占成功 -> 重试+计数 */
    a = Queue_ResolveFullAction(QF_POLICY_EVICT_RETRY, 0, 1, &inc);
    CHECK_EQ_DESC("挤占成功 EVICT_RETRY", a, QF_ACT_EVICT_RETRY);
    CHECK_EQ_DESC("挤占 inc=1", inc, 1);
}

static void test_queue_action_evict_failed(void)
{
    u8 inc = 0xFF;
    QueueFullAction_t a;

    printf("suite queue-action-evict-failed\n");

    /* 安全关键：发送失败 + 挤占失败（队列已空矛盾）-> 不计数 */
    a = Queue_ResolveFullAction(QF_POLICY_EVICT_RETRY, 0, 0, &inc);
    CHECK_EQ_DESC("挤占失败 EVICT_FAILED", a, QF_ACT_EVICT_FAILED);
    CHECK_EQ_DESC("挤占失败 inc=0（无诊断证据）", inc, 0);
}

static void test_queue_null_counter(void)
{
    QueueFullAction_t a;

    printf("suite queue-null-counter\n");

    /* counterInc 可为NULL（调用方不关心计数时） */
    a = Queue_ResolveFullAction(QF_POLICY_DROP_COUNT, 0, 0, (u8 *)0);
    CHECK_EQ_DESC("NULL counterInc 仍返回 DROPPED", a, QF_ACT_DROPPED);
}

/*-----------------------------------------------------------
 * end-to-end：模拟 MotorTask 堵转故障的完整时序
 *----------------------------------------------------------*/
static void test_e2e_motor_stall_sequence(void)
{
    StallCheckCtx_t c;
    StallVerdict_t v;
    int i;

    printf("suite e2e-motor-stall-sequence\n");

    /* 完整时序：起转(宽限期)->稳定运行(无堵转)->堵转发生(窗口)->确认
     * 模拟电机正常起动1s后，转速掉到0，500ms后确认堵转 */
    memset(&c, 0, sizeof(c));
    c.motorRunning = 1;
    c.pidActive = 1;
    c.commandedTarget = 200;
    c.speed = 150.0f;               /* 起转中正常转速 */
    c.startGrace = MOTOR_START_GRACE_MS;

    /* 阶段1：宽限期内（20周期），即使转速低也不判定 */
    for (i = 0; i < MOTOR_START_GRACE_MS / FAULT_PERIOD_MS; i++)
        v = Motor_StallCheck(&c);
    CHECK_EQ_DESC("阶段1 宽限期内 NONE", v, STALL_NONE);

    /* 阶段2：宽限期结束，转速正常，无堵转条件 */
    c.speed = 150.0f;
    v = Motor_StallCheck(&c);
    CHECK_EQ_DESC("阶段2 正常运行 NONE", v, STALL_NONE);
    CHECK_EQ_DESC("阶段2 stallCounter=0", c.stallCounter, 0);

    /* 阶段3：堵转发生（转速掉到0），累计窗口 */
    c.speed = 0.0f;
    for (i = 0; i < MOTOR_STALL_WINDOW_MS / FAULT_PERIOD_MS - 1; i++)
        v = Motor_StallCheck(&c);
    CHECK_EQ_DESC("阶段3 窗口未满 PENDING", v, STALL_PENDING);

    /* 阶段4：第10周期确认 */
    v = Motor_StallCheck(&c);
    CHECK_EQ_DESC("阶段4 确认 CONFIRMED", v, STALL_CONFIRMED);
    CHECK_EQ_DESC("阶段4 stallCounter 复位", c.stallCounter, 0);
}

/*-----------------------------------------------------------
 * end-to-end：模拟传感器超时->恢复完整时序
 *----------------------------------------------------------*/
static void test_e2e_sensor_timeout_recover(void)
{
    SensorHealthCtx_t c;
    SensorHealthEvt_t e;
    int i;

    printf("suite e2e-sensor-timeout-recover\n");

    memset(&c, 0, sizeof(c));
    c.currentFault = FAULT_NONE;

    /* 阶段1：正常工作 */
    c.dhtOk = 1; c.mq2Ok = 1;
    e = Sensor_HealthCheck(&c);
    CHECK_EQ_DESC("阶段1 正常 NONE", e, SENSOR_EVT_NONE);

    /* 阶段2：传感器故障，累计3次失败首次确认 */
    c.dhtOk = 0;
    for (i = 0; i < SENSOR_TIMEOUT_FAILS; i++)
        e = Sensor_HealthCheck(&c);
    CHECK_EQ_DESC("阶段2 首确认 TIMEOUT 位", (e & SENSOR_EVT_TIMEOUT) != 0, 1);

    /* 阶段3：StateManager 处理事件置故障码后，保持重发被抑制 */
    c.currentFault = FAULT_SENSOR_TIMEOUT;
    e = Sensor_HealthCheck(&c);
    CHECK_EQ_DESC("阶段3 fault已置 保持重发抑制 NONE", e, SENSOR_EVT_NONE);

    /* 阶段4：传感器恢复，累计3次成功恢复确认 */
    c.dhtOk = 1; c.mq2Ok = 1;
    for (i = 0; i < SENSOR_RECOVER_SUCCESSES; i++)
        e = Sensor_HealthCheck(&c);
    CHECK_EQ_DESC("阶段4 恢复确认 RECOVERED 位", (e & SENSOR_EVT_RECOVERED) != 0, 1);
    CHECK_EQ_DESC("阶段4 timeoutActive 清零", c.timeoutActive, 0);
}

/*-----------------------------------------------------------
 * end-to-end：队列满三类故障注入
 *----------------------------------------------------------*/
static void test_e2e_queue_full_injection(void)
{
    u8 inc = 0;
    QueueFullAction_t a;

    printf("suite e2e-queue-full-injection\n");

    /* 故障注入1：电机STOP在满队列上 -> 挤占重试（必须送达） */
    a = Queue_ResolveFullAction(Queue_FullPolicy(1), 0, 1, &inc);
    CHECK_EQ_DESC("STOP满队列 挤占重试", a, QF_ACT_EVICT_RETRY);
    CHECK_EQ_DESC("STOP满队列 计数+1", inc, 1);

    /* 故障注入2：普通电机命令在满队列上 -> 丢弃+计数 */
    inc = 0;
    a = Queue_ResolveFullAction(Queue_FullPolicy(0), 0, 0, &inc);
    CHECK_EQ_DESC("普通命令满队列 丢弃", a, QF_ACT_DROPPED);
    CHECK_EQ_DESC("普通命令满队列 计数+1", inc, 1);

    /* 故障注入3：堵转事件在满队列上但挤占失败 -> 不计数（矛盾场景） */
    inc = 0;
    a = Queue_ResolveFullAction(Queue_FullPolicy(1), 0, 0, &inc);
    CHECK_EQ_DESC("堵转事件挤占失败 不计数", a, QF_ACT_EVICT_FAILED);
    CHECK_EQ_DESC("挤占失败 计数+0", inc, 0);
}

int main(void)
{
    printf("=== fault-injection mock ===\n");

    /* 1. 堵转 */
    test_stall_not_running();
    test_stall_latched();
    test_stall_grace_period();
    test_stall_closed_loop_confirm();
    test_stall_open_loop_confirm();
    test_stall_condition_interrupt();
    test_stall_below_threshold_no_stall();
    test_stall_null_defense();

    /* 2. 传感器 */
    test_sensor_normal();
    test_sensor_timeout_first_detect();
    test_sensor_timeout_hold();
    test_sensor_timeout_hold_suppressed_when_fault_set();
    test_sensor_recover();
    test_sensor_recover_interrupt();
    test_sensor_null_defense();

    /* 3. 队列满 */
    test_queue_policy();
    test_queue_action_delivered();
    test_queue_action_dropped();
    test_queue_action_evict_retry();
    test_queue_action_evict_failed();
    test_queue_null_counter();

    /* 4. 端到端时序 */
    test_e2e_motor_stall_sequence();
    test_e2e_sensor_timeout_recover();
    test_e2e_queue_full_injection();

    printf("\n=== %d checks, %d failed ===\n", g_total, g_failed);
    /* IAP 拒绝路径由 step6 mock 覆盖（57/57），本文件不重复 */
    printf("=== IAP rejection path: see step6 mock (test_iap_meta.c) ===\n");
    return g_failed ? 1 : 0;
}
