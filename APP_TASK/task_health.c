/*
 * 任务心跳与健康监督模块实现
 *
 * 实现说明：
 *   - 心跳位图的置位（多任务并发）与交换（HealthTask 单点）都是
 *     read-modify-write，必须在 taskENTER_CRITICAL 临界区内完成；
 *   - HealthTask 用 vTaskDelayUntil 保证监督窗口确定周期；
 *   - HOLD 告警打印采用边沿触发（进入缺失打一次、恢复打一次），
 *     避免 2s 周期性刷屏；打印不在任何被测任务的 DWT 区间内；
 *   - 统计快照读取同样走临界区（与 DWT_SnapshotAndReset 同一惯例）。
 *
 */
#include "task_health.h"
#include "watchdog.h"
#include "standby_policy.h"   /* STANDBY_EVT_* 广播位（Task 14）*/
#include <stdio.h>

/*-----------------------------------------------------------
 * 模块状态
 *----------------------------------------------------------*/
static volatile u32 g_beats = 0;                /* 当前窗口心跳位图 */
static volatile u32 g_counts[TASK_HEALTH_COUNT_MAX] = { 0, 0, 0, 0 }; /* 当前窗口各任务完成周期计数 */
static TaskHealthStats_t g_stats = { 0, 0, 0, 0, 0, { 0, 0, 0, 0 }, 0 };
static u8 g_holding = 0;                        /* 边沿打印状态：1=处于HOLD告警中 */
#if defined(TASK_HEALTH_ENABLED) && TASK_HEALTH_ENABLED
/* Task 14：IWDG 服务查询注入 + 待机事件组 + HealthTask 本地待机视图
 *（g_standby 只在 HealthTask 上下文读写，由事件组消费边沿驱动翻转）*/
static u8 (*g_iwdgQuery)(void) = (u8 (*)(void))0;
static EventGroupHandle_t g_standbyEvents = (EventGroupHandle_t)0;
static u8 g_standby = 0;        /* 0=活动，1=待机 */
static u8 g_exitGrace = 0;      /* Task 14：退出待机过渡窗口标志 */
#endif

/*-----------------------------------------------------------
 * 纯判定函数：必需位齐全 → FEED；否则 → HOLD
 * 不依赖任何全局状态与硬件（主机 mock 测试入口）
 *----------------------------------------------------------*/
TaskHealthVerdict_t TaskHealth_Decide(u32 beats, u32 required, u32 *missing)
{
    u32 miss = required & ~beats;

    if (missing != (u32 *)0)
    {
        *missing = miss;
    }
    return (miss == 0u) ? TASK_HEALTH_FEED : TASK_HEALTH_HOLD;
}

/*-----------------------------------------------------------
 * 存活监督纯判定（升级版）：
 *   位图齐全 且 每个必需任务完成周期数 >= 最小拍数 -> FEED；
 *   任一缺失或欠拍 -> HOLD。
 * counts/min_counts 为 NULL 时退化为仅位图判定（与 Decide 等价）。
 * 位号与计数下标一一对应（bit N <-> counts[N]，N < 4）。
 *----------------------------------------------------------*/
TaskHealthVerdict_t TaskHealth_DecideAlive(u32 beats, u32 required,
                                           const u32 *counts,
                                           const u32 *min_counts,
                                           u32 *missing, u32 *underflow)
{
    u32 miss = required & ~beats;
    u32 under = 0u;
    u32 i;

    if ((counts != (const u32 *)0) && (min_counts != (const u32 *)0))
    {
        for (i = 0u; i < TASK_HEALTH_COUNT_MAX; i++)
        {
            if (((required & (1uL << i)) != 0u) &&
                (counts[i] < min_counts[i]))
            {
                under |= (1uL << i);   /* 欠拍：该任务窗口内完成周期数不足 */
            }
        }
    }

    if (missing != (u32 *)0)
    {
        *missing = miss;
    }
    if (underflow != (u32 *)0)
    {
        *underflow = under;
    }
    return ((miss == 0u) && (under == 0u)) ? TASK_HEALTH_FEED : TASK_HEALTH_HOLD;
}

/*-----------------------------------------------------------
 * Task 14 纯函数：必需位集合切换 + 等待策略决策表
 *----------------------------------------------------------*/
u32 TaskHealth_RequiredMaskFor(u8 standby)
{
    return (standby != 0u) ? TASK_HEALTH_STANDBY_REQUIRED_MASK
                           : TASK_HEALTH_REQUIRED_MASK;
}

TickType_t TaskHealth_WaitTicks(u8 standby, u8 iwdg_service)
{
    if ((standby != 0u) && (iwdg_service == 0u))
    {
        return portMAX_DELAY;                      /* 待机+IWDG 无需服务：无限期 */
    }
    return pdMS_TO_TICKS(TASK_HEALTH_WINDOW_MS);   /* 2s 监督窗口 */
}

u8 TaskHealth_EffectiveStandby(u8 standby, u8 exit_grace)
{
    return (u8)(((standby != 0u) || (exit_grace != 0u)) ? 1u : 0u);
}

#if defined(TASK_HEALTH_ENABLED) && TASK_HEALTH_ENABLED

/*-----------------------------------------------------------
 * 心跳位 -> 计数下标（存活监督）。位号 == 下标，bit N 对应 1uL<<N。
 * 未知位返回 TASK_HEALTH_COUNT_MAX 表示非法（调用方拒绝）。
 *----------------------------------------------------------*/
static u32 TaskHealth_IndexFor(u32 bit)
{
    u32 idx;
    switch (bit)
    {
        case TASK_HEALTH_BIT_STATE_MGR:   idx = TASK_HEALTH_IDX_STATE_MGR;   break;
        case TASK_HEALTH_BIT_MOTOR:       idx = TASK_HEALTH_IDX_MOTOR;       break;
        case TASK_HEALTH_BIT_SENSOR:      idx = TASK_HEALTH_IDX_SENSOR;      break;
        case TASK_HEALTH_BIT_SPEED_CALC:  idx = TASK_HEALTH_IDX_SPEED_CALC;  break;
        default:                          idx = TASK_HEALTH_COUNT_MAX;       break;
    }
    return idx;
}

/*-----------------------------------------------------------
 * 任务心跳：置位本窗口心跳位 + 递增本任务完成周期计数
 * 临界区保证多任务并发 read-modify-write 不丢位/不丢计数
 *----------------------------------------------------------*/
void TaskHealth_Heartbeat(u32 bit)
{
    u32 idx;

    if (bit == 0u)
    {
        return;     /* 非法位直接拒绝 */
    }
    idx = TaskHealth_IndexFor(bit);
    if (idx >= TASK_HEALTH_COUNT_MAX)
    {
        return;     /* 未知心跳位：不置位不计数（防误报） */
    }
    taskENTER_CRITICAL();
    {
        g_beats |= bit;
        g_counts[idx]++;
    }
    taskEXIT_CRITICAL();
}

/*-----------------------------------------------------------
 * 原子交换：读走当前窗口心跳并清零（仅 HealthTask 调用）
 * 交换后新到达的心跳进入下一窗口，不会跨窗口累积
 *----------------------------------------------------------*/
u32 TaskHealth_SwapBeats(void)
{
    u32 beats;

    taskENTER_CRITICAL();
    {
        beats  = g_beats;
        g_beats = 0u;
    }
    taskEXIT_CRITICAL();
    return beats;
}

/*-----------------------------------------------------------
 * 原子交换存活计数：读走当前窗口各任务完成周期计数并清零。
 * 与 SwapBeats 同临界区惯例；两处交换在 HealthTask 内紧邻调用，
 * 计数与位图属于同一窗口的快照（窗口边界不撕裂）。
 *----------------------------------------------------------*/
void TaskHealth_SwapCounts(u32 *out)
{
    u32 i;

    if (out == (u32 *)0)
    {
        return;
    }
    taskENTER_CRITICAL();
    {
        for (i = 0u; i < TASK_HEALTH_COUNT_MAX; i++)
        {
            out[i] = g_counts[i];
            g_counts[i] = 0u;
        }
    }
    taskEXIT_CRITICAL();
}

/*-----------------------------------------------------------
 * 统计快照
 *----------------------------------------------------------*/
void TaskHealth_GetStats(TaskHealthStats_t *out)
{
    if (out == (TaskHealthStats_t *)0)
    {
        return;
    }
    taskENTER_CRITICAL();
    {
        *out = g_stats;
    }
    taskEXIT_CRITICAL();
}

/*-----------------------------------------------------------
 * Task 14：IWDG 服务查询注入（未绑定保守默认 1）+ 待机事件组绑定
 *----------------------------------------------------------*/
u8 TaskHealth_IwdgServiceNow(void)
{
    if (g_iwdgQuery != (u8 (*)(void))0)
    {
        return g_iwdgQuery();
    }
    return 1u;
}

void TaskHealth_BindIwdgQuery(u8 (*query)(void))
{
    g_iwdgQuery = query;
}

void TaskHealth_BindStandbyEvents(EventGroupHandle_t grp)
{
    g_standbyEvents = grp;
}

/*-----------------------------------------------------------
 * 健康监督任务（2s 监督窗口；Task 14 起待机态事件驱动）
 *----------------------------------------------------------*/
void HealthTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(TASK_HEALTH_WINDOW_MS);
    u32 beats;
    u32 counts[TASK_HEALTH_COUNT_MAX];
    u32 missing;
    u32 underflow;
    TaskHealthVerdict_t verdict;

    /* 存活监督最小拍数表（存活监督升级；位号 == 下标） */
    static const u32 s_minCounts[TASK_HEALTH_COUNT_MAX] = {
        TASK_HEALTH_MIN_COUNT_STATE_MGR,     /* idx0  StateManager */
        TASK_HEALTH_MIN_COUNT_MOTOR,         /* idx1  MotorControl */
        TASK_HEALTH_MIN_COUNT_SENSOR,        /* idx2  Sensor       */
        TASK_HEALTH_MIN_COUNT_SPEED_CALC,    /* idx3  SpeedCalc    */
    };

    (void)pvParameters;

    /* 窗口基准对齐任务启动时刻，vTaskDelayUntil 保证长期平均恒为 2s */
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        /* Task 14（设计 §6.7）：等待策略由纯函数决策表给出——
         *   活动 / 待机+IWDG 需服务：2s 超时窗口（监督周期驱动）；
         *   待机+IWDG 无需服务：portMAX_DELAY 无限期阻塞在事件组
         *（内核因此可能给出 eNoTasksWaitingTimeout，这是
         *     EXTI-only STOP 的必要条件之一）。
         * 等待对象为模式边沿广播：进入待机置 STANDBY 位、退出置
         * ACTIVE 位（StateManager 边沿广播，稳态不重发，xClearOnExit
         * 消费即清零）。窗口只会因边沿提前结束（更短 -> 喂狗间隔
         * 只会更小，相对绝对周期的偏差方向安全）。 */
        {
            TickType_t wait;
            EventBits_t bits = 0u;

            wait = TaskHealth_WaitTicks(g_standby, TaskHealth_IwdgServiceNow());
            if (g_standbyEvents != (EventGroupHandle_t)0)
            {
                bits = xEventGroupWaitBits(g_standbyEvents,
                                           STANDBY_EVT_STANDBY_BIT | STANDBY_EVT_ACTIVE_BIT,
                                           pdTRUE, pdFALSE, wait);
            }
            else
            {
                /* 防御：未绑定事件组（接线遗漏）退化为纯 2s 延时窗口，
                 * 绝不无限期阻塞——喂狗节奏优先于深睡机会 */
                vTaskDelayUntil(&xLastWakeTime, xPeriod);
            }
            /* 双位并发（一个窗口体内两次快速边沿）时 ACTIVE 后判生效：
             * 保守回到活动监督，监督链路不死 */
            if ((bits & STANDBY_EVT_STANDBY_BIT) != 0u)
            {
                g_standby = 1u;
                g_exitGrace = 0u;
            }
            if ((bits & STANDBY_EVT_ACTIVE_BIT) != 0u)
            {
                g_standby = 0u;
                g_exitGrace = 1u;   /* 退出待机：本窗口沿待机判据（宽限）*/
            }
        }

        /* 1. 原子交换本窗口心跳位图 + 存活计数（同一窗口快照） */
        beats = TaskHealth_SwapBeats();
        TaskHealth_SwapCounts(counts);

        /* 2. 纯函数判定：待机态空集判据（周期唤醒已消除），
         *    活动态全量判据（mask 切换单一来源）。
         *    存活监督：位图齐全 且 每任务完成周期数 >= 最小拍数，
         *    才能 FEED——区分"正常跑满"与"置位点过了只干 1 拍就卡死"。
         *    退出待机过渡窗口（exit grace）：周期任务首拍尚未发生，
         *    本窗口沿待机空集判据 FEED 喂狗（喂狗间隔仍 <=2s），
         *    下一窗口恢复全量——避免每次退出待机必然 HOLD 一个窗口 */
        {
            u8 effStandby = TaskHealth_EffectiveStandby(g_standby, g_exitGrace);
            verdict = TaskHealth_DecideAlive(beats,
                                             TaskHealth_RequiredMaskFor(effStandby),
                                             counts, s_minCounts,
                                             &missing, &underflow);
            g_exitGrace = 0u;    /* 宽限只覆盖一个窗口 */
        }

        /* 3. 执行判定结果 */
        taskENTER_CRITICAL();
        {
            u32 i;

            g_stats.windows++;
            for (i = 0u; i < TASK_HEALTH_COUNT_MAX; i++)
            {
                g_stats.lastCounts[i] = counts[i];   /* 诊断量化：期望-实际=缺拍数 */
            }
            if (verdict == TASK_HEALTH_FEED)
            {
                g_stats.feedWindows++;
                g_stats.lastMissing = 0u;
                g_stats.lastUnderflow = 0u;
            }
            else
            {
                g_stats.holdWindows++;
                g_stats.lastMissing = missing;
                g_stats.lastUnderflow = underflow;
                g_stats.holdEver = 1u;
            }
        }
        taskEXIT_CRITICAL();

        if (verdict == TASK_HEALTH_FEED)
        {
            /* 心跳齐全：本模块是全系统唯一喂狗点（业务任务禁止喂狗） */
            Watchdog_Feed();

            /* HOLD → FEED 恢复边沿：打印恢复报告 */
            if (g_holding)
            {
                g_holding = 0;
                printf("Health: recovered, all heartbeats present\r\n");
            }
        }
        else
        {
            /* 心跳缺失或欠拍：记录并告警（边沿触发），停止喂狗；
             * 若持续 HOLD，IWDG 超时（4s=2个窗口）后硬件复位芯片 */
            if (!g_holding)
            {
                g_holding = 1;
                printf("Health: HOLD (missing=0x%02lX underflow=0x%02lX), stop feeding\r\n",
                       (unsigned long)missing, (unsigned long)underflow);
            }
        }
    }
}

#endif /* TASK_HEALTH_ENABLED */
