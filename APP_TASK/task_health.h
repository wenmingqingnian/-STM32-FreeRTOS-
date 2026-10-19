/*
 * 任务心跳与健康监督模块：TaskHealth + HealthTask + IWDG
 *
 * 架构（监督链路与业务故障处理严格分离）：
 *
 *   StateManager ─┐
 *   MotorTask     ├─ 每完成一个有效周期 → TaskHealth_Heartbeat(bit)
 *   SensorTask    │（心跳只证明"任务还在跑"，与业务成败无关：
 *   SpeedCalcTask ┘      传感器读取失败任务照常报心跳，业务降级
 *                        由 StateManager 故障状态机负责，两层不混）
 *                      │
 *                      ▼
 *   HealthTask（2s 监督窗口）：原子交换位图 → TaskHealth_Decide()
 *                      │
 *            ┌─────────┴─────────┐
 *            ▼                   ▼
 *      心跳齐全：FEED        心跳缺失：HOLD
 *      Watchdog_Feed()      记录缺失位 + 边沿告警打印
 *（4s 超时后 IWDG 复位芯片）
 *
 * 设计要点：
 *   1. 原子性：位图 read-modify-write（置位）与 read-and-clear（交换）
 *      均在 FreeRTOS 临界区内完成，多任务并发置位不丢拍；
 *   2. 喂狗权唯一：业务任务禁止直接调用 Watchdog_Feed()，
 *      只有 HealthTask 判定齐全后喂——单任务卡死不会被其他任务掩盖；
 *   3. TaskHealth_Decide() 为纯函数（只依赖输入），可在主机 mock 上驱动测试；
 *   4. 窗口 2s：被监督任务最长周期为 SensorTask 1000ms，
 *      正常时每窗口每任务至少 2 拍心跳，判据留 2 倍裕量；
 *      IWDG 超时 4s = 窗口 2 倍，HOLD 一个整窗口后才触发硬件复位，
 *      避免瞬时调度抖动直接咬狗。
 *
 */
#ifndef __TASK_HEALTH_H
#define __TASK_HEALTH_H

#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"   /* Task 14：HealthTask 待机事件组等待 */
#include "watchdog.h"       /* Task 14：IWDG 静态预算编译期断言 */

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * 构建开关（默认开启：纯监督逻辑，无硬件副作用；
 * IWDG 硬件动作由 watchdog.h 的 WATCHDOG_ENABLED 独立控制）
 *----------------------------------------------------------*/
#ifndef TASK_HEALTH_ENABLED
#define TASK_HEALTH_ENABLED  1
#endif

/*-----------------------------------------------------------
 * 心跳位定义（四个关键任务）
 *----------------------------------------------------------*/
#define TASK_HEALTH_BIT_STATE_MGR     (1uL << 0)   /* StateManagerTask（控制平面写者） */
#define TASK_HEALTH_BIT_MOTOR         (1uL << 1)   /* MotorControlTask（电机硬件所有者） */
#define TASK_HEALTH_BIT_SENSOR        (1uL << 2)   /* SensorTask（数据源） */
#define TASK_HEALTH_BIT_SPEED_CALC    (1uL << 3)   /* SpeedCalcTask（转速测量） */

/* 必需心跳集合：任一缺失即判 HOLD（喂狗停止） */
#define TASK_HEALTH_REQUIRED_MASK     (TASK_HEALTH_BIT_STATE_MGR  | \
                                       TASK_HEALTH_BIT_MOTOR      | \
                                       TASK_HEALTH_BIT_SENSOR     | \
                                       TASK_HEALTH_BIT_SPEED_CALC)

/*-----------------------------------------------------------
 * 存活监督（Alive Supervision，可靠性升级）：
 * 位图只证明"任务置过位"（活动性），区分不了"正常跑满 40 拍"
 * 和"置位点过了、只干 1 拍就卡死"（置位点在循环开头时尤其危险）。
 * 升级：每任务维护本窗口"完成周期计数"，判定时同时检查
 *   (1) 位图齐全（活动性） 且  (2) 计数 >= 最小拍数（工作量）。
 * 任一不满足 -> HOLD 停喂狗，持续 HOLD 由 IWDG 兜底复位。
 *
 * 最小拍数设计：期望拍数 = 窗口 2000ms ÷ 任务周期；
 * 容差取期望的一半（2 倍裕量），吸收调度抖动与窗口相位差；
 * 低于该值意味着任务"实质工作轮次"不足——半路卡死的特征。
 *----------------------------------------------------------*/
#define TASK_HEALTH_COUNT_MAX        4u      /* 被监督任务数 = 心跳位宽 */

/* 心跳位 -> 计数下标（位号 == 下标，bit N 对应 1uL<<N） */
#define TASK_HEALTH_IDX_STATE_MGR    0u
#define TASK_HEALTH_IDX_MOTOR        1u
#define TASK_HEALTH_IDX_SENSOR       2u
#define TASK_HEALTH_IDX_SPEED_CALC   3u

/* 每窗口最小完成拍数：期望=窗口/周期，容差 50% */
#define TASK_HEALTH_MIN_COUNT_STATE_MGR    20u   /* 周期50ms   期望40拍 */
#define TASK_HEALTH_MIN_COUNT_MOTOR        20u   /* 周期50ms   期望40拍 */
#define TASK_HEALTH_MIN_COUNT_SENSOR        1u   /* 周期1000ms 期望 2拍 */
#define TASK_HEALTH_MIN_COUNT_SPEED_CALC   20u   /* 周期50ms   期望40拍 */

/*-----------------------------------------------------------
 * 监督窗口参数
 *----------------------------------------------------------*/
#define TASK_HEALTH_WINDOW_MS         2000uL       /* 监督窗口：最长任务周期1000ms的2倍 */

/*-----------------------------------------------------------
 * Task 14（设计 §6.7）：待机态必需心跳集合 = 0。
 * 待机期间被监督任务全部阻塞在事件/队列上（周期唤醒已消除），
 * "每窗口至少一拍"判据在待机态退化为空集；退出待机后的第一个
 * 窗口恢复全量判据（活动任务在 HealthTask 运行前已完成首拍）。
 *----------------------------------------------------------*/
#define TASK_HEALTH_STANDBY_REQUIRED_MASK   0uL

/*-----------------------------------------------------------
 * IWDG 静态安全预算（Task 14，常量单一来源 watchdog.h）：
 * 喂狗最大间隔 = 监督窗口 2s，必须 < LSI 上界 60kHz 下最短硬件
 * 超时 WATCHDOG_TIMEOUT_MIN_MS（2667ms）。编译期锁定本不等式；
 * mock test_standby_blocking.c s7 运行时复查同一断言。
 *----------------------------------------------------------*/
#if (WATCHDOG_TIMEOUT_MIN_MS <= TASK_HEALTH_WINDOW_MS)
#error "IWDG budget violated: watchdog min timeout must exceed health window"
#endif

/*-----------------------------------------------------------
 * 判定结果
 *----------------------------------------------------------*/
typedef enum {
    TASK_HEALTH_FEED = 0,      /* 必需位齐全：允许喂狗 */
    TASK_HEALTH_HOLD,          /* 存在缺失位：禁止喂狗（持续HOLD将触发IWDG复位） */
    TASK_HEALTH_VERDICT_MAX
} TaskHealthVerdict_t;

/*-----------------------------------------------------------
 * 运行统计（诊断任务/诊断可读）
 *----------------------------------------------------------*/
typedef struct {
    u32 windows;            /* 已完成监督窗口数 */
    u32 feedWindows;        /* 判定FEED的窗口数 */
    u32 holdWindows;        /* 判定HOLD的窗口数 */
    u32 lastMissing;        /* 最近一次HOLD窗口的缺失位图（0=无缺失） */
    u32 lastUnderflow;      /* 最近一次HOLD窗口的欠拍位图（0=无欠拍） */
    u32 lastCounts[TASK_HEALTH_COUNT_MAX]; /* 最近窗口各任务实际完成拍数（诊断量化：期望-实际=缺拍数） */
    u32 holdEver;           /* 历史上是否出现过HOLD（0=从未，启动以来任务全健康） */
} TaskHealthStats_t;

#if defined(TASK_HEALTH_ENABLED) && TASK_HEALTH_ENABLED

/* 任务心跳：被监督任务每完成一个有效周期调用一次（任务上下文） */
void TaskHealth_Heartbeat(u32 bit);

/* 原子交换心跳位图：读走当前窗口全部心跳并清零（仅 HealthTask 调用） */
u32 TaskHealth_SwapBeats(void);

/* 原子交换存活计数：读走当前窗口各任务完成周期计数并清零（仅 HealthTask 调用） */
void TaskHealth_SwapCounts(u32 *out);

/* 取运行统计快照（诊断输出用） */
void TaskHealth_GetStats(TaskHealthStats_t *out);

/* 健康监督任务（2s 窗口：交换→判定→喂狗/告警） */
void HealthTask(void *pvParameters);

/* Task 14：IWDG 周期服务需求查询注入。目标侧绑定
 * Watchdog_ServiceRequiredNow()（软件 IWDG 启动状态 + option byte
 * 硬件 IWDG 位）；未绑定时保守返回 1——误判"无需服务"会在硬件
 * IWDG 下睡死咬狗，反向误判只损失深睡机会（方向安全）。 */
u8   TaskHealth_IwdgServiceNow(void);
void TaskHealth_BindIwdgQuery(u8 (*query)(void));

/* Task 14：待机事件组绑定（StateManager 模式边沿广播
 * STANDBY_EVT_STANDBY_BIT / STANDBY_EVT_ACTIVE_BIT）*/
void TaskHealth_BindStandbyEvents(EventGroupHandle_t grp);

#else

/* 关闭时退化为空操作：心跳调用点零开销（保持业务代码不删） */

/* ArmCC v5 (C90) has no "inline" keyword - use compiler-portable variant */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#define TH_STATIC_INLINE static __inline
#elif defined(__GNUC__) || defined(__clang__)
#define TH_STATIC_INLINE static inline
#else
#define TH_STATIC_INLINE static
#endif

TH_STATIC_INLINE void TaskHealth_Heartbeat(u32 bit) { (void)bit; }
TH_STATIC_INLINE u32  TaskHealth_SwapBeats(void)    { return 0u; }
TH_STATIC_INLINE void TaskHealth_SwapCounts(u32 *out)
{
    if (out != (u32 *)0)
    {
        u32 i;
        for (i = 0u; i < TASK_HEALTH_COUNT_MAX; i++) out[i] = 0u;
    }
}
TH_STATIC_INLINE void TaskHealth_GetStats(TaskHealthStats_t *out)
{
    u32 i;

    if (out == (TaskHealthStats_t *)0) return;
    out->windows = out->feedWindows = out->holdWindows = 0;
    out->lastMissing = out->lastUnderflow = out->holdEver = 0;
    for (i = 0u; i < TASK_HEALTH_COUNT_MAX; i++) out->lastCounts[i] = 0u;
}
TH_STATIC_INLINE void HealthTask(void *pvParameters) { (void)pvParameters; }

/* Task 14：关闭时 IWDG 查询保守返回 1（保持周期监督语义），
 * 绑定接口退化为空操作 */
TH_STATIC_INLINE u8   TaskHealth_IwdgServiceNow(void) { return 1u; }
TH_STATIC_INLINE void TaskHealth_BindIwdgQuery(u8 (*query)(void)) { (void)query; }
TH_STATIC_INLINE void TaskHealth_BindStandbyEvents(EventGroupHandle_t grp) { (void)grp; }

#endif

/*-----------------------------------------------------------
 * 纯判定函数（不受构建开关影响，主机 mock 测试入口）：
 * 输入本窗口心跳位图与必需位集合，输出判定结果；
 * missing（可为 NULL）回写缺失位图 = required & ~beats。
 *----------------------------------------------------------*/
TaskHealthVerdict_t TaskHealth_Decide(u32 beats, u32 required, u32 *missing);

/*-----------------------------------------------------------
 * 存活监督纯判定（升级版，主机 mock 测试入口）：
 * 位图齐全 且 每个必需任务完成周期数 >= 最小拍数 -> FEED；
 * 任一缺失或欠拍 -> HOLD。
 *   beats      本窗口心跳位图
 *   required   必需位集合
 *   counts     本窗口各任务完成周期计数（长度 TASK_HEALTH_COUNT_MAX）
 *   min_counts 各任务最小拍数表（长度 TASK_HEALTH_COUNT_MAX，可 NULL=只查位图）
 *   missing（可 NULL）回写缺失位图
 *   underflow（可 NULL）回写欠拍位图（位号 == 计数下标）
 *----------------------------------------------------------*/
TaskHealthVerdict_t TaskHealth_DecideAlive(u32 beats, u32 required,
                                           const u32 *counts,
                                           const u32 *min_counts,
                                           u32 *missing, u32 *underflow);

/*-----------------------------------------------------------
 * Task 14 纯函数（不受构建开关影响，主机 mock 测试入口）
 *----------------------------------------------------------*/

/* 必需位集合随模式切换：活动 -> 全量；待机 -> 0（空集）*/
u32 TaskHealth_RequiredMaskFor(u8 standby);

/* HealthTask 等待策略：待机且 IWDG 无需服务 -> portMAX_DELAY
 *（无限期事件阻塞，内核才可能给出 eNoTasksWaitingTimeout）；
 * 其余（活动态或 IWDG 需要服务）-> 2s 监督窗口 */
TickType_t TaskHealth_WaitTicks(u8 standby, u8 iwdg_service);

/* Task 14：退出待机过渡窗口（exit grace）——ACTIVE 边沿唤醒的窗口，
 * 周期任务首拍尚未发生（beats=0），沿用待机空集判据；下一窗口起
 * 恢复全量。不做宽限则每次退出待机必然 HOLD 一个窗口（beats=0 +
 * 全量 mask），喂狗间隔拉长到 2 窗口=4s，LSI 快偏（>40kHz）时
 * 提前咬狗复位——正常操作路径不应触发 fail-safe。 */
u8 TaskHealth_EffectiveStandby(u8 standby, u8 exit_grace);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_HEALTH_H */
