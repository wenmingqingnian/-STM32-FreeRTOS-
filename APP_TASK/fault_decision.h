/*
 * 故障判定纯函数模块
 *
 * 背景：堵转检测、传感器组健康判定、队列满策略原本内联在
 * MotorTask / SensorTask / Motor_SendCommand / Buzzer_SendCommand /
 * State_SendEvent 中，无法在主机上单独驱动其故障路径（app_tasks.c
 * 依赖 FreeRTOS + 一众 BSP，不可整体编译）。按 task_health.Decide
 * 既有模式，把三类"判定"抽为纯函数：只依赖输入参数，无全局、
 * 无硬件副作用，可在主机 mock 上驱动测试。
 *
 * 设计要点：
 *   1. 纯函数：不读 g_systemState 等全局，全部输入由调用方经
 *      ctx 结构显式传入；判定结果（含需要发的事件）以返回值/位图
 *      回传，硬件副作用（motor_stop / State_SendEvent / 队列挤占）
 *      留在调用方任务上下文执行——判定与执行分层；
 *   2. 行为等价：抽函数仅做"搬位置"，不改变任何阈值/计数语义，
 *      抽取前后 Keil 0错误0警告 + mock 全绿即等价证据；
 *   3. 队列满策略集中化：安全关键命令（电机STOP/堵转事件）满时
 *      "挤占最旧+重试"，非关键"丢弃+计数"，统一由 Queue_FullPolicy/
 *      Queue_ResolveFullAction 描述，避免三处实现各自漂移。
 *
 */
#ifndef __FAULT_DECISION_H
#define __FAULT_DECISION_H

#include "sys.h"

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * 故障码（由 app_tasks.h 经本头引入，单一来源）
 *----------------------------------------------------------*/
#define FAULT_NONE              0   /* 无故障 */
#define FAULT_MOTOR_STALL       1   /* 电机堵转 */
#define FAULT_SENSOR_TIMEOUT    2   /* 传感器超时 */

/*-----------------------------------------------------------
 * 判定周期与阈值（决策参数集中于此，便于 mock 覆盖边界）
 *----------------------------------------------------------*/
/* 堵转检测（MotorTask 每周期执行）：
 * 命令值高于门限 且 实际转速低于门限，连续满足 WINDOW 即确认；
 * 起转后 START_GRACE 内不判定，避免正常起动被误判为堵转 */
#define FAULT_PERIOD_MS             50      /* 判定任务周期(ms)：MotorTask=50 */
#define MOTOR_STALL_RPM_THRESHOLD   60.0f   /* 实际转速堵转门限(RPM)，目标档位190/220留3倍裕量 */
#define MOTOR_STALL_PWM_THRESHOLD   150      /* 开环命令比较值门限(CCR)，低于此值不足以驱动属正常 */
#define MOTOR_STALL_WINDOW_MS       500      /* 堵转条件连续满足时长(ms) */
#define MOTOR_START_GRACE_MS        1000     /* 起转宽限期(ms) */

/* 传感器超时检测（SensorTask 每周期执行，周期=1000ms）：
 * DHT11/MQ2 任一失败即计一次失败，连续 FAILS 次判超时；
 * 超时后连续 SUCCESSES 次成功才判恢复（防单次抖动反复切换） */
#define SENSOR_TIMEOUT_FAILS        3        /* 连续失败次数阈值（3×1000ms=3s判定超时）*/
#define SENSOR_RECOVER_SUCCESSES    3        /* 超时后连续成功次数阈值 */
#define SENSOR_PERIOD_MS            1000     /* SensorTask 判定周期(ms) */

/*-----------------------------------------------------------
 * 1. 电机堵转判定纯函数
 *----------------------------------------------------------*/
typedef enum {
    STALL_NONE = 0,      /* 无堵转条件（或未运行/锁存/宽限期内）：stallCounter 复位或保持 */
    STALL_PENDING,       /* 条件成立但未达窗口：stallCounter += PERIOD */
    STALL_CONFIRMED      /* 连续满足窗口：调用方执行安全停机+锁存+上报 */
} StallVerdict_t;

typedef struct {
    /* 调用方填入（本周期输入） */
    u8   motorRunning;    /* 电机运行状态（MotorTask 私有真实硬件状态） */
    u8   faultLatched;    /* 堵转锁存：置位后拒绝判定，仅 CLEAR_FAULT 清除 */
    u16  startGrace;      /* 起转宽限期剩余(ms) */
    u8   pidActive;       /* 1=闭环RPM控制，0=开环PWM控制 */
    u16  commandedTarget; /* 最近下发的闭环目标转速(RPM) */
    float speed;          /* 实际转速(RPM) */
    u16  openLoopPwm;     /* 开环PWM比较值(CCR) */
    /* 进出（函数内更新） */
    u16  stallCounter;    /* 堵转条件连续满足时长(ms) */
} StallCheckCtx_t;

/* 纯判定：返回本周期堵转结果，并更新 ctx->startGrace（每周期-FAULT_PERIOD_MS）
 * 与 ctx->stallCounter。调用方在 STALL_CONFIRMED 时执行硬件副作用：
 *   motor_stop(); TIM_SetCompare1(TIM1,0); PID_Reset(); motorRunning=0;
 *   pidActive=0; openLoopPwm=0; faultLatched=1; stallCounter=0;
 *   State_SendEvent(STATE_EVENT_FAULT_STALL);
 * 注：startGrace 递减是无条件的（与原内联逻辑一致），即使电机未运行。 */
StallVerdict_t Motor_StallCheck(StallCheckCtx_t *ctx);

/*-----------------------------------------------------------
 * 2. 传感器组健康判定纯函数
 *----------------------------------------------------------*/
/* 返回值位图：一个周期可能同时置 TIMEOUT（首次确认）+ HOLD（保持重发），
 * 调用方按位顺序发送对应 StateEvent（先 TIMEOUT/RECOVERED，再 HOLD）。 */
typedef enum {
    SENSOR_EVT_NONE       = 0,
    SENSOR_EVT_TIMEOUT    = 1u << 0,   /* 首次超时确认：failCount 达 FAILS */
    SENSOR_EVT_RECOVERED  = 1u << 1,   /* 超时后恢复确认：okCount 达 SUCCESSES */
    SENSOR_EVT_TIMEOUT_HOLD = 1u << 2  /* 超时保持重发：timeoutActive && fault==NONE */
} SensorHealthEvt_t;  /* 位图 */

typedef struct {
    /* 调用方填入 */
    u8 dhtOk;           /* DHT11 读取校验成功(ret==1) */
    u8 mq2Ok;           /* MQ2 状态 == MQ2_OK */
    u8 currentFault;    /* g_systemState.fault（超时保持判据用） */
    /* 进出 */
    u8 failCount;       /* 连续失败计数 */
    u8 okCount;         /* 超时后连续成功计数 */
    u8 timeoutActive;   /* 本地超时上报状态（边沿触发，避免重复上报） */
} SensorHealthCtx_t;

/* 纯判定：返回本周期应发事件位图，并更新 ctx 内计数器/状态。
 * 语义与 app_tasks.c 原 SensorTask 内联块逐行等价。 */
SensorHealthEvt_t Sensor_HealthCheck(SensorHealthCtx_t *ctx);

/*-----------------------------------------------------------
 * 3. 队列满策略纯函数（电机命令队列/状态事件队列/蜂鸣器命令队列共用）
 *----------------------------------------------------------*/
typedef enum {
    QF_POLICY_DROP_COUNT = 0,   /* 非关键：丢弃 + 诊断计数+1 */
    QF_POLICY_EVICT_RETRY = 1   /* 安全关键：挤占最旧 + 重试 + 计数+1 */
} QueueFullPolicy_t;

typedef enum {
    QF_ACT_DELIVERED = 0,   /* 发送成功，无额外动作，计数+0 */
    QF_ACT_DROPPED,         /* 丢弃，计数+1 */
    QF_ACT_EVICT_RETRY,     /* 挤占成功，重试发送，计数+1 */
    QF_ACT_EVICT_FAILED     /* 挤占失败（队列已空矛盾），计数+0 */
} QueueFullAction_t;

/* 根据命令/事件是否安全关键，返回队列满策略：
 *   电机 STOP / 状态事件 FAULT_STALL -> EVICT_RETRY（必须送达）
 *   其他                                -> DROP_COUNT（可容忍丢失） */
QueueFullPolicy_t Queue_FullPolicy(u8 isSafetyCritical);

/* 根据发送结果与策略，决定满时动作。
 *   sendOk: xQueueSend 或 SendToFront 结果（1=成功0=失败）
 *   evictOk: 挤占 xQueueReceive 结果（1=成功取到最旧，0=失败）
 *   counterInc(out): 诊断计数应增量（0或1）
 * 调用方按返回动作执行：DROPPED->计数+返回失败；EVICT_RETRY->计数+
 *   Receive(oldest)+重试 SendToFront；EVICT_FAILED->返回失败。 */
QueueFullAction_t Queue_ResolveFullAction(QueueFullPolicy_t policy,
                                          u8 sendOk,
                                          u8 evictOk,
                                          u8 *counterInc);

#ifdef __cplusplus
}
#endif

#endif /* __FAULT_DECISION_H */
