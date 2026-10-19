/*
 * 故障判定纯函数实现
 * 详见 fault_decision.h 设计说明。本文件只含纯逻辑，无硬件/全局依赖，
 * 主机 mock 可直接编译（仅需 sys.h 基本类型）。与原 app_tasks.c 内联
 * 块逐行等价，抽函数仅"搬位置"不改语义。
 */
#include "fault_decision.h"

/*-----------------------------------------------------------
 * 1. 电机堵转判定（等价于 app_tasks.c MotorControlTask 内 961-1010 行）
 *----------------------------------------------------------*/
StallVerdict_t Motor_StallCheck(StallCheckCtx_t *ctx)
{
    u8 stallCondition;

    if (ctx == (StallCheckCtx_t *)0)
    {
        return STALL_NONE;
    }

    /* 起转宽限期递减（无条件，与原逻辑一致） */
    if (ctx->startGrace > 0)
    {
        if (ctx->startGrace >= (u16)FAULT_PERIOD_MS)
        {
            ctx->startGrace -= (u16)FAULT_PERIOD_MS;
        }
        else
        {
            ctx->startGrace = 0;
        }
    }

    /* 仅在运行中、未锁存、宽限期已过时判定 */
    if (ctx->motorRunning && !ctx->faultLatched && ctx->startGrace == 0)
    {
        stallCondition = 0;
        if (ctx->pidActive)
        {
            /* 闭环：目标转速高于门限却测不到转速 */
            if ((float)ctx->commandedTarget > MOTOR_STALL_RPM_THRESHOLD &&
                ctx->speed < MOTOR_STALL_RPM_THRESHOLD)
            {
                stallCondition = 1;
            }
        }
        else
        {
            /* 开环：比较值足以驱动却测不到转速 */
            if (ctx->openLoopPwm > MOTOR_STALL_PWM_THRESHOLD &&
                ctx->speed < MOTOR_STALL_RPM_THRESHOLD)
            {
                stallCondition = 1;
            }
        }

        if (stallCondition)
        {
            ctx->stallCounter += (u16)FAULT_PERIOD_MS;
            if (ctx->stallCounter >= (u16)MOTOR_STALL_WINDOW_MS)
            {
                /* 确认堵转：复位计数器，由调用方执行硬件副作用 */
                ctx->stallCounter = 0;
                return STALL_CONFIRMED;
            }
            return STALL_PENDING;
        }
        else
        {
            /* 条件中断，重新计时 */
            ctx->stallCounter = 0;
            return STALL_NONE;
        }
    }

    /* 未运行 / 已锁存 / 宽限期内：stallCounter 保持不动 */
    return STALL_NONE;
}

/*-----------------------------------------------------------
 * 2. 传感器组健康判定（等价于 app_tasks.c SensorTask 内 362-395 行）
 *----------------------------------------------------------*/
SensorHealthEvt_t Sensor_HealthCheck(SensorHealthCtx_t *ctx)
{
    SensorHealthEvt_t evt = SENSOR_EVT_NONE;

    if (ctx == (SensorHealthCtx_t *)0)
    {
        return evt;
    }

    if (ctx->dhtOk && ctx->mq2Ok)
    {
        /* 传感器组健康：失败计数清零 */
        ctx->failCount = 0;
        if (ctx->timeoutActive)
        {
            if (++ctx->okCount >= (u8)SENSOR_RECOVER_SUCCESSES)
            {
                /* 恢复确认 */
                ctx->timeoutActive = 0;
                ctx->okCount = 0;
                evt |= SENSOR_EVT_RECOVERED;
            }
        }
    }
    else
    {
        /* 任一传感器失败 */
        ctx->okCount = 0;
        if (!ctx->timeoutActive)
        {
            if (++ctx->failCount >= (u8)SENSOR_TIMEOUT_FAILS)
            {
                /* 超时确认（边沿触发，避免重复上报） */
                ctx->timeoutActive = 1;
                ctx->failCount = 0;
                evt |= SENSOR_EVT_TIMEOUT;
            }
        }
    }

    /* 超时状态保持：timeoutActive 且系统故障码仍为NONE时重发TIMEOUT。
     * 注：StateManager 收到首次 TIMEOUT 后将 fault 置 SENSOR_TIMEOUT，
     * 此保持路径在 fault 被清除前每周期重发，确保降级被覆盖。 */
    if (ctx->timeoutActive && ctx->currentFault == FAULT_NONE)
    {
        evt |= SENSOR_EVT_TIMEOUT_HOLD;
    }

    return evt;
}

/*-----------------------------------------------------------
 * 3. 队列满策略（等价于 app_tasks.c Motor_SendCommand/Buzzer_SendCommand/
 *    State_SendEvent 内的满处理块）
 *----------------------------------------------------------*/
QueueFullPolicy_t Queue_FullPolicy(u8 isSafetyCritical)
{
    return isSafetyCritical ? QF_POLICY_EVICT_RETRY : QF_POLICY_DROP_COUNT;
}

QueueFullAction_t Queue_ResolveFullAction(QueueFullPolicy_t policy,
                                         u8 sendOk,
                                         u8 evictOk,
                                         u8 *counterInc)
{
    u8 inc = 0;
    QueueFullAction_t act;

    if (sendOk)
    {
        act = QF_ACT_DELIVERED;     /* 发送成功，计数+0 */
    }
    else if (policy == QF_POLICY_DROP_COUNT)
    {
        act = QF_ACT_DROPPED;       /* 非关键：丢弃+计数 */
        inc = 1;
    }
    else  /* QF_POLICY_EVICT_RETRY */
    {
        if (evictOk)
        {
            act = QF_ACT_EVICT_RETRY;  /* 挤占成功，重试+计数 */
            inc = 1;
        }
        else
        {
            act = QF_ACT_EVICT_FAILED; /* 挤占失败（队列已空矛盾），计数+0 */
        }
    }

    if (counterInc)
    {
        *counterInc = inc;
    }
    return act;
}
