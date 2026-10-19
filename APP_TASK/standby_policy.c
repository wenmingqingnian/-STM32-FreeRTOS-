/*
 * 待机阻塞决策纯函数实现
 *
 * 实现说明：
 *   - 全部为纯函数：无全局状态、无寄存器、无 RTOS API 调用；
 *   - 决策表与 mock 测试（test_standby_blocking.c s1-s4）逐条对应；
 *   - 目标侧接线见 app_tasks.c 各任务循环的阻塞分支。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#include "standby_policy.h"

/*-----------------------------------------------------------
 * 模式边沿：活动<->待机转换检测（StateManager 广播决策）
 *----------------------------------------------------------*/
StandbyEdge_t StandbyMgr_DetectEdge(u8 prev_standby, u8 now_standby)
{
    if ((prev_standby == 0u) && (now_standby != 0u))
    {
        return STANDBY_EDGE_ENTER;
    }
    if ((prev_standby != 0u) && (now_standby == 0u))
    {
        return STANDBY_EDGE_EXIT;
    }
    return STANDBY_EDGE_NONE;
}

/*-----------------------------------------------------------
 * StateManager：待机只发一次 STOP（stop_sent 置位后不重发）
 *----------------------------------------------------------*/
u8 StandbyMgr_ShouldSendStop(u8 standby, u8 stop_sent)
{
    return (u8)((standby != 0u) && (stop_sent == 0u));
}

/*-----------------------------------------------------------
 * StateManager：待机稳态（STOP 已下发）阻塞到状态事件队列
 *----------------------------------------------------------*/
u8 StandbyMgr_ShouldBlockOnEvent(u8 standby, u8 stop_sent)
{
    return (u8)((standby != 0u) && (stop_sent != 0u));
}

/*-----------------------------------------------------------
 * Motor：已停且队列空 -> 阻塞到命令（运行态保持 50ms PID）
 *----------------------------------------------------------*/
u8 StandbyMotor_ShouldBlock(u8 motor_running, u8 queue_empty)
{
    return (u8)((motor_running == 0u) && (queue_empty != 0u));
}

/*-----------------------------------------------------------
 * 周期任务：待机阻塞在事件组 ACTIVE 位
 *----------------------------------------------------------*/
u8 StandbyPeriodic_ShouldBlock(u8 standby)
{
    return (u8)(standby != 0u);
}

/*-----------------------------------------------------------
 * Buzzer STOP 门槛：队列空 且 输出无效（设计 §6.7）
 *----------------------------------------------------------*/
u8 StandbyBuzzer_StopPermitted(u8 queue_empty, u8 output_active)
{
    return (u8)((queue_empty != 0u) && (output_active == 0u));
}
