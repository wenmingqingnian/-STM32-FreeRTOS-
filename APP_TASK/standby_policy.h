/*
 * 待机阻塞决策纯函数
 *
 * 职责：各任务"待机阻塞 / 活动周期"的等待决策单一来源——
 *   1. StateManager：进入待机只下发一次电机 STOP，之后阻塞到状态
 *      事件队列（不再每 50ms 重发 STOP，消除周期唤醒源）；
 *   2. Motor：已停且命令队列空 -> 阻塞到命令；运行时保持 50ms PID
 *（堵转判定依赖周期推进，运行态绝不阻塞）；
 *   3. 周期任务（Sensor/WindSpeed/UI/Diagnostic）：待机阻塞在事件组
 *      ACTIVE 位，退出待机被广播唤醒（SpeedCalcTask 已事件驱动：
 *      TIM4 信号量无限期 take，不经本模块）；
 *   4. Buzzer：命令队列非空或输出有效（鸣叫中）时禁止 STOP——
 *      唤醒在途的鸣叫序列会撕裂 on/off 时序。
 *
 * 纯函数契约：只读输入参数，无全局/寄存器/RTOS 访问；同一输入任意次
 * 调用结果一致。目标侧接线（xQueueReceive/xEventGroupWaitBits 的
 * portMAX_DELAY 阻塞与事件位广播）在 app_tasks.c，本模块只锁决策表
 *（host mock：test_standby_blocking.c 未修改编译本源文件）。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 * 设计：IAP + STOP 低功耗分层设计
 */
#ifndef __STANDBY_POLICY_H
#define __STANDBY_POLICY_H

#include "sys.h"     /* u8/u32：host stub 与目标 stm32f10x.h 均提供 */

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * 待机/活动广播事件位（单一来源：StateManager 置位，其余任务等待）
 *   ACTIVE  —— 退出待机：唤醒所有待机阻塞任务（含 HealthTask）
 *   STANDBY —— 进入待机：HealthTask 切待机监督语义（mask=0/无限期）
 * 两位互不重叠；置位只在模式边沿发生（StandbyMgr_DetectEdge），
 * 等待方 xClearOnExit=pdTRUE 消费后位自动清零。
 *----------------------------------------------------------*/
#define STANDBY_EVT_ACTIVE_BIT    0x01u
#define STANDBY_EVT_STANDBY_BIT   0x02u

/*-----------------------------------------------------------
 * 周期任务独立唤醒位（Task 14 接线，设计 §6.7"待机时阻塞在事件组"）：
 *   Sensor/WindSpeed/UI/Diagnostic 各独占一位，等待时 xClearOnExit=TRUE
 *   消费自己的位。不能四位共用 ACTIVE_BIT：FreeRTOS event group 语义
 *   下多等待者等同一位且 xClearOnExit=TRUE 时，首个被解除阻塞的等待者
 *   即清位，其余等待者不会被唤醒——独立位保证 EXIT 广播一次唤醒全部。
 *   EXIT 广播 = ACTIVE_BIT（HealthTask）| 四个周期位（一次 SetBits）。
 *----------------------------------------------------------*/
#define STANDBY_EVT_SENSOR_BIT    0x04u
#define STANDBY_EVT_WIND_BIT      0x08u
#define STANDBY_EVT_UI_BIT        0x10u
#define STANDBY_EVT_DIAG_BIT      0x20u

/*-----------------------------------------------------------
 * 模式边沿（StateManager 广播决策）：
 *   只在活动<->待机转换的边沿置位事件，稳态不重复广播——
 *   防止粘滞位让 HealthTask 的事件状态机误判二次进待机。
 *----------------------------------------------------------*/
typedef enum {
    STANDBY_EDGE_NONE = 0,      /* 稳态（活动->活动 / 待机->待机）*/
    STANDBY_EDGE_ENTER,         /* 活动 -> 待机：广播 STANDBY 位 */
    STANDBY_EDGE_EXIT           /* 待机 -> 活动：广播 ACTIVE 位 */
} StandbyEdge_t;

/*-----------------------------------------------------------
 * 模式边沿检测（广播决策输入）：只读上周期与本周期待机标志，
 * 稳态返回 EDGE_NONE（防粘滞位重复广播）
 *----------------------------------------------------------*/
StandbyEdge_t StandbyMgr_DetectEdge(u8 prev_standby, u8 now_standby);

/*-----------------------------------------------------------
 * StateManager 决策（Test first 1）
 *----------------------------------------------------------*/

/* 待机首个决策周期下发 STOP（stop_sent 尚未置位），之后不再重发 */
u8 StandbyMgr_ShouldSendStop(u8 standby, u8 stop_sent);

/* 待机稳态（STOP 已下发）：阻塞到状态事件队列，消除 50ms 周期唤醒 */
u8 StandbyMgr_ShouldBlockOnEvent(u8 standby, u8 stop_sent);

/*-----------------------------------------------------------
 * Motor 决策（Test first 2）
 *----------------------------------------------------------*/

/* 已停且命令队列空：阻塞到命令；运行时（或队列有待消费命令）保持周期 */
u8 StandbyMotor_ShouldBlock(u8 motor_running, u8 queue_empty);

/*-----------------------------------------------------------
 * 周期任务决策（Test first 3：Sensor/WindSpeed/UI/Diagnostic）
 *----------------------------------------------------------*/

/* 待机：阻塞在事件组 ACTIVE 位（portMAX_DELAY）；活动：保持原周期 */
u8 StandbyPeriodic_ShouldBlock(u8 standby);

/*-----------------------------------------------------------
 * Buzzer STOP 门槛（Test first 4，设计 §6.7）
 *----------------------------------------------------------*/

/* 命令队列空 且 输出无效（未在鸣叫）才允许 STOP；任一非空闲即拒绝 */
u8 StandbyBuzzer_StopPermitted(u8 queue_empty, u8 output_active);

#ifdef __cplusplus
}
#endif

#endif /* __STANDBY_POLICY_H */
