/*
 * EXTI-only 无限期 STOP 后端
 *
 * 职责：板型无关的"无 RTC 定时"STOP 执行器——仅在低功耗决策层
 *（Task 11 LpPolicy_Decide == LP_STOP_EXTI_ONLY）已确认
 *   1. eTaskConfirmSleepModeStatus() == eNoTasksWaitingTimeout
 *（无任何任务等待超时——tick 冻结不会错过内核 deadline）
 *   2. Watchdog_RequiresPeriodicService() == false
 *（软件 IWDG 未启动且 option byte 未选硬件 IWDG）
 * 前提下方可进入。后端在 Enter 入口再次校验这两个输入（防御性
 * 再确认，设计 §6.4 PRIMASK 下二次检查的参数面），任一不满足
 * 立即拒绝且零 IO 副作用。
 *
 * 本模块绝不设置 RTC Alarm（设计 §6.6：EXTI-only STOP 使用相同外设
 * 静默与 sleep race 检查，但不设置 RTC Alarm）：ExtiStop_Io_t 接口
 * 不存在任何闹钟回调，rtc_timebase.c 也不在本模块链接集——
 * 契约由接口形状与链接集双重证明。
 *
 * 唤醒后 tick 冻结不补偿（设计 §6.3/§6.6）：不调用 vTaskStepTick，
 * 不读 RTC。待机态无绝对时间依赖；唤醒开销造成的 tick 误差在
 * 设计文档声明（host mock 以 vTaskStepTick 计数==0 证明）。
 *
 * 进入序列（设计 §6.6 EXTI-only 进入，test s2 逐位锁定）：
 *   1. systick_stop        —— 停 SysTick（进入顺序 1）
 *   2. peripherals_quiesce —— 关闭 ADC/非唤醒 DMA/USART/LCD 背光等
 *                             耗电外设（进入顺序 4）
 *   3. key_exti_wake_config —— 配置按键 EXTI（PB1 上升沿）为唤醒源
 *（进入顺序 6"保留按键 EXTI"）
 *   4. clear_key_pending   —— 睡前清进入前旧 pending：防 pending
 *                             残留导致 WFI 立即返回的 sleep race
 *（进入顺序 6 后半）
 *   5. pwr_stop_mode_enter —— 清 PWR wake flag（CWUF）、PDDS=0、
 *                             LPDS=1（进入顺序 5）
 *   6. dsb_isb_wfi         —— DSB/ISB 屏障后 WFI（进入顺序 7）
 *
 * 退出序列（设计 §6.6 退出 1/2/3/5，PRIMASK 仍屏蔽由调用方保证）：
 *   1. read_exti_pending   —— 读取并保存唤醒源判定（退出顺序 1）
 *   2. SystemClock_RestoreAfterStop（公共时钟恢复，本模块直接调用；
 *      HSE/PLL/SWS 有界超时，SystemCoreClockUpdate 收尾）
 *   3. peripherals_resume  —— 恢复必要外设（退出顺序 5 前半）
 *   4. systick_resume      —— 恢复 SysTick（退出顺序 5 后半）
 *   时钟恢复失败 -> EXTI_STOP_ERR_CLOCK_RESUME 立即返回：外设与
 *   SysTick 保持关闭（HSI 8 MHz 下不得恢复按 72MHz 配置的
 *   PWM/UART/RTOS tick）；保持 H 桥关闭、累加错误计数并 fail-safe
 *   reset 是调用方（Task 15 wrapper）职责（设计 §6.6）。
 *
 * 纯逻辑 + IO 注入（项目惯例）：本模块不直接访问任何寄存器，也不
 * 依赖 FreeRTOS 头文件——Enter/Exit 的输入由 wrapper 从 RTOS 层
 * 组装传入（host mock 以"链接集无 FreeRTOS 源 + vTaskStepTick 桩
 * 计数"证明）。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#ifndef __EXTI_STOP_BACKEND_H__
#define __EXTI_STOP_BACKEND_H__

#include <stddef.h>            /* NULL — ARMCC 与 host gcc 通用 */
#include "sys.h"               /* u8/u32 */
#include "system_clock_resume.h"   /* 退出序列直接复用公共时钟恢复 */

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * 状态码
 *----------------------------------------------------------*/
typedef enum {
    EXTI_STOP_OK = 0,
    EXTI_STOP_ERR_PARAM,         /* NULL 参数 */
    EXTI_STOP_ERR_PRECONDITION,  /* 决策层前提未被后端确认（防御再校验） */
    EXTI_STOP_ERR_STATE,         /* Enter/Exit 状态机违例 */
    EXTI_STOP_ERR_CLOCK_RESUME   /* 唤醒后时钟恢复失败（调用方 fail-safe） */
} ExtiStop_Status_t;

/*-----------------------------------------------------------
 * 唤醒源分类（退出顺序 1：读取并保存 EXTI pending 判定）
 *----------------------------------------------------------*/
typedef enum {
    EXTI_STOP_WAKE_NONE = 0,     /* 无 EXTI pending（调试器等非 EXTI 唤醒） */
    EXTI_STOP_WAKE_KEY,         /* 按键 EXTI 线（key_line_mask 命中） */
    EXTI_STOP_WAKE_OTHER        /* 其他 EXTI 线 */
} ExtiStop_WakeSource_t;

/*-----------------------------------------------------------
 * EXTI-only STOP 后端接口（依赖注入）
 *
 * 目标侧适配（Task 15 wrapper 提供）：
 *   systick_stop / systick_resume : SysTick CTRL 使能位（或 port 层
 *                                   portSUPPRESS 配套接口）
 *   peripherals_quiesce / resume  : ADC/非唤醒 DMA+USART/LCD 背光等
 *                                   耗电外设静默与恢复（保留按键 EXTI）
 *   key_exti_wake_config          : PB1 上升沿 EXTI + NVIC（幂等）
 *   clear_key_pending             : EXTI_PR 按键位写 1 清除
 *   read_exti_pending             : EXTI_PR 原始读数（唤醒源判定）
 *   pwr_stop_mode_enter           : PWR->CR 清 CWUF，PDDS=0 LPDS=1
 *   dsb_isb_wfi                   : __DSB(); __ISB(); __WFI()
 *----------------------------------------------------------*/
typedef struct {
    void *self;
    void (*systick_stop)(void *self);
    void (*peripherals_quiesce)(void *self);
    void (*key_exti_wake_config)(void *self);
    void (*clear_key_pending)(void *self);
    u32  (*read_exti_pending)(void *self);
    void (*pwr_stop_mode_enter)(void *self);
    void (*dsb_isb_wfi)(void *self);
    void (*peripherals_resume)(void *self);
    void (*systick_resume)(void *self);
} ExtiStop_Io_t;

/*-----------------------------------------------------------
 * 后端上下文
 *----------------------------------------------------------*/
typedef struct {
    const ExtiStop_Io_t *io;
    const SystemClockResume_Io_t *clk_io;   /* 公共时钟恢复 IO */
    u32 key_line_mask;                      /* 按键 EXTI_PR 位掩码（PB1: 1<<1） */
    ExtiStop_Status_t status;               /* 最近一次操作结果 */
    ExtiStop_WakeSource_t wake_source;      /* 最近一次 Exit 的唤醒源 */
    u8 entered;                              /* Enter 成功至 Exit 完成期间置 1 */
} ExtiStopBackend_t;

/*-----------------------------------------------------------
 * 绑定 IO 与按键唤醒线掩码
 *----------------------------------------------------------*/
ExtiStop_Status_t ExtiStop_Init(ExtiStopBackend_t *bk,
                                const ExtiStop_Io_t *io,
                                const SystemClockResume_Io_t *clk_io,
                                u32 key_line_mask);

/*-----------------------------------------------------------
 * 进入 STOP（调用点应处于 PRIMASK 屏蔽下，设计 §6.4）
 *
 * 输入为决策层已确认的事实（no_tasks_waiting_timeout /
 * watchdog_requires_service），后端防御性再校验，任一不满足立即
 * 返回 EXTI_STOP_ERR_PRECONDITION 且零 IO 副作用。
 * WFI 阻塞在 dsb_isb_wfi 回调内；返回即已唤醒（Enter 本身不做
 * 退出动作——显式调用 ExtiStop_Exit）。
 *----------------------------------------------------------*/
ExtiStop_Status_t ExtiStop_Enter(ExtiStopBackend_t *bk,
                                  u8 no_tasks_waiting_timeout,
                                  u8 watchdog_requires_service);

/*-----------------------------------------------------------
 * 退出处理（唤醒后立即调用，PRIMASK 仍屏蔽由调用方保证）
 *
 * 顺序：读 EXTI pending 判定唤醒源 -> SystemClock_RestoreAfterStop ->
 * 恢复外设 -> 恢复 SysTick。时钟恢复失败返回
 * EXTI_STOP_ERR_CLOCK_RESUME（外设/SysTick 保持关闭，调用方
 * fail-safe）。不调用 vTaskStepTick（tick 冻结）。
 * Exit 一次性：无论成败，entered 都被清除。
 *----------------------------------------------------------*/
ExtiStop_Status_t ExtiStop_Exit(ExtiStopBackend_t *bk);

/*-----------------------------------------------------------
 * 最近一次 Exit 的唤醒源查询（Exit 前返回 WAKE_NONE）
 *----------------------------------------------------------*/
ExtiStop_WakeSource_t ExtiStop_GetWakeSource(const ExtiStopBackend_t *bk);

#ifdef __cplusplus
}
#endif

#endif /* __EXTI_STOP_BACKEND_H__ */
