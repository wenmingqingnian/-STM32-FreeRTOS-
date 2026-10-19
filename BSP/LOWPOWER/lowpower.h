/*
 * FreeRTOS 三路 Tickless/STOP wrapper
 *
 * 职责：把 FreeRTOS IdleTask 的 portSUPPRESS_TICKS_AND_SLEEP 入口接到
 * 板型感知的三路分发（决策单一来源 LpPolicy_Decide，Task 11）：
 *   1. LP_STOP_RTC       —— RTC Alarm 定时 STOP（仅 BOARD_C8_LSE，
 *                           设计 §6.5/§6.6 RTC 序列 + vTaskStepTick 补偿）；
 *   2. LP_STOP_EXTI_ONLY —— 无限期按键 EXTI STOP（本板唯一深睡路径，
 *                           复用 Task 12 ExtiStop 后端；tick 冻结不补偿）；
 *   3. LP_SHALLOW_TICKLESS —— 官方 SysTick 浅 Tickless（port.c weak
 *                           vPortSuppressTicksAndSleep，不复制算法）。
 *
 * 分层契约（项目惯例）：
 *   - 本模块是目标侧接线层：直接访问 SPL 寄存器与 FreeRTOS API，
 *     不进 host mock 链接集（决策/换算/后端序列均已 mock 覆盖：
 *     test_lowpower_policy / test_lowpower_math / test_exti_stop_backend /
 *     test_rtc_timebase）；
 *   - 快照输入经 LowPower_Sources_t 由 APP 层绑定注入（BSP 不反向
 *     依赖 APP_TASK，同 TaskHealth_Bind* 先例）；
 *   - 拒绝原因分别计数（LpRefuse_t 分类单一来源，统计经 LowPower_SnapshotStats 快照读取。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 * 设计：IAP + STOP 低功耗分层设计
 * 证据：docs/design-evidence/TICKLESS_STOP_CALLCHAIN.md
 *
 */
#ifndef __LOWPOWER_H__
#define __LOWPOWER_H__

#include <stddef.h>            /* NULL — ARMCC 与 host gcc 通用 */
#include "sys.h"               /* u8/u32 + stm32f10x.h（目标侧） */
#include "FreeRTOS.h"          /* TickType_t */
#include "task.h"              /* eTaskConfirmSleepModeStatus 语义 */
#include "lowpower_policy.h"   /* LpPolicySnapshot_t / LpDecision_t / LpRefuse_t */

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * 快照来源绑定（APP 层 System_Init 注入；lowpower_policy.h L92-101
 * 的组装责任表）。任一回调返回 0 即对应 STOP 门槛不满足。
 *
 * APP 侧适配责任（app_tasks.c）：
 *   standby_mode          <- g_systemState.currentMode == MODE_STANDBY
 *   motor_physically_off  <- TIM1 CCER 通道关闭 + MOTOR_EN 引脚低电平
 *   buzzer_idle           <- StandbyBuzzer_StopPermitted(队列空, 输出无效)
 *   iap_idle              <- APP 侧无 IAP 会话（请求即复位进 Boot，恒 1）
 *   uart_dma_idle         <- USART1 TC（发送完成）标志
 *   key_exti_available    <- KEY EXTI 线已配置（KeyExti_TargetInit 后）
 *----------------------------------------------------------*/
typedef struct {
    void *self;
    u8   (*standby_mode)(void *self);
    u8   (*motor_physically_off)(void *self);
    u8   (*buzzer_idle)(void *self);
    u8   (*iap_idle)(void *self);
    u8   (*uart_dma_idle)(void *self);
    u8   (*key_exti_available)(void *self);
} LowPower_Sources_t;

/*-----------------------------------------------------------
 * 绑定快照来源（NULL 拒绝：wrapper 保持"未绑定"态，全部走官方浅睡，
 * 绝不深睡——绑定缺失时对硬件状态一无所知，深睡不安全）
 *----------------------------------------------------------*/
void LowPower_BindSources(const LowPower_Sources_t *sources);

/*-----------------------------------------------------------
 * 目标侧装配（StartTask 在 KeyExti_TargetInit 之后调用）：
 * ExtiStop 后端 IO 适配 +（LSE 板型）RTC 时间基初始化
 *----------------------------------------------------------*/
void LowPower_TargetInit(void);

/*-----------------------------------------------------------
 * Tickless 入口（portmacro.h 路由目标；IdleTask 上下文，
 * 调度器已由内核 vTaskSuspendAll 挂起）
 *----------------------------------------------------------*/
void LowPower_SuppressTicksAndSleep(TickType_t xExpectedIdleTime);

/*-----------------------------------------------------------
 * 诊断统计快照（DiagnosticTask 唤醒后输出；读取走临界区，
 * 与 DWT_SnapshotAndReset 同一惯例）
 *
 * 拒绝原因分计数：refuse_counts[] 按 LpRefuse_t
 * 独立累加，不折叠成单一失败值；下标 0（LP_REFUSE_NONE）恒为 0。
 *----------------------------------------------------------*/
typedef struct {
    u32 shallow_count;         /* 官方浅 Tickless 路径次数 */
    u32 rtc_stop_count;        /* RTC 定时 STOP 完整次数（唤醒且时钟恢复成功）*/
    u32 exti_stop_count;       /* EXTI-only STOP 完整次数 */
    u32 rtc_degrade_count;     /* RTC 路径不可用降级（预算 0 / 序列失败）*/
    u32 refuse_counts[LP_REFUSE_COUNT];   /* 拒绝原因分计数（含下标 0 保留位）*/
    u32 wake_key;              /* 按键 EXTI 唤醒 */
    u32 wake_alarm;            /* RTC Alarm（EXTI17）唤醒 */
    u32 wake_other;            /* 其他 EXTI 线唤醒 */
    u32 wake_none;             /* 非 EXTI 唤醒（调试器等）*/
    u32 clk_resume_fail;       /* STOP 唤醒后时钟恢复失败（fail-safe 复位前累加）*/
    u32 tick_clamp_count;      /* vTaskStepTick 步进被 clamp（设计 §6.5）*/
} LowPower_Stats_t;

void LowPower_SnapshotStats(LowPower_Stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __LOWPOWER_H__ */
