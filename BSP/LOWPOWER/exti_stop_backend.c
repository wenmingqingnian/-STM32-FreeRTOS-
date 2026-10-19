/*
 * EXTI-only 无限期 STOP 后端实现
 *
 * 纯 IO 注入序列实现：无任何直接寄存器访问、无 FreeRTOS 依赖、
 * 无 RTC 依赖（接口形状与链接集双重证明，见头文件注释）。
 * host mock（test_exti_stop_backend.c）注入脚本化 IO 逐位锁定
 * 进入/退出序列与失败传播。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#include "exti_stop_backend.h"

/*-----------------------------------------------------------
 * 绑定 IO 与按键唤醒线掩码
 *----------------------------------------------------------*/
ExtiStop_Status_t ExtiStop_Init(ExtiStopBackend_t *bk,
                                const ExtiStop_Io_t *io,
                                const SystemClockResume_Io_t *clk_io,
                                u32 key_line_mask)
{
    if ((bk == NULL) || (io == NULL) || (clk_io == NULL)) {
        return EXTI_STOP_ERR_PARAM;
    }

    bk->io           = io;
    bk->clk_io       = clk_io;
    bk->key_line_mask = key_line_mask;
    bk->status       = EXTI_STOP_OK;
    bk->wake_source  = EXTI_STOP_WAKE_NONE;
    bk->entered      = 0u;

    return EXTI_STOP_OK;
}

/*-----------------------------------------------------------
 * 进入 STOP：前提防御再校验 -> 六步进入序列 -> entered=1
 * WFI 阻塞在 io->dsb_isb_wfi 内；返回即已唤醒（退出动作由
 * ExtiStop_Exit 显式触发）。
 *----------------------------------------------------------*/
ExtiStop_Status_t ExtiStop_Enter(ExtiStopBackend_t *bk,
                                 u8 no_tasks_waiting_timeout,
                                 u8 watchdog_requires_service)
{
    if ((bk == NULL) || (bk->io == NULL) || (bk->clk_io == NULL)) {
        return EXTI_STOP_ERR_PARAM;
    }

    /* 状态机违例：未 Exit 的重复 Enter（零新 IO 副作用） */
    if (bk->entered != 0u) {
        bk->status = EXTI_STOP_ERR_STATE;
        return EXTI_STOP_ERR_STATE;
    }

    /* 决策层前提的防御性再校验（设计 §6.4 参数面）：
     *   只接受 eNoTasksWaitingTimeout 且 IWDG 无需周期服务；
     *   任一不满足立即拒绝且零 IO 副作用（test s1）。 */
    if ((no_tasks_waiting_timeout == 0u) || (watchdog_requires_service != 0u)) {
        bk->status = EXTI_STOP_ERR_PRECONDITION;
        return EXTI_STOP_ERR_PRECONDITION;
    }

    /* 进入序列（设计 §6.6 EXTI-only 进入，test s2 逐位锁定） */
    bk->io->systick_stop(bk->io->self);           /* 1. 停 SysTick（tick 冻结起点） */
    bk->io->peripherals_quiesce(bk->io->self);    /* 2. 关 ADC/非唤醒 DMA/USART/LCD 等 */
    bk->io->key_exti_wake_config(bk->io->self);   /* 3. 按键 EXTI（PB1 上升沿）唤醒源 */
    bk->io->clear_key_pending(bk->io->self);      /* 4. 清睡前旧 pending（sleep race） */
    bk->io->pwr_stop_mode_enter(bk->io->self);    /* 5. 清 CWUF，PDDS=0 LPDS=1 */
    bk->io->dsb_isb_wfi(bk->io->self);            /* 6. DSB/ISB 屏障后 WFI（阻塞至此） */

    bk->entered = 1u;
    bk->status  = EXTI_STOP_OK;
    return EXTI_STOP_OK;
}

/*-----------------------------------------------------------
 * 退出处理：读 pending 判定唤醒源 -> 公共时钟恢复 ->
 * 恢复外设 -> 恢复 SysTick。一次性语义：无论成败 entered 都清除。
 *----------------------------------------------------------*/
ExtiStop_Status_t ExtiStop_Exit(ExtiStopBackend_t *bk)
{
    u32 pending;
    ClkResume_Status_t clk_st;

    if (bk == NULL) {
        return EXTI_STOP_ERR_PARAM;
    }
    if ((bk->entered == 0u) || (bk->io == NULL) || (bk->clk_io == NULL)) {
        bk->status = EXTI_STOP_ERR_STATE;
        return EXTI_STOP_ERR_STATE;
    }

    /* 1. 读取 EXTI pending 并判定唤醒源（退出顺序 1）。
     *    判定先于一切恢复动作记录——时钟恢复失败路径也保留结果
     *（test s5：失败后 GetWakeSource 仍返回 KEY）。 */
    pending = bk->io->read_exti_pending(bk->io->self);
    if ((pending & bk->key_line_mask) != 0u) {
        bk->wake_source = EXTI_STOP_WAKE_KEY;     /* key 位命中优先归类 */
    } else if (pending != 0u) {
        bk->wake_source = EXTI_STOP_WAKE_OTHER;
    } else {
        bk->wake_source = EXTI_STOP_WAKE_NONE;    /* 调试器等非 EXTI 唤醒 */
    }

    /* 2. 72 MHz 公共时钟恢复（HSE/PLL/SWS 有界超时，退出顺序 2/3） */
    clk_st = SystemClock_RestoreAfterStop(bk->clk_io);
    if (clk_st != CLK_RESUME_OK) {
        /* 失败传播：外设与 SysTick 保持关闭（HSI 8 MHz 下不得恢复
         * 按 72MHz 配置的 PWM/UART/RTOS tick，设计 §6.6）；
         * fail-safe reset 是调用方（Task 15 wrapper）职责。
         * Exit 一次性：entered 清除（test s5 二次 Exit -> ERR_STATE）。 */
        bk->entered = 0u;
        bk->status  = EXTI_STOP_ERR_CLOCK_RESUME;
        return EXTI_STOP_ERR_CLOCK_RESUME;
    }

    /* 3. 恢复必要外设 -> 4. 恢复 SysTick（退出顺序 5） */
    bk->io->peripherals_resume(bk->io->self);
    bk->io->systick_resume(bk->io->self);

    /* tick 冻结不补偿：不调用 vTaskStepTick、不读 RTC（设计 §6.3/§6.6，
     * host mock 以链接集无 FreeRTOS 源 + 桩计数==0 双证明）。 */

    bk->entered = 0u;
    bk->status  = EXTI_STOP_OK;
    return EXTI_STOP_OK;
}

/*-----------------------------------------------------------
 * 最近一次 Exit 的唤醒源查询
 *----------------------------------------------------------*/
ExtiStop_WakeSource_t ExtiStop_GetWakeSource(const ExtiStopBackend_t *bk)
{
    if (bk == NULL) {
        return EXTI_STOP_WAKE_NONE;
    }
    return bk->wake_source;
}
