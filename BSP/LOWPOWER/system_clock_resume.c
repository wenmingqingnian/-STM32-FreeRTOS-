/*
 * STOP 唤醒后的 72 MHz 公共时钟恢复实现
 *
 * 纯 IO 注入序列实现：无任何直接寄存器访问、无全局状态、无隐藏依赖
 *（"不访问硬件"由链接集证明——host mock 只注入脚本化 IO）。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#include "system_clock_resume.h"

/*-----------------------------------------------------------
 * 有界等待：等待谓词最多 CLK_RESUME_TIMEOUT_LOOPS 轮，每轮之间由
 * IO wait_poll 提供退避（目标侧为空——SysTick 尚未恢复，无毫秒时基）
 *----------------------------------------------------------*/
static ClkResume_Status_t wait_flag(const SystemClockResume_Io_t *io,
                                    u8 (*ready)(void *self))
{
    u32 loop;

    for (loop = 0u; loop < (u32)CLK_RESUME_TIMEOUT_LOOPS; loop++) {
        if (ready(io->self) != 0u) {
            return CLK_RESUME_OK;               /* loop < 上限：有界内就绪 */
        }
        io->wait_poll(io->self);
    }
    return CLK_RESUME_ERR_HSE_TIMEOUT;          /* 语义由调用方覆盖 */
}

ClkResume_Status_t SystemClock_RestoreAfterStop(const SystemClockResume_Io_t *io)
{
    ClkResume_Status_t st;

    if (io == NULL) {
        return CLK_RESUME_ERR_PARAM;
    }

    /* 1. HSE：开启 + 有界等待（唤醒后 SYSCLK=HSI，PLL 已被硬件关闭） */
    io->hse_on(io->self);
    st = wait_flag(io, io->hse_ready);
    if (st != CLK_RESUME_OK) {
        return CLK_RESUME_ERR_HSE_TIMEOUT;      /* 后续配置一概不执行 */
    }

    /* 2. Flash latency 先行：HSI 8MHz 下设 72MHz 档 2WS 是安全方向
     *（降频时升高延迟无害；反向才有取指风险） */
    io->flash_latency_72m_config(io->self);

    /* 3. 总线分频恢复（AHB/1 APB2/1 APB1/2，同 SetSysClockTo72） */
    io->bus_prescalers_72m_config(io->self);

    /* 4. PLL = HSE×9：重新确认配置并使能 + 有界等待 */
    io->pll_hse_x9_enable(io->self);
    st = wait_flag(io, io->pll_ready);
    if (st != CLK_RESUME_OK) {
        return CLK_RESUME_ERR_PLL_TIMEOUT;      /* 选源/更新不执行 */
    }

    /* 5. SYSCLK 切回 PLL + 有界确认 SWS */
    io->select_pll_sysclk(io->self);
    st = wait_flag(io, io->sysclk_is_pll);
    if (st != CLK_RESUME_OK) {
        return CLK_RESUME_ERR_SWITCH_TIMEOUT;   /* 更新不执行 */
    }

    /* 6. 全局时钟变量刷新（设计 §6.6 退出顺序 3） */
    io->system_core_clock_update(io->self);
    return CLK_RESUME_OK;
}
