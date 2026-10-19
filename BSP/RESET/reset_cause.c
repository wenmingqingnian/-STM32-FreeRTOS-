/*
 * 复位原因模块实现
 *
 * 实现说明：
 *   - ResetCause_Decode() 为纯函数，不触碰任何硬件寄存器；
 *   - ResetCause_Init() 读一次 RCC->CSR，判定后写 RMVF 清标志，
 *     并带读回确认（RMVF 是写 1 生效位，标志清除需数个 HCLK 周期）；
 *   - 重复调用 Init 由 g_initialized 拒绝（防止诊断代码误清后续复位原因）。
 *
 */
#include "reset_cause.h"
#include "stm32f10x_rcc.h"
#include <stdio.h>

/*-----------------------------------------------------------
 * 模块状态
 *----------------------------------------------------------*/
static ResetCause_t g_resetCause = RESET_CAUSE_UNKNOWN;  /* RAM 启动记录 */
static u8 g_initialized = 0;                             /* Init 只执行一次 */

/*-----------------------------------------------------------
 * 纯解码函数：只依赖输入 csr，输出复位原因
 * 判定优先级（多标志并发时的取舍依据）：
 *   IWDG > WWDG > 软件复位 > 低功耗 > 上电 > 引脚 > UNKNOWN
 * 说明：上电复位同时置位 PORRSTF+PINRSTF，引脚复位只置 PINRSTF，
 *       因此 POR 判定必须先于 PIN，否则上电被误判为引脚复位；
 *       看门狗复位同样伴随 PINRSTF（内部复位脉冲走 NRST 线），
 *       看门狗判定优先级必须最高，否则被 PINRSTF 掩盖。
 *----------------------------------------------------------*/
ResetCause_t ResetCause_Decode(u32 csr)
{
    if (csr & RESET_CAUSE_CSR_IWDGRSTF)
    {
        return RESET_CAUSE_IWDG_RESET;
    }
    if (csr & RESET_CAUSE_CSR_WWDGRSTF)
    {
        return RESET_CAUSE_WWDG_RESET;
    }
    if (csr & RESET_CAUSE_CSR_SFTRSTF)
    {
        return RESET_CAUSE_SOFTWARE_RESET;
    }
    if (csr & RESET_CAUSE_CSR_LPWRRSTF)
    {
        return RESET_CAUSE_LOW_POWER_RESET;
    }
    if (csr & RESET_CAUSE_CSR_PORRSTF)
    {
        return RESET_CAUSE_POWER_ON;
    }
    if (csr & RESET_CAUSE_CSR_PINRSTF)
    {
        return RESET_CAUSE_PIN_RESET;
    }
    return RESET_CAUSE_UNKNOWN;
}

/*-----------------------------------------------------------
 * 初始化：读取 CSR → 判定 → 保存 RAM → 清除标志
 * 调用时机约束：main() 最早处（外设初始化与清标志之前）
 *----------------------------------------------------------*/
void ResetCause_Init(void)
{
    u32 csr;
    u32 timeout;

    if (g_initialized)
    {
        return;     /* 重复调用拒绝：防止误清后续复位原因 */
    }
    g_initialized = 1;

    /* 1. 快照 CSR（复位标志分布在 bit26..31） */
    csr = RCC->CSR;

    /* 2. 纯函数判定并保存 RAM 启动记录 */
    g_resetCause = ResetCause_Decode(csr);

    /* 3. 写 RMVF 清除全部复位标志，读回确认（带超时防死等） */
    RCC->CSR |= RCC_CSR_RMVF;
    for (timeout = 0u; timeout < 10000u; timeout++)
    {
        if ((RCC->CSR & (RESET_CAUSE_CSR_LPWRRSTF | RESET_CAUSE_CSR_WWDGRSTF |
                         RESET_CAUSE_CSR_IWDGRSTF | RESET_CAUSE_CSR_SFTRSTF  |
                         RESET_CAUSE_CSR_PORRSTF  | RESET_CAUSE_CSR_PINRSTF)) == 0u)
        {
            break;
        }
    }
}

/*-----------------------------------------------------------
 * 取已保存的复位原因
 *----------------------------------------------------------*/
ResetCause_t ResetCause_Get(void)
{
    return g_resetCause;
}

/*-----------------------------------------------------------
 * 复位原因转字符串
 *----------------------------------------------------------*/
const char *ResetCause_GetName(ResetCause_t cause)
{
    static const char * const names[RESET_CAUSE_MAX] = {
        "POWER-ON (POR/PDR)",
        "PIN RESET (NRST)",
        "SOFTWARE RESET",
        "IWDG RESET",
        "WWDG RESET",
        "LOW-POWER RESET",
        "UNKNOWN"
    };

    if ((u32)cause < (u32)RESET_CAUSE_MAX)
    {
        return names[(u32)cause];
    }
    return "UNKNOWN";
}

/*-----------------------------------------------------------
 * 启动复位报告（uart 就绪后调用）
 *----------------------------------------------------------*/
void ResetCause_Report(void)
{
    printf("---- Reset cause: %s ----\r\n", ResetCause_GetName(g_resetCause));
}
