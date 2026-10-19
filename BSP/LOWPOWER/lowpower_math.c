/*
 * STOP 低功耗时间换算实现
 *
 * 实现说明：
 *   - 纯函数：无全局状态、无寄存器访问；全部中间量 u64：
 *     正向 0xFFFFFFFF*1024 = 4398046511040（~4.4e12）、
 *     反向 0xFFFFFFFF*1000 = 4294967295000（~4.3e12）均超 32 bit；
 *   - 正向向上取整（分子+分母-1）/分母：保证闹钟不早于需求唤醒，
 *     防止补 tick 不足越过解阻塞点（设计 §6.5）；
 *   - 反向余数累计数学性质（无漂移的依据）：逐次
 *       num_i = delta_i*1000 + r_{i-1}，e_i = num_i/1024，r_i = num_i%1024
 *     归纳可得 sum(e_i) = floor(1000*sum(delta_i)/1024)，
 *     即任意长序列总误差 < 1 tick（测试 s4 验证）；
 *   - Alarm 预算在 fticks 域扣除 wake_guard 并 cap 到 MAX_STOP，
 *     再经向上取整换算到 rticks；portMAX_DELAY 输入经 u64 路径
 *     安全收敛，不依赖调用方预清洗。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#include "lowpower_math.h"

u32 LpMath_FticksToRticksCeil(u32 fticks)
{
    u64 num;
    u64 rticks;

    /* ceil(fticks * RTC_HZ / RTOS_HZ)：分子加 分母-1 再整除 */
    num = (u64)fticks * (u64)LP_RTC_HZ;
    num += (u64)LP_RTOS_HZ - 1u;
    rticks = num / (u64)LP_RTOS_HZ;

    /* 超出 32 bit 计数域（含 portMAX_DELAY 输入）：饱和而非回绕。
     * 真实调用路径（Alarm 预算）已 cap 到 MAX_STOP，此分支为纯函数
     * 对非法/极端输入域的防御。 */
    if (rticks > 0xFFFFFFFFuL)
    {
        return 0xFFFFFFFFuL;
    }
    return (u32)rticks;
}

u32 LpMath_RticksToElapsed(u32 delta_rtc, u32 remainder_in,
                            u32 *remainder_out)
{
    u64 num;
    u32 elapsed;

    /* numerator = delta_rtc * RTOS_HZ + remainder_in（64 bit 中间量）*/
    num = (u64)delta_rtc * (u64)LP_RTOS_HZ + (u64)remainder_in;

    elapsed = (u32)(num / (u64)LP_RTC_HZ);

    if (remainder_out != NULL)
    {
        *remainder_out = (u32)(num % (u64)LP_RTC_HZ);
    }
    return elapsed;
}

u32 LpMath_ClampElapsed(u32 elapsed_ticks, u32 expected_idle_ticks,
                         u8 *clamped)
{
    if (clamped != NULL)
    {
        *clamped = 0u;
    }

    if (elapsed_ticks > expected_idle_ticks)
    {
        /* portMAX_DELAY 作为 expected 时不可能大于 elapsed
         *（0xFFFFFFFF 是最大值），天然不触发 clamp */
        if (clamped != NULL)
        {
            *clamped = 1u;   /* 调用方累计 tick_clamp_count，不静默掩盖 */
        }
        return expected_idle_ticks;
    }
    return elapsed_ticks;
}

u32 LpMath_RtcDelta(u32 end_rtc, u32 start_rtc)
{
    /* 无符号减法：32 bit 计数回绕天然正确（前提：单次睡眠远小于
     * 半个回绕周期 ~24 天；高低字撕裂防护属 Task 12 稳定快照）*/
    return end_rtc - start_rtc;
}

u32 LpMath_AlarmBudgetRticks(u32 expected_idle_ticks)
{
    u32 budget_fticks;
    u32 max_stop_fticks;

    /* 扣除唤醒保护：不足 guard -> 预算 0（不可进入 RTC STOP）*/
    if (expected_idle_ticks <= LP_WAKE_GUARD_TICKS)
    {
        return 0u;
    }
    budget_fticks = expected_idle_ticks - LP_WAKE_GUARD_TICKS;

    /* cap 到 MAX_STOP_MS（fticks = ms * RTOS_HZ / 1000）*/
    max_stop_fticks = (u32)((u32)LP_MAX_STOP_MS * LP_RTOS_HZ / 1000u);
    if (budget_fticks > max_stop_fticks)
    {
        budget_fticks = max_stop_fticks;
    }

    /* 向上取整换算到 RTC ticks */
    return LpMath_FticksToRticksCeil(budget_fticks);
}
