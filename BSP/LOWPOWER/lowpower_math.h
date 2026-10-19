/*
 * STOP 低功耗时间换算
 *
 * 职责：BOARD_C8_LSE 板型下 FreeRTOS tick（1000 Hz）与 RTC 计数
 *（1024 Hz，LSE 32768Hz / prescaler 31）之间的纯函数换算：
 *
 *   1. 正向（睡眠预算）：fticks -> rticks，向上取整
 *      ceil(fticks * 1024 / 1000)。向上取整保证闹钟不早于需求唤醒，
 *      防止补 tick 不足越过解阻塞点；
 *   2. 反向（唤醒补偿）：delta_rtc -> 已流逝 fticks，余数跨睡眠累计：
 *        numerator = delta_rtc * 1000 + remainder_in
 *        elapsed    = numerator / 1024
 *        remainder  = numerator % 1024
 *      余数累计保证长序列总误差 < 1 tick（无漂移）；
 *   3. Alarm 预算：min(expected_idle - wake_guard, MAX_STOP_MS)；
 *   4. elapsed clamp：步进不得越过下一个解阻塞点。
 *
 * 纯函数契约：不读 RTC/SysTick 寄存器；RTC 计数差值由调用方取得
 *（32bit 回绕安全无符号减法封装为 LpMath_RtcDelta）。
 *
 * 调用前提（由 lowpower_policy 决策保证）：仅 LP_STOP_RTC 路径使用
 * 本换算；BOARD_C8_NO_LSE / PENDING 板型不得调用。
 *
 * 64 bit 中间量：fticks=portMAX_DELAY 时正向乘积约 4.4e12、
 * delta_rtc=0xFFFFFFFF 时反向乘积约 4.3e12，均超 32 bit 表达范围，
 * 全部中间量使用 u64（uint64_t）。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#ifndef __LOWPOWER_MATH_H__
#define __LOWPOWER_MATH_H__

#include <stdint.h>
#include <stddef.h>            /* NULL - ARMCC 与 host gcc 通用 */
#include "sys.h"               /* u8/u32 */
#include "lowpower_policy.h"   /* LP_RTC_HZ/LP_RTOS_HZ/LP_MAX_STOP_MS/LP_WAKE_GUARD_TICKS */

/* 项目类型惯例补缺：stm32f10x.h 只到 u32，64 bit 中间量在此单一引入 */
#ifndef LP_HAVE_U64_TYPEDEF
#define LP_HAVE_U64_TYPEDEF
typedef uint64_t u64;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * 正向换算：FreeRTOS ticks -> RTC ticks（向上取整，64 bit 中间量）
 *
 *   rticks = ceil(fticks * LP_RTC_HZ / LP_RTOS_HZ)
 *
 * 超出 32 bit 计数域（fticks >= 4194304000，含 portMAX_DELAY）时
 * 饱和返回 0xFFFFFFFF：中间量 u64 保证不回绕出错误小值。
 *----------------------------------------------------------*/
u32 LpMath_FticksToRticksCeil(u32 fticks);

/*-----------------------------------------------------------
 * 反向换算：RTC 差值 -> 已流逝 FreeRTOS ticks（余数累计）
 *
 *   numerator = delta_rtc * LP_RTOS_HZ + remainder_in
 *   elapsed   = numerator / LP_RTC_HZ
 *  *remainder_out = numerator % LP_RTC_HZ（可 NULL：仅省略写出）
 *
 * remainder_in 必须来自上一次本函数的输出（初始 0）；
 * delta_rtc = LpMath_RtcDelta(end, start) 的无符号差值。
 * 中间量 u64：delta=0xFFFFFFFF 时不溢出。
 *----------------------------------------------------------*/
u32 LpMath_RticksToElapsed(u32 delta_rtc, u32 remainder_in,
                            u32 *remainder_out);

/*-----------------------------------------------------------
 * elapsed clamp：vTaskStepTick 步进不得越过下一个解阻塞点
 *
 * 返回 min(elapsed_ticks, expected_idle_ticks)；
 * 越界时 *clamped 置 1（可 NULL），调用方累计 tick_clamp_count，
 * 不静默掩盖时钟恢复过慢（设计 §6.5）。
 * expected_idle_ticks = portMAX_DELAY（无任务等待超时）时不 clamp。
 *----------------------------------------------------------*/
u32 LpMath_ClampElapsed(u32 elapsed_ticks, u32 expected_idle_ticks,
                         u8 *clamped);

/*-----------------------------------------------------------
 * RTC 计数差值：end - start 无符号减法
 *
 * RTC 计数 32 bit @1024Hz 约 48.5 天回绕；单次 STOP 远小于半个
 * 回绕周期，无符号减法天然正确（含低 16 bit 回绕场景——高低字
 * 撕裂防护属 Task 12 rtc_timebase 稳定快照职责，不在本函数）。
 *----------------------------------------------------------*/
u32 LpMath_RtcDelta(u32 end_rtc, u32 start_rtc);

/*-----------------------------------------------------------
 * Alarm 预算（RTC ticks）：
 *
 *   budget_fticks = expected_idle_ticks - LP_WAKE_GUARD_TICKS
 *（不足 wake_guard 时返回 0：不可用）
 *   budget        = min(budget_fticks, LP_MAX_STOP_MS * LP_RTOS_HZ / 1000)
 *   返回 ceil(budget * LP_RTC_HZ / LP_RTOS_HZ)
 *
 * portMAX_DELAY 输入经 64 bit 路径安全收敛到 MAX_STOP cap。
 * 返回 0 表示无可用预算，调用方不得进入 RTC 定时 STOP。
 *----------------------------------------------------------*/
u32 LpMath_AlarmBudgetRticks(u32 expected_idle_ticks);

#ifdef __cplusplus
}
#endif

#endif /* __LOWPOWER_MATH_H__ */
