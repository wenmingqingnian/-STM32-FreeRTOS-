/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Host-side mock test for the STOP tick math (FreeRTOS 1000 Hz <-> RTC
 * 1024 Hz conversion).
 * Covers:
 *
 *   1. BOARD_C8_LSE forward conversion is ceiling:
 *      ceil(fticks * 1024 / 1000) -- never truncates (an early alarm
 *      would under-compensate elapsed ticks)
 *   2. Reverse conversion carries the remainder across sleeps; a long
 *      sequence of conversions accumulates less than 1 tick of total
 *      error (no drift)
 *   3. 64-bit intermediates: fticks = portMAX_DELAY (forward product
 *      ~4.4e12) and delta_rtc = 0xFFFFFFFF (reverse product ~4.3e12)
 *      both exceed 32-bit range and must not wrap
 *   4. RTC counter delta is unsigned subtraction: 32-bit wrap and
 *      low-16-bit wrap are both handled (single sleeps are far shorter
 *      than half the ~48.5-day wrap period)
 *   5. portMAX_DELAY handling: alarm budget saturates at the 1 s cap;
 *      elapsed clamp does not fire when no task waits for a timeout
 *   6. 1 s cap: alarm budget = min(idle - wake_guard, MAX_STOP) in
 *      fticks, then ceiling-converted to rticks; idle <= wake_guard
 *      yields 0 (RTC stop not allowed)
 *
 * BOARD_C8_NO_LSE "no RTC conversion" is covered on the policy side
 * (test_lowpower_policy.c s5: NO_LSE/PENDING never decide LP_STOP_RTC,
 * so the wrapper never calls this module) plus this module defining no
 * LSI-based conversion path at all (the link set itself is the proof).
 *
 * Production sources under test are compiled UNMODIFIED from the project
 * tree (BSP\LOWPOWER\lowpower_math.c); sys.h comes from stub/.
 */
#include <stdio.h>
#include "lowpower_math.h"

static int g_failed = 0;
static int g_total = 0;

#define CHECK_DESC(desc, cond) do {                          \
    g_total++;                                                \
    if (cond) { printf("  ok %s\n", desc); }                  \
    else      { g_failed++; printf("  FAIL %s\n", desc); }    \
} while (0)

#define CHECK_EQ_DESC(desc, got, want) do {                  \
    g_total++;                                                \
    if ((unsigned long)(got) == (unsigned long)(want))        \
        printf("  ok %s\n", desc);                           \
    else { g_failed++;                                        \
        printf("  FAIL %s: got %lu, want %lu\n",              \
               desc, (unsigned long)(got),                   \
               (unsigned long)(want)); }                      \
} while (0)

/* portMAX_DELAY 语义（host 侧无 FreeRTOS 头，字面常量）*/
#define PORT_MAX_DELAY_32   0xFFFFFFFFuL

/*--------------------------- s1: 策略常量 ---------------------------*/

static void s1_policy_constants(void)
{
    printf("s1: policy constants (strategy values, not measured)\n");

    /* LSE 路径策略值——换算正确性依赖
     * 这组常量，锁定防止误改后换算静默漂移 */
    CHECK_EQ_DESC("LP_RTC_HZ == 1024 (LSE 32768 / (31+1))",
                  LP_RTC_HZ, 1024u);
    CHECK_EQ_DESC("LP_RTOS_HZ == 1000 (configTICK_RATE_HZ)",
                  LP_RTOS_HZ, 1000u);
    CHECK_EQ_DESC("LP_MAX_STOP_MS == 1000 (safe cap, not optimal)",
                  LP_MAX_STOP_MS, 1000u);
    CHECK_EQ_DESC("LP_WAKE_GUARD_TICKS == 2 (conservative guard)",
                  LP_WAKE_GUARD_TICKS, 2u);
}

/*--------------------------- s2: 正向换算（向上取整） ---------------------------*/

static void s2_forward_ceil(void)
{
    printf("s2: forward conversion ceil(fticks*1024/1000)\n");

    CHECK_EQ_DESC("fticks=0 -> 0", LpMath_FticksToRticksCeil(0u), 0u);
    CHECK_EQ_DESC("fticks=1 -> 2 (ceil 1.024)", LpMath_FticksToRticksCeil(1u), 2u);
    CHECK_EQ_DESC("fticks=2 -> 3 (ceil 2.048)", LpMath_FticksToRticksCeil(2u), 3u);
    CHECK_EQ_DESC("fticks=3 -> 4 (ceil 3.072)", LpMath_FticksToRticksCeil(3u), 4u);
    CHECK_EQ_DESC("fticks=500 -> 512 (exact)", LpMath_FticksToRticksCeil(500u), 512u);
    CHECK_EQ_DESC("fticks=977 -> 1001 (ceil 1000.448)",
                  LpMath_FticksToRticksCeil(977u), 1001u);
    CHECK_EQ_DESC("fticks=1000 -> 1024 (exact 1 s)",
                  LpMath_FticksToRticksCeil(1000u), 1024u);

    /* 64 bit 中间量 + 域饱和：0xFFFFFFFF*1024=4398046511040 超 32 bit，
     * 数学值 ceil=4398046512 超出 32 bit 计数域 -> 饱和返回 0xFFFFFFFF
     *（32 bit 截断路径会回绕出错误小值 103079216）*/
    CHECK_EQ_DESC("fticks=portMAX_DELAY -> saturated 0xFFFFFFFF",
                  LpMath_FticksToRticksCeil(PORT_MAX_DELAY_32), 0xFFFFFFFFuL);

    /* 饱和边界：4194303999 是域内最大输入（ceil 恰为 0xFFFFFFFF）*/
    CHECK_EQ_DESC("fticks=4194303999 -> 0xFFFFFFFF (in-domain max)",
                  LpMath_FticksToRticksCeil(4194303999uL), 0xFFFFFFFFuL);
    CHECK_EQ_DESC("fticks=4194304000 -> saturated 0xFFFFFFFF",
                  LpMath_FticksToRticksCeil(4194304000uL), 0xFFFFFFFFuL);
}

/*--------------------------- s3: 反向换算（余数累计） ---------------------------*/

static void s3_reverse_remainder(void)
{
    u32 r = 0u;

    printf("s3: reverse conversion with remainder carry\n");

    CHECK_EQ_DESC("delta=1024 r=0 -> 1000 ticks r=0",
                  LpMath_RticksToElapsed(1024u, 0u, &r), 1000u);
    CHECK_EQ_DESC("  remainder == 0", r, 0u);

    CHECK_EQ_DESC("delta=512 -> 500 ticks", LpMath_RticksToElapsed(512u, 0u, &r), 500u);

    CHECK_EQ_DESC("delta=1 r=0 -> 0 ticks", LpMath_RticksToElapsed(1u, 0u, &r), 0u);
    CHECK_EQ_DESC("  remainder == 1000", r, 1000u);

    CHECK_EQ_DESC("delta=1 r=1000 -> 1 tick", LpMath_RticksToElapsed(1u, 1000u, &r), 1u);
    CHECK_EQ_DESC("  remainder == 976", r, 976u);

    CHECK_EQ_DESC("delta=3 r=1000 -> 3 ticks", LpMath_RticksToElapsed(3u, 1000u, &r), 3u);
    CHECK_EQ_DESC("  remainder == 928", r, 928u);

    CHECK_EQ_DESC("delta=0 r=500 -> 0 ticks",
                  LpMath_RticksToElapsed(0u, 500u, &r), 0u);
    CHECK_EQ_DESC("  remainder == 500 (unchanged)", r, 500u);

    /* 64 bit 中间量：0xFFFFFFFF*1000=4294967295000 超 32 bit，
     * 期望 elapsed=4194303999 remainder=24 */
    CHECK_EQ_DESC("delta=0xFFFFFFFF -> 4194303999 (64-bit proof)",
                  LpMath_RticksToElapsed(PORT_MAX_DELAY_32, 0u, &r), 4194303999uL);
    CHECK_EQ_DESC("  remainder == 24", r, 24u);

    /* remainder_out 可 NULL（仅省略写出，结果不受影响）*/
    CHECK_EQ_DESC("remainder_out=NULL tolerated",
                  LpMath_RticksToElapsed(1024u, 0u, NULL), 1000u);
}

/*--------------------------- s4: 长序列累计误差 ---------------------------*/

static void s4_long_sequence_drift(void)
{
    u32 r = 0u;
    u32 elapsed_total = 0u;
    u32 i;

    printf("s4: long-sequence drift < 1 tick (remainder carry)\n");

    /* 1024 次单 rtick 睡眠：真实 1024*1000/1024 = 1000 fticks 整；
     * 余数累计数学上精确（sum(e_i) = floor(S/1024)），零漂移 */
    for (i = 0u; i < 1024u; i++)
    {
        elapsed_total += LpMath_RticksToElapsed(1u, r, &r);
    }
    CHECK_EQ_DESC("1024 sleeps of 1 rtick -> exactly 1000 fticks",
                  elapsed_total, 1000u);
    CHECK_EQ_DESC("  final remainder == 0 (no drift)", r, 0u);

    /* 1000 次 3 rtick 睡眠：真实 3000*1000/1024 = 2929.6875；
     * 余数累计总 elapsed = floor(3000000/1024) = 2929，误差 0.6875 < 1 */
    elapsed_total = 0u;
    r = 0u;
    for (i = 0u; i < 1000u; i++)
    {
        elapsed_total += LpMath_RticksToElapsed(3u, r, &r);
    }
    CHECK_EQ_DESC("1000 sleeps of 3 rticks -> 2929 fticks (err 0.6875 < 1)",
                  elapsed_total, 2929u);

    /* 混合序列：交替 1 / 1024 rtick 各 500 次，总 delta = 512500 rticks
     * 总 elapsed = floor(512500*1000/1024) = 500488（精确）*/
    elapsed_total = 0u;
    r = 0u;
    for (i = 0u; i < 500u; i++)
    {
        elapsed_total += LpMath_RticksToElapsed(1u, r, &r);
        elapsed_total += LpMath_RticksToElapsed(1024u, r, &r);
    }
    CHECK_EQ_DESC("mixed 1/1024 rtick x500 -> 500488 fticks (exact)",
                  elapsed_total, 500488u);
}

/*--------------------------- s5: RTC 差值 wrap ---------------------------*/

static void s5_rtc_delta_wrap(void)
{
    printf("s5: RTC delta unsigned subtraction (wrap safe)\n");

    /* 正常无 wrap */
    CHECK_EQ_DESC("delta(1024, 0) == 1024", LpMath_RtcDelta(1024u, 0u), 1024u);

    /* 32 bit 回绕：end < start 的"跨回绕"读数对 */
    CHECK_EQ_DESC("32-bit wrap: delta(0x10, 0xFFFFFFF0) == 0x20",
                  LpMath_RtcDelta(0x10u, 0xFFFFFFF0u), 0x20u);

    /* 低 16 bit 回绕：32 bit 计数跨 0x10000 边界增长（高字 +1、低字
     * 回卷的场景——非 mod-65536 语义，RTC 计数本身 32 bit）*/
    CHECK_EQ_DESC("low-16-bit wrap: delta(0x10010, 0xFFF0) == 0x20",
                  LpMath_RtcDelta(0x00010010u, 0x0000FFF0u), 0x20u);

    /* 48.5 天回绕中点附近 */
    CHECK_EQ_DESC("half-period boundary: delta(0x80001000, 0x7FFFF000) == 0x2000",
                  LpMath_RtcDelta(0x80001000u, 0x7FFFF000u), 0x2000u);

    /* 相等 -> 0 */
    CHECK_EQ_DESC("delta(x, x) == 0", LpMath_RtcDelta(1234u, 1234u), 0u);

    /* 反向输入（end 落后 start）：无符号减法语义，大差值；
     * 调用方契约保证 end 按"后来"读取（稳定快照，Task 12）*/
    CHECK_EQ_DESC("reversed input: delta(0x10, 0x20) == 0xFFFFFFF0",
                  LpMath_RtcDelta(0x10u, 0x20u), 0xFFFFFFF0u);
}

/*--------------------------- s6: elapsed clamp ---------------------------*/

static void s6_clamp(void)
{
    u8 clamped = 0u;

    printf("s6: elapsed clamp to next unblock point\n");

    CHECK_EQ_DESC("elapsed==expected -> unchanged",
                  LpMath_ClampElapsed(100u, 100u, &clamped), 100u);
    CHECK_DESC("  not clamped", clamped == 0u);

    CHECK_EQ_DESC("elapsed=expected+1 -> clamped to expected",
                  LpMath_ClampElapsed(101u, 100u, &clamped), 100u);
    CHECK_DESC("  clamped flag set", clamped == 1u);

    CHECK_EQ_DESC("elapsed=0xFFFFFFFF vs expected=100 -> 100",
                  LpMath_ClampElapsed(0xFFFFFFFFu, 100u, &clamped), 100u);
    CHECK_DESC("  clamped flag set", clamped == 1u);

    /* expected = portMAX_DELAY（无任务等待超时）不 clamp */
    CHECK_EQ_DESC("expected=portMAX_DELAY -> elapsed unchanged",
                  LpMath_ClampElapsed(100u, PORT_MAX_DELAY_32, &clamped), 100u);
    CHECK_DESC("  not clamped", clamped == 0u);

    CHECK_EQ_DESC("elapsed=0 -> 0", LpMath_ClampElapsed(0u, 50u, &clamped), 0u);
    CHECK_DESC("  not clamped", clamped == 0u);

    /* clamped 指针可 NULL */
    CHECK_EQ_DESC("clamped=NULL tolerated",
                  LpMath_ClampElapsed(200u, 100u, NULL), 100u);
}

/*--------------------------- s7: Alarm 预算 ---------------------------*/

static void s7_alarm_budget(void)
{
    printf("s7: alarm budget min(idle - guard, MAX_STOP) ceiling to rticks\n");

    /* idle <= wake_guard：预算 0，不可进入 RTC STOP */
    CHECK_EQ_DESC("idle=0 -> budget 0", LpMath_AlarmBudgetRticks(0u), 0u);
    CHECK_EQ_DESC("idle=1 -> budget 0", LpMath_AlarmBudgetRticks(1u), 0u);
    CHECK_EQ_DESC("idle=2 (== guard) -> budget 0",
                  LpMath_AlarmBudgetRticks(LP_WAKE_GUARD_TICKS), 0u);

    /* idle = guard+1：1 ftick -> ceil(1.024) = 2 rticks */
    CHECK_EQ_DESC("idle=3 (guard+1) -> 2 rticks",
                  LpMath_AlarmBudgetRticks(3u), 2u);

    /* guard 扣除后的中间档：idle=502 -> 500 fticks -> 512 rticks */
    CHECK_EQ_DESC("idle=502 -> 512 rticks",
                  LpMath_AlarmBudgetRticks(502u), 512u);

    /* 1 s cap 精确边界：idle=1002 -> 1000 fticks -> 1024 rticks */
    CHECK_EQ_DESC("idle=1002 (guard+MAX_STOP) -> 1024 rticks (cap edge)",
                  LpMath_AlarmBudgetRticks(1002u), 1024u);

    /* cap 生效：超过 1 s 预算收敛到 1024 rticks */
    CHECK_EQ_DESC("idle=1003 -> 1024 rticks (capped)",
                  LpMath_AlarmBudgetRticks(1003u), 1024u);
    CHECK_EQ_DESC("idle=100000 -> 1024 rticks (capped)",
                  LpMath_AlarmBudgetRticks(100000u), 1024u);

    /* portMAX_DELAY（无任务等待超时但走 RTC 路径的极端输入）安全收敛 */
    CHECK_EQ_DESC("idle=portMAX_DELAY -> 1024 rticks (64-bit safe)",
                  LpMath_AlarmBudgetRticks(PORT_MAX_DELAY_32), 1024u);
}

/*--------------------------- main ---------------------------*/

int main(void)
{
    s1_policy_constants();
    s2_forward_ceil();
    s3_reverse_remainder();
    s4_long_sequence_drift();
    s5_rtc_delta_wrap();
    s6_clamp();
    s7_alarm_budget();

    printf("lowpower-math mock: %d checks, %d failed\n",
           g_total, g_failed);
    return (g_failed == 0) ? 0 : 1;
}
