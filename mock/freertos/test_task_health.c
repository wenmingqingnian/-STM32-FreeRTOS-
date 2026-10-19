// SPDX-License-Identifier: GPL-2.0
/*
 * Host-side mock test for pure logic:
 *   1. TaskHealth_Decide()  - heartbeat verdict + missing bitmap
 *   2. TaskHealth_Heartbeat()/TaskHealth_SwapBeats() - set/read-clear semantics
 *      (host is single-threaded; FreeRTOS critical sections degrade to empty
 *       macros here - atomicity on target is a code-review property)
 *   3. ResetCause_Decode()  - CSR flag -> cause, incl. flag-coalescing edge
 *      cases (POR also sets PINRSTF, watchdog resets also set PINRSTF)
 *
 * Production sources under test are compiled UNMODIFIED from the project
 * tree; only the FreeRTOS/sys/RCC headers come from the stub/ tree.
 */
#include <stdio.h>
#include <string.h>
#include "task_health.h"
#include "reset_cause.h"
/* FreeRTOS stub symbols referenced by task_health.c */
void vTaskDelayUntil(TickType_t *last, TickType_t period)
{
    (void)last; (void)period;
}
TickType_t xTaskGetTickCount(void)
{
    return 0;
}
/* Task 14: HealthTask 现在等待待机事件组（循环不被测试执行，
 * 仅提供链接桩；位语义由 standby_policy 纯函数决策表锁定）*/
EventBits_t xEventGroupWaitBits(EventGroupHandle_t grp, EventBits_t bits,
                                BaseType_t clear, BaseType_t all,
                                TickType_t ticks)
{
    (void)grp; (void)bits; (void)clear; (void)all; (void)ticks;
    return 0;
}

static int g_failed = 0;
static int g_total = 0;

#define CHECK_DESC(desc, cond) do {                     \
    g_total++;                                          \
    if (cond) { printf("  ok %s\n", desc); }            \
    else      { g_failed++; printf("  FAIL %s\n", desc); } \
} while (0)

#define CHECK_EQ_DESC(desc, got, want) do {             \
    g_total++;                                          \
    if ((unsigned long)(got) == (unsigned long)(want))  \
        printf("  ok %s\n", desc);                      \
    else { g_failed++;                                  \
        printf("  FAIL %s: got %lu, want %lu\n",        \
               desc, (unsigned long)(got),              \
               (unsigned long)(want)); }                \
} while (0)

/* ---- TaskHealth_Decide ---- */
static void test_decide(void)
{
    u32 missing;

    printf("suite task-health-decide\n");

    CHECK_DESC("all required present -> FEED",
        TaskHealth_Decide(TASK_HEALTH_REQUIRED_MASK,
                          TASK_HEALTH_REQUIRED_MASK, &missing)
            == TASK_HEALTH_FEED);
    CHECK_EQ_DESC("  and missing == 0", missing, 0);

    CHECK_DESC("empty beats -> HOLD",
        TaskHealth_Decide(0, TASK_HEALTH_REQUIRED_MASK, &missing)
            == TASK_HEALTH_HOLD);
    CHECK_EQ_DESC("  and missing == required",
        missing, TASK_HEALTH_REQUIRED_MASK);

    CHECK_EQ_DESC("sensor missing -> HOLD",
        TaskHealth_Decide(TASK_HEALTH_REQUIRED_MASK & ~TASK_HEALTH_BIT_SENSOR,
                          TASK_HEALTH_REQUIRED_MASK, &missing)
            == TASK_HEALTH_HOLD,
        TASK_HEALTH_HOLD);
    CHECK_EQ_DESC("  missing == SENSOR bit", missing,
        TASK_HEALTH_BIT_SENSOR);

    CHECK_EQ_DESC("motor missing -> HOLD",
        TaskHealth_Decide(TASK_HEALTH_REQUIRED_MASK & ~TASK_HEALTH_BIT_MOTOR,
                          TASK_HEALTH_REQUIRED_MASK, NULL),
        TASK_HEALTH_HOLD);

    CHECK_EQ_DESC("state_mgr+speed_calc missing -> HOLD",
        TaskHealth_Decide(TASK_HEALTH_BIT_SENSOR | TASK_HEALTH_BIT_MOTOR,
                          TASK_HEALTH_REQUIRED_MASK, &missing),
        TASK_HEALTH_HOLD);
    CHECK_EQ_DESC("  missing == STATE_MGR|SPEED_CALC",
        missing,
        TASK_HEALTH_BIT_STATE_MGR | TASK_HEALTH_BIT_SPEED_CALC);

    /* extra unknown bits must not affect the verdict */
    CHECK_DESC("extra non-required bits -> still FEED",
        TaskHealth_Decide(TASK_HEALTH_REQUIRED_MASK | (1uL << 31),
                          TASK_HEALTH_REQUIRED_MASK, &missing)
            == TASK_HEALTH_FEED);

    /* degenerate config: nothing required -> always FEED */
    CHECK_DESC("required == 0 -> FEED",
        TaskHealth_Decide(0, 0, &missing) == TASK_HEALTH_FEED);
    CHECK_EQ_DESC("  missing == 0", missing, 0);

    /* NULL missing pointer must not crash */
    CHECK_DESC("NULL missing ptr tolerated",
        TaskHealth_Decide(TASK_HEALTH_REQUIRED_MASK,
                          TASK_HEALTH_REQUIRED_MASK, NULL)
            == TASK_HEALTH_FEED);
}

/* ---- Heartbeat set / SwapBeats read-clear ---- */
static void test_swap(void)
{
    printf("suite task-health-swap\n");

    CHECK_EQ_DESC("fresh window reads 0", TaskHealth_SwapBeats(), 0);

    TaskHealth_Heartbeat(TASK_HEALTH_BIT_SENSOR);
    TaskHealth_Heartbeat(TASK_HEALTH_BIT_MOTOR);
    CHECK_EQ_DESC("two beats accumulate",
        TaskHealth_SwapBeats(),
        TASK_HEALTH_BIT_SENSOR | TASK_HEALTH_BIT_MOTOR);

    CHECK_EQ_DESC("swap cleared the window", TaskHealth_SwapBeats(), 0);

    TaskHealth_Heartbeat(TASK_HEALTH_BIT_STATE_MGR);
    CHECK_EQ_DESC("beat after swap lands in next window",
        TaskHealth_SwapBeats(), TASK_HEALTH_BIT_STATE_MGR);

    CHECK_DESC("bit 0 rejected (no crash)",
        (TaskHealth_Heartbeat(0), 1));
    CHECK_EQ_DESC("still empty after invalid beat",
        TaskHealth_SwapBeats(), 0);
}

/* ---- Alive counters: Heartbeat increments per-task count ---- */
static void test_counts(void)
{
    u32 c[TASK_HEALTH_COUNT_MAX];

    printf("suite task-health-counts\n");

    /* drain any counts left by earlier suites (suite isolation) */
    TaskHealth_SwapCounts(c);

    /* invalid beats do not touch counts */
    TaskHealth_Heartbeat(0);
    TaskHealth_SwapCounts(c);
    CHECK_EQ_DESC("invalid beat leaves counts 0",
        c[0] + c[1] + c[2] + c[3], 0u);

    /* repeated beats accumulate per index */
    TaskHealth_Heartbeat(TASK_HEALTH_BIT_SENSOR);    /* idx2 */
    TaskHealth_Heartbeat(TASK_HEALTH_BIT_SENSOR);
    TaskHealth_Heartbeat(TASK_HEALTH_BIT_SENSOR);    /* 3x sensor */
    TaskHealth_Heartbeat(TASK_HEALTH_BIT_STATE_MGR); /* 1x state_mgr */
    TaskHealth_SwapCounts(c);
    CHECK_EQ_DESC("sensor count == 3", c[TASK_HEALTH_IDX_SENSOR], 3u);
    CHECK_EQ_DESC("state_mgr count == 1", c[TASK_HEALTH_IDX_STATE_MGR], 1u);
    CHECK_EQ_DESC("motor untouched", c[TASK_HEALTH_IDX_MOTOR], 0u);
    CHECK_EQ_DESC("speed_calc untouched", c[TASK_HEALTH_IDX_SPEED_CALC], 0u);

    /* swap cleared the counts */
    TaskHealth_SwapCounts(c);
    CHECK_EQ_DESC("swap cleared all counts",
        c[0] + c[1] + c[2] + c[3], 0u);
}

/* ---- TaskHealth_DecideAlive: bitmap AND min-count both required ---- */
static void test_decide_alive(void)
{
    static const u32 minc[TASK_HEALTH_COUNT_MAX] = {
        TASK_HEALTH_MIN_COUNT_STATE_MGR,
        TASK_HEALTH_MIN_COUNT_MOTOR,
        TASK_HEALTH_MIN_COUNT_SENSOR,
        TASK_HEALTH_MIN_COUNT_SPEED_CALC,
    };
    u32 c[TASK_HEALTH_COUNT_MAX];
    u32 missing, underflow;

    printf("suite task-health-decide-alive\n");

    /* all required beats present + all counts >= min -> FEED */
    memset(c, 0, sizeof(c));
    c[TASK_HEALTH_IDX_STATE_MGR]  = TASK_HEALTH_MIN_COUNT_STATE_MGR;
    c[TASK_HEALTH_IDX_MOTOR]      = TASK_HEALTH_MIN_COUNT_MOTOR;
    c[TASK_HEALTH_IDX_SENSOR]     = TASK_HEALTH_MIN_COUNT_SENSOR;
    c[TASK_HEALTH_IDX_SPEED_CALC] = TASK_HEALTH_MIN_COUNT_SPEED_CALC;
    CHECK_DESC("full beats + full counts -> FEED",
        TaskHealth_DecideAlive(TASK_HEALTH_REQUIRED_MASK,
                               TASK_HEALTH_REQUIRED_MASK,
                               c, minc, &missing, &underflow)
            == TASK_HEALTH_FEED);
    CHECK_EQ_DESC("  missing == 0", missing, 0u);
    CHECK_EQ_DESC("  underflow == 0", underflow, 0u);

    /* KEY CASE: all beats present (bitmap says alive) but state_mgr ran only
     * 1 cycle in the window (< min 20) -> the task is "alive but starved":
     * this is exactly what the old bitmap-only verdict could NOT catch.
     * (sensor cannot play this role: its period is 1000ms, so the 2s window
     *  expects 2 beats and the 50%-tolerance floor is 1.) */
    memset(c, 0, sizeof(c));
    c[TASK_HEALTH_IDX_STATE_MGR]  = 1u;   /* below min */
    c[TASK_HEALTH_IDX_MOTOR]      = TASK_HEALTH_MIN_COUNT_MOTOR;
    c[TASK_HEALTH_IDX_SENSOR]     = TASK_HEALTH_MIN_COUNT_SENSOR;
    c[TASK_HEALTH_IDX_SPEED_CALC] = TASK_HEALTH_MIN_COUNT_SPEED_CALC;
    CHECK_DESC("all beats but state_mgr under-counted -> HOLD",
        TaskHealth_DecideAlive(TASK_HEALTH_REQUIRED_MASK,
                               TASK_HEALTH_REQUIRED_MASK,
                               c, minc, &missing, &underflow)
            == TASK_HEALTH_HOLD);
    CHECK_EQ_DESC("  missing == 0 (bitmap was fine)", missing, 0u);
    CHECK_EQ_DESC("  underflow == STATE_MGR bit", underflow,
        TASK_HEALTH_BIT_STATE_MGR);

    /* motor under-counted too -> HOLD with underflow mask both */
    memset(c, 0, sizeof(c));
    c[TASK_HEALTH_IDX_STATE_MGR]  = TASK_HEALTH_MIN_COUNT_STATE_MGR;
    c[TASK_HEALTH_IDX_MOTOR]      = TASK_HEALTH_MIN_COUNT_MOTOR - 1u;
    c[TASK_HEALTH_IDX_SENSOR]     = TASK_HEALTH_MIN_COUNT_SENSOR;
    c[TASK_HEALTH_IDX_SPEED_CALC] = TASK_HEALTH_MIN_COUNT_SPEED_CALC;
    CHECK_EQ_DESC("motor under-counted -> HOLD + underflow motor",
        TaskHealth_DecideAlive(TASK_HEALTH_REQUIRED_MASK,
                               TASK_HEALTH_REQUIRED_MASK,
                               c, minc, NULL, &underflow),
        TASK_HEALTH_HOLD);
    CHECK_EQ_DESC("  underflow == MOTOR bit", underflow, TASK_HEALTH_BIT_MOTOR);

    /* bit missing still HOLD even with full counts */
    memset(c, 0, sizeof(c));
    c[TASK_HEALTH_IDX_STATE_MGR]  = TASK_HEALTH_MIN_COUNT_STATE_MGR;
    c[TASK_HEALTH_IDX_MOTOR]      = TASK_HEALTH_MIN_COUNT_MOTOR;
    c[TASK_HEALTH_IDX_SENSOR]     = TASK_HEALTH_MIN_COUNT_SENSOR;
    c[TASK_HEALTH_IDX_SPEED_CALC] = TASK_HEALTH_MIN_COUNT_SPEED_CALC;
    CHECK_DESC("full counts but STATE_MGR beat missing -> HOLD",
        TaskHealth_DecideAlive(TASK_HEALTH_REQUIRED_MASK & ~TASK_HEALTH_BIT_STATE_MGR,
                               TASK_HEALTH_REQUIRED_MASK,
                               c, minc, &missing, &underflow)
            == TASK_HEALTH_HOLD);
    CHECK_EQ_DESC("  missing == STATE_MGR bit", missing, TASK_HEALTH_BIT_STATE_MGR);

    /* NULL counts/min_counts degrades to plain bitmap verdict */
    CHECK_DESC("NULL counts degrades to bitmap-only (FEED)",
        TaskHealth_DecideAlive(TASK_HEALTH_REQUIRED_MASK,
                               TASK_HEALTH_REQUIRED_MASK,
                               NULL, NULL, NULL, NULL)
            == TASK_HEALTH_FEED);
    CHECK_DESC("NULL counts degrades to bitmap-only (HOLD)",
        TaskHealth_DecideAlive(0u, TASK_HEALTH_REQUIRED_MASK,
                               NULL, NULL, NULL, NULL)
            == TASK_HEALTH_HOLD);

    /* standby empty mask: counts irrelevant -> FEED */
    CHECK_DESC("standby empty mask -> FEED even with zero counts",
        TaskHealth_DecideAlive(0u, 0u, c, minc, NULL, NULL)
            == TASK_HEALTH_FEED);

    /* quantitative diagnostic: expect/min are compile-time per-task;
     * lastCounts lets Diagnosis compute "expected - actual = missing
     * beats". Simulate the HealthTask snapshot: SwapCounts moves the
     * window counts into stats so Diagnosis can read exact values. */
    {
        u32 drain[TASK_HEALTH_COUNT_MAX];
        TaskHealth_SwapCounts(drain);                 /* clear window */
        TaskHealth_Heartbeat(TASK_HEALTH_BIT_SENSOR); /* 1 beat only */
        TaskHealth_Heartbeat(TASK_HEALTH_BIT_MOTOR);  /* 2 beats */
        TaskHealth_Heartbeat(TASK_HEALTH_BIT_MOTOR);
        TaskHealth_SwapCounts(c);
        CHECK_EQ_DESC("diagnostic reads exact counts (sensor=1)",
            c[TASK_HEALTH_IDX_SENSOR], 1u);
        CHECK_EQ_DESC("diagnostic reads exact counts (motor=2)",
            c[TASK_HEALTH_IDX_MOTOR], 2u);
    }
}

/* ---- ResetCause_Decode ---- */
static void test_reset_cause(void)
{
    printf("suite reset-cause-decode\n");

    CHECK_EQ_DESC("CSR == 0 -> UNKNOWN",
        ResetCause_Decode(0), RESET_CAUSE_UNKNOWN);

    CHECK_EQ_DESC("PORRSTF only -> POWER_ON",
        ResetCause_Decode(RESET_CAUSE_CSR_PORRSTF), RESET_CAUSE_POWER_ON);

    /* Power-on ALSO asserts PINRSTF internally (NRST pulled low) - must
     * still decode as POWER_ON, not PIN_RESET (ordering edge case). */
    CHECK_EQ_DESC("PORRSTF|PINRSTF (real power-on) -> POWER_ON",
        ResetCause_Decode(RESET_CAUSE_CSR_PORRSTF |
                          RESET_CAUSE_CSR_PINRSTF),
        RESET_CAUSE_POWER_ON);

    CHECK_EQ_DESC("PINRSTF only -> PIN_RESET",
        ResetCause_Decode(RESET_CAUSE_CSR_PINRSTF), RESET_CAUSE_PIN_RESET);

    /* Software reset also accompanies PINRSTF on F103 */
    CHECK_EQ_DESC("SFTRSTF|PINRSTF -> SOFTWARE_RESET",
        ResetCause_Decode(RESET_CAUSE_CSR_SFTRSTF |
                          RESET_CAUSE_CSR_PINRSTF),
        RESET_CAUSE_SOFTWARE_RESET);

    /* Watchdog resets also pulse NRST - must outrank PIN flag */
    CHECK_EQ_DESC("IWDGRSTF|PINRSTF -> IWDG_RESET",
        ResetCause_Decode(RESET_CAUSE_CSR_IWDGRSTF |
                          RESET_CAUSE_CSR_PINRSTF),
        RESET_CAUSE_IWDG_RESET);

    CHECK_EQ_DESC("WWDGRSTF -> WWDG_RESET",
        ResetCause_Decode(RESET_CAUSE_CSR_WWDGRSTF),
        RESET_CAUSE_WWDG_RESET);

    CHECK_EQ_DESC("LPWRRSTF -> LOW_POWER_RESET",
        ResetCause_Decode(RESET_CAUSE_CSR_LPWRRSTF),
        RESET_CAUSE_LOW_POWER_RESET);

    /* Priority ordering with multiple sticky flags */
    CHECK_EQ_DESC("IWDG|WWDG|SFT co-set -> IWDG wins",
        ResetCause_Decode(RESET_CAUSE_CSR_IWDGRSTF |
                          RESET_CAUSE_CSR_WWDGRSTF |
                          RESET_CAUSE_CSR_SFTRSTF),
        RESET_CAUSE_IWDG_RESET);

    CHECK_EQ_DESC("WWDG|SFT co-set -> WWDG wins",
        ResetCause_Decode(RESET_CAUSE_CSR_WWDGRSTF |
                          RESET_CAUSE_CSR_SFTRSTF),
        RESET_CAUSE_WWDG_RESET);

    CHECK_EQ_DESC("all flags set -> IWDG (highest priority)",
        ResetCause_Decode(RESET_CAUSE_CSR_LPWRRSTF |
                          RESET_CAUSE_CSR_WWDGRSTF |
                          RESET_CAUSE_CSR_IWDGRSTF |
                          RESET_CAUSE_CSR_SFTRSTF  |
                          RESET_CAUSE_CSR_PORRSTF  |
                          RESET_CAUSE_CSR_PINRSTF),
        RESET_CAUSE_IWDG_RESET);

    /* Name lookup stays in range */
    CHECK_DESC("GetName bounds-checked",
        strcmp(ResetCause_GetName((ResetCause_t)999), "UNKNOWN") == 0);
}

/* ---- Task 14: active/standby required mask switching + IWDG query ---- */
static u8 stub_iwdg_query_off(void) { return 0; }

static void test_standby_mask(void)
{
    printf("suite task-health-standby-mask\n");

    /* active: all four required bits demanded (current behavior) */
    CHECK_EQ_DESC("active mask == full REQUIRED_MASK",
        TaskHealth_RequiredMaskFor(0u), TASK_HEALTH_REQUIRED_MASK);

    /* standby: no required beats - every supervised task is blocked on an
     * event during standby, so "one beat per valid cycle" is vacuous.
     * HealthTask still runs its 2s window when IWDG requires service and
     * feeds on FEED; the missing-heartbeat veto only exists under the
     * active mask. */
    CHECK_EQ_DESC("standby mask constant is 0",
        TASK_HEALTH_STANDBY_REQUIRED_MASK, 0u);
    CHECK_EQ_DESC("standby mask == 0 (no required beats)",
        TaskHealth_RequiredMaskFor(1u), TASK_HEALTH_STANDBY_REQUIRED_MASK);

    CHECK_DESC("standby: empty beats -> FEED (periodic feed keeps IWDG alive)",
        TaskHealth_Decide(0u, TaskHealth_RequiredMaskFor(1u), NULL)
            == TASK_HEALTH_FEED);
    CHECK_DESC("active: empty beats -> HOLD (stop feeding)",
        TaskHealth_Decide(0u, TaskHealth_RequiredMaskFor(0u), NULL)
            == TASK_HEALTH_HOLD);

    /* IWDG service query injection: bound -> forwards to the callback;
     * unbound -> conservative default 1 (misjudging "no service needed"
     * could sleep through a hardware IWDG; the other direction only
     * costs deep-sleep opportunities). */
    TaskHealth_BindIwdgQuery(stub_iwdg_query_off);
    CHECK_EQ_DESC("bound query forwards (returns 0)",
        TaskHealth_IwdgServiceNow(), 0u);
    TaskHealth_BindIwdgQuery(NULL);
    CHECK_EQ_DESC("unbound again -> conservative default 1",
        TaskHealth_IwdgServiceNow(), 1u);

    /* Task 14 exit grace: the window woken by the ACTIVE edge keeps the
     * standby (empty) verdict because periodic tasks have not run their
     * first active-cycle beat yet. Without grace, every standby exit
     * deterministically HOLDs one window (beats==0 vs full mask), which
     * stretches the feed gap to 2 windows = 4s and trips a fast-LSI
     * (>40kHz) IWDG early. Next window returns to the full mask. */
    CHECK_EQ_DESC("exit grace: active+grace -> standby mask (transition)",
        TaskHealth_RequiredMaskFor(TaskHealth_EffectiveStandby(0u, 1u)),
        TASK_HEALTH_STANDBY_REQUIRED_MASK);
    CHECK_EQ_DESC("active without grace -> full mask",
        TaskHealth_RequiredMaskFor(TaskHealth_EffectiveStandby(0u, 0u)),
        TASK_HEALTH_REQUIRED_MASK);
    CHECK_EQ_DESC("standby ignores grace flag",
        TaskHealth_RequiredMaskFor(TaskHealth_EffectiveStandby(1u, 1u)),
        TASK_HEALTH_STANDBY_REQUIRED_MASK);
    CHECK_DESC("grace window: empty beats -> FEED (feed gap stays <=2s)",
        TaskHealth_Decide(0u,
            TaskHealth_RequiredMaskFor(TaskHealth_EffectiveStandby(0u, 1u)),
            NULL) == TASK_HEALTH_FEED);
}

int main(void)
{
    test_decide();
    test_swap();
    test_counts();
    test_decide_alive();
    test_reset_cause();
    test_standby_mask();

    printf("mock result: %d/%d cases passed (+ Alive Supervision)\n",
           g_total - g_failed, g_total);
    return g_failed ? 1 : 0;
}
