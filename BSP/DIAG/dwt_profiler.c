/*
 * DWT 性能统计模块（实现）
 * 详见 dwt_profiler.h
 */
#include "dwt_profiler.h"

#if defined(DWT_PROFILER_ENABLED) && DWT_PROFILER_ENABLED

#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#ifndef UINT32_MAX
#define UINT32_MAX  ((u32)0xFFFFFFFFu)
#endif

#ifndef CoreDebug_DEMCR_TRCENA_Pos
#define CoreDebug_DEMCR_TRCENA_Pos   (24U)
#define CoreDebug_DEMCR_TRCENA_Msk   (1UL << CoreDebug_DEMCR_TRCENA_Pos)
#endif
#ifndef DWT_CTRL_CYCCNTENA_Pos
#define DWT_CTRL_CYCCNTENA_Pos       (0U)
#define DWT_CTRL_CYCCNTENA_Msk       (1UL << DWT_CTRL_CYCCNTENA_Pos)
#endif

#ifndef DWT_BASE
#define  DWT_BASE        (0xE0001000UL)
typedef struct {
    volatile u32 CTRL;
    volatile u32 CYCCNT;
    volatile u32 CPICNT;
    volatile u32 EXCCNT;
    volatile u32 SLEEPCNT;
    volatile u32 LSUCNT;
    volatile u32 FOLDCNT;
    volatile u32 PCSR;
} DWT_Type;
#define  DWT             ((DWT_Type *) DWT_BASE)
#endif
#ifndef CoreDebug_BASE
#define  CoreDebug_BASE  (0xE000EDF0UL)
typedef struct { volatile u32 DEMCR; } CoreDebug_Type;
#define  CoreDebug       ((CoreDebug_Type *) CoreDebug_BASE)
#endif

static DwtSlotStats_t g_slots[DWT_SLOT_MAX];
static u8            g_initialized = 0;

static const char * const g_slot_names[DWT_SLOT_MAX] = {
    "StateMgr", "Motor",   "Sensor",  "WindSpd",
    "SpeedCalc","KeyScan","UI",      "Diag"
};

DWT_STATIC_INLINE u32 dwt_rdcyc(void) { return DWT->CYCCNT; }

void DWT_Init(void)
{
    unsigned i;
    if (g_initialized) return;

    memset(g_slots, 0, sizeof(g_slots));
    for (i = 0; i < (unsigned)DWT_SLOT_MAX; i++) {
        g_slots[i].exec_cycles_min   = UINT32_MAX;
        g_slots[i].period_cycles_min = UINT32_MAX;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    __DSB();
    DWT->CYCCNT  = 0u;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();

    g_initialized = 1u;
}

void DWT_TaskPeriodMark(DwtSlotId_t slot)
{
    u32 now;
    DwtSlotStats_t *s;

    if (!g_initialized || (unsigned)slot >= (unsigned)DWT_SLOT_MAX) return;
    s   = &g_slots[(unsigned)slot];
    now = dwt_rdcyc();

    if (s->period_count == 0u && s->_priv_last_period == 0u) {
        s->_priv_last_period = now;
        return;
    }
    {
        u32 delta = now - s->_priv_last_period;
        s->_priv_last_period = now;
        s->period_count++;
        if (delta < s->period_cycles_min) s->period_cycles_min = delta;
        if (delta > s->period_cycles_max) s->period_cycles_max = delta;
    }
}

void DWT_ExecBegin(DwtSlotId_t slot)
{
    DwtSlotStats_t *s;
    u32 now;

    if (!g_initialized || (unsigned)slot >= (unsigned)DWT_SLOT_MAX) return;
    s = &g_slots[(unsigned)slot];
    if (s->_priv_in_exec) return;

    now = dwt_rdcyc();
    DWT_TaskPeriodMark(slot);
    s->_priv_last_start = now;
    s->_priv_in_exec    = 1u;
}

void DWT_ExecEnd(DwtSlotId_t slot)
{
    DwtSlotStats_t *s;
    u32 now, delta;

    if (!g_initialized || (unsigned)slot >= (unsigned)DWT_SLOT_MAX) return;
    s = &g_slots[(unsigned)slot];
    if (!s->_priv_in_exec) return;

    now   = dwt_rdcyc();
    delta = now - s->_priv_last_start;

    s->_priv_in_exec = 0u;
    s->exec_count++;
    s->exec_cycles_sum += delta;
    if (delta < s->exec_cycles_min) s->exec_cycles_min = delta;
    if (delta > s->exec_cycles_max) s->exec_cycles_max = delta;
}

void DWT_SnapshotAndReset(DwtSnapshot_t *out)
{
    unsigned i;
    u32 cyc;
    DwtSlotStats_t   *s;
    DwtSlotSnapshot_t *o;

    if (out == (DwtSnapshot_t *)0) return;

    taskENTER_CRITICAL();
    {
        cyc = dwt_rdcyc();
        for (i = 0; i < (unsigned)DWT_SLOT_MAX; i++) {
            s = &g_slots[i];
            o = &out->slots[i];

            o->exec_count        = s->exec_count;
            o->exec_cycles_sum   = s->exec_cycles_sum;
            o->exec_cycles_min   = (s->exec_cycles_min   == UINT32_MAX) ? 0u : s->exec_cycles_min;
            o->exec_cycles_max   = s->exec_cycles_max;

            o->period_count      = s->period_count;
            o->period_cycles_min = (s->period_cycles_min == UINT32_MAX) ? 0u : s->period_cycles_min;
            o->period_cycles_max = s->period_cycles_max;

            s->exec_count        = 0u;
            s->exec_cycles_sum   = 0u;
            s->exec_cycles_min   = UINT32_MAX;
            s->exec_cycles_max   = 0u;
            s->period_count      = 0u;
            s->period_cycles_min = UINT32_MAX;
            s->period_cycles_max = 0u;
        }
        out->cpu_hz         = (u32)SystemCoreClock;
        out->snapshot_cycle = cyc;
    }
    taskEXIT_CRITICAL();
}

const char *DWT_SlotName(DwtSlotId_t slot)
{
    return ((unsigned)slot < (unsigned)DWT_SLOT_MAX)
        ? g_slot_names[(unsigned)slot]
        : "Unknown";
}

#endif /* DWT_PROFILER_ENABLED */
