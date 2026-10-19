/*
 * DWT 性能统计模块：基于 DWT->CYCCNT 的指令周期测量
 *
 * 能力：
 *   1. 执行区间 (exec) 统计：count, cycles_sum, cycles_min, cycles_max
 *   2. 唤醒间隔 (period) 统计：count, interval_min, interval_max
 *   3. 原子快照复制 + 窗口清零：DiagnosticTask 复制快照后 printf，
 *      被测任务只写 RAM，不允许直接 printf
 *   4. 32 位 CYCCNT 回绕：(end-start) 无符号减法天然正确；
 *      72 MHz 下 ≈ 59.6 s 回绕一次，仅用于短执行区间和有限统计窗口。
 *
 * 构建开关：
 *   #define DWT_PROFILER_ENABLED 1  启用；未定义或 0 时宏为空，0 影响。
 *
 */
#ifndef __DWT_PROFILER_H
#define __DWT_PROFILER_H

#include "sys.h"

/* Build switch lives HERE so every translation unit (main.c / app_tasks.c /
 * dwt_profiler.c / prio_inversion.c) sees the same value via this header.
 * Default 0 = off: empty macros, zero code / zero RAM in production image.
 * Turn on by setting 1 here (or via compiler -D flag). */
#ifndef DWT_PROFILER_ENABLED
#define DWT_PROFILER_ENABLED 0
#endif
/* ArmCC v5 (C90) has no "inline" keyword - use compiler-portable variant */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#define DWT_STATIC_INLINE static __inline
#elif defined(__GNUC__) || defined(__clang__)
#define DWT_STATIC_INLINE static inline
#else
#define DWT_STATIC_INLINE static
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DWT_SLOT_STATE_MGR = 0,
    DWT_SLOT_MOTOR,
    DWT_SLOT_SENSOR,
    DWT_SLOT_WIND_SPEED,
    DWT_SLOT_SPEED_CALC,
    DWT_SLOT_KEY_SCAN,
    DWT_SLOT_UI,
    DWT_SLOT_DIAG,
    DWT_SLOT_MAX
} DwtSlotId_t;

typedef struct {
    u32  exec_count;
    u32  exec_cycles_sum;
    u32  exec_cycles_min;
    u32  exec_cycles_max;
    u32  period_count;
    u32  period_cycles_min;
    u32  period_cycles_max;
    u32  _priv_last_start;
    u32  _priv_last_period;
    u8   _priv_in_exec;
} DwtSlotStats_t;

typedef struct {
    u32  exec_count;
    u32  exec_cycles_sum;
    u32  exec_cycles_min;
    u32  exec_cycles_max;
    u32  period_count;
    u32  period_cycles_min;
    u32  period_cycles_max;
} DwtSlotSnapshot_t;

typedef struct {
    DwtSlotSnapshot_t slots[DWT_SLOT_MAX];
    u32               cpu_hz;
    u32               snapshot_cycle;
} DwtSnapshot_t;

#if defined(DWT_PROFILER_ENABLED) && DWT_PROFILER_ENABLED

void        DWT_Init(void);
void        DWT_ExecBegin(DwtSlotId_t slot);
void        DWT_ExecEnd(DwtSlotId_t slot);
void        DWT_TaskPeriodMark(DwtSlotId_t slot);
void        DWT_SnapshotAndReset(DwtSnapshot_t *out);
const char *DWT_SlotName(DwtSlotId_t slot);

#define DWT_INIT()                 DWT_Init()
#define DWT_EXEC_BEGIN(slot)       DWT_ExecBegin(slot)
#define DWT_EXEC_END(slot)         DWT_ExecEnd(slot)
#define DWT_PERIOD_MARK(slot)      DWT_TaskPeriodMark(slot)

#else

DWT_STATIC_INLINE void DWT_Init(void) { }
DWT_STATIC_INLINE void DWT_ExecBegin(DwtSlotId_t slot)       { (void)slot; }
DWT_STATIC_INLINE void DWT_ExecEnd(DwtSlotId_t slot)         { (void)slot; }
DWT_STATIC_INLINE void DWT_TaskPeriodMark(DwtSlotId_t slot)  { (void)slot; }
DWT_STATIC_INLINE void DWT_SnapshotAndReset(DwtSnapshot_t *out)
{
    unsigned i;
    if (out == (DwtSnapshot_t *)0) return;
    for (i = 0; i < (unsigned)DWT_SLOT_MAX; i++) {
        out->slots[i].exec_count        = 0;
        out->slots[i].exec_cycles_sum   = 0;
        out->slots[i].exec_cycles_min   = 0;
        out->slots[i].exec_cycles_max   = 0;
        out->slots[i].period_count      = 0;
        out->slots[i].period_cycles_min = 0;
        out->slots[i].period_cycles_max = 0;
    }
    out->cpu_hz         = 0;
    out->snapshot_cycle = 0;
}
DWT_STATIC_INLINE const char *DWT_SlotName(DwtSlotId_t slot)
{
    static const char * const s_names[DWT_SLOT_MAX] = {
        "StateMgr", "Motor",   "Sensor",  "WindSpd",
        "SpeedCalc","KeyScan","UI",      "Diag"
    };
    return ((unsigned)slot < (unsigned)DWT_SLOT_MAX)
        ? s_names[(unsigned)slot] : "Unknown";
}

#define DWT_INIT()                 ((void)0)
#define DWT_EXEC_BEGIN(slot)       ((void)0)
#define DWT_EXEC_END(slot)         ((void)0)
#define DWT_PERIOD_MARK(slot)      ((void)0)

#endif

#ifdef __cplusplus
}
#endif

#endif
