/*
 * 优先级反转对照实验：Binary Semaphore 与 FreeRTOS Mutex 的三任务对比
 *
 * 实验设计（对照）：
 *   两组共用三任务模型：
 *     LowTask    ：低优先级，持有共享资源后做长时间「持有区自旋」
 *     MediumTask ：中优先级，CPU 密集循环（不碰共享资源）
 *     HighTask   ：高优先级，延后启动，等待共享资源
 *
 *   Group A：共享资源用 Binary Semaphore（无优先级继承）
 *     → 预期：Medium 抢占 Low 但未释放信号量，High 被 Medium 无限阻塞
 *（经典优先级反转）
 *   Group B：共享资源用 FreeRTOS Mutex（带优先级继承）
 *     → 预期：Low 被临时提升到 High 的优先级，Medium 无法抢占；
 *       Low 释放后立即交还优先级，High 立刻拿到锁（继承解反转）
 *
 *   DWT 记录指标（可由 DiagnosticTask 或实验自身串口打印）：
 *     - high_block_cycles ：High 从尝试拿锁 → 拿到锁的耗时
 *     - low_release_cycles：Low 进入临界区 → 释放锁的真实耗时
 *
 * 使用方式：
 *   1) 在 app_tasks.h / FreeRTOSConfig.h 中
 *      #define PRIORITY_INVERSION_EXPERIMENT  1
 *   2) 在 StartTask 中调用 PriInv_Start(PRIINV_GROUP_MUTEX) 或 ..._BINARY；
 *   3) 运行约 5~10 秒，串口会输出两组对照指标。
 *
 * 使用限制：
 *   本模块只提供测量代码路径与指标输出能力。high_block_cycles 与
 *   low_release_cycles 的具体数值必须在 STM32F103 实测后才有意义；
 *   未实测前不得引用任何定量结果，只说明机制与代码路径。
 *
 * 注意：
 *   - 该模块默认关闭，不启用时 Start 函数为空实现，0 代码 / 0 RAM；
 *   - 启用后会自动创建 3 个专用任务，不与业务任务抢占同名硬件资源
 *（仅使用 DWT CYCCNT 和串口，不触碰 GPIO/PWM/电机/编码器）。
 */
#ifndef __PRIORITY_INVERSION_H
#define __PRIORITY_INVERSION_H

#include "sys.h"

/* Build switch lives HERE (single source of truth, included before use).
 * Default 0 = off: empty inline stub, zero code / zero RAM in production.
 * NOTE: requires DWT_PROFILER_ENABLED=1 when on (enforced in .c). */
#ifndef PRIORITY_INVERSION_EXPERIMENT
#define PRIORITY_INVERSION_EXPERIMENT 0
#endif
/* ArmCC v5 (C90) has no "inline" keyword */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#define PRIINV_STATIC_INLINE static __inline
#elif defined(__GNUC__) || defined(__clang__)
#define PRIINV_STATIC_INLINE static inline
#else
#define PRIINV_STATIC_INLINE static
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PRIINV_GROUP_BINARY = 0,   /* Binary Semaphore 组（预期反转）*/
    PRIINV_GROUP_MUTEX         /* Mutex 组（预期继承解反转）*/
} PriInvGroup_t;

#if defined(PRIORITY_INVERSION_EXPERIMENT) && PRIORITY_INVERSION_EXPERIMENT

typedef struct {
    u32 high_block_cycles;     /* High 被阻塞总耗时（cycles）*/
    u32 low_hold_cycles;       /* Low 持锁区耗时（cycles）*/
    u32 medium_loop_count;     /* Medium 在实验窗口内的循环次数（用于证据）*/
    u8  done;                  /* 1 = 指标已就绪可打印 */
} PriInvResult_t;

/* 启动实验：创建 3 个任务并运行一轮（结果写入 out）。
 * 实验结束后 out->done == 1，方可读取数值。*/
void PriInv_Start(PriInvGroup_t group, PriInvResult_t *out);

#else  /* 关闭：空实现 */

typedef struct { u32 a; u32 b; u32 c; u8 d; } PriInvResult_t;

PRIINV_STATIC_INLINE void PriInv_Start(PriInvGroup_t group, PriInvResult_t *out)
{ (void)group; if (out) { out->a=0; out->b=0; out->c=0; out->d=0; } }

#endif

#ifdef __cplusplus
}
#endif

#endif
