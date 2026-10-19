/*
 * 独立看门狗模块（任务心跳监督的最终执行者）
 *
 * 设计要点（证据链：监督链路设计）：
 *   1. IWDG 独立于主时钟（LSI ≈ 40kHz），主时钟失控仍能复位芯片；
 *   2. 喂狗权唯一：只有 HealthTask 判定心跳齐全后才允许调用
 *      Watchdog_Feed()，正式业务任务禁止自行喂狗
 *（否则单任务卡死也可能被其他任务的喂狗掩盖）；
 *   3. 超时配置：监督窗口 2s（见 task_health.h）+ 2 倍裕量 = 4s。
 *      LSI 40kHz 标称下 prescaler=64 → 计数周期 1.6ms，
 *      reload=2500 → 超时 4.0s；
 *      LSI 有出厂偏差（30~60kHz），实际超时在 2.7~5.3s 之间漂移，
 *      未做实测标定，不引用具体超时数值规则；
 *   4. 调试友好：DBGMCU_Config(DBGMCU_IWDG_STOP) 使调试器暂停内核时
 *      同步冻结 IWDG 计数，防止单步调试被误咬复位；
 *   5. 构建开关 WATCHDOG_ENABLED 默认 0：
 *      开发/演示构建不启用硬件看门狗（避免调试中意外复位），
 *      HealthTask 的心跳收集与判定逻辑独立于本开关照常运行。
 *      量产构建置 1 后，喂狗动作才真正写入 IWDG 重载寄存器。
 *
 */
#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#include "sys.h"

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * 构建开关（默认关闭：开发构建零行为变化）
 *----------------------------------------------------------*/
#ifndef WATCHDOG_ENABLED
#define WATCHDOG_ENABLED     0
#endif

/*-----------------------------------------------------------
 * 超时参数（LSI 40kHz 标称）+ 静态安全预算（Task 14，设计 §6.7）
 *
 * LSI 出厂偏差 30~60kHz（无实测标定，故不引用具体实际超时数值）：标称 40kHz 下超时 4.0s；上界 60kHz 下最短超时
 * 2667ms，下界 30kHz 下最长超时 5335ms。
 *
 * 预算不等式（task_health.h 编译期 #error 锁定，mock s7 运行时复查）：
 *   喂狗最大间隔 = HealthTask 监督窗口 2s
 *              < 最短超时 WATCHDOG_TIMEOUT_MIN_MS = 2667ms
 * 正常运行的任何窗口节奏（含事件唤醒抖动，微秒级）都不会先于判定
 * 窗口咬狗；HOLD 恢复场景（间隔 2 窗口=4s）在最坏 LSI 下会提前
 * 复位——方向安全（fail-safe 提前，不会漏咬）。
 *----------------------------------------------------------*/
#define WATCHDOG_TIMEOUT_MS  4000uL                    /* 超时 = 2s 窗口 + 2 倍裕量（标称）*/
#define WATCHDOG_LSI_HZ      40000uL                   /* LSI 标称频率 */
#define WATCHDOG_PRESCALER   64uL                      /* 计数周期 = 64/40kHz = 1.6ms */
#define WATCHDOG_RELOAD      (WATCHDOG_TIMEOUT_MS * WATCHDOG_LSI_HZ / WATCHDOG_PRESCALER / 1000uL)  /* = 2500 */

#define WATCHDOG_LSI_HZ_MIN      30000uL               /* LSI 出厂下界（最长超时）*/
#define WATCHDOG_LSI_HZ_MAX      60000uL               /* LSI 出厂上界（最短超时，保守预算用）*/
/* (2500+1)*64/60kHz = 2667ms（RM0008: timeout=(reload+1)*prescaler/LSI）*/
#define WATCHDOG_TIMEOUT_MIN_MS  ((WATCHDOG_RELOAD + 1uL) * WATCHDOG_PRESCALER * 1000uL / WATCHDOG_LSI_HZ_MAX)
/* (2500+1)*64/30kHz = 5335ms */
#define WATCHDOG_TIMEOUT_MAX_MS  ((WATCHDOG_RELOAD + 1uL) * WATCHDOG_PRESCALER * 1000uL / WATCHDOG_LSI_HZ_MIN)

/* ArmCC v5 (C90) has no "inline" keyword - use compiler-portable variant */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#define WDG_STATIC_INLINE static __inline
#elif defined(__GNUC__) || defined(__clang__)
#define WDG_STATIC_INLINE static inline
#else
#define WDG_STATIC_INLINE static
#endif

#if defined(WATCHDOG_ENABLED) && WATCHDOG_ENABLED

/* 初始化并启动 IWDG（含调试冻结配置） */
void Watchdog_Init(void);

/* 喂狗：仅 HealthTask 允许调用 */
void Watchdog_Feed(void);

#else

WDG_STATIC_INLINE void Watchdog_Init(void) { }
WDG_STATIC_INLINE void Watchdog_Feed(void) { }

#endif

/*-----------------------------------------------------------
 * IWDG 周期服务需求查询（Task 14，设计 §6.7；条件外定义：
 * 软件 IWDG 关闭时 option byte 仍可能选择硬件 IWDG）
 *
 * 组装：WATCHDOG_ENABLED（软件路径是否已启动）+ FLASH->OBR 的
 * WDG_SW 位（复位后硬件是否自动运行 IWDG），判定经 lowpower_policy.c
 * 的纯函数 Watchdog_RequiresPeriodicService()（Task 11，主机可测）。
 * 返回 1 时：HealthTask 必须保持 2s 周期监督（喂狗），且低功耗决策
 * 层禁止无限期 EXTI-only STOP（强制浅睡）。
 * 目标侧专用（读 FLASH 寄存器）；host 测试经 TaskHealth_BindIwdgQuery
 * 注入替代（task_health.c 不直接依赖本函数）。
 *----------------------------------------------------------*/
u8 Watchdog_ServiceRequiredNow(void);

#ifdef __cplusplus
}
#endif

#endif /* __WATCHDOG_H */
