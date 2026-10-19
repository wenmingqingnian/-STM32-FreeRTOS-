/*
 * 板型感知的低功耗深睡决策
 *
 * 职责：
 *   1. 输入快照 -> 三态决策：LP_SHALLOW_TICKLESS / LP_STOP_RTC /
 *      LP_STOP_EXTI_ONLY。不用单一 bool 掩盖三条路径的差异
 *      ；
 *   2. IWDG 服务需求判定 Watchdog_RequiresPeriodicService()：
 *      软件 IWDG 已启动，或 option byte 选择了硬件 IWDG
 *（FLASH->OBR 的 WDG_SW 位未置位）时返回"需要周期服务"，
 *      此时禁止无限期 EXTI-only STOP（设计 §6.3/§6.7）。
 *
 * 纯函数契约：
 *   - 决策只读调用方组装的 LpPolicySnapshot_t，不读 GPIO、RTOS
 *     全局或任何外设寄存器；硬件层（Task 12 wrapper）负责组装快照；
 *   - 同一快照任意次调用结果一致，无隐藏状态。
 *
 * 决策优先顺序（设计 §6.4）：
 *   公共 STOP 门槛（待机+电机物理关断+蜂鸣器/IAP/UART/DMA 空闲+时钟
 *   健康）任一不满足 -> 浅睡；
 *   满足后：LSE 板型且 RTC ready 且预期空闲 > 唤醒保护 -> RTC 定时 STOP；
 *   否则：无任务等待超时且 IWDG 无需服务且存在有效按键 EXTI ->
 *   EXTI-only STOP；
 *   其余 -> 官方 SysTick 浅 Tickless。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 * 证据：docs/design-evidence/IAP_STOP_BASELINE.md
 *
 */
#ifndef __LOWPOWER_POLICY_H__
#define __LOWPOWER_POLICY_H__

#include <stddef.h>  /* NULL - ARMCC 与 host gcc 通用 */
#include "sys.h"     /* u8/u32：host stub 与目标 stm32f10x.h 均提供 */

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * 决策三态
 *----------------------------------------------------------*/
typedef enum {
    LP_SHALLOW_TICKLESS = 0,  /* 官方 SysTick 浅 Tickless（默认安全兜底） */
    LP_STOP_RTC,              /* RTC Alarm 定时 STOP（仅 BOARD_C8_LSE） */
    LP_STOP_EXTI_ONLY         /* 无限期按键 EXTI-only STOP（tick 冻结） */
} LpDecision_t;

/*-----------------------------------------------------------
 * 拒绝原因分类
 *
 * 语义：返回"阻止深睡（LP_STOP_RTC / LP_STOP_EXTI_ONLY 任一路径
 * 可达）的第一原因"，检查顺序与 LpPolicy_Decide 单一来源：
 *   公共门槛（按枚举顺序逐字段）-> RTC 路径可达即 NONE ->
 *   EXTI-only 前提（按序）。RTC 不可用属"降级到 EXTI 候选"，
 *   不构成拒绝原因（设计 §6.4 决策顺序）。
 * LP_REFUSE_COUNT 为尾哨：wrapper 统计数组边界。LP_REFUSE_NONE
 * 恒为 0——深睡达成的情形无需拒绝计数（下标 0 保留不计数）。
 *----------------------------------------------------------*/
typedef enum {
    LP_REFUSE_NONE = 0,          /* 深睡路径可达，无拒绝 */
    LP_REFUSE_NOT_STANDBY,       /* 公共门槛：非 MODE_STANDBY */
    LP_REFUSE_MOTOR_ON,          /* 公共门槛：电机未物理关断 */
    LP_REFUSE_BUZZER_BUSY,       /* 公共门槛：蜂鸣器队列/输出忙 */
    LP_REFUSE_IAP_BUSY,          /* 公共门槛：IAP 会话进行中 */
    LP_REFUSE_UART_BUSY,         /* 公共门槛：UART TX/DMA 未完成 */
    LP_REFUSE_CLOCK_UNHEALTHY,   /* 公共门槛：系统时钟降级 */
    LP_REFUSE_TASKS_WAITING,     /* EXTI 前提：有任务等待超时 */
    LP_REFUSE_IWDG_SERVICE,      /* EXTI 前提：IWDG 需要周期服务 */
    LP_REFUSE_NO_EXTI,           /* EXTI 前提：无有效按键 EXTI 唤醒源 */
    LP_REFUSE_COUNT              /* 尾哨：原因总数（含 NONE）*/
} LpRefuse_t;

/*-----------------------------------------------------------
 * 板型（生产由 CONFIG/board_profile.h 编译期宏映射，硬件层组装
 * 快照时填入；host 测试直接注入以覆盖三种板型）
 *
 * LP_BOARD_PENDING 期间禁止启用任何 LSE/RTC 深睡路径
 *（board_profile.h Task 1 门禁；本决策再次校验）
 *----------------------------------------------------------*/
typedef enum {
    LP_BOARD_PENDING = 0,
    LP_BOARD_C8_LSE,
    LP_BOARD_C8_NO_LSE
} LpBoardKind_t;

/*-----------------------------------------------------------
 * OBR.WDG_SW 位（对应 SPL FLASH_OBR_WDG_SW，bit16）
 *   =1 软件看门狗（IWDG 不由硬件自动启动）
 *   =0 硬件看门狗（复位后自动运行 IWDG，不可关闭）
 * 纯函数不 include SPL flash 头，位掩码在此自定义单一来源
 *----------------------------------------------------------*/
#define LP_FLASH_OBR_WDG_SW   (1uL << 16)

/*-----------------------------------------------------------
 * LSE 路径策略常量
 *
 * 这些是"策略值"，不是硬件实测值：
 *   LP_RTC_HZ   = 1024：LSE 32768Hz / (prescaler 31 + 1) 的设计配置；
 *   LP_RTOS_HZ  = 1000：configTICK_RATE_HZ（FreeRTOS tick）；
 *   LP_MAX_STOP_MS = 1000：无板测证据下的安全单次睡眠上限，
 *                 非功耗最优值（设计 §6.5）；
 *   LP_WAKE_GUARD_TICKS = 2：保守唤醒保护，板测后才能调优
 *（设计 §6.5 wake_guard）。
 * 换算实现见 lowpower_math.h/c（共用本组常量）。
 *----------------------------------------------------------*/
#define LP_RTC_HZ             1024u
#define LP_RTOS_HZ            1000u
#define LP_MAX_STOP_MS        1000u
#define LP_WAKE_GUARD_TICKS   2u

/*-----------------------------------------------------------
 * 决策快照：硬件/RTOS 层组装，决策函数只读（字段非 0 即真）
 *
 * 组装责任（Task 12 wrapper 接入时落地）：
 *   board                     <- board_profile.h 编译期宏映射
 *   standby_mode              <- WorkMode_t == MODE_STANDBY
 *   motor_physically_off      <- PWM/H 桥寄存器级确认
 *   buzzer_idle / iap_idle / uart_dma_idle <- 各外设忙标志
 *   clock_healthy             <- 时钟降级检测
 *   rtc_ready                 <- LSERDY + RTC 可用（仅 LSE 板型）
 *   no_tasks_waiting_timeout  <- eTaskConfirmSleepModeStatus()
 *   watchdog_requires_service <- Watchdog_RequiresPeriodicService()
 *   key_exti_available        <- 已确认的按键 EXTI 唤醒源
 *   expected_idle_ticks       <- xExpectedIdleTime
 *----------------------------------------------------------*/
typedef struct {
    LpBoardKind_t board;               /* 板型 */
    u8 standby_mode;                   /* 系统 MODE_STANDBY */
    u8 motor_physically_off;           /* 电机物理关断（PWM/H 桥安全） */
    u8 buzzer_idle;                    /* 蜂鸣器空闲 */
    u8 iap_idle;                       /* IAP 空闲 */
    u8 uart_dma_idle;                  /* UART TX/DMA 空闲 */
    u8 clock_healthy;                  /* 系统时钟无降级 */
    u8 rtc_ready;                      /* LSE/RTC ready（仅 LSE 板型有效） */
    u8 no_tasks_waiting_timeout;       /* eNoTasksWaitingTimeout */
    u8 watchdog_requires_service;      /* IWDG 需要周期喂狗 */
    u8 key_exti_available;             /* 至少一个有效按键 EXTI 唤醒源 */
    u32 expected_idle_ticks;           /* xExpectedIdleTime（ticks） */
} LpPolicySnapshot_t;

/*-----------------------------------------------------------
 * 三态决策（纯函数，只读快照）
 *
 * 返回 LP_SHALLOW_TICKLESS 的情形包括：快照为 NULL（防御）、
 * 公共 STOP 门槛任一不满足、RTC/EXTI-only 专属条件不满足。
 *----------------------------------------------------------*/
LpDecision_t LpPolicy_Decide(const LpPolicySnapshot_t *snap);

/*-----------------------------------------------------------
 * 拒绝原因分类（纯函数，只读快照
 *
 * 与 LpPolicy_Decide 的检查序列单一来源：任一深睡路径可达返回
 * LP_REFUSE_NONE，否则返回第一失败原因。NULL 快照按最保守归类
 * LP_REFUSE_NOT_STANDBY（与 Decide 的 NULL->浅睡防御一致）。
 *----------------------------------------------------------*/
LpRefuse_t LpPolicy_RefuseReason(const LpPolicySnapshot_t *snap);

/*-----------------------------------------------------------
 * IWDG 服务需求判定（纯函数）
 *
 * 输入：
 *   sw_iwdg_started —— 软件路径 IWDG 已启动（WATCHDOG_ENABLED 构建
 *                      且 Watchdog_Init() 已执行）
 *   flash_obr       —— FLASH->OBR 原始读数
 * 返回 1（需要周期服务）当且仅当：
 *   sw_iwdg_started 为真；或 OBR 的 WDG_SW 位未置位（硬件 IWDG）。
 * 需要服务时：禁止无限期 EXTI-only STOP；RTC 定时 STOP 的睡眠时长
 * 受 LP_MAX_STOP_MS 上限约束（设计 §6.7 静态安全预算）。
 *----------------------------------------------------------*/
u8 Watchdog_RequiresPeriodicService(u8 sw_iwdg_started, u32 flash_obr);

#ifdef __cplusplus
}
#endif

#endif /* __LOWPOWER_POLICY_H__ */
