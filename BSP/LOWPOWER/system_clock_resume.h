/*
 * STOP 唤醒后的 72 MHz 公共时钟恢复
 *
 * 职责（RTC 定时 STOP 与 EXTI-only STOP 共用，设计 §6.6 退出顺序 2/3）：
 *   STOP 唤醒后硬件自动以 HSI 为系统时钟、PLL 已被硬件关闭。本模块按
 *   SetSysClockTo72() 的同等配置（Flash 2WS+预取、AHB/1 APB2/1 APB1/2、
 *   PLL = HSE×9、SYSCLK=PLL）恢复时钟，但每段等待都有界：
 *     HSE 就绪等待、PLL 就绪等待、SWS 切换确认，任一超时立即返回
 *     对应错误码，后续配置一概不执行——绝不在 HSI 8 MHz 下假装恢复
 *     成功（调用方据此保持 H 桥关闭并 fail-safe reset，设计 §6.6）。
 *
 * 与完整 SystemInit() 的区别（设计 §6.6 明确要求）：
 *   不重置全局时钟状态（不写 RCC 复位序列、不动 SCB->VTOR）——
 *   SystemInit 只允许复位后调用一次。
 *
 * 纯逻辑 + IO 注入（项目惯例：寄存器访问封装以便故障注入测试）：
 *   本模块不直接访问任何寄存器。目标侧适配（RCC/FLASH +
 *   SystemCoreClockUpdate）在 USER/system_stm32f10x.c 提供（Task 12
 *   接线）；host mock（test_exti_stop_backend.c）注入脚本化 IO，
 *   覆盖成功序列/三种超时/边界。
 *
 * 顺序契约（test_exti_stop_backend.c s6 逐位锁定）：
 *   hse_on -> [有界等待 hse_ready] -> flash_latency_72m_config ->
 *   bus_prescalers_72m_config -> pll_hse_x9_enable ->
 *   [有界等待 pll_ready] -> select_pll_sysclk ->
 *   [有界等待 sysclk_is_pll] -> system_core_clock_update
 *
 *   Flash latency 在切回 72 MHz 之前设置（低频下设高延迟安全）；
 *   总线分频在 PLL 使能前恢复；SystemCoreClockUpdate 最后执行。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#ifndef __SYSTEM_CLOCK_RESUME_H__
#define __SYSTEM_CLOCK_RESUME_H__

#include <stddef.h>            /* NULL — ARMCC 与 host gcc 通用 */
#include "sys.h"               /* u8 */

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * 状态码（设计 §6.6：时钟恢复失败 = 调用方 fail-safe 依据）
 *----------------------------------------------------------*/
typedef enum {
    CLK_RESUME_OK = 0,
    CLK_RESUME_ERR_PARAM,          /* NULL io */
    CLK_RESUME_ERR_HSE_TIMEOUT,    /* HSE 未在有界窗口内就绪 */
    CLK_RESUME_ERR_PLL_TIMEOUT,    /* PLL 未在有界窗口内就绪 */
    CLK_RESUME_ERR_SWITCH_TIMEOUT  /* SWS 未在有界窗口内切到 PLL */
} ClkResume_Status_t;

/*-----------------------------------------------------------
 * 超时策略常量
 *
 * CLK_RESUME_TIMEOUT_LOOPS：三段等待（HSERDY/PLLRDY/SWS）共用的循环
 *   上限，量级对齐 stm32f10x.h 的 HSE_STARTUP_TIMEOUT(0x0500)。
 *   注意：STOP 唤醒后 SysTick 未恢复，wait_poll 没有毫秒时基，
 *   目标侧退化为纯循环计数（与 SystemInit 的 do-while 同模式），
 *   这是"循环次数上限"而非"毫秒数"。
 *----------------------------------------------------------*/
#define CLK_RESUME_TIMEOUT_LOOPS   0x0500uL

/*-----------------------------------------------------------
 * 时钟恢复后端接口（依赖注入）
 *
 * 目标侧适配（USER/system_stm32f10x.c Task 12 接线）：
 *   hse_on                   : RCC->CR |= HSEON
 *   hse_ready                : (RCC->CR & HSERDY) != 0
 *   flash_latency_72m_config : 预取使能 + 2 wait states（72MHz 档）
 *   bus_prescalers_72m_config: HPRE=DIV1, PPRE2=DIV1, PPRE1=DIV2
 *   pll_hse_x9_enable        : PLLSRC=HSE, PLLMUL=×9, PLLON
 *（工程为 STM32F10X_MD 非 CL：无 PLL2/
 *                              PREDIV1 分支）
 *   pll_ready                : (RCC->CR & PLLRDY) != 0
 *   select_pll_sysclk        : SW=PLL（先清后置）
 *   sysclk_is_pll            : (RCC->CFGR & SWS) == 0x08
 *   system_core_clock_update : SystemCoreClockUpdate()
 *   wait_poll                : 轮询间隔（目标侧空——SysTick 未恢复）
 *----------------------------------------------------------*/
typedef struct {
    void *self;
    void (*hse_on)(void *self);
    u8   (*hse_ready)(void *self);
    void (*flash_latency_72m_config)(void *self);
    void (*bus_prescalers_72m_config)(void *self);
    void (*pll_hse_x9_enable)(void *self);
    u8   (*pll_ready)(void *self);
    void (*select_pll_sysclk)(void *self);
    u8   (*sysclk_is_pll)(void *self);
    void (*system_core_clock_update)(void *self);
    void (*wait_poll)(void *self);
} SystemClockResume_Io_t;

/*-----------------------------------------------------------
 * STOP 唤醒后的 72 MHz 时钟恢复（无状态纯序列，IO 注入）
 *
 * 返回 CLK_RESUME_OK 当且仅当三段等待全部在有界窗口内完成，
 * 且 SystemCoreClockUpdate 已执行；任何超时都在"尚未执行的后续
 * 配置"之前立即返回（fail-safe：HSI 下不得恢复按 72MHz 配置的
 * PWM/UART/SysTick，设计 §6.6）。
 *----------------------------------------------------------*/
ClkResume_Status_t SystemClock_RestoreAfterStop(const SystemClockResume_Io_t *io);

/*-----------------------------------------------------------
 * 目标侧 IO 适配访问
 *
 * 实现位于 USER/system_stm32f10x.c（直接寄存器访问，配置链与
 * SetSysClockTo72() 一致）。低功耗 wrapper（Task 15）经本函数取得
 * IO 表后交给 ExtiStop_Exit / RTC STOP 退出路径调用。
 * host mock 不链接该目标文件（不调用本函数），"目标适配不进
 * host 测试链接集"由链接集本身证明。
 *----------------------------------------------------------*/
const SystemClockResume_Io_t *SystemClockResume_TargetIo(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_CLOCK_RESUME_H__ */
