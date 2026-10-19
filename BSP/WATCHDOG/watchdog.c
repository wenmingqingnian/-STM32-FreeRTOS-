/*
 * 独立看门狗模块实现
 *
 * 标准库 IWDG 序列（RM0008 规定的解锁顺序）：
 *   1. KR = 0x5555 解锁 PR/RLR 写权限；
 *   2. 写 PR（分频）与 RLR（重载值）；
 *   3. KR = 0xAAAA 重载计数器（喂狗）；
 *   4. KR = 0xCCCC 启动看门狗（启动后不可停止，只能靠复位）。
 *
 */
#include "watchdog.h"
#include "lowpower_policy.h"   /* Watchdog_RequiresPeriodicService 纯函数（Task 11）*/

/*-----------------------------------------------------------
 * IWDG 周期服务需求查询（Task 14，设计 §6.7）
 * 条件外定义：软件 IWDG 关闭（WATCHDOG_ENABLED=0）时 option byte
 * 仍可能选择硬件 IWDG（OBR.WDG_SW=0，复位即运行不可关闭）——
 * 此时同样必须周期喂狗并禁止无限期 STOP。
 *----------------------------------------------------------*/
u8 Watchdog_ServiceRequiredNow(void)
{
    u8 swStarted;

#if defined(WATCHDOG_ENABLED) && WATCHDOG_ENABLED
    swStarted = 1u;     /* Watchdog_Init() 已在启动路径调用（软件 IWDG 运行中）*/
#else
    swStarted = 0u;
#endif
    /* OBR.WDG_SW 位语义（RM0008 §2.5）：=1 软件看门狗，=0 硬件看门狗 */
    return Watchdog_RequiresPeriodicService(swStarted, FLASH->OBR);
}

#if defined(WATCHDOG_ENABLED) && WATCHDOG_ENABLED

#include "stm32f10x_iwdg.h"
#include "stm32f10x_dbgmcu.h"

/*-----------------------------------------------------------
 * 初始化并启动 IWDG
 *----------------------------------------------------------*/
void Watchdog_Init(void)
{
    /* 调试冻结：内核因断点暂停时 IWDG 计数同步冻结，
     * 防止单步调试期间看门狗超时误复位（调试构建友好性） */
    DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);

    /* 解锁 PR/RLR 写权限 */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    /* 配置超时：LSI 40kHz / 64 = 1.6ms 计数周期，重载 2500 → 4.0s */
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    IWDG_SetReload((u16)WATCHDOG_RELOAD);

    /* 重载计数器并启动（启动后硬件上不可停止） */
    IWDG_ReloadCounter();
    IWDG_Enable();
}

/*-----------------------------------------------------------
 * 喂狗（仅 HealthTask 判定心跳齐全后调用）
 *----------------------------------------------------------*/
void Watchdog_Feed(void)
{
    IWDG_ReloadCounter();
}

#endif /* WATCHDOG_ENABLED */
