/*
 * 板型感知的低功耗深睡决策实现
 *
 * 实现说明：
 *   - 纯函数：只读 LpPolicySnapshot_t，无任何全局变量、寄存器或
 *     RTOS API 访问；硬件层组装快照的
 *     接线属 Task 12 wrapper 职责；
 *   - 决策顺序严格对应设计 §6.4：公共 STOP 门槛 -> RTC 分支 ->
 *     EXTI-only 分支 -> 浅睡兜底；
 *   - 板型硬门禁：LP_STOP_RTC 仅 LP_BOARD_C8_LSE 可达——即使快照
 *     组装错误注入了 rtc_ready，NO_LSE/PENDING 也不会走 RTC 路径
 *（board_profile.h Task 1 门禁的决策面复查）；
 *   - OBR.WDG_SW 位语义（RM0008 §2.5/9.2）：=1 软件看门狗（IWDG
 *     不自动启动），=0 硬件看门狗（复位即运行、不可关闭）。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#include "lowpower_policy.h"

/*-----------------------------------------------------------
 * 内部检查序列（Decide 与 RefuseReason 的单一来源：
 * 三个静态 helper 按设计 §6.4 顺序逐段判定，两个公共入口共享，
 * 保证"决策"与"拒绝原因分类"永不漂移。
 *----------------------------------------------------------*/

/* 公共 STOP 门槛失败归类（设计 §6.4 步骤 2-5，按字段顺序） */
static LpRefuse_t prvCommonGateRefusal(const LpPolicySnapshot_t *snap)
{
    if (snap->standby_mode == 0u)
    {
        return LP_REFUSE_NOT_STANDBY;
    }
    if (snap->motor_physically_off == 0u)
    {
        return LP_REFUSE_MOTOR_ON;
    }
    if (snap->buzzer_idle == 0u)
    {
        return LP_REFUSE_BUZZER_BUSY;
    }
    if (snap->iap_idle == 0u)
    {
        return LP_REFUSE_IAP_BUSY;
    }
    if (snap->uart_dma_idle == 0u)
    {
        return LP_REFUSE_UART_BUSY;
    }
    if (snap->clock_healthy == 0u)
    {
        return LP_REFUSE_CLOCK_UNHEALTHY;
    }
    return LP_REFUSE_NONE;
}

/* RTC 定时 STOP 路径可达（设计 §6.4 步骤 6，优先于 EXTI-only）：
 * 仅 BOARD_C8_LSE + RTC ready + 预期空闲时间 > 唤醒保护。
 * 睡眠时长由 LP_MAX_STOP_MS 上限约束（Alarm 预算，lowpower_math），
 * IWDG 需要服务时该上限保证监督窗口内醒来（设计 §6.7）。 */
static u8 prvRtcPathReachable(const LpPolicySnapshot_t *snap)
{
    return (u8)((snap->board == LP_BOARD_C8_LSE) &&
                (snap->rtc_ready != 0u) &&
                (snap->expected_idle_ticks > LP_WAKE_GUARD_TICKS));
}

/* EXTI-only 无限期 STOP 前提失败归类（设计 §6.4 步骤 7 / §6.3）：
 * 无任务等待超时 + IWDG 无需周期服务 + 至少一个有效按键 EXTI。
 * 板型不限（不依赖 LSE）；tick 冻结在无 deadline 场景安全。 */
static LpRefuse_t prvExtiPreconditionRefusal(const LpPolicySnapshot_t *snap)
{
    if (snap->no_tasks_waiting_timeout == 0u)
    {
        return LP_REFUSE_TASKS_WAITING;
    }
    if (snap->watchdog_requires_service != 0u)
    {
        return LP_REFUSE_IWDG_SERVICE;
    }
    if (snap->key_exti_available == 0u)
    {
        return LP_REFUSE_NO_EXTI;
    }
    return LP_REFUSE_NONE;
}

LpDecision_t LpPolicy_Decide(const LpPolicySnapshot_t *snap)
{
    if (snap == NULL)
    {
        return LP_SHALLOW_TICKLESS;   /* 防御：无快照不深睡 */
    }

    /* 公共 STOP 门槛任一不满足 -> 官方浅 Tickless（设计 §6.4 步骤 2-5）*/
    if (prvCommonGateRefusal(snap) != LP_REFUSE_NONE)
    {
        return LP_SHALLOW_TICKLESS;
    }

    /* RTC 定时 STOP 优先于 EXTI-only（设计 §6.4 步骤 6）*/
    if (prvRtcPathReachable(snap) != 0u)
    {
        return LP_STOP_RTC;
    }

    /* EXTI-only 前提全满足才允许（设计 §6.4 步骤 7）*/
    if (prvExtiPreconditionRefusal(snap) == LP_REFUSE_NONE)
    {
        return LP_STOP_EXTI_ONLY;
    }

    return LP_SHALLOW_TICKLESS;
}

LpRefuse_t LpPolicy_RefuseReason(const LpPolicySnapshot_t *snap)
{
    LpRefuse_t gate;

    if (snap == NULL)
    {
        /* 防御：无快照无法确认状态，按最保守归类
         *（与 Decide 的 NULL -> 浅睡兜底一致）*/
        return LP_REFUSE_NOT_STANDBY;
    }

    gate = prvCommonGateRefusal(snap);
    if (gate != LP_REFUSE_NONE)
    {
        return gate;
    }

    /* RTC 路径可达即无拒绝（RTC 不可用只是降级到 EXTI 候选）*/
    if (prvRtcPathReachable(snap) != 0u)
    {
        return LP_REFUSE_NONE;
    }

    /* RTC 不可达时由 EXTI-only 前提给出拒绝原因 */
    return prvExtiPreconditionRefusal(snap);
}

u8 Watchdog_RequiresPeriodicService(u8 sw_iwdg_started, u32 flash_obr)
{
    /* 软件 IWDG 已启动（WATCHDOG_ENABLED 构建且已 Init）*/
    if (sw_iwdg_started != 0u)
    {
        return 1u;
    }

    /* 硬件 IWDG：OBR.WDG_SW 位未置位 -> 复位后自动运行，不可关闭 */
    if ((flash_obr & LP_FLASH_OBR_WDG_SW) == 0u)
    {
        return 1u;
    }

    /* 软件看门狗未启动且 option byte 未选硬件看门狗 -> 无需服务 */
    return 0u;
}
