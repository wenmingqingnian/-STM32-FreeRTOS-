/*
 * FreeRTOS 三路 Tickless/STOP wrapper 实现
 *
 * 调用链：
 *   IdleTask -> xExpectedIdleTime(>=configEXPECTED_IDLE_TIME_BEFORE_SLEEP)
 *   -> vTaskSuspendAll -> portSUPPRESS_TICKS_AND_SLEEP(=本模块)
 *   -> PRIMASK 屏蔽 + eTaskConfirmSleepModeStatus
 *   -> LpPolicy_Decide 三路分发：
 *        LP_SHALLOW_TICKLESS -> 官方 vPortSuppressTicksAndSleep（浅睡）
 *        LP_STOP_RTC         -> RTC Alarm 定时 STOP（仅 LSE 板型可达）
 *        LP_STOP_EXTI_ONLY   -> 按键 EXTI 无限期 STOP（tick 冻结）
 *   -> xTaskResumeAll。
 *
 * 实现要点（设计 §6.4/§6.6）：
 *   - 深睡序列全程 PRIMASK 屏蔽（__disable_irq 而非 BASEPRI 临界区：
 *     后者屏蔽不了数值优先级高于 syscall 屏蔽线的中断）；
 *   - 浅睡路径调用 port.c 官方 weak vPortSuppressTicksAndSleep，不复制
 *     其 SysTick 补偿算法；
 *   - EXTI-only 路径复用 Task 12 ExtiStop 后端（进入/退出序列 host mock
 *     逐位锁定），本文件只提供目标侧 IO 适配；
 *   - RTC 路径按设计 §6.6 RTC 序列直连 rtc_timebase + 共享适配。非 LSE
 *     板型 rtc_timebase 为零硬件 stub（决策层板型门禁之外的第二道
 *     防线：stub 的 NOT_SUPPORTED -> 降级浅睡）；
 *   - 拒绝原因经 LpPolicy_RefuseReason 分类独立计数，不折叠（步骤 6）；
 *   - 所有深睡失败路径恢复 SysTick/PRIMASK 后转浅睡，不留死机路径
 *（步骤 7）；时钟恢复失败 = H 桥保持断电 + fail-safe 复位。
 *
 * 目标侧适配为纯寄存器 + CMSIS 访问（零 SPL 函数调用），不引入新链接
 * 依赖；唤醒后清 SLEEPDEEP（步骤 5）。NOT_BOARD_TESTED（设计 §8）。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 * 设计：IAP + STOP 低功耗分层设计
 */
#include "lowpower.h"            /* 自身接口（含 lowpower_policy.h/FreeRTOS.h/task.h）*/
#include "board_profile.h"       /* BOARD_C8_LSE 板型门禁 */
#include "board_pins.h"          /* MOTOR_EN 引脚（fail-safe H 桥断电）*/
#include "exti_stop_backend.h"   /* ExtiStop 后端（Task 12，进入/退出序列）*/
#include "rtc_timebase.h"        /* RTC 时间基（Task 12；非 LSE 板型为 stub）*/
#include "lowpower_math.h"       /* LpMath_*（RTC 路径 tick 换算，Task 11）*/
#include "key_exti.h"            /* KEY_EXTI_LINE_MASK（PB1 唤醒线掩码）*/
#include "watchdog.h"            /* Watchdog_ServiceRequiredNow */
#include "system_clock_resume.h" /* SystemClockResume_TargetIo（Task 12 接线）*/

/* port.c 官方 weak 实现（浅睡路径，portmacro.h 已路由本模块）*/
extern void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime);

/*-----------------------------------------------------------
 * 模块常量
 *----------------------------------------------------------*/
#define LP_EXTI17_MASK     (1uL << 17)      /* RTC Alarm 唤醒线（设计 §7）*/
#define LP_RTC_RTOFF_LOOPS 0x10000uL        /* RTC 事务 RTOFF 等待上限（防御有界）*/
#define LP_ADC_CAL_LOOPS   0x10000uL        /* ADC 校准等待上限（防御有界）*/

/*-----------------------------------------------------------
 * 模块状态（统计写入只在 IdleTask 上下文的 PRIMASK 段内：
 * PRIMASK 阻断 PendSV，单写者不可被抢占；快照读取方 DiagnosticTask
 * 走 BASEPRI 临界区防撕裂——与 DWT_SnapshotAndReset 同一惯例）
 *----------------------------------------------------------*/
static LowPower_Stats_t s_stats;                 /* 诊断统计 */
static const LowPower_Sources_t *s_sources;      /* APP 层快照源（NULL=未绑定）*/
static u8 s_bound;                               /* 快照源已绑定 */
static u8 s_armed;                               /* ExtiStop 后端已装配 */
static ExtiStopBackend_t s_extiBk;               /* EXTI-only STOP 后端上下文 */
static RtcTimebase_t s_rtcTb;                    /* RTC 时间基上下文（非 LSE 恒 stub）*/
static const SystemClockResume_Io_t *s_clkIo;    /* 公共时钟恢复 IO（两条深睡路径共用）*/
static u32 s_rtcRemainder;                       /* RTC->tick 反向换算余数累计（§6.5）*/
static u8 s_lcdBlSaved;                          /* 睡前 LCD 背光（PB6）电平 */

/*-----------------------------------------------------------
 * SysTick 适配（两条深睡路径共用；浅睡路径由官方实现自管）
 *----------------------------------------------------------*/
static void prvSysTickStop(void *self)
{
    (void)self;
    SysTick->CTRL &= (u32)~SysTick_CTRL_ENABLE_Msk;
    /* 清可能已 pending 的 SysTick（__disable_irq 与停止之间到达的那次
     * 到 NVIC pending 线——不清理则 WFI 立即返回，深睡永不发生）。
     * 深睡期间 tick 本就冻结，丢弃该次计数符合"待机态无绝对时间
     * 依赖"语义（设计 §6.3）*/
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
}

static void prvSysTickResume(void *self)
{
    (void)self;
    /* CNT 保持冻结值，恢复后从剩余计数继续完成当前 tick 周期 */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

/*-----------------------------------------------------------
 * 外设静默/恢复（设计 §6.6 步骤 4"耗电外设"+ 唤醒源纪律）
 * 静默面（本板实测使能的中断源：TIM2/TIM4/USART1/EXTI1）：
 *   TIM4 —— 1ms 速度计算触发：不停止则周期 pending 让 WFI
 *           立即返回，深睡永不发生（停止计数 + 清标志 + 清 NVIC pending）；
 *   TIM2 —— 编码器计数：电机已物理关断，防御残余旋转溢出中断唤醒；
 *   USART1 —— 非唤醒外设：RXNE 中断屏蔽（STOP 期间 APB 时钟硬件
 *           门控，此处只处理进入前 pending 竞态；DR 未读数据保留）；
 *   ADC1 —— 模拟域断电（ADON=0）；
 *   LCD 背光（PB6）—— 熄灭，唤醒后恢复睡前电平；
 *   EXTI1（按键唤醒线）—— 保持使能（唯一深睡唤醒源）。
 *----------------------------------------------------------*/
static void prvPeripheralsQuiesce(void *self)
{
    (void)self;
    /* TIM4：停止计数 + 清更新标志 + 清 NVIC pending */
    TIM4->CR1 &= (u16)~TIM_CR1_CEN;
    TIM4->SR = (u16)~TIM_SR_UIF;
    NVIC_ClearPendingIRQ(TIM4_IRQn);

    /* TIM2：同序列（编码器计数静止后防御残余旋转）*/
    TIM2->CR1 &= (u16)~TIM_CR1_CEN;
    TIM2->SR = (u16)~TIM_SR_UIF;
    NVIC_ClearPendingIRQ(TIM2_IRQn);

    /* USART1：RXNE 中断屏蔽（唤醒后重开；RXNE 已置位时重开后
     * 立即重新 pending，ISR 照常消费 DR，不丢字符）*/
    USART1->CR1 &= (u16)~USART_CR1_RXNEIE;
    NVIC_ClearPendingIRQ(USART1_IRQn);

    /* ADC1 模拟域断电（设计 §6.6 步骤 4"关闭 ADC"）*/
    ADC1->CR2 &= (u32)~ADC_CR2_ADON;

    /* LCD 背光（PB6，lcd.h LED=6）：保存电平后熄灭 */
    s_lcdBlSaved = ((GPIOB->ODR & GPIO_Pin_6) != (u32)0) ? 1u : 0u;
    GPIOB->BRR = (u32)GPIO_Pin_6;
}

static void prvPeripheralsResume(void *self)
{
    u32 guard;

    (void)self;

    /* ADC1 重新上电 + 校准（RM0008：上电后需重新校准；与 MQ2_Init
     * 同序列，纯轮询无毫秒时基依赖；有界循环防御硬件异常）*/
    ADC1->CR2 |= (u32)ADC_CR2_ADON;
    ADC1->CR2 |= (u32)ADC_CR2_RSTCAL;
    for (guard = 0u; guard < LP_ADC_CAL_LOOPS; guard++)
    {
        if ((ADC1->CR2 & ADC_CR2_RSTCAL) == (u32)0) { break; }
    }
    ADC1->CR2 |= (u32)ADC_CR2_CAL;
    for (guard = 0u; guard < LP_ADC_CAL_LOOPS; guard++)
    {
        if ((ADC1->CR2 & ADC_CR2_CAL) == (u32)0) { break; }
    }

    /* LCD 背光恢复睡前电平 */
    if (s_lcdBlSaved != 0u)
    {
        GPIOB->BSRR = (u32)GPIO_Pin_6;
    }

    /* USART1 RX 中断恢复 */
    USART1->CR1 |= (u16)USART_CR1_RXNEIE;

    /* TIM2 计数恢复（CNT 保持冻结值，从停止点继续）。
     * TIM4 不在此恢复：电机物理关断是深睡前置门槛，唤醒后仍停转，
     * TIM4 保持关闭，由 motor_start() 按电机实际启停再恢复。 */
    TIM2->CR1 |= (u16)TIM_CR1_CEN;
}

/*-----------------------------------------------------------
 * 按键 EXTI 唤醒线 / PWR STOP / WFI 适配
 *----------------------------------------------------------*/
static void prvKeyExtiWakeConfig(void *self)
{
    (void)self;
    /* 幂等确保唤醒线激活：IMR 位 + NVIC 使能。边沿配置保持
     * KeyExti_TargetInit（Task 13）的双边沿语义——按下沿负责 STOP
     * 唤醒、释放沿让 RELEASE 事件零延迟到达，此处不得改写触发方式 */
    EXTI->IMR |= KEY_EXTI_LINE_MASK;
    NVIC_EnableIRQ(EXTI1_IRQn);
}

static void prvClearKeyPending(void *self)
{
    (void)self;
    /* 睡前清旧 pending（写 1 清除）：防残留 pending 让 WFI 立即
     * 返回的 sleep race（设计 §6.6；clear 与 WFI 间新到的按键
     * 会形成新 pending 并让 WFI 立即返回——正确行为）*/
    EXTI->PR = KEY_EXTI_LINE_MASK;
}

static u32 prvReadExtiPending(void *self)
{
    (void)self;
    return EXTI->PR;    /* 只读：唤醒源判定（不清除，留给 ISR）*/
}

static void prvPwrStopModeEnter(void *self)
{
    (void)self;
    RCC->APB1ENR |= (u32)RCC_APB1ENR_PWREN;   /* PWR 接口时钟（幂等）*/
    PWR->CR |= (u32)PWR_CR_CWUF;              /* 清 WUF 唤醒标志（写 1 清除）*/
    PWR->CR &= (u32)~PWR_CR_PDDS;             /* PDDS=0：STOP 模式（非待机）*/
    PWR->CR |= (u32)PWR_CR_LPDS;              /* LPDS=1：调压器低功耗档 */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;        /* 深睡使能（WFI -> STOP）*/
}

static void prvDsbIsbWfi(void *self)
{
    (void)self;
    __DSB();
    __ISB();
    __WFI();    /* 阻塞至此：唤醒源 pending 即返回（PRIMASK 只延迟 ISR 执行）*/
    /* 唤醒后立即清 SLEEPDEEP*/
    SCB->SCR &= (u32)~SCB_SCR_SLEEPDEEP_Msk;
}

/*-----------------------------------------------------------
 * ExtiStop 后端 IO 表（Task 12 契约：进入/退出序列 mock 逐位锁定）
 *----------------------------------------------------------*/
static const ExtiStop_Io_t s_extiStopIo = {
    NULL,
    prvSysTickStop,
    prvPeripheralsQuiesce,
    prvKeyExtiWakeConfig,
    prvClearKeyPending,
    prvReadExtiPending,
    prvPwrStopModeEnter,
    prvDsbIsbWfi,
    prvPeripheralsResume,
    prvSysTickResume
};

/*-----------------------------------------------------------
 * RTC 时间基 IO 适配（仅 BOARD_C8_LSE 板型被 rtc_timebase 完整实现
 * 消费；PENDING/NO_LSE 板型 stub 不解引用本表——表仍参与编译，
 * 同一套 mock 覆盖，板型激活无需改代码）
 *
 * BKP DR1..DR10 地址映射（RM0008，中容量）：BKP_BASE+0x04+(n-1)*4；
 * RTC 事务纪律：RTOFF=1 才允许写 -> CNF 置位 -> 写寄存器 -> CNF 清零。
 *----------------------------------------------------------*/
static void RtcIo_PwrBkpEnable(void *self)
{
    (void)self;
    RCC->APB1ENR |= (u32)(RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN);
    PWR->CR |= (u32)PWR_CR_DBP;               /* 备份域写允许 */
}

static u32 RtcIo_ReadBkpDr(void *self, u32 dr_index)
{
    (void)self;
    /* 仅 DR1..DR10（签名寄存器 DR10 在范围内）*/
    return (u32)(*(__IO u16 *)(BKP_BASE + 0x04u + (dr_index - 1u) * 4u));
}

static void RtcIo_WriteBkpDr(void *self, u32 dr_index, u32 value)
{
    (void)self;
    *(__IO u16 *)(BKP_BASE + 0x04u + (dr_index - 1u) * 4u) = (u16)value;
}

static void RtcIo_BackupDomainReset(void *self)
{
    (void)self;
    RCC->BDCR |= (u32)RCC_BDCR_BDRST;         /* 备份域复位（脉冲）*/
    RCC->BDCR &= (u32)~RCC_BDCR_BDRST;
}

static void RtcIo_LseOn(void *self)
{
    (void)self;
    RCC->BDCR |= (u32)RCC_BDCR_LSEON;
}

static u8 RtcIo_LseReady(void *self)
{
    (void)self;
    return ((RCC->BDCR & RCC_BDCR_LSERDY) != (u32)0) ? (u8)1 : (u8)0;
}

static void RtcIo_RtcSelectLse(void *self)
{
    (void)self;
    /* RTCSEL=01（LSE）；仅备份域复位后允许（rtc_timebase 序列保证）*/
    RCC->BDCR |= (u32)RCC_BDCR_RTCSEL_LSE;
}

static void RtcIo_SetPrescaler(void *self, u32 prl)
{
    u32 guard;

    (void)self;
    for (guard = 0u; guard < LP_RTC_RTOFF_LOOPS; guard++)
    {
        if ((RTC->CRL & RTC_CRL_RTOFF) != (u16)0) { break; }
    }
    RTC->CRL |= (u16)RTC_CRL_CNF;             /* 进入配置模式 */
    RTC->PRLH = (u16)(prl >> 16);
    RTC->PRLL = (u16)(prl & 0xFFFFu);
    RTC->CRL &= (u16)~RTC_CRL_CNF;            /* 退出配置模式（触发写入）*/
}

static void RtcIo_WaitRtcSync(void *self)
{
    u32 guard;

    (void)self;
    RTC->CRL &= (u16)~RTC_CRL_RSF;            /* 清同步标志 */
    for (guard = 0u; guard < LP_RTC_RTOFF_LOOPS; guard++)
    {
        if ((RTC->CRL & RTC_CRL_RSF) != (u16)0) { break; }
    }
}

static void RtcIo_AlarmExtiNvicInit(void *self)
{
    (void)self;
    EXTI->IMR |= LP_EXTI17_MASK;              /* EXTI17 中断模式 */
    EXTI->RTSR |= LP_EXTI17_MASK;             /* 上升沿（ALRF 置位沿）*/
    EXTI->FTSR &= ~LP_EXTI17_MASK;
    EXTI->PR = LP_EXTI17_MASK;                /* 清残留 pending */
    NVIC_EnableIRQ(RTCAlarm_IRQn);
}

static u16 RtcIo_ReadCntHigh(void *self)
{
    (void)self;
    return (u16)RTC->CNTH;
}

static u16 RtcIo_ReadCntLow(void *self)
{
    (void)self;
    return (u16)RTC->CNTL;
}

static void RtcIo_ClearAlarmFlags(void *self)
{
    (void)self;
    RTC->CRL &= (u16)~RTC_CRL_ALRF;           /* ALRF 写 0 清除（RM0008）*/
    EXTI->PR = LP_EXTI17_MASK;                /* EXTI17 pending 写 1 清除 */
}

static void RtcIo_SetAlarm(void *self, u32 counter)
{
    u32 guard;

    (void)self;
    for (guard = 0u; guard < LP_RTC_RTOFF_LOOPS; guard++)
    {
        if ((RTC->CRL & RTC_CRL_RTOFF) != (u16)0) { break; }
    }
    RTC->CRL |= (u16)RTC_CRL_CNF;
    RTC->ALRH = (u16)(counter >> 16);
    RTC->ALRL = (u16)(counter & 0xFFFFu);
    RTC->CRL &= (u16)~RTC_CRL_CNF;
}

static void RtcIo_CancelAlarm(void *self)
{
    u32 guard;

    (void)self;
    /* RTC 闹钟比较无法禁止：写不可达值（0xFFFFFFFF，约 48.5 天外）*/
    for (guard = 0u; guard < LP_RTC_RTOFF_LOOPS; guard++)
    {
        if ((RTC->CRL & RTC_CRL_RTOFF) != (u16)0) { break; }
    }
    RTC->CRL |= (u16)RTC_CRL_CNF;
    RTC->ALRH = (u16)0xFFFF;
    RTC->ALRL = (u16)0xFFFF;
    RTC->CRL &= (u16)~RTC_CRL_CNF;
}

static void RtcIo_WaitPoll(void *self)
{
    (void)self;
    /* LSE 启动轮询间隔约 1ms：本表只在任务上下文（LowPower_TargetInit
     * <- StartTask）被 RtcTimebase_Init 消费，vTaskDelay 合法 */
    vTaskDelay(pdMS_TO_TICKS(1));
}

static const RtcTimebase_Io_t s_rtcTbIo = {
    NULL,
    RtcIo_PwrBkpEnable,
    RtcIo_ReadBkpDr,
    RtcIo_WriteBkpDr,
    RtcIo_BackupDomainReset,
    RtcIo_LseOn,
    RtcIo_LseReady,
    RtcIo_RtcSelectLse,
    RtcIo_SetPrescaler,
    RtcIo_WaitRtcSync,
    RtcIo_AlarmExtiNvicInit,
    RtcIo_ReadCntHigh,
    RtcIo_ReadCntLow,
    RtcIo_ClearAlarmFlags,
    RtcIo_SetAlarm,
    RtcIo_CancelAlarm,
    RtcIo_WaitPoll
};

/*-----------------------------------------------------------
 * fail-safe 复位（时钟恢复失败，设计 §6.6）
 * 已在 PRIMASK 下：H 桥断电重申（进入门槛已确认关断，纯防御）后
 * 整机复位。禁止 printf：USART 处于静默态，8MHz HSI 下不可信。
 *----------------------------------------------------------*/
static void prvFailSafeReset(void)
{
    TIM1->CCER &= (u16)~((u16)TIM_CCER_CC1E | (u16)TIM_CCER_CC1NE);
    TIM1->CCR1 = (u16)0;
    MOTOR_EN_GPIO_PORT->BRR = (u32)MOTOR_EN_GPIO_PIN;
    s_stats.clk_resume_fail++;   /* 复位前累加（RAM 供调试器查验）*/
    NVIC_SystemReset();          /* 不返回 */
}

/*-----------------------------------------------------------
 * 时钟健康检查（快照公共门槛字段 clock_healthy 的组装）
 * SYSCLK 仍来自 PLL（SWS=10）且 HSE 就绪；任一不成立视为降级
 *（深睡唤醒时钟恢复的前提不成立，拒绝深睡）。
 *----------------------------------------------------------*/
static u8 prvClockHealthy(void)
{
    if ((RCC->CFGR & RCC_CFGR_SWS) != (u32)0x00000008) { return 0u; }
    if ((RCC->CR & RCC_CR_HSERDY) == (u32)0) { return 0u; }
    return 1u;
}

/*-----------------------------------------------------------
 * 快照组装（lowpower_policy.h L117-127 责任表的目标侧落地）
 * 单回调 NULL -> 对应门槛不满足（保守拒绝深睡）。
 *----------------------------------------------------------*/
static void prvAssembleSnapshot(LpPolicySnapshot_t *snap,
                                TickType_t expected_idle,
                                u8 no_tasks_waiting)
{
#if (BOARD_C8_LSE != 0)
    snap->board = LP_BOARD_C8_LSE;
#elif (BOARD_C8_NO_LSE != 0)
    snap->board = LP_BOARD_C8_NO_LSE;
#else
    snap->board = LP_BOARD_PENDING;
#endif

    snap->standby_mode = ((s_sources->standby_mode != NULL) &&
                          (s_sources->standby_mode(s_sources->self) != 0u))
                         ? 1u : 0u;
    snap->motor_physically_off = ((s_sources->motor_physically_off != NULL) &&
                                  (s_sources->motor_physically_off(s_sources->self) != 0u))
                                 ? 1u : 0u;
    snap->buzzer_idle = ((s_sources->buzzer_idle != NULL) &&
                         (s_sources->buzzer_idle(s_sources->self) != 0u))
                        ? 1u : 0u;
    snap->iap_idle = ((s_sources->iap_idle != NULL) &&
                      (s_sources->iap_idle(s_sources->self) != 0u))
                     ? 1u : 0u;
    snap->uart_dma_idle = ((s_sources->uart_dma_idle != NULL) &&
                           (s_sources->uart_dma_idle(s_sources->self) != 0u))
                          ? 1u : 0u;
    snap->key_exti_available = ((s_sources->key_exti_available != NULL) &&
                                (s_sources->key_exti_available(s_sources->self) != 0u))
                               ? 1u : 0u;

    snap->clock_healthy = prvClockHealthy();
    snap->rtc_ready = (RtcTimebase_IsReady(&s_rtcTb) != 0u) ? 1u : 0u;
    snap->no_tasks_waiting_timeout = no_tasks_waiting;
    snap->watchdog_requires_service =
        (Watchdog_ServiceRequiredNow() != 0u) ? 1u : 0u;
    snap->expected_idle_ticks = (u32)expected_idle;
}

/*-----------------------------------------------------------
 * EXTI-only 无限期 STOP 路径（本板唯一深睡路径
 * 返回 0=STOP 完整进出（tick 冻结不补偿）；1=降级浅睡。
 * 前提（决策层确认 + 后端防御再校验）：eNoTasksWaitingTimeout 且
 * IWDG 无需周期服务——tick 冻结不会错过内核 deadline，无喂狗义务。
 *----------------------------------------------------------*/
static u8 prvStopExtiOnlyPath(u8 no_tasks_waiting, u8 wd_requires)
{
    if (ExtiStop_Enter(&s_extiBk, no_tasks_waiting, wd_requires) != EXTI_STOP_OK)
    {
        return 1u;    /* 前提防御失败：零 IO 副作用 -> 浅睡 */
    }

    if (ExtiStop_Exit(&s_extiBk) != EXTI_STOP_OK)
    {
        /* 失败即时钟恢复失败（其余状态码在 Enter 成功后不可达）：
         * 外设/SysTick 保持关闭，不允许在 8MHz HSI 下继续使用按
         * 72MHz 配置的外设 -> fail-safe 复位（设计 §6.6）*/
        prvFailSafeReset();
    }

    /* 唤醒源分类统计（退出顺序 1 已判定并保存）*/
    switch (ExtiStop_GetWakeSource(&s_extiBk))
    {
        case EXTI_STOP_WAKE_KEY:
            s_stats.wake_key++;
            break;
        case EXTI_STOP_WAKE_OTHER:
            s_stats.wake_other++;
            break;
        default:
            s_stats.wake_none++;   /* 非 EXTI 唤醒（调试器等）*/
            break;
    }
    s_stats.exti_stop_count++;
    return 0u;
}

/*-----------------------------------------------------------
 * RTC Alarm 定时 STOP 路径（仅 LSE 板型可达，设计 §6.5/§6.6）
 * 返回 0=STOP 完整进出（tick 已按 RTC 差值补偿）；1=降级浅睡。
 * 进入失败恢复 SysTick 后降级（不留死机路径，步骤 7）。
 *----------------------------------------------------------*/
static u8 prvStopRtcPath(TickType_t expected_idle)
{
    u32 budget_rticks;
    u32 rtc_start = 0u;
    u32 rtc_end = 0u;
    u32 elapsed;
    u32 clamped;
    u32 remainder;
    u32 pending;
    u8 clamp_flag = 0u;
    ClkResume_Status_t clk_st;

    /* Alarm 预算：min(expected_idle - wake_guard, MAX_STOP_MS)（§6.5）*/
    budget_rticks = LpMath_AlarmBudgetRticks((u32)expected_idle);
    if (budget_rticks == 0u)
    {
        /* 不可达（决策层已要求 idle > guard）；防御降级 */
        s_stats.rtc_degrade_count++;
        return 1u;
    }

    /* 进入序列（设计 §6.6 RTC 定时 STOP 进入，步骤 1-7）*/
    prvSysTickStop(NULL);                                /* 1. 停 SysTick */
    if (RtcTimebase_ReadCounterStable(&s_rtcTb, &rtc_start) != RTC_TB_OK)
    {
        prvSysTickResume(NULL);                          /* 2. 稳定读取失败 */
        s_stats.rtc_degrade_count++;
        return 1u;
    }
    if (RtcTimebase_SetAlarm(&s_rtcTb, rtc_start + budget_rticks) != RTC_TB_OK)
    {
        prvSysTickResume(NULL);                          /* 3. 闹钟设置失败 */
        s_stats.rtc_degrade_count++;
        return 1u;
    }
    prvPeripheralsQuiesce(NULL);                         /* 4. 外设静默 */
    prvPwrStopModeEnter(NULL);                           /* 5. PDDS=0 LPDS=1 */
    prvKeyExtiWakeConfig(NULL);                          /* 6. 保留按键 EXTI */
    prvClearKeyPending(NULL);                            /*    清睡前旧 pending */
    prvDsbIsbWfi(NULL);                                  /* 7. DSB/ISB WFI（阻塞）*/

    /* --- 唤醒后（PRIMASK 仍屏蔽，设计 §6.6 RTC 退出，步骤 1-6）--- */
    pending = prvReadExtiPending(NULL);                  /* 1. 唤醒源判定 */
    if ((pending & LP_EXTI17_MASK) != 0u)
    {
        s_stats.wake_alarm++;    /* Alarm 线优先（本路径的预期唤醒源）*/
    }
    else if ((pending & KEY_EXTI_LINE_MASK) != 0u)
    {
        s_stats.wake_key++;
    }
    else if (pending != 0u)
    {
        s_stats.wake_other++;
    }
    else
    {
        s_stats.wake_none++;
    }

    clk_st = SystemClock_RestoreAfterStop(s_clkIo);      /* 2/3. 72MHz 恢复 */
    if (clk_st != CLK_RESUME_OK)
    {
        prvFailSafeReset();      /* 不返回 */
    }

    if (RtcTimebase_ReadCounterStable(&s_rtcTb, &rtc_end) != RTC_TB_OK)
    {
        rtc_end = rtc_start;     /* 读取撕裂：差值退化为 0（不补偿）*/
    }
    (void)RtcTimebase_CancelAlarm(&s_rtcTb);             /* 4. 取消闹钟+清标志 */

    prvPeripheralsResume(NULL);                          /* 5. 恢复外设 */
    prvSysTickResume(NULL);                              /*    恢复 SysTick */

    /* 6. tick 补偿（§6.5）：无符号差值 + 余数累计反向换算 + clamp
     *（步进不得越过下一个解阻塞点；越界计数不静默）*/
    elapsed = LpMath_RticksToElapsed(LpMath_RtcDelta(rtc_end, rtc_start),
                                     s_rtcRemainder, &remainder);
    s_rtcRemainder = remainder;
    clamped = LpMath_ClampElapsed(elapsed, (u32)expected_idle, &clamp_flag);
    if (clamp_flag != 0u)
    {
        s_stats.tick_clamp_count++;
    }
    vTaskStepTick(clamped);

    s_stats.rtc_stop_count++;
    return 0u;
}

/*-----------------------------------------------------------
 * 公共 API
 *----------------------------------------------------------*/
void LowPower_BindSources(const LowPower_Sources_t *sources)
{
    if (sources == NULL)
    {
        return;    /* NULL 拒绝：保持未绑定（全部走官方浅睡，绝不深睡）*/
    }
    s_sources = sources;
    s_bound = 1u;
}

void LowPower_TargetInit(void)
{
    /* ExtiStop 后端装配：IO 适配 + 公共时钟恢复 IO（Task 12 接线）+
     * PB1 按键唤醒线掩码（Task 13 单一来源）*/
    s_clkIo = SystemClockResume_TargetIo();
    if (ExtiStop_Init(&s_extiBk, &s_extiStopIo, s_clkIo,
                      KEY_EXTI_LINE_MASK) == EXTI_STOP_OK)
    {
        s_armed = 1u;
    }

    /* RTC 时间基初始化（板型门禁在 rtc_timebase.c 内部：LSE 完整
     * 初始化，其余板型 stub 返回 NOT_SUPPORTED -> ready 恒 0，
     * 决策层自然避开 RTC 路径——上层无需 #if 包裹）*/
    (void)RtcTimebase_Init(&s_rtcTb, &s_rtcTbIo);
    s_rtcRemainder = 0u;
}

void LowPower_SuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    LpPolicySnapshot_t snap;
    LpDecision_t decision;
    LpRefuse_t refuse;
    eSleepModeStatus confirm;
    u8 no_tasks_waiting;

    /* 快照源未绑定或后端未装配：对系统状态一无所知，绝不深睡 */
    if ((s_bound == 0u) || (s_armed == 0u))
    {
        s_stats.shallow_count++;
        vPortSuppressTicksAndSleep(xExpectedIdleTime);
        return;
    }

    /* PRIMASK 屏蔽（设计 §6.4：深睡入口二次检查。BASEPRI 临界区
     * 屏蔽不了高优先级中断，深睡序列必须 PRIMASK）*/
    __disable_irq();
    __DSB();
    __ISB();

    /* RTOS 层确认（设计 §6.4 步骤 1）：eAbortSleep -> 立即放弃 */
    confirm = eTaskConfirmSleepModeStatus();
    if (confirm == eAbortSleep)
    {
        __enable_irq();
        s_stats.shallow_count++;
        vPortSuppressTicksAndSleep(xExpectedIdleTime);
        return;
    }
    no_tasks_waiting = (confirm == eNoTasksWaitingTimeout) ? 1u : 0u;

    /* 快照组装 -> 拒绝原因分类计数（不折叠）-> 三态决策 */
    prvAssembleSnapshot(&snap, xExpectedIdleTime, no_tasks_waiting);
    refuse = LpPolicy_RefuseReason(&snap);
    if (refuse != LP_REFUSE_NONE)
    {
        s_stats.refuse_counts[refuse]++;
    }
    decision = LpPolicy_Decide(&snap);

    if (decision == LP_STOP_RTC)
    {
        if (prvStopRtcPath(xExpectedIdleTime) == 0u)
        {
            __enable_irq();    /* 成功：pending 的唤醒 ISR 到此执行 */
            return;
        }
        /* RTC 序列失败 -> 降级浅睡（degrade 计数在路径内累加）*/
    }
    else if (decision == LP_STOP_EXTI_ONLY)
    {
        if (prvStopExtiOnlyPath(no_tasks_waiting,
                                snap.watchdog_requires_service) == 0u)
        {
            __enable_irq();    /* 成功：按键 ISR 通知 KeyScanTask */
            return;
        }
        /* 前提防御失败（决策与后端间状态漂移，罕见）-> 浅睡 */
    }

    /* 浅睡兜底：官方 SysTick Tickless（不复制算法，步骤 2）*/
    __enable_irq();
    s_stats.shallow_count++;
    vPortSuppressTicksAndSleep(xExpectedIdleTime);
}

void LowPower_SnapshotStats(LowPower_Stats_t *out)
{
    if (out == NULL)
    {
        return;
    }
    /* BASEPRI 临界区防撕裂快照（写者处于 PRIMASK 段不可被抢占，
     * 读者只需防自身复制中途被抢占——DWT_SnapshotAndReset 惯例）*/
    taskENTER_CRITICAL();
    *out = s_stats;
    taskEXIT_CRITICAL();
}
