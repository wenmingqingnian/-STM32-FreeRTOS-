/*
 * 按键 EXTI 唤醒通道
 *
 * 职责边界（设计 §6.7）：
 *   EXTI 只产生"唤醒通知"——ISR 体 = 读 pending -> 只清本线位 ->
 *   vTaskNotifyGiveFromISR；去抖、短按/长按/释放判定仍由 key.c 的
 *   状态机在任务上下文完成（KeyExti_t 不携带任何按键事件字段）。
 *
 * 唤醒线：PB1/EXTI1（KEY1）。KEY2_ENABLED=0：PB12 所有权未确认
 *（TIM1_BKIN 冲突，见 CONFIG/board_pins.h），不注册任何 KEY2 唤醒线。
 *
 * 主机可测性：硬件访问经 KeyExtiIo_t 注入（mock/freertos/
 * test_key_wakeup.c 编译未修改的本源文件；不链接任何 FreeRTOS 源——
 * "ISR 不延时、不 printf、不判按键"由 IO 形状 + 链接集双证明）。
 *
 */
#ifndef __KEY_EXTI_H
#define __KEY_EXTI_H

#include <stdint.h>
#include "sys.h"         /* u8 / u32（host: stub；目标：USER/sys.h）*/
#include "FreeRTOS.h"    /* TickType_t / portMAX_DELAY（host: stub）*/

/* 唤醒线掩码：EXTI line 1 = PB1（KEY1，活动低电平）。
 * 目标侧 line_config 配置双边沿触发：按下沿负责 STOP 唤醒并让任务
 * 在按住期间完成消抖/长按判定；释放沿让 RELEASE 事件零延迟到达。 */
#define KEY_EXTI_LINE_MASK      ((u32)0x0001u << 1)

/* EXTI ISR 的 NVIC 抢占优先级（数值越大优先级越低）。
 * 契约：6 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(3)，
 * 即本 ISR 数值优先级低于（更"慢"于）内核 syscall 屏蔽线，
 * 才允许调用 vTaskNotifyGiveFromISR（设计 §6.7；NVIC Group 4）。 */
#define KEY_EXTI_NVIC_PRIORITY  6u

#if defined(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY) && \
    (KEY_EXTI_NVIC_PRIORITY < configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)
#error "KEY EXTI 优先级数值必须 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY（FromISR API 合法区间）"
#endif

/* 活动态扫描周期：有限超时维护消抖/长按计时（状态机依赖周期扫描推进）*/
#define KEY_EXTI_ACTIVE_SCAN_MS   ((TickType_t)10u)

/*-----------------------------------------------------------
 * IO 注入接口（目标侧适配器在 USER/stm32f10x_it.c）
 *----------------------------------------------------------*/
typedef struct KeyExtiIo {
    void *self;
    /* EXTI 线 + NVIC 配置（幂等）：EXTI1 双边沿、优先级 6、EXTI1_IRQn */
    void (*line_config)(void *self);
    /* 读 EXTI->PR（唤醒源判定：只读不清）*/
    u32  (*read_pending)(void *self);
    /* EXTI->PR 写 1 清除（仅清 KEY_EXTI_LINE_MASK）*/
    void (*clear_pending)(void *self, u32 mask);
    /* vTaskNotifyGiveFromISR + portYIELD_FROM_ISR（目标适配器实现）*/
    void (*notify_give_from_isr)(void *self);
} KeyExtiIo_t;

/*-----------------------------------------------------------
 * 后端状态（KeyExti_t 不含任何按键事件字段——事件判定归 key.c）
 *----------------------------------------------------------*/
typedef struct {
    u32 isr_entry_count;   /* ISR 进入计数（诊断证据，DiagnosticTask 可读）*/
    u32 key_irq_count;     /* KEY1 边沿中断计数（含抖动边沿）*/
    u32 foreign_count;     /* 非 KEY1 pending 进入计数（不应发生：异常诊断）*/
} KeyExti_t;

/* 初始化：清零计数并经 IO 配置 EXTI 线（在 KeyScanTask 创建后调用，
 * 否则 notify 目标句柄为 NULL——目标适配器依赖任务句柄）*/
void KeyExti_Init(KeyExti_t *ke, const KeyExtiIo_t *io);

/* EXTI1_IRQHandler 的 ISR 体：读 pending -> 属于 KEY 线则清 + 计数 +
 * 通知；外来位只计数不清（所有权纪律：低功耗退出路径只读不清，
 * 本 ISR 是 KEY pending 的唯一清除者，设计 §6.6 步 4）*/
void KeyExti_IsrBody(KeyExti_t *ke, const KeyExtiIo_t *io);

/* 按键扫描阻塞策略（纯函数，app_tasks.c KeyScanTask 使用）：
 *   待机 && 无按键处理中 -> portMAX_DELAY（只由 EXTI 通知唤醒；
 *     eNoTasksWaitingTimeout 是 STOP 深睡的硬前提
 *   其余 -> KEY_EXTI_ACTIVE_SCAN_MS 有限超时（消抖/长按计时必须
 *     周期推进；活动态轮询兜底兼容无 EXTI 场景）*/
TickType_t KeyExti_ScanWaitTicks(u8 standby, u8 key_in_progress);

/* 目标侧适配器装配（USER/stm32f10x_it.c 实现）：把真实 EXTI1/NVIC 寄存器
 * 与 vTaskNotifyGiveFromISR 组装为 KeyExtiIo_t 后调用 KeyExti_Init。
 * 时机约束：必须在 KeyScanTask 创建之后调用（notify 目标句柄存在）。
 * host mock 不提供本符号（测试自带 IO 注入；未引用不参与链接）*/
void KeyExti_TargetInit(void);

#endif /* __KEY_EXTI_H */
