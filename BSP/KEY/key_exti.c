/*
 * 按键 EXTI 唤醒通道实现
 *
 * 本文件是主机可测核心：硬件/内核访问全部经 KeyExtiIo_t 注入，
 * 目标侧适配（EXTI/NVIC 寄存器 + vTaskNotifyGiveFromISR）在
 * USER/stm32f10x_it.c。不链接任何 FreeRTOS 源也能编译运行——
 * "ISR 不延时、不 printf、不做按键判定"由链接集本身证明。
 *
 */
#include <stddef.h>     /* NULL（ARMCC 与 host gcc 均需显式包含）*/
#include "key_exti.h"

void KeyExti_Init(KeyExti_t *ke, const KeyExtiIo_t *io)
{
    if ((ke == NULL) || (io == NULL))
    {
        return;
    }

    ke->isr_entry_count = 0u;
    ke->key_irq_count = 0u;
    ke->foreign_count = 0u;

    io->line_config(io->self);   /* EXTI 线 + NVIC（幂等配置）*/
}

void KeyExti_IsrBody(KeyExti_t *ke, const KeyExtiIo_t *io)
{
    u32 pending;

    if ((ke == NULL) || (io == NULL))
    {
        return;
    }

    ke->isr_entry_count++;

    pending = io->read_pending(io->self);
    if ((pending & KEY_EXTI_LINE_MASK) != 0u)
    {
        /* 只清本线 pending：低功耗退出路径只读不清（设计 §6.6 步 4），
         * 本 ISR 是 KEY pending 的唯一清除者 */
        io->clear_pending(io->self, KEY_EXTI_LINE_MASK);
        ke->key_irq_count++;

        /* 抖动合并由通知计数语义实现：N 个边沿 -> N 次 give ->
         * 任务一次 ulTaskNotifyTake(pdTRUE) 消费全部累计值 ->
         * 一轮扫描 -> 状态机 30ms 消抖窗内只产生一次按下事件 */
        io->notify_give_from_isr(io->self);
    }
    if ((pending & ~KEY_EXTI_LINE_MASK) != 0u)
    {
        /* 观测到外来 pending 位：不清（所有权归其他 ISR，本 ISR 从不触碰），
         * 只计数——即使 KEY 位同时存在也不掩盖外来观测（诊断完整性）。
         * 外来位保持 pending，由其所属 ISR 自行清除 */
        ke->foreign_count++;
    }
}

TickType_t KeyExti_ScanWaitTicks(u8 standby, u8 key_in_progress)
{
    if ((standby != 0u) && (key_in_progress == 0u))
    {
        return portMAX_DELAY;   /* 只由按键 EXTI 通知唤醒 */
    }
    return KEY_EXTI_ACTIVE_SCAN_MS;
}
