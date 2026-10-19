/*
 * Boot UART DMA 环形接收
 *
 * 实现要点：
 *   - DMA1_Channel5 循环模式（DMA_Mode_Circular）：NDTR 到 0 硬件自动
 *     重装，接收永不停止，无需 ISR 重装 NDTR（旧 Normal 模式 + 重装在
 *     Flash 擦写阻塞期间会丢数据）；
 *   - USART1 只开 IDLE + 错误中断（RXNE 不开，数据走 DMA）；F1 清 IDLE
 *     标志按参考手册序列：读 SR 再读 DR；ORE 同样读 SR+DR 清除；
 *   - 消费者/生产者绝对位置只在主循环上下文访问（单读者单写者，
 *     消费者只有主循环写，生产者只有 Poll() 写，二者都在主循环执行），
 *     不需要临界区。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#include <stddef.h>   /* NULL — ARMCC 与 host gcc 通用 */
#include "boot_uart_dma.h"

#define BOOT_RING_MASK   (BOOT_UART_DMA_RING_SIZE - 1u)

/*-----------------------------------------------------------
 * 环形缓冲与统计（ISR 与主循环共享的只有 s_idleCount/s_errCount/s_tcCount）
 *----------------------------------------------------------*/
static u8 s_ring[BOOT_UART_DMA_RING_SIZE] __attribute__((aligned(4)));

static volatile u32 s_idleCount = 0u;   /* USART IDLE 中断次数 */
static volatile u32 s_errCount  = 0u;   /* 接收错误次数 */
static volatile u32 s_tcCount   = 0u;   /* DMA 环绕（传输完成）次数 */

/* 生产者/消费者状态（仅主循环上下文访问） */
static u32 s_prodAbs = 0u;   /* 生产者绝对位置（字节计数） */
static u16 s_prodPos = 0u;   /* 生产者环内位置（上次 Poll 时的快照） */
static u32 s_consAbs = 0u;   /* 消费者绝对位置 */
static u8  s_overrun = 0u;   /* 粘滞溢出标志 */

void Boot_UartDma_Init(void)
{
    GPIO_InitTypeDef gpioInit;
    USART_InitTypeDef uartInit;
    DMA_InitTypeDef dmaInit;
    NVIC_InitTypeDef nvicInit;

    /* 时钟：USART1、GPIOA（APB2）与 DMA1（AHB） */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* PA9 = USART1_TX：复用推挽输出 */
    gpioInit.GPIO_Pin   = GPIO_Pin_9;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInit.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpioInit);

    /* PA10 = USART1_RX：浮空输入 */
    gpioInit.GPIO_Pin  = GPIO_Pin_10;
    gpioInit.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpioInit);

    /* NVIC：USART1（IDLE/错误）与 DMA1 通道5（环绕/错误）。
     * Boot 裸机无 RTOS 抢占优先级约束，分组沿用复位默认（全部抢占 0）。 */
    nvicInit.NVIC_IRQChannel                    = USART1_IRQn;
    nvicInit.NVIC_IRQChannelPreemptionPriority  = 0u;
    nvicInit.NVIC_IRQChannelSubPriority         = 0u;
    nvicInit.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Init(&nvicInit);

    nvicInit.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    NVIC_Init(&nvicInit);

    /* USART1 参数：115200-8N1，无流控，收发使能；RXNE 中断不开（走 DMA），
     * 只开 IDLE（帧间隔证据）与错误中断（计数） */
    uartInit.USART_BaudRate            = BOOT_UART_BAUDRATE;
    uartInit.USART_WordLength          = USART_WordLength_8b;
    uartInit.USART_StopBits            = USART_StopBits_1;
    uartInit.USART_Parity              = USART_Parity_No;
    uartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uartInit.USART_Mode                 = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &uartInit);

    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_ITConfig(USART1, USART_IT_PE,  ENABLE);
    USART_ITConfig(USART1, USART_IT_ERR, ENABLE);   /* ORE/NE/FE */
    USART_Cmd(USART1, ENABLE);

    /* DMA1_Channel5：USART1_RX -> 环形缓冲，循环模式，字节宽度 */
    DMA_DeInit(DMA1_Channel5);
    dmaInit.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    dmaInit.DMA_MemoryBaseAddr     = (uint32_t)s_ring;
    dmaInit.DMA_DIR                = DMA_DIR_PeripheralSRC;
    dmaInit.DMA_BufferSize         = BOOT_UART_DMA_RING_SIZE;
    dmaInit.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dmaInit.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dmaInit.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dmaInit.DMA_Mode               = DMA_Mode_Circular;
    dmaInit.DMA_Priority           = DMA_Priority_High;
    dmaInit.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &dmaInit);

    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);   /* 环绕计数（证据） */
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TE, ENABLE);   /* 传输错误计数 */
    DMA_Cmd(DMA1_Channel5, ENABLE);

    /* 消费/生产状态复位（Boot 只初始化一次） */
    s_prodAbs = 0u;
    s_prodPos = 0u;
    s_consAbs = 0u;
    s_overrun = 0u;
}

/*
 * 更新生产者绝对位置：环内位置 = RING - NDTR。
 * 位置回跳（新位置 < 上次快照）说明发生一次环绕，绝对位置进一整圈。
 * 溢出检测：待消费字节数超过环容量即置粘滞标志。
 */
void Boot_UartDma_Poll(void)
{
    u16 pos = (u16)(BOOT_UART_DMA_RING_SIZE -
                   DMA_GetCurrDataCounter(DMA1_Channel5));

    if (pos < s_prodPos)
    {
        /* 环绕一圈：绝对位置 = 上次整圈边界 + RING + 新位置 */
        s_prodAbs += (u32)(BOOT_UART_DMA_RING_SIZE - s_prodPos) + (u32)pos;
    }
    else
    {
        s_prodAbs += (u32)(pos - s_prodPos);
    }
    s_prodPos = pos;

    if ((s_prodAbs - s_consAbs) > (u32)BOOT_UART_DMA_RING_SIZE)
    {
        s_overrun = 1u;   /* 生产者套圈消费者：数据已丢失，粘滞 */
    }
}

u16 Boot_UartDma_Pending(const u8 **span0, u16 *len0,
                         const u8 **span1, u16 *len1)
{
    u32 pending = s_prodAbs - s_consAbs;
    u16 consPos = (u16)(s_consAbs & BOOT_RING_MASK);

    if (pending > (u32)BOOT_UART_DMA_RING_SIZE)
    {
        pending = (u32)BOOT_UART_DMA_RING_SIZE;   /* 溢出后按满环报告 */
    }

    if (pending == 0u)
    {
        if (span0 != NULL) { *span0 = s_ring; }
        if (len0 != NULL)  { *len0 = 0u; }
        if (span1 != NULL) { *span1 = NULL; }
        if (len1 != NULL)  { *len1 = 0u; }
        return 0u;
    }

    /* 第一段：从消费者到环尾 */
    {
        u16 first = (u16)((u32)BOOT_UART_DMA_RING_SIZE - consPos);
        if ((u32)first > pending)
        {
            first = (u16)pending;
        }

        if (span0 != NULL) { *span0 = &s_ring[consPos]; }
        if (len0 != NULL)  { *len0 = first; }

        /* 第二段：环回绕部分 */
        if (pending > (u32)first)
        {
            if (span1 != NULL) { *span1 = s_ring; }
            if (len1 != NULL)  { *len1 = (u16)(pending - first); }
        }
        else
        {
            if (span1 != NULL) { *span1 = NULL; }
            if (len1 != NULL)  { *len1 = 0u; }
        }
    }
    return (u16)pending;
}

void Boot_UartDma_Consume(u16 n)
{
    s_consAbs += (u32)n;
}

u8 Boot_UartDma_Overrun(void)
{
    return s_overrun;
}

void Boot_UartDma_ResetRx(void)
{
    Boot_UartDma_Poll();
    s_consAbs = s_prodAbs;   /* 排空：消费者追上生产者 */
    s_overrun = 0u;
}

u32 Boot_UartDma_GetIdleCount(void)
{
    return s_idleCount;
}

u32 Boot_UartDma_GetErrorCount(void)
{
    return s_errCount;
}

void Boot_UartDma_Send(const u8 *data, u16 len)
{
    while (len-- != 0u)
    {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
        {
        }
        USART_SendData(USART1, *data++);
    }
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {
    }
}

/*-----------------------------------------------------------
 * 中断服务程序（只清标志 + 计数；不解析协议、不打印）
 *----------------------------------------------------------*/
void USART1_IRQHandler(void)
{
    /* F1 清 IDLE/ORE 序列：读 SR 再读 DR */
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        (void)USART1->SR;
        (void)USART1->DR;
        s_idleCount++;
    }

    if ((USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET) ||
        (USART_GetFlagStatus(USART1, USART_FLAG_NE) != RESET) ||
        (USART_GetFlagStatus(USART1, USART_FLAG_FE) != RESET) ||
        (USART_GetFlagStatus(USART1, USART_FLAG_PE) != RESET))
    {
        (void)USART1->SR;
        (void)USART1->DR;
        s_errCount++;
    }
}

void DMA1_Channel5_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC5) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC5);
        s_tcCount++;   /* 环绕一次（证据计数，生产者追踪用轮询推导） */
    }
    if (DMA_GetITStatus(DMA1_IT_TE5) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TE5);
        s_errCount++;
    }
}
