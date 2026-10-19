/* SPDX-License-Identifier: GPL-2.0 */
/* Host-side stub of stm32f10x.h: minimal SPL surface needed by BSP\MQ2\mq2.c.
 * Mirrors the real sys.h -> stm32f10x.h include chain so the production
 * source compiles UNMODIFIED on the host. Only declarations live here;
 * mock bodies are provided by the test harness (test_mq2.c).
 *
 * NOTE: RCC_TypeDef / RCC are deliberately NOT defined here - the
 * reset-cause mock supplies its own via stub/stm32f10x_rcc.h. */
#ifndef MOCK_STUB_STM32F10X_H
#define MOCK_STUB_STM32F10X_H

#include "sys.h"

typedef enum { DISABLE = 0, ENABLE = 1 } FunctionalState;
#define SET    1
#define RESET  0
typedef u8 FlagStatus;

/* ---- GPIO (only what MQ2_Init touches) ---- */
typedef struct {
    u32 GPIO_Pin;
    u32 GPIO_Mode;
    u32 GPIO_Speed;   /* boot_uart_dma.c 也设置速度；SPL 中本成员在 Mode 之后 */
} GPIO_InitTypeDef;

typedef struct { u32 _reg[10]; } GPIO_TypeDef;
extern GPIO_TypeDef GPIOA_MOCK_INSTANCE;
#define GPIOA        (&GPIOA_MOCK_INSTANCE)

#define GPIO_Pin_4               ((u16)0x0010)
#define GPIO_Pin_1               ((u16)0x0002)

/* ---- KEY 域（key.c / key_exti.c host 编译所需---- */
#define GPIO_Mode_IPU            0x48u   /* 上拉输入（与 SPL 枚举值一致）*/
extern GPIO_TypeDef GPIOB_MOCK_INSTANCE;
#define GPIOB        (&GPIOB_MOCK_INSTANCE)
u8 GPIO_ReadInputDataBit(GPIO_TypeDef *port, u16 pin);   /* 测试体提供：电平可脚本 */
#define GPIO_Mode_AIN            0x0u

/* ---- RCC clock gates referenced by MQ2_Init ---- */
#define RCC_APB2Periph_GPIOA     ((u32)0x00000004)
#define RCC_APB2Periph_ADC1      ((u32)0x00000200)
#define RCC_PCLK2_Div6           ((u32)0x00080000)

/* ---- ADC ---- */
typedef struct {
    u32 ADC_Mode;
    u32 ADC_ScanConvMode;
    u32 ADC_ContinuousConvMode;
    u32 ADC_ExternalTrigConv;
    u32 ADC_DataAlign;
    u32 ADC_NbrOfChannel;
} ADC_InitTypeDef;

typedef struct { u32 _reg[20]; } ADC_TypeDef;
extern ADC_TypeDef ADC1_MOCK_INSTANCE;
#define ADC1         (&ADC1_MOCK_INSTANCE)

#define ADC_Mode_Independent          ((u16)0x0000)
#define ADC_ExternalTrigConv_None     ((u32)0x000E0000)
#define ADC_DataAlign_Right           ((u16)0x0000)
#define ADC_Channel_4                 ((u8)0x04)
#define ADC_SampleTime_239Cycles5     ((u8)0x07)
#define ADC_FLAG_EOC                  ((u32)0x00000002)

/* ---- SPL prototypes referenced by mq2.c ---- */
void GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
void RCC_APB2PeriphClockCmd(u32 periph, FunctionalState state);
void RCC_ADCCLKConfig(u32 clk);
void ADC_DeInit(ADC_TypeDef *adc);
void ADC_Init(ADC_TypeDef *adc, ADC_InitTypeDef *init);
void ADC_Cmd(ADC_TypeDef *adc, FunctionalState state);
void ADC_ResetCalibration(ADC_TypeDef *adc);
FlagStatus ADC_GetResetCalibrationStatus(ADC_TypeDef *adc);
void ADC_StartCalibration(ADC_TypeDef *adc);
FlagStatus ADC_GetCalibrationStatus(ADC_TypeDef *adc);
void ADC_RegularChannelConfig(ADC_TypeDef *adc, u8 channel, u8 rank, u8 sample_time);
void ADC_SoftwareStartConvCmd(ADC_TypeDef *adc, FunctionalState state);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef *adc, u32 flag);
u16 ADC_GetConversionValue(ADC_TypeDef *adc);

/*===========================================================================
 * Task 8：Boot UART DMA 环形接收（boot_uart_dma.c）
 *
 * 本段为 mock 面：声明 + 常量的值全部取自工程内真实 SPL 头，
 * 以免 mock 与真机语义漂移（枚举值出处见各行注释）。
 * 函数体由测试体提供（test_boot_uart_dma.c），与 mq2/ADC 的做法一致。
 *===========================================================================*/

/* ---- GPIO 补充（boot_uart_dma.c 的 PA9/PA10） ---- */
#define GPIO_Pin_9               ((u16)0x0200)   /* stm32f10x_gpio.h:136 */
#define GPIO_Pin_10              ((u16)0x0400)   /* stm32f10x_gpio.h:137 */
#define GPIO_Speed_50MHz         0x03u           /* GPIOSpeed_TypeDef 第 3 项 */
#define GPIO_Mode_AF_PP          0x18u           /* stm32f10x_gpio.h:79 */
#define GPIO_Mode_IN_FLOATING    0x04u           /* stm32f10x_gpio.h:73 */

/* ---- RCC 补充 ---- */
#define RCC_APB2Periph_USART1    ((u32)0x00004000)  /* stm32f10x_rcc.h:510 */
#define RCC_AHBPeriph_DMA1       ((u32)0x00000001)  /* stm32f10x_rcc.h:470 */

/* ---- USART ---- */
typedef struct {
    u32 USART_BaudRate;
    u16 USART_WordLength;
    u16 USART_StopBits;
    u16 USART_Parity;
    u16 USART_Mode;
    u16 USART_HardwareFlowControl;
} USART_InitTypeDef;

typedef struct {
    volatile u16 SR;
    volatile u16 DR;
} USART_TypeDef;
extern USART_TypeDef USART1_MOCK_INSTANCE;
#define USART1 (&USART1_MOCK_INSTANCE)

#define USART_WordLength_8b              ((u16)0x0000)  /* usart.h:125 */
#define USART_StopBits_1                 ((u16)0x0000)  /* usart.h:138 */
#define USART_Parity_No                  ((u16)0x0000)  /* usart.h:154 */
#define USART_Mode_Rx                    ((u16)0x0004)  /* usart.h:168 */
#define USART_Mode_Tx                    ((u16)0x0008)  /* usart.h:169 */
#define USART_HardwareFlowControl_None   ((u16)0x0000)  /* usart.h:178 */
#define USART_DMAReq_Tx                  ((u16)0x0080)  /* usart.h:272 */
#define USART_DMAReq_Rx                  ((u16)0x0040)  /* usart.h:273 */
#define USART_IT_PE                      ((u16)0x0028)  /* usart.h:242 */
#define USART_IT_IDLE                    ((u16)0x0424)  /* usart.h:246 */
#define USART_IT_ERR                     ((u16)0x0060)  /* usart.h:249 */
#define USART_FLAG_TXE                   ((u16)0x0080)  /* usart.h:323 */
#define USART_FLAG_TC                    ((u16)0x0040)  /* usart.h:324 */
#define USART_FLAG_ORE                   ((u16)0x0008)  /* usart.h:327 */
#define USART_FLAG_NE                    ((u16)0x0004)  /* usart.h:328 */
#define USART_FLAG_FE                    ((u16)0x0002)  /* usart.h:329 */
#define USART_FLAG_PE                    ((u16)0x0001)  /* usart.h:330 */

/* ---- DMA 通道（只建模生产代码真正读写的字段） ----
 * CNDTR 是唯一被生产代码读取的寄存器（Boot_UartDma_Poll 的 pos=1024-NDTR）；
 * CMAR 由 mock 的 DMA_Init 从 DMA_MemoryBaseAddr 捕获，测试体据此直接
 * 向环形缓冲写入"已被 DMA 搬进来的字节"，从而可验证跨环尾两段拼接的
 * 字节顺序，而不需要改动生产代码暴露内部缓冲。 */
typedef struct {
    u32 DMA_PeripheralBaseAddr;
    u32 DMA_MemoryBaseAddr;
    u32 DMA_DIR;
    u32 DMA_BufferSize;
    u32 DMA_PeripheralInc;
    u32 DMA_MemoryInc;
    u32 DMA_PeripheralDataSize;
    u32 DMA_MemoryDataSize;
    u32 DMA_Mode;
    u32 DMA_Priority;
    u32 DMA_M2M;
} DMA_InitTypeDef;

typedef struct {
    volatile u32 CCR;
    volatile u16 CNDTR;
    volatile u32 CPAR;
    volatile u32 CMAR;
} DMA_Channel_TypeDef;
extern DMA_Channel_TypeDef DMA1_Channel5_MOCK_INSTANCE;
#define DMA1_Channel5 (&DMA1_Channel5_MOCK_INSTANCE)

#define DMA_DIR_PeripheralSRC             ((u32)0x00000000)  /* dma.h:113 */
#define DMA_PeripheralInc_Disable         ((u32)0x00000000)  /* dma.h:125 */
#define DMA_MemoryInc_Enable              ((u32)0x00000080)  /* dma.h:136 */
#define DMA_PeripheralDataSize_Byte       ((u32)0x00000000)  /* dma.h:148 */
#define DMA_MemoryDataSize_Byte           ((u32)0x00000000)  /* dma.h:162 */
#define DMA_Mode_Circular                 ((u32)0x00000020)  /* dma.h:176 */
#define DMA_Priority_High                 ((u32)0x00002000)  /* dma.h:188 */
#define DMA_M2M_Disable                   ((u32)0x00000000)  /* dma.h:204 */
#define DMA_IT_TC                         ((u32)0x00000002)  /* dma.h:215 */
#define DMA_IT_TE                         ((u32)0x00000008)  /* dma.h:217 */
#define DMA1_IT_TC5                       ((u32)0x00020000)  /* dma.h:237 */
#define DMA1_IT_TE5                       ((u32)0x00080000)  /* dma.h:239 */

/* ---- NVIC ---- */
#define USART1_IRQn              37u    /* USER/stm32f10x.h:222（IRQn_Type） */
#define DMA1_Channel5_IRQn       15u    /* USER/stm32f10x.h:202 */

typedef struct {
    u8  NVIC_IRQChannel;
    u8  NVIC_IRQChannelPreemptionPriority;
    u8  NVIC_IRQChannelSubPriority;
    FunctionalState NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;

/* ---- SPL 原型（函数体由 test_boot_uart_dma.c 提供） ---- */
typedef u8 ITStatus;
void RCC_AHBPeriphClockCmd(u32 periph, FunctionalState state);
void NVIC_Init(NVIC_InitTypeDef *init);
void USART_Init(USART_TypeDef *usart, USART_InitTypeDef *init);
void USART_DMACmd(USART_TypeDef *usart, u16 req, FunctionalState state);
void USART_ITConfig(USART_TypeDef *usart, u16 it, FunctionalState state);
void USART_Cmd(USART_TypeDef *usart, FunctionalState state);
void USART_SendData(USART_TypeDef *usart, u16 data);
ITStatus USART_GetITStatus(USART_TypeDef *usart, u16 it);
FlagStatus USART_GetFlagStatus(USART_TypeDef *usart, u16 flag);
void DMA_DeInit(DMA_Channel_TypeDef *ch);
void DMA_Init(DMA_Channel_TypeDef *ch, DMA_InitTypeDef *init);
void DMA_ITConfig(DMA_Channel_TypeDef *ch, u32 it, FunctionalState state);
void DMA_Cmd(DMA_Channel_TypeDef *ch, FunctionalState state);
u16  DMA_GetCurrDataCounter(DMA_Channel_TypeDef *ch);
ITStatus DMA_GetITStatus(u32 dma_it);
void DMA_ClearITPendingBit(u32 dma_it);

#endif /* MOCK_STUB_STM32F10X_H */
