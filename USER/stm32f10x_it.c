/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "board_profile.h"    /* BOARD_C8_LSE 板型门禁（IAP+STOP Task 12） */
#include "FreeRTOS.h"         /* vTaskNotifyGiveFromISR / portYIELD_FROM_ISR */
#include "task.h"
#include "key_exti.h"         /* KEY EXTI 唤醒通道核心（IAP+STOP Task 13）*/
#include "app_tasks.h"        /* App_GetKeyScanTaskHandle：通知目标句柄 */


 
void NMI_Handler(void)
{
}
 
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
 
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

 
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
 
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
 
//void SVC_Handler(void)
//{
//}
 
void DebugMon_Handler(void)
{
}
 
//void PendSV_Handler(void)
//{
//}
// 
//void SysTick_Handler(void)
//{
//}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available interrupt handler's name please refer to the startup           */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

#if (BOARD_C8_LSE != 0)
/**
 * @brief  RTC Alarm 中断（RTC 定时 STOP 的唤醒源，EXTI line 17）
 *         
 * @note   本 ISR 只清标志（ALRF 写 0 + EXTI17 pending 写 1），保证
 *         唤醒后中断标志不残留；STOP 退出序列（闹钟取消/时钟恢复/
 *         外设恢复）由低功耗 wrapper（Task 15）在 PRIMASK 屏蔽下
 *         统一完成——ISR 不延时、不 printf（项目纪律）。
 */
void RTCAlarm_IRQHandler(void)
{
  if ((RTC->CRL & RTC_CRL_ALRF) != (uint16_t)RESET)
  {
    RTC->CRL &= (uint16_t)~RTC_CRL_ALRF;    /* ALRF 写 0 清除（RM0008）*/
  }
  EXTI->PR = EXTI_PR_PR17;                 /* EXTI17 pending 写 1 清除 */
}
#endif /* BOARD_C8_LSE */

/******************************************************************************/
/*          KEY EXTI 唤醒通道目标侧适配器            */
/*     EXTI1 = PB1 = KEY1（CONFIG/board_pins.h）；key_exti.c 为主机可测核心    */
/******************************************************************************/
/**
  * @brief  KEY EXTI 唤醒通道目标侧 IO 适配（设计 §6.6 步 4 / §6.7）
  *         key_exti.c 的硬件访问全部经 KeyExtiIo_t 注入，本文件装配真实
  *         EXTI1/NVIC 寄存器与 vTaskNotifyGiveFromISR。ISR 体纪律：
  *         只读/清本线 pending、计数、通知任务——不延时、不 printf、
  *         不做按键判定（判定与消抖在 KeyScanTask 任务上下文完成）。
  * @note   EXTI1 抢占优先级 6 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(3)
  *（key_exti.h 编译期校验；NVIC Group 4）。PB1 输入模式（上拉）
  *         已由 Key_Init() 配置，此处只装 EXTI 复用层；双边沿触发：
  *         按下沿负责 STOP 唤醒并让任务在按住期间完成消抖/长按判定，
  *         释放沿让 RELEASE 事件零延迟到达。
  */
static KeyExti_t s_keyExti;

static void KeyExti_LineConfigImpl(void *self)
{
    EXTI_InitTypeDef extiInit;
    NVIC_InitTypeDef nvicInit;

    (void)self;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);          /* EXTI 复用时钟 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);    /* PB1 -> EXTI1 */

    extiInit.EXTI_Line    = EXTI_Line1;
    extiInit.EXTI_Mode    = EXTI_Mode_Interrupt;
    extiInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    extiInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&extiInit);

    EXTI->PR = KEY_EXTI_LINE_MASK;   /* 清残留 pending（写 1 清除）*/

    nvicInit.NVIC_IRQChannel = EXTI1_IRQn;
    nvicInit.NVIC_IRQChannelPreemptionPriority = KEY_EXTI_NVIC_PRIORITY;  /* 6 */
    nvicInit.NVIC_IRQChannelSubPriority = 0;
    nvicInit.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInit);
}

static u32 KeyExti_ReadPendingImpl(void *self)
{
    (void)self;
    return EXTI->PR;             /* 只读：唤醒源判定 */
}

static void KeyExti_ClearPendingImpl(void *self, u32 mask)
{
    (void)self;
    EXTI->PR = mask;             /* 写 1 清除（掩码由调用方限定为 KEY 线）*/
}

static void KeyExti_NotifyGiveFromIsrImpl(void *self)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    (void)self;

    vTaskNotifyGiveFromISR(App_GetKeyScanTaskHandle(), &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

static const KeyExtiIo_t s_keyExtiIo = {
    NULL,
    KeyExti_LineConfigImpl,
    KeyExti_ReadPendingImpl,
    KeyExti_ClearPendingImpl,
    KeyExti_NotifyGiveFromIsrImpl
};

/**
  * @brief  KEY EXTI 唤醒通道装配入口（StartTask 在 KeyScanTask 创建后调用）
  *         幂等：重复调用只重新配置 EXTI/NVIC 并清零诊断计数。
  */
void KeyExti_TargetInit(void)
{
    KeyExti_Init(&s_keyExti, &s_keyExtiIo);
}

/**
  * @brief  EXTI1 中断服务程序：读 pending -> 只清本线位 -> 通知 KeyScanTask。
  *         去抖、短按/长按/释放判定全部在任务上下文完成（设计 §6.7）。
  */
void EXTI1_IRQHandler(void)
{
    KeyExti_IsrBody(&s_keyExti, &s_keyExtiIo);
}
