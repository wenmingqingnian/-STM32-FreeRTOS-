#ifndef __MAIN_H
#define __MAIN_H
/*
 * 油烟机控制系统主程序头文件
 * 基于STM32F103C8T6 + FreeRTOS
 * 
 * 功能说明：
 * 1. 待机模式：风机停止，持续计算风速
 * 2. 手动模式：两档调速（LOW/HIGH），PID控制
 * 3. 自动模式：根据传感器自动调节风速
 * 4. 防回流模式：气体浓度超阈值时启动风机
 * 5. 固件更新：通过usart+dma接收固件，接收完成后跳转到app区，boot+app双区架构
 * 6. UI显示功能
 * 按键功能：
 * - 按键1(PB1)：短按切换模式
 * - 按键2(PB12)：短按切换档位，长按开关风机
 *
 */
#include "gpiox.h"
#include "delay.h"
#include "key.h"
#include "usart.h"
#include "lcd.h"
#include "gui.h"
#include "beep.h"
#include "motor.h"
#include "wind_speed.h"
#include "mq2.h"
#include "pid.h"
#include "sys.h"


/*-----------------------------------------------------------
 * 全局变量
 *----------------------------------------------------------*/
/* PID速度环控制相关变量 */
PID_TypeDef g_speedPID;             /* 速度环PID控制器 */
led_d bep;							/*beep结构体变量--注*/
led_d dht;							/*DHT11结构体变量*/

volatile int overflow=0;			/* 溢出计数器 */
volatile float speed;				/*电机转速*/


/*函数声明*/
void System_Init(void);
void StartTask_Create(void);
static void Hardware_Init(void);
#endif
