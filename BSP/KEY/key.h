/*
 * 按键驱动头文件（状态机消抖）
 * 按键1: PB1 - 模式切换
 * 按键2: PB12 - 档位切换/风机开关
 *
 */
#ifndef __KEY_H
#define __KEY_H

#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpiox.h"
#include "board_pins.h"

/* 按键引脚定义：统一由 CONFIG/board_pins.h 声明（单一 pin 配置入口），
 * 本文件不再散落硬编码引脚常量。
 *
 * KEY2（PB12）双重所有权说明：
 *   PB12 同时是 KEY2 按键（教程接线）与 TIM1_BKIN 刹车输入（motor.c
 *   厂商 #if 0 死代码）的声明对象。默认 owner 为刹车输入预留，
 *   KEY2_ENABLED 编译开关（默认 0）禁用按键驱动；待原理图确认接线后
 *   置 1 启用（PB12 归按键，刹车路径必须保持禁用）。详见 board_pins.h。 */

/* 按键电平读取（假设按下为低电平） */
#define KEY1_READ()         GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_PIN)
#define KEY2_READ()         GPIO_ReadInputDataBit(KEY2_GPIO_PORT, KEY2_GPIO_PIN)

/* 按键按下电平 */
#define KEY_PRESSED_LEVEL   0   /* 按下为低电平 */

/* 时间参数定义（单位：毫秒，使用xTaskGetTickCount()精确计时） */
#define KEY_DEBOUNCE_TIME_MS    30      /* 消抖时间：30ms */
#define KEY_LONG_PRESS_TIME_MS  1000    /* 长按时间：1000ms（1秒） */

/* 按键事件定义 */
typedef enum {
    KEY_EVENT_NONE = 0,         /* 无事件 */
    KEY_EVENT_SHORT_PRESS,      /* 短按事件 */
    KEY_EVENT_LONG_PRESS,       /* 长按事件 */
    KEY_EVENT_LONG_PRESSING,    /* 长按持续中（用于蜂鸣器持续鸣叫） */
    KEY_EVENT_RELEASE           /* 按键释放事件 */
} KeyEvent_t;

/* 按键状态机状态定义 */
typedef enum {
    KEY_STATE_IDLE = 0,         /* 空闲状态 */
    KEY_STATE_DEBOUNCE,         /* 消抖状态 */
    KEY_STATE_PRESSED,          /* 按下确认状态 */
    KEY_STATE_LONG_PRESS,       /* 长按状态 */
    KEY_STATE_WAIT_RELEASE      /* 等待释放状态 */
} KeyState_t;

/* 按键控制结构体 */
typedef struct {
    KeyState_t state;           /* 当前状态 */
    TickType_t pressStartTick;  /* 按下开始时间戳（用于精确计时） */
    u8 (*readPin)(void);        /* 读取引脚电平函数指针 */
    KeyEvent_t event;           /* 当前事件 */
    u8 longPressTriggered;      /* 长按事件是否已触发 */
} Key_t;

/* 函数声明 */
void Key_Init(void);                        /* 初始化按键 */
void Key_Scan(void);                        /* 按键扫描 */
KeyEvent_t Key1_GetEvent(void);             /* 获取按键1事件 */
KeyEvent_t Key2_GetEvent(void);             /* 获取按键2事件 */
void Key1_ClearEvent(void);                 /* 清除按键1事件 */
void Key2_ClearEvent(void);                 /* 清除按键2事件 */
u8 Key1_IsPressed(void);                    /* 检查按键1是否按下 */
u8 Key1_InProgress(void);                   /* 状态机处理中（非 IDLE）：KeyScanTask 待机阻塞的豁免信号（Task 13）*/
u8 Key2_IsPressed(void);                    /* 检查按键2是否按下 */

#endif /* __KEY_H */
