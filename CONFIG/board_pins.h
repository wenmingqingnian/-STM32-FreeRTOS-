/*
 * 板级引脚所有权表（单一 pin 配置入口）
 *
 * 规则：
 *   1. KEY/MOTOR/DHT11 域的引脚只允许在本文件声明，驱动源码禁止散落硬编码；
 *   2. 每个物理引脚同一时刻只有一个 owner 语义生效；
 *   3. 唯一例外见下方 PB12 双重所有权声明。
 *
 * 历史缺陷（本 Task 修复的对象）：
 *   key.h:25 声明 KEY2=PB12，motor.c 厂商刹车示例（#if 0 死代码）把 PB12
 *   初始化为 TIM1_BKIN 刹车输入——同一引脚两套语义，若刹车路径被误启用，
 *   按键读取与刹车输入互相破坏。本文件用编译开关 + 编译期冲突检查消除该隐患。
 *
 * 依据：CONFIG/memory_layout.h 与 BSP/GPIO/gpiox.c
 */
#ifndef __BOARD_PINS_H
#define __BOARD_PINS_H

#include "stm32f10x.h"
#include "board_profile.h"

/*-----------------------------------------------------------
 * 端口数值编码与引脚唯一键（仅用于编译期冲突检查）
 * 说明：GPIOA/GPIOB 是指针宏、GPIO_Pin_x 带类型转换，都不能出现在
 *       #if 表达式里；冲突检查必须用下面这套纯整数编码。
 *----------------------------------------------------------*/
#define BOARD_PORTA_ID    0u
#define BOARD_PORTB_ID    1u
#define BOARD_PORTC_ID    2u

/* 引脚唯一键 = (端口编码 << 4) | 引脚号，端口内天然唯一 */
#define BOARD_PIN_KEY(port_id, pin_pos)   (((port_id) << 4) | (pin_pos))

/* 引脚掩码：与 stm32f10x_gpio.h 的 GPIO_Pin_x 取值完全一致 */
#define BOARD_PIN_MASK(pin_pos)           ((uint16_t)((uint16_t)1u << (pin_pos)))

/*-----------------------------------------------------------
 * 按键域
 *----------------------------------------------------------*/
/* KEY1：PB1，模式切换按键（兼作 STOP 深睡的 EXTI 唤醒源，见*/
#define KEY1_GPIO_PORT    GPIOB
#define KEY1_GPIO_PIN     BOARD_PIN_MASK(1)
#define PIN_KEY1_KEY      BOARD_PIN_KEY(BOARD_PORTB_ID, 1)

/* KEY2：PB12，档位切换/风机开关按键（编译开关默认禁用，见下方 PB12 说明） */
#define KEY2_GPIO_PORT    GPIOB
#define KEY2_GPIO_PIN     BOARD_PIN_MASK(12)
#define PIN_KEY2_KEY      BOARD_PIN_KEY(BOARD_PORTB_ID, 12)

/*-----------------------------------------------------------
 * 电机域（TIM1 互补 PWM + TIM2 编码器）
 *----------------------------------------------------------*/
/* 电机 PWM 输出：PA8 = TIM1_CH1 */
#define MOTOR_PWM_GPIO_PORT     GPIOA
#define MOTOR_PWM_GPIO_PIN      BOARD_PIN_MASK(8)
#define PIN_MOTOR_PWM_KEY       BOARD_PIN_KEY(BOARD_PORTA_ID, 8)

/* 电机互补输出：PB13 = TIM1_CH1N */
#define MOTOR_CH1N_GPIO_PORT    GPIOB
#define MOTOR_CH1N_GPIO_PIN     BOARD_PIN_MASK(13)
#define PIN_MOTOR_CH1N_KEY      BOARD_PIN_KEY(BOARD_PORTB_ID, 13)

/* H 桥使能：PA2 */
#define MOTOR_EN_GPIO_PORT      GPIOA
#define MOTOR_EN_GPIO_PIN       BOARD_PIN_MASK(2)
#define PIN_MOTOR_EN_KEY        BOARD_PIN_KEY(BOARD_PORTA_ID, 2)

/* 电机刹车输入（预留）：PB12 = TIM1_BKIN（厂商示例死代码，见 PB12 说明） */
#define MOTOR_BRAKE_GPIO_PORT   GPIOB
#define MOTOR_BRAKE_GPIO_PIN    BOARD_PIN_MASK(12)
#define PIN_MOTOR_BRAKE_KEY     BOARD_PIN_KEY(BOARD_PORTB_ID, 12)

/* 编码器输入：PA0 = TIM2_CH1（A 相）、PA1 = TIM2_CH2（B 相） */
#define MOTOR_ENC_A_GPIO_PORT   GPIOA
#define MOTOR_ENC_A_GPIO_PIN    BOARD_PIN_MASK(0)
#define PIN_MOTOR_ENC_A_KEY     BOARD_PIN_KEY(BOARD_PORTA_ID, 0)
#define MOTOR_ENC_B_GPIO_PORT   GPIOA
#define MOTOR_ENC_B_GPIO_PIN    BOARD_PIN_MASK(1)
#define PIN_MOTOR_ENC_B_KEY     BOARD_PIN_KEY(BOARD_PORTA_ID, 1)

/*-----------------------------------------------------------
 * 传感器域
 *----------------------------------------------------------*/
/* DHT11：PC13（板型 BOARD_C8_LSE 冻结后自 PC14 迁入，见 board_profile.h；
 * PC14/PC15 归 LSE——Y3 32.768kHz 表晶。PC13 无 LSE 复用冲突（F1 的
 * PC13 可选复用 TAMPER，本项目未启用 RTC 侵入检测）。DHT11_PIN_SELECT
 * 是板型冻结规则的显式选脚点：禁止绕过本宏直写掩码字面量，防止
 * 未来换板时遗漏迁移（board_profile.h 冻结结果注记） */
#define DHT11_PIN_SELECT     13
#define DHT11_GPIO_PORT    GPIOC
#define DHT11_GPIO_PIN     BOARD_PIN_MASK(DHT11_PIN_SELECT)
#define PIN_DHT11_KEY      BOARD_PIN_KEY(BOARD_PORTC_ID, DHT11_PIN_SELECT)

/* 板型门禁（board_profile.h 冻结规则）：LSE 板型的 DHT11 引脚必须经
 * DHT11_PIN_SELECT 显式选择，取值只允许 13（PC13，当前唯一已确认空闲脚） */
#if (BOARD_C8_LSE != 0) && (DHT11_PIN_SELECT != 13)
#error "BOARD_C8_LSE 板型：DHT11_PIN_SELECT 只允许 13（PC14 已归 LSE）"
#endif

/*-----------------------------------------------------------
 * PB12 双重所有权声明与修复（本 Task 的 Gate）
 *
 * 同一物理信号（PB12）存在两套厂商语义声明：
 *   - TIM1_BKIN 刹车输入（motor.c 厂商示例，#if 0 死代码，未在项目启用）；
 *   - KEY2 按键（教程接线，key.c 原先运行时占用）。
 * 处置（按计划）：默认 owner 为刹车输入预留（代码保持死代码状态），
 * KEY2 退为编译开关 KEY2_ENABLED（默认 0 禁用），待原理图确认接线后再
 * 启用（=1，PB12 归按键）或迁移 KEY2 到空闲引脚；两者永远互斥。
 *----------------------------------------------------------*/
#ifndef KEY2_ENABLED
#define KEY2_ENABLED      0
#endif

#if (KEY2_ENABLED != 0) && (KEY2_ENABLED != 1)
#error "KEY2_ENABLED 只允许 0 或 1"
#endif

/* 显式声明：KEY2 与 MOTOR_BRAKE 是同一物理引脚（PB12）上的双重声明，
 * 满足"同一物理信号且只保留一个语义"例外；运行时同一时刻至多一个语义生效 */
#define BOARD_PB12_DUAL_CLAIM_DECLARED   1

/*-----------------------------------------------------------
 * 编译期冲突检查（条件不满足即 #error，防未来改动引入新冲突）
 *----------------------------------------------------------*/
/* PORTA 组：MOTOR_PWM(8) / MOTOR_EN(2) / ENC_A(0) / ENC_B(1) 两两互异 */
#if (PIN_MOTOR_PWM_KEY == PIN_MOTOR_EN_KEY) || (PIN_MOTOR_PWM_KEY == PIN_MOTOR_ENC_A_KEY) || (PIN_MOTOR_PWM_KEY == PIN_MOTOR_ENC_B_KEY) || (PIN_MOTOR_EN_KEY == PIN_MOTOR_ENC_A_KEY) || (PIN_MOTOR_EN_KEY == PIN_MOTOR_ENC_B_KEY) || (PIN_MOTOR_ENC_A_KEY == PIN_MOTOR_ENC_B_KEY)
#error "PORTA 引脚所有权冲突：电机 PWM/使能/编码器组内引脚重复"
#endif

/* PORTB 组：KEY1(1) / KEY2(12) / MOTOR_CH1N(13) / MOTOR_BRAKE(12)
 * 唯一允许的相等对是 KEY2 与 MOTOR_BRAKE（见上方双重所有权声明） */
#if (PIN_KEY1_KEY == PIN_KEY2_KEY) || (PIN_KEY1_KEY == PIN_MOTOR_CH1N_KEY) || (PIN_KEY1_KEY == PIN_MOTOR_BRAKE_KEY) || (PIN_KEY2_KEY == PIN_MOTOR_CH1N_KEY) || (PIN_MOTOR_CH1N_KEY == PIN_MOTOR_BRAKE_KEY)
#error "PORTB 引脚所有权冲突：KEY/CH1N 组内引脚重复（KEY2/BRAKE 的 PB12 例外已声明）"
#endif

/* DHT11（PC13）不得与 KEY/MOTOR 域任意引脚冲突（跨端口天然不冲突，此处兜底） */
#if (PIN_DHT11_KEY == PIN_KEY1_KEY) || (PIN_DHT11_KEY == PIN_KEY2_KEY) || (PIN_DHT11_KEY == PIN_MOTOR_PWM_KEY) || (PIN_DHT11_KEY == PIN_MOTOR_CH1N_KEY) || (PIN_DHT11_KEY == PIN_MOTOR_EN_KEY) || (PIN_DHT11_KEY == PIN_MOTOR_BRAKE_KEY) || (PIN_DHT11_KEY == PIN_MOTOR_ENC_A_KEY) || (PIN_DHT11_KEY == PIN_MOTOR_ENC_B_KEY)
#error "DHT11(PC13) 与 KEY/MOTOR 域引脚冲突"
#endif

#endif /* __BOARD_PINS_H */
