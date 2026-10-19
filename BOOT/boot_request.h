/*
 * Boot 升级请求与恢复入口
 *
 * 职责：
 *   1. BKP_DR2 升级请求的读取与清除：APP 侧（Task 10 iap_request.c）请求
 *      升级时先安全停机，再向 BKP_DR2 写入 BOOT_REQUEST_MAGIC 并复位，
 *      Boot 复位后据此进入升级模式；握手超时回退旧 APP 前必须清请求，
 *      防止下次复位再次落入升级模式；
 *   2. 恢复键（KEY1，PB1，见 CONFIG/board_pins.h）在复位时保持按下：
 *      强制进入升级模式，无握手超时回退（用户显式恢复意图）。
 *
 * 备份域纪律（设计文档 §5.1）：
 *   - Boot 在读取请求前不得重置 backup domain（否则请求丢失）；
 *   - 本模块只开 PWR/BKP 时钟并置 DBP 位，永不触碰 RTC/BKP 复位路径。
 *
 * 恢复键语义与 APP 侧 key.c 的 KEY1 相同：PB1 输入上拉，按下为低电平。
 * Boot 不链接 FreeRTOS/延时驱动，消抖用模块内短忙等完成。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#ifndef __BOOT_REQUEST_H__
#define __BOOT_REQUEST_H__

#include "stm32f10x.h"

/*-----------------------------------------------------------
 * BKP_DR2 升级请求魔数（Task 10 的 APP 侧 iap_request.c 必须写同一值）
 *----------------------------------------------------------*/
#define BOOT_REQUEST_MAGIC      0xA55Au

/* 恢复键采样参数 */
#define BOOT_REQ_KEY_SAMPLES    8u      /* 采样次数 */
#define BOOT_REQ_KEY_DELAY_LOOPS 2000u  /* 两次采样间的短忙等循环数 */

void BootRequest_Init(void);
u8   BootRequest_IsRequested(void);
void BootRequest_Clear(void);
u8   BootRequest_RecoveryKeyPressed(void);

#endif /* __BOOT_REQUEST_H__ */
