/*
 * APP 侧升级请求
 *
 * 职责（与 BOOT/boot_request.c 的读取侧配对，魔数单一定义点在
 * BOOT/boot_request.h：APP 与 Boot 都包含该头，禁止两处各写一份）：
 *   1. 安全停机：电机 H 桥禁止 + 蜂鸣器静音（直接寄存器级关断，
 *      不经命令队列——复位在即，确定性优先，见设计 §5.5）；
 *   2. 写 BKP_DR2 = BOOT_REQUEST_MAGIC：备份域跨复位保持，Boot
 *      复位后据此进入升级模式（握手超时回退前由 Boot 清除）；
 *   3. __DSB() 确保后备域写入完成，再 NVIC_SystemReset()。
 *
 * 纪律：APP 绝不擦写自身 Flash（Boot 才有写入权）；本函数不返回。
 *
 * 触发路径：串口命令 "IAP\r\n"（tools/iap_uart_sender.py --kick），
 * 由 APP_TASK/app_tasks.c 的 KeyScanTask 周期轮询 USART_RX_STA 检出行。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#ifndef __IAP_REQUEST_H__
#define __IAP_REQUEST_H__

#include "sys.h"

/* 请求升级并复位进 Boot（不返回；内部完成安全停机+请求登记） */
void IAP_Request_Update(void);

#endif /* __IAP_REQUEST_H__ */
