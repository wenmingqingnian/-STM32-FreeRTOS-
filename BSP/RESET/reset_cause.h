/*
 * 复位原因模块：RCC CSR 复位标志读取与保存
 *
 * 设计要点（证据链：复位链路可观测性）：
 *   1. ResetCause_Init() 必须在 main() 最早处、任何外设初始化之前调用：
 *      RCC CSR 复位标志只能读一次，读后立即置 RMVF 清除，
 *      防止后续复位把本次原因覆盖混淆；
 *   2. 原因判定为纯函数 ResetCause_Decode(csr)：只依赖输入的 CSR 值，
 *      可脱离硬件做主机 mock 测试；
 *   3. 判定优先级：IWDG > WWDG > 软件复位 > 低功耗 > 上电 > 引脚。
 *      上电复位会同时置位 PORRSTF 与 PINRSTF（NRST 被内部拉低），
 *      故 PINRSTF 必须放在 PORRSTF 之后判定，否则上电会被误报为引脚复位；
 *   4. 结果仅保存在 RAM 启动记录（本 bootloader 不做跨复位持久化；
 *      如需跨复位保存软件请求原因，须使用 BKP 寄存器且板上必须有备份电源，
 *      否则不得声称长期持久化）。
 *
 */
#ifndef __RESET_CAUSE_H
#define __RESET_CAUSE_H

#include "sys.h"

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * 复位原因枚举（判定优先级从高到低排列）
 *----------------------------------------------------------*/
typedef enum {
    RESET_CAUSE_POWER_ON = 0,       /* 上电/掉电复位（POR/PDR） */
    RESET_CAUSE_PIN_RESET,          /* NRST 引脚复位 */
    RESET_CAUSE_SOFTWARE_RESET,     /* 软件复位（NVIC SystemReset） */
    RESET_CAUSE_IWDG_RESET,         /* 独立看门狗复位 */
    RESET_CAUSE_WWDG_RESET,         /* 窗口看门狗复位 */
    RESET_CAUSE_LOW_POWER_RESET,    /* 低功耗管理复位 */
    RESET_CAUSE_UNKNOWN,            /* 无任何已知标志（异常状态） */
    RESET_CAUSE_MAX
} ResetCause_t;

/*-----------------------------------------------------------
 * CSR 标志位掩码（RM0008 定义，与 stm32f10x_rcc.h 数值一致；
 * 独立定义是为了让纯解码函数不依赖标准库头，便于主机 mock 测试）
 *----------------------------------------------------------*/
#define RESET_CAUSE_CSR_LPWRRSTF    (1uL << 31)  /* 低功耗复位标志 */
#define RESET_CAUSE_CSR_WWDGRSTF    (1uL << 30)  /* 窗口看门狗复位标志 */
#define RESET_CAUSE_CSR_IWDGRSTF    (1uL << 29)  /* 独立看门狗复位标志 */
#define RESET_CAUSE_CSR_SFTRSTF     (1uL << 28)  /* 软件复位标志 */
#define RESET_CAUSE_CSR_PORRSTF     (1uL << 27)  /* 上电/掉电复位标志 */
#define RESET_CAUSE_CSR_PINRSTF     (1uL << 26)  /* 引脚复位标志 */

/*-----------------------------------------------------------
 * 接口
 *----------------------------------------------------------*/
/* 读取 RCC CSR 并判定复位原因，保存到 RAM 后清除全部标志。
 * 必须在 main() 最早处（外设初始化前）调用，只允许调用一次。 */
void ResetCause_Init(void);

/* 取已保存的复位原因（Init 之后的任意时刻可读） */
ResetCause_t ResetCause_Get(void);

/* 复位原因转字符串（诊断输出用） */
const char *ResetCause_GetName(ResetCause_t cause);

/* 串口打印启动复位报告（须在 uart 初始化后调用） */
void ResetCause_Report(void);

/* 纯解码函数：输入 CSR 值，输出复位原因（主机 mock 测试入口） */
ResetCause_t ResetCause_Decode(u32 csr);

#ifdef __cplusplus
}
#endif

#endif /* __RESET_CAUSE_H */
