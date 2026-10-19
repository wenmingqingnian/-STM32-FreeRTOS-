/*
 * Boot 最小骨架头文件
 *
 * 边界声明（与计划一致）：
 *   - Boot 为裸机程序：不包含 FreeRTOS、不使用 printf/半主机；
 *   - 真实主流程（UART DMA 分包流式写入 + 镜像校验 + 无栈跳转）在
 *     Task 6/7/8/9 中逐步填充，本头文件只暴露骨架阶段已存在的接口；
 *   - 地址常量（APP 基址等）只允许出现在 CONFIG/memory_layout.h（Task 3
 *     建立）与 Keil Target 配置中，本文件不定义任何分区地址。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#ifndef __BOOT_MINIMAL_H
#define __BOOT_MINIMAL_H

#include "stm32f10x.h"
#include "board_profile.h"

/* USART1 波特率：与 APP 侧一致，PC 上位机共用同一条串口链路 */
#define BOOT_UART_BAUDRATE      115200u

/* Boot 阶段状态机（Task 8 决策主流程使用；观测用，不参与控制流） */
typedef enum
{
    BOOT_PHASE_INIT = 0,        /* 外设初始化完成 */
    BOOT_PHASE_WAIT_FRAME,      /* 升级会话常驻循环 */
    BOOT_PHASE_JUMP             /* 决策为启动 APP（Task 9 无栈跳转） */
} Boot_Phase_t;

Boot_Phase_t Boot_GetPhase(void);

#endif /* __BOOT_MINIMAL_H */
