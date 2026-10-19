/* SPDX-License-Identifier: GPL-2.0 */
/* Host-side stub of stm32f10x_rcc.h: minimal RCC register block + RMVF bit.
 * ResetCause_Init() touches RCC->CSR (read / RMVF write); the mock never
 * calls Init, but the symbols must link. CSR values fed to the pure decoder
 * come from reset_cause.h's own masks (RM0008). */
#ifndef MOCK_STUB_STM32F10X_RCC_H
#define MOCK_STUB_STM32F10X_RCC_H

#include "sys.h"

typedef struct {
    u32 CR;
    u32 CFGR;
    u32 CIR;
    u32 APB2RSTR;
    u32 APB1RSTR;
    u32 AHBENR;
    u32 APB2ENR;
    u32 APB1ENR;
    u32 BDCR;
    u32 CSR;
} RCC_TypeDef;

static RCC_TypeDef RCC_MOCK_INSTANCE;
#define RCC   (&RCC_MOCK_INSTANCE)

#define RCC_CSR_RMVF    (1uL << 24)

#endif /* MOCK_STUB_STM32F10X_RCC_H */
