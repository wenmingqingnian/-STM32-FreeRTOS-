/* SPDX-License-Identifier: GPL-2.0 */
/* Host-side stub of gpiox.h: only io_set() is referenced by the production
 * key.c compiled in the key-wakeup suite . The body is provided
 * by the test harness; the real BSP\GPIO\gpiox.h is shadowed via -I order. */
#ifndef MOCK_STUB_GPIOX_H
#define MOCK_STUB_GPIOX_H

#include "stm32f10x.h"    /* host stub: GPIO_TypeDef / GPIO_Mode_IPU */

void io_set(GPIO_TypeDef *port, u16 pin, u32 mode);

#endif /* MOCK_STUB_GPIOX_H */