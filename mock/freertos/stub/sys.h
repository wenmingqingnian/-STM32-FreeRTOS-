/* SPDX-License-Identifier: GPL-2.0 */
/* Host-side stub of the project's sys.h basic types. */
#ifndef MOCK_STUB_SYS_H
#define MOCK_STUB_SYS_H

#include <stdint.h>
#include <stddef.h>

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;

/* Mirror the real sys.h -> stm32f10x.h chain so production sources that
 * use SPL symbols (e.g. mq2.c) compile UNMODIFIED on the host. */
#include "stm32f10x.h"

#endif /* MOCK_STUB_SYS_H */
