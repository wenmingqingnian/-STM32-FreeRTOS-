/* SPDX-License-Identifier: GPL-2.0 */
/* Host-side stub of SYSTEM\delay\delay.h (the real file is GBK-encoded;
 * this stub shadows it via -I order and keeps the host build ASCII-clean).
 * Bodies are provided by the test harness. */
#ifndef MOCK_STUB_DELAY_H
#define MOCK_STUB_DELAY_H

#include "sys.h"

void delay_init(void);
void delay_ms(u32 nms);
void delay_us(u32 nus);
void delay_xms(u32 nms);

#endif /* MOCK_STUB_DELAY_H */
