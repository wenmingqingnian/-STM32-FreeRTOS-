/* SPDX-License-Identifier: GPL-2.0 */
/* Host-side stub of FreeRTOS task API for task_health.c / reset_cause.c mock.
 *
 * Only the symbols referenced by the production sources are provided:
 * critical sections degrade to empty macros (host run is single-threaded,
 * so read-modify-write atomicity is a target-only property verified by
 * code review on target), time APIs are trivial stubs. */
#ifndef MOCK_STUB_TASK_H
#define MOCK_STUB_TASK_H

#include <stdint.h>

typedef uint32_t TickType_t;
typedef int BaseType_t;

#define pdTRUE      1
#define pdFALSE     0
#define pdMS_TO_TICKS(ms)   (ms)
#define portMAX_DELAY       ((TickType_t)0xFFFFFFFFUL)

/* 与目标 FreeRTOSConfig.h 一致：FromISR API 允许的最低（数值最大）优先级。
 * key_exti.h 用它做 KEY EXTI 优先级的编译期校验*/
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY   3

/* Host: empty critical sections (single-threaded mock) */
#define taskENTER_CRITICAL()
#define taskEXIT_CRITICAL()

/* Not called by the mock test main; linked for completeness */
void vTaskDelayUntil(TickType_t *last, TickType_t period);
TickType_t xTaskGetTickCount(void);

#endif /* MOCK_STUB_TASK_H */
