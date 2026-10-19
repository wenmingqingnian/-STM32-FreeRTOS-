/* SPDX-License-Identifier: GPL-2.0 */
/* Host-side stub: FreeRTOS event_groups.h (Task 14 standby blocking).
 *
 * Only the symbols referenced by the production sources are provided:
 * EventBits_t / EventGroupHandle_t typedefs plus the wait API declaration
 * used by task_health.c (HealthTask). The implementation is linked by the
 * test main (host run is single-threaded; bits semantics are exercised via
 * the pure decision tables in standby_policy.c instead). */
#ifndef MOCK_STUB_EVENT_GROUPS_H
#define MOCK_STUB_EVENT_GROUPS_H

#include <stdint.h>
#include "FreeRTOS.h"   /* TickType_t / BaseType_t / pdTRUE / pdFALSE */

typedef uint32_t EventBits_t;
typedef void *EventGroupHandle_t;

/* Not executed by the mock test main (HealthTask loop is never entered);
 * linked so task_health.c references resolve. */
EventBits_t xEventGroupWaitBits(EventGroupHandle_t grp, EventBits_t bits,
                                BaseType_t clear, BaseType_t all,
                                TickType_t ticks);

#endif /* MOCK_STUB_EVENT_GROUPS_H */