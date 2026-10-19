#!/bin/sh
# Host-side mock run of Plan-A Step5 pure logic (task heartbeat verdict +
# reset-cause decode).  L2 evidence for the resume/evidence matrix.
#
# Usage: sh run-mock-step5.sh
#
# Compiles the UNMODIFIED production sources (task_health.c, reset_cause.c)
# against the stub/ FreeRTOS/sys/RCC headers with the host gcc and runs the
# suites.  Exit status is non-zero when any case fails.
set -e

cd "$(dirname "$0")"
ROOT="../.."

gcc -Wall -Wextra -Istub -I"$ROOT/APP_TASK" -I"$ROOT/BSP/RESET" -I"$ROOT/BSP/WATCHDOG" \
    -o /tmp/step5-mock \
    test_task_health.c \
    "$ROOT/APP_TASK/task_health.c" \
    "$ROOT/BSP/RESET/reset_cause.c"

echo "gcc exit: $?"
/tmp/step5-mock
