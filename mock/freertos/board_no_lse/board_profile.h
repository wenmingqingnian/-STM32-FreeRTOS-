/*
 * 影子 NO_LSE 板型（仅 host mock 编译用，板型冻结 BOARD_C8_LSE 后新增）
 *
 * 用途：实物板型已冻结 BOARD_C8_LSE（CONFIG/board_profile.h，2026-09-06
 * 用户确认表晶），rtc-stub 套件不能再从 CONFIG 解析 PENDING 板型——改用
 * 本目录影子让 rtc_timebase.c 的零硬件 stub 分支保持可验证
 *（-I 顺序：stub -> board_no_lse -> BSP\LOWPOWER，board_profile.h 从这里解析）。
 *
 * 与 CONFIG/board_profile.h 的唯一差异：三个板型宏的取值（PENDING=0，
 * LSE=0，NO_LSE=1），满足"恰好一个为 1"的互斥断言。
 *
 * 纪律（board_profile.h 冻结规则）：
 *   - 本文件绝不进入任何 Keil 工程的 include 路径；
 *   - 实物板型已冻结 LSE：NO_LSE 只是 stub 分支的主机回归载体，
 *     不代表实物板型（stub 分支代码在目标侧不参与链接）；
 *   - NO_LSE 路径的 STOP 行为验证由 exti-stop 套件覆盖
 *（不依赖 board_profile.h 板型宏）。
 */
#ifndef __BOARD_PROFILE_H
#define __BOARD_PROFILE_H

#define BOARD_PROFILE_PENDING     0
#define BOARD_C8_LSE              0
#define BOARD_C8_NO_LSE           1

#if (BOARD_C8_LSE != 0) && (BOARD_C8_NO_LSE != 0)
#error "板型冲突：BOARD_C8_LSE 与 BOARD_C8_NO_LSE 互斥，只能选一个"
#endif

#if (BOARD_PROFILE_PENDING != 0) && ((BOARD_C8_LSE != 0) || (BOARD_C8_NO_LSE != 0))
#error "板型冲突：BOARD_PROFILE_PENDING 期间禁止冻结 LSE 或 NO_LSE"
#endif

#if (BOARD_PROFILE_PENDING == 0) && (BOARD_C8_LSE == 0) && (BOARD_C8_NO_LSE == 0)
#error "板型未定义：PENDING/LSE/NO_LSE 必须恰好置一"
#endif

#endif /* __BOARD_PROFILE_H */
