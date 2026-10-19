/*
 * 影子 LSE 板型（仅 host mock 编译用，IAP+STOP*
 * 用途：run-mock-iap-stop.ps1 编译 rtc_timebase.c 的 BOARD_C8_LSE 完整
 * 分支时，用本目录影子覆盖 CONFIG\board_profile.h 的板型宏（-I 顺序：
 * stub -> board_lse -> BSP\LOWPOWER，board_profile.h 从这里解析）。
 *
 * 与 CONFIG/board_profile.h 的唯一差异：三个板型宏的取值（PENDING=0，
 * LSE=1，NO_LSE=0），满足"恰好一个为 1"的互斥断言。
 *
 * 纪律：
 *   - 本文件绝不进入任何 Keil 工程的 include 路径；
 *   - 实物板型已冻结 BOARD_C8_LSE（2026-09-06 用户确认表晶）；影子与
 *     CONFIG 同口径（LSE=1），仅为 rtc-lse 套件的来源显式化保留；
 *   - LSE 完整分支的目标侧真实编译证据由板型冻结后的 Keil 五 Target
 *     板级验证提供；本影子板型只做主机编译与行为覆盖。
 */
#ifndef __BOARD_PROFILE_H
#define __BOARD_PROFILE_H

#define BOARD_PROFILE_PENDING     0
#define BOARD_C8_LSE              1
#define BOARD_C8_NO_LSE           0

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
