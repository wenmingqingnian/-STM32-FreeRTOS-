/*
 * 板级硬件事实档案（STM32F103RCT6 双分区版）
 *
 * 本文件只冻结"芯片级事实"（容量/页粒度/晶振有无），全部为尺寸与布尔常量。
 * 分区地址常量（Boot/App/Slot/Meta 边界）一律不允许出现在本文件，
 * 统一由 CONFIG/memory_layout.h 建立。
 *
 * 变更记录（芯片与板型迁移）：
 *   - 由 CBT6（LQFP48，128 KiB / 20 KiB）改为 RCT6（LQFP64，256 KiB / 48 KiB）。
 *     原因：128 KiB 放不下两份 55084 B 镜像（每槽差 212 B）。
 *     分区与容量依据见 docs/design-evidence/MEMORY_LAYOUT.md。
 *   - 板型门禁仍在：实物新板焊有 32.768kHz 表晶（Y3 在 PC14/PC15），
 *     板型保持 BOARD_C8_LSE（宏名保留历史符号，语义= "本板有 LSE"）。
 *     换板后必须重新确认 LSE 晶振是否焊有：若无，改 BOARD_C8_NO_LSE。
 *   - 引脚随封装迁移：LQFP64 是 LQFP48 超集，全部在用引脚位置不变
 *（引脚归属见 CONFIG/board_pins.h）。
 *
 */
#ifndef __BOARD_PROFILE_H
#define __BOARD_PROFILE_H

/*-----------------------------------------------------------
 * MCU 型号事实
 *----------------------------------------------------------*/
#define BOARD_MCU_STM32F103RC      1   /* 大容量（High Density）；宏名保留历史风格 */

/* 官方边界（仅注释记录，地址常量归 memory_layout.h 所有）：
 *   Flash: [0x08000000, 0x08040000) = 256 KiB（STM32F103RCT6）
 *   SRAM : [0x20000000, 0x2000C000) = 48 KiB
 * Flash 基地址 0x08000000 使用 stm32f10x.h 的 FLASH_BASE，不另行定义。
 */

/*-----------------------------------------------------------
 * Flash 物理参数（IAP 擦写路径的硬约束）
 *----------------------------------------------------------*/
#define BOARD_FLASH_SIZE_BYTES     0x40000UL  /* 256 KiB（RC 官方容量，克隆片隐藏容量不得使用） */
/* 高密度（High Density）F1 页擦除粒度 = 2 KiB。
 * ST RM0008：中容量（64/128 KiB）页 1 KiB，高容量（256/384/512 KiB）页 2 KiB。
 * RCT6 = RC = 高密度（见本文件 BOARD_MCU_STM32F103RC 注释），故为 0x800。
 * ⚠️ 历史错误：换板（CBT6 128 KiB 中容量 → RCT6 256 KiB 高密度）时漏改此项，
 *    曾沿用 1 KiB，导致元数据"双页"实为同一物理页（详见 memory_layout.h 修复记录）。 */
#define BOARD_FLASH_PAGE_SIZE      0x800UL    /* 2 KiB 页擦除粒度（高密度 F1） */
#define BOARD_FLASH_HALFWORD_PROGRAM  1      /* 半字（16 bit）编程粒度 */

/*-----------------------------------------------------------
 * SRAM 物理参数
 *----------------------------------------------------------*/
#define BOARD_SRAM_SIZE_BYTES      0xC000UL   /* 48 KiB */

/*-----------------------------------------------------------
 * 晶振事实与板型门禁（低功耗路线的硬约束）
 *
 * 证据链：
 *   - 2026-09-06 板型冻结（旧 CBT6 板）：用户确认实物板焊有 32.768kHz
 *     表晶（Y3 在 PC14/PC15），冻结为 BOARD_C8_LSE；
 *   - 2026-10-11 换板（RCT6）：板型语义（有 LSE）沿用，但【换板后必须
 *     重新确认】：新板若未焊 32.768kHz 表晶，必须改 BOARD_C8_NO_LSE
 *（引脚归属以本板原理图实测为准）。
 *
 * 冻结结果（三个宏恰好一个为 1，当前为 LSE=1）：
 *   BOARD_PROFILE_PENDING : 未确认（历史态，已退出）。机制保留：换板重审
 *                           可整体回 PENDING 重走三态门禁。
 *   BOARD_C8_LSE          : 已置 1（当前值）。PC14/PC15 归 LSE，DHT11
 *                           在 PC13（board_pins.h DHT11_PIN_SELECT 门禁保持），
 *                           深睡走 RTC Alarm 定时 STOP。
 *   BOARD_C8_NO_LSE       : 0。DHT11 不得回迁 PC14；深睡的 EXTI-only STOP
 *                           分支（exti_stop_backend.c）保留但非本板路径。
 *----------------------------------------------------------*/
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
