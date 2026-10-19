/* SPDX-License-Identifier: GPL-2.0 */
/*
 * fake_flash.h -- 主机端 Flash 行为模型
 *
 * 模拟 STM32F103 NOR Flash 的三个关键物理行为（容量从 memory_layout.h
 * 单一事实来源派生：C8 64 KiB / CB 128 KiB，Task 16 CB 冻结后为 128 KiB）：
 *   1. 擦除：整页写 0xFF（2 KiB 页）；
 *   2. 编程：半字写入只允许 1->0（任何需要 0->1 的编程必须失败，
 *      与真实 F1 PGERR 语义一致）；
 *   3. 断电注入：每成功执行 N 次操作后"断电"——后续所有操作失败，
 *      Flash 内容停留在断电瞬间的状态（用于重启决策测试）。
 *
 * 通过 IAP_MetaIo_t 后端接口注入 iap_meta.c（同一接口目标侧接
 * 真实 FLASH_ErasePage/FLASH_ProgramHalfWord，见设计 §5.3/§5.4）。
 */
#ifndef MOCK_FAKE_FLASH_H
#define MOCK_FAKE_FLASH_H

#include "sys.h"
#include "memory_layout.h"

/* 容量/页大小从 memory_layout.h 派生（禁止在此硬编码第二个布局事实） */
#define FAKE_FLASH_TOTAL_SIZE   ((FLASH_END_EXCL) - (FLASH_BASE))
#define FAKE_FLASH_PAGE_SIZE    (FLASH_PAGE_SIZE)
#define FAKE_FLASH_PAGE_COUNT   (FAKE_FLASH_TOTAL_SIZE / FAKE_FLASH_PAGE_SIZE)

/* 断电注入禁用标记（cut_after_ops = 该值表示不剪） */
#define FAKE_FLASH_NO_CUT        0xFFFFFFFFuL

typedef struct {
    u8  mem[FAKE_FLASH_TOTAL_SIZE];
    u32 cut_after_ops;     /* 成功执行该次数后断电；FAKE_FLASH_NO_CUT=不剪 */
    u32 ops_done;          /* 已成功执行的操作数（erase+program 合计） */
    u32 erase_count;       /* 成功擦除次数（诊断） */
    u32 program_count;     /* 成功编程次数（诊断） */
} FakeFlash_t;

/* 全片 0xFF（模拟全新芯片）；不改变断电注入配置 */
void FakeFlash_Init(FakeFlash_t *f);
void FakeFlash_DisableCut(FakeFlash_t *f);
void FakeFlash_SetCutAfter(FakeFlash_t *f, u32 ops);

u8  FakeFlash_ReadByte(const FakeFlash_t *f, u32 addr);
u16 FakeFlash_ReadHalfword(const FakeFlash_t *f, u32 addr);
u32 FakeFlash_OpsDone(const FakeFlash_t *f);

/* IAP_MetaIo_t 后端（host 实现；返回 0 成功，-1 失败） */
int FakeFlash_ErasePage(void *self, u32 page_addr);
int FakeFlash_ProgramHalfword(void *self, u32 addr, u16 value);

#endif /* MOCK_FAKE_FLASH_H */
