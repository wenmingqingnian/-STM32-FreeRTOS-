/* SPDX-License-Identifier: GPL-2.0 */
/*
 * fake_flash.c -- 主机端 Flash 行为模型实现
 * 详见 fake_flash.h
 */
#include <string.h>
#include "fake_flash.h"

void FakeFlash_Init(FakeFlash_t *f)
{
    memset(f->mem, 0xFF, sizeof(f->mem));
    f->ops_done = 0u;
    f->erase_count = 0u;
    f->program_count = 0u;
}

void FakeFlash_DisableCut(FakeFlash_t *f)
{
    f->cut_after_ops = FAKE_FLASH_NO_CUT;
}

void FakeFlash_SetCutAfter(FakeFlash_t *f, u32 ops)
{
    f->cut_after_ops = ops;
}

u8 FakeFlash_ReadByte(const FakeFlash_t *f, u32 addr)
{
    return f->mem[addr];
}

u16 FakeFlash_ReadHalfword(const FakeFlash_t *f, u32 addr)
{
    return (u16)((u16)f->mem[addr] | ((u16)f->mem[addr + 1u] << 8));
}

u32 FakeFlash_OpsDone(const FakeFlash_t *f)
{
    return f->ops_done;
}

int FakeFlash_ErasePage(void *self, u32 page_addr)
{
    FakeFlash_t *f = (FakeFlash_t *)self;

    /* 断电检查：本次操作前已成功执行的操作数达到断电点则失败，
     * 且 Flash 内容保持断电瞬间状态（本次操作未开始） */
    if (f->ops_done >= f->cut_after_ops) {
        return -1;
    }
    /* 页对齐与边界（与真实 F1 页擦除约束一致） */
    if ((page_addr % FAKE_FLASH_PAGE_SIZE) != 0u) {
        return -1;
    }
    if (page_addr >= FAKE_FLASH_TOTAL_SIZE) {
        return -1;
    }

    memset(&f->mem[page_addr], 0xFF, FAKE_FLASH_PAGE_SIZE);
    f->ops_done++;
    f->erase_count++;
    return 0;
}

int FakeFlash_ProgramHalfword(void *self, u32 addr, u16 value)
{
    FakeFlash_t *f = (FakeFlash_t *)self;
    u16 cur;

    if (f->ops_done >= f->cut_after_ops) {
        return -1;
    }
    /* 半字对齐与边界 */
    if ((addr & 1u) != 0u) {
        return -1;
    }
    if (addr >= (FAKE_FLASH_TOTAL_SIZE - 1u)) {
        return -1;
    }

    cur = (u16)((u16)f->mem[addr] | ((u16)f->mem[addr + 1u] << 8));

    /* 物理约束：编程只能 1->0。value 中任何为 1 的位若当前为 0，
     * 即为 0->1 尝试，必须失败（真实 F1 返回 PGERR） */
    if (((u32)value & ~((u32)cur)) != 0u) {
        return -1;
    }

    f->mem[addr]      = (u8)(value & 0xFFu);
    f->mem[addr + 1u] = (u8)((value >> 8) & 0xFFu);
    f->ops_done++;
    f->program_count++;
    return 0;
}
