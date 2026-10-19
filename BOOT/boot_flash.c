// SPDX-License-Identifier: GPL-2.0
/*
 * Boot 专用 Flash 写入器实现
 *
 * 见 boot_flash.h 的契约。纯逻辑 + 经注入的 Flash IO：
 * 本文件不访问任何寄存器（host mock 接 fake_flash，目标侧后端
 * 由 Task 8 提供），也不维护任何页镜像/整页缓存。
 *
 * 写操作（擦除/编程/元数据擦除）的统一骨架：
 *   参数校验（不解锁）-> 边界/对齐校验（不解锁）
 *   -> 解锁一次 -> 逐半字/逐页操作，每次检查后端状态，
 *      任一失败立即停止并保留错误码与 failAddr
 *   -> 全部返回路径统一上锁。
 * 读操作（回读/CRC）不解锁。
 *
 */
#include <string.h>
#include "boot_flash.h"
#include "crc32.h"   /* 整镜像 CRC 复用 Task 4 流式接口 */

/*-----------------------------------------------------------
 * 内部工具
 *----------------------------------------------------------*/

/* 记录结果（保留错误码直至下一次操作） */
static BootFlash_Status_t bf_set(BootFlash_t *f, BootFlash_Status_t st, u32 failAddr)
{
    f->status = (u8)st;
    f->failAddr = failAddr;
    return st;
}

/* 写操作收尾：上锁（有后端才上锁） */
static void bf_lock(const BootFlash_Io_t *io)
{
    if (io->lock != NULL)
        io->lock(io->self);
}

/* 写操作开锁：有后端才解锁（解锁一次，全部操作共享） */
static void bf_unlock(const BootFlash_Io_t *io)
{
    if (io->unlock != NULL)
        io->unlock(io->self);
}

/* 通用范围校验：addr 2 B 对齐、[addr, addr+len) 在 APP 区内。
 * 全部通过返回 BOOTFLASH_OK（不修改 status——调用方在入口已重置）。
 * 边界判定先减后比：len > appLimitExcl - addr，杜绝 addr+len 回绕。 */
static BootFlash_Status_t bf_check_range(BootFlash_t *f, u32 addr, u32 len)
{
    const BootFlash_Config_t *c = &f->cfg;

    if (addr < c->appBase || addr >= c->appLimitExcl)
        return BOOTFLASH_ERR_BOUNDS;
    if ((addr & 1u) != 0u)
        return BOOTFLASH_ERR_ALIGN;
    if (len > c->appLimitExcl - addr)
        return BOOTFLASH_ERR_BOUNDS;
    return BOOTFLASH_OK;
}

/*-----------------------------------------------------------
 * Init
 *----------------------------------------------------------*/

void BootFlash_Init(BootFlash_t *f, const BootFlash_Io_t *io,
                    const BootFlash_Config_t *cfg)
{
    if (f == NULL)
        return;
    memset(f, 0, sizeof(*f));
    if (io != NULL && cfg != NULL) {
        f->io = io;
        f->cfg = *cfg;
        f->status = (u8)BOOTFLASH_OK;
    }
    /* io 或 cfg 为 NULL：保持全 0，后续所有操作按 PARAM 拒绝 */
}

/*-----------------------------------------------------------
 * APP 分区页擦除
 *----------------------------------------------------------*/

BootFlash_Status_t BootFlash_ErasePages(BootFlash_t *f, u32 startAddr, u32 numPages)
{
    const BootFlash_Config_t *c;
    const BootFlash_Io_t *io;
    u32 i;

    if (f == NULL || f->io == NULL)
        return BOOTFLASH_ERR_PARAM;
    c = &f->cfg;
    io = f->io;

    if (numPages == 0u) {
        return bf_set(f, BOOTFLASH_ERR_PARAM, startAddr);
    }
    if (startAddr < c->appBase || startAddr >= c->appLimitExcl) {
        return bf_set(f, BOOTFLASH_ERR_BOUNDS, startAddr);
    }
    if ((startAddr & (c->pageSize - 1u)) != 0u) {
        return bf_set(f, BOOTFLASH_ERR_ALIGN, startAddr);
    }
    /* 先减后比：numPages 巨大时 startAddr+numPages*pageSize 会回绕，
     * 本判定用剩余页数比较，无溢出路径 */
    if (numPages > (c->appLimitExcl - startAddr) / c->pageSize) {
        return bf_set(f, BOOTFLASH_ERR_BOUNDS, startAddr);
    }

    bf_unlock(io);
    for (i = 0; i < numPages; i++) {
        u32 page = startAddr + i * c->pageSize;
        if (io->erase_page(io->self, page) != 0) {
            /* 立即停止：后续页保持原样，错误码与失败页地址保留 */
            bf_lock(io);
            return bf_set(f, BOOTFLASH_ERR_ERASE, page);
        }
    }
    bf_lock(io);
    return bf_set(f, BOOTFLASH_OK, 0);
}

/*-----------------------------------------------------------
 * 元数据页擦除（独立 API，仅 A/B 两页）
 *----------------------------------------------------------*/

BootFlash_Status_t BootFlash_EraseMetaPage(BootFlash_t *f, u8 metaSel)
{
    const BootFlash_Io_t *io;
    u32 page;

    if (f == NULL || f->io == NULL)
        return BOOTFLASH_ERR_PARAM;
    io = f->io;

    if (metaSel == (u8)BOOTFLASH_META_A) {
        page = f->cfg.metaABase;
    } else if (metaSel == (u8)BOOTFLASH_META_B) {
        page = f->cfg.metaBBase;
    } else {
        return bf_set(f, BOOTFLASH_ERR_PARAM, 0);
    }

    bf_unlock(io);
    if (io->erase_page(io->self, page) != 0) {
        bf_lock(io);
        return bf_set(f, BOOTFLASH_ERR_ERASE, page);
    }
    bf_lock(io);
    return bf_set(f, BOOTFLASH_OK, 0);
}

/*-----------------------------------------------------------
 * DATA 包半字编程
 *----------------------------------------------------------*/

BootFlash_Status_t BootFlash_ProgramData(BootFlash_t *f, u32 addr,
                                         const u8 *data, u32 len)
{
    const BootFlash_Io_t *io;
    BootFlash_Status_t st;
    u32 nHalf, h;

    if (f == NULL || f->io == NULL || data == NULL || len == 0u)
        return BOOTFLASH_ERR_PARAM;
    io = f->io;

    st = bf_check_range(f, addr, len);
    if (st != BOOTFLASH_OK) {
        return bf_set(f, st, addr);
    }

    nHalf = len / 2u;
    bf_unlock(io);
    for (h = 0; h < nHalf; h++) {
        u16 v = (u16)((u16)data[2u * h] | (u16)((u16)data[2u * h + 1u] << 8));
        u32 a = addr + 2u * h;
        if (io->program_halfword(io->self, a, v) != 0) {
            /* 立即停止：从失败半字起保持原样，错误码与地址保留 */
            bf_lock(io);
            return bf_set(f, BOOTFLASH_ERR_PROGRAM, a);
        }
    }
    if ((len & 1u) != 0u) {
        /* 奇数末字节：0xFF 补高字节（补位不参与镜像 CRC/回读） */
        u16 v = (u16)((u16)data[len - 1u] | (u16)0xFF00u);
        u32 a = addr + len - 1u;
        if (io->program_halfword(io->self, a, v) != 0) {
            bf_lock(io);
            return bf_set(f, BOOTFLASH_ERR_PROGRAM, a);
        }
    }
    bf_lock(io);
    return bf_set(f, BOOTFLASH_OK, 0);
}

/*-----------------------------------------------------------
 * 包级回读（只读，不解锁）
 *----------------------------------------------------------*/

BootFlash_Status_t BootFlash_ReadbackVerify(BootFlash_t *f, u32 addr,
                                           const u8 *data, u32 len)
{
    const BootFlash_Io_t *io;
    BootFlash_Status_t st;
    u32 nHalf, h;

    if (f == NULL || f->io == NULL || data == NULL || len == 0u)
        return BOOTFLASH_ERR_PARAM;
    io = f->io;

    st = bf_check_range(f, addr, len);
    if (st != BOOTFLASH_OK) {
        return bf_set(f, st, addr);
    }

    nHalf = len / 2u;
    for (h = 0; h < nHalf; h++) {
        u16 expect = (u16)((u16)data[2u * h] | (u16)((u16)data[2u * h + 1u] << 8));
        u32 a = addr + 2u * h;
        if (io->read_halfword(io->self, a) != expect) {
            /* 不一致：Task 8 据此 NACK 该包，不得继续 */
            return bf_set(f, BOOTFLASH_ERR_READBACK, a);
        }
    }
    if ((len & 1u) != 0u) {
        /* 末半字只比较低字节（补位 0xFF 不参与回读） */
        u32 a = addr + len - 1u;
        if ((u8)(io->read_halfword(io->self, a) & 0xFFu) != data[len - 1u]) {
            return bf_set(f, BOOTFLASH_ERR_READBACK, a);
        }
    }
    return bf_set(f, BOOTFLASH_OK, 0);
}

/*-----------------------------------------------------------
 * 整镜像 CRC 校验（只读，不解锁；复用 Task 4 流式接口）
 *----------------------------------------------------------*/

BootFlash_Status_t BootFlash_VerifyCrc(BootFlash_t *f, u32 addr, u32 size,
                                       u32 expectedCrc)
{
    const BootFlash_Io_t *io;
    BootFlash_Status_t st;
    CRC32_StreamCtx_t c;
    u32 nHalf, h;

    if (f == NULL || f->io == NULL || size == 0u)
        return BOOTFLASH_ERR_PARAM;
    io = f->io;

    st = bf_check_range(f, addr, size);
    if (st != BOOTFLASH_OK) {
        return bf_set(f, st, addr);
    }

    CRC32_Init(&c);
    nHalf = size / 2u;
    for (h = 0; h < nHalf; h++) {
        u16 v = io->read_halfword(io->self, addr + 2u * h);
        u8 b[2];
        b[0] = (u8)(v & 0xFFu);
        b[1] = (u8)((v >> 8) & 0xFFu);
        CRC32_Update(&c, b, 2u);
    }
    if ((size & 1u) != 0u) {
        /* 奇数末字节只取半字低字节：补位 0xFF 不计入镜像 CRC */
        u16 v = io->read_halfword(io->self, addr + size - 1u);
        u8 b = (u8)(v & 0xFFu);
        CRC32_Update(&c, &b, 1u);
    }

    if (CRC32_Final(&c) != expectedCrc) {
        /* 整镜像级失败：无单一失败地址，置 0 */
        return bf_set(f, BOOTFLASH_ERR_CRC, 0);
    }
    return bf_set(f, BOOTFLASH_OK, 0);
}
