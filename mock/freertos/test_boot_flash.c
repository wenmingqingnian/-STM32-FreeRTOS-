// SPDX-License-Identifier: GPL-2.0
/*
 * Host-side mock test for the bounded halfword IAP flash writer
 * Covers:
 *
 *   1. APP start/end boundary, metadata pages and boot region all
 *      rejected by the APP-plane APIs (erase/program/readback/CRC)
 *   2. erase address not 1 KiB aligned rejected
 *   3. program address not 2 B aligned rejected; odd final byte
 *      programmed with 0xFF as the high byte of the last halfword
 *      (image CRC / readback cover only the real bytes)
 *   4. every erase/program backend error stops immediately and the
 *      error code stays retained in the context (failAddr diagnostics)
 *   5. packet-level readback mismatch detected (-> NACK decision in
 *      Task 8); whole-image CRC mismatch detected (-> no VALID commit)
 *   6. image_size == APP_CAPACITY exactly is writable; +1 rejected
 *
 * Additional contract checks (plan "Implement" bullets):
 *   - unlock exactly once per write operation, locked again on every
 *     error path, never unlocked for read-only or parameter errors
 *   - subtract-first bounds checks (len > limit-addr): the naive
 *     addr+len <= limit form would wrap on addr=APP_BASE,len=0xFFFFFFFF
 *   - no 1 KiB page mirror: writes go halfword by halfword (verified
 *     by the fake_flash op counter and partial-content checks)
 *
 * Production sources under test are compiled UNMODIFIED from the project
 * tree (BOOT\boot_flash.c + BSP\CRC32\crc32.c); sys.h comes from stub/,
 * flash physics (page erase, 1->0 program, power-cut -> op failure)
 * from fake_flash.c behind a counting wrapper IO.
 */
#include <stdio.h>
#include <string.h>
#include "boot_flash.h"
#include "crc32.h"
#include "fake_flash.h"
#include "memory_layout.h"

static int g_failed = 0;
static int g_total = 0;

#define CHECK_DESC(desc, cond) do {                      \
    g_total++;                                            \
    if (cond) { printf("  ok %s\n", desc); }              \
    else      { g_failed++; printf("  FAIL %s\n", desc); } \
} while (0)

#define CHECK_EQ_DESC(desc, got, want) do {               \
    g_total++;                                            \
    if ((unsigned long)(got) == (unsigned long)(want))    \
        printf("  ok %s\n", desc);                        \
    else { g_failed++;                                    \
        printf("  FAIL %s: got %lu, want %lu\n",           \
               desc, (unsigned long)(got),                \
               (unsigned long)(want)); }                  \
} while (0)

/*--------------------------- wrapper IO ---------------------------*/

/* fake_flash 使用片内偏移；本 wrapper 做绝对地址->偏移换算并统计解锁/上锁 */
static FakeFlash_t g_ff;
static int g_unlockCnt;
static int g_lockCnt;

#define OFF(x) ((x) - FLASH_BASE)

static void w_unlock(void *self) { (void)self; g_unlockCnt++; }
static void w_lock(void *self)   { (void)self; g_lockCnt++; }

static int w_erase(void *self, u32 page_addr)
{
    return FakeFlash_ErasePage(self, OFF(page_addr));
}

static int w_program(void *self, u32 addr, u16 value)
{
    return FakeFlash_ProgramHalfword(self, OFF(addr), value);
}

static u16 w_read(void *self, u32 addr)
{
    return FakeFlash_ReadHalfword((const FakeFlash_t *)self, OFF(addr));
}

static BootFlash_Io_t g_io = { &g_ff, w_unlock, w_lock, w_erase, w_program, w_read };

/*--------------------------- harness ---------------------------*/

static BootFlash_t g_bf;

static const BootFlash_Config_t g_cfg = {
    APP_BASE,       /* appBase（memory_layout.h，未硬编码在模块内） */
    APP_LIMIT_EXCL, /* appLimitExcl */
    META_A_BASE,    /* metaABase */
    META_B_BASE,    /* metaBBase */
    FLASH_PAGE_SIZE /* pageSize = 1 KiB */
};

static void harness_reset(void)
{
    FakeFlash_Init(&g_ff);
    FakeFlash_DisableCut(&g_ff);
    g_unlockCnt = 0;
    g_lockCnt = 0;
    BootFlash_Init(&g_bf, &g_io, &g_cfg);
}

static void fill_pattern(u8 *buf, u32 len, u8 seed)
{
    u32 i;
    for (i = 0; i < len; i++) buf[i] = (u8)(seed + (u8)i);
}

/* 往 fake_flash 半字编程原始值（测试侧直接布置现场，绕过被测模块） */
static void ff_poke(u32 absAddr, u16 value)
{
    (void)FakeFlash_ProgramHalfword(&g_ff, OFF(absAddr), value);
}

static u16 ff_peek(u32 absAddr)
{
    return FakeFlash_ReadHalfword(&g_ff, OFF(absAddr));
}

/* 向某页填入非 0xFF 的旧数据（验证"哪些页真被擦除"用） */
static void junk_page(u32 pageAddr)
{
    u32 i;
    for (i = 0; i < FLASH_PAGE_SIZE / 2; i++)
        ff_poke(pageAddr + 2u * i, (u16)(0xA5A5u + i));
}

/* 断电点按"当前已完成操作数 + n"设置：布置现场的编程本身不计入
 *（fake_flash 的 cut_after_ops 是全局累计计数，直接设小值会把
 * 布置操作也算成已断电） */
static void cut_after_now(u32 n)
{
    FakeFlash_SetCutAfter(&g_ff, FakeFlash_OpsDone(&g_ff) + n);
}

/*--------------------------- 1. 区域边界 ---------------------------*/

static void s1_bounds(void)
{
    u8 d[4] = {0x11, 0x22, 0x33, 0x44};

    printf("== 1. region bounds: boot/meta/end all rejected ==\n");

    /* 1.1 erase：Boot 区 / APP 下方 / 元数据页 / APP 终点（含）均拒绝 */
    harness_reset();
    CHECK_EQ_DESC("s1 erase boot page rejected",
                  BootFlash_ErasePages(&g_bf, FLASH_BASE, 1), BOOTFLASH_ERR_BOUNDS);
    CHECK_EQ_DESC("s1 erase below app rejected",
                  BootFlash_ErasePages(&g_bf, APP_BASE - FLASH_PAGE_SIZE, 1), BOOTFLASH_ERR_BOUNDS);
    CHECK_EQ_DESC("s1 erase metaA rejected",
                  BootFlash_ErasePages(&g_bf, META_A_BASE, 1), BOOTFLASH_ERR_BOUNDS);
    /* failAddr 断言须紧跟 metaA 调用：后续拒绝会覆盖它 */
    CHECK_EQ_DESC("s1 erase failAddr kept",
                  g_bf.failAddr, META_A_BASE);
    CHECK_EQ_DESC("s1 erase metaB rejected",
                  BootFlash_ErasePages(&g_bf, META_B_BASE, 1), BOOTFLASH_ERR_BOUNDS);
    CHECK_EQ_DESC("s1 erase at app limit rejected",
                  BootFlash_ErasePages(&g_bf, APP_LIMIT_EXCL, 1), BOOTFLASH_ERR_BOUNDS);
    CHECK_EQ_DESC("s1 erase status retained",
                  g_bf.status, BOOTFLASH_ERR_BOUNDS);

    /* 1.2 erase：APP 起点 / 最后一页接受 */
    harness_reset();
    CHECK_EQ_DESC("s1 erase app first page ok",
                  BootFlash_ErasePages(&g_bf, APP_BASE, 1), BOOTFLASH_OK);
    CHECK_EQ_DESC("s1 erase app last page ok",
                  BootFlash_ErasePages(&g_bf, APP_LIMIT_EXCL - FLASH_PAGE_SIZE, 1), BOOTFLASH_OK);

    /* 1.3 erase：numPages=0 拒绝；numPages 溢出回绕（0xFFFFFFFF）拒绝 */
    CHECK_EQ_DESC("s1 erase zero pages rejected",
                  BootFlash_ErasePages(&g_bf, APP_BASE, 0), BOOTFLASH_ERR_PARAM);
    CHECK_EQ_DESC("s1 erase huge page count rejected",
                  BootFlash_ErasePages(&g_bf, APP_BASE, 0xFFFFFFFFuL), BOOTFLASH_ERR_BOUNDS);

    /* 1.4 program：Boot 区 / APP 下方 / 元数据页 / APP 终点均拒绝 */
    harness_reset();
    CHECK_EQ_DESC("s1 program boot rejected",
                  BootFlash_ProgramData(&g_bf, FLASH_BASE, d, 4), BOOTFLASH_ERR_BOUNDS);
    CHECK_EQ_DESC("s1 program below app rejected",
                  BootFlash_ProgramData(&g_bf, APP_BASE - 2, d, 2), BOOTFLASH_ERR_BOUNDS);
    CHECK_EQ_DESC("s1 program metaA rejected",
                  BootFlash_ProgramData(&g_bf, META_A_BASE, d, 4), BOOTFLASH_ERR_BOUNDS);
    CHECK_EQ_DESC("s1 program metaB rejected",
                  BootFlash_ProgramData(&g_bf, META_B_BASE, d, 4), BOOTFLASH_ERR_BOUNDS);
    /* failAddr 断言须紧跟 metaB 调用：at-limit 拒绝会覆盖它 */
    CHECK_EQ_DESC("s1 program failAddr kept",
                  g_bf.failAddr, META_B_BASE);
    CHECK_EQ_DESC("s1 program at app limit rejected",
                  BootFlash_ProgramData(&g_bf, APP_LIMIT_EXCL, d, 2), BOOTFLASH_ERR_BOUNDS);

    /* 1.5 program：终点半字（恰好写到 APP_LIMIT_EXCL-2..-1）接受；
     *     越过终点一个字节拒绝 */
    CHECK_EQ_DESC("s1 program last halfword ok",
                  BootFlash_ProgramData(&g_bf, APP_LIMIT_EXCL - 2, d, 2), BOOTFLASH_OK);
    CHECK_EQ_DESC("s1 program one past limit rejected",
                  BootFlash_ProgramData(&g_bf, APP_LIMIT_EXCL - 2, d, 3), BOOTFLASH_ERR_BOUNDS);

    /* 1.6 先减后比溢出证明：addr=APP_BASE,len=0xFFFFFFFF 朴素写法
     *     addr+len 会回绕成 APP_BASE-1 < limit 而放行；先减后比必须拒绝 */
    CHECK_EQ_DESC("s1 program len wrap rejected",
                  BootFlash_ProgramData(&g_bf, APP_BASE, d, 0xFFFFFFFFuL), BOOTFLASH_ERR_BOUNDS);
    CHECK_EQ_DESC("s1 program wrap no flash op",
                  g_ff.ops_done, 1);   /* 仅 1.5 的一次半字编程，无写入循环 */

    /* 1.7 readback / CRC 同样受边界约束 */
    CHECK_EQ_DESC("s1 readback meta rejected",
                  BootFlash_ReadbackVerify(&g_bf, META_A_BASE, d, 4), BOOTFLASH_ERR_BOUNDS);
    CHECK_EQ_DESC("s1 readback at limit rejected",
                  BootFlash_ReadbackVerify(&g_bf, APP_LIMIT_EXCL, d, 2), BOOTFLASH_ERR_BOUNDS);
    CHECK_EQ_DESC("s1 crc meta rejected",
                  BootFlash_VerifyCrc(&g_bf, META_A_BASE, 8, 0), BOOTFLASH_ERR_BOUNDS);
    CHECK_EQ_DESC("s1 crc zero size rejected",
                  BootFlash_VerifyCrc(&g_bf, APP_BASE, 0, 0), BOOTFLASH_ERR_PARAM);
}

/*--------------------------- 2. 对齐与奇数补 0xFF ---------------------------*/

static void s2_align_parity(void)
{
    static u8 d5[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
    u8 d4[4] = {0xAA, 0xBB, 0xCC, 0xDD};

    printf("== 2. alignment & odd-length 0xFF padding ==\n");

    /* 2.1 erase 非 1 KiB 对齐拒绝（区域内但不对齐） */
    harness_reset();
    CHECK_EQ_DESC("s2 erase +2 misaligned rejected",
                  BootFlash_ErasePages(&g_bf, APP_BASE + 2, 1), BOOTFLASH_ERR_ALIGN);
    /* failAddr 断言须紧跟 +2 调用：+0x401 拒绝会覆盖它 */
    CHECK_EQ_DESC("s2 erase failAddr kept",
                  g_bf.failAddr, APP_BASE + 2);
    CHECK_EQ_DESC("s2 erase +0x401 misaligned rejected",
                  BootFlash_ErasePages(&g_bf, APP_BASE + 0x401, 1), BOOTFLASH_ERR_ALIGN);
    CHECK_EQ_DESC("s2 erase failAddr updated",
                  g_bf.failAddr, APP_BASE + 0x401);

    /* 2.2 program 非 2 B 对齐拒绝 */
    CHECK_EQ_DESC("s2 program odd addr rejected",
                  BootFlash_ProgramData(&g_bf, APP_BASE + 1, d4, 4), BOOTFLASH_ERR_ALIGN);
    CHECK_EQ_DESC("s2 program failAddr kept",
                  g_bf.failAddr, APP_BASE + 1);

    /* 2.3 奇数长度：末字节以 0xFF 补高字节的半字编程 */
    harness_reset();
    CHECK_EQ_DESC("s2 odd len program ok",
                  BootFlash_ProgramData(&g_bf, APP_BASE, d5, 5), BOOTFLASH_OK);
    CHECK_EQ_DESC("s2 hw0 value", ff_peek(APP_BASE), 0x2211u);
    CHECK_EQ_DESC("s2 hw1 value", ff_peek(APP_BASE + 2), 0x4433u);
    CHECK_EQ_DESC("s2 hw2 padded 0xFF high byte", ff_peek(APP_BASE + 4), 0xFF55u);
    CHECK_EQ_DESC("s2 odd readback ok",
                  BootFlash_ReadbackVerify(&g_bf, APP_BASE, d5, 5), BOOTFLASH_OK);
    CHECK_EQ_DESC("s2 odd readback misaligned rejected",
                  BootFlash_ReadbackVerify(&g_bf, APP_BASE + 1, d5, 4), BOOTFLASH_ERR_ALIGN);

    /* 2.4 偶数长度不受补字节影响 */
    CHECK_EQ_DESC("s2 even program ok",
                  BootFlash_ProgramData(&g_bf, APP_BASE + 8, d4, 4), BOOTFLASH_OK);
    CHECK_EQ_DESC("s2 even readback ok",
                  BootFlash_ReadbackVerify(&g_bf, APP_BASE + 8, d4, 4), BOOTFLASH_OK);
    CHECK_EQ_DESC("s2 even next halfword untouched",
                  ff_peek(APP_BASE + 12), 0xFFFFu);
}

/*--------------------------- 3. 状态错误立即停止 ---------------------------*/

static void s3_fail_fast(void)
{
    u8 d16[16];
    u8 d8[8];
    fill_pattern(d16, sizeof(d16), 0x30);
    fill_pattern(d8, sizeof(d8), 0x00);

    printf("== 3. backend error stops immediately, code retained ==\n");

    /* 3.1 擦除第 3 页失败：前两页已擦、后两页保留旧数据，
     *     错误码与 failAddr 保留，错误路径统一上锁 */
    harness_reset();
    junk_page(APP_BASE);
    junk_page(APP_BASE + FLASH_PAGE_SIZE);
    junk_page(APP_BASE + 2u * FLASH_PAGE_SIZE);
    junk_page(APP_BASE + 3u * FLASH_PAGE_SIZE);
    /* 布置现场的 junk_page 编程已推进 ops_done，断电点须从当前计数起算 */
    cut_after_now(2);
    CHECK_EQ_DESC("s3 erase fail code",
                  BootFlash_ErasePages(&g_bf, APP_BASE, 4), BOOTFLASH_ERR_ERASE);
    CHECK_EQ_DESC("s3 erase failAddr is 3rd page",
                  g_bf.failAddr, APP_BASE + 2u * FLASH_PAGE_SIZE);
    CHECK_EQ_DESC("s3 erase status retained",
                  g_bf.status, BOOTFLASH_ERR_ERASE);
    CHECK_EQ_DESC("s3 page0 erased", ff_peek(APP_BASE), 0xFFFFu);
    CHECK_EQ_DESC("s3 page1 erased",
                  ff_peek(APP_BASE + FLASH_PAGE_SIZE), 0xFFFFu);
    CHECK_DESC("s3 page2 keeps old data",
               ff_peek(APP_BASE + 2u * FLASH_PAGE_SIZE) != 0xFFFFu);
    CHECK_DESC("s3 page3 keeps old data",
               ff_peek(APP_BASE + 3u * FLASH_PAGE_SIZE) != 0xFFFFu);
    CHECK_EQ_DESC("s3 erase fail locked once", g_lockCnt, 1);
    CHECK_EQ_DESC("s3 erase fail unlocked once", g_unlockCnt, 1);

    /* 3.2 第 4 个半字编程失败：前 3 个半字已写、其余保持 0xFF */
    harness_reset();
    FakeFlash_SetCutAfter(&g_ff, 3);
    CHECK_EQ_DESC("s3 program fail code",
                  BootFlash_ProgramData(&g_bf, APP_BASE, d16, 16), BOOTFLASH_ERR_PROGRAM);
    CHECK_EQ_DESC("s3 program failAddr is 4th halfword",
                  g_bf.failAddr, APP_BASE + 6);
    CHECK_EQ_DESC("s3 program status retained",
                  g_bf.status, BOOTFLASH_ERR_PROGRAM);
    CHECK_EQ_DESC("s3 first 3 halfwords written",
                  ff_peek(APP_BASE), (u16)(d16[0] | ((u16)d16[1] << 8)));
    CHECK_EQ_DESC("s3 halfword 3 written",
                  ff_peek(APP_BASE + 4), (u16)(d16[4] | ((u16)d16[5] << 8)));
    CHECK_EQ_DESC("s3 halfword 4 untouched",
                  ff_peek(APP_BASE + 6), 0xFFFFu);
    CHECK_EQ_DESC("s3 tail untouched",
                  ff_peek(APP_BASE + 14), 0xFFFFu);
    CHECK_EQ_DESC("s3 program fail locked once", g_lockCnt, 1);

    /* 3.3 首个操作即失败（cut=0） */
    harness_reset();
    FakeFlash_SetCutAfter(&g_ff, 0);
    CHECK_EQ_DESC("s3 first erase fails",
                  BootFlash_ErasePages(&g_bf, APP_BASE, 1), BOOTFLASH_ERR_ERASE);
    CHECK_EQ_DESC("s3 first program fails",
                  BootFlash_ProgramData(&g_bf, APP_BASE, d8, 8), BOOTFLASH_ERR_PROGRAM);
    CHECK_EQ_DESC("s3 failAddr is start",
                  g_bf.failAddr, APP_BASE);

    /* 3.4 包级回读不一致：NACK 依据（Task 8） */
    harness_reset();
    CHECK_EQ_DESC("s3 rb setup program ok",
                  BootFlash_ProgramData(&g_bf, APP_BASE, d8, 8), BOOTFLASH_OK);
    ff_poke(APP_BASE + 4, 0x0100);   /* 0x0504 -> 0x0100（只清位，物理合法） */
    CHECK_EQ_DESC("s3 readback mismatch code",
                  BootFlash_ReadbackVerify(&g_bf, APP_BASE, d8, 8), BOOTFLASH_ERR_READBACK);
    CHECK_EQ_DESC("s3 readback failAddr is hw2",
                  g_bf.failAddr, APP_BASE + 4);
    CHECK_EQ_DESC("s3 readback status retained",
                  g_bf.status, BOOTFLASH_ERR_READBACK);
    /* 修正数据后重验通过 */
    d8[4] = 0x00; d8[5] = 0x01;
    CHECK_EQ_DESC("s3 readback corrected ok",
                  BootFlash_ReadbackVerify(&g_bf, APP_BASE, d8, 8), BOOTFLASH_OK);
}

/*--------------------------- 4. 整镜像 CRC ---------------------------*/

static void s4_image_crc(void)
{
    static u8 img[1025];
    u8 d5[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
    u8 d6[6];
    u32 crc5, crc6, crcImg;
    CRC32_StreamCtx_t sc;

    printf("== 4. whole-image CRC (VALID gate in Task 8) ==\n");

    /* 4.1 补位 0xFF 不计入镜像 CRC 的证明：
     *     CRC(5 真实字节) != CRC(5 字节 + 补 0xFF) */
    harness_reset();
    CHECK_EQ_DESC("s4 pad setup ok",
                  BootFlash_ProgramData(&g_bf, APP_BASE, d5, 5), BOOTFLASH_OK);
    memcpy(d6, d5, 5);
    d6[5] = 0xFF;
    crc5 = CRC32_Calculate(d5, 5);
    crc6 = CRC32_Calculate(d6, 6);
    CHECK_DESC("s4 crc5 != crc6 (pad matters)", crc5 != crc6);
    CHECK_EQ_DESC("s4 crc over real size ok",
                  BootFlash_VerifyCrc(&g_bf, APP_BASE, 5, crc5), BOOTFLASH_OK);
    CHECK_EQ_DESC("s4 crc excluding pad only",
                  BootFlash_VerifyCrc(&g_bf, APP_BASE, 5, crc6), BOOTFLASH_ERR_CRC);
    CHECK_EQ_DESC("s4 crc explicit size semantics",
                  BootFlash_VerifyCrc(&g_bf, APP_BASE, 6, crc6), BOOTFLASH_OK);

    /* 4.2 1025 B 奇数镜像分 512+512+1 三包写入后整镜像 CRC 通过 */
    harness_reset();
    fill_pattern(img, sizeof(img), 0x50);
    CHECK_EQ_DESC("s4 erase 2 pages ok",
                  BootFlash_ErasePages(&g_bf, APP_BASE, 2), BOOTFLASH_OK);
    CHECK_EQ_DESC("s4 chunk1 ok",
                  BootFlash_ProgramData(&g_bf, APP_BASE, img, 512), BOOTFLASH_OK);
    CHECK_EQ_DESC("s4 chunk2 ok",
                  BootFlash_ProgramData(&g_bf, APP_BASE + 512, img + 512, 512), BOOTFLASH_OK);
    CHECK_EQ_DESC("s4 final odd chunk ok",
                  BootFlash_ProgramData(&g_bf, APP_BASE + 1024, img + 1024, 1), BOOTFLASH_OK);
    CHECK_EQ_DESC("s4 chunk1 readback ok",
                  BootFlash_ReadbackVerify(&g_bf, APP_BASE, img, 512), BOOTFLASH_OK);
    CHECK_EQ_DESC("s4 chunk2 readback ok",
                  BootFlash_ReadbackVerify(&g_bf, APP_BASE + 512, img + 512, 512), BOOTFLASH_OK);
    CHECK_EQ_DESC("s4 final chunk readback ok",
                  BootFlash_ReadbackVerify(&g_bf, APP_BASE + 1024, img + 1024, 1), BOOTFLASH_OK);
    crcImg = CRC32_Calculate(img, 1025);
    CHECK_EQ_DESC("s4 odd image crc ok",
                  BootFlash_VerifyCrc(&g_bf, APP_BASE, 1025, crcImg), BOOTFLASH_OK);
    CHECK_EQ_DESC("s4 wrong crc rejected",
                  BootFlash_VerifyCrc(&g_bf, APP_BASE, 1025, crcImg ^ 1uL), BOOTFLASH_ERR_CRC);
    CHECK_EQ_DESC("s4 crc status retained",
                  g_bf.status, BOOTFLASH_ERR_CRC);

    /* 4.3 分包流式 CRC 与一次性 CRC 等价（Task 4 契约在 Flash 读回路径复验） */
    CRC32_Init(&sc);
    CRC32_Update(&sc, img, 512);
    CRC32_Update(&sc, img + 512, 512);
    CRC32_Update(&sc, img + 1024, 1);
    CHECK_EQ_DESC("s4 chunked crc equal",
                  CRC32_Final(&sc), crcImg);

    /* 4.4 镜像中途字节损坏 -> 包级回读发现（指向坏包） */
    ff_poke(APP_BASE + 600, 0x0000);   /* 清位损坏 */
    CHECK_EQ_DESC("s4 corrupt caught by chunk readback",
                  BootFlash_ReadbackVerify(&g_bf, APP_BASE + 512, img + 512, 512),
                  BOOTFLASH_ERR_READBACK);
    CHECK_EQ_DESC("s4 corrupt failAddr",
                  g_bf.failAddr, APP_BASE + 600);
    CHECK_EQ_DESC("s4 corrupt caught by image crc",
                  BootFlash_VerifyCrc(&g_bf, APP_BASE, 1025, crcImg), BOOTFLASH_ERR_CRC);
}

/*--------------------------- 5. 满容量边界 ---------------------------*/

static void s5_full_capacity(void)
{
    /* 双槽改造：容量基元从"APP_LIMIT_EXCL-APP_BASE"改为 SLOT_CAPACITY
     *（单槽容量）。本套件用 Slot A 作被测槽：Slot A 满容量写入后，
     * Slot A 终点（= SLOT_B_BASE）之前尚余整个 Slot B，故"越过槽容量"
     * 的判定不再等价于"越过 APP_LIMIT_EXCL"——写入器边界是
     * cfg.appLimitExcl（两槽合计上界），不是单槽容量。这里相应地把
     * "容量越界"改为直接在 APP_LIMIT_EXCL 处验证。 */
    static u8 big[APP_CAPACITY];
    u8 tail[3] = {0x01, 0x02, 0x03};
    u32 crcBig;
    u32 k;
    int allOk = 1;
    u32 pages = APP_CAPACITY / FLASH_PAGE_SIZE;

    printf("== 5. slot capacity / absolute limit boundary ==\n");

    /* image_size == SLOT_CAPACITY：单槽内连续页全可写
（0x1DC00/0x800 = 59.5 -> 59 页，布局再变必须同步更新本断言）
       ⚠️ SLOT_CAPACITY 不是 2 KiB 页倍数（0x1DC00 % 0x800 == 0x400）：
       它是镜像越界判定上界、非擦除单元，真正的对齐约束是 VTOR 0x200。 */
    harness_reset();
    CHECK_EQ_DESC("s5 SLOT_CAPACITY VTOR 0x200 aligned",
                  APP_CAPACITY % 0x200uL, 0uL);
    CHECK_EQ_DESC("s5 pages count is 59 (dual-slot)", pages, 59uL);
    CHECK_EQ_DESC("s5 erase full slot capacity ok",
                  BootFlash_ErasePages(&g_bf, APP_BASE, pages), BOOTFLASH_OK);

    /* 从 Slot A 起点再往后再擦 SLOT_CAPACITY+1 页 -> 仍需在
     * APP_LIMIT_EXCL 内：Slot A 起点 + 60 页 = 0x08003800+0x1E000
     * = 0x08021800，仍 < APP_LIMIT_EXCL(0x0803F000)，故**允许**。
     * 真正越界点 = APP_LIMIT_EXCL：从 SLOT_B_BASE 擦满整槽后 +1 页。 */
    CHECK_EQ_DESC("s5 erase one page past absolute limit rejected",
                  BootFlash_ErasePages(&g_bf, APP_LIMIT_EXCL - FLASH_PAGE_SIZE,
                                       pages + 1u), BOOTFLASH_ERR_BOUNDS);
    CHECK_EQ_DESC("s5 erase failAddr kept",
                  g_bf.failAddr, APP_LIMIT_EXCL - FLASH_PAGE_SIZE);
    CHECK_EQ_DESC("s5 erase exactly to absolute limit ok",
                  BootFlash_ErasePages(&g_bf, APP_LIMIT_EXCL - FLASH_PAGE_SIZE,
                                       1u), BOOTFLASH_OK);

    /* 单槽满容量 512 B 包写入 + 整镜像 CRC 通过（槽内不越界） */
    harness_reset();
    (void)BootFlash_ErasePages(&g_bf, APP_BASE, pages);
    fill_pattern(big, sizeof(big), 0x70);
    for (k = 0; k < sizeof(big) / 512; k++) {
        if (BootFlash_ProgramData(&g_bf, APP_BASE + 512u * k,
                                  big + 512u * k, 512) != BOOTFLASH_OK)
            allOk = 0;
    }
    CHECK_DESC("s5 program slot capacity chunks of 512 B", allOk);
    crcBig = CRC32_Calculate(big, sizeof(big));
    CHECK_EQ_DESC("s5 full-slot crc ok",
                  BootFlash_VerifyCrc(&g_bf, APP_BASE, (u32)sizeof(big), crcBig),
                  BOOTFLASH_OK);

    /* CRC 侧的绝对上界：从 SLOT_B_BASE 起 size=SLOT_CAPACITY+1 / +2
     * 越过 APP_LIMIT_EXCL，必须越界拒绝 */
    CHECK_EQ_DESC("s5 crc one past absolute limit rejected",
                  BootFlash_VerifyCrc(&g_bf, SLOT_B_BASE, SLOT_CAPACITY + 1u, crcBig),
                  BOOTFLASH_ERR_BOUNDS);
    CHECK_EQ_DESC("s5 crc two past absolute limit rejected",
                  BootFlash_VerifyCrc(&g_bf, SLOT_B_BASE, SLOT_CAPACITY + 2u, crcBig),
                  BOOTFLASH_ERR_BOUNDS);

    /* 终点半字重写（同值重编程物理合法）；越过终点一个字节拒绝 */
    CHECK_EQ_DESC("s5 last halfword reprogram ok",
                  BootFlash_ProgramData(&g_bf, APP_LIMIT_EXCL - 2,
                                        big + sizeof(big) - 2, 2), BOOTFLASH_OK);
    CHECK_EQ_DESC("s5 one past limit rejected",
                  BootFlash_ProgramData(&g_bf, APP_LIMIT_EXCL - 2, tail, 3),
                  BOOTFLASH_ERR_BOUNDS);
}

/*--------------------------- 6. 元数据独立 API ---------------------------*/

static void s6_meta_api(void)
{
    printf("== 6. metadata erase API restricted to A/B pages ==\n");

    /* 6.1 只能擦 A/B 两页，其余页不可通过该 API 触达 */
    harness_reset();
    junk_page(META_A_BASE);
    junk_page(META_B_BASE);
    CHECK_EQ_DESC("s6 erase metaA ok",
                  BootFlash_EraseMetaPage(&g_bf, BOOTFLASH_META_A), BOOTFLASH_OK);
    CHECK_EQ_DESC("s6 metaA erased", ff_peek(META_A_BASE), 0xFFFFu);
    CHECK_DESC("s6 metaB keeps junk", ff_peek(META_B_BASE) != 0xFFFFu);
    CHECK_EQ_DESC("s6 erase metaB ok",
                  BootFlash_EraseMetaPage(&g_bf, BOOTFLASH_META_B), BOOTFLASH_OK);
    CHECK_EQ_DESC("s6 metaB erased", ff_peek(META_B_BASE), 0xFFFFu);

    /* 6.2 非法选择子拒绝 */
    CHECK_EQ_DESC("s6 meta selector 2 rejected",
                  BootFlash_EraseMetaPage(&g_bf, 2), BOOTFLASH_ERR_PARAM);
    CHECK_EQ_DESC("s6 meta selector 0xFF rejected",
                  BootFlash_EraseMetaPage(&g_bf, 0xFF), BOOTFLASH_ERR_PARAM);

    /* 6.3 元数据页擦除失败：错误码 + failAddr + 错误路径上锁 */
    harness_reset();
    junk_page(META_A_BASE);
    FakeFlash_SetCutAfter(&g_ff, 0);
    CHECK_EQ_DESC("s6 meta erase fail code",
                  BootFlash_EraseMetaPage(&g_bf, BOOTFLASH_META_A), BOOTFLASH_ERR_ERASE);
    CHECK_EQ_DESC("s6 meta erase failAddr",
                  g_bf.failAddr, META_A_BASE);
    CHECK_EQ_DESC("s6 meta erase locked on error", g_lockCnt, 1);
    CHECK_DESC("s6 metaA keeps junk on fail",
               ff_peek(META_A_BASE) != 0xFFFFu);
}

/*--------------------------- 7. 解锁纪律与 NULL 安全 ---------------------------*/

static void s7_unlock_discipline(void)
{
    static u8 buf[512];
    u8 d[4] = {0x01, 0x02, 0x03, 0x04};
    BootFlash_t bf2;

    printf("== 7. unlock-once discipline & NULL safety ==\n");

    fill_pattern(buf, sizeof(buf), 0x90);

    /* 7.1 512 B = 256 个半字编程只解锁一次 */
    harness_reset();
    CHECK_EQ_DESC("s7 erase for program ok",
                  BootFlash_ErasePages(&g_bf, APP_BASE, 1), BOOTFLASH_OK);
    CHECK_EQ_DESC("s7 unlock once for erase", g_unlockCnt, 1);
    CHECK_EQ_DESC("s7 512B program ok",
                  BootFlash_ProgramData(&g_bf, APP_BASE, buf, 512), BOOTFLASH_OK);
    CHECK_EQ_DESC("s7 unlock once for 256 halfwords", g_unlockCnt, 2);
    CHECK_EQ_DESC("s7 lock paired", g_lockCnt, 2);
    CHECK_EQ_DESC("s7 ops are halfword-wise",
                  g_ff.program_count, 256uL);   /* 无 1 KiB 页镜像读改写 */

    /* 7.2 满容量页连续擦除仍只解锁一次（Task 16 CB 冻结：112 页，
       页数从布局派生，避免再硬编码第二个布局事实） */
    harness_reset();
    CHECK_EQ_DESC("s7 full-capacity page erase ok",
                  BootFlash_ErasePages(&g_bf, APP_BASE,
                                       APP_CAPACITY / FLASH_PAGE_SIZE), BOOTFLASH_OK);
    CHECK_EQ_DESC("s7 unlock once for all pages", g_unlockCnt, 1);
    CHECK_EQ_DESC("s7 lock once for all pages", g_lockCnt, 1);

    /* 7.3 只读操作不解锁 */
    harness_reset();
    CHECK_EQ_DESC("s7 read setup ok",
                  BootFlash_ProgramData(&g_bf, APP_BASE, d, 4), BOOTFLASH_OK);
    g_unlockCnt = 0;
    g_lockCnt = 0;
    CHECK_EQ_DESC("s7 readback ok",
                  BootFlash_ReadbackVerify(&g_bf, APP_BASE, d, 4), BOOTFLASH_OK);
    CHECK_EQ_DESC("s7 crc ok",
                  BootFlash_VerifyCrc(&g_bf, APP_BASE, 4,
                                      CRC32_Calculate(d, 4)), BOOTFLASH_OK);
    CHECK_EQ_DESC("s7 read ops never unlock", g_unlockCnt, 0);
    CHECK_EQ_DESC("s7 read ops never lock", g_lockCnt, 0);

    /* 7.4 参数错误不解锁（未触碰 Flash） */
    CHECK_EQ_DESC("s7 NULL data rejected",
                  BootFlash_ProgramData(&g_bf, APP_BASE, NULL, 4), BOOTFLASH_ERR_PARAM);
    CHECK_EQ_DESC("s7 zero len rejected",
                  BootFlash_ProgramData(&g_bf, APP_BASE, d, 0), BOOTFLASH_ERR_PARAM);
    CHECK_EQ_DESC("s7 NULL data readback rejected",
                  BootFlash_ReadbackVerify(&g_bf, APP_BASE, NULL, 4), BOOTFLASH_ERR_PARAM);
    CHECK_EQ_DESC("s7 param errors never unlock", g_unlockCnt, 0);

    /* 7.5 NULL 上下文安全 */
    BootFlash_Init(NULL, &g_io, &g_cfg);
    CHECK_EQ_DESC("s7 NULL ctx erase safe",
                  BootFlash_ErasePages(NULL, APP_BASE, 1), BOOTFLASH_ERR_PARAM);
    CHECK_EQ_DESC("s7 NULL ctx program safe",
                  BootFlash_ProgramData(NULL, APP_BASE, d, 4), BOOTFLASH_ERR_PARAM);
    CHECK_EQ_DESC("s7 NULL ctx readback safe",
                  BootFlash_ReadbackVerify(NULL, APP_BASE, d, 4), BOOTFLASH_ERR_PARAM);
    CHECK_EQ_DESC("s7 NULL ctx crc safe",
                  BootFlash_VerifyCrc(NULL, APP_BASE, 4, 0), BOOTFLASH_ERR_PARAM);
    CHECK_EQ_DESC("s7 NULL ctx meta safe",
                  BootFlash_EraseMetaPage(NULL, BOOTFLASH_META_A), BOOTFLASH_ERR_PARAM);

    /* 7.6 io=NULL 上下文安全 */
    BootFlash_Init(&bf2, NULL, &g_cfg);
    CHECK_EQ_DESC("s7 NULL io erase safe",
                  BootFlash_ErasePages(&bf2, APP_BASE, 1), BOOTFLASH_ERR_PARAM);
    CHECK_EQ_DESC("s7 NULL io program safe",
                  BootFlash_ProgramData(&bf2, APP_BASE, d, 4), BOOTFLASH_ERR_PARAM);
    CHECK_EQ_DESC("s7 NULL io crc safe",
                  BootFlash_VerifyCrc(&bf2, APP_BASE, 4, 0), BOOTFLASH_ERR_PARAM);
    CHECK_EQ_DESC("s7 NULL io meta safe",
                  BootFlash_EraseMetaPage(&bf2, BOOTFLASH_META_A), BOOTFLASH_ERR_PARAM);

    /* 7.7 NULL config 安全 */
    BootFlash_Init(&bf2, &g_io, NULL);
    CHECK_EQ_DESC("s7 NULL cfg erase safe",
                  BootFlash_ErasePages(&bf2, APP_BASE, 1), BOOTFLASH_ERR_PARAM);
}

/*--------------------------- main ---------------------------*/

int main(void)
{
    s1_bounds();
    s2_align_parity();
    s3_fail_fast();
    s4_image_crc();
    s5_full_capacity();
    s6_meta_api();
    s7_unlock_discipline();

    printf("boot-flash: %d checks, %d failed\n", g_total, g_failed);
    if (g_failed == 0) {
        printf("boot-flash: all cases passed\n");
        return 0;
    }
    printf("boot-flash: some cases FAILED\n");
    return 1;
}
