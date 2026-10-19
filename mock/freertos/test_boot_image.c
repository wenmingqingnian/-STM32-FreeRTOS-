// SPDX-License-Identifier: GPL-2.0
/*
 * Host-side mock test for the app-image validation chain
 * Covers:
 *
 *   1. MSP boundary: 0x20000000 / 0x20005000 both accepted (top is
 *      inclusive: APP initial MSP == SRAM top), not-8B-aligned, one
 *      byte below and one byte above rejected
 *   2. Reset Handler bit0=0 rejected; after clearing the Thumb bit the
 *      PC must fall in [APP_BASE, APP_BASE+image_size): first address
 *      and last valid even address accepted, first address outside
 *      rejected (range decided on the CLEARED value)
 *   3. image_size 7 / 0 / APP_CAPACITY+1 rejected; app_base mismatch
 *      rejected
 *   4. VALID record but whole-image CRC mismatch rejected; RECEIVING
 *      record rejected even with perfect vectors and CRC
 *   5. Chain order per design §5.5: state -> size -> base -> MSP -> PC
 *      -> CRC (each failure provably takes precedence over the rest)
 *   6. odd image_size: CRC covers only real bytes (0xFF pad excluded)
 *
 * Production sources under test are compiled UNMODIFIED from the project
 * tree (BOOT\boot_image.c + BSP\CRC32\crc32.c); sys.h comes from stub/,
 * flash physics (page erase, 1->0 program) from fake_flash.c behind a
 * plain read IO so the validator sees memory-mapped halfwords.
 */
#include <stdio.h>
#include <string.h>
#include "boot_image.h"
#include "crc32.h"
#include "fake_flash.h"
#include "iap_meta.h"
#include "memory_layout.h"

static int g_failed = 0;
static int g_total = 0;

#define CHECK_DESC(desc, cond) do {                      \
    g_total++;                                            \
    if (cond) { printf("  ok %s\n", desc); }             \
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

static FakeFlash_t g_ff;

#define OFF(x) ((x) - FLASH_BASE)

static u16 w_read(void *self, u32 addr)
{
    return FakeFlash_ReadHalfword((const FakeFlash_t *)self, OFF(addr));
}

static Boot_ImageIo_t g_io = { &g_ff, w_read };

/* SRAM facts (design §4 / board_profile.h; host stub has no SRAM_BASE):
 * 0x20000000 <= MSP <= 0x20005000 inclusive, 8 B aligned */
static const Boot_ImageConfig_t g_cfg = {
    APP_BASE,       /* appBase */
    APP_CAPACITY,   /* appCapacity */
    0x20000000UL,   /* sramBase */
    0x20005000UL    /* sramLimit（MSP 可等于：APP 初始 MSP=SRAM 顶）*/
};

/*--------------------------- harness ---------------------------*/

#define MAX_IMAGE  1024u   /* 本测试最大镜像字节数（<= APP_CAPACITY） */

static IAP_MetaRecord_t g_rec;
static u8 g_img[MAX_IMAGE];

static void fill_image(u32 size, u32 msp, u32 pc, u8 seed)
{
    u32 i;
    for (i = 0; i < size; i++)
        g_img[i] = (u8)(seed + (u8)i);
    g_img[0] = (u8)(msp & 0xFFu);
    g_img[1] = (u8)((msp >> 8) & 0xFFu);
    g_img[2] = (u8)((msp >> 16) & 0xFFu);
    g_img[3] = (u8)((msp >> 24) & 0xFFu);
    g_img[4] = (u8)(pc & 0xFFu);
    g_img[5] = (u8)((pc >> 8) & 0xFFu);
    g_img[6] = (u8)((pc >> 16) & 0xFFu);
    g_img[7] = (u8)((pc >> 24) & 0xFFu);
}

/* 布置现场：RAM 镜像 -> CRC -> 逐半字编程进 fake_flash（奇数末字节高
 * 半字节保持擦除态 0xFF，与 BootFlash_ProgramData 奇补规则一致）*/
static u32 program_image(u32 size)
{
    u32 i;
    u32 hw;
    u32 crc;

    crc = CRC32_Calculate(g_img, size);
    for (i = 0; i < ((size + 1u) / 2u); i++)
    {
        hw = g_img[2u * i];
        if ((2u * i + 1u) < size)
            hw |= ((u32)g_img[2u * i + 1u]) << 8;
        else
            hw |= 0xFF00u;   /* 奇数末字节：0xFF 填充不入 CRC */
        (void)FakeFlash_ProgramHalfword(&g_ff, OFF(APP_BASE) + 2u * i,
                                        (u16)hw);
    }
    return crc;
}

/* 默认合法记录：VALID + 匹配字段（stateWord/size/base 可后置改写）*/
static void reset_record(u32 size, u32 crc)
{
    memset(&g_rec, 0, sizeof(g_rec));
    g_rec.valid       = 1u;
    g_rec.stateWord   = IAP_META_STATE_VALID;
    g_rec.appBase     = APP_BASE;
    g_rec.imageSize   = size;
    g_rec.imageCrc    = crc;
    g_rec.imageVersion = 0x0100u;
}

/* 全新现场：擦净 Flash + 合法镜像（0x100 B，MSP=SRAM 顶，PC=+0x100|1）*/
static void harness_default(u32 size)
{
    u32 crc;
    fill_image(size, 0x20005000UL, (APP_BASE + 0x40u) | 1u, 0x20);
    FakeFlash_Init(&g_ff);
    FakeFlash_DisableCut(&g_ff);
    crc = program_image(size);
    reset_record(size, crc);
}

/*--------------------------- s1: 全链基准 ---------------------------*/

static void s1_baseline(void)
{
    printf("s1: full-chain baseline (VALID + vectors + CRC)\n");
    harness_default(0x100u);
    CHECK_EQ_DESC("s1 valid image passes",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg), BOOT_IMAGE_OK);
}

/*--------------------------- s2: MSP ---------------------------*/

static void s2_msp(void)
{
    printf("s2: initial MSP boundaries and alignment\n");

    harness_default(0x100u);
    fill_image(0x100u, 0x20000000UL, (APP_BASE + 0x40u) | 1u, 0x20);
    FakeFlash_Init(&g_ff);
    FakeFlash_DisableCut(&g_ff);
    g_rec.imageCrc = program_image(0x100u);
    CHECK_EQ_DESC("s2 MSP=0x20000000 (SRAM bottom) accepted",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg), BOOT_IMAGE_OK);

    harness_default(0x100u);
    CHECK_EQ_DESC("s2 MSP=0x20005000 (SRAM top, inclusive) accepted",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg), BOOT_IMAGE_OK);

    harness_default(0x100u);
    fill_image(0x100u, 0x20004FF8UL, (APP_BASE + 0x40u) | 1u, 0x20);
    FakeFlash_Init(&g_ff);
    FakeFlash_DisableCut(&g_ff);
    g_rec.imageCrc = program_image(0x100u);
    CHECK_EQ_DESC("s2 MSP=0x20004FF8 (top-8, aligned) accepted",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg), BOOT_IMAGE_OK);

    harness_default(0x100u);
    fill_image(0x100u, 0x20000004UL, (APP_BASE + 0x40u) | 1u, 0x20);
    FakeFlash_Init(&g_ff);
    FakeFlash_DisableCut(&g_ff);
    g_rec.imageCrc = program_image(0x100u);
    CHECK_EQ_DESC("s2 MSP=0x20000004 (not 8 B aligned) rejected",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_MSP);

    harness_default(0x100u);
    fill_image(0x100u, 0x1FFFFFFFUL, (APP_BASE + 0x40u) | 1u, 0x20);
    FakeFlash_Init(&g_ff);
    FakeFlash_DisableCut(&g_ff);
    g_rec.imageCrc = program_image(0x100u);
    CHECK_EQ_DESC("s2 MSP=0x1FFFFFFF (1 B below SRAM) rejected",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_MSP);

    harness_default(0x100u);
    fill_image(0x100u, 0x20005001UL, (APP_BASE + 0x40u) | 1u, 0x20);
    FakeFlash_Init(&g_ff);
    FakeFlash_DisableCut(&g_ff);
    g_rec.imageCrc = program_image(0x100u);
    CHECK_EQ_DESC("s2 MSP=0x20005001 (1 B above SRAM top) rejected",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_MSP);
}

/*--------------------------- s3: Reset Handler / PC ---------------------------*/

static void s3_pc(void)
{
    printf("s3: reset handler Thumb bit and cleared-PC range\n");

    /* bit0=0：非 Thumb 拒绝 */
    harness_default(0x100u);
    fill_image(0x100u, 0x20005000UL, (APP_BASE + 0x100u) & ~((u32)1u), 0x20);
    FakeFlash_Init(&g_ff);
    FakeFlash_DisableCut(&g_ff);
    g_rec.imageCrc = program_image(0x100u);
    CHECK_EQ_DESC("s3 PC bit0=0 rejected",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_PC);

    /* 清 bit0 后=APP 首地址：接受 */
    harness_default(0x100u);
    fill_image(0x100u, 0x20005000UL, APP_BASE | 1u, 0x20);
    FakeFlash_Init(&g_ff);
    FakeFlash_DisableCut(&g_ff);
    g_rec.imageCrc = program_image(0x100u);
    CHECK_EQ_DESC("s3 cleared PC=APP_BASE (first address) accepted",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg), BOOT_IMAGE_OK);

    /* 清 bit0 后=镜像内最后合法（偶）地址：接受 */
    harness_default(0x100u);
    fill_image(0x100u, 0x20005000UL,
               (APP_BASE + 0x100u - 2u) | 1u, 0x20);
    FakeFlash_Init(&g_ff);
    FakeFlash_DisableCut(&g_ff);
    g_rec.imageCrc = program_image(0x100u);
    CHECK_EQ_DESC("s3 cleared PC=APP_BASE+size-2 (last valid) accepted",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg), BOOT_IMAGE_OK);

    /* bit0=1 但清后=终点（镜像外第一地址）：拒绝——证明范围判定用清后值 */
    harness_default(0x100u);
    fill_image(0x100u, 0x20005000UL, (APP_BASE + 0x100u) | 1u, 0x21);
    FakeFlash_Init(&g_ff);
    FakeFlash_DisableCut(&g_ff);
    g_rec.imageCrc = program_image(0x100u);
    CHECK_EQ_DESC("s3 cleared PC=APP_BASE+size (first outside) rejected",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_PC);

    /* 清 bit0 后=APP_BASE 前一地址：拒绝 */
    harness_default(0x100u);
    fill_image(0x100u, 0x20005000UL, (APP_BASE - 2u) | 1u, 0x20);
    FakeFlash_Init(&g_ff);
    FakeFlash_DisableCut(&g_ff);
    g_rec.imageCrc = program_image(0x100u);
    CHECK_EQ_DESC("s3 cleared PC=APP_BASE-2 rejected",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_PC);
}

/*--------------------------- s4: size / base ---------------------------*/

static void s4_size_base(void)
{
    printf("s4: image_size and app_base checks\n");

    harness_default(0x100u);
    g_rec.imageSize = 7u;
    CHECK_EQ_DESC("s4 image_size=7 rejected",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_SIZE);

    harness_default(0x100u);
    g_rec.imageSize = 0u;
    CHECK_EQ_DESC("s4 image_size=0 rejected",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_SIZE);

    harness_default(0x100u);
    g_rec.imageSize = APP_CAPACITY + 1u;
    CHECK_EQ_DESC("s4 image_size=APP_CAPACITY+1 rejected",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_SIZE);

    harness_default(0x100u);
    g_rec.appBase = APP_BASE + 0x400u;
    CHECK_EQ_DESC("s4 app_base mismatch rejected",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_BASE);

    /* 最小 size=8：恰好装下向量表（PC 必须同样落在 8 B 镜像内，
     * harness_default 的默认 PC=+0x40 对 size=8 越界，故重布现场）*/
    harness_default(8u);
    fill_image(8u, 0x20005000UL, (APP_BASE + 4u) | 1u, 0x20);
    FakeFlash_Init(&g_ff);
    FakeFlash_DisableCut(&g_ff);
    g_rec.imageCrc = program_image(8u);
    CHECK_EQ_DESC("s4 image_size=8 (vectors only) accepted",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg), BOOT_IMAGE_OK);
}

/*--------------------------- s5: state / param / order ---------------------------*/

static void s5_state_order(void)
{
    printf("s5: record state, params and chain order\n");

    /* RECEIVING：即使向量与 CRC 全部正确也拒绝 */
    harness_default(0x100u);
    g_rec.stateWord = IAP_META_STATE_RECEIVING;
    CHECK_EQ_DESC("s5 RECEIVING rejected despite perfect image",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_STATE);

    /* 记录解码无效（valid=0）：拒绝 */
    harness_default(0x100u);
    g_rec.valid = 0u;
    CHECK_EQ_DESC("s5 undecodable record (valid=0) rejected",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_STATE);

    /* PARAM：NULL 指针 */
    harness_default(0x100u);
    CHECK_EQ_DESC("s5 NULL record rejected",
                  Boot_ImageValidate(NULL, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_PARAM);
    CHECK_EQ_DESC("s5 NULL io rejected",
                  Boot_ImageValidate(&g_rec, NULL, &g_cfg),
                  BOOT_IMAGE_ERR_PARAM);
    CHECK_EQ_DESC("s5 NULL cfg rejected",
                  Boot_ImageValidate(&g_rec, &g_io, NULL),
                  BOOT_IMAGE_ERR_PARAM);

    /* 链序：state 最先 */
    harness_default(0x100u);
    g_rec.stateWord = IAP_META_STATE_RECEIVING;
    g_rec.imageSize = 7u;
    CHECK_EQ_DESC("s5 state checked before size",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_STATE);

    /* 链序：size 在 base 之前 */
    harness_default(0x100u);
    g_rec.imageSize = 7u;
    g_rec.appBase = APP_BASE + 0x400u;
    CHECK_EQ_DESC("s5 size checked before base",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_SIZE);

    /* 链序：base 在 MSP 之前（改坏 MSP 且改坏 base） */
    harness_default(0x100u);
    g_rec.appBase = APP_BASE + 0x400u;
    fill_image(0x100u, 0x20000004UL, (APP_BASE + 0x40u) | 1u, 0x20);
    FakeFlash_Init(&g_ff);
    FakeFlash_DisableCut(&g_ff);
    g_rec.imageCrc = program_image(0x100u);
    CHECK_EQ_DESC("s5 base checked before MSP",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_BASE);

    /* 链序：MSP 在 PC 之前（MSP 与 PC 同时改坏） */
    harness_default(0x100u);
    fill_image(0x100u, 0x20000004UL, (APP_BASE + 0x100u) & ~((u32)1u), 0x20);
    FakeFlash_Init(&g_ff);
    FakeFlash_DisableCut(&g_ff);
    g_rec.imageCrc = program_image(0x100u);
    CHECK_EQ_DESC("s5 MSP checked before PC",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_MSP);

    /* 链序：PC 在 CRC 之前（PC 与 CRC 同时错） */
    harness_default(0x100u);
    fill_image(0x100u, 0x20005000UL, (APP_BASE + 0x100u) & ~((u32)1u), 0x20);
    FakeFlash_Init(&g_ff);
    FakeFlash_DisableCut(&g_ff);
    g_rec.imageCrc = program_image(0x100u) ^ 0xDEADBEEFu;
    CHECK_EQ_DESC("s5 PC checked before CRC",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_PC);
}

/*--------------------------- s6: whole-image CRC ---------------------------*/

static void s6_crc(void)
{
    printf("s6: whole-image CRC (incl. vectors; odd size excludes 0xFF pad)\n");

    /* 记录 CRC 与实际不符：拒绝 */
    harness_default(0x100u);
    g_rec.imageCrc ^= 0x1u;
    CHECK_EQ_DESC("s6 CRC mismatch rejected",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_CRC);

    /* Flash 内容被改坏（向量区）：CRC 拒绝。改写必须只清位（1->0）：
     * MSP 低半字 0x5000 -> 0x4000，MSP 变为 0x20004000 仍对齐且在
     * SRAM 内（不触发 MSP 级短路），保证由 CRC 级捕获 */
    harness_default(0x100u);
    {
        u16 hw;
        hw = (u16)(0x20004000UL & 0xFFFFu);   /* MSP 低半字 0x5000->0x4000 */
        (void)FakeFlash_ProgramHalfword(&g_ff, OFF(APP_BASE), hw);
    }
    CHECK_EQ_DESC("s6 corrupted vector word rejected by CRC",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_CRC);

    /* 奇数 size：CRC 只覆盖真实字节（0xFF 填充不计入） */
    harness_default(0x101u);
    CHECK_EQ_DESC("s6 odd size (0x101 B) with real-byte CRC accepted",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg), BOOT_IMAGE_OK);

    /* 奇数 size 但记录值按含填充计算（错误值）：拒绝 */
    harness_default(0x101u);
    g_rec.imageCrc ^= 0x1u;
    CHECK_EQ_DESC("s6 odd size wrong CRC rejected",
                  Boot_ImageValidate(&g_rec, &g_io, &g_cfg),
                  BOOT_IMAGE_ERR_CRC);
}

int main(void)
{
    s1_baseline();
    s2_msp();
    s3_pc();
    s4_size_base();
    s5_state_order();
    s6_crc();

    printf("boot-image mock: %d checks, %d failed\n", g_total, g_failed);
    return (g_failed == 0) ? 0 : 1;
}
