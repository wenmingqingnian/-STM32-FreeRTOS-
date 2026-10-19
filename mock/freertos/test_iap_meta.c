// SPDX-License-Identifier: GPL-2.0
/*
 * Host-side mock test for the dual-page power-fail-safe IAP metadata
 * Covers:
 *
 *   1. explicit little-endian byte encoding round-trip (no struct memcpy)
 *   2. record CRC / commit marker / magic / format / header size damage
 *      -> record invalid
 *   3. A/B page selection: one VALID vs one RECEIVING/VALID/corrupted
 *      page -> latest generation wins
 *   4. generation wraparound 0xFFFFFFFE -> 0xFFFFFFFF -> 0 -> 1 and the
 *      0x80000000 ambiguity rejection
 *   5. fake flash enforces 1->0-only programming (0->1 must fail)
 *   6. power-cut injection at EVERY erase/halfword-program position of
 *      both the RECEIVING ("start") record and the VALID ("confirm")
 *      record: a partially written APP can never boot
 *   7. cut before the START record commits -> old VALID still boots;
 *      cut after RECEIVING committed -> boot must stay in the bootloader
 *
 * Production sources under test are compiled UNMODIFIED from the project
 * tree (BSP/IAP/iap_meta.c, BSP/CRC32/crc32.c); sys.h comes from stub/.
 * Flash physical behaviour (page erase, 1->0 program, power-cut) comes
 * from fake_flash.c.
 */
#include <stdio.h>
#include <string.h>
#include "iap_meta.h"
#include "crc32.h"
#include "fake_flash.h"
#include "memory_layout.h"

#define REC_BYTES       IAP_META_RECORD_SIZE

/* 会话操作预算：每条记录 = 1 次擦除 + 19 次半字编程（38 B / 2） */
#define OPS_PER_RECORD  20uL
#define SESSION_OPS     (2uL * OPS_PER_RECORD)

typedef struct {
    IAP_BootAction_t boot;
    IAP_MetaSelectResult_t select;
    const IAP_MetaRecord_t *selRec;
    IAP_MetaRecord_t recA;   /* A 页解码视图（META_A_BASE） */
    IAP_MetaRecord_t recB;   /* B 页解码视图（META_B_BASE） */
} RestartView_t;

static int g_failed = 0;
static int g_total = 0;

#define CHECK_DESC(desc, cond) do {                     \
    g_total++;                                          \
    if (cond) { printf("  ok %s\n", desc); }            \
    else      { g_failed++; printf("  FAIL %s\n", desc); } \
} while (0)

#define CHECK_EQ_DESC(desc, got, want) do {             \
    g_total++;                                          \
    if ((unsigned long)(got) == (unsigned long)(want))  \
        printf("  ok %s\n", desc);                      \
    else { g_failed++;                                  \
        printf("  FAIL %s: got %lu, want %lu\n",        \
               desc, (unsigned long)(got),              \
               (unsigned long)(want)); }                \
} while (0)

/*--------------------------- fake flash 后端 ---------------------------*/
static FakeFlash_t g_flash;

/* 绝对地址（memory_layout.h） -> fake flash 片内偏移 */
static int io_erase(void *self, u32 page_addr)
{
    return FakeFlash_ErasePage(self, page_addr - FLASH_BASE);
}

static int io_program(void *self, u32 addr, u16 value)
{
    return FakeFlash_ProgramHalfword(self, addr - FLASH_BASE, value);
}

static u16 io_read(void *self, u32 addr)
{
    return FakeFlash_ReadHalfword((const FakeFlash_t *)self, addr - FLASH_BASE);
}

static IAP_MetaIo_t g_io = { &g_flash, io_erase, io_program, io_read };

/* 构造纯逻辑记录（供 Select/BootDecision 矩阵测试） */
static IAP_MetaRecord_t mkrec(u32 gen, u16 stateWord)
{
    IAP_MetaRecord_t r;
    memset(&r, 0, sizeof(r));
    r.magic         = IAP_META_MAGIC;
    r.formatVersion = IAP_META_FORMAT_VERSION;
    r.headerSize    = (u16)IAP_META_RECORD_SIZE;
    r.generation    = gen;
    r.stateWord     = stateWord;
    r.slotId        = IAP_META_SLOT_A;
    r.appBase       = SLOT_A_BASE;
    r.imageSize     = 4096uL;
    r.imageCrc      = 0xDEADBEEFuL;
    r.imageVersion  = 1uL;
    r.recordCrc     = 0x11223344uL;
    r.commitMarker  = IAP_META_COMMIT_DONE;
    r.valid         = 1u;
    return r;
}

/* "重启"：从双页读回并做选择/启动决策 */
static void boot_view(RestartView_t *v)
{
    u8 bufA[REC_BYTES];
    u8 bufB[REC_BYTES];

    IAP_MetaReadRecord(&g_io, META_A_BASE, bufA);
    IAP_MetaReadRecord(&g_io, META_B_BASE, bufB);
    IAP_MetaDecodeRecord(bufA, &v->recA);
    IAP_MetaDecodeRecord(bufB, &v->recB);
    v->select = IAP_MetaSelectLatest(&v->recA, &v->recB, &v->selRec);
    v->boot   = IAP_BootDecision(&v->recA, &v->recB,
                                 SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY);
}

/*--------------------------- 1. 编码布局 ---------------------------*/

static void test_encode_layout(void)
{
    IAP_MetaRecordFields_t f;
    IAP_MetaRecord_t rec;
    u8 buf[REC_BYTES];
    u32 crcExpected;
    u32 v32;

    printf("suite meta-encode-layout\n");

    f.generation   = 0x89ABCDEFuL;
    f.stateWord    = IAP_META_STATE_VALID;
    f.slotId       = IAP_META_SLOT_A;
    f.appBase      = SLOT_A_BASE;           /* 0x08003800（双槽布局 Slot A） */
    f.imageSize    = 4096uL;                /* 0x00001000 */
    f.imageCrc     = 0xCAFEBABEuL;
    f.imageVersion = 0x01020304uL;

    IAP_MetaEncodeRecord(&f, buf);

    /* 显式小端字节布局：逐字节断言固定字段位置（非 memcpy(struct)） */
    CHECK_EQ_DESC("magic[0] == 'I' (0x49)", buf[0], 0x49);
    CHECK_EQ_DESC("magic[1] == 'P' (0x50)", buf[1], 0x50);
    CHECK_EQ_DESC("magic[2] == 'A' (0x41)", buf[2], 0x41);
    CHECK_EQ_DESC("magic[3] == '2' (0x32)", buf[3], 0x32);
    CHECK_EQ_DESC("format_version lo == 0x02", buf[4], 0x02);
    CHECK_EQ_DESC("format_version hi == 0x00", buf[5], 0x00);
    CHECK_EQ_DESC("header_size lo == 38", buf[6], 38);
    CHECK_EQ_DESC("header_size hi == 0", buf[7], 0);
    CHECK_EQ_DESC("generation b0 == 0xEF", buf[8], 0xEF);
    CHECK_EQ_DESC("generation b1 == 0xCD", buf[9], 0xCD);
    CHECK_EQ_DESC("generation b2 == 0xAB", buf[10], 0xAB);
    CHECK_EQ_DESC("generation b3 == 0x89", buf[11], 0x89);
    CHECK_EQ_DESC("state lo", buf[12], IAP_META_STATE_VALID & 0xFFu);
    CHECK_EQ_DESC("state hi", buf[13], (IAP_META_STATE_VALID >> 8) & 0xFFu);
    CHECK_EQ_DESC("slot_id lo == 1 (SLOT_A)", buf[14], 0x01);
    CHECK_EQ_DESC("slot_id hi == 0", buf[15], 0x00);
    CHECK_EQ_DESC("app_base b0", buf[16], (u8)((SLOT_A_BASE >> 0) & 0xFFu));
    CHECK_EQ_DESC("app_base b1", buf[17], (u8)((SLOT_A_BASE >> 8) & 0xFFu));
    CHECK_EQ_DESC("app_base b2", buf[18], (u8)((SLOT_A_BASE >> 16) & 0xFFu));
    CHECK_EQ_DESC("app_base b3", buf[19], (u8)((SLOT_A_BASE >> 24) & 0xFFu));
    CHECK_EQ_DESC("image_size b0 == 0x00", buf[20], 0x00);
    CHECK_EQ_DESC("image_size b1 == 0x10", buf[21], 0x10);
    CHECK_EQ_DESC("image_size b2 == 0x00", buf[22], 0x00);
    CHECK_EQ_DESC("image_size b3 == 0x00", buf[23], 0x00);
    CHECK_EQ_DESC("image_crc b0 == 0xBE", buf[24], 0xBE);
    CHECK_EQ_DESC("image_crc b3 == 0xCA", buf[27], 0xCA);
    CHECK_EQ_DESC("image_version LE == 0x01020304",
        (u32)((u32)buf[28] | ((u32)buf[29] << 8) |
              ((u32)buf[30] << 16) | ((u32)buf[31] << 24)), 0x01020304uL);
    CHECK_EQ_DESC("commit LE lo == 0xCC", buf[36], 0xCC);
    CHECK_EQ_DESC("commit LE hi == 0x33", buf[37], 0x33);

    /* record CRC 覆盖 [0,32)：与生产 CRC32 实现逐字节一致 */
    crcExpected = CRC32_Calculate(buf, 32uL);
    v32 = (u32)((u32)buf[32] | ((u32)buf[33] << 8) |
                ((u32)buf[34] << 16) | ((u32)buf[35] << 24));
    CHECK_EQ_DESC("record_crc field == CRC32(buf[0..32))", v32, crcExpected);

    /* round-trip */
    IAP_MetaDecodeRecord(buf, &rec);
    CHECK_DESC("decode valid", rec.valid);
    CHECK_EQ_DESC("generation round-trip", rec.generation, 0x89ABCDEFuL);
    CHECK_EQ_DESC("stateWord round-trip", rec.stateWord, IAP_META_STATE_VALID);
    CHECK_EQ_DESC("slotId round-trip", rec.slotId, IAP_META_SLOT_A);
    CHECK_EQ_DESC("appBase round-trip", rec.appBase, SLOT_A_BASE);
    CHECK_EQ_DESC("imageSize round-trip", rec.imageSize, 4096uL);
    CHECK_EQ_DESC("imageCrc round-trip", rec.imageCrc, 0xCAFEBABEuL);
    CHECK_EQ_DESC("imageVersion round-trip", rec.imageVersion, 0x01020304uL);
    CHECK_EQ_DESC("recordCrc round-trip", rec.recordCrc, crcExpected);
    CHECK_EQ_DESC("commitMarker round-trip", rec.commitMarker, IAP_META_COMMIT_DONE);

    /* 0xFFFFFFFF generation 全字节 0xFF 不丢失 */
    f.generation = 0xFFFFFFFFuL;
    IAP_MetaEncodeRecord(&f, buf);
    IAP_MetaDecodeRecord(buf, &rec);
    CHECK_EQ_DESC("generation 0xFFFFFFFF round-trip", rec.generation, 0xFFFFFFFFuL);
    CHECK_DESC("still valid with all-ones generation", rec.valid);

    /* 全 0xFF 页（从未编程）解码无效 */
    memset(buf, 0xFF, sizeof(buf));
    IAP_MetaDecodeRecord(buf, &rec);
    CHECK_DESC("erased page decodes invalid", !rec.valid);

    /* NULL 容忍 */
    IAP_MetaEncodeRecord(NULL, buf);
    CHECK_DESC("Encode(NULL fields) tolerated", 1);
    IAP_MetaEncodeRecord(&f, NULL);
    CHECK_DESC("Encode(NULL buf) tolerated", 1);
    IAP_MetaDecodeRecord(buf, NULL);
    CHECK_DESC("Decode(NULL out) tolerated", 1);
    memset(&rec, 0, sizeof(rec));
    IAP_MetaDecodeRecord(NULL, &rec);
    CHECK_DESC("Decode(NULL buf) -> invalid", !rec.valid);
}

/*--------------------------- 2. 字段损坏 ---------------------------*/

static void test_field_corruption(void)
{
    IAP_MetaRecordFields_t f;
    IAP_MetaRecord_t rec;
    u8 ref[REC_BYTES];
    u8 buf[REC_BYTES];

    printf("suite meta-field-corruption\n");

    f.generation   = 5uL;
    f.stateWord    = IAP_META_STATE_VALID;
    f.slotId       = IAP_META_SLOT_A;
    f.appBase      = SLOT_A_BASE;
    f.imageSize    = 4096uL;
    f.imageCrc     = 0x11223344uL;
    f.imageVersion = 1uL;
    IAP_MetaEncodeRecord(&f, ref);
    IAP_MetaDecodeRecord(ref, &rec);
    CHECK_DESC("reference record valid", rec.valid);

    /* magic 破坏 */
    memcpy(buf, ref, sizeof(buf)); buf[0] ^= 0x01u;
    IAP_MetaDecodeRecord(buf, &rec);
    CHECK_DESC("corrupted magic -> invalid", !rec.valid);

    /* format_version 破坏 */
    memcpy(buf, ref, sizeof(buf)); buf[4] ^= 0x01u;
    IAP_MetaDecodeRecord(buf, &rec);
    CHECK_DESC("corrupted format_version -> invalid", !rec.valid);

    /* header_size 破坏 */
    memcpy(buf, ref, sizeof(buf)); buf[6] ^= 0x01u;
    IAP_MetaDecodeRecord(buf, &rec);
    CHECK_DESC("corrupted header_size -> invalid", !rec.valid);

    /* record_crc 破坏（存储值与计算值不符） */
    memcpy(buf, ref, sizeof(buf)); buf[32] ^= 0x01u;
    IAP_MetaDecodeRecord(buf, &rec);
    CHECK_DESC("corrupted record_crc -> invalid", !rec.valid);

    /* commit marker 未写（断电发生在 commit 编程前：半字保持擦除态） */
    memcpy(buf, ref, sizeof(buf)); buf[36] = 0xFFu; buf[37] = 0xFFu;
    IAP_MetaDecodeRecord(buf, &rec);
    CHECK_DESC("commit marker erased (uncommitted) -> invalid", !rec.valid);

    /* commit marker 位错（半编程/损坏态） */
    memcpy(buf, ref, sizeof(buf)); buf[36] ^= 0x01u;
    IAP_MetaDecodeRecord(buf, &rec);
    CHECK_DESC("commit marker bit-off -> invalid", !rec.valid);

    /* generation 字节翻转（CRC 覆盖字段） */
    memcpy(buf, ref, sizeof(buf)); buf[8] ^= 0x80u;
    IAP_MetaDecodeRecord(buf, &rec);
    CHECK_DESC("corrupted generation (CRC catch) -> invalid", !rec.valid);

    /* image_size 字节翻转 */
    memcpy(buf, ref, sizeof(buf)); buf[18] ^= 0x10u;
    IAP_MetaDecodeRecord(buf, &rec);
    CHECK_DESC("corrupted image_size (CRC catch) -> invalid", !rec.valid);
}

/*--------------------------- 3. generation 环绕 ---------------------------*/

static void test_generation_wrap(void)
{
    u32 seq[4];
    int i;
    int ok;

    printf("suite meta-generation-wrap\n");

    CHECK_DESC("0xFFFFFFFE older than 0xFFFFFFFF",
        !IAP_GenerationIsNewer(0xFFFFFFFEuL, 0xFFFFFFFFuL));
    CHECK_DESC("0xFFFFFFFF newer than 0xFFFFFFFE",
        IAP_GenerationIsNewer(0xFFFFFFFFuL, 0xFFFFFFFEuL));
    CHECK_DESC("0 newer than 0xFFFFFFFF (wrap)",
        IAP_GenerationIsNewer(0uL, 0xFFFFFFFFuL));
    CHECK_DESC("0xFFFFFFFF older than 0 (wrap reverse)",
        !IAP_GenerationIsNewer(0xFFFFFFFFuL, 0uL));
    CHECK_DESC("1 newer than 0", IAP_GenerationIsNewer(1uL, 0uL));
    CHECK_DESC("0 older than 1", !IAP_GenerationIsNewer(0uL, 1uL));
    CHECK_DESC("equal not newer", !IAP_GenerationIsNewer(7uL, 7uL));

    /* 环绕序列 0xFFFFFFFE -> 0xFFFFFFFF -> 0 -> 1 严格递增 */
    seq[0] = 0xFFFFFFFEuL; seq[1] = 0xFFFFFFFFuL; seq[2] = 0uL; seq[3] = 1uL;
    ok = 1;
    for (i = 0; i < 3; i++) {
        if (!IAP_GenerationIsNewer(seq[i + 1], seq[i])) ok = 0;
    }
    CHECK_DESC("wrap sequence strictly increasing", ok);
    ok = 1;
    for (i = 0; i < 3; i++) {
        if (IAP_GenerationIsNewer(seq[i], seq[i + 1])) ok = 0;
    }
    CHECK_DESC("wrap sequence reverse all older", ok);

    /* 相差 0x80000000 判歧义并拒绝 */
    CHECK_DESC("diff 0x80000000 ambiguous",
        IAP_GenerationIsAmbiguous(0x80000000uL, 0uL));
    CHECK_DESC("diff 0x80000000 ambiguous (reverse)",
        IAP_GenerationIsAmbiguous(0uL, 0x80000000uL));
    CHECK_DESC("normal diff not ambiguous", !IAP_GenerationIsAmbiguous(5uL, 7uL));
    CHECK_DESC("equal not ambiguous by diff", !IAP_GenerationIsAmbiguous(7uL, 7uL));
    CHECK_DESC("neither newer when ambiguous (a->b)",
        !IAP_GenerationIsNewer(0x80000000uL, 0uL));
    CHECK_DESC("neither newer when ambiguous (b->a)",
        !IAP_GenerationIsNewer(0uL, 0x80000000uL));
}

/*--------------------------- 4. 双页选择 ---------------------------*/

static void test_select_latest(void)
{
    IAP_MetaRecord_t a, b;
    const IAP_MetaRecord_t *sel = NULL;

    printf("suite meta-select-latest\n");

    /* A=VALID(5) 旧 / B=RECEIVING(6) 新：升级进行中，选 B */
    a = mkrec(5uL, IAP_META_STATE_VALID);
    b = mkrec(6uL, IAP_META_STATE_RECEIVING);
    CHECK_EQ_DESC("A=VALID5,B=RECEIVING6 -> select B",
        IAP_MetaSelectLatest(&a, &b, &sel), IAP_META_SELECT_PAGE_B);
    CHECK_EQ_DESC("selected generation == 6", sel->generation, 6uL);

    /* A=VALID(7) 新 / B=RECEIVING(6) 旧：升级已提交，选 A */
    a = mkrec(7uL, IAP_META_STATE_VALID);
    b = mkrec(6uL, IAP_META_STATE_RECEIVING);
    CHECK_EQ_DESC("A=VALID7,B=RECEIVING6 -> select A",
        IAP_MetaSelectLatest(&a, &b, &sel), IAP_META_SELECT_PAGE_A);
    CHECK_EQ_DESC("selected generation == 7", sel->generation, 7uL);

    /* 双 VALID：最新 generation 胜 */
    a = mkrec(5uL, IAP_META_STATE_VALID);
    b = mkrec(6uL, IAP_META_STATE_VALID);
    CHECK_EQ_DESC("A=VALID5,B=VALID6 -> select B",
        IAP_MetaSelectLatest(&a, &b, &sel), IAP_META_SELECT_PAGE_B);
    a = mkrec(6uL, IAP_META_STATE_VALID);
    b = mkrec(5uL, IAP_META_STATE_VALID);
    CHECK_EQ_DESC("A=VALID6,B=VALID5 -> select A",
        IAP_MetaSelectLatest(&a, &b, &sel), IAP_META_SELECT_PAGE_A);

    /* 一页损坏 */
    a = mkrec(5uL, IAP_META_STATE_VALID);
    b = mkrec(6uL, IAP_META_STATE_RECEIVING); b.valid = 0u;
    CHECK_EQ_DESC("B corrupted -> select A",
        IAP_MetaSelectLatest(&a, &b, &sel), IAP_META_SELECT_PAGE_A);
    a.valid = 0u;
    CHECK_EQ_DESC("both corrupted -> NONE",
        IAP_MetaSelectLatest(&a, &b, &sel), IAP_META_SELECT_NONE);
    CHECK_DESC("selected == NULL on NONE", sel == NULL);

    /* NULL 参数按无效页处理 */
    a = mkrec(5uL, IAP_META_STATE_VALID);
    CHECK_EQ_DESC("recB NULL -> select A",
        IAP_MetaSelectLatest(&a, NULL, &sel), IAP_META_SELECT_PAGE_A);
    CHECK_EQ_DESC("both NULL -> NONE",
        IAP_MetaSelectLatest(NULL, NULL, &sel), IAP_META_SELECT_NONE);

    /* 环绕：gen 0 新于 gen 0xFFFFFFFF */
    a = mkrec(0xFFFFFFFFuL, IAP_META_STATE_VALID);
    b = mkrec(0uL, IAP_META_STATE_RECEIVING);
    CHECK_EQ_DESC("wrap: gen0 newer than max -> select B",
        IAP_MetaSelectLatest(&a, &b, &sel), IAP_META_SELECT_PAGE_B);

    /* 相差 0x80000000：无法排序，保守拒绝 */
    a = mkrec(0uL, IAP_META_STATE_VALID);
    b = mkrec(0x80000000uL, IAP_META_STATE_VALID);
    CHECK_EQ_DESC("diff 0x80000000 -> AMBIGUOUS",
        IAP_MetaSelectLatest(&a, &b, &sel), IAP_META_SELECT_AMBIGUOUS);
    CHECK_DESC("ambiguous -> selected NULL", sel == NULL);

    /* 两页同 generation：正常序列不可能出现，判歧义 */
    a = mkrec(5uL, IAP_META_STATE_VALID);
    b = mkrec(5uL, IAP_META_STATE_RECEIVING);
    CHECK_EQ_DESC("equal generation -> AMBIGUOUS (conservative)",
        IAP_MetaSelectLatest(&a, &b, &sel), IAP_META_SELECT_AMBIGUOUS);
}

/*--------------------------- 5. 启动决策 ---------------------------*/

static void test_boot_decision(void)
{
    IAP_MetaRecord_t a, b;

    printf("suite meta-boot-decision\n");

    /* 最新 VALID 且字段合法 -> CONTINUE */
    a = mkrec(7uL, IAP_META_STATE_VALID);
    b = mkrec(6uL, IAP_META_STATE_RECEIVING);
    CHECK_EQ_DESC("latest VALID -> CONTINUE",
        IAP_BootDecision(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_BOOT_CONTINUE);

    /* 最新 RECEIVING -> 停留 Boot */
    a = mkrec(5uL, IAP_META_STATE_VALID);
    b = mkrec(6uL, IAP_META_STATE_RECEIVING);
    CHECK_EQ_DESC("latest RECEIVING -> REJECT (stay in boot)",
        IAP_BootDecision(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_BOOT_REJECT);

    /* 无有效记录 */
    a = mkrec(5uL, IAP_META_STATE_VALID); a.valid = 0u;
    b = mkrec(6uL, IAP_META_STATE_RECEIVING); b.valid = 0u;
    CHECK_EQ_DESC("no valid record -> REJECT",
        IAP_BootDecision(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_BOOT_REJECT);

    /* NULL 容忍 */
    CHECK_EQ_DESC("both NULL -> REJECT",
        IAP_BootDecision(NULL, NULL, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_BOOT_REJECT);

    /* appBase 不符（slot A 记录却写了别的基址）-> REJECT */
    a = mkrec(7uL, IAP_META_STATE_VALID);
    b = mkrec(6uL, IAP_META_STATE_RECEIVING);
    a.appBase = SLOT_A_BASE + 0x400uL;
    CHECK_EQ_DESC("appBase mismatch -> REJECT",
        IAP_BootDecision(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_BOOT_REJECT);

    /* slot_id 与 app_base 交叉不符（slot A 却写 B 基址）-> REJECT */
    a = mkrec(7uL, IAP_META_STATE_VALID);
    a.slotId  = IAP_META_SLOT_A;
    a.appBase = SLOT_B_BASE;
    CHECK_EQ_DESC("slotId/appBase cross-mismatch -> REJECT",
        IAP_BootDecision(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_BOOT_REJECT);

    /* slot_id 非法（0）-> REJECT */
    a = mkrec(7uL, IAP_META_STATE_VALID);
    a.slotId = 0u;
    CHECK_EQ_DESC("slotId 0 (illegal) -> REJECT",
        IAP_BootDecision(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_BOOT_REJECT);

    /* Slot B 记录合法 -> CONTINUE（双槽核心能力） */
    a = mkrec(7uL, IAP_META_STATE_VALID);
    a.slotId  = IAP_META_SLOT_B;
    a.appBase = SLOT_B_BASE;
    CHECK_EQ_DESC("valid slot B record -> CONTINUE",
        IAP_BootDecision(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_BOOT_CONTINUE);

    /* imageSize 边界：8<=size<=SLOT_CAPACITY */
    a = mkrec(7uL, IAP_META_STATE_VALID);
    a.imageSize = 7uL;
    CHECK_EQ_DESC("imageSize 7 (<8) -> REJECT",
        IAP_BootDecision(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_BOOT_REJECT);
    a.imageSize = 8uL;
    CHECK_EQ_DESC("imageSize 8 (boundary) -> CONTINUE",
        IAP_BootDecision(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_BOOT_CONTINUE);
    a.imageSize = SLOT_CAPACITY;
    CHECK_EQ_DESC("imageSize == SLOT_CAPACITY -> CONTINUE",
        IAP_BootDecision(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_BOOT_CONTINUE);
    a.imageSize = SLOT_CAPACITY + 1uL;
    CHECK_EQ_DESC("imageSize capacity+1 -> REJECT",
        IAP_BootDecision(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_BOOT_REJECT);

    /* 歧义 -> REJECT */
    a = mkrec(0uL, IAP_META_STATE_VALID);
    b = mkrec(0x80000000uL, IAP_META_STATE_VALID);
    CHECK_EQ_DESC("ambiguous generations -> REJECT",
        IAP_BootDecision(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_BOOT_REJECT);

    /* 未知状态字（非 VALID 非 RECEIVING）-> REJECT */
    a = mkrec(7uL, 0x1234u);
    b = mkrec(6uL, IAP_META_STATE_RECEIVING);
    CHECK_EQ_DESC("unknown state word -> REJECT",
        IAP_BootDecision(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_BOOT_REJECT);
}

/*--------------------------- 5b. 槽选择辅助 ---------------------------*/

static void test_slot_helpers(void)
{
    IAP_MetaRecord_t r;

    printf("suite meta-slot-helpers\n");

    r = mkrec(1uL, IAP_META_STATE_VALID);
    r.slotId = IAP_META_SLOT_A; r.appBase = SLOT_A_BASE;
    CHECK_EQ_DESC("SlotBaseFromRecord A", 
        IAP_SlotBaseFromRecord(&r, SLOT_A_BASE, SLOT_B_BASE), SLOT_A_BASE);

    r.slotId = IAP_META_SLOT_B; r.appBase = SLOT_B_BASE;
    CHECK_EQ_DESC("SlotBaseFromRecord B",
        IAP_SlotBaseFromRecord(&r, SLOT_A_BASE, SLOT_B_BASE), SLOT_B_BASE);

    r.slotId = IAP_META_SLOT_B; r.appBase = SLOT_A_BASE;  /* 交叉不符 */
    CHECK_EQ_DESC("SlotBaseFromRecord cross-mismatch -> 0",
        IAP_SlotBaseFromRecord(&r, SLOT_A_BASE, SLOT_B_BASE), 0u);

    CHECK_EQ_DESC("SlotBaseFromRecord NULL -> 0",
        IAP_SlotBaseFromRecord(NULL, SLOT_A_BASE, SLOT_B_BASE), 0u);

    CHECK_EQ_DESC("SlotOther(A) -> B",
        IAP_SlotOther(SLOT_A_BASE, SLOT_A_BASE, SLOT_B_BASE), SLOT_B_BASE);
    CHECK_EQ_DESC("SlotOther(B) -> A",
        IAP_SlotOther(SLOT_B_BASE, SLOT_A_BASE, SLOT_B_BASE), SLOT_A_BASE);
    CHECK_EQ_DESC("SlotOther(illegal) -> 0",
        IAP_SlotOther(0x08010000uL, SLOT_A_BASE, SLOT_B_BASE), 0u);
}

/*--------------------------- 6. fake flash 物理约束 ---------------------------*/

static void test_flash_physics(void)
{
    printf("suite fake-flash-physics\n");

    FakeFlash_Init(&g_flash);
    FakeFlash_DisableCut(&g_flash);

    /* 编程只能 1->0 */
    CHECK_EQ_DESC("program 0x1234 into erased page ok",
        FakeFlash_ProgramHalfword(&g_flash, 0x0000uL, 0x1234u), 0);
    CHECK_EQ_DESC("0->1 program (bit0 set) rejected",
        FakeFlash_ProgramHalfword(&g_flash, 0x0000uL, 0x1235u), -1);
    CHECK_EQ_DESC("same-value reprogram allowed (no 0->1 bits)",
        FakeFlash_ProgramHalfword(&g_flash, 0x0000uL, 0x1234u), 0);
    CHECK_EQ_DESC("1->0 bit clear allowed",
        FakeFlash_ProgramHalfword(&g_flash, 0x0000uL, 0x0234u), 0);
    CHECK_EQ_DESC("program 0xFFFF over programmed value rejected",
        FakeFlash_ProgramHalfword(&g_flash, 0x0000uL, 0xFFFFu), -1);

    /* 对齐约束 */
    CHECK_EQ_DESC("odd address program rejected",
        FakeFlash_ProgramHalfword(&g_flash, 0x0003uL, 0x1111u), -1);
    CHECK_EQ_DESC("unaligned page erase rejected",
        FakeFlash_ErasePage(&g_flash, 0x0001uL), -1);

    /* 越界（边界随布局派生：C8 64 KiB / CB 128 KiB） */
    CHECK_EQ_DESC("program beyond flash end rejected",
        FakeFlash_ProgramHalfword(&g_flash, FAKE_FLASH_TOTAL_SIZE, 0x1111u), -1);
    CHECK_EQ_DESC("erase beyond flash end rejected",
        FakeFlash_ErasePage(&g_flash, FAKE_FLASH_TOTAL_SIZE), -1);

    /* 擦除恢复 0xFF 后可再编程 */
    CHECK_EQ_DESC("page erase ok",
        FakeFlash_ErasePage(&g_flash, 0x0000uL), 0);
    CHECK_EQ_DESC("erased halfword reads 0xFFFF",
        FakeFlash_ReadHalfword(&g_flash, 0x0000uL), 0xFFFFu);
    CHECK_EQ_DESC("reprogram after erase ok",
        FakeFlash_ProgramHalfword(&g_flash, 0x0000uL, 0xABCDu), 0);
    CHECK_EQ_DESC("readback matches",
        FakeFlash_ReadHalfword(&g_flash, 0x0000uL), 0xABCDu);

    /* 断电语义：到达断电点后所有操作失败且内容不变 */
    FakeFlash_Init(&g_flash);
    FakeFlash_SetCutAfter(&g_flash, 0uL);
    CHECK_EQ_DESC("erase after cut rejected",
        FakeFlash_ErasePage(&g_flash, 0x0000uL), -1);
    CHECK_EQ_DESC("content unchanged after cut",
        FakeFlash_ReadHalfword(&g_flash, 0x0000uL), 0xFFFFu);
}

/*--------------------------- 7. 记录写入事务 ---------------------------*/

static void test_write_record(void)
{
    IAP_MetaRecordFields_t f;
    IAP_MetaRecord_t rec;
    u8 buf[REC_BYTES];
    u32 erases, programs;

    printf("suite meta-write-record\n");

    f.generation   = 0x89ABCDEFuL;
    f.stateWord    = IAP_META_STATE_VALID;
    f.slotId       = IAP_META_SLOT_A;
    f.appBase      = SLOT_A_BASE;
    f.imageSize    = 4096uL;
    f.imageCrc     = 0xCAFEBABEuL;
    f.imageVersion = 0x01020304uL;

    /* NULL 参数拒绝 */
    CHECK_EQ_DESC("WriteRecord(NULL io) -> -1",
        IAP_MetaWriteRecord(NULL, META_A_BASE, &f), -1);
    CHECK_EQ_DESC("WriteRecord(NULL fields) -> -1",
        IAP_MetaWriteRecord(&g_io, META_A_BASE, NULL), -1);
    CHECK_EQ_DESC("ReadRecord(NULL io) -> -1",
        IAP_MetaReadRecord(NULL, META_A_BASE, buf), -1);
    CHECK_EQ_DESC("ReadRecord(NULL buf) -> -1",
        IAP_MetaReadRecord(&g_io, META_A_BASE, NULL), -1);

    /* 正常写入：1 次擦除 + 19 次半字编程（38 B 记录） */
    FakeFlash_Init(&g_flash);
    FakeFlash_DisableCut(&g_flash);
    erases   = g_flash.erase_count;
    programs = g_flash.program_count;
    CHECK_EQ_DESC("WriteRecord ok",
        IAP_MetaWriteRecord(&g_io, META_A_BASE, &f), 0);
    CHECK_EQ_DESC("exactly one page erase", g_flash.erase_count - erases, 1);
    CHECK_EQ_DESC("exactly 19 halfword programs",
        g_flash.program_count - programs, 19);

    /* 读回解码全字段一致 */
    IAP_MetaReadRecord(&g_io, META_A_BASE, buf);
    IAP_MetaDecodeRecord(buf, &rec);
    CHECK_DESC("readback record valid", rec.valid);
    CHECK_EQ_DESC("readback generation", rec.generation, 0x89ABCDEFuL);
    CHECK_EQ_DESC("readback stateWord", rec.stateWord, IAP_META_STATE_VALID);
    CHECK_EQ_DESC("readback appBase", rec.appBase, SLOT_A_BASE);
    CHECK_EQ_DESC("readback imageSize", rec.imageSize, 4096uL);
    CHECK_EQ_DESC("readback imageCrc", rec.imageCrc, 0xCAFEBABEuL);
    CHECK_EQ_DESC("readback imageVersion", rec.imageVersion, 0x01020304uL);
    CHECK_EQ_DESC("readback commitMarker", rec.commitMarker, IAP_META_COMMIT_DONE);
}

static void test_write_record_power_cut(void)
{
    IAP_MetaRecordFields_t f;
    IAP_MetaRecord_t rec;
    u8 buf[REC_BYTES];
    u16 hw;

    printf("suite meta-write-power-cut\n");

    f.generation   = 6uL;
    f.stateWord    = IAP_META_STATE_RECEIVING;
    f.slotId       = IAP_META_SLOT_A;
    f.appBase      = SLOT_A_BASE;
    f.imageSize    = 2048uL;
    f.imageCrc     = 0x99AABBCCuL;
    f.imageVersion = 2uL;

    /* 断电在擦除（会话第 0 次操作后）：-2，页保持 0xFF */
    FakeFlash_Init(&g_flash);
    FakeFlash_SetCutAfter(&g_flash, 0uL);
    CHECK_EQ_DESC("cut at erase -> -2",
        IAP_MetaWriteRecord(&g_io, META_B_BASE, &f), -2);
    IAP_MetaReadRecord(&g_io, META_B_BASE, buf);
    IAP_MetaDecodeRecord(buf, &rec);
    CHECK_DESC("untouched page -> record invalid", !rec.valid);

    /* 断电在字段半字（会话第 10 次操作后）：-3，记录无 commit 无效 */
    FakeFlash_Init(&g_flash);
    FakeFlash_SetCutAfter(&g_flash, 10uL);
    CHECK_EQ_DESC("cut at field halfword -> -3",
        IAP_MetaWriteRecord(&g_io, META_B_BASE, &f), -3);
    IAP_MetaReadRecord(&g_io, META_B_BASE, buf);
    IAP_MetaDecodeRecord(buf, &rec);
    CHECK_DESC("partial record without commit -> invalid", !rec.valid);
    /* 字段已部分写入，但 commit 半字保持擦除态：证明 commit 最后编程 */
    hw = FakeFlash_ReadHalfword(&g_flash, (META_B_BASE - FLASH_BASE) + 36uL);
    CHECK_EQ_DESC("commit halfword still erased (0xFFFF)", hw, 0xFFFFu);

    /* 断电恰好停在 commit 编程（会话第 19 次操作后）：-4 */
    FakeFlash_Init(&g_flash);
    FakeFlash_SetCutAfter(&g_flash, 19uL);
    CHECK_EQ_DESC("cut at commit program -> -4",
        IAP_MetaWriteRecord(&g_io, META_B_BASE, &f), -4);
    IAP_MetaReadRecord(&g_io, META_B_BASE, buf);
    IAP_MetaDecodeRecord(buf, &rec);
    CHECK_DESC("all fields written but no commit -> invalid", !rec.valid);
}

/*--------------------------- 8. 断电注入 ---------------------------*/

/*
 * 模拟一次完整升级会话（从 A=VALID(gen5) 出发，按设计 §5.4）：
 *   1. 开始记录：擦非当前页 B，写 RECEIVING(gen6)，commit 最后编程
 *   2. 有效记录：擦另一页 A，写 VALID(gen7)，commit 最后编程
 *（APP 区擦写在 Task 7 boot_flash 套件覆盖，此处只验证元数据层
 *       安全不变量：决策永远不允许启动部分写入的记录）
 * cutOffset = FAKE_FLASH_NO_CUT 表示不注电；否则会话内第 cutOffset 次
 * 操作之后"断电"（后续所有 erase/program 失败，Flash 停在断电瞬间）。
 * 返回会话实际执行的操作数（无断电时 = 2*(1 擦除 + 19 编程) = 40）。
 */
static u32 run_update_scenario(u32 cutOffset, RestartView_t *v)
{
    IAP_MetaRecordFields_t f;
    u32 ops0;

    FakeFlash_Init(&g_flash);
    FakeFlash_DisableCut(&g_flash);

    /* 工厂状态：A 页 VALID(gen5) */
    f.generation   = 5uL;
    f.stateWord    = IAP_META_STATE_VALID;
    f.slotId       = IAP_META_SLOT_A;
    f.appBase      = SLOT_A_BASE;
    f.imageSize    = 4096uL;
    f.imageCrc     = 0x55667788uL;
    f.imageVersion = 0x00010005uL;
    IAP_MetaWriteRecord(&g_io, META_A_BASE, &f);

    ops0 = FakeFlash_OpsDone(&g_flash);

    if (cutOffset == FAKE_FLASH_NO_CUT) {
        FakeFlash_DisableCut(&g_flash);
    } else {
        FakeFlash_SetCutAfter(&g_flash, ops0 + cutOffset);
    }

    /* 开始记录：擦非当前页 B，写 RECEIVING(gen6) */
    f.generation   = 6uL;
    f.stateWord    = IAP_META_STATE_RECEIVING;
    IAP_MetaWriteRecord(&g_io, META_B_BASE, &f);

    /* 有效记录：擦另一页 A，写 VALID(gen7) */
    f.generation   = 7uL;
    f.stateWord    = IAP_META_STATE_VALID;
    IAP_MetaWriteRecord(&g_io, META_A_BASE, &f);

    boot_view(v);
    return FakeFlash_OpsDone(&g_flash) - ops0;
}

static void test_power_cut_injection(void)
{
    RestartView_t view;
    u32 totalOps;
    u32 k;
    int points = 0;
    int continueOld = 0, continueNew = 0, rejected = 0;
    int okUnexpected = 1, okOld = 1, okReject = 1, okNew = 1;

    printf("suite meta-power-cut-injection\n");

    /* 无断电基线：新 VALID(gen7) 启动 */
    totalOps = run_update_scenario(FAKE_FLASH_NO_CUT, &view);
    CHECK_EQ_DESC("session = 2 erases + 38 programs (40 ops)", totalOps, SESSION_OPS);
    CHECK_EQ_DESC("no cut: boot new VALID (gen7)",
        view.boot == IAP_BOOT_CONTINUE &&
        view.selRec->generation == 7uL, 1);

    /* 在每个 erase/halfword program 位置注入断电（含会话起点前） */
    for (k = 0uL; k <= totalOps; k++) {
        run_update_scenario(k, &view);
        points++;

        if (view.boot == IAP_BOOT_CONTINUE) {
            u32 g = view.selRec->generation;
            u16 st = view.selRec->stateWord;
            /* 唯一允许的启动来源：旧 VALID(gen5) 或新 VALID(gen7) */
            if (g == 5uL && st == IAP_META_STATE_VALID) {
                continueOld++;
            } else if (g == 7uL && st == IAP_META_STATE_VALID) {
                continueNew++;
            } else {
                okUnexpected = 0;   /* 部分写入记录被启动：安全违规 */
            }
        } else {
            rejected++;
        }
    }

    /* 分段不变量（每条记录 = 1 擦除 + 19 编程 = 20 次操作） */
    /* k=0..19：START 记录未提交（或未开始）断电 -> 旧 VALID 仍可启动 */
    for (k = 0uL; k <= 19uL; k++) {
        run_update_scenario(k, &view);
        if (!(view.boot == IAP_BOOT_CONTINUE &&
              view.selRec->generation == 5uL)) okOld = 0;
    }
    /* k=20..39：RECEIVING 已提交后断电 -> 必须停留 Boot */
    for (k = 20uL; k <= 39uL; k++) {
        run_update_scenario(k, &view);
        if (view.boot != IAP_BOOT_REJECT) okReject = 0;
    }
    /* k=40：最终 commit 完成后 -> 新 VALID 启动 */
    run_update_scenario(totalOps, &view);
    if (!(view.boot == IAP_BOOT_CONTINUE &&
          view.selRec->generation == 7uL)) okNew = 0;

    printf("  injection points=%d, continue-old=%d, reject=%d, continue-new=%d\n",
           points, continueOld, rejected, continueNew);

    CHECK_DESC("injection point count non-zero", points > 0);
    CHECK_DESC("no partial-write record ever booted", okUnexpected);
    CHECK_DESC("cut before START commit: old VALID(gen5) still boots (k=0..19)", okOld);
    CHECK_DESC("cut after RECEIVING committed: stay in boot (k=20..39)", okReject);
    CHECK_DESC("cut after final commit: new VALID(gen7) boots (k=40)", okNew);
    CHECK_DESC("continue-old occurred", continueOld > 0);
    CHECK_DESC("reject occurred", rejected > 0);
    CHECK_DESC("continue-new occurred", continueNew > 0);
}

/* 连续两次完整升级（无断电）：验证 generation 链与页交替正确 */
static void test_double_update_no_cut(void)
{
    IAP_MetaRecordFields_t f;
    RestartView_t view;

    printf("suite meta-double-update\n");

    FakeFlash_Init(&g_flash);
    FakeFlash_DisableCut(&g_flash);

    f.slotId       = IAP_META_SLOT_A;
    f.appBase      = SLOT_A_BASE;
    f.imageSize    = 4096uL;
    f.imageCrc     = 0x11111111uL;
    f.imageVersion = 1uL;

    /* 第一次升级：A=VALID5 -> B=RECEIVING6 -> A=VALID7 */
    f.generation = 5uL; f.stateWord = IAP_META_STATE_VALID;
    IAP_MetaWriteRecord(&g_io, META_A_BASE, &f);
    f.generation = 6uL; f.stateWord = IAP_META_STATE_RECEIVING;
    IAP_MetaWriteRecord(&g_io, META_B_BASE, &f);
    f.generation = 7uL; f.stateWord = IAP_META_STATE_VALID;
    IAP_MetaWriteRecord(&g_io, META_A_BASE, &f);
    boot_view(&view);
    CHECK_EQ_DESC("after update 1: CONTINUE gen7",
        view.boot == IAP_BOOT_CONTINUE &&
        view.selRec->generation == 7uL, 1);

    /* 第二次升级中途：RECEIVING8 已提交、A 仍 VALID7 -> 拒绝启动 */
    f.generation = 8uL; f.stateWord = IAP_META_STATE_RECEIVING;
    f.imageVersion = 2uL;
    IAP_MetaWriteRecord(&g_io, META_B_BASE, &f);
    boot_view(&view);
    CHECK_EQ_DESC("mid update 2 (RECEIVING8 latest): REJECT",
        view.boot, IAP_BOOT_REJECT);

    /* 第二次升级提交：A=VALID9 */
    f.generation = 9uL; f.stateWord = IAP_META_STATE_VALID;
    IAP_MetaWriteRecord(&g_io, META_A_BASE, &f);
    boot_view(&view);
    CHECK_EQ_DESC("after update 2: select page A",
        view.select, IAP_META_SELECT_PAGE_A);
    CHECK_EQ_DESC("after update 2: CONTINUE gen9",
        view.boot == IAP_BOOT_CONTINUE &&
        view.selRec->generation == 9uL, 1);
}

/*-----------------------------------------------------------------
 * F-01 端到端回归：真实 Flash 元数据页 + 真实解码 + 生产活动槽解析
 *
 * 与 test_dual_slot.c 的纯结构用例互补：这里走 fake_flash 真实擦写/
 * 编程元数据页，再由 IAP_MetaReadRecord + IAP_MetaDecodeRecord 解码，
 * 最后调用生产函数 IAP_ActiveSlotOf()/IAP_SlotOther()，验证
 * "升级途中断电 -> 重试升级不会擦掉唯一的好镜像"。
 *-----------------------------------------------------------------*/
static u32 meta_active_target(u32 *outActive)
{
    IAP_MetaRecord_t recA, recB;
    u8 bufA[IAP_META_RECORD_SIZE], bufB[IAP_META_RECORD_SIZE];
    IAP_ActiveSlot_t act;
    u32 active, target;

    (void)IAP_MetaReadRecord(&g_io, META_A_BASE, bufA);
    (void)IAP_MetaReadRecord(&g_io, META_B_BASE, bufB);
    IAP_MetaDecodeRecord(bufA, &recA);
    IAP_MetaDecodeRecord(bufB, &recB);

    act= IAP_ActiveSlotOf(&recA, &recB, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY);
    active = IAP_ActiveSlotBase(act, SLOT_A_BASE, SLOT_B_BASE);
    if (outActive != NULL) *outActive = active;
    if (act == IAP_ACTIVE_AMBIGUOUS) return 0u;      /* 保守拒绝 */
    target = (active == 0u) ? SLOT_A_BASE
               : IAP_SlotOther(active, SLOT_A_BASE, SLOT_B_BASE);
    return target;
}

static void test_f01_powerloss_retry_e2e(void)
{
    IAP_MetaRecordFields_t f;
    u32 active = 0u, target;

    printf("-- F-01 e2e: power-loss retry keeps the only good slot\n");

    /* 全新芯片：擦掉两页元数据 */
    FakeFlash_Init(&g_flash);
    FakeFlash_DisableCut(&g_flash);

    /* 1) 首次升级 -> Slot A 提交 VALID(gen1) 写入 A 页 */
    memset(&f, 0, sizeof(f));
    f.generation   = 1uL;
    f.stateWord    = IAP_META_STATE_RECEIVING;
    f.slotId       = IAP_META_SLOT_A;
    f.appBase      = SLOT_A_BASE;
    f.imageSize    = 0x1000uL;
    f.imageCrc     = 0x11111111uL;
    f.imageVersion = 1uL;
    CHECK_EQ_DESC("e2e: write RECEIVING gen1@A", IAP_MetaWriteRecord(&g_io, META_A_BASE, &f), 0);

    f.generation = 2uL;
    f.stateWord  = IAP_META_STATE_VALID;
    CHECK_EQ_DESC("e2e: commit VALID gen2@A (page B)",
                  IAP_MetaWriteRecord(&g_io, META_B_BASE, &f), 0);

    target = meta_active_target(&active);
    CHECK_EQ_DESC("e2e: after 1st upgrade active = Slot A", active, SLOT_A_BASE);
    CHECK_EQ_DESC("e2e: retry target = Slot B", target, SLOT_B_BASE);

    /* 2) 第二次升级：Slot B 的 RECEIVING(gen3) 提交到 A 页（覆盖 gen1），
     *    B 页仍是 VALID(gen2) —— 这就是"升级途中断电"的真实 Flash 状态。
     *    注意页位置与槽无固定对应：VALID gen2 虽写在 B 页，它描述的是 Slot A。*/
    f.generation = 3uL;
    f.stateWord  = IAP_META_STATE_RECEIVING;
    f.slotId     = IAP_META_SLOT_B;   /* 本次目标槽 = Slot B */
    f.appBase    = SLOT_B_BASE;
    CHECK_EQ_DESC("e2e: write RECEIVING gen3@B (power cut after this)",
                  IAP_MetaWriteRecord(&g_io, META_A_BASE, &f), 0);

    /* 此刻：A=RECEIVING gen3(描述 Slot B), B=VALID gen2(描述 Slot A)
     * 活动槽必须是 Slot A（唯一 VALID，也是回滚能力所在），
     * 重试升级的目标槽必须是 Slot B（覆盖半成品） */
    target = meta_active_target(&active);
    CHECK_EQ_DESC("e2e: power cut -> active is still the VALID slot (A)",
                  active, SLOT_A_BASE);
    CHECK_EQ_DESC("e2e: power cut -> retry target = Slot B", target, SLOT_B_BASE);
    CHECK_DESC("e2e: VALID slot is never the erase target",
               target != active);

    /* 3) 关键回归：最新记录是 RECEIVING 时，【不能】退化成"无活动槽"
     *    —— 旧实现会返回 0，目标槽回落到 Slot A（即 VALID 的那个） */
    {
        IAP_MetaRecord_t recA, recB;
        u8 bufA[IAP_META_RECORD_SIZE], bufB[IAP_META_RECORD_SIZE];
        const IAP_MetaRecord_t *sel = NULL;

        (void)IAP_MetaReadRecord(&g_io, META_A_BASE, bufA);
        (void)IAP_MetaReadRecord(&g_io, META_B_BASE, bufB);
        IAP_MetaDecodeRecord(bufA, &recA);
        IAP_MetaDecodeRecord(bufB, &recB);
        CHECK_EQ_DESC("e2e: latest record is the RECEIVING one",
                      IAP_MetaSelectLatest(&recA, &recB, &sel), IAP_META_SELECT_PAGE_A);
        CHECK_EQ_DESC("e2e: latest is RECEIVING (not VALID)",
                      (sel != NULL) && (sel->stateWord == IAP_META_STATE_RECEIVING), 1);
        CHECK_EQ_DESC("e2e: ...yet active slot is still resolved (not NONE)",
                      IAP_ActiveSlotOf(&recA, &recB, SLOT_A_BASE, SLOT_B_BASE,
                                      SLOT_CAPACITY), IAP_ACTIVE_SLOT_A);
    }

    /* 4) 歧义元数据：拒绝升级，不选任何擦除目标 */
    f.generation = 3uL;
    f.stateWord  = IAP_META_STATE_VALID;
    f.slotId     = IAP_META_SLOT_B;
    f.appBase    = SLOT_B_BASE;
    CHECK_EQ_DESC("e2e: create equal-generation VALID on both pages (A gen3, B gen3)",
                  IAP_MetaWriteRecord(&g_io, META_A_BASE, &f), 0);
    /* B 页当前是 VALID gen2 -> 改成 gen3 使两页 generation 相同 */
    f.generation = 3uL;
    CHECK_EQ_DESC("e2e: write VALID gen3@B (now gen equal on both pages)",
                  IAP_MetaWriteRecord(&g_io, META_B_BASE, &f), 0);
    target = meta_active_target(&active);
    CHECK_EQ_DESC("e2e: ambiguous metadata -> refuse upgrade (target=0)", target, 0u);
}

int main(void)
{
    test_encode_layout();
    test_field_corruption();
    test_generation_wrap();
    test_select_latest();
    test_boot_decision();
    test_slot_helpers();
    test_flash_physics();
    test_write_record();
    test_write_record_power_cut();
    test_power_cut_injection();
    test_double_update_no_cut();
    test_f01_powerloss_retry_e2e();

    printf("meta: %d checks, %d failed (dual-page power-fail-safe metadata)\n",
           g_total, g_failed);
    if (g_failed == 0) {
        printf("meta: all cases passed\n");
    }
    return g_failed ? 1 : 0;
}
