// SPDX-License-Identifier: GPL-2.0
/*
 * Host-side mock test for streaming CRC32:
 *   1. 标准检查值："123456789" -> 0xCBF43926（一次性 + 流式 + 分块）
 *   2. 分块等价：同一数据按 1 B / 7 B / 512 B / 不规则分块，
 *      结果必须与一次性 API 完全一致
 *   3. 边界：零长度、NULL+0、跨 64 KiB 长度、奇数字节长度、Final 幂等
 *   4. 兼容：CRC32_Calculate()/CRC32_VerifyFirmware() 走包装路径不受影响
 *
 * Production source under test (BSP\CRC32\crc32.c) is compiled UNMODIFIED;
 * only sys.h/usart.h come from the stub/ tree.
 *
 * 运行：powershell -ExecutionPolicy Bypass -File mock\freertos\run-mock-iap-stop.ps1 -Suite crc32
 * 预期：crc32-stream: all cases passed
 */
#include <stdio.h>
#include <string.h>
#include "crc32.h"

static int g_failed = 0;
static int g_total = 0;

#define CHECK_DESC(desc, cond) do {                       \
    g_total++;                                            \
    if (cond) { printf("  ok %s\n", desc); }              \
    else      { g_failed++; printf("  FAIL %s\n", desc); } \
} while (0)

#define CHECK_EQ_DESC(desc, got, want) do {               \
    g_total++;                                            \
    if ((unsigned long)(got) == (unsigned long)(want))    \
        printf("  ok %s\n", desc);                        \
    else { g_failed++;                                    \
        printf("  FAIL %s: got %lu, want %lu\n",          \
               desc, (unsigned long)(got),                \
               (unsigned long)(want)); }                  \
} while (0)

/* 测试镜像：70 KB 略多，覆盖 >64 KiB 与奇数字节长度 */
#define MIRROR_SIZE 70001uL
static u8 g_mirror[MIRROR_SIZE];

/* 不规则分块模式（确定性：与周期取余，覆盖比 512 小/大、奇/偶块） */
static const size_t IRREGULAR_PATTERN[] = { 1, 3, 2, 7, 5, 13, 21, 34, 8, 512, 1 };
#define IRREGULAR_N (sizeof(IRREGULAR_PATTERN) / sizeof(IRREGULAR_PATTERN[0]))

/* 确定性 LCG 填充，避免全零镜像掩盖查表索引错误 */
static void fill_mirror(void)
{
    u32 seed = 0x12345678uL;
    u32 i;
    for (i = 0; i < MIRROR_SIZE; i++) {
        seed = seed * 1103515245uL + 12345uL;
        g_mirror[i] = (u8)(seed >> 16);
    }
}

/* 流式一次性参考实现（与生产 Update/Final 同路径，作为等价基准） */
static u32 stream_whole(const u8 *buf, size_t len)
{
    CRC32_StreamCtx_t ctx;
    CRC32_Init(&ctx);
    if (len > 0) {
        CRC32_Update(&ctx, buf, (u32)len);
    }
    return CRC32_Final(&ctx);
}

/* 固定块大小分块流式计算 */
static u32 stream_chunked_fixed(const u8 *buf, size_t len, size_t chunk)
{
    CRC32_StreamCtx_t ctx;
    size_t off = 0;
    CRC32_Init(&ctx);
    while (off < len) {
        size_t take = (len - off < chunk) ? (len - off) : chunk;
        CRC32_Update(&ctx, buf + off, (u32)take);
        off += take;
    }
    return CRC32_Final(&ctx);
}

/* 不规则分块流式计算（块大小按模式循环） */
static u32 stream_chunked_irregular(const u8 *buf, size_t len)
{
    CRC32_StreamCtx_t ctx;
    size_t off = 0, idx = 0;
    CRC32_Init(&ctx);
    while (off < len) {
        size_t take = IRREGULAR_PATTERN[idx % IRREGULAR_N];
        if (take > len - off) {
            take = len - off;
        }
        CRC32_Update(&ctx, buf + off, (u32)take);
        off += take;
        idx++;
    }
    return CRC32_Final(&ctx);
}

/*-----------------------------------------------------------
 * 1. 标准检查值："123456789" -> 0xCBF43926
 *----------------------------------------------------------*/
static void test_check_value(void)
{
    static const char *check = "123456789";
    size_t len = strlen(check);
    const u8 *p = (const u8 *)check;

    printf("suite check-value\n");
    CHECK_EQ_DESC("one-shot \"123456789\"", CRC32_Calculate((u8 *)p, (u32)len), 0xCBF43926uL);
    CHECK_EQ_DESC("stream whole", stream_whole(p, len), 0xCBF43926uL);
    CHECK_EQ_DESC("chunked 1 B", stream_chunked_fixed(p, len, 1), 0xCBF43926uL);
    CHECK_EQ_DESC("chunked 7 B", stream_chunked_fixed(p, len, 7), 0xCBF43926uL);
    CHECK_EQ_DESC("chunked 512 B", stream_chunked_fixed(p, len, 512), 0xCBF43926uL);
    CHECK_EQ_DESC("chunked irregular", stream_chunked_irregular(p, len), 0xCBF43926uL);
}

/*-----------------------------------------------------------
 * 2. 分块等价 + 跨 64 KiB / 奇数字节长度
 *----------------------------------------------------------*/
static void test_chunk_equivalence_size(size_t len)
{
    u32 ref = CRC32_Calculate(g_mirror, (u32)len);
    CHECK_EQ_DESC("stream == one-shot", stream_whole(g_mirror, len), ref);
    CHECK_EQ_DESC("chunked 1 B == one-shot", stream_chunked_fixed(g_mirror, len, 1), ref);
    CHECK_EQ_DESC("chunked 7 B == one-shot", stream_chunked_fixed(g_mirror, len, 7), ref);
    CHECK_EQ_DESC("chunked 512 B == one-shot", stream_chunked_fixed(g_mirror, len, 512), ref);
    CHECK_EQ_DESC("chunked irregular == one-shot", stream_chunked_irregular(g_mirror, len), ref);
}

static void test_chunk_equivalence(void)
{
    static const size_t sizes[] = {
        0, 1, 2, 3, 7, 511, 512, 513, 1024,
        65535, 65536, 65537,   /* 跨 64 KiB 边界 */
        65549, 70001           /* 64 KiB 以上 + 奇数字节镜像（70001 = MIRROR_SIZE） */
    };
    size_t i;

    printf("suite chunk-equivalence\n");
    for (i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
        printf(" size %lu\n", (unsigned long)sizes[i]);
        test_chunk_equivalence_size(sizes[i]);
    }
}

/*-----------------------------------------------------------
 * 3. 边界：零长度、NULL+0、Final 幂等
 *----------------------------------------------------------*/
static void test_boundaries(void)
{
    CRC32_StreamCtx_t ctx;

    printf("suite boundaries\n");

    /* 零长度：标准 CRC32(b"") == 0 */
    CHECK_EQ_DESC("one-shot empty == 0", CRC32_Calculate(g_mirror, 0), 0);
    CHECK_EQ_DESC("stream empty == 0", stream_whole(g_mirror, 0), 0);

    /* NULL + 0：不得解引用，结果与空数据一致 */
    CHECK_EQ_DESC("one-shot NULL+0 == 0", CRC32_Calculate(NULL, 0), 0);
    CRC32_Init(&ctx);
    CRC32_Update(&ctx, NULL, 0);
    CHECK_EQ_DESC("update NULL+0 == 0", CRC32_Final(&ctx), 0);

    /* Final 幂等：对同一上下文重复 Final 必须返回同一值（只读语义） */
    CRC32_Init(&ctx);
    CRC32_Update(&ctx, g_mirror, 1024);
    {
        u32 first = CRC32_Final(&ctx);
        u32 second = CRC32_Final(&ctx);
        CHECK_EQ_DESC("final idempotent", second, first);
        CHECK_EQ_DESC("final == one-shot", first, CRC32_Calculate(g_mirror, 1024));
    }

    /* 分块间夹杂零长度 Update 不得影响结果 */
    CRC32_Init(&ctx);
    CRC32_Update(&ctx, g_mirror, 100);
    CRC32_Update(&ctx, NULL, 0);
    CRC32_Update(&ctx, g_mirror + 100, 0);
    CRC32_Update(&ctx, g_mirror + 100, 28);
    CHECK_EQ_DESC("zero-len update no-op", CRC32_Final(&ctx),
                  CRC32_Calculate(g_mirror, 128));
}

/*-----------------------------------------------------------
 * 4. 兼容：VerifyFirmware 走包装路径
 *----------------------------------------------------------*/
static void test_compat_verify_firmware(void)
{
    static u8 fw[300 + 4];   /* [300 B 数据][4 B CRC] */
    const u32 dataLen = 300;
    u32 crc;
    u8 *p = fw;

    printf("suite compat\n");
    memcpy(fw, g_mirror, dataLen);
    crc = CRC32_Calculate(fw, dataLen);

    /* 固件格式：[数据][4 字节 CRC 小端] */
    fw[dataLen + 0] = (u8)(crc & 0xFF);
    fw[dataLen + 1] = (u8)((crc >> 8) & 0xFF);
    fw[dataLen + 2] = (u8)((crc >> 16) & 0xFF);
    fw[dataLen + 3] = (u8)((crc >> 24) & 0xFF);
    CHECK_EQ_DESC("verify ok via wrapper", CRC32_VerifyFirmware(p, dataLen + 4), 1);

    fw[7] ^= 0x40;
    CHECK_EQ_DESC("verify corrupted data fails", CRC32_VerifyFirmware(p, dataLen + 4), 0);
}

int main(void)
{
    fill_mirror();
    test_check_value();
    test_chunk_equivalence();
    test_boundaries();
    test_compat_verify_firmware();

    printf("crc32-stream: %d checks, %d failed\n", g_total, g_failed);
    if (g_failed == 0) {
        printf("crc32-stream: all cases passed\n");
        return 0;
    }
    return 1;
}
