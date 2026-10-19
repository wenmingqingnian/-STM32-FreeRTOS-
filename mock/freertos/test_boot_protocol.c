// SPDX-License-Identifier: GPL-2.0
/*
 * Host-side mock test for the framed streaming IAP protocol and the
 * upgrade session state machine.
 * Covers:
 *
 *   1. normal START -> DATA xN -> END flow
 *   2. header split across DMA fragments, payload spanning the ring
 *      tail (arbitrary chunked feeding), noise before SOF, two frames
 *      back-to-back in a single feed
 *   3. protocol_version/header_len/reserved/type invalid,
 *      payload_len 0/513, frame CRC mismatch -> frame rejected
 *   4. offset+len overflow (subtract-first proof), out-of-bounds,
 *      sequence jump, out-of-order, duplicate re-send of the previous
 *      accepted frame (re-ACK without re-programming)
 *   5. misaligned offset rejected; odd payload length rejected unless
 *      it is the final packet; final odd packet accepted
 *   6. START target_base != APP_BASE, image_size < 8 and
 *      image_size == APP_CAPACITY+1 rejected (boundaries accepted)
 *   7. parser reset after timeout resynchronizes on a fresh frame and
 *      keeps the session alive
 *
 * "No Flash access" evidence: this suite links ONLY BOOT/boot_protocol.c
 * and BSP/CRC32/crc32.c as production sources -- no fake_flash backend
 * and no SPL Flash/register symbol is compiled or referenced; any Flash
 * or register access from the module under test would fail to link here.
 *
 * The session is additionally exercised with hand-built frame structs
 * (no parser in between) to prove the state machine stays mockable on
 * its own.
 */
#include <stdio.h>
#include <string.h>
#include "boot_protocol.h"
#include "crc32.h"
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
               desc, (unsigned long)(got),                 \
               (unsigned long)(want)); }                   \
} while (0)

/*--------------------------- wire helpers ---------------------------*/

#define WIRE_MAX (BOOTPROTO_HEADER_LEN + BOOTPROTO_MAX_PAYLOAD)

static void put_u16le(u8 *p, u16 v)
{
    p[0] = (u8)(v & 0xFFu);
    p[1] = (u8)((v >> 8) & 0xFFu);
}

static void put_u32le(u8 *p, u32 v)
{
    p[0] = (u8)(v & 0xFFu);
    p[1] = (u8)((v >> 8) & 0xFFu);
    p[2] = (u8)((v >> 16) & 0xFFu);
    p[3] = (u8)((v >> 24) & 0xFFu);
}

/* 负面用例的帧头覆盖项（全 0 = 不覆盖，按协议正常值生成） */
typedef struct {
    int hasVersion;    u8  version;      /* 覆盖 protocol_version */
    int hasType;       u8  type;         /* 覆盖 type */
    int hasHdrLen;     u16 hdrLen;       /* 覆盖 header_len */
    int hasReserved;   u16 reserved;     /* 覆盖 reserved */
    int hasPayloadLen; u16 payloadLen;   /* 覆盖 payload_len（帧头声明值） */
    u32 crcXor;                          /* 非 0：帧 CRC 字段异或破坏 */
} Ovr_t;

static const Ovr_t OVR_NONE;   /* 静态存储零初始化 = 无覆盖 */

/* 构造一帧线上字节流。payload/copiedLen 为真实提供的负载；帧头 payload_len
 * 可被 ovr 覆盖（用于 0/513 等非法声明值：解析器必须在收头阶段就拒绝）。 */
static u32 build_frame(u8 *out, u8 type, u32 seq, u32 offset,
                       const u8 *payload, u16 copiedLen, const Ovr_t *ov)
{
    CRC32_StreamCtx_t c;
    u32 crc;
    u32 n = BOOTPROTO_HEADER_LEN;

    out[0] = 'I'; out[1] = 'A'; out[2] = 'P'; out[3] = '1';
    out[4] = (ov && ov->hasVersion)  ? ov->version : (u8)BOOTPROTO_VERSION;
    out[5] = (ov && ov->hasType)     ? ov->type   : type;
    put_u16le(out + 6,  (ov && ov->hasHdrLen)     ? ov->hdrLen     : (u16)BOOTPROTO_HEADER_LEN);
    put_u32le(out + 8,  seq);
    put_u32le(out + 12, offset);
    put_u16le(out + 16, (ov && ov->hasPayloadLen) ? ov->payloadLen : copiedLen);
    put_u16le(out + 18, (ov && ov->hasReserved)   ? ov->reserved   : 0u);
    if (copiedLen && payload) {
        memcpy(out + BOOTPROTO_HEADER_LEN, payload, copiedLen);
        n += copiedLen;
    }
    /* frame_crc32 覆盖 [4,20) + payload，不含 SOF 与 CRC 字段本身 */
    CRC32_Init(&c);
    CRC32_Update(&c, out + 4, 16);
    if (copiedLen && payload)
        CRC32_Update(&c, out + BOOTPROTO_HEADER_LEN, copiedLen);
    crc = CRC32_Final(&c);
    put_u32le(out + 20, crc ^ (ov ? ov->crcXor : 0u));
    return n;
}

static void fill_pattern(u8 *buf, u32 len, u8 seed)
{
    u32 i;
    for (i = 0; i < len; i++) buf[i] = (u8)(seed + i);
}

/*--------------------------- 测试 harness ---------------------------*/

static BootProto_Parser_t  g_p;
static BootProto_Session_t g_s;
static BootProto_Event_t   g_evt;         /* 最近一次会话事件 */
static u8  g_kinds[16];                   /* 最近 16 个会话事件 kind */
static int g_nEvt;
static int g_frames;                      /* onFrame 计数 */
static int g_errors;                      /* onError 计数 */
static int g_dataAcc;                     /* DATA_ACCEPTED 计数（重复不得编程） */
static BootProto_FrameErr_t g_lastErr;

static void on_frame(void *user, const BootProto_Frame_t *f)
{
    (void)user;
    g_frames++;
    BootProto_SessionInput(&g_s, f, &g_evt);
    if (g_nEvt < (int)(sizeof(g_kinds) / sizeof(g_kinds[0])))
        g_kinds[g_nEvt] = g_evt.kind;
    g_nEvt++;
    if (g_evt.kind == BOOTPROTO_EVT_DATA_ACCEPTED)
        g_dataAcc++;
}

static void on_error(void *user, BootProto_FrameErr_t err)
{
    (void)user;
    g_errors++;
    g_lastErr = err;
}

static void harness_reset(void)
{
    BootProto_ParserInit(&g_p, on_frame, on_error, NULL);
    BootProto_SessionInit(&g_s, APP_BASE, APP_CAPACITY);
    memset(&g_evt, 0, sizeof(g_evt));
    memset(g_kinds, 0, sizeof(g_kinds));
    g_nEvt = g_frames = g_errors = g_dataAcc = 0;
    g_lastErr = BOOTPROTO_FRAME_ERR_NONE;
}

/* 默认合法 START（seq=0，target_base=APP_BASE） */
static void send_start(u32 size, u32 crc, u32 ver)
{
    u8 man[16];
    u8 wire[BOOTPROTO_HEADER_LEN + 16];
    u32 n;
    put_u32le(man + 0, APP_BASE);
    put_u32le(man + 4, size);
    put_u32le(man + 8, crc);
    put_u32le(man + 12, ver);
    n = build_frame(wire, BOOTPROTO_TYPE_START, 0, 0, man, 16, &OVR_NONE);
    BootProto_ParserFeed(&g_p, wire, n);
}

/* 可指定 seq / target_base 的 START（负面用例） */
static void send_start_raw(u32 seq, u32 base, u32 size, u32 crc, u32 ver)
{
    u8 man[16];
    u8 wire[BOOTPROTO_HEADER_LEN + 16];
    u32 n;
    put_u32le(man + 0, base);
    put_u32le(man + 4, size);
    put_u32le(man + 8, crc);
    put_u32le(man + 12, ver);
    n = build_frame(wire, BOOTPROTO_TYPE_START, seq, 0, man, 16, &OVR_NONE);
    BootProto_ParserFeed(&g_p, wire, n);
}

static void send_data(u32 seq, u32 offset, const u8 *payload, u16 len)
{
    u8 wire[WIRE_MAX];
    u32 n = build_frame(wire, BOOTPROTO_TYPE_DATA, seq, offset, payload, len, &OVR_NONE);
    BootProto_ParserFeed(&g_p, wire, n);
}

static void send_end(u32 seq)
{
    u8 wire[BOOTPROTO_HEADER_LEN];
    u32 n = build_frame(wire, BOOTPROTO_TYPE_END, seq, 0, NULL, 0, &OVR_NONE);
    BootProto_ParserFeed(&g_p, wire, n);
}

/*--------------------------- 1. 正常流程 ---------------------------*/

static void s1_normal_flow(void)
{
    printf("== 1. normal START -> DATA xN -> END ==\n");

    harness_reset();
    send_start(10, 0xAAAA1111uL, 3);
    CHECK_EQ_DESC("s1 start accepted", g_evt.kind, BOOTPROTO_EVT_START_ACCEPTED);
    CHECK_EQ_DESC("s1 start base", g_evt.startBase, APP_BASE);
    CHECK_EQ_DESC("s1 start size", g_evt.startSize, 10);
    CHECK_EQ_DESC("s1 start crc", g_evt.startCrc, 0xAAAA1111uL);
    CHECK_EQ_DESC("s1 start version", g_evt.startVersion, 3);

    {
        u8 d1[4] = {1, 2, 3, 4};
        u8 d2[6] = {5, 6, 7, 8, 9, 10};
        send_data(1, 0, d1, 4);
        CHECK_EQ_DESC("s1 data1 accepted", g_evt.kind, BOOTPROTO_EVT_DATA_ACCEPTED);
        CHECK_EQ_DESC("s1 data1 offset", g_evt.dataOffset, 0);
        CHECK_EQ_DESC("s1 data1 len", g_evt.dataLen, 4);
        CHECK_DESC("s1 data1 payload", g_evt.payload != NULL &&
                   memcmp(g_evt.payload, d1, 4) == 0);
        send_data(2, 4, d2, 6);
        CHECK_EQ_DESC("s1 data2 accepted", g_evt.kind, BOOTPROTO_EVT_DATA_ACCEPTED);
        CHECK_EQ_DESC("s1 data2 offset", g_evt.dataOffset, 4);
        send_end(3);
        CHECK_EQ_DESC("s1 end accepted", g_evt.kind, BOOTPROTO_EVT_END_ACCEPTED);
    }
    CHECK_EQ_DESC("s1 frame count", g_frames, 4);
    CHECK_EQ_DESC("s1 error count", g_errors, 0);

    /* 1025 B：512+512+1（最终奇数字节包） */
    harness_reset();
    send_start(1025, 0, 1);
    {
        u8 big[512];
        u8 last[1] = {0xAB};
        fill_pattern(big, sizeof(big), 0x40);
        send_data(1, 0, big, 512);
        CHECK_EQ_DESC("s1 512B accepted", g_evt.kind, BOOTPROTO_EVT_DATA_ACCEPTED);
        CHECK_EQ_DESC("s1 512B len", g_evt.dataLen, 512);
        send_data(2, 512, big, 512);
        CHECK_EQ_DESC("s1 second 512B accepted", g_evt.kind, BOOTPROTO_EVT_DATA_ACCEPTED);
        send_data(3, 1024, last, 1);
        CHECK_EQ_DESC("s1 final odd accepted", g_evt.kind, BOOTPROTO_EVT_DATA_ACCEPTED);
        CHECK_EQ_DESC("s1 final odd offset", g_evt.dataOffset, 1024);
        CHECK_EQ_DESC("s1 final odd len", g_evt.dataLen, 1);
        send_end(4);
        CHECK_EQ_DESC("s1 end after odd final", g_evt.kind, BOOTPROTO_EVT_END_ACCEPTED);
    }
    CHECK_EQ_DESC("s1 big session frames", g_frames, 5);
    CHECK_EQ_DESC("s1 big session errors", g_errors, 0);
}

/*--------------------------- 2. 解析器分片鲁棒性 ---------------------------*/

static void s2_parser_robustness(void)
{
    u8 man[16];
    u8 wire[WIRE_MAX];
    u32 n;

    printf("== 2. fragmentation / ring-tail / noise / back-to-back ==\n");

    put_u32le(man + 0, APP_BASE);
    put_u32le(man + 4, 64);
    put_u32le(man + 8, 0x12345678uL);
    put_u32le(man + 12, 1);

    /* 2.1 逐字节喂入（header 被拆成 24 个 DMA 片段的极限） */
    harness_reset();
    n = build_frame(wire, BOOTPROTO_TYPE_START, 0, 0, man, 16, &OVR_NONE);
    {
        u32 i;
        for (i = 0; i < n; i++)
            BootProto_ParserFeed(&g_p, wire + i, 1);
    }
    CHECK_EQ_DESC("s2 byte-by-byte frames", g_frames, 1);
    CHECK_EQ_DESC("s2 byte-by-byte errors", g_errors, 0);
    CHECK_EQ_DESC("s2 byte-by-byte event", g_evt.kind, BOOTPROTO_EVT_START_ACCEPTED);

    /* 2.2 任意位置切两刀（含 header 中间 / 头负载边界 / payload 中间）：
     *    payload 跨环尾 = 调用方把环形缓冲的两段连续区分别喂入，
     *    对解析器的契约就是任意分片等价。 */
    {
        u8 pay[300];
        int allOk = 1;
        u32 k;
        fill_pattern(pay, sizeof(pay), 0x11);
        n = build_frame(wire, BOOTPROTO_TYPE_DATA, 1, 0, pay, (u16)sizeof(pay), &OVR_NONE);
        for (k = 0; k <= n; k++) {
            harness_reset();
            send_start(1024, 0, 1);
            BootProto_ParserFeed(&g_p, wire, k);
            BootProto_ParserFeed(&g_p, wire + k, n - k);
            if (g_frames != 2 || g_errors != 0 ||
                g_evt.kind != BOOTPROTO_EVT_DATA_ACCEPTED ||
                g_evt.dataLen != sizeof(pay) ||
                g_evt.payload == NULL ||
                memcmp(g_evt.payload, pay, sizeof(pay)) != 0)
                allOk = 0;
        }
        CHECK_DESC("s2 split at every position decodes identically", allOk);
    }

    /* 2.3 SOF 前有噪声（含尾部半截 SOF "IA"） */
    harness_reset();
    {
        u8 noise[40];
        u32 i;
        for (i = 0; i < sizeof(noise); i++)
            noise[i] = (u8)('x' + (i % 3));   /* xyz 循环，不含 "IAP1" */
        noise[37] = 'I';
        noise[38] = 'A';                       /* 噪声结尾半截 SOF */
        n = build_frame(wire, BOOTPROTO_TYPE_START, 0, 0, man, 16, &OVR_NONE);
        BootProto_ParserFeed(&g_p, noise, sizeof(noise));
        BootProto_ParserFeed(&g_p, wire, n);
    }
    CHECK_EQ_DESC("s2 noise frames", g_frames, 1);
    CHECK_EQ_DESC("s2 noise errors", g_errors, 0);
    CHECK_EQ_DESC("s2 noise event", g_evt.kind, BOOTPROTO_EVT_START_ACCEPTED);

    /* 2.4 单次 Feed 内连续两个帧 */
    harness_reset();
    {
        u8 two[WIRE_MAX * 2];
        u8 pay[4] = {0xAA, 0xBB, 0xCC, 0xDD};
        u32 n1 = build_frame(two, BOOTPROTO_TYPE_START, 0, 0, man, 16, &OVR_NONE);
        u32 n2 = build_frame(two + n1, BOOTPROTO_TYPE_DATA, 1, 0, pay, 4, &OVR_NONE);
        BootProto_ParserFeed(&g_p, two, n1 + n2);
        CHECK_EQ_DESC("s2 back-to-back frames", g_frames, 2);
        CHECK_EQ_DESC("s2 back-to-back errors", g_errors, 0);
        CHECK_EQ_DESC("s2 back-to-back kind[0]", g_kinds[0], BOOTPROTO_EVT_START_ACCEPTED);
        CHECK_EQ_DESC("s2 back-to-back kind[1]", g_kinds[1], BOOTPROTO_EVT_DATA_ACCEPTED);
        CHECK_DESC("s2 back-to-back payload", g_evt.payload != NULL &&
                   memcmp(g_evt.payload, pay, 4) == 0);
    }
}

/*--------------------------- 3. 帧级字段校验 ---------------------------*/

static void expect_frame_error(const char *desc, u8 type, u32 seq, u32 offset,
                               const u8 *payload, u16 copiedLen, const Ovr_t *ovr,
                               BootProto_FrameErr_t want)
{
    u8 wire[WIRE_MAX];
    char d2[160];
    u32 n;

    harness_reset();
    n = build_frame(wire, type, seq, offset, payload, copiedLen, ovr);
    BootProto_ParserFeed(&g_p, wire, n);
    sprintf(d2, "%s [err]", desc);
    CHECK_EQ_DESC(d2, g_lastErr, want);
    sprintf(d2, "%s [frames=0]", desc);
    CHECK_EQ_DESC(d2, g_frames, 0);
    sprintf(d2, "%s [errors=1]", desc);
    CHECK_EQ_DESC(d2, g_errors, 1);
}

static void s3_frame_validation(void)
{
    u8 man[16];
    u8 pay[4] = {0x11, 0x22, 0x33, 0x44};

    printf("== 3. header/payload_len/CRC frame rejection ==\n");

    put_u32le(man + 0, APP_BASE);
    put_u32le(man + 4, 64);
    put_u32le(man + 8, 0);
    put_u32le(man + 12, 1);

    {
        Ovr_t o;
        memset(&o, 0, sizeof(o));

        o.hasVersion = 1; o.version = 0;
        expect_frame_error("s3 version=0", BOOTPROTO_TYPE_START, 0, 0,
                           man, 16, &o, BOOTPROTO_FRAME_ERR_VERSION);
        o.version = 2;
        expect_frame_error("s3 version=2", BOOTPROTO_TYPE_START, 0, 0,
                           man, 16, &o, BOOTPROTO_FRAME_ERR_VERSION);
        memset(&o, 0, sizeof(o));

        o.hasType = 1; o.type = 0x00;
        expect_frame_error("s3 type=0x00", BOOTPROTO_TYPE_START, 0, 0,
                           man, 16, &o, BOOTPROTO_FRAME_ERR_TYPE);
        o.type = 0x06;
        expect_frame_error("s3 type=0x06", BOOTPROTO_TYPE_START, 0, 0,
                           man, 16, &o, BOOTPROTO_FRAME_ERR_TYPE);
        o.type = BOOTPROTO_TYPE_ACK;    /* ACK/NACK 仅 Boot->主机方向 */
        expect_frame_error("s3 host sends ACK", BOOTPROTO_TYPE_ACK, 0, 0,
                           man, 16, &o, BOOTPROTO_FRAME_ERR_TYPE);
        o.type = BOOTPROTO_TYPE_NACK;
        expect_frame_error("s3 host sends NACK", BOOTPROTO_TYPE_NACK, 0, 0,
                           man, 16, &o, BOOTPROTO_FRAME_ERR_TYPE);
        memset(&o, 0, sizeof(o));

        o.hasHdrLen = 1; o.hdrLen = 23;
        expect_frame_error("s3 header_len=23", BOOTPROTO_TYPE_START, 0, 0,
                           man, 16, &o, BOOTPROTO_FRAME_ERR_HEADER_LEN);
        o.hdrLen = 25;
        expect_frame_error("s3 header_len=25", BOOTPROTO_TYPE_START, 0, 0,
                           man, 16, &o, BOOTPROTO_FRAME_ERR_HEADER_LEN);
        memset(&o, 0, sizeof(o));

        o.hasReserved = 1; o.reserved = 1;
        expect_frame_error("s3 reserved=1", BOOTPROTO_TYPE_START, 0, 0,
                           man, 16, &o, BOOTPROTO_FRAME_ERR_RESERVED);
        memset(&o, 0, sizeof(o));

        /* DATA 声明 payload_len=0 / 513：解析器必须在收头阶段拒绝 */
        o.hasPayloadLen = 1; o.payloadLen = 0;
        expect_frame_error("s3 DATA payload_len=0", BOOTPROTO_TYPE_DATA, 0, 0,
                           pay, 4, &o, BOOTPROTO_FRAME_ERR_PAYLOAD_LEN);
        o.payloadLen = 513;
        expect_frame_error("s3 DATA payload_len=513", BOOTPROTO_TYPE_DATA, 0, 0,
                           pay, 4, &o, BOOTPROTO_FRAME_ERR_PAYLOAD_LEN);
        memset(&o, 0, sizeof(o));

        o.hasPayloadLen = 1; o.payloadLen = 15;   /* START 必须恰 16 */
        expect_frame_error("s3 START payload_len=15", BOOTPROTO_TYPE_START, 0, 0,
                           man, 16, &o, BOOTPROTO_FRAME_ERR_PAYLOAD_LEN);
        o.payloadLen = 17;
        expect_frame_error("s3 START payload_len=17", BOOTPROTO_TYPE_START, 0, 0,
                           man, 16, &o, BOOTPROTO_FRAME_ERR_PAYLOAD_LEN);
        memset(&o, 0, sizeof(o));

        o.hasPayloadLen = 1; o.payloadLen = 1;    /* END 必须恰 0 */
        expect_frame_error("s3 END payload_len=1", BOOTPROTO_TYPE_END, 0, 0,
                           pay, 1, &o, BOOTPROTO_FRAME_ERR_PAYLOAD_LEN);
        memset(&o, 0, sizeof(o));

        o.crcXor = 1;                              /* CRC 字段被破坏 */
        expect_frame_error("s3 frame CRC corrupt", BOOTPROTO_TYPE_DATA, 0, 0,
                           pay, 4, &o, BOOTPROTO_FRAME_ERR_CRC);
        memset(&o, 0, sizeof(o));
    }

    /* 负载字节被破坏 -> 帧 CRC 不匹配 */
    {
        u8 wire[WIRE_MAX];
        u32 n;
        harness_reset();
        n = build_frame(wire, BOOTPROTO_TYPE_DATA, 0, 0, pay, 4, &OVR_NONE);
        wire[BOOTPROTO_HEADER_LEN + 2] ^= 0xFF;
        BootProto_ParserFeed(&g_p, wire, n);
        CHECK_EQ_DESC("s3 payload corrupt [err]", g_lastErr, BOOTPROTO_FRAME_ERR_CRC);
        CHECK_EQ_DESC("s3 payload corrupt [frames=0]", g_frames, 0);
        CHECK_EQ_DESC("s3 payload corrupt [errors=1]", g_errors, 1);
    }

    /* 正向对照：DATA payload_len=512 边界合法 */
    {
        u8 big[512];
        u8 wire[WIRE_MAX];
        u32 n;
        fill_pattern(big, sizeof(big), 0x77);
        harness_reset();
        n = build_frame(wire, BOOTPROTO_TYPE_DATA, 0, 0, big, (u16)sizeof(big), &OVR_NONE);
        BootProto_ParserFeed(&g_p, wire, n);
        CHECK_EQ_DESC("s3 DATA 512 boundary frames", g_frames, 1);
        CHECK_EQ_DESC("s3 DATA 512 boundary errors", g_errors, 0);
    }
}

/*--------------------------- 4. 顺序 / 重复 ---------------------------*/

static void s4_ordering(void)
{
    u8 pay[8];
    fill_pattern(pay, sizeof(pay), 0x00);

    printf("== 4. overflow / bounds / seq jump / reorder / duplicate ==\n");

    /* 4.1 跳号：DATA seq 超前（跳过 1），期望值回填正确，可恢复 */
    harness_reset();
    send_start(100, 0, 1);
    send_data(2, 0, pay, 4);
    CHECK_EQ_DESC("s4 seq jump kind", g_evt.kind, BOOTPROTO_EVT_NACK);
    CHECK_EQ_DESC("s4 seq jump reason", g_evt.nackReason, BOOTPROTO_NACK_BAD_SEQ);
    CHECK_EQ_DESC("s4 seq jump expected seq", g_evt.expectedSeq, 1);
    CHECK_EQ_DESC("s4 seq jump expected offset", g_evt.expectedOffset, 0);
    send_data(1, 0, pay, 4);
    CHECK_EQ_DESC("s4 recovery after jump", g_evt.kind, BOOTPROTO_EVT_DATA_ACCEPTED);

    /* 4.2 乱序：重发更早（非上一包）的帧 */
    harness_reset();
    send_start(100, 0, 1);
    send_data(1, 0, pay, 4);
    send_data(2, 4, pay, 4);
    send_data(3, 8, pay, 4);
    send_data(1, 0, pay, 4);
    CHECK_EQ_DESC("s4 reorder kind", g_evt.kind, BOOTPROTO_EVT_NACK);
    CHECK_EQ_DESC("s4 reorder reason", g_evt.nackReason, BOOTPROTO_NACK_OUT_OF_ORDER);
    CHECK_EQ_DESC("s4 reorder expected seq", g_evt.expectedSeq, 4);

    /* 4.3 上一包重复重发：重新 ACK，不得重复编程，顺序不受影响 */
    harness_reset();
    send_start(100, 0, 1);
    send_data(1, 0, pay, 4);
    CHECK_EQ_DESC("s4 dup dataAcc before", g_dataAcc, 1);
    send_data(1, 0, pay, 4);          /* 完全相同重发 */
    CHECK_EQ_DESC("s4 dup kind", g_evt.kind, BOOTPROTO_EVT_DUPLICATE);
    CHECK_EQ_DESC("s4 dup no second program", g_dataAcc, 1);
    send_data(2, 4, pay, 4);
    CHECK_EQ_DESC("s4 dup then next accepted", g_evt.kind, BOOTPROTO_EVT_DATA_ACCEPTED);
    CHECK_EQ_DESC("s4 dup next offset", g_evt.dataOffset, 4);

    /* 4.4 重复 START（相同 manifest）：幂等重 ACK，会话不被重置 */
    harness_reset();
    send_start(100, 0, 1);
    CHECK_EQ_DESC("s4 dup start first", g_evt.kind, BOOTPROTO_EVT_START_ACCEPTED);
    send_start(100, 0, 1);
    CHECK_EQ_DESC("s4 dup start kind", g_evt.kind, BOOTPROTO_EVT_DUPLICATE);
    send_data(1, 0, pay, 4);
    CHECK_EQ_DESC("s4 dup start then data", g_evt.kind, BOOTPROTO_EVT_DATA_ACCEPTED);
    CHECK_EQ_DESC("s4 dup start data offset", g_evt.dataOffset, 0);

    /* 4.5 会话完成后重复 END / 新帧 */
    harness_reset();
    send_start(8, 0, 1);
    {
        u8 d[8];
        fill_pattern(d, sizeof(d), 0x22);
        send_data(1, 0, d, 8);
        send_end(2);
        CHECK_EQ_DESC("s4 complete end", g_evt.kind, BOOTPROTO_EVT_END_ACCEPTED);
        send_end(2);
        CHECK_EQ_DESC("s4 dup end kind", g_evt.kind, BOOTPROTO_EVT_DUPLICATE);
        send_start(8, 0, 1);
        CHECK_EQ_DESC("s4 start after done kind", g_evt.kind, BOOTPROTO_EVT_NACK);
        CHECK_EQ_DESC("s4 start after done reason", g_evt.nackReason,
                      BOOTPROTO_NACK_SESSION_DONE);
    }

    /* 4.6 NACK 幂等：同一坏帧重发仍是同一拒绝 */
    harness_reset();
    send_start(100, 0, 1);
    send_data(1, 1, pay, 4);          /* offset 未对齐 */
    CHECK_EQ_DESC("s4 nack repeatable 1st", g_evt.nackReason,
                  BOOTPROTO_NACK_MISALIGNED);
    send_data(1, 1, pay, 4);
    CHECK_EQ_DESC("s4 nack repeatable 2nd", g_evt.nackReason,
                  BOOTPROTO_NACK_MISALIGNED);
    CHECK_EQ_DESC("s4 nack no program", g_dataAcc, 0);
}

/*--------------------------- 5. 越界 / 对齐 / 奇偶 ---------------------------*/

static void s5_bounds_align_parity(void)
{
    u8 pay[8];
    u8 big[512];
    fill_pattern(pay, sizeof(pay), 0x05);
    fill_pattern(big, sizeof(big), 0x55);

    printf("== 5. bounds (subtract-first) / alignment / parity ==\n");

    /* 5.1 溢出防护证明：offset=0xFFFFFF00 + len=0x200 朴素写法
     *    `offset+len <= size` 回绕成 0x100=256 <= 1024 会放行；
     *    先减后比实现必须给出 OUT_OF_BOUNDS（原因码即证据）。 */
    harness_reset();
    send_start(1024, 0, 1);
    send_data(1, 0xFFFFFF00uL, big, 512);
    CHECK_EQ_DESC("s5 overflow wrap kind", g_evt.kind, BOOTPROTO_EVT_NACK);
    CHECK_EQ_DESC("s5 overflow wrap reason", g_evt.nackReason,
                  BOOTPROTO_NACK_OUT_OF_BOUNDS);
    CHECK_EQ_DESC("s5 overflow no program", g_dataAcc, 0);

    /* 第二组溢出模式：0xFFFFFFF8 + 0x10 回绕成 8 */
    send_data(1, 0xFFFFFFF8uL, pay, 16);
    CHECK_EQ_DESC("s5 overflow wrap2 reason", g_evt.nackReason,
                  BOOTPROTO_NACK_OUT_OF_BOUNDS);

    /* 5.2 范围内越界：offset==期望值，但 len 超过剩余空间 */
    harness_reset();
    send_start(100, 0, 1);
    send_data(1, 0, pay, 96);
    CHECK_EQ_DESC("s5 near-full accepted", g_evt.kind, BOOTPROTO_EVT_DATA_ACCEPTED);
    send_data(2, 96, pay, 5);          /* 96+5=101 > 100 */
    CHECK_EQ_DESC("s5 in-range overflow reason", g_evt.nackReason,
                  BOOTPROTO_NACK_OUT_OF_BOUNDS);

    /* 5.3 offset 非 2 字节对齐 */
    harness_reset();
    send_start(100, 0, 1);
    send_data(1, 1, pay, 4);
    CHECK_EQ_DESC("s5 misaligned reason", g_evt.nackReason,
                  BOOTPROTO_NACK_MISALIGNED);

    /* 5.4 对齐但非期望 offset */
    harness_reset();
    send_start(100, 0, 1);
    send_data(1, 4, pay, 4);
    CHECK_EQ_DESC("s5 wrong offset reason", g_evt.nackReason,
                  BOOTPROTO_NACK_BAD_OFFSET);

    /* 5.5 非最终包奇数长度拒绝 */
    harness_reset();
    send_start(100, 0, 1);
    send_data(1, 0, pay, 3);
    CHECK_EQ_DESC("s5 odd non-final reason", g_evt.nackReason,
                  BOOTPROTO_NACK_ODD_LENGTH);

    /* 5.6 最终奇数字节包正确接受并 END */
    harness_reset();
    send_start(101, 0, 1);
    send_data(1, 0, pay, 100);
    CHECK_EQ_DESC("s5 5.6 full even accepted", g_evt.kind, BOOTPROTO_EVT_DATA_ACCEPTED);
    send_data(2, 100, pay, 1);
    CHECK_EQ_DESC("s5 final odd accepted", g_evt.kind, BOOTPROTO_EVT_DATA_ACCEPTED);
    send_end(3);
    CHECK_EQ_DESC("s5 end after final odd", g_evt.kind, BOOTPROTO_EVT_END_ACCEPTED);

    /* 5.7 512 B 满包边界 */
    harness_reset();
    send_start(1024, 0, 1);
    send_data(1, 0, big, 512);
    CHECK_EQ_DESC("s5 512 first accepted", g_evt.kind, BOOTPROTO_EVT_DATA_ACCEPTED);
    send_data(2, 512, big, 512);
    CHECK_EQ_DESC("s5 512 second accepted", g_evt.kind, BOOTPROTO_EVT_DATA_ACCEPTED);
    send_end(3);
    CHECK_EQ_DESC("s5 512 end accepted", g_evt.kind, BOOTPROTO_EVT_END_ACCEPTED);
    CHECK_EQ_DESC("s5 512 errors", g_errors, 0);
}

/*--------------------------- 6. START manifest 校验 ---------------------------*/

static void s6_start_manifest(void)
{
    u8 pay[4] = {0, 0, 0, 0};

    printf("== 6. START manifest validation (end-to-end) ==\n");

    /* target_base 不是 APP_BASE */
    harness_reset();
    send_start_raw(0, APP_BASE + 4, 100, 0, 1);
    CHECK_EQ_DESC("s6 bad base kind", g_evt.kind, BOOTPROTO_EVT_NACK);
    CHECK_EQ_DESC("s6 bad base reason", g_evt.nackReason,
                  BOOTPROTO_NACK_BAD_START_BASE);

    /* size < 8（含 0） */
    harness_reset();
    send_start_raw(0, APP_BASE, 7, 0, 1);
    CHECK_EQ_DESC("s6 size=7 reason", g_evt.nackReason,
                  BOOTPROTO_NACK_BAD_START_SIZE);
    send_start_raw(0, APP_BASE, 0, 0, 1);
    CHECK_EQ_DESC("s6 size=0 reason", g_evt.nackReason,
                  BOOTPROTO_NACK_BAD_START_SIZE);

    /* size = APP_CAPACITY+1 */
    harness_reset();
    send_start_raw(0, APP_BASE, APP_CAPACITY + 1, 0, 1);
    CHECK_EQ_DESC("s6 size=cap+1 reason", g_evt.nackReason,
                  BOOTPROTO_NACK_BAD_START_SIZE);

    /* 边界：size = APP_CAPACITY 与 size = 8 均接受 */
    harness_reset();
    send_start_raw(0, APP_BASE, APP_CAPACITY, 0, 1);
    CHECK_EQ_DESC("s6 size=cap accepted", g_evt.kind, BOOTPROTO_EVT_START_ACCEPTED);
    CHECK_EQ_DESC("s6 size=cap value", g_evt.startSize, APP_CAPACITY);

    harness_reset();
    send_start_raw(0, APP_BASE, 8, 0, 1);
    CHECK_EQ_DESC("s6 size=8 accepted", g_evt.kind, BOOTPROTO_EVT_START_ACCEPTED);
    send_data(1, 0, pay, 8);
    CHECK_EQ_DESC("s6 size=8 final data", g_evt.kind, BOOTPROTO_EVT_DATA_ACCEPTED);
    send_end(2);
    CHECK_EQ_DESC("s6 size=8 end", g_evt.kind, BOOTPROTO_EVT_END_ACCEPTED);

    /* START sequence != 0 */
    harness_reset();
    send_start_raw(5, APP_BASE, 100, 0, 1);
    CHECK_EQ_DESC("s6 start seq=5 reason", g_evt.nackReason,
                  BOOTPROTO_NACK_BAD_SEQ);
    CHECK_EQ_DESC("s6 start seq=5 expected", g_evt.expectedSeq, 0);

    /* DATA / END 先于 START */
    harness_reset();
    send_data(0, 0, pay, 4);
    CHECK_EQ_DESC("s6 data before start reason", g_evt.nackReason,
                  BOOTPROTO_NACK_NOT_ACTIVE);
    harness_reset();
    send_end(0);
    CHECK_EQ_DESC("s6 end before start reason", g_evt.nackReason,
                  BOOTPROTO_NACK_NOT_ACTIVE);

    /* 会话进行中收到不同 manifest 的 START */
    harness_reset();
    send_start(100, 0, 1);
    send_start_raw(1, APP_BASE, 200, 0, 1);
    CHECK_EQ_DESC("s6 conflicting start reason", g_evt.nackReason,
                  BOOTPROTO_NACK_SESSION_BUSY);

    /* 会话状态机独立可 mock：不经解析器直接喂帧结构体 */
    {
        BootProto_Session_t s;
        BootProto_Event_t ev;
        u8 man[16];
        BootProto_Frame_t f;

        put_u32le(man + 0, APP_BASE);
        put_u32le(man + 4, 64);
        put_u32le(man + 8, 0xDEADBEEFuL);
        put_u32le(man + 12, 9);

        memset(&f, 0, sizeof(f));
        f.type = BOOTPROTO_TYPE_START;
        f.sequence = 0;
        f.imageOffset = 0;
        f.payloadLen = BOOTPROTO_START_PAYLOAD_LEN;
        f.payload = man;

        BootProto_SessionInit(&s, APP_BASE, APP_CAPACITY);
        BootProto_SessionInput(&s, &f, &ev);
        CHECK_EQ_DESC("s6 direct start kind", ev.kind,
                      BOOTPROTO_EVT_START_ACCEPTED);
        CHECK_EQ_DESC("s6 direct start crc", ev.startCrc, 0xDEADBEEFuL);
        CHECK_EQ_DESC("s6 direct start version", ev.startVersion, 9);

        f.type = BOOTPROTO_TYPE_DATA;
        f.sequence = 1;
        f.imageOffset = 0;
        f.payloadLen = 4;
        f.payload = pay;
        BootProto_SessionInput(&s, &f, &ev);
        CHECK_EQ_DESC("s6 direct data kind", ev.kind,
                      BOOTPROTO_EVT_DATA_ACCEPTED);
        CHECK_EQ_DESC("s6 direct data len", ev.dataLen, 4);
    }
}

/*--------------------------- 7. 超时复位与重同步 ---------------------------*/

static void s7_timeout_reset(void)
{
    u8 man[16];
    u8 wire[WIRE_MAX];
    u32 n;
    u8 big[512];

    printf("== 7. timeout reset & resync (no Flash access) ==\n");

    put_u32le(man + 0, APP_BASE);
    put_u32le(man + 4, 64);
    put_u32le(man + 8, 0);
    put_u32le(man + 12, 1);
    fill_pattern(big, sizeof(big), 0x66);

    /* 7.1 半截 header 后超时复位，再整帧重发 */
    harness_reset();
    n = build_frame(wire, BOOTPROTO_TYPE_START, 0, 0, man, 16, &OVR_NONE);
    BootProto_ParserFeed(&g_p, wire, 10);
    CHECK_EQ_DESC("s7 partial header no frame", g_frames, 0);
    BootProto_ParserReset(&g_p);
    BootProto_ParserFeed(&g_p, wire, n);
    CHECK_EQ_DESC("s7 resync after mid-header reset", g_frames, 1);
    CHECK_EQ_DESC("s7 resync errors", g_errors, 0);
    CHECK_EQ_DESC("s7 resync event", g_evt.kind, BOOTPROTO_EVT_START_ACCEPTED);

    /* 7.2 半截 payload 后超时复位；会话状态不受影响（仍期望 seq=1/offset=0） */
    harness_reset();
    send_start(1024, 0, 1);
    n = build_frame(wire, BOOTPROTO_TYPE_DATA, 1, 0, big, (u16)sizeof(big), &OVR_NONE);
    BootProto_ParserFeed(&g_p, wire, 300);
    CHECK_EQ_DESC("s7 partial payload no frame", g_frames, 1);   /* 仅 START */
    BootProto_ParserReset(&g_p);
    BootProto_ParserFeed(&g_p, wire, n);
    CHECK_EQ_DESC("s7 resync after mid-payload reset frames", g_frames, 2);
    CHECK_EQ_DESC("s7 resync errors 2", g_errors, 0);
    CHECK_EQ_DESC("s7 session survives reset", g_evt.kind,
                  BOOTPROTO_EVT_DATA_ACCEPTED);
    CHECK_EQ_DESC("s7 session offset kept", g_evt.dataOffset, 0);
    CHECK_EQ_DESC("s7 session len kept", g_evt.dataLen, 512);

    /* 7.3 复位后先噪声再整帧（重同步鲁棒性） */
    harness_reset();
    {
        u8 noise[16];
        u32 i;
        for (i = 0; i < sizeof(noise); i++)
            noise[i] = (u8)(0xE0 + i);
        BootProto_ParserFeed(&g_p, wire, 30);     /* 半截旧帧 */
        BootProto_ParserReset(&g_p);
        BootProto_ParserFeed(&g_p, noise, sizeof(noise));
        n = build_frame(wire, BOOTPROTO_TYPE_START, 0, 0, man, 16, &OVR_NONE);
        BootProto_ParserFeed(&g_p, wire, n);
        CHECK_EQ_DESC("s7 noise then frame frames", g_frames, 1);
        CHECK_EQ_DESC("s7 noise then frame errors", g_errors, 0);
        CHECK_EQ_DESC("s7 noise then frame event", g_evt.kind,
                      BOOTPROTO_EVT_START_ACCEPTED);
    }

    /* 7.4 不访问 Flash：本套件仅链接 boot_protocol.c + crc32.c，
     *    无任何 Flash/SPL 符号可解析——链接通过本身即证据。 */
    printf("  ok s7 flash independence (suite links no flash backend)\n");
    g_total++;
}

/*--------------------------- main ---------------------------*/

int main(void)
{
    s1_normal_flow();
    s2_parser_robustness();
    s3_frame_validation();
    s4_ordering();
    s5_bounds_align_parity();
    s6_start_manifest();
    s7_timeout_reset();

    printf("boot-protocol: %d checks, %d failed\n", g_total, g_failed);
    if (g_failed == 0) {
        printf("boot-protocol: all cases passed\n");
        return 0;
    }
    printf("boot-protocol: some cases FAILED\n");
    return 1;
}
