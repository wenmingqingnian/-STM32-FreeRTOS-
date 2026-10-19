// SPDX-License-Identifier: GPL-2.0
/*
 * IAP 帧协议解析器与升级会话状态机实现
 *
 * 见 boot_protocol.h 的协议契约。本文件只做纯逻辑，不访问任何寄存器
 * 或 Flash——"不访问 Flash"由 protocol mock 套件的链接集证明：
 * 只链接本文件 + BSP/CRC32/crc32.c，任何 Flash/SPL 符号引用都会链接失败。
 *
 * 解析器状态：SYNC（滑窗找 SOF）-> HDR（24 B 收齐即校验字段）
 *             -> PAYLOAD（收满声明的 payload_len 后验帧 CRC）
 * 会话状态：  IDLE -> ACTIVE（START 接受）-> DONE（END 接受）
 *
 */
#include <string.h>
#include "boot_protocol.h"

/*-----------------------------------------------------------
 * 小端解码（逐字节拼装，禁止结构体强转/memcpy 解码）
 *----------------------------------------------------------*/
static u16 bp_rd16(const u8 *p)
{
    return (u16)((u16)p[0] | (u16)((u16)p[1] << 8));
}

static u32 bp_rd32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

/* SOF 滑窗匹配值："IAP1" 依字节移入后的窗口值 */
#define BP_SOF_WINDOW   0x49415031uL   /* 'I'<<24 | 'A'<<16 | 'P'<<8 | '1' */

/* 解析器内部状态 */
#define BP_PSTATE_SYNC      0u
#define BP_PSTATE_HDR       1u
#define BP_PSTATE_PAYLOAD   2u

/* 会话内部状态 */
#define BP_SSTATE_IDLE      0u
#define BP_SSTATE_ACTIVE    1u
#define BP_SSTATE_DONE      2u

/*-----------------------------------------------------------
 * 解析器
 *----------------------------------------------------------*/

void BootProto_ParserInit(BootProto_Parser_t *p,
                           void (*onFrame)(void *user, const BootProto_Frame_t *frame),
                           void (*onError)(void *user, BootProto_FrameErr_t err),
                           void *user)
{
    if (p == NULL)
        return;
    memset(p, 0, sizeof(*p));
    p->onFrame = onFrame;
    p->onError = onError;
    p->user = user;
}

void BootProto_ParserReset(BootProto_Parser_t *p)
{
    if (p == NULL)
        return;
    /* 只清半截帧状态，回调与 user 配置保留 */
    p->state = BP_PSTATE_SYNC;
    p->syncWindow = 0;
    p->hdrFilled = 0;
    p->payloadFilled = 0;
}

/* 帧完成（接受或拒绝）后统一回到同步扫描 */
static void bp_to_sync(BootProto_Parser_t *p)
{
    p->state = BP_PSTATE_SYNC;
    p->syncWindow = 0;
    p->hdrFilled = 0;
    p->payloadFilled = 0;
}

static void bp_emit_error(BootProto_Parser_t *p, BootProto_FrameErr_t err)
{
    if (p->onError != NULL)
        p->onError(p->user, err);
}

/* 24 B 帧头收齐后的字段校验：合法返回 1 并回填 *need（该类型允许的
 * payload 长度已在下面按类型判死，收头阶段即可拒绝非法声明值）。 */
static int bp_header_validate(BootProto_Parser_t *p, u16 *need,
                               BootProto_FrameErr_t *err)
{
    u8  ver      = p->hdr[4];
    u8  type     = p->hdr[5];
    u16 hdrLen   = bp_rd16(p->hdr + 6);
    u16 plen     = bp_rd16(p->hdr + 16);
    u16 reserved = bp_rd16(p->hdr + 18);

    if (ver != (u8)BOOTPROTO_VERSION) {
        *err = BOOTPROTO_FRAME_ERR_VERSION;
        return 0;
    }
    if (hdrLen != (u16)BOOTPROTO_HEADER_LEN) {
        *err = BOOTPROTO_FRAME_ERR_HEADER_LEN;
        return 0;
    }
    if (reserved != 0) {
        *err = BOOTPROTO_FRAME_ERR_RESERVED;
        return 0;
    }
    if (type != (u8)BOOTPROTO_TYPE_START &&
        type != (u8)BOOTPROTO_TYPE_DATA &&
        type != (u8)BOOTPROTO_TYPE_END) {
        *err = BOOTPROTO_FRAME_ERR_TYPE;   /* ACK/NACK 仅 Boot->主机方向 */
        return 0;
    }
    if (type == (u8)BOOTPROTO_TYPE_START) {
        if (plen != (u16)BOOTPROTO_START_PAYLOAD_LEN) {
            *err = BOOTPROTO_FRAME_ERR_PAYLOAD_LEN;
            return 0;
        }
    } else if (type == (u8)BOOTPROTO_TYPE_DATA) {
        if (plen == 0 || plen > (u16)BOOTPROTO_MAX_PAYLOAD) {
            *err = BOOTPROTO_FRAME_ERR_PAYLOAD_LEN;
            return 0;
        }
    } else { /* END */
        if (plen != 0) {
            *err = BOOTPROTO_FRAME_ERR_PAYLOAD_LEN;
            return 0;
        }
    }
    *need = plen;
    return 1;
}

/* 整帧收齐：验帧 CRC -> onFrame。payload 指向解析器内部缓冲，
 * 回调返回后本函数才回到同步扫描，故回调内读 payload 安全。 */
static void bp_frame_complete(BootProto_Parser_t *p)
{
    BootProto_Frame_t f;
    u32 declared = bp_rd32(p->hdr + 20);

    if (CRC32_Final(&p->crc) != declared) {
        bp_emit_error(p, BOOTPROTO_FRAME_ERR_CRC);
        bp_to_sync(p);
        return;
    }

    memset(&f, 0, sizeof(f));
    f.version    = p->hdr[4];
    f.type       = p->hdr[5];
    f.headerLen  = bp_rd16(p->hdr + 6);
    f.sequence   = bp_rd32(p->hdr + 8);
    f.imageOffset = bp_rd32(p->hdr + 12);
    f.payloadLen = bp_rd16(p->hdr + 16);
    f.reserved   = bp_rd16(p->hdr + 18);
    f.frameCrc   = declared;
    f.payload    = (p->payloadNeed > 0) ? p->payload : NULL;

    if (p->onFrame != NULL)
        p->onFrame(p->user, &f);
    bp_to_sync(p);
}

void BootProto_ParserFeed(BootProto_Parser_t *p, const u8 *data, u32 len)
{
    u32 i;

    if (p == NULL || data == NULL)
        return;

    for (i = 0; i < len; i++) {
        u8 b = data[i];

        switch (p->state) {
        case BP_PSTATE_SYNC:
            /* 滑窗同步：噪声字节静默丢弃，尾部半截 SOF 不会误同步 */
            p->syncWindow = (p->syncWindow << 8) | (u32)b;
            if (p->syncWindow == BP_SOF_WINDOW) {
                p->hdr[0] = 'I';
                p->hdr[1] = 'A';
                p->hdr[2] = 'P';
                p->hdr[3] = '1';
                p->hdrFilled = 4u;
                p->payloadFilled = 0u;
                CRC32_Init(&p->crc);   /* 覆盖 [4,20)+payload，不含 SOF/CRC 字段 */
                p->state = BP_PSTATE_HDR;
            }
            break;

        case BP_PSTATE_HDR: {
            u32 idx = p->hdrFilled;
            p->hdr[p->hdrFilled++] = b;
            if (idx >= 4u && idx < 20u)
                CRC32_Update(&p->crc, &b, 1u);   /* 头部 [4,20) 计入帧 CRC */
            if (p->hdrFilled >= (u16)BOOTPROTO_HEADER_LEN) {
                u16 need = 0;
                BootProto_FrameErr_t err = BOOTPROTO_FRAME_ERR_NONE;
                if (bp_header_validate(p, &need, &err)) {
                    p->payloadNeed = need;
                    p->payloadFilled = 0u;
                    if (need == 0)
                        bp_frame_complete(p);   /* END：无负载直接验 CRC */
                    else
                        p->state = BP_PSTATE_PAYLOAD;
                } else {
                    bp_emit_error(p, err);
                    bp_to_sync(p);
                }
            }
            break;
        }

        case BP_PSTATE_PAYLOAD:
            p->payload[p->payloadFilled++] = b;
            CRC32_Update(&p->crc, &b, 1u);
            if (p->payloadFilled >= p->payloadNeed)
                bp_frame_complete(p);
            break;

        default:
            bp_to_sync(p);
            break;
        }
    }
}

/*-----------------------------------------------------------
 * 升级会话状态机
 *----------------------------------------------------------*/

void BootProto_SessionInit(BootProto_Session_t *s, u32 appBase, u32 appCapacity)
{
    if (s == NULL)
        return;
    memset(s, 0, sizeof(*s));
    s->appBase = appBase;
    s->appCapacity = appCapacity;
}

void BootProto_SessionReset(BootProto_Session_t *s)
{
    u32 base, cap;

    if (s == NULL)
        return;
    base = s->appBase;
    cap = s->appCapacity;
    memset(s, 0, sizeof(*s));
    s->appBase = base;
    s->appCapacity = cap;
}

/* NACK 输出（expectedSeq/expectedOffset 已在入口回填当前期望值） */
static void bp_nack(BootProto_Event_t *out, u8 reason)
{
    out->kind = (u8)BOOTPROTO_EVT_NACK;
    out->nackReason = reason;
}

/* START manifest 解码（经解析器到达时 payload 长度已保证为 16 B） */
static void bp_decode_manifest(const BootProto_Frame_t *f,
                                u32 *base, u32 *size, u32 *crc, u32 *ver)
{
    *base = bp_rd32(f->payload + 0);
    *size = bp_rd32(f->payload + 4);
    *crc  = bp_rd32(f->payload + 8);
    *ver  = bp_rd32(f->payload + 12);
}

/* 与最近已接受 START 的 manifest 是否一致（重复帧判定用） */
static int bp_manifest_equal(const BootProto_Session_t *s, const BootProto_Frame_t *f)
{
    u32 base, size, crc, ver;

    if (f->payload == NULL || f->payloadLen != (u16)BOOTPROTO_START_PAYLOAD_LEN)
        return 0;
    bp_decode_manifest(f, &base, &size, &crc, &ver);
    return (base == s->lastStartBase) && (size == s->lastStartSize) &&
           (crc == s->lastStartCrc) && (ver == s->lastStartVersion);
}

/* 记录最近一次"已接受"帧（重复帧重 ACK 判定依据） */
static void bp_record_last(BootProto_Session_t *s, const BootProto_Frame_t *f)
{
    s->haveLast = 1u;
    s->lastType = f->type;
    s->lastSeq = f->sequence;
    s->lastOffset = f->imageOffset;
    s->lastLen = (u32)f->payloadLen;
}

void BootProto_SessionInput(BootProto_Session_t *s, const BootProto_Frame_t *f,
                             BootProto_Event_t *out)
{
    if (s == NULL || f == NULL || out == NULL)
        return;

    memset(out, 0, sizeof(*out));
    out->expectedSeq = s->expectedSeq;
    out->expectedOffset = s->expectedOffset;

    /* 1. 重复帧（最高优先级）：与最近一次"已接受"帧 type/seq/offset/len
     *    全一致（START 另要求 manifest 一致）-> 重 ACK，不重复编程 */
    if (s->haveLast &&
        f->type == s->lastType &&
        f->sequence == s->lastSeq &&
        f->imageOffset == s->lastOffset &&
        (u32)f->payloadLen == s->lastLen) {
        if (f->type != (u8)BOOTPROTO_TYPE_START || bp_manifest_equal(s, f)) {
            out->kind = (u8)BOOTPROTO_EVT_DUPLICATE;
            return;
        }
    }

    switch (f->type) {
    case (u8)BOOTPROTO_TYPE_START: {
        u32 base, size, crc, ver;

        if (s->state == BP_SSTATE_DONE) {
            bp_nack(out, (u8)BOOTPROTO_NACK_SESSION_DONE);
            return;
        }
        if (s->state == BP_SSTATE_ACTIVE) {
            /* 重复 START 已在上方拦截；到这里的必是不同 seq/manifest */
            bp_nack(out, (u8)BOOTPROTO_NACK_SESSION_BUSY);
            return;
        }

        /* IDLE：START 建立会话 */
        if (f->payload == NULL || f->payloadLen != (u16)BOOTPROTO_START_PAYLOAD_LEN) {
            bp_nack(out, (u8)BOOTPROTO_NACK_BAD_START_BASE);   /* manifest 不可用 */
            return;
        }
        bp_decode_manifest(f, &base, &size, &crc, &ver);

        if (f->sequence != s->expectedSeq) {   /* IDLE 下期望 0 */
            bp_nack(out, (u8)BOOTPROTO_NACK_BAD_SEQ);
            return;
        }
        if (base != s->appBase) {
            bp_nack(out, (u8)BOOTPROTO_NACK_BAD_START_BASE);
            return;
        }
        if (size < (u32)BOOTPROTO_MIN_IMAGE_SIZE || size > s->appCapacity) {
            bp_nack(out, (u8)BOOTPROTO_NACK_BAD_START_SIZE);
            return;
        }

        s->state = BP_SSTATE_ACTIVE;
        s->imageSize = size;
        s->imageCrc = crc;
        s->imageVersion = ver;
        s->expectedSeq = 1u;
        s->expectedOffset = 0u;
        bp_record_last(s, f);
        s->lastStartBase = base;
        s->lastStartSize = size;
        s->lastStartCrc = crc;
        s->lastStartVersion = ver;

        out->kind = (u8)BOOTPROTO_EVT_START_ACCEPTED;
        out->startBase = base;
        out->startSize = size;
        out->startCrc = crc;
        out->startVersion = ver;
        return;
    }

    case (u8)BOOTPROTO_TYPE_DATA:
        if (s->state == BP_SSTATE_IDLE) {
            bp_nack(out, (u8)BOOTPROTO_NACK_NOT_ACTIVE);
            return;
        }
        if (s->state == BP_SSTATE_DONE) {
            bp_nack(out, (u8)BOOTPROTO_NACK_SESSION_DONE);
            return;
        }
        if (f->sequence > s->expectedSeq) {            /* 跳号（超前） */
            bp_nack(out, (u8)BOOTPROTO_NACK_BAD_SEQ);
            return;
        }
        if (f->sequence < s->expectedSeq) {            /* 旧帧且非上一包 */
            bp_nack(out, (u8)BOOTPROTO_NACK_OUT_OF_ORDER);
            return;
        }
        if ((f->imageOffset & 1u) != 0u) {             /* 半字编程前提 */
            bp_nack(out, (u8)BOOTPROTO_NACK_MISALIGNED);
            return;
        }
        /* 先减后比（防溢出）：offset<=size 且 len<=size-offset。
         * 0xFFFFFF00+0x200 类输入无法通过（offset>size 直接拒绝，
         * 朴素写法 offset+len<=size 会回绕放行）。 */
        if (f->imageOffset > s->imageSize ||
            (u32)f->payloadLen > s->imageSize - f->imageOffset) {
            bp_nack(out, (u8)BOOTPROTO_NACK_OUT_OF_BOUNDS);
            return;
        }
        if (f->imageOffset != s->expectedOffset) {     /* 对齐但非期望值 */
            bp_nack(out, (u8)BOOTPROTO_NACK_BAD_OFFSET);
            return;
        }
        /* 奇数长度仅允许最终包（offset+len==imageSize） */
        if (((f->payloadLen & 1u) != 0u) &&
            f->imageOffset + (u32)f->payloadLen != s->imageSize) {
            bp_nack(out, (u8)BOOTPROTO_NACK_ODD_LENGTH);
            return;
        }

        out->kind = (u8)BOOTPROTO_EVT_DATA_ACCEPTED;
        out->payload = f->payload;
        out->dataOffset = f->imageOffset;
        out->dataLen = f->payloadLen;
        s->expectedSeq++;
        s->expectedOffset = f->imageOffset + (u32)f->payloadLen;
        bp_record_last(s, f);
        return;

    case (u8)BOOTPROTO_TYPE_END:
        if (s->state == BP_SSTATE_IDLE) {
            bp_nack(out, (u8)BOOTPROTO_NACK_NOT_ACTIVE);
            return;
        }
        if (s->state == BP_SSTATE_DONE) {
            bp_nack(out, (u8)BOOTPROTO_NACK_SESSION_DONE);
            return;
        }
        if (f->sequence > s->expectedSeq) {
            bp_nack(out, (u8)BOOTPROTO_NACK_BAD_SEQ);
            return;
        }
        if (f->sequence < s->expectedSeq) {
            bp_nack(out, (u8)BOOTPROTO_NACK_OUT_OF_ORDER);
            return;
        }
        if (s->expectedOffset != s->imageSize) {       /* 数据未到齐 */
            bp_nack(out, (u8)BOOTPROTO_NACK_INCOMPLETE);
            return;
        }

        s->state = BP_SSTATE_DONE;
        s->expectedSeq++;
        bp_record_last(s, f);
        out->kind = (u8)BOOTPROTO_EVT_END_ACCEPTED;
        return;

    default:
        /* 解析器已过滤非主机方向类型；直接 mock 喂未知类型时静默忽略 */
        return;
    }
}
