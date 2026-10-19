/*
 * IAP 帧协议解析器与升级会话状态机
 *
 * 职责：
 *   1. 字节流 -> 帧的解析器：SOF 同步/重同步（噪声跳过、任意分片喂入、
 *      帧可跨环形缓冲尾部）、24 B 帧头逐字节小端解码、帧 CRC32 校验；
 *   2. 升级会话状态机：START manifest 校验、DATA 顺序/对齐/越界
 *（先减后比防溢出）、END 完整性判定、上一已收帧重复重 ACK；
 *   3. 只输出事件，不直接操作任何寄存器，也不访问 Flash——
 *      DATA_ACCEPTED 事件携带负载指针，由调用方（Task 7/8 的
 *      boot_flash / boot_update）完成编程与回读后再 ACK。
 *
 * 帧格式（设计文档 §5.2，固定小端，停止等待 stop-and-wait）：
 *
 *   off  size  field
 *    0    4    SOF = "IAP1"
 *    4    1    protocol_version = 1
 *    5    1    type = START/DATA/END（ACK/NACK 仅 Boot->主机方向）
 *    6    2    header_len = 24
 *    8    4    sequence（会话内逐帧 +1，START 必须为 0）
 *   12    4    image_offset
 *   16    2    payload_len（DATA 1..512，START 恰 16，END 恰 0）
 *   18    2    reserved，必须为 0
 *   20    4    frame_crc32：IEEE CRC32（反射多项式 0xEDB88320，复用
 *              BSP/CRC32 流式接口），覆盖 [4,20) + payload，
 *              不覆盖 SOF 与 CRC 字段本身
 *
 * START payload 16 B manifest（逐字段小端解码，禁止结构体强转）：
 *   off 0  target_base   必须严格等于会话注入的 appBase
 *（双槽改造：= bu_start 选定的目标槽基址）
 *   off 4  image_size    8 .. appCapacity（= SLOT_CAPACITY）
 *   off 8  image_crc32   仅记录/透传（整镜像校验在 END 后由调用方做）
 *   off 12 image_version 仅记录，不作安全防回滚
 *
 * DATA 规则：
 *   - 只接受 offset == next_expected_offset（半字编程前提）；
 *   - offset 必须 2 字节对齐；除镜像最后一包外 payload_len 必须为偶数，
 *     只有最终包允许奇数长度；
 *   - 长度检查一律"先减后比"（len <= image_size - offset），禁止可被
 *     0xFFFFFF00 + 0x200 类输入绕过的 offset+len <= size 写法；
 *   - 上一帧（与最近已接受帧 type/seq/offset/len 及 START manifest 均
 *     相同）重复重发：重新 ACK，事件不再触发编程；
 *   - 跳号、越界、乱序、未知类型均 NACK，并回填期望 sequence/offset。
 *
 * 解析器契约（对应 1024 B DMA 环形缓冲 + USART IDLE）：
 *   - Feed 可按任意大小分片调用（1 字节、半截 header、payload 跨环尾
 *     的两段连续区），解码结果与整帧一次喂入完全一致；
 *   - SOF 前的噪声字节被静默丢弃，滑窗保证噪声尾部的半截 SOF 不误同步；
 *   - 超时（调用方判定）后 BootProto_ParserReset() 清半帧回到同步扫描，
 *     会话状态不受影响，主机可整帧重发。
 */
#ifndef __BOOT_PROTOCOL_H__
#define __BOOT_PROTOCOL_H__

#include <stddef.h>   /* NULL — ARMCC 与 host gcc 通用 */
#include "sys.h"
#include "crc32.h"    /* CRC32_StreamCtx_t（Task 4 流式接口） */

/*-----------------------------------------------------------
 * 协议常量（设计文档 §5.2）
 *----------------------------------------------------------*/
#define BOOTPROTO_HEADER_LEN        24u   /* 帧头固定长度 */
#define BOOTPROTO_MAX_PAYLOAD       512u  /* DATA 负载上限 */

#define BOOTPROTO_VERSION           1u    /* protocol_version 合法值 */

/* 帧类型：低 3 种是主机->Boot 方向；ACK/NACK 仅 Boot->主机 */
#define BOOTPROTO_TYPE_START        0x01u
#define BOOTPROTO_TYPE_DATA         0x02u
#define BOOTPROTO_TYPE_END          0x03u
#define BOOTPROTO_TYPE_ACK          0x04u
#define BOOTPROTO_TYPE_NACK         0x05u

#define BOOTPROTO_START_PAYLOAD_LEN 16u   /* START manifest 固定长度 */
#define BOOTPROTO_MIN_IMAGE_SIZE   8u    /* 至少容纳 MSP+PC（设计 §5.5） */

/*-----------------------------------------------------------
 * 解码后的帧（解析器 -> 会话/调用方）
 *----------------------------------------------------------*/
typedef struct {
    u8         version;      /* 已校验 == 1 */
    u8         type;        /* 已校验为主机方向三种之一 */
    u16        headerLen;    /* 已校验 == 24 */
    u32        sequence;
    u32        imageOffset;
    u16        payloadLen;  /* 已按类型校验 */
    u16        reserved;     /* 已校验 == 0 */
    u32        frameCrc;     /* 帧内收到的 CRC 字段值 */
    const u8  *payload;      /* 指向解析器内部缓冲，下次 Feed 前有效 */
} BootProto_Frame_t;

/*-----------------------------------------------------------
 * 帧级错误（解析器拒绝；调用方据此 NACK 并让主机重传）
 *----------------------------------------------------------*/
typedef enum {
    BOOTPROTO_FRAME_ERR_NONE = 0,
    BOOTPROTO_FRAME_ERR_VERSION,      /* protocol_version != 1 */
    BOOTPROTO_FRAME_ERR_HEADER_LEN,   /* header_len != 24 */
    BOOTPROTO_FRAME_ERR_RESERVED,     /* reserved != 0 */
    BOOTPROTO_FRAME_ERR_TYPE,         /* type 非主机方向三种（含主机发 ACK/NACK） */
    BOOTPROTO_FRAME_ERR_PAYLOAD_LEN,  /* payload_len 与该类型允许范围不符 */
    BOOTPROTO_FRAME_ERR_CRC           /* 帧 CRC32 不匹配 */
} BootProto_FrameErr_t;

/*-----------------------------------------------------------
 * 会话事件（会话状态机输出；调用方按 kind 决定 ACK/编程/提交）
 *----------------------------------------------------------*/
typedef enum {
    BOOTPROTO_EVT_NONE = 0,
    BOOTPROTO_EVT_START_ACCEPTED,   /* 会话建立，manifest 见 start* 字段 */
    BOOTPROTO_EVT_DATA_ACCEPTED,   /* 按序 DATA：编程 payloadLen 字节到 dataOffset */
    BOOTPROTO_EVT_DUPLICATE,        /* 最近已接受帧重发：重新 ACK，不得再编程 */
    BOOTPROTO_EVT_END_ACCEPTED,     /* 数据到齐：可做整镜像 CRC 校验并提交 VALID */
    BOOTPROTO_EVT_NACK              /* 拒绝：nackReason + 期望 seq/offset 回填 */
} BootProto_EventKind_t;

typedef enum {
    BOOTPROTO_NACK_NONE = 0,
    BOOTPROTO_NACK_NOT_ACTIVE,      /* START 之前收到 DATA/END */
    BOOTPROTO_NACK_BAD_SEQ,        /* sequence 超前跳号 */
    BOOTPROTO_NACK_OUT_OF_ORDER,    /* sequence 落后（重发旧帧且非上一包） */
    BOOTPROTO_NACK_BAD_OFFSET,     /* offset 对齐但非期望值 */
    BOOTPROTO_NACK_MISALIGNED,     /* offset 非 2 字节对齐 */
    BOOTPROTO_NACK_OUT_OF_BOUNDS,   /* 越界（先减后比：len > image_size - offset） */
    BOOTPROTO_NACK_ODD_LENGTH,     /* 非最终包 payload_len 为奇数 */
    BOOTPROTO_NACK_BAD_START_BASE, /* START target_base != 会话注入 appBase */
    BOOTPROTO_NACK_BAD_START_SIZE, /* START image_size < 8 或 > appCapacity */
    BOOTPROTO_NACK_SESSION_BUSY,   /* 会话进行中收到不同 manifest 的 START */
    BOOTPROTO_NACK_INCOMPLETE,     /* END 早于全部数据到齐 */
    BOOTPROTO_NACK_SESSION_DONE    /* 会话完成后仍收到新帧 */
} BootProto_NackReason_t;

typedef struct {
    u8   kind;            /* BootProto_EventKind_t */
    u8   nackReason;     /* kind == NACK 时有效 */
    u16  pad16;
    u32  expectedSeq;    /* NACK/DUPLICATE 时回填给主机的期望 sequence */
    u32  expectedOffset; /* NACK/DUPLICATE 时回填给主机的期望 offset */
    /* kind == DATA_ACCEPTED：负载指针在下次 ParserFeed 前有效 */
    const u8 *payload;
    u32  dataOffset;
    u16  dataLen;
    /* kind == START_ACCEPTED：解码后的 manifest */
    u32  startBase;
    u32  startSize;
    u32  startCrc;
    u32  startVersion;
} BootProto_Event_t;

/*-----------------------------------------------------------
 * 解析器（字节流 -> 帧；纯逻辑，无寄存器/Flash 访问）
 *----------------------------------------------------------*/
typedef struct {
    /* Init 注入的回调（可为 NULL） */
    void (*onFrame)(void *user, const BootProto_Frame_t *frame);
    void (*onError)(void *user, BootProto_FrameErr_t err);
    void *user;
    /* 内部状态（勿直接改动） */
    u8   state;          /* 0=同步扫描 1=收头 2=收负载 */
    u32  syncWindow;     /* 最近 4 字节滑窗，SOF 检测 */
    u16  hdrFilled;
    u16  payloadNeed;    /* 帧头声明的 payload_len（已按类型校验） */
    u16  payloadFilled;
    u8   hdr[BOOTPROTO_HEADER_LEN];
    u8   payload[BOOTPROTO_MAX_PAYLOAD];
    CRC32_StreamCtx_t crc;   /* frame_crc32 流式计算 */
} BootProto_Parser_t;

void BootProto_ParserInit(BootProto_Parser_t *p,
                           void (*onFrame)(void *user, const BootProto_Frame_t *frame),
                           void (*onError)(void *user, BootProto_FrameErr_t err),
                           void *user);

/* 任意分片喂入；一个 Feed 内可完成多帧（连续帧） */
void BootProto_ParserFeed(BootProto_Parser_t *p, const u8 *data, u32 len);

/* 超时清状态：丢掉半截帧回到同步扫描；回调与配置保留，
 * 会话状态机不受影响，主机可整帧重发。 */
void BootProto_ParserReset(BootProto_Parser_t *p);

/*-----------------------------------------------------------
 * 升级会话状态机（帧 -> 事件；纯逻辑，独立于解析器可 mock）
 *
 * appBase/appCapacity 由调用方注入（目标侧取 CONFIG/memory_layout.h；
 * 双槽改造后 appBase 由 bu_start 按"活动槽的另一槽"在运行期选定，
 * appCapacity = SLOT_CAPACITY；本模块不硬编码任何分区地址）。
 *----------------------------------------------------------*/
typedef struct {
    u8   state;          /* 0=IDLE 1=ACTIVE 2=DONE（内部） */
    u8   haveLast;
    u8   lastType;
    u16  pad16;
    u32  appBase;        /* Init 注入：START target_base 必须严格等于它
（双槽：运行期目标槽基址） */
    u32  appCapacity;    /* Init 注入：image_size 上界（SLOT_CAPACITY） */
    u32  expectedSeq;    /* 下一帧期望 sequence */
    u32  expectedOffset; /* 下一 DATA 期望 image_offset */
    u32  imageSize;
    u32  imageCrc;
    u32  imageVersion;
    /* 最近一次"已接受"帧标识（重复帧重 ACK 判定） */
    u32  lastSeq;
    u32  lastOffset;
    u32  lastLen;
    u32  lastStartBase;
    u32  lastStartSize;
    u32  lastStartCrc;
    u32  lastStartVersion;
} BootProto_Session_t;

void BootProto_SessionInit(BootProto_Session_t *s, u32 appBase, u32 appCapacity);

/* 会话重置（保留注入的 appBase/appCapacity）；目标侧在重启升级链路时使用 */
void BootProto_SessionReset(BootProto_Session_t *s);

/* 输入一帧（通常来自解析器回调；也可直接构造帧结构体做 mock 测试），
 * 输出唯一事件。f/out 为 NULL 时不做任何事。 */
void BootProto_SessionInput(BootProto_Session_t *s, const BootProto_Frame_t *f,
                           BootProto_Event_t *out);

#endif /* __BOOT_PROTOCOL_H__ */
