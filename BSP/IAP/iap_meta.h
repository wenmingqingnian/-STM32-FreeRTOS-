/*
 * IAP 双页事务元数据
 *
 * 职责：
 *   1. 定义双页元数据记录（page 126/127 页首各一条）的显式小端字节布局
 *      与编解码，禁止依赖结构体 padding；
 *   2. generation 无符号环绕比较与双页"最新有效记录"选择；
 *   3. 上电启动决策（仅最新 VALID 且字段合法才允许进入向量/CRC 校验链）；
 *   4. 记录写入事务：擦页 -> 逐半字编程 -> commit 半字最后编程。
 *
 * 断电原子性（原理要点）：
 *   - commit_marker 是整条记录最后一次半字编程。任何断电只要发生在
 *     commit 编程之前，该记录解码即为无效（magic/版本/CRC/marker 任一
 *     不满足），Flash 里最多留下一条"未提交"的半成品记录；
 *   - 因此"记录可解码"蕴含"commit 已编程"，而 commit 之前所有字段
 *     已全部落盘——这就是本设计的提交点（commit point）；
 *   - 与旧单页设计的区别：旧方案靠"VALID 状态字只在回读 CRC 通过后
 *     写入"保证原子性，但升级开始即作废旧 APP，断电后无路可退；
 *     双页设计下旧 VALID 记录保持到新 RECEIVING 提交都完好，断电在
 *     开始记录提交前仍可启动旧 APP。
 *
 * 迁移说明：旧 0x1111/0x2222 单页擦写状态设计（IAP_Meta_t、
 * IAP_STATE_WORD_xxx、IAP_StateTransitionValid、IAP_CommitDecision）随本
 * Task 退役，不做向后兼容；旧 iap.c/iap.h 一并删除。
 *
 * 本文件只含纯逻辑 + 经注入的 Flash IO 接口，不访问任何寄存器，
 * 便于主机 fake_flash 断电注入测试。地址常量一律由调用方从
 * CONFIG/memory_layout.h 传入（META_A_BASE/META_B_BASE/APP_BASE/...），
 * 本模块不硬编码任何分区地址。
 */
#ifndef __IAP_META_H__
#define __IAP_META_H__

#include <stddef.h>   /* NULL — portable across ARMCC and host gcc */
#include "sys.h"

/*-----------------------------------------------------------
 * 记录布局（显式小端，偏移/长度固定，禁止结构体 memcpy）
 *
 *  off  size  field
 *   0    4    magic            固定 IAP_META_MAGIC（"IPA2"）
 *   4    2    format_version   固定 IAP_META_FORMAT_VERSION
 *   6    2    header_size      固定 IAP_META_RECORD_SIZE
 *   8    4    generation       无符号环绕递增
 *  12    2    state            RECEIVING / VALID
 *  14    2    slot_id          本记录描述的槽（SLOT_ID_A / SLOT_ID_B）
 *  16    4    app_base         必须等于该槽基址（SLOT_A_BASE / SLOT_B_BASE）
 *  20    4    image_size       8..SLOT_CAPACITY
 *  24    4    image_crc32      整镜像 CRC32
 *  28    4    image_version    仅记录，不作安全防回滚
 *  32    4    record_crc32     覆盖 [0,32) 的 CRC32（本字段置零后计算）
 *  36    2    commit_marker    整条记录最后一次半字编程
 *
 * 版本升级说明：旧 36 B 记录（含 app_base off14、
 * record_crc32 off30、commit off34）随单分区设计退役，format_version
 * 由 0x0001 升至 0x0002；旧记录因 format_version 不匹配一律判无效
 *（不做向后兼容——单分区镜像不可直接用于双槽布局，必须重烧）。
 *----------------------------------------------------------*/
#define IAP_META_RECORD_SIZE     38u
#define IAP_META_CRC_COVER_SIZE  32u   /* record_crc32 覆盖的前缀字节数 */
#define IAP_META_COMMIT_OFFSET   36u   /* commit 半字在记录内的偏移 */

#define IAP_META_MAGIC           0x32415049uL   /* "IPA2" 小端（版本升级标识） */
#define IAP_META_FORMAT_VERSION  0x0002u

/* 状态字（u16）：与擦除态 0xFFFF 区分，两状态互不位包含 */
#define IAP_META_STATE_ERASED     0xFFFFu
#define IAP_META_STATE_RECEIVING  0x5AA5u   /* 升级写入中（已提交"开始"记录） */
#define IAP_META_STATE_VALID      0x3CC3u   /* 镜像完整，允许启动校验链 */

/* 槽编号（u16）：0 保留为"非法/擦除态"；与 memory_layout.h 的
 * SLOT_ID_A/SLOT_ID_B 同值——此处的定义属于【元数据格式契约】，
 * 不依赖任何分区地址，故放在本头文件而非布局头。 */
#define IAP_META_SLOT_NONE        0u
#define IAP_META_SLOT_A           1u
#define IAP_META_SLOT_B           2u

/* commit marker：记录生效标志，写入器必须最后编程该半字 */
#define IAP_META_COMMIT_NONE     0xFFFFu   /* 擦除态：未提交 */
#define IAP_META_COMMIT_DONE     0x33CCu

/*-----------------------------------------------------------
 * Flash 后端接口（依赖注入）：
 *   host 测试侧接 fake_flash.c；目标侧适配 FLASH_ErasePage()/
 *   FLASH_ProgramHalfWord()/直接读半字。返回 0 成功，非 0 失败。
 *----------------------------------------------------------*/
typedef struct {
    void *self;
    int  (*erase_page)(void *self, u32 page_addr);
    int  (*program_halfword)(void *self, u32 addr, u16 value);
    u16  (*read_halfword)(void *self, u32 addr);
} IAP_MetaIo_t;

/* 上电启动决策 */
typedef enum {
    IAP_BOOT_REJECT = 0,    /* 停留 Boot：无有效记录/RECEIVING/歧义/字段非法 */
    IAP_BOOT_CONTINUE        /* 最新记录为 VALID 且字段合法，进入向量/CRC 链 */
} IAP_BootAction_t;

/* 双页选择结果 */
typedef enum {
    IAP_META_SELECT_NONE = 0,     /* 两页均无有效记录 */
    IAP_META_SELECT_PAGE_A,        /* A 页（META_A_BASE）最新 */
    IAP_META_SELECT_PAGE_B,        /* B 页（META_B_BASE）最新 */
    IAP_META_SELECT_AMBIGUOUS     /* 两页均有效但 generation 无法排序 */
} IAP_MetaSelectResult_t;

/* 待编码字段（magic/版本/头长/记录 CRC/commit 由编码器生成） */
typedef struct {
    u32 generation;
    u16 stateWord;     /* IAP_META_STATE_RECEIVING / IAP_META_STATE_VALID */
    u16 slotId;        /* SLOT_ID_A / SLOT_ID_B（0 非法） */
    u32 appBase;       /* 与 slotId 匹配的槽基址 */
    u32 imageSize;
    u32 imageCrc;
    u32 imageVersion;
} IAP_MetaRecordFields_t;

/* 解码视图（valid=1 表示通过全部字段校验，可参与选择/决策） */
typedef struct {
    u32 magic;
    u16 formatVersion;
    u16 headerSize;
    u32 generation;
    u16 stateWord;
    u16 slotId;
    u32 appBase;
    u32 imageSize;
    u32 imageCrc;
    u32 imageVersion;
    u32 recordCrc;
    u16 commitMarker;
    u8  valid;
} IAP_MetaRecord_t;

/*-----------------------------------------------------------
 * 编解码（纯逻辑，主机可测）
 *----------------------------------------------------------*/
void IAP_MetaEncodeRecord(const IAP_MetaRecordFields_t *f,
                          u8 buf[IAP_META_RECORD_SIZE]);

/* 解码并校验：magic/format/header_size/commit marker/record CRC 任一
 * 错误均置 valid=0。out 为 NULL 直接返回；buf 为 NULL 置 valid=0。 */
void IAP_MetaDecodeRecord(const u8 buf[IAP_META_RECORD_SIZE],
                          IAP_MetaRecord_t *out);

/*-----------------------------------------------------------
 * generation 无符号环绕比较
 *----------------------------------------------------------*/

/* a 比 b 新（环绕安全）：a!=b 且 (a-b) 的无符号差小于 0x80000000 */
u8 IAP_GenerationIsNewer(u32 a, u32 b);

/* a 与 b 相差恰为 0x80000000：无法排序，必须保守拒绝 */
u8 IAP_GenerationIsAmbiguous(u32 a, u32 b);

/*-----------------------------------------------------------
 * 双页选择与启动决策（纯逻辑，主机可测）
 *----------------------------------------------------------*/

/* 返回最新有效记录。recA/recB 可为 NULL（按无效页处理）；selected
 * 可为 NULL（不需要回填）。两页均有效但 generation 相等或相差
 * 0x80000000 时返回 AMBIGUOUS（selected 置 NULL，保守拒绝）。 */
IAP_MetaSelectResult_t IAP_MetaSelectLatest(const IAP_MetaRecord_t *recA,
                                            const IAP_MetaRecord_t *recB,
                                            const IAP_MetaRecord_t **selected);

/* 上电决策（双槽版）：最新有效记录为 VALID、slot_id 合法且 app_base
 * 与该槽规定基址一致、8<=image_size<=slotCapacity 才 CONTINUE；
 * 其余（无记录/RECEIVING/歧义/字段非法）一律 REJECT 停留 Boot。
 * recA/recB 可为 NULL。
 *
 * slotABase/slotBBase 由调用方从 memory_layout.h 注入（本模块不硬编码）。 */
IAP_BootAction_t IAP_BootDecision(const IAP_MetaRecord_t *recA,
                                  const IAP_MetaRecord_t *recB,
                                  u32 slotABase, u32 slotBBase,
                                  u32 slotCapacity);

/*-----------------------------------------------------------
 * 槽选择（双槽启动决策的纯逻辑）
 *----------------------------------------------------------*/

/* 由 slot_id + app_base 反查槽基址：合法返回 SLOT_A_BASE/SLOT_B_BASE，
 * 非法（slot_id 未知 或 app_base 与规定基址不符）返回 0。 */
u32 IAP_SlotBaseFromRecord(const IAP_MetaRecord_t *rec,
                           u32 slotABase, u32 slotBBase);

/* 选择备用槽（用于升级写入目标）：给定当前活动槽基址，返回另一槽基址。
 * 活动槽必须等于 slotABase 或 slotBBase，否则返回 0（参数非法）。 */
u32 IAP_SlotOther(u32 activeBase, u32 slotABase, u32 slotBBase);

/*-----------------------------------------------------------
 * 活动槽解析（F-01 修复下沉点：生产代码与主机测试共用同一实现）
 *----------------------------------------------------------*/

/* 活动槽解析结果。显式区分多种"取不到活动槽"的原因（F-06），
 * 禁止再用单一返回值 0 同时表达"全新芯片"与"元数据异常"。 */
typedef enum {
    IAP_ACTIVE_NONE = 0,   /* 两页均无 VALID 记录：全新芯片 / 元数据全非法，
                             *   此时升级目标取 Slot A（首次烧录语义） */
    IAP_ACTIVE_SLOT_A,     /* 活动槽 = Slot A */
    IAP_ACTIVE_SLOT_B,     /* 活动槽 = Slot B */
    IAP_ACTIVE_AMBIGUOUS   /* 两页都是合法 VALID 但generation 无法排序：
                             *   保守拒绝升级（不擦任何槽） */
} IAP_ActiveSlot_t;

/* 从两页元数据中解析【活动槽】：在所有"解码有效 + state==VALID +
 * slot_id/app_base 自洽 + 8<=image_size<=slotCapacity"的记录中，取
 * generation 最新者所属的槽。
 *
 * 关键语义（F-01）：只要求"存在 VALID 记录"，而【不是】"最新记录就是
 * VALID"。升级途中断电时最新记录是 RECEIVING（generation 更大），但旧
 * VALID 记录仍完好——它就是活动槽，也是回滚能力所在。若此时把活动槽
 * 误判为"无"，目标槽会落到唯一的好镜像上并被擦除，回滚能力被摧毁。
 *
 * recA/recB 可为 NULL（按无效页处理）。
 * 返回 IAP_ACTIVE_AMBIGUOUS 时调用方【不得】擦除任何槽。 */
IAP_ActiveSlot_t IAP_ActiveSlotOf(const IAP_MetaRecord_t *recA,
                                  const IAP_MetaRecord_t *recB,
                                  u32 slotABase, u32 slotBBase,
                                  u32 slotCapacity);

/* 活动槽解析结果 -> 槽基址（SLOT_A_BASE/SLOT_B_BASE）；NONE/AMBIGUOUS
 * 均返回 0。 */
u32 IAP_ActiveSlotBase(IAP_ActiveSlot_t act, u32 slotABase, u32 slotBBase);

/*-----------------------------------------------------------
 * 记录写入事务 / 读回（经注入的 Flash IO）
 *----------------------------------------------------------*/

/* 写一条记录到 pageAddr 页首：擦页 -> 编程除 commit 外的 17 个半字 ->
 * 最后编程 commit 半字。返回 0 成功；-1 参数非法；-2 擦除失败；
 * -3 字段半字编程失败；-4 commit 半字编程失败。
 * 任一失败点断电，页上记录因缺 commit 解码无效（安全方向）。 */
int IAP_MetaWriteRecord(const IAP_MetaIo_t *io, u32 pageAddr,
                        const IAP_MetaRecordFields_t *f);

/* 从 pageAddr 页首按半字读回整条记录到 buf（供 Decode）。
 * 返回 0 成功；-1 参数非法。 */
int IAP_MetaReadRecord(const IAP_MetaIo_t *io, u32 pageAddr,
                       u8 buf[IAP_META_RECORD_SIZE]);

#endif /* __IAP_META_H__ */
