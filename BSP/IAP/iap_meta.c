/*
 * IAP 双页事务元数据实现
 * 详见 iap_meta.h
 *
 * 主机测试：mock/freertos/test_iap_meta.c + fake_flash.c
 *（断电注入覆盖每个 erase/半字编程位置）
 */
#include "iap_meta.h"
#include "crc32.h"

/*-----------------------------------------------------------
 * 小端显式编解码助手（不依赖主机/目标字节序）
 *----------------------------------------------------------*/
static void meta_put16(u8 *p, u16 v)
{
    p[0] = (u8)(v & 0xFFu);
    p[1] = (u8)((v >> 8) & 0xFFu);
}

static void meta_put32(u8 *p, u32 v)
{
    p[0] = (u8)(v & 0xFFu);
    p[1] = (u8)((v >> 8) & 0xFFu);
    p[2] = (u8)((v >> 16) & 0xFFu);
    p[3] = (u8)((v >> 24) & 0xFFu);
}

static u16 meta_get16(const u8 *p)
{
    return (u16)((u16)p[0] | ((u16)p[1] << 8));
}

static u32 meta_get32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

/*-----------------------------------------------------------
 * 编解码
 *----------------------------------------------------------*/
void IAP_MetaEncodeRecord(const IAP_MetaRecordFields_t *f,
                          u8 buf[IAP_META_RECORD_SIZE])
{
    if (f == NULL || buf == NULL) return;

    meta_put32(&buf[0],  IAP_META_MAGIC);
    meta_put16(&buf[4],  IAP_META_FORMAT_VERSION);
    meta_put16(&buf[6],  IAP_META_RECORD_SIZE);
    meta_put32(&buf[8],  f->generation);
    meta_put16(&buf[12], f->stateWord);
    meta_put16(&buf[14], f->slotId);
    meta_put32(&buf[16], f->appBase);
    meta_put32(&buf[20], f->imageSize);
    meta_put32(&buf[24], f->imageCrc);
    meta_put32(&buf[28], f->imageVersion);

    /* record_crc32 覆盖 [0,32)：字段置零后计算，再写回 */
    meta_put32(&buf[IAP_META_CRC_COVER_SIZE], 0u);
    meta_put32(&buf[IAP_META_CRC_COVER_SIZE],
               CRC32_Calculate(buf, IAP_META_CRC_COVER_SIZE));

    /* commit 值由写入器最后编程（IAP_MetaWriteRecord） */
    meta_put16(&buf[IAP_META_COMMIT_OFFSET], IAP_META_COMMIT_DONE);
}

void IAP_MetaDecodeRecord(const u8 buf[IAP_META_RECORD_SIZE],
                          IAP_MetaRecord_t *out)
{
    if (out == NULL) return;
    out->valid = 0u;
    if (buf == NULL) return;

    out->magic         = meta_get32(&buf[0]);
    out->formatVersion = meta_get16(&buf[4]);
    out->headerSize    = meta_get16(&buf[6]);
    out->generation    = meta_get32(&buf[8]);
    out->stateWord     = meta_get16(&buf[12]);
    out->slotId        = meta_get16(&buf[14]);
    out->appBase       = meta_get32(&buf[16]);
    out->imageSize     = meta_get32(&buf[20]);
    out->imageCrc      = meta_get32(&buf[24]);
    out->imageVersion  = meta_get32(&buf[28]);
    out->recordCrc     = meta_get32(&buf[32]);
    out->commitMarker  = meta_get16(&buf[IAP_META_COMMIT_OFFSET]);

    /* 任一校验失败均判无效：magic/格式/头长/commit/记录 CRC */
    if (out->magic != IAP_META_MAGIC)                          return;
    if (out->formatVersion != IAP_META_FORMAT_VERSION)         return;
    if (out->headerSize != (u16)IAP_META_RECORD_SIZE)          return;
    if (out->commitMarker != IAP_META_COMMIT_DONE)             return;
    /* CRC32_Calculate 只读不改数据，此处去 const 与生产接口签名对齐 */
    if (out->recordCrc != CRC32_Calculate((u8 *)buf, IAP_META_CRC_COVER_SIZE)) {
        return;
    }
    out->valid = 1u;
}

/*-----------------------------------------------------------
 * generation 环绕比较
 *----------------------------------------------------------*/
u8 IAP_GenerationIsNewer(u32 a, u32 b)
{
    if (a == b) return 0u;
    return ((u32)(a - b) < 0x80000000u) ? 1u : 0u;
}

u8 IAP_GenerationIsAmbiguous(u32 a, u32 b)
{
    if (a == b) return 0u;
    return ((u32)(a - b) == 0x80000000u) ? 1u : 0u;
}

/*-----------------------------------------------------------
 * 双页选择与启动决策
 *----------------------------------------------------------*/
IAP_MetaSelectResult_t IAP_MetaSelectLatest(const IAP_MetaRecord_t *recA,
                                            const IAP_MetaRecord_t *recB,
                                            const IAP_MetaRecord_t **selected)
{
    u8 aOk = (recA != NULL) && (recA->valid != 0u);
    u8 bOk = (recB != NULL) && (recB->valid != 0u);

    if (selected != NULL) {
        *selected = NULL;
    }

    if (aOk && bOk) {
        /* 正常序列两页 generation 永不相等；相等即异常，保守拒绝 */
        if (recA->generation == recB->generation) {
            return IAP_META_SELECT_AMBIGUOUS;
        }
        if (IAP_GenerationIsAmbiguous(recA->generation, recB->generation)) {
            return IAP_META_SELECT_AMBIGUOUS;
        }
        if (IAP_GenerationIsNewer(recA->generation, recB->generation)) {
            if (selected != NULL) *selected = recA;
            return IAP_META_SELECT_PAGE_A;
        }
        if (selected != NULL) *selected = recB;
        return IAP_META_SELECT_PAGE_B;
    }
    if (aOk) {
        if (selected != NULL) *selected = recA;
        return IAP_META_SELECT_PAGE_A;
    }
    if (bOk) {
        if (selected != NULL) *selected = recB;
        return IAP_META_SELECT_PAGE_B;
    }
    return IAP_META_SELECT_NONE;
}

IAP_BootAction_t IAP_BootDecision(const IAP_MetaRecord_t *recA,
                                  const IAP_MetaRecord_t *recB,
                                  u32 slotABase, u32 slotBBase,
                                  u32 slotCapacity)
{
    const IAP_MetaRecord_t *sel = NULL;
    u32 slotBase;

    if (IAP_MetaSelectLatest(recA, recB, &sel) == IAP_META_SELECT_AMBIGUOUS) {
        return IAP_BOOT_REJECT;
    }
    if (sel == NULL) {
        return IAP_BOOT_REJECT;            /* 无有效记录（含全新芯片） */
    }
    if (sel->stateWord != IAP_META_STATE_VALID) {
        return IAP_BOOT_REJECT;            /* RECEIVING/未知状态：停留 Boot */
    }
    slotBase = IAP_SlotBaseFromRecord(sel, slotABase, slotBBase);
    if (slotBase == 0u) {
        return IAP_BOOT_REJECT;            /* slot_id 非法 / app_base 与槽不符 */
    }
    if (sel->imageSize < 8u || sel->imageSize > slotCapacity) {
        return IAP_BOOT_REJECT;            /* 长度非法（镜像头最小 8 B） */
    }
    return IAP_BOOT_CONTINUE;
}

u32 IAP_SlotBaseFromRecord(const IAP_MetaRecord_t *rec,
                           u32 slotABase, u32 slotBBase)
{
    if (rec == NULL) {
        return 0u;
    }
    if (rec->slotId == IAP_META_SLOT_A && rec->appBase == slotABase) {
        return slotABase;
    }
    if (rec->slotId == IAP_META_SLOT_B && rec->appBase == slotBBase) {
        return slotBBase;
    }
    return 0u;
}

u32 IAP_SlotOther(u32 activeBase, u32 slotABase, u32 slotBBase)
{
    if (activeBase == slotABase) {
        return slotBBase;
    }
    if (activeBase == slotBBase) {
        return slotABase;
    }
    return 0u;
}

/*-----------------------------------------------------------
 * 活动槽解析（F-01）
 *----------------------------------------------------------*/

/* 单条记录是否可作为"活动槽"候选：解码有效 + VALID + 槽自洽 + 长度合法 */
static u8 meta_is_active_candidate(const IAP_MetaRecord_t *rec,
                                   u32 slotABase, u32 slotBBase,
                                   u32 slotCapacity)
{
    if (rec == NULL || rec->valid == 0u) {
        return 0u;
    }
    if (rec->stateWord != IAP_META_STATE_VALID) {
        return 0u;                       /* RECEIVING 不是活动槽 */
    }
    if (IAP_SlotBaseFromRecord(rec, slotABase, slotBBase) == 0u) {
        return 0u;                       /* slot_id / app_base 不自洽 */
    }
    if (rec->imageSize < 8u || rec->imageSize > slotCapacity) {
        return 0u;
    }
    return 1u;
}

IAP_ActiveSlot_t IAP_ActiveSlotOf(const IAP_MetaRecord_t *recA,
                                  const IAP_MetaRecord_t *recB,
                                  u32 slotABase, u32 slotBBase,
                                  u32 slotCapacity)
{
    u8 aOk = meta_is_active_candidate(recA, slotABase, slotBBase, slotCapacity);
    u8 bOk = meta_is_active_candidate(recB, slotABase, slotBBase, slotCapacity);
    const IAP_MetaRecord_t *win;

    if (!aOk && !bOk) {
        return IAP_ACTIVE_NONE;
    }
    if (aOk && !bOk) {
        win = recA;
    } else if (!aOk && bOk) {
        win = recB;
    } else {
        /* 两页都有合法 VALID：按 generation 取更新者 */
        if (recA->generation == recB->generation ||
            IAP_GenerationIsAmbiguous(recA->generation, recB->generation)) {
            return IAP_ACTIVE_AMBIGUOUS;     /* 保守拒绝，不擦任何槽 */
        }
        win = IAP_GenerationIsNewer(recA->generation, recB->generation)
                  ? recA : recB;
    }

    /* 落在哪个槽由【记录的 slot_id】决定，而不是它恰好在哪一页：
     * 两页是事务日志，页位置与槽无固定对应关系（每次升级写非最新页）。 */
    if (win->slotId == IAP_META_SLOT_A) return IAP_ACTIVE_SLOT_A;
    if (win->slotId == IAP_META_SLOT_B) return IAP_ACTIVE_SLOT_B;
    return IAP_ACTIVE_NONE;   /* 理论不可达（候选检查已过滤） */
}

u32 IAP_ActiveSlotBase(IAP_ActiveSlot_t act, u32 slotABase, u32 slotBBase)
{
    if (act == IAP_ACTIVE_SLOT_A) return slotABase;
    if (act == IAP_ACTIVE_SLOT_B) return slotBBase;
    return 0u;
}

/*-----------------------------------------------------------
 * 记录写入事务 / 读回
 *----------------------------------------------------------*/
int IAP_MetaWriteRecord(const IAP_MetaIo_t *io, u32 pageAddr,
                        const IAP_MetaRecordFields_t *f)
{
    u8 buf[IAP_META_RECORD_SIZE];
    u32 off;
    u16 v;

    if (io == NULL || io->erase_page == NULL ||
        io->program_halfword == NULL || f == NULL) {
        return -1;
    }

    IAP_MetaEncodeRecord(f, buf);

    if (io->erase_page(io->self, pageAddr) != 0) {
        return -2;
    }

    /* 先编程除 commit 外的全部半字（偏移 0..32） */
    for (off = 0u; off < IAP_META_COMMIT_OFFSET; off += 2u) {
        v = (u16)((u16)buf[off] | ((u16)buf[off + 1u] << 8));
        if (io->program_halfword(io->self, pageAddr + off, v) != 0) {
            return -3;
        }
    }

    /* commit 半字最后编程：此前任何断电都使记录解码无效 */
    v = (u16)((u16)buf[IAP_META_COMMIT_OFFSET] |
              ((u16)buf[IAP_META_COMMIT_OFFSET + 1u] << 8));
    if (io->program_halfword(io->self, pageAddr + IAP_META_COMMIT_OFFSET, v) != 0) {
        return -4;
    }

    return 0;
}

int IAP_MetaReadRecord(const IAP_MetaIo_t *io, u32 pageAddr,
                       u8 buf[IAP_META_RECORD_SIZE])
{
    u32 off;
    u16 v;

    if (io == NULL || io->read_halfword == NULL || buf == NULL) {
        return -1;
    }

    for (off = 0u; off < IAP_META_RECORD_SIZE; off += 2u) {
        v = io->read_halfword(io->self, pageAddr + off);
        buf[off]      = (u8)(v & 0xFFu);
        buf[off + 1u] = (u8)((v >> 8) & 0xFFu);
    }
    return 0;
}
