/*
 * Boot 升级引擎实现
 *
 * 接线关系（各模块契约见各自头文件）：
 *   boot_uart_dma.c  --DMA 环消费-->  boot_protocol.c（解析器+会话）
 *   会话事件 --Flash 写入--> boot_flash.c（有界半字写入器）
 *                    |----> iap_meta.c（双页事务元数据，经本文件的目标侧
 *                           IO 后端 + 显式解锁/上锁事务包裹）
 *   ACK/NACK 经 boot_uart_dma.c 轮询发送（等 TXE/TC，不用 printf/DMA TX）
 *
 * 会话级失败策略（原理要点）：
 *   任何 Flash 层失败（元数据提交/擦除/编程/回读/整镜像 CRC）都不得写
 *   VALID。失败点 NACK（reason>=0x40）后复位会话与解析器——会话状态机
 *   已把失败帧视为"已接受"并推进了期望 offset，同帧重发只会被判为重复
 *   而重 ACK，永远补不上丢失的数据；唯一安全恢复路径是主机从 START
 *   重启整个升级（新 generation RECEIVING 落到另一页，旧页保持可回滚）。
 *   RECEIVING 一旦提交，旧 APP 不再可信，握手超时也不回退（设计 §5.1）。
 *
 * IWDG 纪律：只写 0xAAAA（刷新计数），绝不写 0xCCCC（启动）。未运行的
 * 软件看门狗对刷新写无响应；由 option bytes 启用的硬件看门狗则被安全喂狗。
 * 擦页后端逐页喂狗，主循环每轮喂狗。
 *
 * 接收环轮询纪律：擦页后端【逐页】调 Boot_UartDma_Poll()（与逐页喂狗同一
 * 位置）。这是"擦写期间对 Flash 的读访问会让总线停住、故 ISR 进不来"
 * 的直接后果——擦除窗口内唯一可靠的进度采样点就是页与页的间隙。判据与
 * 推导见 tio_erase_page 注释。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#include <string.h>
#include "boot_update.h"
#include "boot_protocol.h"
#include "boot_flash.h"
#include "boot_uart_dma.h"

/* stm32f10x.h（经 sys.h 链进入）与 memory_layout.h 都定义 FLASH_BASE 且
 * 同值不同形：先取消 SPL 版本，让布局头成为唯一权威定义，避免 ARMCC
 * 宏重定义告警。两者恒等 0x08000000 由 tools/check_memory_layout.py 强制。 */
#undef FLASH_BASE
#include "memory_layout.h"
#include "stm32f10x_flash.h"

/*-----------------------------------------------------------
 * 工厂 Flash 容量寄存器（中容量 F1：16 bit，单位 KiB）
 *----------------------------------------------------------*/
#define BU_FLASH_SIZE_REG_ADDR   0x1FFFF7E0uL

/* 引擎内部状态 */
#define BU_STATE_IDLE            0u   /* 等待 START（握手窗口活跃） */
#define BU_STATE_RECEIVING       1u   /* START 已接受，等 DATA/END */

/*-----------------------------------------------------------
 * 目标侧 Flash IO 后端（BootFlash_Io_t 与 IAP_MetaIo_t 共用）
 *----------------------------------------------------------*/
static void tio_unlock(void *self)
{
    (void)self;
    FLASH_Unlock();
}

static void tio_lock(void *self)
{
    (void)self;
    FLASH_Lock();
}

static int tio_erase_page(void *self, u32 page_addr)
{
    (void)self;
    /* 页擦除约 20-40 ms：硬件 IWDG 场景下逐页安全喂狗（只刷新不起动） */
    IWDG->KR = 0xAAAAu;

    /* 擦页间隙轮询接收环：本函数由 BootFlash_ErasePages/BootFlash_EraseMetaPage
     * 逐页调用，此刻上一页已擦完、Flash 空闲——这里插 Poll 不会因取指停顿失效。
     *
     * 为什么必须挂在这里：擦写期间对 Flash 的读访问会让总线停住，而本文件与
     * ISR 的代码都在 Flash 里，故整个擦除窗口内没有任何中断能进来（含
     * DMA1_Channel5 的 TC）。若不逐页轮询，两个 Poll 之间会横跨【整次擦除】
     * 的时间——页数 = ceil(image_size/1024)（boot_update.c 的 bu_start），
     * 整槽最坏 120 页 × 20-40 ms ≈ 2.4-4.8 s，55 KB 镜像典型 54 页 ≈ 1.08 s；
     * 无论哪种都远超环形缓冲单圈可分辨上限（40 ms 页窗口 ≈ 460 B）。
     * 逐页轮询把窗口压回"一页 = 40 ms ≈ 460 B < 1024 B"，
     * Boot_UartDma_Poll() 的位置回跳检测才成立。
     *
     * 判据（后续新增阻塞点照此判断，勿滥用）：只有当某段阻塞可能超过
     * min(环预算 88.9 ms@115200, 单圈 1024 B) 时才需要挂钩子。
     * 半字编程每帧 512 B ≈ 13-18 ms，且主机 stop-and-wait 正在等 ACK，
     * 故 tio_program_halfword 不挂。
     *
     * ⚠️ 改波特率必须同步放大环形缓冲：窗口字节数 = t_ERASE_max × 波特率/10
     *（115200 → 461 B ✓ 1024 够；460800 → 1843 B ✗ 须 ≥2048；921600 → 3686 B
     * 须 ≥4096），否则本条不变量被悄悄破坏。 */
    Boot_UartDma_Poll();

    return (FLASH_ErasePage(page_addr) == FLASH_COMPLETE) ? 0 : 1;
}

static int tio_program_halfword(void *self, u32 addr, u16 value)
{
    (void)self;
    return (FLASH_ProgramHalfWord(addr, value) == FLASH_COMPLETE) ? 0 : 1;
}

static u16 tio_read_halfword(void *self, u32 addr)
{
    (void)self;
    return *(volatile u16 *)addr;
}

static const BootFlash_Io_t s_flashIo =
{
    NULL,                  /* self：无后端私有状态 */
    tio_unlock,
    tio_lock,
    tio_erase_page,
    tio_program_halfword,
    tio_read_halfword
};

static const IAP_MetaIo_t s_metaIo =
{
    NULL,                  /* self */
    tio_erase_page,
    tio_program_halfword,
    tio_read_halfword
};

static const BootFlash_Config_t s_flashCfg =
{
    SLOT_A_BASE,           /* appBase：Boot_Update_Init 改写为运行期目标槽 */
    APP_LIMIT_EXCL,        /* appLimitExcl（两槽合计上界） */
    META_A_BASE,           /* metaABase（page 126） */
    META_B_BASE,           /* metaBBase（page 127） */
    FLASH_PAGE_SIZE        /* pageSize */
};

/*-----------------------------------------------------------
 * 引擎上下文（静态分配，Boot 无堆）
 *----------------------------------------------------------*/
static BootProto_Parser_t s_parser;
static BootProto_Session_t s_session;
static BootFlash_t s_flash;
static u8 s_txBuf[BOOTPROTO_HEADER_LEN + 4u];   /* ACK 24 B / NACK 28 B */

static u8  s_flashSizeOk;        /* 工厂容量 >= 布局要求 */
static u8  s_state;              /* BU_STATE_* */
static u8  s_entry;              /* Boot_UpdateEntry_t */
static u32 s_handshakeDeadline; /* IDLE + BKP 入口的握手窗口 */
static u32 s_frameDeadline;     /* RECEIVING 帧间超时 */
static u32 s_lastFrameSeq;       /* ACK 回显：最近完整帧的 sequence */
static u32 s_lastFrameOffset;    /* ACK 回显：最近完整帧的 image_offset */
static u32 s_receivingPage;     /* 本次会话 RECEIVING 记录所在页基址 */
static u32 s_receivingGen;      /* 本次会话 RECEIVING 的 generation */
static u32 s_targetSlot;        /* 本次会话升级写入的目标槽基址（运行期） */
static u32 s_targetSlotId;      /* 与 s_targetSlot 匹配的 SLOT_ID_A/B */

/*-----------------------------------------------------------
 * 1 ms 时基（Boot 独占 SysTick；Task 9 跳转前负责关闭并清 pending）
 *----------------------------------------------------------*/
static volatile u32 s_tickMs = 0u;

void SysTick_Handler(void)
{
    s_tickMs++;
}

static u32 bu_now(void)
{
    return s_tickMs;   /* 对齐 u32 单次读取，M3 上原子 */
}

/* 截止时刻判定（无符号差比较，绕回安全）：now >= deadline */
static u8 bu_expired(u32 now, u32 deadline)
{
    return ((u32)(now - deadline) < 0x80000000u) ? 1u : 0u;
}

/*-----------------------------------------------------------
 * 小端编码 + ACK/NACK 帧发送
 *----------------------------------------------------------*/
static void bu_wr16(u8 *p, u16 v)
{
    p[0] = (u8)(v & 0xFFu);
    p[1] = (u8)((v >> 8) & 0xFFu);
}

static void bu_wr32(u8 *p, u32 v)
{
    p[0] = (u8)(v & 0xFFu);
    p[1] = (u8)((v >> 8) & 0xFFu);
    p[2] = (u8)((v >> 16) & 0xFFu);
    p[3] = (u8)((v >> 24) & 0xFFu);
}

/* 统一帧发送：payload 仅 NACK 使用（4 B 原因码），ACK 为 0 长度。
 * frame_crc32 覆盖 [4,20)（16 B）+ payload（与主机方向帧同构）。 */
static void bu_send_frame(u8 type, u32 seq, u32 offset,
                          u32 nackReason, u8 hasPayload)
{
    CRC32_StreamCtx_t crc;
    u16 plen = (hasPayload != 0u) ? 4u : 0u;

    s_txBuf[0] = (u8)'I';
    s_txBuf[1] = (u8)'A';
    s_txBuf[2] = (u8)'P';
    s_txBuf[3] = (u8)'1';
    s_txBuf[4] = (u8)BOOTPROTO_VERSION;
    s_txBuf[5] = type;
    bu_wr16(&s_txBuf[6], (u16)BOOTPROTO_HEADER_LEN);
    bu_wr32(&s_txBuf[8], seq);
    bu_wr32(&s_txBuf[12], offset);
    bu_wr16(&s_txBuf[16], plen);
    bu_wr16(&s_txBuf[18], 0u);
    if (hasPayload != 0u)
        bu_wr32(&s_txBuf[24], nackReason);

    /* CRC 覆盖两段：[4,20) 头字段 + [24,24+plen) 负载（CRC 字段本身除外） */
    CRC32_Init(&crc);
    CRC32_Update(&crc, &s_txBuf[4], 16u);
    if (plen != 0u)
        CRC32_Update(&crc, &s_txBuf[24], (u32)plen);
    bu_wr32(&s_txBuf[20], CRC32_Final(&crc));

    Boot_UartDma_Send(s_txBuf, (u16)(BOOTPROTO_HEADER_LEN + plen));
}

static void bu_send_ack(u32 seq, u32 offset)
{
    bu_send_frame((u8)BOOTPROTO_TYPE_ACK, seq, offset, 0u, 0u);
}

static void bu_send_nack(u32 reason, u32 expectedSeq, u32 expectedOffset)
{
    bu_send_frame((u8)BOOTPROTO_TYPE_NACK, expectedSeq, expectedOffset,
                  reason, 1u);
}

/*-----------------------------------------------------------
 * 双页元数据读取（启动决策与 START 提交点共用）
 *----------------------------------------------------------*/
static IAP_MetaSelectResult_t bu_read_meta(IAP_MetaRecord_t *pRecA,
                                            IAP_MetaRecord_t *pRecB,
                                            const IAP_MetaRecord_t **ppSel)
{
    u8 bufA[IAP_META_RECORD_SIZE];
    u8 bufB[IAP_META_RECORD_SIZE];

    (void)IAP_MetaReadRecord(&s_metaIo, META_A_BASE, bufA);
    (void)IAP_MetaReadRecord(&s_metaIo, META_B_BASE, bufB);
    IAP_MetaDecodeRecord(bufA, pRecA);
    IAP_MetaDecodeRecord(bufB, pRecB);
    return IAP_MetaSelectLatest(pRecA, pRecB, ppSel);
}

/*-----------------------------------------------------------
 * 槽解析：由当前元数据判定【活动槽】（正在运行/可回退的槽）
 *
 * F-01 修复：语义从"最新记录必须是 VALID"改为"两页中存在 VALID 记录
 * 即可，取其中 generation 最新者"。升级途中断电时最新记录是
 * RECEIVING（generation 更大），而【旧的 VALID 记录才是活动槽】。
 * 原实现此时返回 0，导致目标槽回落到 Slot A——正好是唯一的好镜像，
 * 断电重试升级会把它擦掉，回滚能力被摧毁。
 *
 * 纯逻辑已下沉到 iap_meta.c 的 IAP_ActiveSlotOf()，生产代码与主机测试
 * 共用同一实现，避免测试用"重写副本"验证错误逻辑（F-16）。
 *----------------------------------------------------------*/
static IAP_ActiveSlot_t bu_active_slot_kind(void)
{
    IAP_MetaRecord_t recA, recB;

    (void)bu_read_meta(&recA, &recB, NULL);
    return IAP_ActiveSlotOf(&recA, &recB, SLOT_A_BASE, SLOT_B_BASE,
                            SLOT_CAPACITY);
}

static u32 bu_active_slot(void)
{
    return IAP_ActiveSlotBase(bu_active_slot_kind(),
                              SLOT_A_BASE, SLOT_B_BASE);
}

/* 由槽基址反查 SLOT_ID（用于写入元数据 slot_id 字段） */
static u32 bu_slot_id_of(u32 slotBase)
{
    if (slotBase == SLOT_A_BASE) return (u32)SLOT_ID_A;
    if (slotBase == SLOT_B_BASE) return (u32)SLOT_ID_B;
    return 0u;
}

/* 选定本次会话的目标槽 + 同步重设写入器/协议会话的基址。
 *
 * 【必须三处同步】目标槽基址要同时写进：
 *   1. s_targetSlot           —— 擦写/CRC 的地址基准（bu_data/bu_end）
 *   2. s_flash.cfg.appBase    —— BootFlash 越界判定边界（值拷贝，非指针）
 *   3. s_session.appBase      —— 协议层 START manifest 的 target_base 校验基准
 *
 * 第 3 项极易漏掉：BootProto_SessionReset() 会 memset 整个会话结构，只保留
 * 上一次 Init/赋值时的 appBase。若不同步，当活动槽为 Slot B（目标槽 = Slot A
 * 以外）时，主机按 Slot A 发 START 仍会通过（appBase 恰为 SLOT_A_BASE 别名），
 * 但目标槽实际是 Slot A —— 首次烧录场景侥幸正确；而一旦**活动槽是 Slot A、
 * 目标槽是 Slot B**，主机必须按 Slot B 发 target_base，此时 s_session.appBase
 * 仍是 SLOT_A_BASE，START 会被协议层 NACK(BAD_START_BASE)，升级永远无法开始。
 */
static void bu_set_target_slot(u32 slotBase, u32 slotId)
{
    s_targetSlot = slotBase;
    s_targetSlotId = slotId;
    s_flash.cfg.appBase = slotBase;   /* 2. 擦写边界随目标槽移动（值拷贝） */
    s_session.appBase = slotBase;     /* 3. 协议层 START 校验基准同步 */
}

/* 元数据记录事务：显式解锁 -> 擦页+编程+commit -> 上锁（错误路径同样成对） */
static int bu_meta_write(u32 pageAddr, const IAP_MetaRecordFields_t *f)
{
    int rc;

    FLASH_Unlock();
    rc = IAP_MetaWriteRecord(&s_metaIo, pageAddr, f);
    FLASH_Lock();
    return rc;
}

/*-----------------------------------------------------------
 * 会话级失败处理：NACK（期望 seq/offset=0，即"从 START 重启"）
 * 后复位会话与解析器（理由见文件头注释）
 *----------------------------------------------------------*/
static void bu_fatal_nack(u32 reason)
{
    bu_send_nack(reason, 0u, 0u);
    BootProto_SessionReset(&s_session);
    BootProto_ParserReset(&s_parser);
    s_state = BU_STATE_IDLE;
    if (s_entry == (u8)BOOT_UPDATE_ENTRY_BKP_REQUEST)
        s_handshakeDeadline = bu_now() + BOOT_UPDATE_HANDSHAKE_TIMEOUT_MS;
}

/* 解析目标槽并把基址同步到全部基准（见 bu_set_target_slot 注释）。
 * 在每次喂帧【之前】调用：协议层的 START target_base 校验必须用本次
 * 目标槽作基准，否则 Slot B 目标会被协议层误判为 BAD_START_BASE。
 * 返回 1 成功；0 表示活动槽非 A/B（元数据异常），调用方应 NACK 拒绝。 */
static u8 bu_arm_target_slot(void)
{
    IAP_ActiveSlot_t act = bu_active_slot_kind();
    u32 activeSlot;
    u32 targetSlot;

    /* 两页都是合法 VALID 但generation 无法排序：元数据不可信，
     * 保守拒绝升级——绝不能在此情况下擦任何一个槽。 */
    if (act == IAP_ACTIVE_AMBIGUOUS) {
        return 0u;
    }

    activeSlot = IAP_ActiveSlotBase(act, SLOT_A_BASE, SLOT_B_BASE);
    if (activeSlot == 0u) {
        targetSlot = SLOT_A_BASE;   /* 无任何 VALID 记录：首次烧录写 Slot A */
    } else {
        targetSlot = IAP_SlotOther(activeSlot, SLOT_A_BASE, SLOT_B_BASE);
    }
    if (targetSlot == 0u) {
        return 0u;                  /* 活动槽非法（元数据被篡改） */
    }

    bu_set_target_slot(targetSlot, bu_slot_id_of(targetSlot));
    return 1u;
}

/*-----------------------------------------------------------
 * 会话事件处理（设计 §5.3 事务步骤）
 *----------------------------------------------------------*/

/* START：提交新一代 RECEIVING 到非最新页 -> 擦目标槽 -> ACK
 *
 * 双槽关键点：升级【永远写活动槽的另一槽】。
 *   - 目标槽已在 bu_arm_target_slot()（喂帧前）解析并同步到
 *     s_targetSlot / s_flash.cfg.appBase / s_session.appBase；
 *    这里只负责提交 RECEIVING 与擦写。
 *   - 元数据 RECEIVING 立即携带目标槽 slot_id/app_base（declare-before-
 *     erase）：此后任何断电 Boot 都因最新记录为 RECEIVING 而拒绝启动，
 *     旧活动槽毫发无损，可恢复。
 */
static void bu_start(const BootProto_Event_t *evt)
{
    IAP_MetaRecord_t recA, recB;
    const IAP_MetaRecord_t *latest = NULL;
    IAP_MetaSelectResult_t sel;
    IAP_MetaRecordFields_t f;
    u32 pageBase;
    u32 newGen;
    u32 pages;
    BootFlash_Status_t st;

    /* 工厂容量门禁：低于布局要求拒绝一切擦写（不得用克隆片隐藏容量） */
    if (s_flashSizeOk == 0u)
    {
        bu_fatal_nack(BOOT_UPDATE_NACK_FLASH_SIZE);
        return;
    }

    /* 目标槽已在 bu_arm_target_slot() 解析（喂帧前），此处直接用 s_targetSlot。
     * 若该函数曾解析失败，本帧不会走到 START_ACCEPTED（已在喂帧前 NACK）。 */

    /* 最新页判定（现场重读：会话重启后元数据可能已变化） */
    sel = bu_read_meta(&recA, &recB, &latest);
    if (sel == IAP_META_SELECT_PAGE_A)
    {
        pageBase = META_B_BASE;
        newGen = latest->generation + 1u;
    }
    else if (sel == IAP_META_SELECT_PAGE_B)
    {
        pageBase = META_A_BASE;
        newGen = latest->generation + 1u;
    }
    else
    {
        /* 两页皆无有效记录/歧义：从 A 页重开日志，generation 从 1 起 */
        pageBase = META_A_BASE;
        newGen = 1u;
    }

    /* 1) declare-before-erase：RECEIVING 先落盘（commit 半字最后编程），
     *    此后任何断电最新记录均为 RECEIVING，Boot 拒绝跳转旧 APP */
    f.generation   = newGen;
    f.stateWord    = IAP_META_STATE_RECEIVING;
    f.slotId       = (u16)s_targetSlotId;
    f.appBase      = s_targetSlot;
    f.imageSize    = evt->startSize;
    f.imageCrc     = evt->startCrc;
    f.imageVersion = evt->startVersion;
    if (bu_meta_write(pageBase, &f) != 0)
    {
        bu_fatal_nack(BOOT_UPDATE_NACK_META_WRITE);
        return;
    }
    s_receivingPage = pageBase;
    s_receivingGen  = newGen;

    /* 2) 擦除目标槽（页数 = ceil(image_size/page)，逐页检查状态）
     *    目标槽取 bu_arm_target_slot() 解析结果（= 活动槽的另一槽） */
    pages = (evt->startSize + FLASH_PAGE_SIZE - 1u) / FLASH_PAGE_SIZE;
    st = BootFlash_ErasePages(&s_flash, s_targetSlot, pages);
    if (st != BOOTFLASH_OK)
    {
        bu_fatal_nack(BOOT_UPDATE_NACK_ERASE);
        return;
    }

    /* 3) 会话进入 RECEIVING，ACK START（主机此后 stop-and-wait 发 DATA） */
    s_state = BU_STATE_RECEIVING;
    s_frameDeadline = bu_now() + BOOT_UPDATE_FRAME_TIMEOUT_MS;
    bu_send_ack(s_lastFrameSeq, s_lastFrameOffset);
}

/* DATA：半字编程 -> 包级回读 -> ACK / NACK（失败按会话级失败处理） */
static void bu_data(const BootProto_Event_t *evt)
{
    BootFlash_Status_t st;
    u32 addr = s_targetSlot + evt->dataOffset;

    st = BootFlash_ProgramData(&s_flash, addr, evt->payload, evt->dataLen);
    if (st != BOOTFLASH_OK)
    {
        bu_fatal_nack(BOOT_UPDATE_NACK_PROGRAM);
        return;
    }

    st = BootFlash_ReadbackVerify(&s_flash, addr, evt->payload, evt->dataLen);
    if (st != BOOTFLASH_OK)
    {
        bu_fatal_nack(BOOT_UPDATE_NACK_READBACK);
        return;
    }

    bu_send_ack(s_lastFrameSeq, s_lastFrameOffset);
}

/* END：整镜像 CRC（从 Flash 重算，不信任 RAM）-> 提交 VALID -> ACK -> 复位 */
static void bu_end(const BootProto_Event_t *evt)
{
    BootFlash_Status_t st;
    IAP_MetaRecordFields_t f;
    u32 validPage;

    (void)evt;   /* 镜像参数取会话 manifest（END 事件本身无负载） */

    st = BootFlash_VerifyCrc(&s_flash, s_targetSlot, s_session.imageSize,
                             s_session.imageCrc);
    if (st != BOOTFLASH_OK)
    {
        bu_fatal_nack(BOOT_UPDATE_NACK_IMAGE_CRC);
        return;
    }

    /* VALID 落到 RECEIVING 之外的另一页，generation = RECEIVING + 1
     *（上一代 VALID 记录所在页被本事务覆盖，双页交替） */
    validPage = (s_receivingPage == META_A_BASE) ? META_B_BASE : META_A_BASE;
    f.generation   = s_receivingGen + 1u;
    f.stateWord    = IAP_META_STATE_VALID;
    f.slotId       = (u16)s_targetSlotId;
    f.appBase      = s_targetSlot;
    f.imageSize    = s_session.imageSize;
    f.imageCrc     = s_session.imageCrc;
    f.imageVersion = s_session.imageVersion;
    if (bu_meta_write(validPage, &f) != 0)
    {
        bu_fatal_nack(BOOT_UPDATE_NACK_META_WRITE);
        return;
    }

    /* 设计 §5.3(8)：ACK 后系统复位，由 Boot 重新走完整启动校验链，
     * 不从升级函数直接跳 APP。Send 等待 TC，ACK 完整移出后复位。 */
    bu_send_ack(s_lastFrameSeq, s_lastFrameOffset);
    NVIC_SystemReset();
}

static void bu_handle_event(const BootProto_Event_t *evt)
{
    switch (evt->kind)
    {
    case (u8)BOOTPROTO_EVT_START_ACCEPTED:
        bu_start(evt);
        break;
    case (u8)BOOTPROTO_EVT_DATA_ACCEPTED:
        bu_data(evt);
        break;
    case (u8)BOOTPROTO_EVT_DUPLICATE:
        /* 最近已接受帧重发：重 ACK，不重复编程（会话级不变量） */
        bu_send_ack(s_lastFrameSeq, s_lastFrameOffset);
        break;
    case (u8)BOOTPROTO_EVT_END_ACCEPTED:
        bu_end(evt);
        break;
    case (u8)BOOTPROTO_EVT_NACK:
        bu_send_nack((u32)evt->nackReason, evt->expectedSeq, evt->expectedOffset);
        break;
    default:
        break;
    }
}

/*-----------------------------------------------------------
 * 解析器回调（在 ParserFeed 内部被调：payload 指针仅在下次 Feed 前有效）
 *----------------------------------------------------------*/
static void bu_on_frame(void *user, const BootProto_Frame_t *frame)
{
    BootProto_Event_t evt;

    (void)user;
    s_lastFrameSeq = frame->sequence;
    s_lastFrameOffset = frame->imageOffset;
    s_frameDeadline = bu_now() + BOOT_UPDATE_FRAME_TIMEOUT_MS;

    /* 喂帧前解析目标槽并同步 s_session.appBase：协议层用 s_session.appBase
     * 校验 START 的 target_base，必须与实际目标槽一致，否则 Slot B 目标会被
     * 误判为 BAD_START_BASE（见 bu_set_target_slot 注释）。
     * 仅在 IDLE（尚未建立会话）时重算：会话进行中不得改道。 */
    if (s_state == BU_STATE_IDLE)
    {
        if (bu_arm_target_slot() == 0u)
        {
            /* 无任何 VALID 记录之外的情形：AMBIGUOUS（两页 VALID 但
             * generation 无法排序）或元数据非法 —— 拒绝一切升级请求，
             * 保守停留，且不擦除任何槽。 */
            bu_fatal_nack(BOOT_UPDATE_NACK_META_WRITE);
            return;
        }
    }

    BootProto_SessionInput(&s_session, frame, &evt);
    bu_handle_event(&evt);
}

static void bu_on_error(void *user, BootProto_FrameErr_t err)
{
    u32 reason;

    (void)user;
    switch (err)
    {
    case BOOTPROTO_FRAME_ERR_VERSION:
        reason = BOOT_UPDATE_NACK_FRAME_VERSION;
        break;
    case BOOTPROTO_FRAME_ERR_CRC:
        reason = BOOT_UPDATE_NACK_FRAME_CRC;
        break;
    default:
        reason = BOOT_UPDATE_NACK_FRAME_FORMAT;
        break;
    }
    /* 期望值取会话当前状态（IDLE 时为 0/0：主机应从 START 开始） */
    bu_send_nack(reason, s_session.expectedSeq, s_session.expectedOffset);
}

/*-----------------------------------------------------------
 * 公开 API
 *----------------------------------------------------------*/

void Boot_Update_Init(void)
{
    u16 sizeKb;

    /* 1 ms 时基：SysTick 由 Boot 独占（裸机，无 RTOS tick 竞争） */
    (void)SysTick_Config(SystemCoreClock / 1000u);

    BootFlash_Init(&s_flash, &s_flashIo, &s_flashCfg);
    BootProto_ParserInit(&s_parser, bu_on_frame, bu_on_error, NULL);
    BootProto_SessionInit(&s_session, SLOT_A_BASE, SLOT_CAPACITY);

    /* 本次会话目标槽：初始化时活动槽未知，先默认 Slot A；
     * 真正的目标槽在 bu_start 里按当时元数据（活动槽的另一槽）重定。 */
    bu_set_target_slot(SLOT_A_BASE, (u32)SLOT_ID_A);

    /* 工厂容量寄存器（KiB）：低于布局要求即拒绝擦写。
     * 克隆 C8 片可能存在隐藏 128 KiB 之说，但出厂容量寄存器低于 64 KiB
     * 的芯片连本布局的元数据页都不可靠，一律拒绝。 */
    sizeKb = *(volatile u16 *)BU_FLASH_SIZE_REG_ADDR;
    s_flashSizeOk = ((u32)sizeKb * 1024u >= (FLASH_END_EXCL - FLASH_BASE))
                    ? 1u : 0u;

    s_state = BU_STATE_IDLE;
    s_entry = (u8)BOOT_UPDATE_ENTRY_RECOVERY_KEY;
    s_handshakeDeadline = 0u;
    s_frameDeadline = 0u;
    s_lastFrameSeq = 0u;
    s_lastFrameOffset = 0u;
    s_receivingPage = META_A_BASE;
    s_receivingGen = 0u;
}

IAP_BootAction_t Boot_Update_MetaDecision(IAP_MetaRecord_t *latestOut)
{
    IAP_MetaRecord_t recA, recB;
    const IAP_MetaRecord_t *sel = NULL;
    IAP_MetaSelectResult_t r;

    r = bu_read_meta(&recA, &recB, &sel);

    if (latestOut != NULL)
    {
        if ((r == IAP_META_SELECT_PAGE_A) || (r == IAP_META_SELECT_PAGE_B))
            *latestOut = *sel;
        else
            memset(latestOut, 0, sizeof(*latestOut));
    }
    return IAP_BootDecision(&recA, &recB, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY);
}

/* 活动槽基址（供 boot_main 跳转链注入）：最新 VALID 记录指向的槽；
 * 无有效记录返回 0（调用方此时本就不会跳到 APP）。 */
u32 Boot_Update_ActiveSlot(void)
{
    return bu_active_slot();
}

u8 Boot_Update_FlashSizeOk(void)
{
    return s_flashSizeOk;
}

void Boot_Update_SessionBegin(Boot_UpdateEntry_t entry)
{
    s_entry = (u8)entry;
    s_state = BU_STATE_IDLE;
    BootProto_SessionReset(&s_session);
    BootProto_ParserReset(&s_parser);
    Boot_UartDma_ResetRx();   /* 排空初始化期间到达的噪声 */

    if (entry == BOOT_UPDATE_ENTRY_BKP_REQUEST)
        s_handshakeDeadline = bu_now() + BOOT_UPDATE_HANDSHAKE_TIMEOUT_MS;
}

Boot_UpdateRun_t Boot_Update_SessionPoll(void)
{
    const u8 *span0;
    const u8 *span1;
    u16 len0;
    u16 len1;
    u16 pending;
    u32 now;

    /* 生产者位置更新（环绕检测 + 溢出判定） */
    Boot_UartDma_Poll();

    if (Boot_UartDma_Overrun() != 0u)
    {
        /* 生产者套圈：数据已丢失。排空接收环并丢半截帧；会话状态保持
         *（帧 CRC + 主机整帧重传负责收敛） */
        Boot_UartDma_ResetRx();
        BootProto_ParserReset(&s_parser);
    }

    /* 消费 [消费者,生产者) 跨度：跨环尾自动拆两段，解析器支持任意分片 */
    pending = Boot_UartDma_Pending(&span0, &len0, &span1, &len1);
    if (pending != 0u)
    {
        BootProto_ParserFeed(&s_parser, span0, (u32)len0);
        if (len1 != 0u)
            BootProto_ParserFeed(&s_parser, span1, (u32)len1);
        Boot_UartDma_Consume(pending);
    }

    now = bu_now();
    if (s_state == BU_STATE_IDLE)
    {
        /* 握手窗口仅对 BKP 请求入口有意义（恢复键/强制入口无限等待） */
        if ((s_entry == (u8)BOOT_UPDATE_ENTRY_BKP_REQUEST) &&
            (bu_expired(now, s_handshakeDeadline) != 0u))
        {
            /* 上报后重启窗口：main 决定回退旧 APP 还是继续等待 */
            s_handshakeDeadline = now + BOOT_UPDATE_HANDSHAKE_TIMEOUT_MS;
            return BOOT_UPDATE_RUN_HANDSHAKE_TIMEOUT;
        }
    }
    else
    {
        /* 帧间超时：只丢半截帧回同步扫描，会话状态不受影响，
         * 主机超时重传整帧即可（设计 §5.2 解析器契约） */
        if (bu_expired(now, s_frameDeadline) != 0u)
        {
            BootProto_ParserReset(&s_parser);
            s_frameDeadline = now + BOOT_UPDATE_FRAME_TIMEOUT_MS;
        }
    }
    return BOOT_UPDATE_RUN_ACTIVE;
}

u8 Boot_Update_IsReceiving(void)
{
    return (s_state == BU_STATE_RECEIVING) ? 1u : 0u;
}
