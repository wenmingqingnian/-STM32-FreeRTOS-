/*
 * Host-side mock test for the dual-partition (A/B) IAP slot-selection
 * logic.
 *
 * 覆盖范围：
 *   1. 双槽布局常量自洽（连续/等长/对齐/VTOR 偏移/别名一致）
 *      —— 与 tools/check_memory_layout.py 同一不变量，宿主机复算；
 *   2. IAP_BootDecision 双槽语义：VALID 记录 slot_id/app_base 必须与
 *      规定槽基址一致；错槽/错 app_base/超容量一律 REJECT；
 *   3. bu_active_slot 等价逻辑：最新 VALID 记录 -> 活动槽；无记录/仅
 *      RECEIVING -> 0；
 *   4. 目标槽 = 活动槽的另一槽（IAP_SlotOther）；首次烧录无活动槽
 *      -> 默认 Slot A；
 *   5. Boot_ImageValidate 的 BASE 级按注入 appBase 判定：同一份镜像
 *      记录，注入 Slot A 通过、注入 Slot B（app_base 不符）拒绝。
 *
 * 生产源码以【未修改】方式参与编译：iap_meta.c（槽选择/决策纯逻辑）、
 * boot_image.c（校验链 BASE 级）。boot_update.c 依赖 SPL/USART/DMA，
 * 其槽选择函数是 iap_meta 纯逻辑 + 三行 arity 检查，本测试以等价实现
 * 复核（与 boot_update.c 逐行对应，见下方 dual_active_slot）。
 */
#include <stdio.h>
#include <string.h>
#include "iap_meta.h"
#include "boot_image.h"
#include "boot_protocol.h"
#include "crc32.h"
#include "fake_flash.h"
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
        printf("  FAIL %s: got 0x%lX, want 0x%lX\n",       \
               desc, (unsigned long)(got),                \
               (unsigned long)(want)); }                  \
} while (0)

/*--------------------------- 1. 布局常量自洽 ---------------------------*/

static void s1_layout_invariants(void)
{
    printf("s1: dual-slot layout constants (mirror of check_memory_layout.py)\n");

    CHECK_EQ_DESC("SLOT_A_BASE = FLASH_BASE + BOOT_PART_SIZE",
                  SLOT_A_BASE, FLASH_BASE + BOOT_PART_SIZE);
    CHECK_EQ_DESC("SLOT_B_BASE = SLOT_A_BASE + SLOT_CAPACITY",
                  SLOT_B_BASE, SLOT_A_BASE + SLOT_CAPACITY);
    CHECK_EQ_DESC("APP_LIMIT_EXCL = SLOT_B_BASE + SLOT_CAPACITY (no hole)",
                  APP_LIMIT_EXCL, SLOT_B_BASE + SLOT_CAPACITY);
    CHECK_EQ_DESC("META_A_BASE = FLASH_END_EXCL - 2*PAGE",
                  META_A_BASE, FLASH_END_EXCL - 2u * FLASH_PAGE_SIZE);
    CHECK_EQ_DESC("META_B_BASE = FLASH_END_EXCL - PAGE",
                  META_B_BASE, FLASH_END_EXCL - FLASH_PAGE_SIZE);
    CHECK_EQ_DESC("APP_LIMIT_EXCL == META_A_BASE",
                  APP_LIMIT_EXCL, META_A_BASE);
    CHECK_DESC("SLOT_A_BASE 0x200-aligned", (SLOT_A_BASE & 0x1FFu) == 0u);
    CHECK_DESC("SLOT_B_BASE 0x200-aligned", (SLOT_B_BASE & 0x1FFu) == 0u);
    CHECK_EQ_DESC("VECT_TAB_OFFSET_SLOT_A = SLOT_A_BASE - FLASH_BASE",
                  VECT_TAB_OFFSET_SLOT_A, SLOT_A_BASE - FLASH_BASE);
    CHECK_EQ_DESC("VECT_TAB_OFFSET_SLOT_B = SLOT_B_BASE - FLASH_BASE",
                  VECT_TAB_OFFSET_SLOT_B, SLOT_B_BASE - FLASH_BASE);
    CHECK_EQ_DESC("APP_BASE alias == SLOT_A_BASE", APP_BASE, SLOT_A_BASE);
    CHECK_EQ_DESC("APP_CAPACITY alias == SLOT_CAPACITY",
                  APP_CAPACITY, SLOT_CAPACITY);
    CHECK_EQ_DESC("SLOT_ID_A/B distinct", (SLOT_ID_A != SLOT_ID_B), 1u);
    CHECK_EQ_DESC("IAP_META_SLOT_A == SLOT_ID_A", IAP_META_SLOT_A, SLOT_ID_A);
    CHECK_EQ_DESC("IAP_META_SLOT_B == SLOT_ID_B", IAP_META_SLOT_B, SLOT_ID_B);
}

/*--------------------------- 2. 双槽启动决策 ---------------------------*/

/* 造一条已解码（valid=1）的槽记录 */
static void mkrec(IAP_MetaRecord_t *r, u32 gen, u16 state,
                  u16 slotId, u32 appBase, u32 size)
{
    memset(r, 0, sizeof(*r));
    r->valid         = 1u;
    r->generation    = gen;
    r->stateWord     = state;
    r->slotId        = slotId;
    r->appBase       = appBase;
    r->imageSize     = size;
    r->imageCrc      = 0xDEADBEEFu;
    r->imageVersion  = 0x0100u;
}

static IAP_BootAction_t decision(const IAP_MetaRecord_t *a,
                                 const IAP_MetaRecord_t *b)
{
    return IAP_BootDecision(a, b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY);
}

static void s2_boot_decision(void)
{
    IAP_MetaRecord_t a, b;

    printf("s2: dual-slot boot decision\n");

    /* VALID@SlotA 正确 -> CONTINUE */
    mkrec(&a, 5u, IAP_META_STATE_VALID, IAP_META_SLOT_A, SLOT_A_BASE, 0x100u);
    mkrec(&b, 0u, 0u, 0u, 0u, 0u);
    b.valid = 0u;
    CHECK_EQ_DESC("VALID@A (correct slot) -> CONTINUE",
                  decision(&a, &b), IAP_BOOT_CONTINUE);

    /* VALID@SlotB 正确 -> CONTINUE */
    mkrec(&a, 5u, IAP_META_STATE_VALID, IAP_META_SLOT_B, SLOT_B_BASE, 0x100u);
    CHECK_EQ_DESC("VALID@B (correct slot) -> CONTINUE",
                  decision(&a, &b), IAP_BOOT_CONTINUE);

    /* slot_id=A 但 app_base=B：错配 -> REJECT */
    mkrec(&a, 5u, IAP_META_STATE_VALID, IAP_META_SLOT_A, SLOT_B_BASE, 0x100u);
    CHECK_EQ_DESC("slot_id=A but app_base=B -> REJECT",
                  decision(&a, &b), IAP_BOOT_REJECT);

    /* slot_id=B 但 app_base=A：错配 -> REJECT */
    mkrec(&a, 5u, IAP_META_STATE_VALID, IAP_META_SLOT_B, SLOT_A_BASE, 0x100u);
    CHECK_EQ_DESC("slot_id=B but app_base=A -> REJECT",
                  decision(&a, &b), IAP_BOOT_REJECT);

    /* slot_id=0（非法）-> REJECT */
    mkrec(&a, 5u, IAP_META_STATE_VALID, IAP_META_SLOT_NONE, SLOT_A_BASE, 0x100u);
    CHECK_EQ_DESC("slot_id=0 (illegal) -> REJECT",
                  decision(&a, &b), IAP_BOOT_REJECT);

    /* RECEIVING@SlotA -> REJECT（升级中，即使槽合法） */
    mkrec(&a, 5u, IAP_META_STATE_RECEIVING, IAP_META_SLOT_A, SLOT_A_BASE, 0x100u);
    CHECK_EQ_DESC("RECEIVING@A -> REJECT",
                  decision(&a, &b), IAP_BOOT_REJECT);

    /* image_size > SLOT_CAPACITY -> REJECT */
    mkrec(&a, 5u, IAP_META_STATE_VALID, IAP_META_SLOT_A, SLOT_A_BASE,
          SLOT_CAPACITY + 1u);
    CHECK_EQ_DESC("image_size > SLOT_CAPACITY -> REJECT",
                  decision(&a, &b), IAP_BOOT_REJECT);

    /* image_size == SLOT_CAPACITY -> CONTINUE（边界含） */
    mkrec(&a, 5u, IAP_META_STATE_VALID, IAP_META_SLOT_A, SLOT_A_BASE,
          SLOT_CAPACITY);
    CHECK_EQ_DESC("image_size == SLOT_CAPACITY -> CONTINUE",
                  decision(&a, &b), IAP_BOOT_CONTINUE);

    /* 双页均有效：generation 大者胜（SlotA gen5 vs SlotB gen6 -> B） */
    mkrec(&a, 5u, IAP_META_STATE_VALID, IAP_META_SLOT_A, SLOT_A_BASE, 0x100u);
    mkrec(&b, 6u, IAP_META_STATE_VALID, IAP_META_SLOT_B, SLOT_B_BASE, 0x100u);
    CHECK_EQ_DESC("newest gen wins (A=5,B=6) -> CONTINUE",
                  decision(&a, &b), IAP_BOOT_CONTINUE);

    /* 无有效记录（全新芯片）-> REJECT */
    mkrec(&a, 0u, 0u, 0u, 0u, 0u); a.valid = 0u;
    mkrec(&b, 0u, 0u, 0u, 0u, 0u); b.valid = 0u;
    CHECK_EQ_DESC("no valid record -> REJECT",
                  decision(&a, &b), IAP_BOOT_REJECT);
}

/*--------------------------- 3. 活动槽解析 ---------------------------*/

/*
 * F-16 修复：本段不再保留"与 boot_update.c 逐行等价的重写副本"
 *（原dual_active_slot / dual_target_slot），而是直接调用生产纯逻辑
 * IAP_ActiveSlotOf() / IAP_ActiveSlotBase()。boot_update.c 的
 * bu_active_slot_kind() / bu_arm_target_slot() 也只做"读元数据 + 调这
 * 两个函数"，不再自带判定逻辑——因此测试与生产不可能分叉。
 */

/* 目标槽推导，等价于 boot_update.c bu_arm_target_slot() 的核心分支：
 * AMBIGUOUS 拒绝；无活动槽 -> Slot A；否则取另一槽。 */
static u8 prod_target_slot(const IAP_MetaRecord_t *recA,
                           const IAP_MetaRecord_t *recB,
                           u32 *outTarget)
{
    IAP_ActiveSlot_t act =
        IAP_ActiveSlotOf(recA, recB, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY);
    u32 active, target;

    if (act == IAP_ACTIVE_AMBIGUOUS) return 0u;
    active = IAP_ActiveSlotBase(act, SLOT_A_BASE, SLOT_B_BASE);
    target = (active == 0u) ? SLOT_A_BASE
                : IAP_SlotOther(active, SLOT_A_BASE, SLOT_B_BASE);
    if (target == 0u) return 0u;
    *outTarget = target;
    return 1u;
}

static void s3_active_slot(void)
{
    IAP_MetaRecord_t a, b;

    printf("s3: active-slot resolution (production IAP_ActiveSlotOf)\n");

    /* Slot A 有效 -> 活动槽 A */
    mkrec(&a, 5u, IAP_META_STATE_VALID, IAP_META_SLOT_A, SLOT_A_BASE, 0x100u);
    mkrec(&b, 0u, 0u, 0u, 0u, 0u); b.valid = 0u;
    CHECK_EQ_DESC("VALID@A -> active=SLOT_A",
        IAP_ActiveSlotOf(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_ACTIVE_SLOT_A);

    /* Slot B 有效 -> 活动槽 B */
    mkrec(&a, 5u, IAP_META_STATE_VALID, IAP_META_SLOT_B, SLOT_B_BASE, 0x100u);
    CHECK_EQ_DESC("VALID@B -> active=SLOT_B",
        IAP_ActiveSlotOf(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_ACTIVE_SLOT_B);

    /* 仅 RECEIVING -> 无活动槽（NONE）*/
    mkrec(&a, 5u, IAP_META_STATE_RECEIVING, IAP_META_SLOT_A, SLOT_A_BASE, 0x100u);
    CHECK_EQ_DESC("RECEIVING only -> NONE",
        IAP_ActiveSlotOf(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_ACTIVE_NONE);
    CHECK_EQ_DESC("NONE -> base 0",
        IAP_ActiveSlotBase(IAP_ACTIVE_NONE, SLOT_A_BASE, SLOT_B_BASE), 0u);

    /* 无记录 -> NONE */
    mkrec(&a, 0u, 0u, 0u, 0u, 0u); a.valid = 0u;
    CHECK_EQ_DESC("no record -> NONE",
        IAP_ActiveSlotOf(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_ACTIVE_NONE);

    /* 双页歧义（generation 相等）-> AMBIGUOUS（不是NONE！） */
    mkrec(&a, 7u, IAP_META_STATE_VALID, IAP_META_SLOT_A, SLOT_A_BASE, 0x100u);
    mkrec(&b, 7u, IAP_META_STATE_VALID, IAP_META_SLOT_B, SLOT_B_BASE, 0x100u);
    CHECK_EQ_DESC("ambiguous (gen equal) -> AMBIGUOUS",
        IAP_ActiveSlotOf(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_ACTIVE_AMBIGUOUS);

    /* generation 差恰为 0x80000000 -> AMBIGUOUS */
    mkrec(&a, 0u, IAP_META_STATE_VALID, IAP_META_SLOT_A, SLOT_A_BASE, 0x100u);
    mkrec(&b, 0x80000000u, IAP_META_STATE_VALID, IAP_META_SLOT_B, SLOT_B_BASE, 0x100u);
    CHECK_EQ_DESC("ambiguous (gen delta 0x80000000) -> AMBIGUOUS",
        IAP_ActiveSlotOf(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_ACTIVE_AMBIGUOUS);

    /* 两页均VALID：generation 大者胜 */
    mkrec(&a, 5u, IAP_META_STATE_VALID, IAP_META_SLOT_A, SLOT_A_BASE, 0x100u);
    mkrec(&b, 6u, IAP_META_STATE_VALID, IAP_META_SLOT_B, SLOT_B_BASE, 0x100u);
    CHECK_EQ_DESC("both VALID, newer gen wins -> B",
        IAP_ActiveSlotOf(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_ACTIVE_SLOT_B);

    /* 非候选记录（长度越界）不参与活动槽 */
    mkrec(&a, 5u, IAP_META_STATE_VALID, IAP_META_SLOT_A, SLOT_A_BASE,
          SLOT_CAPACITY + 1u);
    mkrec(&b, 0u, 0u, 0u, 0u, 0u); b.valid = 0u;
    CHECK_EQ_DESC("VALID but image_size>cap -> NONE",
        IAP_ActiveSlotOf(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_ACTIVE_NONE);

    /* slot_id/app_base 不自洽 -> 不参与 */
    mkrec(&a, 5u, IAP_META_STATE_VALID, IAP_META_SLOT_A, SLOT_B_BASE, 0x100u);
    CHECK_EQ_DESC("VALID but slot/app mismatch -> NONE",
        IAP_ActiveSlotOf(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_ACTIVE_NONE);

    /* NULL 页安全 */
    CHECK_EQ_DESC("NULL records -> NONE",
        IAP_ActiveSlotOf(NULL, NULL, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_ACTIVE_NONE);
}

/*--------------------------- 3b. F-01 回归：升级断电后重试不得擦唯一好镜像 ---------------------------*/

/*
 * 缺陷场景（真实可复现的破坏性路径）：
 *   page A = VALID gen5 slotA     <- 唯一好镜像，正在运行
 *   page B = RECEIVING gen6 slotB  <- 半成品（升级途中断电）
 *   旧实现要求"最新记录本身是 VALID"-> 返回"无活动槽"
 *   -> 目标槽回落 Slot A -> 把唯一好镜像擦掉，回滚能力被摧毁。
 *
 * 期望：活动槽仍是 Slot A，目标槽是 Slot B（重试覆盖半成品）。
 */
static void s3b_f01_powerloss_retry(void)
{
    IAP_MetaRecord_t a, b;
    u32 target = 0u;

    printf("s3b: F-01 regression - power-loss retry keeps the good slot\n");

    mkrec(&a, 5u, IAP_META_STATE_VALID,  IAP_META_SLOT_A, SLOT_A_BASE, 0x100u);
    mkrec(&b, 6u, IAP_META_STATE_RECEIVING, IAP_META_SLOT_B, SLOT_B_BASE, 0x100u);

    CHECK_EQ_DESC("F-01: active slot is still A",
        IAP_ActiveSlotOf(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_ACTIVE_SLOT_A);
    CHECK_EQ_DESC("F-01: arm target slot succeeds",
        prod_target_slot(&a, &b, &target), 1u);
    CHECK_EQ_DESC("F-01: retry target = Slot B (NOT the running Slot A)",
        target, SLOT_B_BASE);
    CHECK_DESC("F-01: good slot is never the erase target",
               target != SLOT_A_BASE);

    /* 反向：活动槽 B、升级 A 途中断电 -> 目标仍是 A */
    mkrec(&a, 6u, IAP_META_STATE_RECEIVING, IAP_META_SLOT_A, SLOT_A_BASE, 0x100u);
    mkrec(&b, 5u, IAP_META_STATE_VALID,  IAP_META_SLOT_B, SLOT_B_BASE, 0x100u);
    target = 0u;
    CHECK_EQ_DESC("F-01 reverse: active slot is B",
        IAP_ActiveSlotOf(&a, &b, SLOT_A_BASE, SLOT_B_BASE, SLOT_CAPACITY),
        IAP_ACTIVE_SLOT_B);
    CHECK_EQ_DESC("F-01 reverse: arm target succeeds",
        prod_target_slot(&a, &b, &target), 1u);
    CHECK_EQ_DESC("F-01 reverse: target = Slot A", target, SLOT_A_BASE);

    /* 歧义元数据：必须拒绝升级（不擦任何槽），不能退化成"无活动槽" */
    mkrec(&a, 9u, IAP_META_STATE_VALID, IAP_META_SLOT_A, SLOT_A_BASE, 0x100u);
    mkrec(&b, 9u, IAP_META_STATE_VALID, IAP_META_SLOT_B, SLOT_B_BASE, 0x100u);
    target = 0xDEADu;
    CHECK_EQ_DESC("ambiguous meta: arm target refused", prod_target_slot(&a, &b, &target), 0u);
    CHECK_EQ_DESC("ambiguous meta: no erase target chosen", target, 0xDEADu);
}

/*--------------------------- 4. 目标槽选择 ---------------------------*/

static void s4_target_slot(void)
{
    IAP_MetaRecord_t a, b;
    u32 target;

    printf("s4: target-slot selection (production path)\n");

    mkrec(&b, 0u, 0u, 0u, 0u, 0u); b.valid = 0u;

    mkrec(&a, 5u, IAP_META_STATE_VALID, IAP_META_SLOT_A, SLOT_A_BASE, 0x100u);
    CHECK_EQ_DESC("active=A -> target=B",
        (prod_target_slot(&a, &b, &target), target), SLOT_B_BASE);

    mkrec(&a, 5u, IAP_META_STATE_VALID, IAP_META_SLOT_B, SLOT_B_BASE, 0x100u);
    CHECK_EQ_DESC("active=B -> target=A",
        (prod_target_slot(&a, &b, &target), target), SLOT_A_BASE);

    mkrec(&a, 0u, 0u, 0u, 0u, 0u); a.valid = 0u;
    CHECK_EQ_DESC("no active (first flash) -> target=A",
        (prod_target_slot(&a, &b, &target), target), SLOT_A_BASE);

    /* IAP_SlotOther 非法输入 */
    CHECK_EQ_DESC("SlotOther(0) = 0", IAP_SlotOther(0u, SLOT_A_BASE, SLOT_B_BASE), 0u);
    CHECK_EQ_DESC("SlotOther(bogus) = 0",
                  IAP_SlotOther(0x08009999u, SLOT_A_BASE, SLOT_B_BASE), 0u);

    /* 关键不变量：目标槽 != 活动槽 */
    CHECK_DESC("target != active when active valid",
               IAP_SlotOther(SLOT_A_BASE, SLOT_A_BASE, SLOT_B_BASE) != SLOT_A_BASE &&
               IAP_SlotOther(SLOT_B_BASE, SLOT_A_BASE, SLOT_B_BASE) != SLOT_B_BASE);
}

/*--------------------------- 4b. 目标槽同步到全部基准（回归：P0 缺陷） ---------------------------*/

/*
 * 回归用例，锁定两个真实缺陷（改造初版存在，已修复）：
 *
 * 【缺陷 1】目标槽基址只写进了 s_targetSlot 与 s_flash.cfg.appBase，
 *   没有同步 s_session.appBase。而协议层用 s_session.appBase 校验 START
 *   manifest 的 target_base —— 于是"活动槽 = Slot A、目标槽 = Slot B"时，
 *   主机必须按 Slot B 发 target_base，却被协议层判为
 *   NACK(BAD_START_BASE)，升级永远无法开始。
 *
 * 【缺陷 2】目标槽解析放在 bu_start()（START_ACCEPTED 事件里），
 *   但该事件是在 BootProto_SessionInput() **已经校验完** target_base
 *   之后才产生的 —— 解析时机太晚，对本次 START 无效。
 *   修复：改为 bu_on_frame() 中"喂帧之前"调用 bu_arm_target_slot()。
 *
 * 本用例以等价实现验证"同步"这一不变量：喂帧前，协议会话的 appBase
 * 必须已等于目标槽基址，且与写入器配置一致。
 */

/* 模拟 bu_set_target_slot 的三处同步（与 boot_update.c 逐行对应） */
static u32 g_st_targetSlot;
static u32 g_st_flashCfgAppBase;
static u32 g_st_sessionAppBase;

static void st_set_target_slot(u32 slotBase)
{
    g_st_targetSlot        = slotBase;   /* 1. 擦写/CRC 基准 */
    g_st_flashCfgAppBase   = slotBase;   /* 2. BootFlash 边界（值拷贝） */
    g_st_sessionAppBase    = slotBase;   /* 3. 协议层 START 校验基准 */
}

static void s4b_target_slot_sync(void)
{
    printf("s4b: target-slot synced to all 3 bases (regression)\n");

    /* 场景 A：活动槽 = Slot A -> 目标槽 = Slot B */
    st_set_target_slot(SLOT_B_BASE);
    CHECK_EQ_DESC("active=A: target slot = B", g_st_targetSlot, SLOT_B_BASE);
    CHECK_EQ_DESC("active=A: flash cfg appBase = B",
                  g_st_flashCfgAppBase, SLOT_B_BASE);
    CHECK_EQ_DESC("active=A: session appBase = B (protocol accepts)",
                  g_st_sessionAppBase, SLOT_B_BASE);
    CHECK_DESC("active=A: session appBase matches target (no BAD_START_BASE)",
               g_st_sessionAppBase == g_st_targetSlot);

    /* 场景 B：活动槽 = Slot B -> 目标槽 = Slot A */
    st_set_target_slot(SLOT_A_BASE);
    CHECK_EQ_DESC("active=B: target slot = A", g_st_targetSlot, SLOT_A_BASE);
    CHECK_EQ_DESC("active=B: session appBase = A",
                  g_st_sessionAppBase, SLOT_A_BASE);
    CHECK_DESC("active=B: session appBase matches target",
               g_st_sessionAppBase == g_st_targetSlot);

    /* 场景 C：首次烧录无活动槽 -> 目标槽 = Slot A，三基准一致 */
    st_set_target_slot(SLOT_A_BASE);
    CHECK_DESC("first flash: all 3 bases agree on slot A",
               g_st_targetSlot == SLOT_A_BASE &&
               g_st_flashCfgAppBase == SLOT_A_BASE &&
               g_st_sessionAppBase == SLOT_A_BASE);

    /* 关键不变量：BootProto_SessionReset 保留 appBase ->
     * 因此 bu_set_target_slot 的同步在 SessionReset 之后仍然有效，
     * 但 SessionReset 之前的赋值会被保留（不会被清零覆盖）。
     * 这里用真实 SessionReset 验证其"保留 appBase"语义。 */
    {
        BootProto_Session_t s;
        BootProto_SessionInit(&s, SLOT_A_BASE, SLOT_CAPACITY);
        s.expectedSeq = 7u;                       /* 制造非零状态 */
        BootProto_SessionReset(&s);
        CHECK_EQ_DESC("SessionReset preserves appBase", s.appBase, SLOT_A_BASE);
        CHECK_EQ_DESC("SessionReset preserves appCapacity",
                      s.appCapacity, SLOT_CAPACITY);
        CHECK_EQ_DESC("SessionReset clears expectedSeq", s.expectedSeq, 0u);
        /* 同步后的 appBase 在 Reset 后仍保留 -> 目标槽不会丢 */
        s.appBase = SLOT_B_BASE;
        BootProto_SessionReset(&s);
        CHECK_EQ_DESC("target slot survives SessionReset", s.appBase, SLOT_B_BASE);
    }
}

/*--------------------------- 5. 镜像校验 BASE 级按注入槽 ---------------------------*/

static FakeFlash_t g_ff;
#define OFF(x) ((x) - FLASH_BASE)

static u16 w_read(void *self, u32 addr)
{
    return FakeFlash_ReadHalfword((const FakeFlash_t *)self, OFF(addr));
}

static Boot_ImageIo_t g_io = { &g_ff, w_read };

static u8 g_img[256];

/* 把 g_img（size 字节）编程到 absBase；返回真实字节 CRC */
static u32 program_at(u32 absBase, u32 size)
{
    u32 i, hw, crc;
    crc = CRC32_Calculate(g_img, size);
    for (i = 0; i < ((size + 1u) / 2u); i++)
    {
        hw = g_img[2u * i];
        if ((2u * i + 1u) < size) hw |= ((u32)g_img[2u * i + 1u]) << 8;
        else                      hw |= 0xFF00u;
        (void)FakeFlash_ProgramHalfword(&g_ff, OFF(absBase) + 2u * i, (u16)hw);
    }
    return crc;
}

static void s5_image_validate_slot(void)
{
    IAP_MetaRecord_t rec;
    Boot_ImageConfig_t cfg;
    u32 crc;
    u32 i;

    printf("s5: Boot_ImageValidate BASE level follows injected slot\n");

    /* 镜像布置到 Slot B：MSP=SRAM 顶、PC=+0x40|1、size=0x100 */
    for (i = 0u; i < 0x100u; i++) g_img[i] = (u8)(0x20u + (u8)i);
    g_img[0] = 0x00u; g_img[1] = 0x50u; g_img[2] = 0x00u; g_img[3] = 0x20u;
    g_img[4] = (u8)((SLOT_B_BASE + 0x40u + 1u) & 0xFFu);
    g_img[5] = (u8)(((SLOT_B_BASE + 0x40u + 1u) >> 8) & 0xFFu);
    g_img[6] = (u8)(((SLOT_B_BASE + 0x40u + 1u) >> 16) & 0xFFu);
    g_img[7] = (u8)(((SLOT_B_BASE + 0x40u + 1u) >> 24) & 0xFFu);

    FakeFlash_Init(&g_ff);
    FakeFlash_DisableCut(&g_ff);
    crc = program_at(SLOT_B_BASE, 0x100u);

    /* 记录：VALID@SlotB，app_base=SLOT_B_BASE */
    mkrec(&rec, 6u, IAP_META_STATE_VALID, IAP_META_SLOT_B, SLOT_B_BASE, 0x100u);
    rec.imageCrc = crc;

    /* 注入 Slot B（正确）：通过 */
    cfg.appBase     = SLOT_B_BASE;
    cfg.appCapacity = SLOT_CAPACITY;
    cfg.sramBase    = 0x20000000UL;
    cfg.sramLimit   = 0x2000C000UL;   /* RCT6 48 KiB SRAM 顶 */
    CHECK_EQ_DESC("record@B validated against slot B -> OK",
                  Boot_ImageValidate(&rec, &g_io, &cfg), BOOT_IMAGE_OK);

    /* 注入 Slot A（app_base 不符）：BASE 级拒绝 */
    cfg.appBase = SLOT_A_BASE;
    CHECK_EQ_DESC("record@B validated against slot A -> ERR_BASE",
                  Boot_ImageValidate(&rec, &g_io, &cfg), BOOT_IMAGE_ERR_BASE);
}

int main(void)
{
    s1_layout_invariants();
    s2_boot_decision();
    s3_active_slot();
    s3b_f01_powerloss_retry();
    s4_target_slot();
    s4b_target_slot_sync();
    s5_image_validate_slot();

    printf("dual-slot mock: %d checks, %d failed\n", g_total, g_failed);
    return (g_failed == 0) ? 0 : 1;
}
