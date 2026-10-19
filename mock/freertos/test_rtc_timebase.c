/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Host-side mock test for the RTC timebase backend
 * Covers:
 *
 * The SAME test source is compiled TWICE against the UNMODIFIED production
 * source BSP\LOWPOWER\rtc_timebase.c (see run-mock-iap-stop.ps1):
 *
 *   1. LSE build : -I board_lse shadows CONFIG\board_profile.h with
 *                  BOARD_C8_LSE=1 -> the full IO-injected implementation is
 *                  linked (init fast/slow paths, LSE timeout, stable read,
 *                  alarm ordering).
 *   2. stub build: -I CONFIG keeps BOARD_PROFILE_PENDING -> the zero-
 *                  hardware stub is linked; every API must return
 *                  RTC_TB_NOT_SUPPORTED and NO IO callback may fire
 *                  (no PC14/PC15 or backup-domain RTC
 *                  source access on non-LSE boards).
 *
 * Covers:
 *   - first-init sequence (backup-domain reset -> LSE w/ bounded timeout ->
 *     RTCSEL=LSE -> prescaler 31 -> sync -> alarm EXTI init -> signature
 *     write) with strict ordering assertions
 *   - fast path on layout-signature match + LSE running (no backup-domain
 *     reset, no LSE restart, no signature rewrite: BKP_DR2 upgrade requests
 *     and RTC counting survive resets, design §5.1/§6.2)
 *   - LSE timeout -> RTC_TB_ERR_LSE_TIMEOUT with ready=0 (degradation signal
 *     for the Task 11 policy snapshot), no RTCSEL/prescaler/signature
 *     writes, bounded wait_poll count, off-by-one timeout boundary
 *   - high-low-high stable read: clean read / one torn round (low-word wrap
 *     simulation) / permanently torn -> bounded retries then
 *     RTC_TB_ERR_READ_UNSTABLE (design §6.2)
 *   - SetAlarm clear-then-set and CancelAlarm cancel-then-clear ordering
 *     (design §6.6 enter step 3 / exit step 4)
 *   - NOT_READY guard before Init succeeds; NULL-parameter defense
 */
#include <stdio.h>
#include <string.h>
#include "board_profile.h"
#include "rtc_timebase.h"

static int g_failed = 0;
static int g_total = 0;

#define CHECK_DESC(desc, cond) do {                          \
    g_total++;                                                \
    if (cond) { printf("  ok %s\n", desc); }                  \
    else      { g_failed++; printf("  FAIL %s\n", desc); }    \
} while (0)

#define CHECK_EQ_DESC(desc, got, want) do {                  \
    g_total++;                                                \
    if ((long)(got) == (long)(want))                          \
        printf("  ok %s\n", desc);                            \
    else { g_failed++;                                        \
        printf("  FAIL %s: got %ld, want %ld\n",              \
               desc, (long)(got), (long)(want)); }            \
} while (0)

#if (BOARD_C8_LSE != 0)

/*-----------------------------------------------------------
 * 脚本化 RTC IO（全部寄存器访问被注入替换）
 *----------------------------------------------------------*/
typedef enum {
    OP_PWR_BKP_ENABLE = 1,
    OP_READ_BKP,
    OP_WRITE_BKP,
    OP_BKP_DOMAIN_RESET,
    OP_LSE_ON,
    OP_LSE_READY,
    OP_RTC_SEL_LSE,
    OP_SET_PRL,
    OP_WAIT_RTC_SYNC,
    OP_ALARM_EXTI_INIT,
    OP_READ_CNT_H,
    OP_READ_CNT_L,
    OP_CLEAR_ALARM_FLAGS,
    OP_SET_ALARM,
    OP_CANCEL_ALARM,
    OP_WAIT_POLL,
    OP_MAX
} FakeOp_t;

#define FAKE_ORDER_CAP  256

typedef struct {
    u32 order[FAKE_ORDER_CAP];
    u32 order_len;
    u32 count[OP_MAX + 1u];
    u32 bkp_dr[16];            /* BKP_DR1..DR15 影子 */
    u32 bkp_reset_done;
    u32 lse_ready_calls;
    u32 lse_ready_at_call;      /* 第 N 次查询起返回 1；0=永不 ready */
    u32 lse_on_count;
    u16 cnt_high, cnt_low;     /* 计数器影子初值 */
    u32 tear_rounds;           /* 撕裂注入轮数（模拟低字回绕进位） */
    u32 prl_value;
    u32 alarm_value;
    u32 cancel_count;
} FakeRtc_t;

static void rec(FakeRtc_t *f, FakeOp_t op)
{
    if (f->order_len < FAKE_ORDER_CAP) {
        f->order[f->order_len++] = (u32)op;
    }
    f->count[op]++;
}

/* from 起首次出现序号；未出现返回 -1 */
static int idx_of(const FakeRtc_t *f, u32 from, FakeOp_t op)
{
    u32 i;
    for (i = from; i < f->order_len; i++) {
        if (f->order[i] == (u32)op) {
            return (int)i;
        }
    }
    return -1;
}

/* from 起首次出现 op 的序号 == 期待值（精确位置断言用） */
static int idx_is(const FakeRtc_t *f, u32 from, FakeOp_t op, int expect)
{
    int got = idx_of(f, from, op);
    return (got == expect);
}

/* a 在 b 之前（from 起各自首次出现且 ia < ib） */
static int a_before_b(const FakeRtc_t *f, u32 from, FakeOp_t a, FakeOp_t b)
{
    int ia = idx_of(f, from, a);
    int ib = idx_of(f, from, b);
    return (ia >= 0) && (ib >= 0) && (ia < ib);
}

/*--------------------------- IO 回调 ---------------------------*/

static void fake_pwr_bkp_enable(void *self)
{
    rec((FakeRtc_t *)self, OP_PWR_BKP_ENABLE);
}

static u32 fake_read_bkp_dr(void *self, u32 dr_index)
{
    FakeRtc_t *f = (FakeRtc_t *)self;
    rec(f, OP_READ_BKP);
    if (dr_index < 16u) {
        return f->bkp_dr[dr_index];
    }
    return 0u;
}

static void fake_write_bkp_dr(void *self, u32 dr_index, u32 value)
{
    FakeRtc_t *f = (FakeRtc_t *)self;
    rec(f, OP_WRITE_BKP);
    if (dr_index < 16u) {
        f->bkp_dr[dr_index] = value;
    }
}

static void fake_backup_domain_reset(void *self)
{
    FakeRtc_t *f = (FakeRtc_t *)self;
    rec(f, OP_BKP_DOMAIN_RESET);
    f->bkp_reset_done = 1u;
}

static void fake_lse_on(void *self)
{
    FakeRtc_t *f = (FakeRtc_t *)self;
    rec(f, OP_LSE_ON);
    f->lse_on_count++;
}

static u8 fake_lse_ready(void *self)
{
    FakeRtc_t *f = (FakeRtc_t *)self;
    rec(f, OP_LSE_READY);
    f->lse_ready_calls++;
    if (f->lse_ready_at_call == 0u) {
        return 0u;                              /* 永不 ready（超时注入） */
    }
    return (f->lse_ready_calls >= f->lse_ready_at_call) ? 1u : 0u;
}

static void fake_rtc_select_lse(void *self)
{
    rec((FakeRtc_t *)self, OP_RTC_SEL_LSE);
}

static void fake_set_prescaler(void *self, u32 prl)
{
    FakeRtc_t *f = (FakeRtc_t *)self;
    rec(f, OP_SET_PRL);
    f->prl_value = prl;
}

static void fake_wait_rtc_sync(void *self)
{
    rec((FakeRtc_t *)self, OP_WAIT_RTC_SYNC);
}

static void fake_alarm_exti_nvic_init(void *self)
{
    rec((FakeRtc_t *)self, OP_ALARM_EXTI_INIT);
}

/* 撕裂注入：前 tear_rounds 轮内每次读高字递增（h1 != h2），
 * 之后恒定返回 cnt_high + 2*tear_rounds（稳定值） */
static u16 fake_read_cnt_high(void *self)
{
    FakeRtc_t *f = (FakeRtc_t *)self;
    u32 calls = f->count[OP_READ_CNT_H];
    rec(f, OP_READ_CNT_H);
    if (calls < (2u * f->tear_rounds)) {
        return (u16)(f->cnt_high + (u16)(calls + 1u));
    }
    return (u16)(f->cnt_high + (u16)(2u * f->tear_rounds));
}

static u16 fake_read_cnt_low(void *self)
{
    FakeRtc_t *f = (FakeRtc_t *)self;
    rec(f, OP_READ_CNT_L);
    return f->cnt_low;
}

static void fake_clear_alarm_flags(void *self)
{
    rec((FakeRtc_t *)self, OP_CLEAR_ALARM_FLAGS);
}

static void fake_set_alarm(void *self, u32 counter)
{
    FakeRtc_t *f = (FakeRtc_t *)self;
    rec(f, OP_SET_ALARM);
    f->alarm_value = counter;
}

static void fake_cancel_alarm(void *self)
{
    FakeRtc_t *f = (FakeRtc_t *)self;
    rec(f, OP_CANCEL_ALARM);
    f->cancel_count++;
}

static void fake_wait_poll(void *self)
{
    rec((FakeRtc_t *)self, OP_WAIT_POLL);
}

static const RtcTimebase_Io_t g_fake_io = {
    NULL,
    fake_pwr_bkp_enable,
    fake_read_bkp_dr,
    fake_write_bkp_dr,
    fake_backup_domain_reset,
    fake_lse_on,
    fake_lse_ready,
    fake_rtc_select_lse,
    fake_set_prescaler,
    fake_wait_rtc_sync,
    fake_alarm_exti_nvic_init,
    fake_read_cnt_high,
    fake_read_cnt_low,
    fake_clear_alarm_flags,
    fake_set_alarm,
    fake_cancel_alarm,
    fake_wait_poll
};

/*--------------------------- 场景测试 ---------------------------*/

/* s0: NULL 参数防御 */
static void s0_param_defense(void)
{
    RtcTimebase_t tb;
    u32 out = 0u;

    printf("s0: NULL parameter defense\n");

    CHECK_EQ_DESC("Init(NULL tb) -> ERR_PARAM",
                  RtcTimebase_Init(NULL, &g_fake_io), RTC_TB_ERR_PARAM);
    CHECK_EQ_DESC("Init(NULL io) -> ERR_PARAM",
                  RtcTimebase_Init(&tb, NULL), RTC_TB_ERR_PARAM);

    CHECK_EQ_DESC("IsReady(NULL) -> 0", RtcTimebase_IsReady(NULL), 0);
    CHECK_EQ_DESC("ReadCounterStable(NULL) -> ERR_PARAM",
                  RtcTimebase_ReadCounterStable(NULL, &out), RTC_TB_ERR_PARAM);
    CHECK_EQ_DESC("SetAlarm(NULL) -> ERR_PARAM",
                  RtcTimebase_SetAlarm(NULL, 1u), RTC_TB_ERR_PARAM);
    CHECK_EQ_DESC("CancelAlarm(NULL) -> ERR_PARAM",
                  RtcTimebase_CancelAlarm(NULL), RTC_TB_ERR_PARAM);
}

/* s1: 首次完整初始化（签名不匹配 + LSE 立即就绪） */
static void s1_first_init_full_sequence(void)
{
    RtcTimebase_t tb;
    FakeRtc_t f;
    RtcTimebase_Status_t st;
    RtcTimebase_Io_t io = g_fake_io;              /* 复制并绑定脚本账本 */

    memset(&tb, 0, sizeof(tb));
    memset(&f, 0, sizeof(f));
    io.self = &f;
    f.lse_ready_at_call = 1u;                    /* LSE 立即就绪 */
    f.cnt_high = 0x0001u;
    f.cnt_low = 0x8000u;

    printf("s1: first init full sequence (signature mismatch)\n");

    st = RtcTimebase_Init(&tb, &io);
    CHECK_EQ_DESC("Init fresh board -> OK", st, RTC_TB_OK);
    CHECK_EQ_DESC("IsReady -> 1", RtcTimebase_IsReady(&tb), 1);

    /* 完整序列顺序（设计 §6.2 +*/
    CHECK_DESC("pwr_bkp_enable runs first",
               idx_is(&f, 0u, OP_PWR_BKP_ENABLE, 0));
    CHECK_DESC("backup_domain_reset before lse_on",
               a_before_b(&f, 0u, OP_BKP_DOMAIN_RESET, OP_LSE_ON));
    CHECK_DESC("lse_on before rtc_select_lse",
               a_before_b(&f, 0u, OP_LSE_ON, OP_RTC_SEL_LSE));
    CHECK_DESC("rtc_select_lse before set_prescaler",
               a_before_b(&f, 0u, OP_RTC_SEL_LSE, OP_SET_PRL));
    CHECK_DESC("set_prescaler before wait_rtc_sync",
               a_before_b(&f, 0u, OP_SET_PRL, OP_WAIT_RTC_SYNC));
    CHECK_DESC("wait_rtc_sync before alarm_exti_nvic_init",
               a_before_b(&f, 0u, OP_WAIT_RTC_SYNC, OP_ALARM_EXTI_INIT));
    CHECK_DESC("signature write is the last init op",
               idx_is(&f, 0u, OP_WRITE_BKP,
                      (int)f.order_len - 1));

    /* prescaler=31（32768/1024-1，设计 §6.2） */
    CHECK_EQ_DESC("prescaler value is 31 (1024 Hz)", f.prl_value, RTC_TB_PRL);
    CHECK_EQ_DESC("RTC_TB_PRL is 31", RTC_TB_PRL, 31u);

    /* 签名写入 DR10（与 BKP_DR2 升级请求分离） */
    CHECK_EQ_DESC("signature written to BKP_DR10",
                  f.bkp_dr[RTC_TB_SIGNATURE_DR], RTC_TB_LAYOUT_SIGNATURE);
    CHECK_EQ_DESC("signature DR index is 10", RTC_TB_SIGNATURE_DR, 10u);
    CHECK_EQ_DESC("backup domain reset happened once",
                  f.bkp_reset_done, 1u);
    CHECK_EQ_DESC("no wait_poll consumed (LSE ready at 1st poll)",
                  f.count[OP_WAIT_POLL], 0u);
}

/* s2: 快速路径（签名匹配 + LSE 运行） */
static void s2_fast_path(void)
{
    RtcTimebase_t tb;
    FakeRtc_t f;
    RtcTimebase_Status_t st;
    RtcTimebase_Io_t io = g_fake_io;

    memset(&tb, 0, sizeof(tb));
    memset(&f, 0, sizeof(f));
    io.self = &f;
    f.bkp_dr[RTC_TB_SIGNATURE_DR] = RTC_TB_LAYOUT_SIGNATURE;
    f.lse_ready_at_call = 1u;

    printf("s2: fast path (signature match + LSE running)\n");

    st = RtcTimebase_Init(&tb, &io);
    CHECK_EQ_DESC("Init warm board -> OK", st, RTC_TB_OK);
    CHECK_EQ_DESC("IsReady -> 1", RtcTimebase_IsReady(&tb), 1);

    /* 不重置备份域：RTC 计数与 BKP_DR2 升级请求得以保留（设计 §6.2） */
    CHECK_EQ_DESC("no backup domain reset on fast path",
                  f.count[OP_BKP_DOMAIN_RESET], 0u);
    CHECK_EQ_DESC("no LSE restart on fast path", f.count[OP_LSE_ON], 0u);
    CHECK_EQ_DESC("no prescaler rewrite on fast path",
                  f.count[OP_SET_PRL], 0u);
    CHECK_EQ_DESC("no signature rewrite on fast path",
                  f.count[OP_WRITE_BKP], 0u);

    /* 快速路径只做：使能写权 -> 读签名 -> LSE 就绪查询 -> 同步 -> Alarm 初始化 */
    CHECK_EQ_DESC("fast path sequence is exactly 5 ops",
                  f.order_len, 5u);
    CHECK_DESC("fast path order: pwr->read_bkp->lse_ready->sync->alarm_init",
               idx_is(&f, 0u, OP_PWR_BKP_ENABLE, 0) &&
               idx_is(&f, 0u, OP_READ_BKP, 1) &&
               idx_is(&f, 0u, OP_LSE_READY, 2) &&
               idx_is(&f, 0u, OP_WAIT_RTC_SYNC, 3) &&
               idx_is(&f, 0u, OP_ALARM_EXTI_INIT, 4));
    CHECK_EQ_DESC("backup domain reset flag stays clear",
                  f.bkp_reset_done, 0u);
}

/* s3: LSE 超时路径 */
static void s3_lse_timeout(void)
{
    RtcTimebase_t tb;
    FakeRtc_t f;
    RtcTimebase_Status_t st;
    RtcTimebase_Io_t io = g_fake_io;
    u32 out = 0u;

    memset(&tb, 0, sizeof(tb));
    memset(&f, 0, sizeof(f));
    io.self = &f;
    f.lse_ready_at_call = 0u;                    /* LSE 永不就绪 */

    printf("s3: LSE timeout degradation signal\n");

    st = RtcTimebase_Init(&tb, &io);
    CHECK_EQ_DESC("Init with dead LSE -> ERR_LSE_TIMEOUT",
                  st, RTC_TB_ERR_LSE_TIMEOUT);
    CHECK_EQ_DESC("IsReady -> 0 (degradation input for policy)",
                  RtcTimebase_IsReady(&tb), 0);

    /* 超时后不得选源/设分频/写签名（下次复位可重试，设计 §6.2） */
    CHECK_EQ_DESC("no rtc_select_lse after timeout",
                  f.count[OP_RTC_SEL_LSE], 0u);
    CHECK_EQ_DESC("no prescaler write after timeout",
                  f.count[OP_SET_PRL], 0u);
    CHECK_EQ_DESC("no signature write after timeout",
                  f.count[OP_WRITE_BKP], 0u);
    CHECK_EQ_DESC("no alarm EXTI init after timeout",
                  f.count[OP_ALARM_EXTI_INIT], 0u);

    /* 有界等待：恰好 RTC_TB_LSE_TIMEOUT_MS 次轮询 */
    CHECK_EQ_DESC("wait_poll bounded to RTC_TB_LSE_TIMEOUT_MS",
                  f.count[OP_WAIT_POLL], RTC_TB_LSE_TIMEOUT_MS);
    CHECK_EQ_DESC("lse_ready polled exactly timeout times",
                  f.lse_ready_calls, RTC_TB_LSE_TIMEOUT_MS);

    /* NOT_READY 守卫：Init 失败后读取/闹钟 API 全部拒绝 */
    CHECK_EQ_DESC("ReadCounterStable after timeout -> ERR_NOT_READY",
                  RtcTimebase_ReadCounterStable(&tb, &out), RTC_TB_ERR_NOT_READY);
    CHECK_EQ_DESC("SetAlarm after timeout -> ERR_NOT_READY",
                  RtcTimebase_SetAlarm(&tb, 1u), RTC_TB_ERR_NOT_READY);
    CHECK_EQ_DESC("CancelAlarm after timeout -> ERR_NOT_READY",
                  RtcTimebase_CancelAlarm(&tb), RTC_TB_ERR_NOT_READY);
}

/* s3b: LSE 超时边界（off-by-one） */
static void s3b_lse_timeout_boundary(void)
{
    RtcTimebase_t tb;
    FakeRtc_t f;
    RtcTimebase_Status_t st;
    RtcTimebase_Io_t io = g_fake_io;

    io.self = &f;
    printf("s3b: LSE timeout boundary (off-by-one)\n");

    /* 恰好在第 TIMEOUT 次轮询就绪 -> 成功（poll=TIMEOUT-1 break） */
    memset(&tb, 0, sizeof(tb));
    memset(&f, 0, sizeof(f));
    f.lse_ready_at_call = RTC_TB_LSE_TIMEOUT_MS;
    st = RtcTimebase_Init(&tb, &io);
    CHECK_EQ_DESC("ready at last poll -> OK", st, RTC_TB_OK);
    CHECK_EQ_DESC("wait_poll == TIMEOUT-1 on last-poll success",
                  f.count[OP_WAIT_POLL], RTC_TB_LSE_TIMEOUT_MS - 1u);

    /* 第 TIMEOUT+1 次才就绪 -> 超时（循环上限） */
    memset(&tb, 0, sizeof(tb));
    memset(&f, 0, sizeof(f));
    f.lse_ready_at_call = RTC_TB_LSE_TIMEOUT_MS + 1u;
    st = RtcTimebase_Init(&tb, &io);
    CHECK_EQ_DESC("ready one poll too late -> ERR_LSE_TIMEOUT",
                  st, RTC_TB_ERR_LSE_TIMEOUT);
    CHECK_EQ_DESC("wait_poll == TIMEOUT on failure",
                  f.count[OP_WAIT_POLL], RTC_TB_LSE_TIMEOUT_MS);
}

/* s3c: 签名匹配但 LSE 停振 -> 走完整重初始化 */
static void s3c_signature_match_lse_dead(void)
{
    RtcTimebase_t tb;
    FakeRtc_t f;
    RtcTimebase_Status_t st;
    RtcTimebase_Io_t io = g_fake_io;

    memset(&tb, 0, sizeof(tb));
    memset(&f, 0, sizeof(f));
    io.self = &f;
    f.bkp_dr[RTC_TB_SIGNATURE_DR] = RTC_TB_LAYOUT_SIGNATURE;
    /* 快速路径的就绪查询（第 1 次）返回 0，重初始化路径第 2 次起 1 */
    f.lse_ready_at_call = 2u;

    printf("s3c: signature match but LSE dead -> full re-init\n");

    st = RtcTimebase_Init(&tb, &io);
    CHECK_EQ_DESC("Init with stopped LSE -> OK", st, RTC_TB_OK);
    CHECK_EQ_DESC("backup domain reset executed", f.bkp_reset_done, 1u);
    CHECK_EQ_DESC("full re-init rewrote signature",
                  f.bkp_dr[RTC_TB_SIGNATURE_DR], RTC_TB_LAYOUT_SIGNATURE);
    CHECK_EQ_DESC("prescaler reprogrammed", f.prl_value, RTC_TB_PRL);
}

/* s4: high-low-high 稳定读取 */
static void s4_stable_read(void)
{
    RtcTimebase_t tb;
    FakeRtc_t f;
    RtcTimebase_Io_t io = g_fake_io;
    u32 out = 0u;

    printf("s4: high-low-high stable counter read\n");

    /* 4a: 干净读取，一次成功 */
    memset(&tb, 0, sizeof(tb));
    memset(&f, 0, sizeof(f));
    io.self = &f;
    f.bkp_dr[RTC_TB_SIGNATURE_DR] = RTC_TB_LAYOUT_SIGNATURE;
    f.lse_ready_at_call = 1u;
    f.cnt_high = 0x0001u;
    f.cnt_low = 0x8000u;
    (void)RtcTimebase_Init(&tb, &io);
    f.order_len = 0u;                            /* 清序号，只看读取操作 */
    memset(f.count, 0, sizeof(f.count));

    CHECK_EQ_DESC("clean read -> OK",
                  RtcTimebase_ReadCounterStable(&tb, &out), RTC_TB_OK);
    CHECK_EQ_DESC("clean read value", out, 0x00018000u);
    CHECK_EQ_DESC("clean read: exactly 2 high + 1 low reads",
                  f.count[OP_READ_CNT_H] == 2u && f.count[OP_READ_CNT_L] == 1u,
                  1);

    /* 4b: 一轮撕裂（低字回绕进位模拟）后第 2 轮稳定 -> 成功 */
    memset(&f, 0, sizeof(f));
    f.tear_rounds = 1u;
    f.cnt_high = 0x0001u;
    f.cnt_low = 0xFFF0u;
    CHECK_EQ_DESC("one torn round -> OK on retry",
                  RtcTimebase_ReadCounterStable(&tb, &out), RTC_TB_OK);
    /* 撕裂注入：稳定值 = cnt_high + 2*tear_rounds */
    CHECK_EQ_DESC("retried read value uses stable high word",
                  out, ((0x0001u + 2u) << 16) | 0xFFF0u);
    CHECK_EQ_DESC("torn round consumed 4 high reads (2 rounds)",
                  f.count[OP_READ_CNT_H], 4u);

    /* 4c: 持续撕裂 -> 有界重试用尽后报错 */
    memset(&f, 0, sizeof(f));
    f.tear_rounds = RTC_TB_STABLE_READ_MAX_RETRY;    /* 每轮都撕裂 */
    CHECK_EQ_DESC("permanently torn -> ERR_READ_UNSTABLE",
                  RtcTimebase_ReadCounterStable(&tb, &out),
                  RTC_TB_ERR_READ_UNSTABLE);
    CHECK_EQ_DESC("bounded retries: 2*MAX_RETRY high reads",
                  f.count[OP_READ_CNT_H], 2u * RTC_TB_STABLE_READ_MAX_RETRY);

    /* 4d: 输出指针 NULL */
    CHECK_EQ_DESC("NULL out -> ERR_PARAM",
                  RtcTimebase_ReadCounterStable(&tb, NULL), RTC_TB_ERR_PARAM);
}

/* s5: SetAlarm / CancelAlarm 顺序契约 */
static void s5_alarm_ordering(void)
{
    RtcTimebase_t tb;
    FakeRtc_t f;
    RtcTimebase_Io_t io = g_fake_io;

    memset(&tb, 0, sizeof(tb));
    memset(&f, 0, sizeof(f));
    io.self = &f;
    f.bkp_dr[RTC_TB_SIGNATURE_DR] = RTC_TB_LAYOUT_SIGNATURE;
    f.lse_ready_at_call = 1u;
    (void)RtcTimebase_Init(&tb, &io);

    printf("s5: SetAlarm / CancelAlarm ordering (design 6.6)\n");

    /* SetAlarm：先清旧 ALRF/EXTI17 pending，再写新闹钟（进入顺序 3） */
    f.order_len = 0u;
    memset(f.count, 0, sizeof(f.count));
    CHECK_EQ_DESC("SetAlarm -> OK",
                  RtcTimebase_SetAlarm(&tb, 0x00023456u), RTC_TB_OK);
    CHECK_DESC("SetAlarm clears old flags BEFORE setting alarm",
               a_before_b(&f, 0u, OP_CLEAR_ALARM_FLAGS, OP_SET_ALARM));
    CHECK_EQ_DESC("alarm counter forwarded", f.alarm_value, 0x00023456u);

    /* CancelAlarm：先取消闹钟源，再清 ALRF/EXTI17（退出顺序 4） */
    f.order_len = 0u;
    memset(f.count, 0, sizeof(f.count));
    CHECK_EQ_DESC("CancelAlarm -> OK",
                  RtcTimebase_CancelAlarm(&tb), RTC_TB_OK);
    CHECK_DESC("CancelAlarm cancels source BEFORE clearing flags",
               a_before_b(&f, 0u, OP_CANCEL_ALARM, OP_CLEAR_ALARM_FLAGS));
    CHECK_EQ_DESC("cancel executed once", f.cancel_count, 1u);
}

int main(void)
{
    printf("rtc-timebase mock (BOARD_C8_LSE full path)\n");
    s0_param_defense();
    s1_first_init_full_sequence();
    s2_fast_path();
    s3_lse_timeout();
    s3b_lse_timeout_boundary();
    s3c_signature_match_lse_dead();
    s4_stable_read();
    s5_alarm_ordering();
    printf("rtc-timebase mock: %d checks, %d failed\n", g_total, g_failed);
    return (g_failed == 0) ? 0 : 1;
}

#else /* BOARD_PROFILE_PENDING（或 NO_LSE）：零硬件访问 stub */

/*-----------------------------------------------------------
 * stub 板型测试：所有 API 返回 NOT_SUPPORTED 且零 IO 调用
 *----------------------------------------------------------*/

static u32 g_io_calls = 0;

static void any_io(void *self)
{
    (void)self;
    g_io_calls++;
}

static u32 any_read_bkp(void *self, u32 dr)
{
    (void)self;
    (void)dr;
    g_io_calls++;
    return 0u;
}

static u16 any_read16(void *self)
{
    (void)self;
    g_io_calls++;
    return 0u;
}

static u8 any_ready(void *self)
{
    (void)self;
    g_io_calls++;
    return 0u;
}

/* 带参回调的签名适配包装（杜绝 -Wincompatible-pointer-types） */
static void any_write_bkp(void *self, u32 dr, u32 value)
{
    (void)self;
    (void)dr;
    (void)value;
    g_io_calls++;
}

static void any_set_prescaler(void *self, u32 prl)
{
    (void)self;
    (void)prl;
    g_io_calls++;
}

static void any_set_alarm(void *self, u32 counter)
{
    (void)self;
    (void)counter;
    g_io_calls++;
}

/* 板型门禁在编译期已选定 stub 分支：IO 表不可达（链接集证明），
 * 但仍构造完整表以证明"即使注入也绝不被调用" */
static const RtcTimebase_Io_t g_fake_io = {
    NULL,
    any_io,            /* pwr_bkp_enable */
    any_read_bkp,      /* read_bkp_dr */
    any_write_bkp,     /* write_bkp_dr */
    any_io,            /* backup_domain_reset */
    any_io,            /* lse_on */
    any_ready,         /* lse_ready */
    any_io,            /* rtc_select_lse */
    any_set_prescaler, /* set_prescaler */
    any_io,            /* wait_rtc_sync */
    any_io,            /* alarm_exti_nvic_init */
    any_read16,        /* read_cnt_high */
    any_read16,        /* read_cnt_low */
    any_io,            /* clear_alarm_flags */
    any_set_alarm,     /* set_alarm */
    any_io,            /* cancel_alarm */
    any_io             /* wait_poll */
};

int main(void)
{
    RtcTimebase_t tb;
    u32 out = 0xDEADBEEFu;

    printf("rtc-timebase mock (NO_LSE/PENDING board: zero-hardware stub)\n");

    /* Init 返回 NOT_SUPPORTED，ready 恒 0 */
    CHECK_EQ_DESC("Init on NO_LSE/PENDING board -> NOT_SUPPORTED",
                  RtcTimebase_Init(&tb, &g_fake_io), RTC_TB_NOT_SUPPORTED);
    CHECK_EQ_DESC("IsReady -> 0", RtcTimebase_IsReady(&tb), 0);

    /* 所有运行期 API 返回 NOT_SUPPORTED */
    CHECK_EQ_DESC("ReadCounterStable -> NOT_SUPPORTED",
                  RtcTimebase_ReadCounterStable(&tb, &out),
                  RTC_TB_NOT_SUPPORTED);
    CHECK_EQ_DESC("SetAlarm -> NOT_SUPPORTED",
                  RtcTimebase_SetAlarm(&tb, 1u), RTC_TB_NOT_SUPPORTED);
    CHECK_EQ_DESC("CancelAlarm -> NOT_SUPPORTED",
                  RtcTimebase_CancelAlarm(&tb), RTC_TB_NOT_SUPPORTED);

    /* 输出确定性置零 */
    CHECK_EQ_DESC("out zeroed deterministically", out, 0u);

    /* 零硬件访问证明：任何 IO 回调都未被触发 */
    CHECK_EQ_DESC("zero IO callbacks fired (no PC14/PC15, no BDCR)",
                  g_io_calls, 0u);

    /* NULL 防御仍然有效 */
    CHECK_EQ_DESC("Init(NULL tb) -> ERR_PARAM",
                  RtcTimebase_Init(NULL, &g_fake_io), RTC_TB_ERR_PARAM);

    printf("rtc-timebase mock: %d checks, %d failed\n", g_total, g_failed);
    return (g_failed == 0) ? 0 : 1;
}

#endif /* BOARD_C8_LSE */
