/*
 * RTC 时间基准实现
 *
 * 本文件按 CONFIG/board_profile.h 板型门禁二选一编译：
 *   - BOARD_C8_LSE：完整 IO 注入实现（无任何直接寄存器访问）；
 *   - 其余板型（NO_LSE / PENDING）：零硬件访问 stub——：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#include "rtc_timebase.h"

#if (BOARD_C8_LSE != 0)

/*-----------------------------------------------------------
 * 完整实现（仅 LSE 板型链接）
 *----------------------------------------------------------*/

static RtcTimebase_Status_t rtb_init_impl(RtcTimebase_t *tb)
{
    u32 sig;
    u32 poll;

    /* 1. PWR/BKP 时钟与后备域写权 */
    tb->io->pwr_bkp_enable(tb->io->self);

    /* 2. 布局签名检查：匹配且 LSE 仍运行 -> 快速路径（不动备份域，
     *    保住 RTC 计数与 BKP_DR2 内容；设计 §6.2）*/
    sig = tb->io->read_bkp_dr(tb->io->self, RTC_TB_SIGNATURE_DR);
    if ((sig == RTC_TB_LAYOUT_SIGNATURE) &&
        (tb->io->lse_ready(tb->io->self) != 0u)) {
        tb->io->wait_rtc_sync(tb->io->self);
        tb->io->alarm_exti_nvic_init(tb->io->self);
        tb->ready = 1u;
        tb->status = RTC_TB_OK;
        return RTC_TB_OK;
    }

    /* 3. 签名不匹配或 LSE 停振：复位备份域后完整初始化
     *（App 侧执行；Boot 已先消费 BKP_DR2 升级请求，见头文件纪律）*/
    tb->io->backup_domain_reset(tb->io->self);
    tb->io->lse_on(tb->io->self);

    /* LSE 启动等待（有界；每次轮询间隔由 IO wait_poll 提供时基）*/
    for (poll = 0u; poll < (u32)RTC_TB_LSE_TIMEOUT_MS; poll++) {
        if (tb->io->lse_ready(tb->io->self) != 0u) {
            break;
        }
        tb->io->wait_poll(tb->io->self);
    }
    if (poll >= (u32)RTC_TB_LSE_TIMEOUT_MS) {
        /* LSE 超时：不选源、不设分频、不写签名——ready 保持 0，
         * 调用方按设计 §6.2 降级（EXTI-only / 浅睡），下次复位重试 */
        tb->ready = 0u;
        tb->status = RTC_TB_ERR_LSE_TIMEOUT;
        return RTC_TB_ERR_LSE_TIMEOUT;
    }

    tb->io->rtc_select_lse(tb->io->self);
    tb->io->set_prescaler(tb->io->self, RTC_TB_PRL);
    tb->io->wait_rtc_sync(tb->io->self);
    tb->io->alarm_exti_nvic_init(tb->io->self);
    tb->io->write_bkp_dr(tb->io->self, RTC_TB_SIGNATURE_DR,
                         RTC_TB_LAYOUT_SIGNATURE);
    tb->ready = 1u;
    tb->status = RTC_TB_OK;
    return RTC_TB_OK;
}

RtcTimebase_Status_t RtcTimebase_Init(RtcTimebase_t *tb,
                                      const RtcTimebase_Io_t *io)
{
    if ((tb == NULL) || (io == NULL)) {
        if (tb != NULL) {
            tb->status = RTC_TB_ERR_PARAM;
        }
        return RTC_TB_ERR_PARAM;
    }
    tb->io = io;
    tb->ready = 0u;
    return rtb_init_impl(tb);
}

u8 RtcTimebase_IsReady(const RtcTimebase_t *tb)
{
    return ((tb != NULL) && (tb->ready != 0u)) ? 1u : 0u;
}

RtcTimebase_Status_t RtcTimebase_ReadCounterStable(RtcTimebase_t *tb,
                                                    u32 *out)
{
    u32 retry;

    if ((tb == NULL) || (out == NULL)) {
        if (tb != NULL) {
            tb->status = RTC_TB_ERR_PARAM;
        }
        return RTC_TB_ERR_PARAM;
    }
    if (tb->ready == 0u) {
        tb->status = RTC_TB_ERR_NOT_READY;
        return RTC_TB_ERR_NOT_READY;
    }

    /* high-low-high：低 16bit 回绕会造成两次读高字不一致，
     * 检测到撕裂即整体重读（设计 §6.2*/
    for (retry = 0u; retry < (u32)RTC_TB_STABLE_READ_MAX_RETRY; retry++) {
        u16 h1 = tb->io->read_cnt_high(tb->io->self);
        u16 l  = tb->io->read_cnt_low(tb->io->self);
        u16 h2 = tb->io->read_cnt_high(tb->io->self);

        if (h1 == h2) {
            *out = ((u32)h2 << 16) | (u32)l;
            tb->status = RTC_TB_OK;
            return RTC_TB_OK;
        }
    }

    tb->status = RTC_TB_ERR_READ_UNSTABLE;
    return RTC_TB_ERR_READ_UNSTABLE;
}

RtcTimebase_Status_t RtcTimebase_SetAlarm(RtcTimebase_t *tb, u32 counter)
{
    if (tb == NULL) {
        return RTC_TB_ERR_PARAM;
    }
    if (tb->ready == 0u) {
        tb->status = RTC_TB_ERR_NOT_READY;
        return RTC_TB_ERR_NOT_READY;
    }

    /* 先清旧 ALRF/EXTI17 pending，再写新闹钟（设计 §6.6 进入顺序 3：
     * 残留 pending 会让 WFI 立即返回）*/
    tb->io->clear_alarm_flags(tb->io->self);
    tb->io->set_alarm(tb->io->self, counter);
    tb->status = RTC_TB_OK;
    return RTC_TB_OK;
}

RtcTimebase_Status_t RtcTimebase_CancelAlarm(RtcTimebase_t *tb)
{
    if (tb == NULL) {
        return RTC_TB_ERR_PARAM;
    }
    if (tb->ready == 0u) {
        tb->status = RTC_TB_ERR_NOT_READY;
        return RTC_TB_ERR_NOT_READY;
    }

    /* 先取消闹钟源，再清 ALRF/EXTI17（设计 §6.6 退出顺序 4；
     * 按键 EXTI pending 不在此清——留给 ISR 产生按键通知，Task 13）*/
    tb->io->cancel_alarm(tb->io->self);
    tb->io->clear_alarm_flags(tb->io->self);
    tb->status = RTC_TB_OK;
    return RTC_TB_OK;
}

#else

/*-----------------------------------------------------------
 * 零硬件访问 stub（BOARD_C8_NO_LSE / BOARD_PROFILE_PENDING）
 *
 *：不触碰 PC14/PC15 与 backup-domain RTC 源。
 * stub 分支没有任何 tb->io-> 调用——host mock 以 IO 调用计数==0
 * 证明。
 * 上层无需板型 #if：NOT_SUPPORTED / ready==0 使决策层自然避开
 * RTC 深睡路径（LpPolicy_Decide 的 rtc_ready 输入恒 0）。
 *----------------------------------------------------------*/

RtcTimebase_Status_t RtcTimebase_Init(RtcTimebase_t *tb,
                                      const RtcTimebase_Io_t *io)
{
    (void)io;
    if (tb == NULL) {
        return RTC_TB_ERR_PARAM;
    }
    tb->io = io;
    tb->ready = 0u;                       /* 恒不就绪 */
    tb->status = RTC_TB_NOT_SUPPORTED;
    return RTC_TB_NOT_SUPPORTED;
}

u8 RtcTimebase_IsReady(const RtcTimebase_t *tb)
{
    (void)tb;
    return 0u;
}

RtcTimebase_Status_t RtcTimebase_ReadCounterStable(RtcTimebase_t *tb,
                                                    u32 *out)
{
    if (tb == NULL) {
        return RTC_TB_ERR_PARAM;
    }
    if (out != NULL) {
        *out = 0u;                        /* 输出确定性置零 */
    }
    tb->status = RTC_TB_NOT_SUPPORTED;
    return RTC_TB_NOT_SUPPORTED;
}

RtcTimebase_Status_t RtcTimebase_SetAlarm(RtcTimebase_t *tb, u32 counter)
{
    (void)counter;
    if (tb == NULL) {
        return RTC_TB_ERR_PARAM;
    }
    tb->status = RTC_TB_NOT_SUPPORTED;
    return RTC_TB_NOT_SUPPORTED;
}

RtcTimebase_Status_t RtcTimebase_CancelAlarm(RtcTimebase_t *tb)
{
    if (tb == NULL) {
        return RTC_TB_ERR_PARAM;
    }
    tb->status = RTC_TB_NOT_SUPPORTED;
    return RTC_TB_NOT_SUPPORTED;
}

#endif /* BOARD_C8_LSE 板型门禁 */

