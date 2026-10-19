/*
 * RTC 时间基准
 *
 * 职责（仅 BOARD_C8_LSE 板型，CONFIG/board_profile.h 编译期门禁）：
 *   1. 初次初始化：PWR/BKP 写权 -> 布局签名检查 ->（签名不匹配时）
 *      backup domain 复位 -> LSE 启动（带超时）-> RTCSEL=LSE ->
 *      prescaler=31（1024 Hz）-> RTC 同步 -> 写回签名；
 *      签名匹配且 LSE 运行时只做同步，不重置备份域
 *（避免每次复位丢失 RTC 计数与 BKP_DR2 升级请求）；
 *   2. high-low-high 稳定计数读取：防 RTC 32 bit 计数器低 16 bit
 *      回绕时高低字撕裂（设计 §6.2）；
 *   3. RTC Alarm 管理：SetAlarm 先清旧 ALRF/EXTI17 pending 再写
 *      新闹钟值（设计 §6.6 进入顺序 3）；CancelAlarm 先取消闹钟
 *      再清 ALRF/EXTI17（设计 §6.6 退出顺序 4）。
 *
 * 板型契约（board_profile.h Task 1 门禁）：
 *   - BOARD_C8_LSE：编译完整实现（本文件 .c 的 #if 分支）；
 *   - BOARD_C8_NO_LSE / BOARD_PROFILE_PENDING：编译为零硬件访问 stub——
 *     所有函数返回 RTC_TB_NOT_SUPPORTED / 0，不触碰 PC14/PC15 与
 *     backup domain RTC 源。上层无需 #if 包裹调用：非 LSE 板型
 *     Init 返回 NOT_SUPPORTED -> rtc_ready=0 -> 决策层自然避开 RTC 路径。
 *
 * 备份域纪律：
 *   - 本模块仅链接进 APP（Boot 不包含 rtc_timebase.c）；
 *   - Boot 复位后先读 BKP_DR2 升级请求再跳转 APP，因此 APP 侧
 *     "签名不匹配 -> backup domain 复位" 不会吞掉未消费的升级请求；
 *   - 签名常驻 BKP_DR10（与 BKP_DR2 请求寄存器分离）。
 *
 * 纯逻辑 + IO 注入（项目惯例：寄存器访问封装以便故障注入测试）：
 *   本模块不直接访问任何寄存器；目标侧适配（PWR/BKP/RCC/RTC/EXTI）
 *   由 Task 15 wrapper 提供，host mock 注入脚本化 IO。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#ifndef __RTC_TIMEBASE_H__
#define __RTC_TIMEBASE_H__

#include <stddef.h>            /* NULL — ARMCC 与 host gcc 通用 */
#include "sys.h"               /* u8/u16/u32 */
#include "board_profile.h"     /* BOARD_C8_LSE 板型门禁 */
#include "lowpower_policy.h"   /* LP_RTC_HZ（1024 Hz 设计配置单一来源） */

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * 状态码（保留在 ctx->status 直至下一次操作）
 *----------------------------------------------------------*/
typedef enum {
    RTC_TB_OK = 0,
    RTC_TB_ERR_PARAM,        /* NULL 参数 */
    RTC_TB_ERR_LSE_TIMEOUT,  /* LSE 未在超时窗口内就绪（降级依据，设计 §6.2） */
    RTC_TB_ERR_NOT_READY,   /* Init 未成功即调用读取/Alarm API */
    RTC_TB_ERR_READ_UNSTABLE,/* 稳定读取重试用尽（高低字持续撕裂） */
    RTC_TB_NOT_SUPPORTED     /* 非 LSE 板型零硬件 stub */
} RtcTimebase_Status_t;

/*-----------------------------------------------------------
 * 策略常量
 *
 * RTC_TB_LSE_HZ            ：LSE 标称 32768 Hz（设计 §6.2）；
 * RTC_TB_PRL               ：prescaler 重装值 = 32768/LP_RTC_HZ - 1 = 31，
 *                             得 1024 Hz 自由计数（与 lowpower_math 换算一致）；
 * RTC_TB_LSE_TIMEOUT_MS    ：LSE 启动等待上限（策略值：无板测证据下的
 *                             保守值；每次轮询间隔由 IO wait_poll 决定）；
 * RTC_TB_STABLE_READ_MAX_RETRY：high-low-high 读取重试上限（正常 1 次
 *                             即可，回绕撕裂概率极低）。
 *----------------------------------------------------------*/
#define RTC_TB_LSE_HZ                 32768uL
#define RTC_TB_PRL                    (RTC_TB_LSE_HZ / LP_RTC_HZ - 1uL)
#define RTC_TB_LSE_TIMEOUT_MS         2000u
#define RTC_TB_STABLE_READ_MAX_RETRY  4u

/*-----------------------------------------------------------
 * 备份域布局签名（BKP_DR10；与 BKP_DR2 升级请求分离）
 * 签名值变更 == 声明备份域布局不兼容（要求下次复位重初始化 RTC）
 *----------------------------------------------------------*/
#define RTC_TB_SIGNATURE_DR           10u
#define RTC_TB_LAYOUT_SIGNATURE       0x52544331uL   /* "RTC1" */

/*-----------------------------------------------------------
 * RTC 后端接口（依赖注入）：
 *   host 测试侧注入脚本化 mock（含低字回绕/超时注入）；
 *   目标侧适配（Task 15 wrapper）封装 SPL：
 *     pwr_bkp_enable        : PWR/BKP 时钟 + PWR_BackupAccessCmd(ENABLE)
 *     read/write_bkp_dr     : BKP_ReadBackupRegister / Write...
 *     backup_domain_reset   : RCC->BDCR |= BDRST 后清零（脉冲）
 *     lse_on / lse_ready    : RCC_BDCR LSEON / LSERDY
 *     rtc_select_lse        : RTCSEL=LSE（仅备份域复位后允许）
 *     set_prescaler         : CNF 进出 + PRLH/PRLL 写 + RTOFF 等待
 *     wait_rtc_sync         : RSF 清零后等待置位（同步 32bit 影子寄存器）
 *     alarm_exti_nvic_init  : EXTI17 上升沿 + RTCAlarm_IRQn NVIC 使能（幂等）
 *     read_cnt_high/low     : RTC_CNTRH / RTC_CNTRL（16bit 各一次读）
 *     clear_alarm_flags    : 清 ALRF + EXTI17 pending（EXTI_PR 写 1）
 *     set_alarm / cancel    : CNF/RTOFF 事务写/取消 ALRH/ALRL
 *     wait_poll             : LSE 等待轮询间隔（目标侧约 1ms；host 为空）
 *----------------------------------------------------------*/
typedef struct {
    void *self;
    void (*pwr_bkp_enable)(void *self);
    u32  (*read_bkp_dr)(void *self, u32 dr_index);
    void (*write_bkp_dr)(void *self, u32 dr_index, u32 value);
    void (*backup_domain_reset)(void *self);
    void (*lse_on)(void *self);
    u8   (*lse_ready)(void *self);
    void (*rtc_select_lse)(void *self);
    void (*set_prescaler)(void *self, u32 prl);
    void (*wait_rtc_sync)(void *self);
    void (*alarm_exti_nvic_init)(void *self);
    u16  (*read_cnt_high)(void *self);
    u16  (*read_cnt_low)(void *self);
    void (*clear_alarm_flags)(void *self);
    void (*set_alarm)(void *self, u32 counter);
    void (*cancel_alarm)(void *self);
    void (*wait_poll)(void *self);
} RtcTimebase_Io_t;

/*-----------------------------------------------------------
 * 时间基准上下文
 *----------------------------------------------------------*/
typedef struct {
    const RtcTimebase_Io_t *io;
    RtcTimebase_Status_t status;   /* 最近一次操作结果 */
    u8 ready;                      /* Init 成功后置 1 */
} RtcTimebase_t;

/*-----------------------------------------------------------
 * 初始化（板型门禁内为完整实现，否则零硬件 stub）
 *
 * 完整路径序列（设计 §6.2 +：
 *   1. pwr_bkp_enable；
 *   2. 读 BKP 签名：匹配且 LSE 运行 -> 仅 wait_rtc_sync +
 *      alarm_exti_nvic_init -> OK（不重置备份域）；
 *   3. 否则（签名不匹配或 LSE 停振）：backup_domain_reset ->
 *      lse_on -> 轮询 lse_ready（超时 RTC_TB_LSE_TIMEOUT_MS 次则
 *      返回 RTC_TB_ERR_LSE_TIMEOUT，不写签名，供下次复位重试）->
 *      rtc_select_lse -> set_prescaler(PRL) -> wait_rtc_sync ->
 *      alarm_exti_nvic_init -> 写签名 -> ready=1。
 *
 * LSE 超时后的降级决策（EXTI-only 或浅睡）由调用方依据
 * rtc_ready==0 + 本返回值组装快照后交给 LpPolicy_Decide（Task 11）。
 *----------------------------------------------------------*/
RtcTimebase_Status_t RtcTimebase_Init(RtcTimebase_t *tb,
                                      const RtcTimebase_Io_t *io);

/*-----------------------------------------------------------
 * 就绪查询：Init 成功且板型为 LSE 时返回 1
 *----------------------------------------------------------*/
u8 RtcTimebase_IsReady(const RtcTimebase_t *tb);

/*-----------------------------------------------------------
 * high-low-high 稳定读取（设计 §6.2
 *
 *   h1 = CNT_H; l = CNT_L; h2 = CNT_H;
 *   h1 == h2 -> (h2<<16)|l；否则（低字回绕撕裂）重试，
 *   至多 RTC_TB_STABLE_READ_MAX_RETRY 次，仍撕裂则
 *   RTC_TB_ERR_READ_UNSTABLE（有界，不无限循环）。
 *
 * 32bit 回绕安全：返回原始计数，差值换算由 lowpower_math 承担。
 *----------------------------------------------------------*/
RtcTimebase_Status_t RtcTimebase_ReadCounterStable(RtcTimebase_t *tb,
                                                    u32 *out);

/*-----------------------------------------------------------
 * 设置未来闹钟（绝对计数；预算换算由调用方经 lowpower_math 完成）
 *
 * 顺序契约（设计 §6.6 进入顺序 3）：先 clear_alarm_flags（清旧
 * ALRF/EXTI17 pending，防上一次残留让 WFI 立即返回），再 set_alarm。
 *----------------------------------------------------------*/
RtcTimebase_Status_t RtcTimebase_SetAlarm(RtcTimebase_t *tb, u32 counter);

/*-----------------------------------------------------------
 * 取消闹钟（STOP 退出路径调用；此时应已读取 RTC end）
 *
 * 顺序契约（设计 §6.6 退出顺序 4）：先 cancel_alarm（关闹钟源），
 * 再 clear_alarm_flags（清 ALRF + EXTI17 pending）。
 * 按键 EXTI pending 不在本模块职责内（留给 ISR，Task 13）。
 *----------------------------------------------------------*/
RtcTimebase_Status_t RtcTimebase_CancelAlarm(RtcTimebase_t *tb);

#ifdef __cplusplus
}
#endif

#endif /* __RTC_TIMEBASE_H__ */
