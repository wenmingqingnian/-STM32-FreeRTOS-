/*
 * 优先级反转对照实验（实现）
 * 实验设计与使用限制见 prio_inversion.h。
 */
#include "prio_inversion.h"   /* switch default lives here - must precede the #if below */
#include "dwt_profiler.h"    /* 复用 DWT 头以保证 DWT 开关一致 */

#if defined(PRIORITY_INVERSION_EXPERIMENT) && PRIORITY_INVERSION_EXPERIMENT
/* This experiment measures with DWT->CYCCNT: require the profiler to be on */
#if !defined(DWT_PROFILER_ENABLED) || !DWT_PROFILER_ENABLED
#error "prio_inversion.c requires DWT_PROFILER_ENABLED=1 (DWT->CYCCNT provider)"
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>

/* ===== 实验优先级 =====
 * 必须高于所有业务任务，避免业务行为污染反转指标。
 * 本工程最高业务优先级 = TASK_IAP_PRIORITY = 7。
 * 这里使用 8/9/10（共 32 级，FreeRTOSConfig.h 已开 configMAX_PRIORITIES=32）*/
#define PRIINV_PRI_LOW      8u
#define PRIINV_PRI_MEDIUM   9u
#define PRIINV_PRI_HIGH     10u

/* Low 持锁区的「CPU 消耗」用 DWT 精确控制：
 * 以 72 MHz 下 ~1 ms 的忙等为基准（约 72 000 cycles），
 * 确保 MediumTask 有足够窗口抢占并引发反转。
 * 不使用 vTaskDelay（延时期间 Medium 会抢占但 Low 不持有锁，
 * 无法稳定复现「持锁期间被抢占」场景）。*/
#define PRIINV_LOW_HOLD_CYCLES   (72000u * 2u)   /* ~2 ms */
/* High 启动延后：让 Low 先拿到锁、Medium 已就绪，
 * 再触发 High 抢锁——稳定命中「Low 持锁 → Medium 抢占 → High 阻塞」时序。*/
#define PRIINV_HIGH_START_DELAY_MS   3u
/* Medium CPU 忙等循环：每个迭代用一个简单的内存访问循环，
 * 防止被 -O 优化完全移除。*/

/* ===== 全局实验上下文 ===== */
typedef struct {
    PriInvGroup_t    group;
    SemaphoreHandle_t shared_lock;   /* Semaphore（A）或 Mutex（B）*/
    TaskHandle_t      h_low;
    TaskHandle_t      h_medium;
    TaskHandle_t      h_high;

    volatile u32      low_enter_cycle;   /* Low 进入临界区时刻 */
    volatile u32      low_exit_cycle;    /* Low 释放锁时刻 */
    volatile u32      high_req_cycle;    /* High 请求锁时刻 */
    volatile u32      high_got_cycle;    /* High 获得锁时刻 */
    volatile u32      medium_count;      /* Medium 忙等计数 */
    volatile u8       phase_low_done;    /* Low 做完一轮 */
    volatile u8       phase_high_done;   /* High 做完一轮 */

    PriInvResult_t   *out;               /* 外部写回结构 */
} PriInvCtx_t;

static PriInvCtx_t g_ctx;

/* 工具：DWT 忙等指定 cycles（不释放 CPU，确保持锁窗口连续） */
static void priinv_busywait_cycles(u32 cycles)
{
    u32 start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) { __NOP(); }
}

/* ================================================================
 * Low 任务：拿锁 → 持锁忙等 2ms → 释放锁
 * ==============================================================*/
static void priinv_low_task(void *pv)
{
    PriInvCtx_t *c = (PriInvCtx_t *)pv;

    /* 1. 拿锁（Mutex/Binary 统一用 Take API） */
    if (xSemaphoreTake(c->shared_lock, portMAX_DELAY) == pdTRUE) {
        c->low_enter_cycle = DWT->CYCCNT;

        /* 2. 持锁忙等 ~2ms：
         *    期间 Medium 会抢占（若优先级高于 Low 且 Low 没被继承提升）。
         *    用 DWT 忙等而不 delay，确保持锁期间 CPU 仍有可抢占窗口 */
        priinv_busywait_cycles(PRIINV_LOW_HOLD_CYCLES);

        c->low_exit_cycle = DWT->CYCCNT;
        (void)xSemaphoreGive(c->shared_lock);
    }
    c->phase_low_done = 1u;

    /* 一轮结束：自删除（Start 函数会在 High 完成后清理）*/
    vTaskDelete(NULL);
}

/* ================================================================
 * Medium 任务：纯 CPU 忙循环（不碰锁），用于制造反转
 * ==============================================================*/
static void priinv_medium_task(void *pv)
{
    PriInvCtx_t *c = (PriInvCtx_t *)pv;
    volatile u32 dummy = 0;

    /* 只要 High 没做完就一直占 CPU，验证「Medium 抢占 Low → High 被 Medium 阻塞」*/
    while (c->phase_high_done == 0u) {
        u32 i;
        for (i = 0; i < 2000u; i++) { dummy += i; }
        c->medium_count++;
    }
    (void)dummy;
    vTaskDelete(NULL);
}

/* ================================================================
 * High 任务：延时启动后请求锁 → 记录阻塞时间
 * ==============================================================*/
static void priinv_high_task(void *pv)
{
    PriInvCtx_t *c = (PriInvCtx_t *)pv;

    /* 给 Low 和 Medium 留 3ms 进入稳定态：
     *   Low 应该已经拿锁并进入持锁区，Medium 应该在就绪或运行。*/
    vTaskDelay(pdMS_TO_TICKS(PRIINV_HIGH_START_DELAY_MS));

    c->high_req_cycle = DWT->CYCCNT;
    if (xSemaphoreTake(c->shared_lock, portMAX_DELAY) == pdTRUE) {
        c->high_got_cycle = DWT->CYCCNT;
        /* 拿到锁立即释放（不做额外工作，只测阻塞时间）*/
        (void)xSemaphoreGive(c->shared_lock);
    }
    c->phase_high_done = 1u;

    vTaskDelete(NULL);
}

/* ================================================================
 * 入口：启动实验并在 out 中返回一轮结果（阻塞到实验完成）。
 *   - 若调用者在任务上下文：可直接用 vTaskDelay 轮询 phase_high_done；
 *     实验本身只创建三个专用任务，不占用调用者栈。
 *   - 互斥保护：函数非重入；若重入则直接在 out->done = 0 返回。
 * ==============================================================*/
void PriInv_Start(PriInvGroup_t group, PriInvResult_t *out)
{
    PriInvCtx_t *c = &g_ctx;
    if (out == (PriInvResult_t *)0) return;

    /* 幂等：若上一轮未清理完则拒绝 */
    if (c->out != (PriInvResult_t *)0) { out->done = 0; return; }

    out->high_block_cycles = 0;
    out->low_hold_cycles   = 0;
    out->medium_loop_count = 0;
    out->done              = 0;

    c->group            = group;
    c->out              = out;
    c->low_enter_cycle  = 0;
    c->low_exit_cycle   = 0;
    c->high_req_cycle   = 0;
    c->high_got_cycle   = 0;
    c->medium_count     = 0;
    c->phase_low_done   = 0;
    c->phase_high_done  = 0;

    /* DWT 可能已由业务侧在 StartTask 中打开；这里兜底确保不读 0 */
    DWT_INIT();

    /* 创建锁 */
    if (group == PRIINV_GROUP_MUTEX) {
        c->shared_lock = xSemaphoreCreateMutex();
    } else {
        /* Binary Semaphore：初始值 = 1（把它当成「1 资源互斥锁」用）。
         * 注意 Binary Semaphore 「Give 会递增，超过 1 也可」的经典特性，
         * 但只要 Low 不重复 Give，模型等价于互斥。*/
        c->shared_lock = xSemaphoreCreateBinary();
        if (c->shared_lock != (SemaphoreHandle_t)0) {
            /* Binary 创建后计数 = 0；先 Give 一次让 Low 可以立即 Take */
            (void)xSemaphoreGive(c->shared_lock);
        }
    }
    if (c->shared_lock == (SemaphoreHandle_t)0) {
        c->out = (PriInvResult_t *)0;
        return;
    }

    /* 创建顺序：Low → Medium → High，确保 Low 在 Medium 运行前就能拿到锁。
     *（FreeRTOS 抢占式：High 会立即抢占 StartTask 创建流程，但 High 会先
     *   vTaskDelay，所以 Medium 后创建也行。这里按理论上最安全顺序创建。）*/
    xTaskCreate(priinv_low_task,    "PI_Low",    128, c, PRIINV_PRI_LOW,    &c->h_low);
    xTaskCreate(priinv_medium_task, "PI_Med",    128, c, PRIINV_PRI_MEDIUM, &c->h_medium);
    xTaskCreate(priinv_high_task,   "PI_High",   128, c, PRIINV_PRI_HIGH,   &c->h_high);

    /* 阻塞到实验完成（High 做完 → Medium 自删；至多 15ms）*/
    {
        u32 wait_ticks = 0;
        while (c->phase_high_done == 0u && wait_ticks < pdMS_TO_TICKS(50)) {
            vTaskDelay(1);
            wait_ticks++;
        }
    }

    /* 回写结果：
     *   block  = high_got - high_req（无符号减法 → 回绕正确）
     *   hold   = low_exit - low_enter
     *   count  = medium_count（侧面证据：Binary 组 count 大，Mutex 组 count≈0）*/
    if (c->phase_high_done) {
        out->low_hold_cycles   = c->low_exit_cycle  - c->low_enter_cycle;
        out->high_block_cycles = c->high_got_cycle  - c->high_req_cycle;
        out->medium_loop_count = c->medium_count;
        out->done              = 1u;
    }

    /* 清理：锁删除 + 上下文复位。
     *（Medium 任务在 phase_high_done==1 后自删，可能还未真正进入 vTaskDelete
     *   到回收阶段，但实验用一次性模块，保留锁到下次启动前不会泄漏。）*/
    if (c->shared_lock != (SemaphoreHandle_t)0) {
        vSemaphoreDelete(c->shared_lock);
        c->shared_lock = (SemaphoreHandle_t)0;
    }
    c->out = (PriInvResult_t *)0;
}

#endif /* PRIORITY_INVERSION_EXPERIMENT */
