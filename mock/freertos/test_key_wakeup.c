/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Host-side mock test for the key EXTI wake path,

 *
 * Production sources under test are compiled UNMODIFIED from the project
 * tree (BSP\KEY\key_exti.c + BSP\KEY\key.c); sys.h / FreeRTOS.h / gpiox.h
 * come from stub/.
 *
 * Link-set evidence baked into this suite:
 *   - key_exti.c links WITHOUT any FreeRTOS source: the "EXTI ISR only
 *     notifies, never delays / never printf / never classifies presses"
 *     contract is enforced by the IO shape (notify goes through a callback)
 *     and the link set (no kernel symbols available here).
 *   - key_exti.c has no KeyEvent knowledge: KeyExti_t carries no event
 *     field; short/long/release classification stays in key.c's state
 *     machine (design 6.7: debounce + press classification stay in task
 *     context).
 *
 * Covers:
 *   - EXTI produces only a notification: ISR body = read pending ->
 *     clear OWN bit -> notify; foreign pending bits are counted, not
 *     cleared (ownership discipline, design 6.6 step 4: key pending is
 *     cleared by the ISR, never by the low-power exit path)
 *   - bounce coalescing: N rapid edges -> N notifies -> ONE
 *     ulTaskNotifyTake consumes the accumulated count -> ONE scan pass ->
 *     the 30 ms debounce state machine sees ONE press event
 *   - edges around sleep are not lost: a press pending set during sleep
 *     (or between the enter-side clear and WFI - the sleep race window)
 *     is still processed by the ISR after wake
 *   - PB1-only registration: line_config called exactly once, mask is
 *     EXTI line 1; KEY2_ENABLED must be 0 (PB12 ownership unconfirmed)
 *   - NVIC priority: KEY_EXTI_NVIC_PRIORITY(6) >=
 *     configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(3) - compiled check in
 *     key_exti.h (this stub FreeRTOS.h defines 3, so the check is ACTIVE on
 *     the host too) plus a runtime value assertion
 *   - scan blocking policy: standby + no key in progress ->
 *     portMAX_DELAY (only EXTI notification wakes; STOP prerequisite);
 *     anything else -> 10 ms finite timeout (debounce / long-press timing)
 */
#include <stdio.h>
#include <string.h>
#include "key_exti.h"
#include "key.h"

/* PB12 ownership unconfirmed: KEY2 wake config must stay compiled out
 * (board_pins.h default 0). */
#if KEY2_ENABLED != 0
#error "KEY2_ENABLED must stay 0 until PB12 ownership is confirmed"
#endif

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

/*-----------------------------------------------------------
 * Scripted IO: EXTI pending / GPIO level / tick / notify value
 *----------------------------------------------------------*/
typedef enum {
    OP_LINE_CONFIG = 1,
    OP_READ_PENDING,
    OP_CLEAR_PENDING,
    OP_NOTIFY
} FakeOp_t;

#define FAKE_ORDER_CAP 64

typedef struct {
    u32 order[FAKE_ORDER_CAP];
    u32 order_len;
    u32 line_config_count;
    u32 pending;            /* EXTI_PR script (foreign bits injected directly) */
} FakeKey_t;

static FakeKey_t g_fk;
static u32 g_notify_value;     /* FreeRTOS notify: give accumulates, take clears */
static u32 g_tick;              /* xTaskGetTickCount script */
static u8  g_key1_pin_level;    /* KEY1 level: 0=pressed (active low), 1=released */

static void rec(FakeKey_t *f, u32 op)
{
    if (f->order_len < FAKE_ORDER_CAP) {
        f->order[f->order_len++] = op;
    }
}

static int idx_of(const FakeKey_t *f, u32 from, u32 op)
{
    u32 i;
    for (i = from; i < f->order_len; i++) {
        if (f->order[i] == op) {
            return (int)i;
        }
    }
    return -1;
}

/*------- key_exti IO callbacks (host fakes) -------*/
static void fake_line_config(void *self)
{
    FakeKey_t *f = (FakeKey_t *)self;
    f->line_config_count++;
    rec(f, OP_LINE_CONFIG);
}

static u32 fake_read_pending(void *self)
{
    FakeKey_t *f = (FakeKey_t *)self;
    rec(f, OP_READ_PENDING);
    return f->pending;
}

static void fake_clear_pending(void *self, u32 mask)
{
    FakeKey_t *f = (FakeKey_t *)self;
    rec(f, OP_CLEAR_PENDING);
    f->pending &= ~mask;    /* PR write-1-to-clear semantics */
}

static void fake_notify(void *self)
{
    rec((FakeKey_t *)self, OP_NOTIFY);
    g_notify_value++;       /* vTaskNotifyGiveFromISR accumulates */
}

/*------- stubs for key.c dependencies (provided by this test) -------*/
static KeyExtiIo_t g_io = {
    &g_fk,
    fake_line_config,
    fake_read_pending,
    fake_clear_pending,
    fake_notify
};

u8 GPIO_ReadInputDataBit(GPIO_TypeDef *port, u16 pin)
{
    (void)port;
    (void)pin;
    return g_key1_pin_level;
}

void io_set(GPIO_TypeDef *port, u16 pin, u32 mode)
{
    (void)port; (void)pin; (void)mode;   /* Key_Init GPIO config: host no-op */
}

TickType_t xTaskGetTickCount(void)
{
    return g_tick;
}

GPIO_TypeDef GPIOA_MOCK_INSTANCE;
GPIO_TypeDef GPIOB_MOCK_INSTANCE;

/*------- task-side notify consumption (KeyScanTask blocking semantics) -------*/
static u32 notify_take_clear(void)
{
    u32 v = g_notify_value;
    g_notify_value = 0;
    return v;               /* ulTaskNotifyTake(pdTRUE, ...) returns + clears */
}

/* One task-loop iteration scan part: one wake -> exactly one scan */
static void scan_pass(void)
{
    Key_Scan();
}

/*-----------------------------------------------------------
 * Shared state: key_exti instance (re-initialized per scenario)
 *----------------------------------------------------------*/
static KeyExti_t g_ke;

static void suite_reset(void)
{
    memset(&g_fk, 0, sizeof(g_fk));
    memset(&g_ke, 0, sizeof(g_ke));
    g_notify_value = 0;
    g_tick = 0;
    g_key1_pin_level = 1;      /* released (pull-up) */
    g_io.self = &g_fk;
}

/*-----------------------------------------------------------
 * s1: EXTI ISR produces only a notification - role separation
 *----------------------------------------------------------*/
static void s1_isr_notify_only(void)
{
    KeyExti_t ke;
    printf("s1: EXTI ISR produces only a notification\n");

    suite_reset();
    ke = g_ke;
    KeyExti_Init(&ke, &g_io);
    CHECK_EQ_DESC("Init: line_config called exactly once",
                  g_fk.line_config_count, 1);
    CHECK_EQ_DESC("Init: entry counter zeroed", ke.isr_entry_count, 0);
    CHECK_EQ_DESC("Init: key irq counter zeroed", ke.key_irq_count, 0);

    /* PR1 arrival: read -> clear -> notify */
    g_fk.pending = KEY_EXTI_LINE_MASK;
    g_fk.order_len = 0;
    KeyExti_IsrBody(&ke, &g_io);
    CHECK_EQ_DESC("s1 isr entry counted", ke.isr_entry_count, 1);
    CHECK_EQ_DESC("s1 key irq counted", ke.key_irq_count, 1);
    CHECK_EQ_DESC("s1 notify given once", g_notify_value, 1);
    CHECK_EQ_DESC("s1 PR1 cleared in fake", g_fk.pending, 0);
    CHECK_EQ_DESC("s1 order: read before clear",
                  idx_of(&g_fk, 0, OP_READ_PENDING)
                  < idx_of(&g_fk, 0, OP_CLEAR_PENDING), 1);
    CHECK_EQ_DESC("s1 order: clear before notify",
                  idx_of(&g_fk, 0, OP_CLEAR_PENDING)
                  < idx_of(&g_fk, 0, OP_NOTIFY), 1);

    /* Foreign pending (EXTI17 = RTC Alarm bit): no notify, no clear
     * (ownership discipline - key ISR never touches foreign lines) */
    g_fk.order_len = 0;
    g_fk.pending = 0x00020000u;    /* PR17 */
    KeyExti_IsrBody(&ke, &g_io);
    CHECK_EQ_DESC("s1 foreign pending: no notify", g_notify_value, 1);
    CHECK_EQ_DESC("s1 foreign pending: not cleared", g_fk.pending, 0x00020000u);
    CHECK_EQ_DESC("s1 foreign counted", ke.foreign_count, 1);

    /* Two sources arriving together: key bit processed + foreign bit
     * counted - both recorded simultaneously, neither lost */
    g_fk.pending = KEY_EXTI_LINE_MASK | 0x00020000u;
    KeyExti_IsrBody(&ke, &g_io);
    CHECK_EQ_DESC("s1 both pending: key processed", ke.key_irq_count, 2);
    CHECK_EQ_DESC("s1 both pending: foreign recorded", ke.foreign_count, 2);
    CHECK_EQ_DESC("s1 both pending: only key bit cleared",
                  g_fk.pending, 0x00020000u);
    CHECK_EQ_DESC("s1 notify total", g_notify_value, 2);
}

/*-----------------------------------------------------------
 * s2: contact bounce coalesces to ONE press event
 *----------------------------------------------------------*/
static void s2_bounce_single_press(void)
{
    int i;
    u32 wakes;
    printf("s2: contact bounce coalesces to one press event\n");

    suite_reset();
    Key_Init();

    /* 5 bounce edges within ~1 ms */
    for (i = 0; i < 5; i++) {
        g_fk.pending |= KEY_EXTI_LINE_MASK;
        KeyExti_IsrBody(&g_ke, &g_io);
    }
    CHECK_EQ_DESC("s2 five edges -> five notifies", g_notify_value, 5);

    /* ONE take consumes the whole accumulated value -> one wake ->
     * exactly one scan pass */
    wakes = notify_take_clear();
    CHECK_EQ_DESC("s2 one take consumes all 5", wakes, 5);
    g_key1_pin_level = 0;                 /* user holds the key */
    scan_pass();                          /* t=0: IDLE -> DEBOUNCE */

    /* advance through the debounce window (one scan per 10 ms) */
    g_tick = 10; scan_pass();
    g_tick = 20; scan_pass();
    g_tick = 30; scan_pass();              /* t=30: DEBOUNCE -> PRESSED */
    CHECK_EQ_DESC("s2 pressed confirmed once", Key1_IsPressed(), 1);

    /* 3 more bounce edges arrive while held: coalesce again, no new event */
    for (i = 0; i < 3; i++) {
        g_fk.pending |= KEY_EXTI_LINE_MASK;
        KeyExti_IsrBody(&g_ke, &g_io);
    }
    wakes = notify_take_clear();
    CHECK_EQ_DESC("s2 in-hold bounce coalesced", wakes, 3);
    g_tick = 40; scan_pass();             /* still the SAME press */
    CHECK_EQ_DESC("s2 no spurious event during hold",
                  Key1_GetEvent(), KEY_EVENT_NONE);

    /* release (<1 s: short press) */
    g_key1_pin_level = 1;
    g_tick = 100; scan_pass();            /* -> IDLE + SHORT_PRESS */
    CHECK_EQ_DESC("s2 exactly one SHORT_PRESS", Key1_GetEvent(),
                  KEY_EVENT_SHORT_PRESS);
    Key1_ClearEvent();
    CHECK_EQ_DESC("s2 event consumed once", Key1_GetEvent(),
                  KEY_EVENT_NONE);
}

/*-----------------------------------------------------------
 * s3: edges around sleep are not lost
 *----------------------------------------------------------*/
static void s3_sleep_edge_not_lost(void)
{
    u32 wakes;
    printf("s3: edges around sleep are not lost\n");

    suite_reset();
    Key_Init();

    /* Enter sequence: stale pending cleared BEFORE WFI (sleep-race
     * prevention, already proven in Task 12 exti-s2) */
    g_fk.pending |= KEY_EXTI_LINE_MASK;
    g_fk.pending &= ~KEY_EXTI_LINE_MASK;  /* emulate enter-side clear */

    /* Press DURING sleep: PR sets, WFI returns -> ISR runs */
    g_fk.pending |= KEY_EXTI_LINE_MASK;
    KeyExti_IsrBody(&g_ke, &g_io);        /* ISR sees the sleeping edge */
    CHECK_EQ_DESC("s3 sleeping edge still notifies", g_notify_value, 1);
    CHECK_EQ_DESC("s3 sleeping edge cleared by ISR", g_fk.pending, 0);

    /* After wake the task consumes the notify and scans: full press flow */
    wakes = notify_take_clear();
    CHECK_EQ_DESC("s3 wake take returns 1", wakes, 1);
    g_key1_pin_level = 0;                 /* finger still on the key */
    g_tick = 0;   scan_pass();
    g_tick = 35;  scan_pass();            /* DEBOUNCE -> PRESSED */
    CHECK_EQ_DESC("s3 press captured after wake", Key1_IsPressed(), 1);
    g_key1_pin_level = 1;
    g_tick = 100; scan_pass();
    CHECK_EQ_DESC("s3 full short press survives sleep", Key1_GetEvent(),
                  KEY_EVENT_SHORT_PRESS);
}

/*-----------------------------------------------------------
 * s4: classification stays in the state machine (unchanged semantics)
 *----------------------------------------------------------*/
static void s4_state_machine_unchanged(void)
{
    printf("s4: press classification stays in the state machine\n");

    suite_reset();
    Key_Init();

    /* long press: hold > 1 s */
    g_key1_pin_level = 0;
    g_tick = 0;    scan_pass();           /* IDLE -> DEBOUNCE */
    g_tick = 30;  scan_pass();            /* DEBOUNCE -> PRESSED */
    g_tick = 500; scan_pass();
    CHECK_EQ_DESC("s4 no event before long threshold",
                  Key1_GetEvent(), KEY_EVENT_NONE);
    g_tick = 1031; scan_pass();           /* elapsed=1001 > 1000 -> LONG_PRESS */
    CHECK_EQ_DESC("s4 LONG_PRESS from state machine", Key1_GetEvent(),
                  KEY_EVENT_LONG_PRESS);
    Key1_ClearEvent();
    CHECK_EQ_DESC("s4 still pressed during long press", Key1_IsPressed(), 1);

    g_tick = 1041; scan_pass();           /* long press ongoing */
    CHECK_EQ_DESC("s4 LONG_PRESSING repeats", Key1_GetEvent(),
                  KEY_EVENT_LONG_PRESSING);

    g_key1_pin_level = 1;
    g_tick = 1051; scan_pass();           /* release */
    CHECK_EQ_DESC("s4 RELEASE on release", Key1_GetEvent(),
                  KEY_EVENT_RELEASE);
    Key1_ClearEvent();
    CHECK_EQ_DESC("s4 idle after release", Key1_IsPressed(), 0);

    /* short press path re-check (quick press-release) */
    g_key1_pin_level = 0;
    g_tick = 2000; scan_pass();
    g_tick = 2035; scan_pass();           /* PRESSED */
    g_key1_pin_level = 1;
    g_tick = 2100; scan_pass();
    CHECK_EQ_DESC("s4 SHORT_PRESS still classified", Key1_GetEvent(),
                  KEY_EVENT_SHORT_PRESS);
}

/*-----------------------------------------------------------
 * s5: PB1-only registration + NVIC priority contract
 *----------------------------------------------------------*/
static void s5_pb1_only_and_priority(void)
{
    printf("s5: PB1-only registration and NVIC priority contract\n");

    suite_reset();

    CHECK_EQ_DESC("s5 wake line is EXTI1 (PR1 bit)", KEY_EXTI_LINE_MASK, 0x0002u);
    CHECK_EQ_DESC("s5 KEY2 disabled (PB12 unconfirmed)", KEY2_ENABLED, 0);

    /* re-Init: line_config idempotent (exactly one call per Init) */
    KeyExti_Init(&g_ke, &g_io);
    KeyExti_Init(&g_ke, &g_io);
    CHECK_EQ_DESC("s5 line_config once per Init",
                  g_fk.line_config_count, 2);
    CHECK_EQ_DESC("s5 counters reset by re-Init", g_ke.isr_entry_count, 0);

    /* NVIC priority contract: numeric 6 >= syscall 3 (FromISR-legal zone) */
    CHECK_EQ_DESC("s5 key EXTI priority is 6", KEY_EXTI_NVIC_PRIORITY, 6);
    CHECK_EQ_DESC("s5 syscall priority is 3",
                  configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 3);
    CHECK_DESC("s5 priority numerically >= syscall floor (lower prio)",
               KEY_EXTI_NVIC_PRIORITY
               >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    /* The compile-time check itself lives in key_exti.h; this stub
     * defines the syscall priority as 3 so the check is ACTIVE on the
     * host as well - this file compiling IS the first proof, the
     * runtime value assertions above are the second. */
}

/*-----------------------------------------------------------
 * s6: scan blocking policy (KeyScanTask wait decision table)
 *----------------------------------------------------------*/
static void s6_scan_wait_policy(void)
{
    printf("s6: scan blocking policy\n");

    CHECK_EQ_DESC("s6 standby + idle -> portMAX_DELAY",
                  KeyExti_ScanWaitTicks(1, 0), portMAX_DELAY);
    CHECK_EQ_DESC("s6 standby + in-progress -> finite",
                  KeyExti_ScanWaitTicks(1, 1), KEY_EXTI_ACTIVE_SCAN_MS);
    CHECK_EQ_DESC("s6 active + idle -> finite",
                  KeyExti_ScanWaitTicks(0, 0), KEY_EXTI_ACTIVE_SCAN_MS);
    CHECK_EQ_DESC("s6 active + in-progress -> finite",
                  KeyExti_ScanWaitTicks(0, 1), KEY_EXTI_ACTIVE_SCAN_MS);
    CHECK_EQ_DESC("s6 finite period is 10ms",
                  KEY_EXTI_ACTIVE_SCAN_MS, 10);
}

/*-----------------------------------------------------------
 * s7: Key1_InProgress - the in-progress exemption for standby block
 *----------------------------------------------------------*/
static void s7_in_progress_gate(void)
{
    printf("s7: Key1_InProgress gates the standby block\n");

    suite_reset();
    Key_Init();
    CHECK_EQ_DESC("s7 idle at start", Key1_InProgress(), 0);

    g_key1_pin_level = 0;
    g_tick = 0;   scan_pass();           /* -> DEBOUNCE: in progress */
    CHECK_EQ_DESC("s7 debounce counts as in progress", Key1_InProgress(), 1);
    g_tick = 30;  scan_pass();           /* -> PRESSED */
    CHECK_EQ_DESC("s7 pressed counts as in progress", Key1_InProgress(), 1);
    g_key1_pin_level = 1;
    g_tick = 100; scan_pass();           /* -> IDLE */
    CHECK_EQ_DESC("s7 idle again after release", Key1_InProgress(), 0);
}

int main(void)
{
    s1_isr_notify_only();
    s2_bounce_single_press();
    s3_sleep_edge_not_lost();
    s4_state_machine_unchanged();
    s5_pb1_only_and_priority();
    s6_scan_wait_policy();
    s7_in_progress_gate();

    printf("key-wakeup: %d checks, %d failed\n", g_total, g_failed);
    return (g_failed == 0) ? 0 : 1;
}