// SPDX-License-Identifier: GPL-2.0
/*
 * Host-side mock test for MQ2 error semantics:
 *   1. MQ2_ComputeConcentration() - pure logic, three input classes:
 *        normal (OK) / invalid sample (adc==0, NULL) / math error (RS<=0)
 *   2. MQ2_GetAdcValueSafe() - hardware layer with controllable ADC mock:
 *        normal EOC / multi-poll EOC / EOC stuck -> MQ2_ERR_ADC_TIMEOUT
 *   3. MQ2_ReadGasConcentration() - 10-sample average integration, incl.
 *      mid-stream ADC failure (fail-fast, output NOT written on error)
 *   4. legacy wrappers - old "return 0 on failure" semantics preserved
 *      (documents the ambiguity the new status-code API removes)
 *
 * Production source under test (BSP\MQ2\mq2.c) is compiled UNMODIFIED;
 * SPL/GPIO/ADC/delay bodies come from the controllable mocks below.
 */
#include <stdio.h>
#include <math.h>
#include "mq2.h"

/* ---- mock instances referenced by stub/stm32f10x.h ---- */
GPIO_TypeDef GPIOA_MOCK_INSTANCE;
ADC_TypeDef  ADC1_MOCK_INSTANCE;

/* ---- controllable ADC mock state ---- */
static u16 mock_adc_value   = 0;          /* value latched into DR            */
static u32 mock_eoc_latency = 1;          /* EOC polls before ready; 0=stuck  */
static u32 mock_ok_budget   = 0xFFFFFFFFu;/* conversions allowed to succeed   */
static u32 mock_poll_count  = 0;          /* polls since last conversion start*/
static u32 mock_conv_starts = 0;          /* ADC_SoftwareStartConvCmd(ENABLE) */
static u32 mock_delay_calls = 0;          /* delay_us() invocations           */

static void mock_reset(u16 adc_value, u32 eoc_latency)
{
    mock_adc_value   = adc_value;
    mock_eoc_latency = eoc_latency;
    mock_ok_budget   = 0xFFFFFFFFu;
    mock_poll_count  = 0;
    mock_conv_starts = 0;
    mock_delay_calls = 0;
}

/* ---- SPL mock bodies ---- */
void GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{ (void)port; (void)init; }

void RCC_APB2PeriphClockCmd(u32 periph, FunctionalState state)
{ (void)periph; (void)state; }

void RCC_ADCCLKConfig(u32 clk)
{ (void)clk; }

void ADC_DeInit(ADC_TypeDef *adc)                    { (void)adc; }
void ADC_Init(ADC_TypeDef *adc, ADC_InitTypeDef *i)  { (void)adc; (void)i; }
void ADC_Cmd(ADC_TypeDef *adc, FunctionalState s)    { (void)adc; (void)s; }
void ADC_ResetCalibration(ADC_TypeDef *adc)          { (void)adc; }
FlagStatus ADC_GetResetCalibrationStatus(ADC_TypeDef *adc) { (void)adc; return 0; }
void ADC_StartCalibration(ADC_TypeDef *adc)          { (void)adc; }
FlagStatus ADC_GetCalibrationStatus(ADC_TypeDef *adc)      { (void)adc; return 0; }

void ADC_RegularChannelConfig(ADC_TypeDef *adc, u8 ch, u8 rank, u8 st)
{ (void)adc; (void)ch; (void)rank; (void)st; }

void ADC_SoftwareStartConvCmd(ADC_TypeDef *adc, FunctionalState state)
{
    (void)adc;
    if (state == ENABLE)
    {
        mock_conv_starts++;
        mock_poll_count = 0;
    }
}

FlagStatus ADC_GetFlagStatus(ADC_TypeDef *adc, u32 flag)
{
    (void)adc; (void)flag;
    mock_poll_count++;
    if (mock_conv_starts <= mock_ok_budget &&
        mock_eoc_latency != 0 &&
        mock_poll_count >= mock_eoc_latency)
    {
        return SET;
    }
    return RESET;
}

u16 ADC_GetConversionValue(ADC_TypeDef *adc)
{ (void)adc; return mock_adc_value; }

/* ---- delay mock ---- */
void delay_init(void)          { }
void delay_ms(u32 nms)         { (void)nms; }
void delay_xms(u32 nms)        { (void)nms; }
void delay_us(u32 nus)         { (void)nus; mock_delay_calls++; }

/* ---- tiny check framework (same shape as test_task_health.c) ---- */
static int g_failed = 0;
static int g_total = 0;

#define CHECK_DESC(desc, cond) do {                       \
    g_total++;                                            \
    if (cond) { printf("  ok %s\n", desc); }              \
    else      { g_failed++; printf("  FAIL %s\n", desc); }\
} while (0)

#define CHECK_EQ_DESC(desc, got, want) do {               \
    g_total++;                                            \
    if ((unsigned long)(got) == (unsigned long)(want))    \
        printf("  ok %s\n", desc);                        \
    else { g_failed++;                                    \
        printf("  FAIL %s: got %lu, want %lu\n",          \
               desc, (unsigned long)(got),                \
               (unsigned long)(want)); }                  \
} while (0)

#define CHECK_FNEAR(desc, got, want, tol) do {            \
    g_total++;                                            \
    if (fabsf((got) - (want)) <= (tol))                   \
        printf("  ok %s\n", desc);                        \
    else { g_failed++;                                    \
        printf("  FAIL %s: got %f, want %f (tol %f)\n",   \
               desc, (double)(got), (double)(want),       \
               (double)(tol)); }                          \
} while (0)

/* Reference formula (same ops as production, for plumbing check only;
 * hard anchors below independently pin the expected numbers) */
static float ref_gas(u16 adc)
{
    float tmep = (float)adc * (3.3f / 4096.0f);
    float RS   = (5.0f - tmep) / tmep * 0.5f;
    return powf(11.5428f * 2 / RS, 0.6549f) * 100;
}

/* ====================================================================
 * Suite 1: MQ2_ComputeConcentration - pure logic, three input classes
 * ==================================================================== */
static void test_compute(void)
{
    float gas;
    float sentinel = -12345.0f;

    printf("suite mq2-compute-concentration\n");

    /* --- input class 1: invalid sample --- */
    gas = sentinel;
    CHECK_EQ_DESC("NULL out -> MQ2_ERR_INVALID_SAMPLE",
        MQ2_ComputeConcentration(2000, NULL), MQ2_ERR_INVALID_SAMPLE);

    gas = sentinel;
    CHECK_EQ_DESC("adc_avg == 0 -> MQ2_ERR_INVALID_SAMPLE",
        MQ2_ComputeConcentration(0, &gas), MQ2_ERR_INVALID_SAMPLE);
    CHECK_FNEAR("  and output NOT written on error", gas, sentinel, 0.0f);

    /* --- input class 2: normal --- */
    gas = sentinel;
    CHECK_EQ_DESC("normal adc=2000 -> MQ2_OK",
        MQ2_ComputeConcentration(2000, &gas), MQ2_OK);
    /* hard anchor: tmep=1.6113, RS=1.0515 -> ~756.1 ppm (independent pin) */
    CHECK_FNEAR("  value matches hard anchor 756.1", gas, 756.1f, 3.0f);
    CHECK_FNEAR("  value matches reference formula", gas, ref_gas(2000), 0.05f);

    gas = sentinel;
    CHECK_EQ_DESC("full-scale adc=4095 (real 12-bit max) -> MQ2_OK",
        MQ2_ComputeConcentration(4095, &gas), MQ2_OK);
    CHECK_FNEAR("  value matches reference formula", gas, ref_gas(4095), 0.05f);

    /* --- input class 3: math error (RS<=0) ---
     * u16 adc_avg is NOT clamped to 12-bit: tmep = adc*3.3/4096 crosses 5V
     * at adc >= 6207, making RS = (5-tmep)/tmep*0.5 negative */
    gas = sentinel;
    CHECK_EQ_DESC("boundary adc=6206 (tmep just below 5V) -> MQ2_OK",
        MQ2_ComputeConcentration(6206, &gas), MQ2_OK);

    gas = sentinel;
    CHECK_EQ_DESC("boundary adc=6207 (RS<=0) -> MQ2_ERR_MATH",
        MQ2_ComputeConcentration(6207, &gas), MQ2_ERR_MATH);
    CHECK_FNEAR("  and output NOT written on error", gas, sentinel, 0.0f);

    gas = sentinel;
    CHECK_EQ_DESC("far out-of-range adc=60000 -> MQ2_ERR_MATH",
        MQ2_ComputeConcentration(60000, &gas), MQ2_ERR_MATH);
}

/* ====================================================================
 * Suite 2: MQ2_GetAdcValueSafe - hardware layer (ADC timeout scenario)
 * ==================================================================== */
static void test_adc_safe(void)
{
    u16 adc;
    u16 sentinel16 = 0xBEEF;

    printf("suite mq2-get-adc-safe\n");

    CHECK_EQ_DESC("NULL out -> MQ2_ERR_INVALID_SAMPLE",
        MQ2_GetAdcValueSafe(NULL), MQ2_ERR_INVALID_SAMPLE);

    mock_reset(1234, 1);
    adc = sentinel16;
    CHECK_EQ_DESC("normal conversion (EOC after 1 poll) -> MQ2_OK",
        MQ2_GetAdcValueSafe(&adc), MQ2_OK);
    CHECK_EQ_DESC("  adc_out == 1234", adc, 1234);
    CHECK_EQ_DESC("  exactly 1 conversion started", mock_conv_starts, 1);

    mock_reset(1234, 5);
    adc = sentinel16;
    CHECK_EQ_DESC("multi-poll EOC (ready after 5 polls) -> MQ2_OK",
        MQ2_GetAdcValueSafe(&adc), MQ2_OK);
    CHECK_EQ_DESC("  adc_out == 1234", adc, 1234);
    CHECK_EQ_DESC("  5 flag polls taken", mock_poll_count, 5);

    /* --- ADC_TIMEOUT scenario: EOC never sets (hardware fault) --- */
    mock_reset(1234, 0);              /* latency 0 = EOC stuck low */
    adc = sentinel16;
    CHECK_EQ_DESC("EOC stuck -> MQ2_ERR_ADC_TIMEOUT",
        MQ2_GetAdcValueSafe(&adc), MQ2_ERR_ADC_TIMEOUT);
    CHECK_EQ_DESC("  output NOT written (sentinel intact)", adc, sentinel16);
    CHECK_DESC("  timeout guard tripped (polls > MQ2_ADC_TIMEOUT)",
        mock_poll_count > MQ2_ADC_TIMEOUT);
}

/* ====================================================================
 * Suite 3: MQ2_ReadGasConcentration - 10-sample average integration
 * ==================================================================== */
static void test_read_gas(void)
{
    float gas;
    float sentinel = -12345.0f;

    printf("suite mq2-read-gas-concentration\n");

    CHECK_EQ_DESC("NULL out -> MQ2_ERR_INVALID_SAMPLE",
        MQ2_ReadGasConcentration(NULL), MQ2_ERR_INVALID_SAMPLE);

    /* 10 uniform samples of 2000 -> average 2000 */
    mock_reset(2000, 1);
    gas = sentinel;
    CHECK_EQ_DESC("10x adc=2000 -> MQ2_OK",
        MQ2_ReadGasConcentration(&gas), MQ2_OK);
    CHECK_FNEAR("  value == compute(2000)", gas, ref_gas(2000), 0.05f);
    CHECK_EQ_DESC("  10 conversions started", mock_conv_starts, 10);
    CHECK_EQ_DESC("  10 inter-sample delays taken", mock_delay_calls, 10);

    /* uniform 1001 -> integer average 1001 (sum 10010 / 10) */
    mock_reset(1001, 1);
    gas = sentinel;
    CHECK_EQ_DESC("10x adc=1001 -> MQ2_OK",
        MQ2_ReadGasConcentration(&gas), MQ2_OK);
    CHECK_FNEAR("  value == compute(1001)", gas, ref_gas(1001), 0.05f);

    /* --- mid-stream ADC failure: 2 samples OK, 3rd conversion EOC stuck ---
     * contract: fail-fast (no retry), output NOT written, caller keeps
     * last good value (SensorTask relies on this) */
    mock_reset(2000, 1);
    mock_ok_budget = 2;
    gas = sentinel;
    CHECK_EQ_DESC("ADC dies after 2 samples -> MQ2_ERR_ADC_TIMEOUT",
        MQ2_ReadGasConcentration(&gas), MQ2_ERR_ADC_TIMEOUT);
    CHECK_FNEAR("  output NOT written (sentinel intact)", gas, sentinel, 0.0f);
    CHECK_EQ_DESC("  fail-fast: only 3 conversions attempted",
        mock_conv_starts, 3);
    CHECK_EQ_DESC("  delays only for the 2 successful samples",
        mock_delay_calls, 2);

    /* invalid average propagates as INVALID_SAMPLE (not 0 concentration) */
    mock_reset(0, 1);                 /* all samples read 0 */
    gas = sentinel;
    CHECK_EQ_DESC("all-zero samples -> MQ2_ERR_INVALID_SAMPLE",
        MQ2_ReadGasConcentration(&gas), MQ2_ERR_INVALID_SAMPLE);
    CHECK_FNEAR("  output NOT written on error", gas, sentinel, 0.0f);
}

/* ====================================================================
 * Suite 4: legacy wrappers keep old semantics (regression pin)
 * ==================================================================== */
static void test_legacy(void)
{
    printf("suite mq2-legacy-wrappers\n");

    mock_reset(1234, 1);
    CHECK_EQ_DESC("MQ2_GetAdcValue normal -> raw value",
        MQ2_GetAdcValue(), 1234);

    mock_reset(1234, 0);              /* EOC stuck */
    CHECK_EQ_DESC("MQ2_GetAdcValue on timeout -> 0 (legacy ambiguity)",
        MQ2_GetAdcValue(), 0);

    mock_reset(2000, 1);
    CHECK_FNEAR("MQ2_GetGasConcentration normal -> ~756.1",
        MQ2_GetGasConcentration(), 756.1f, 3.0f);

    mock_reset(2000, 0);              /* EOC stuck: failure reads as 0.0 */
    CHECK_FNEAR("MQ2_GetGasConcentration on timeout -> 0.0 (the ambiguity "
        "the new MQ2_Status_t API removes)",
        MQ2_GetGasConcentration(), 0.0f, 0.0f);

    mock_reset(2000, 1);
    CHECK_EQ_DESC("MQ2_IsGasDetected(th=100) with ~756 ppm -> 1",
        MQ2_IsGasDetected(100), 1);
    CHECK_EQ_DESC("MQ2_IsGasDetected(th=2000) with ~756 ppm -> 0",
        MQ2_IsGasDetected(2000), 0);
}

int main(void)
{
    test_compute();
    test_adc_safe();
    test_read_gas();
    test_legacy();

    printf("mock result: %d/%d cases passed (MQ2 error semantics)\n",
           g_total - g_failed, g_total);
    return g_failed ? 1 : 0;
}
