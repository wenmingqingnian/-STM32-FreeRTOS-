/*
 * Boot 升级请求与恢复入口
 *
 * 实现说明：
 *   - BKP_DR2 读取不需要 DBP 位，但 Clear() 写入需要，Init() 统一置位；
 *   - 恢复键只在 Boot 入口采样一次（多次去抖后全 1 判定"保持按下"），
 *     Boot 运行期间不再扫描按键——升级模式退出只靠复位，没有运行时
 *     "按键退出"路径，避免引入按键扫描状态机。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#include "boot_request.h"
#include "board_pins.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_bkp.h"

/* 短忙等（只用于按键采样去抖，无精确时序要求） */
static void BootReq_ShortDelay(void)
{
    volatile u32 n = BOOT_REQ_KEY_DELAY_LOOPS;
    while (n-- != 0u)
    {
    }
}

void BootRequest_Init(void)
{
    GPIO_InitTypeDef gpioInit;

    /* PWR/BKP 时钟（APB1）+ 后备域写权限；不重置后备域（设计 §5.1） */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);

    /* 恢复键 KEY1 = PB1：输入上拉（与 APP 侧 key.c 的 KEY1 配置一致，
     * 按下为低电平，KEY_PRESSED_LEVEL=0） */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    gpioInit.GPIO_Pin  = KEY1_GPIO_PIN;
    gpioInit.GPIO_Mode = GPIO_Mode_IPU;
    gpioInit.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(KEY1_GPIO_PORT, &gpioInit);
}

u8 BootRequest_IsRequested(void)
{
    return (BKP_ReadBackupRegister(BKP_DR2) == BOOT_REQUEST_MAGIC) ? 1u : 0u;
}

void BootRequest_Clear(void)
{
    /* 写 0 即失效（0 不是魔数；BKP 寄存器复位值也是 0） */
    BKP_WriteBackupRegister(BKP_DR2, 0u);
}

u8 BootRequest_RecoveryKeyPressed(void)
{
    u8 i;

    /* 多次采样全部为按下才判定"复位时保持按下"（去抖） */
    for (i = 0u; i < BOOT_REQ_KEY_SAMPLES; i++)
    {
        if (GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_PIN) != Bit_RESET)
        {
            return 0u;   /* 任一次为高电平：未保持按下 */
        }
        BootReq_ShortDelay();
    }
    return 1u;
}
