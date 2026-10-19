/*
 * APP 侧升级请求实现
 *
 * 实现说明：
 *   - 停机走 motor_stop()/Beep_off() 直接关断：PWM 通道禁用 + H 桥使能
 *     脚拉低 + 蜂鸣器 GPIO 拉低。不经 g_motorCmdQueue——队列消费要等
 *     MotorControlTask 调度，而本函数在复位前必须确定性地完成关断；
 *   - BKP_DR2 写入前开 PWR/BKP 时钟并置 DBP（后备域写权限），
 *     与 Boot 侧 BootRequest_Init() 同一纪律：只开权限，
 *     绝不触发后备域复位（否则请求丢失，设计 §5.1）；
 *   - __DSB() 保证 BKP 写入真正落到后备域后再复位；
 *   - 电机惯性滑行不做等待：PWM 关断即为电气安全态，机械减速
 *     时间（秒级）远大于本函数执行时间（微秒级）。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#include "iap_request.h"
#include "boot_request.h"          /* BOOT_REQUEST_MAGIC 单一定义点（../BOOT）*/
#include "motor.h"
#include "beep.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_bkp.h"

void IAP_Request_Update(void)
{
    /* 1. 安全停机：直接寄存器级关断（见文件头实现说明） */
    motor_stop();
    Beep_off(&bep);

    /* 2. 登记 BKP_DR2 升级请求（备份域跨复位保持） */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    BKP_WriteBackupRegister(BKP_DR2, BOOT_REQUEST_MAGIC);

    /* 3. 确保写入完成，复位进 Boot */
    __DSB();
    NVIC_SystemReset();             /* 不返回 */
}
