 /***********************
* 使用ADC1通道4读取模拟信号
* 引脚: PA4 (ADC1_CH4)


************************/ 
#ifndef __MQ2_H
#define __MQ2_H

#include <stddef.h>   /* NULL — portable across ARMCC and host gcc */
#include "sys.h"

/* MQ2 ADC引脚定义 */
#define MQ2_ADC_GPIO_PORT       GPIOA
#define MQ2_ADC_GPIO_PIN        GPIO_Pin_4
#define MQ2_ADC_GPIO_CLK        RCC_APB2Periph_GPIOA
#define MQ2_ADC_CHANNEL         ADC_Channel_4       /* ADC通道4 */

/* 气体浓度阈值定义（用于防回流模式） */
#define GAS_THRESHOLD_NORMAL    100.0f                  /* 正常阈值 */
#define GAS_THRESHOLD_HIGH      2000.0f                 /* 切换后的高阈值 */

/* ADC转换超时循环次数（正确性修复：防止EOC死等）*/
#define MQ2_ADC_TIMEOUT         10000UL

/*-----------------------------------------------------------
 * 状态码（消除"返回0"歧义）
 * 旧接口超时/无效采样/换算错误统一返回0，调用方无法区分
 * "浓度真的是0"与"传感器故障"——新接口显式返回状态码，
 * 失败时输出参数不写入（调用方保留上一有效值）
 *----------------------------------------------------------*/
typedef enum {
    MQ2_OK = 0,                 /* 采样与换算全部成功 */
    MQ2_ERR_ADC_TIMEOUT,        /* ADC转换超时（EOC未置位，硬件级故障）*/
    MQ2_ERR_INVALID_SAMPLE,     /* 采样值无效（adc_avg==0或电压非法）*/
    MQ2_ERR_MATH                /* 换算数学错误（RS<=0或结果NaN）*/
} MQ2_Status_t;

/* 函数声明 */
void MQ2_Init(void);                                /* 初始化MQ2传感器 */
u16 MQ2_GetAdcValue(void);                          /* 获取ADC原始值（legacy：超时返回0）*/
float MQ2_GetGasConcentration(void);                /* 获取气体浓度值（legacy：失败返回0）*/
u8 MQ2_IsGasDetected(u16 threshold);                /* 检测是否超过阈值 */

/* 新接口（状态码+输出参数，失败不写输出）*/
MQ2_Status_t MQ2_GetAdcValueSafe(u16 *adc_out);     /* 带状态的ADC采样 */
MQ2_Status_t MQ2_ComputeConcentration(u16 adc_avg, float *gas_out);  /* 纯逻辑：ADC均值→浓度（主机可测）*/
MQ2_Status_t MQ2_ReadGasConcentration(float *gas_out);  /* 10次采样平均+换算 */

#endif /* __MQ2_H */
