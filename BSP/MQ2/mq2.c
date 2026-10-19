 /***********************
* 使用ADC1通道4读取模拟信号
* 引脚: PA4 (ADC1_CH4)


************************/  
#include "mq2.h"
#include "delay.h"
#include <math.h> 

/**
 * @brief 初始化MQ2传感器（ADC）
 */
void MQ2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    
    /* 使能GPIO和ADC1时钟 */
    RCC_APB2PeriphClockCmd(MQ2_ADC_GPIO_CLK | RCC_APB2Periph_ADC1, ENABLE);
    
    /* 设置ADC分频因子 72M/6=12M，ADC最大时钟不能超过14M */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    
    /* 配置PA4为模拟输入 */
    GPIO_InitStructure.GPIO_Pin = MQ2_ADC_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;   /* 模拟输入 */
    GPIO_Init(MQ2_ADC_GPIO_PORT, &GPIO_InitStructure);
    
    /* 复位ADC1 */
    ADC_DeInit(ADC1);
    
    /* ADC1配置 */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;              /* 独立模式 */
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;                   /* 单通道模式 */
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;             /* 单次转换模式 */
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; /* 软件触发 */
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;          /* 右对齐 */
    ADC_InitStructure.ADC_NbrOfChannel = 1;                         /* 1个转换通道 */
    ADC_Init(ADC1, &ADC_InitStructure);
    
    /* 使能ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    
    /* ADC校准 */
    ADC_ResetCalibration(ADC1);                     /* 复位校准寄存器 */
    while (ADC_GetResetCalibrationStatus(ADC1));    /* 等待复位完成 */
    ADC_StartCalibration(ADC1);                     /* 开始校准 */
    while (ADC_GetCalibrationStatus(ADC1));         /* 等待校准完成 */
    
    /* MQ2需要预热时间，但不在此处等待 */
}

/**
 * @brief 获取ADC原始值（legacy接口，超时返回0）
 * @return ADC转换值（0-4095），转换超时返回0（与真实0采样无法区分，
 *         新代码请用 MQ2_GetAdcValueSafe 获取显式状态）
 * @note 原代码 while(!EOC) 无超时等待，ADC故障时会死循环卡死SensorTask，
 *       此处增加超时保护（正确性修复）
 */
u16 MQ2_GetAdcValue(void)
{
    u16 adc = 0;

    (void)MQ2_GetAdcValueSafe(&adc);    /* 状态码丢弃：保持legacy返回值语义 */
    return adc;
}

/**
 * @brief 获取ADC原始值（带状态码，消除返回0歧义）
 * @param adc_out: 输出参数，成功时写入采样值（0-4095），失败不写入
 * @return MQ2_OK: 转换成功；MQ2_ERR_ADC_TIMEOUT: EOC等待超时（硬件故障）
 */
MQ2_Status_t MQ2_GetAdcValueSafe(u16 *adc_out)
{
    u32 timeout = 0;

    if (adc_out == NULL)
    {
        return MQ2_ERR_INVALID_SAMPLE;
    }

    /* 配置ADC1通道4，采样时间239.5个周期 */
    ADC_RegularChannelConfig(ADC1, MQ2_ADC_CHANNEL, 1, ADC_SampleTime_239Cycles5);

    /* 启动ADC转换 */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    /* 等待转换完成（带超时保护，防止硬件异常时死循环） */
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
    {
        if (++timeout > MQ2_ADC_TIMEOUT)
        {
            return MQ2_ERR_ADC_TIMEOUT;    /* 超时：显式状态，不再伪装成采样值0 */
        }
    }

    /* 获取转换结果 */
    *adc_out = ADC_GetConversionValue(ADC1);

    return MQ2_OK;
}

/**
 * @brief ADC平均值→气体浓度换算（纯逻辑，主机可测）
 * @param adc_avg: 10次采样的平均值
 * @param gas_out: 输出浓度，仅在返回MQ2_OK时写入
 * @return MQ2_OK: 换算成功
 *         MQ2_ERR_INVALID_SAMPLE: adc_avg==0或电压非法（tmep<=0）
 *         MQ2_ERR_MATH: RS<=0或换算结果NaN
 * @note 状态分类即mock测试的三类输入：
 *       正常（OK）/ 无效采样（INVALID_SAMPLE）/ 数学错误（MATH）；
 *       ADC_TIMEOUT属于MQ2_GetAdcValueSafe的硬件层，不在本函数
 */
MQ2_Status_t MQ2_ComputeConcentration(u16 adc_avg, float *gas_out)
{
    float tmep, RS;
    float gas_concentration;

    if (gas_out == NULL)
    {
        return MQ2_ERR_INVALID_SAMPLE;
    }

    /* ADC值为0说明采样无效（超时/故障），禁止进入后续电阻计算，
     * 否则 tmep=0 会在 RS=(5-tmep)/tmep 处除零（正确性修复）*/
    if (adc_avg == 0)
    {
        return MQ2_ERR_INVALID_SAMPLE;
    }

    /*获取模拟电压值*/
    tmep = (float)adc_avg * (3.3f/4096.0f);

    /* 通过模拟电压值计算气体浓度，公式如下所示
     * RS=(5 - tmep)/tmep * 0.5，tmep合法范围(0,3.3]，
     * 若电压异常（≤0）则拒绝，避免除零和负数开幂产生NaN */
    if (tmep <= 0.0f)
    {
        return MQ2_ERR_INVALID_SAMPLE;
    }
    RS = (5.0f - tmep)/tmep * 0.5f;

    /* RS必须为正，否则pow()对负底数取非整数幂会产生NaN */
    if (RS <= 0.0f)
    {
        return MQ2_ERR_MATH;
    }

    gas_concentration = pow(11.5428f*2/RS, 0.6549f)*100;

    /* 限制输出为有限值，防止异常数据进入控制环 */
    if (gas_concentration != gas_concentration)  /* NaN判断 */
    {
        return MQ2_ERR_MATH;
    }

    *gas_out = gas_concentration;
    return MQ2_OK;
}

/**
 * @brief 获取气体浓度值（legacy接口：多次采样取平均，失败返回0）
 * @return 气体浓度值，失败（超时/无效采样/数学错误）返回0
 * @note 与"真实浓度0"无法区分，新代码请用 MQ2_ReadGasConcentration
 */
float MQ2_GetGasConcentration(void)
{
    float gas = 0.0f;

    (void)MQ2_ReadGasConcentration(&gas);    /* 状态码丢弃：保持legacy语义 */
    return gas;
}

/**
 * @brief 获取气体浓度值（带状态码+输出参数）
 * @param gas_out: 输出浓度，仅在返回MQ2_OK时写入（失败时调用方
 *                 保留上一有效值，不写0污染控制平面）
 * @return 透传采样/换算的状态码：ADC_TIMEOUT / INVALID_SAMPLE / MATH / OK
 * @note 10次采样中任一次超时立即中止返回（硬件故障重试无意义）
 */
MQ2_Status_t MQ2_ReadGasConcentration(float *gas_out)
{
    u8 i;
    u32 sum = 0;
    u16 adc;
    MQ2_Status_t st;

    if (gas_out == NULL)
    {
        return MQ2_ERR_INVALID_SAMPLE;
    }

    /* 采样10次取平均值 */
    for (i = 0; i < 10; i++)
    {
        st = MQ2_GetAdcValueSafe(&adc);
        if (st != MQ2_OK)
        {
            return st;    /* 采样失败：超时立即中止（硬件级故障）*/
        }
        sum += adc;
        delay_us(100);
    }

    return MQ2_ComputeConcentration((u16)(sum / 10), gas_out);
}

/**
 * @brief 检测气体浓度是否超过阈值
 * @param threshold: 浓度阈值
 * @return 1:超过阈值; 0:未超过阈值
 */
u8 MQ2_IsGasDetected(u16 threshold)
{
    /* 用float接收，避免浓度值超过65535时float->u16截断为错误小值 */
    float gas_value = MQ2_GetGasConcentration();

    if (gas_value > (float)threshold)
    {
        return 1;
    }
    return 0;
}
