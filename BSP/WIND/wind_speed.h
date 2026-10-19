 /*
 * 风速算法模块头文件
 * 根据传感器数据计算PWM占空比
 *
 */
#ifndef __WIND_SPEED_H
#define __WIND_SPEED_H

#include "sys.h"

#define MAXCCR              1000        /* 定时器比较值最大值 */

/*-----------------------------------------------------------
 * 算法参数定义
 *----------------------------------------------------------*/
/* 温度参数 */
#define TEMP_BASE           20       /* 温度基准值 (℃) */
#define TEMP_MAX            35       /* 温度上限值 (℃) - 厨房实际最高温度 */

/* 湿度参数 */
#define HUMIDITY_BASE       40       /* 湿度基准值 (%) */
#define HUMIDITY_MAX        75       /* 湿度上限值 (%) - 烹饪时实际最高湿度 */

/* 气体浓度参数 */
#define GAS_BASE            80.0f       /* 气体浓度基准值 - 正常空气 */
#define GAS_MAX             450.0f      /* 气体浓度上限 - 根据实际测量值调整 */

/* 权重参数 */
#define WEIGHT_TEMP         0.2f        /* 温度权重 */
#define WEIGHT_HUMIDITY     0.2f        /* 湿度权重 */
#define WEIGHT_GAS          0.6f        /* 气体浓度权重 */

/* PWM参数 */
#define PWM_MIN             20.0f        /* PWM占空比最小值 (%) */
#define PWM_MAX             100.0f      /* PWM占空比最大值 (%) */

/*-----------------------------------------------------------
 * Cooking Event判定条件
 * 温度 > 26 && (湿度 > 50 && 气体浓度 > 100)
 *----------------------------------------------------------*/
#define COOKING_TEMP_THRESHOLD      26       /* Cooking Event温度阈值 */
#define COOKING_HUMIDITY_THRESHOLD  50       /* Cooking Event湿度阈值 */
#define COOKING_GAS_THRESHOLD       100.0f   /* Cooking Event气体浓度阈值 */

/*-----------------------------------------------------------
 * 手动模式档位对应转速 (RPM)
 *----------------------------------------------------------*/
#define SPEED_LOW_RPM       190         /* 低档转速 */
#define SPEED_HIGH_RPM      220         /* 高档转速 */

/*-----------------------------------------------------------
 * 风速数据结构体
 *----------------------------------------------------------*/
typedef struct {
    
    float f_T;                  /* 温度影响系数 (归一化后) */
    float f_H;                  /* 湿度影响系数 (归一化后) */
    float f_G;                  /* 气体浓度影响系数 (归一化后) */
    
    float fusionValue;          /* 融合后的值 F */
    float pwmValue;             /* 计算得到的PWM值 (%) */
    
    u8 isCookingEvent;          /* 是否为Cooking Event */
} WindSpeed_t;

/*-----------------------------------------------------------
 * 函数声明
 *----------------------------------------------------------*/
/* 初始化风速算法模块 */
void WindSpeed_Init(void);

/* 更新传感器数据并计算风速 */
void WindSpeed_Update(u8 temp, u8 humidity, float gas);

/* 获取计算得到的PWM值 */
float WindSpeed_GetPWM(void);

/* 获取PWM值对应的占空比（0-1000映射到定时器比较值） */
u16 WindSpeed_GetPWMCompare(u16 maxCompare);

/* 检测是否为Cooking Event */
u8 WindSpeed_IsCookingEvent(void);

/* 获取风速数据结构体指针（用于显示详细信息） */
WindSpeed_t* WindSpeed_GetData(void);

/* 根据档位获取目标转速 */
u16 WindSpeed_GetTargetRPM(u8 level);

#endif /* __WIND_SPEED_H */
