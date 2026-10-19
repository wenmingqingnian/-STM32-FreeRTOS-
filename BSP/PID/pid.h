#ifndef __PID_H
#define __PID_H
  
#include "sys.h"

/**
 * @brief PID控制器结构体（位置式PID）
 */
typedef struct {
    float Kp;           /* 比例系数 */
    float Ki;           /* 积分系数 */
    float Kd;           /* 微分系数 */
    
    float target;       /* 目标值（设定值） */
    float actual;       /* 实际值（反馈值） */
    
    float error;        /* 当前误差 */
    float last_error;   /* 上次误差 */
    float integral;     /* 误差积分累加值 */
    
    float output;       /* PID输出值 */
    float output_max;   /* 输出上限 */
    float output_min;   /* 输出下限 */
    
    float integral_max; /* 积分上限（防止积分饱和） */
} PID_TypeDef;

/**
 * @brief 初始化PID控制器
 * @param pid: PID结构体指针
 * @param Kp: 比例系数
 * @param Ki: 积分系数
 * @param Kd: 微分系数
 * @param out_max: 输出上限
 * @param out_min: 输出下限
 */
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float out_max, float out_min);

/**
 * @brief 设置PID目标值
 * @param pid: PID结构体指针
 * @param target: 目标值
 */
void PID_SetTarget(PID_TypeDef *pid, float target);

/**
 * @brief PID计算（位置式）
 * @param pid: PID结构体指针
 * @param actual: 当前实际值
 * @return PID输出值
 */
float PID_Calculate(PID_TypeDef *pid, float actual);

/**
 * @brief 复位PID控制器
 * @param pid: PID结构体指针
 */
void PID_Reset(PID_TypeDef *pid);

#endif /* __PID_H */
