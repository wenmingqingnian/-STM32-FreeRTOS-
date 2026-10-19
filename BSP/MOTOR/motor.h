#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"




#include "gpiox.h"

/*扩展作用域*/
extern volatile float speed;                /* 实际转速 */
extern volatile int overflow;               /* 编码器溢出计数器 */

/*电机转向枚举类型*/
typedef enum{
	stright=0,
	invert
}direction;

/*********函数声明*************************/
void io_set(gpioled port,u16 pin,GPIOMode_TypeDef mode);
void TIM1_dead_pwm_init(u16 arr,u16 psc,u16 ccr,u16 dtg);
void motor_stop(void);
void motor_start(void);
void motor_dir(direction para);
void motor_speed(u16 ccr);
void TIM2_encode_init(u16 arr, u16 psc);
void TIM4_init(u16 arr,u16 psc);
int get_encoder_value(void);
u8 TIM_GetDirection(TIM_TypeDef* TIMx);
float get_speed(int encode_value,u16 ms);
void motor_pwm_set(float para);
void motor_init(void);
/**********函数声明**************************/

#endif
