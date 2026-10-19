#ifndef __GPIOX_H
#define __GPIOX_H

#include "sys.h"

typedef GPIO_TypeDef*   gpioled;

/*gpio数据结构*/
typedef struct{
		gpioled port;
		uint16_t pin;
}led_d;


/********函数声明*******/
void io_set(gpioled port,u16 pin,GPIOMode_TypeDef mode);
void io_set_bit(gpioled port,u16 pin);
void io_reset_bit(gpioled port,u16 pin);
/********函数声明*******/
#endif
