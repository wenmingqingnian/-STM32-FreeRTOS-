#ifndef __BEEP_H
#define __BEEP_H	 
#include "sys.h"
#include "gpiox.h"

/*
 * 有源蜂鸣器驱动头文件
 * 引脚: PB15
 *
 */ 

/*扩展作用域--注*/
extern  led_d bep;


void Beep_Init(led_d *io,gpioled port,u16 pin);
void Beep_config(led_d *io);
void Beep_off(led_d *io);
void Beep_on(led_d *io);
void Buzzer_Beep(u16 duration_ms,led_d *io);
#endif
