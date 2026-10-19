#ifndef __DHT11_H
#define __DHT11_H 			   

#include "sys.h"
#include "stm32f10x.h"
#include "delay.h"
#include "gpiox.h"

extern led_d dht;       /*扩展作用域*/
	 
 void DHT11_Start(led_d *io);
 void DHT11_Read(led_d *io);
 u8 DHT_Read_Byte(led_d *io);
 u8 DHT_Read_Data(u8 *temp,u8 *humi,gpioled port,u16 pin,led_d *io);
 void chushi(led_d *io);
 u8 readpin(led_d *io);
#endif
