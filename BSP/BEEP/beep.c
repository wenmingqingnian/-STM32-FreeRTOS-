/*
 * 有源蜂鸣器驱动源文件
 * 引脚: PB15
 * 有源蜂鸣器：高电平响，低电平不响
 *
 */
#include "beep.h"
#include "usart.h"	
#include "delay.h"

 
 
/***-----------------------------------------------------------------------**/
/**
* @brief: 初始化BEEP的io口
* @param:结构体变量
* @param:端口（GPIOA-G）
* @param:管脚（0-16）
*/
void Beep_Init(led_d *io,gpioled port,u16 pin)
{
	if(port==GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	else if(port==GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	else if(port==GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	else if(port==GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	else if(port==GPIOE) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	else if(port==GPIOF) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
	else if(port==GPIOG) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);

	
	io->port=port;
	io->pin=pin;
	
	Beep_config(io);
}
/***-----------------------------------------------------------------------**/


/***-----------------------------------------------------------------------**/
/** 
* @brief:  配置io口
* @param:  蜂鸣器结构体变量
* @return: none
**/ 
void Beep_config(led_d *io)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = io->pin;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(io->port, &GPIO_InitStructure);					
	Beep_off(io);
}
/***-----------------------------------------------------------------------**/


/***-----------------------------------------------------------------------**/
/** 
* @brief:  关闭蜂鸣器
* @param:  蜂鸣器结构体变量
* @return: none
**/ 
void Beep_off(led_d *io)
{
	GPIO_ResetBits(io->port,io->pin);
}
/***-----------------------------------------------------------------------**/


/***-----------------------------------------------------------------------**/
/** 
* @brief:  打开蜂鸣器
* @param:  蜂鸣器结构体变量
* @return: none
**/ 
void Beep_on(led_d *io)
{
	GPIO_SetBits(io->port,io->pin);
}
/***-----------------------------------------------------------------------**/

/**
 * @brief 蜂鸣器鸣叫指定时间
 * @param duration_ms: 鸣叫时间，单位毫秒
 * @note 
 */
void Buzzer_Beep(u16 duration_ms,led_d *io)
{
    Beep_on(io);
    delay_ms(duration_ms);
    Beep_off(io);
}

