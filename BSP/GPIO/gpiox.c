#include "gpiox.h"


/* GPIO初始化函数 */
void io_set(gpioled port,u16 pin,GPIOMode_TypeDef mode)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	if(port==GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	else if(port==GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	else if(port==GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	else if(port==GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	else if(port==GPIOE) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	else if(port==GPIOF) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
	else if(port==GPIOG) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = pin;  
	GPIO_InitStructure.GPIO_Mode = mode;  		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(port, &GPIO_InitStructure);			// 初始化GPIO
}

/* 设置GPIO引脚电平为高 */
void io_set_bit(gpioled port,u16 pin)
{
	GPIO_SetBits(port,pin);
}

/* 设置GPIO引脚电平为低 */
void io_reset_bit(gpioled port,u16 pin)
{
	GPIO_ResetBits(port,pin);
}
