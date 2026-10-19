#include "dht11.h"
#include "gpiox.h"

/**********************************************



【引脚所有权】
本驱动不硬编码引脚，port/pin 由调用方传入（app_tasks.c SensorTask 使用
CONFIG/board_pins.h 的 DHT11_GPIO_PORT/DHT11_GPIO_PIN，即 PC13）。
板型已冻结 BOARD_C8_LSE（板载 32.768k 表晶，见 CONFIG/board_profile.h），
PC14/PC15 归 LSE，DHT11 自 PC14 迁至 PC13；
禁止在本驱动内引入其他固定引脚。
**********************************************/   

u8  BUF[5]={0x00,0x00,0x00,0x00,0x00};    //存储读取的温湿度信息
u32 sum=0;         						  //校验值

/***-----------------------------------------------------------------------**/
/** 
* @brief: dht11开始信号
* @param: 结构体变量
* @return:无
**/ 
 void DHT11_Start(led_d *io)  //DHT11数据输入引脚PC13
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	chushi(io);
	/* 修复：推挽输出 → 开漏输出
	 * 推挽输出时，若DHT11拉低总线应答会造成IO口短路
	 * 开漏输出只能拉低，释放后靠外部上拉电阻回高，安全 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; //开漏输出模式
	GPIO_InitStructure.GPIO_Pin = io->pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(io->port, &GPIO_InitStructure);
	GPIO_ResetBits(io->port,io->pin); 					//拉低数据线
	delay_ms(20);    									//拉低至少18ms
	GPIO_SetBits(io->port,io->pin); 					//释放数据线，外部上拉回高
	delay_us(30);     									//主机拉高20~40us
}
/***-----------------------------------------------------------------------**/

/***-----------------------------------------------------------------------**/
/** 
* @brief: 配置io口
* @param: 结构体变量
* @return:无
**/ 
void DHT11_Read(led_d *io)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	chushi(io);
	/* 修复：浮空输入 → 上拉输入
	 * DHT11空闲时总线应为高电平，上拉输入保证空闲态稳定
	 * 同时防止传感器拔掉后总线浮空误触发 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_InitStructure.GPIO_Pin = io->pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(io->port, &GPIO_InitStructure);
}
/***-----------------------------------------------------------------------**/

/***-----------------------------------------------------------------------**/
/** 
* @brief: dht11读一个字节
* @param: 结构体变量
* @return: readDat
**/ 
u8 DHT_Read_Byte(led_d *io)
{
	u8 temp;  
	u8 ReadDat=0; 
	u8 t = 0;
	u8 i; 
 
	for(i=0;i<8;i++)
	{
		while(readpin(io)==0&&t<100)  
		{		
			delay_us(1);
			t++;  //防止卡死
		}
		t=0;
		//由于‘0’代码高电平时间26~28us，1代码高电平时间70us，延时30us，可判断高低电平，数字0读取到的是低电平，高电平则反之
		/* 优化：关键30µs判定窗口关中断
		 * DHT11位判定窗口只有约30µs，若被高优先级任务抢占会导致时序不准、读错位
		 * 仅在这30µs+读引脚的极短窗口关中断，对系统实时性影响极小（每次约35µs）
		 * 采用PRIMASK保存恢复方式，安全兼容任何调用场景 */
		{
			u32 primask = __get_PRIMASK();
			__disable_irq();
			delay_us(30);
			temp = 0;
			if(readpin(io) == 1) temp = 1;
			if(!primask)
			{
				__enable_irq();
			}
		}		
			
		while(readpin(io)==1&&t<100)
		{		
			delay_us(1);
			t++;
		}
		t=0;
		ReadDat<<=1; //左移1位，DHT11数据由高到低发送
		ReadDat|=temp;
	}	
	return ReadDat;
}
/***-----------------------------------------------------------------------**/

/***-----------------------------------------------------------------------**/
/** 
* @brief: dht11采集数据
* @param: 结构体变量
* @return:1成功，0失败
**/ 
u8 DHT_Read_Data(u8 *temp,u8 *humi,gpioled port,u16 pin,led_d *io)
{
	u8 i;
	u8 t = 0;
	io->port=port;
	io->pin=pin;
	DHT11_Start(io);
	DHT11_Read(io);
	delay_us(20);
	//延时20us，低电平80us，还剩60us，检查是否是低电平以确定是否有响应信号
	if(readpin(io)==0)  //如果读取到低电平，证明DHT11响应
	{
		while(readpin(io)==0&&t<100)//接收响应信号低电平剩余60us，等待变高电平
		{
			delay_us(1);
			t++;			
		}
		t=0;//超过100us自动向下运行，以免卡死
		while(readpin(io)==1&&t<100)//接收响应信号高电平80us，等待变低电平
			{
				delay_us(1);
				t++;			
			}
		t=0;
		for(i=0;i<5;i++)  //接收40位数据
			{
				BUF[i]=DHT_Read_Byte(io);//读出1个字节
			}
		delay_us(50);//结束信号
	}
	sum=BUF[0]+BUF[1]+BUF[2]+BUF[3];		// BUF[0]：湿度整数值 BUF[1]：湿度小数值 BUF[2]：温度整数值 BUF[3]：温度小数值 BUF[4]：校验位，都为8位数据
	if(BUF[4]==(u8)sum)  //校验  
	{
		*humi=BUF[0];
		*temp=BUF[2];
		return 1;    
	}
	else
		return 0;    
}
/***-----------------------------------------------------------------------**/

/***-----------------------------------------------------------------------**/
/** 
* @brief: 时钟选择
* @param: 结构体变量
* @return:无
**/ 
void chushi(led_d *io)
{
		if(io->port==GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
		else if(io->port==GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		else if(io->port==GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		else if(io->port==GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
		else if(io->port==GPIOE) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
		else if(io->port==GPIOF) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
		else if(io->port==GPIOG) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);

}
/***-----------------------------------------------------------------------**/


/***-----------------------------------------------------------------------**/
/** 
* @brief: 读取IO口状态
* @param: 结构体变量
* @return:0代表输入低电平
**/ 
u8 readpin(led_d *io)
{
	return GPIO_ReadInputDataBit(io->port,io->pin);
}
/***-----------------------------------------------------------------------**/

