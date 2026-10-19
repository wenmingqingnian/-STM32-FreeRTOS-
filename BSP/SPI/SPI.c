#include "spi.h"

/*写一个字节 */
u8 SPI_WriteByte(SPI_TypeDef* SPIx,u8 Byte)
{
	while((SPIx->STATR&SPI_I2S_FLAG_TXE)==RESET);		//等待发送区空(现在程序卡在这里说明发送区不是空的不能发送)  
	SPIx->DATAR=Byte;	 								//发送一个byte   
	while((SPIx->STATR&SPI_I2S_FLAG_RXNE)==RESET);		//等待接收完一个byte  
	return SPIx->DR;          	     					//返回收到的数据			
} 

/*设置SPI速度 */
void SPI_SetSpeed(SPI_TypeDef* SPIx,u8 SpeedSet)
{
	SPIx->CTLR1&=0XFFC7;
	if(SpeedSet==1)//高速
	{
		SPIx->CTLR1|=SPI_BaudRatePrescaler_2;//Fsck=Fpclk/2	
	}
	else//低速
	{
		SPIx->CTLR1|=SPI_BaudRatePrescaler_32; //Fsck=Fpclk/32
	}
	SPIx->CTLR1|=1<<6; //SPI设备使能
} 

/*初始化SPI1 */
void SPI1_Init(void)	
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	 
	//配置SPI1管脚
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*PA6在该LCD驱动中并没有用到，因为该GPIO是SPI的MISO线
	也就是从机向主机传输数据的线，但在LCD中，不需要LCD向MCU传输数据，
	因此不用该管脚，此处代码中对其初始化，仅仅为了保持SPI的驱动代码完整，无其他作用
	请各用户悉知*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
	//SPI1配置选项
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 ,ENABLE);
	   
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	//使能SPI1
	SPI_Cmd(SPI1, ENABLE);   
}
