#include "sys.h"
#include "usart.h"	  

#if SYSTEM_SUPPORT_OS

#include "FreeRTOS.h"					/*freertosʹ��--�����ĵ�����ע*/
#include "task.h"
#include "app_tasks.h"     /* App_GetKeyScanTaskHandle (RX line-complete wake) */

#endif
/*
 * ���ڳ�ʼ��ģ��
 * ��������ԭ�Ӵ����޸�
 *
 * �޸����ڣ�2026/2/1
 * ��Ŀ�������Ȩ����������
 */
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*ʹ��microLib�ķ���*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  
  
void uart_init(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��

	//USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9

	//USART1_RX	  GPIOA.10��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

	/*����Ŀû�õ������жϣ�����±ߵ��ж����ȼ����������--�����ĵ�����ע*/
	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

	//USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

	USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
	/*��ʹ�ô���DMA����ʱ����ʱ��Ҫ�رմ��ڽ����жϣ����ע�͵�--�����ĵ�����ע*/
    /* Task10: RX path uses RXNE interrupt to receive "IAP" update command
	   (line assembler in USART1_IRQHandler below). The DMA RX request
	   line was never served by any DMA channel in this APP, so it is
	   disabled; the Boot project has its own independent UART DMA driver. */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	
	//USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	/*Task10: disabled, RX is RXNE-interrupt driven now*/	/*ʹ�ܴ���DMA����--�����ĵ�����ע*/ 
	USART_Cmd(USART1, ENABLE);                   	 //ʹ�ܴ���1 

}

void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
/**************************************************************/
/*��ʹ��FreeRtosʱ������Ҫ�����������ע�͵�--�����ĵ�����ע*/
//#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
//	OSIntEnter();    
//#endif
/**************************************************************/
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		{
		Res =USART_ReceiveData(USART1);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
			{
			if(USART_RX_STA&0x4000)//���յ���0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else {USART_RX_STA|=0x8000;	//��������� 
				/* RX line complete -> wake KeyScanTask,
				 * the only App_PollIapCommand() consumer. KeyScanTask blocks
				 * portMAX_DELAY in standby; EXTI-only wake would starve the
				 * IAP update command (default mode after boot is standby).
				 * USART1 preemption prio 3 == configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
				 * -> FromISR API legal. NULL handle (KeyScanTask not created yet,
				 * scheduler not started): drop notify; the line is consumed by
				 * the first App_PollIapCommand() pass before the first take. */
				{
				TaskHandle_t hKeyScan = App_GetKeyScanTaskHandle();
				if (hKeyScan != NULL)
					{
					BaseType_t xHigherPriorityTaskWoken = pdFALSE;
					vTaskNotifyGiveFromISR(hKeyScan, &xHigherPriorityTaskWoken);
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
					}
				}
				}
				}
			else //��û�յ�0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}   		 
     } 
/**********************************************************/
/*��ʹ��FreeRtosʱ������Ҫ�����������ע�͵�--�����ĵ�����ע*/
//#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
//	OSIntExit();  											 
//#endif
/***************************************************/
} 
#endif	

