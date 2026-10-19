/*
 * ���̻�����ϵͳ������
 * ����STM32F103C8T6 + FreeRTOS
 * 
 * ����˵����
 * 1. ����ģʽ�����ֹͣ�������������
 * 2. �ֶ�ģʽ���ֶ����٣�PID����
 * 3. �Զ�ģʽ�����ݴ������Զ����ڷ���
 * 4. ������ģʽ������Ũ�ȳ���ֵʱ�������
 * 5. �̼����£�ͨ��usart+dma���չ̼���������ɺ���ת��app����boot+app˫���ܹ�
 * 6. UI��ʾ����
 * 7.���������ٹ��ܣ�ͨ��1ms�жϼ���������ȡ�ļ����仯����������ת��
 * 8.ֱ����ˢ����������ܣ�ͨ��PWM�����������H��
 * 9.CRC32���鹦�ܣ�ͨ��python�ű�Ԥ����app.bin������CRC32�����룬���ڹ̼�����
 * �������ܣ�
 * - ����1(PB1)���̰��л�ģʽ
 * - ����2(PB12)���̰��л���λ���������ط��
 *
 *�޸����ڣ�2026/2/1
 *��Ŀ�������Ȩ����������
 */
#include "main.h"
#include "dwt_profiler.h"   /* DWT_INIT macro (empty when disabled) */
#include "reset_cause.h"    /* RCC CSR reset cause capture/report */
#include "watchdog.h"       /* IWDG init (empty when WATCHDOG_ENABLED=0) */

/*-----------------------------------------------------------
 * Ӳ����ʼ������
 *----------------------------------------------------------*/
static void Hardware_Init(void)
{
	/************************************************************************************************************/
	/*���ڹ���4���жϣ������жϷ���Ϊ4��4λ��ռ���ȼ���0λ�����ȼ�����ΪFree��֧�������ȼ�������
	 ����Free�д����������ȼ���ֵ�ĸ���ù��̱�������Ϊ3�����������Ҫ����API�������ж����ȼ����ܳ���3��
	 ����DMA�ж����ȼ�Ϊ4����ʱ��4�ж����ȼ�Ϊ5��TIM2�ж����ȼ�Ϊ6�������ȼ�������Ϊ0--�����ĵ�����ע*/
	/*************************************************************************************************************/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	/* ��ʱ������ʼ�� */
    delay_init();
	
	/* ���PWM��ʼ��
     * TIM1: PWMƵ�� = 72MHz / 72 / 1000 = 1kHz
     * ����ʱ��: 100 * 16/72MHz �� 22us
     */
    TIM1_dead_pwm_init(1000-1, 72-1, 0, 100);
    motor_init();

    /* ��������ʼ����TIM2������Ƶ��72MHZ�ļ���Ƶ��,ARRΪ���ֵ*/
    TIM2_encode_init(0xFFFF, 0);
	
	/*��ʼ��������*/
	Beep_Init(&bep,GPIOB,GPIO_Pin_15);			 
    
    /* ������ʼ�� */
    Key_Init();
    
    /* MQ2���崫������ʼ�� */
    MQ2_Init();
	
	/* PID��������ʼ��
     * ����˵����
     * Kp=14.0:   ����ϵ��
     * Ki=1.65:  ����ϵ��
     * Kd=0.00: ΢��ϵ��
     * ����޷���0~1000
     */
    PID_Init(&g_speedPID, 14.0f, 1.65f, 0.0f, 1000.0f, 0.0f);
	
    /* �����㷨ģ���ʼ�� */
    WindSpeed_Init();
    
    /* LCD��ʼ�� */
    LCD_Init();
	
	/* ���ڳ�ʼ�� */
    uart_init(115200);		/*������ں�߳�ʼ���������Ӱ����ת��*/

    /* ��ʾ��ʼ����� */
    Show_Str(0, 0, BLUE, WHITE, "Init Complete! ", 16, 0);
    delay_ms(500);
}


/*-----------------------------------------------------------
 * ������
 *----------------------------------------------------------*/
int main(void)
{
    /* capture RCC CSR reset flags FIRST, before any peripheral
     * init can touch/clear them; cause is latched in RAM, printed after
     * uart_init() via ResetCause_Report() below. */
    ResetCause_Init();

    /* Ӳ��ʼ�� */
    Hardware_Init();
    ResetCause_Report();    /* boot reset-cause log (uart ready) */
    DWT_INIT();    /* ����A ��4����DWT CYCCNT ��ʼ�����ر�ʱΪ�պ��*/
    
    /* ϵͳ״̬��ʼ�� */
    System_Init();

    /* �����ʼ�� */
    StartTask_Create();

    /* start IWDG before the scheduler (no-op when disabled);
     * once enabled only HealthTask feeds it after heartbeat verdicts */
    Watchdog_Init();

    /* ����FreeRTOS������� */
    vTaskStartScheduler();
    
    /* ��������²���ִ�е����� */
    while(1)
    {
    }
}


