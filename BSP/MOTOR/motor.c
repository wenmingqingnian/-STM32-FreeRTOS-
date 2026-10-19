#include "motor.h"
#include "gpiox.h"
#include "board_pins.h"
/***********************
*
*
*当前市面基本是HAL库编写的电机驱动，本驱动为手敲标准库，如果需要hal库，请查看相关资料
************************/  
 	 			
/*-----------------------------------------------------------------------*/
/**该函数是带刹车输入的电机驱动初始化函数，但在项目中未用到该函数，
**此处仅为扩展，提供给用户使用；因此此处先将其注释掉。
**
**【PB12 双重所有权】
**本示例把 PB12 初始化为 TIM1_BKIN 刹车输入，与 KEY2 按键（key.c）声明
**同一引脚。PB12 的默认 owner 是本刹车预留；KEY2 已退为编译开关
**KEY2_ENABLED（默认 0，见 CONFIG/board_pins.h）。若要启用本示例，
**必须保持 KEY2_ENABLED=0，并先完成原理图确认。**/
/*-----------------------------------------------------------------------*/
#if 0
/*
 * @brief: 带刹车功能的死区互补PWM初始化
 * @param: u16 arr - 自动重装载值
 * @param: u16 psc - 预分频值
 * @param: u16 ccr - 比较值(占空比)
 * @param: u16 dtg - 死区时间
 * @return: none
 */
void TIM1_dead_pwm_init(u16 arr,u16 psc,u16 ccr,u16 dtg)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	// 使能定时器1时钟
 	io_set(MOTOR_PWM_GPIO_PORT,MOTOR_PWM_GPIO_PIN,GPIO_Mode_AF_PP);			// CH1输出引脚
 	io_set(MOTOR_CH1N_GPIO_PORT,MOTOR_CH1N_GPIO_PIN,GPIO_Mode_AF_PP);			// CH1N互补输出引脚
 	io_set(MOTOR_BRAKE_GPIO_PORT,MOTOR_BRAKE_GPIO_PIN,GPIO_Mode_IPU);			// 刹车输入IO初始化，上拉输入
	
	
   // 初始化TIM1时基单元
	TIM_TimeBaseStructure.TIM_Period = arr; 					// 设置在下一个更新事件装入自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 					// 设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4; 	// CKD[1:0] = 10, tDTS = 4 * tCK_INT
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  // TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 			 // 根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
		 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; // 选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High; // 互补输出极性:TIM输出比较极性高
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;	// 互补输出使能
	TIM_OCInitStructure.TIM_Pulse = ccr;							// 设置占空比
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  					// 根据指定的参数初始化TIM1 OC1
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  			// 使能TIM1在CCR上的预装载寄存器
	
	
	/* 配置刹车和死区 */
	TIM_BDTRStructInit(&TIM_BDTRInitStructure);									// 将BDTR初始为默认值后初始化
	TIM_BDTRInitStructure.TIM_DeadTime=dtg;										// 设置死区时间
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;							// 使能刹车功能
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low; 			// BKIN低电平触发刹车
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;		// 使能AOE位，刹车后自动恢复输出
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);								// BDTR初始化
	TIM_CtrlPWMOutputs(TIM1, ENABLE);											// 使能MOE位
	TIM_Cmd(TIM1, ENABLE);  													// 使能TIM1
}
#endif



/*
 * @brief: 直流有刷电机驱动
 * @param: u16 arr - 自动重装载值
 * @param: u16 psc - 预分频值
 * @param: u16 ccr - 比较值(占空比)
 * @param: u16 dtg - 死区时间
 * @return: none
 */
void TIM1_dead_pwm_init(u16 arr,u16 psc,u16 ccr,u16 dtg)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	// 使能定时器1时钟
 	io_set(MOTOR_PWM_GPIO_PORT,MOTOR_PWM_GPIO_PIN,GPIO_Mode_AF_PP);			// CH1输出引脚
 	io_set(MOTOR_CH1N_GPIO_PORT,MOTOR_CH1N_GPIO_PIN,GPIO_Mode_AF_PP);			// CH1N互补输出引脚
 	io_set(MOTOR_EN_GPIO_PORT,MOTOR_EN_GPIO_PIN,GPIO_Mode_Out_PP);				// 使能控制IO初始化，推挽输出
	
	
	// 初始化TIM1时基单元
	TIM_TimeBaseStructure.TIM_Period = arr; 					// 设置在下一个更新事件装入自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 					// 设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4; 	// CKD[1:0] = 10, tDTS = 4 * tCK_INT
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 			// 根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
		 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 			// 选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 	// 输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High; 	// 互补输出极性:TIM输出比较极性高
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	// 比较输出使能
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;	// 互补输出使能
	TIM_OCInitStructure.TIM_Pulse = ccr;							// 设置初始占空比
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  						// 根据指定的参数初始化TIM1 OC1
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  				// 使能TIM1在CCR上的预装载寄存器
	
	
	/* 配置BDTR寄存器 */
	TIM_BDTRStructInit(&TIM_BDTRInitStructure);									// 将BDTR初始为默认值后初始化
	TIM_BDTRInitStructure.TIM_DeadTime=dtg;										// 设置死区时间
	TIM_BDTRInitStructure.TIM_OSSRState=TIM_OSSRState_Enable;					// OSSR位设置为1
	TIM_BDTRInitStructure.TIM_OSSIState=TIM_OSSIState_Disable;					// OSSI位设置为0
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;						// 注意：在本例程中没有用到刹车
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low; 			// BKIN低电平触发刹车
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;		// 使能AOE位，刹车后自动恢复输出
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);								// BDTR初始化
	TIM_CtrlPWMOutputs(TIM1, ENABLE);											// 使能MOE位
	TIM_Cmd(TIM1, ENABLE);  													// 使能TIM1
}


/* 电机停止 */
void motor_stop(void)
{
	/* 电机停转后测速无意义：停TIM4并清残留中断，消除1ms常驻唤醒源 */
	TIM_Cmd(TIM4, DISABLE);
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	NVIC_ClearPendingIRQ(TIM4_IRQn);
	TIM_CCxCmd(TIM1,TIM_Channel_1,TIM_CCx_Disable);		// 关闭CH1通道PWM输出
	TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Disable); 	// 关闭CH1N通道PWM输出
	io_reset_bit(MOTOR_EN_GPIO_PORT,MOTOR_EN_GPIO_PIN);						//禁止驱动H桥电路
}

/* 电机启动 */
void motor_start(void)
{
	/* 重启测速定时器：计数清零+清残留中断后使能，恢复1ms测速 */
	TIM_SetCounter(TIM4, 0);
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	NVIC_ClearPendingIRQ(TIM4_IRQn);
	TIM_Cmd(TIM4, ENABLE);
	io_set_bit(MOTOR_EN_GPIO_PORT,MOTOR_EN_GPIO_PIN);						//使能驱动H桥电路
}

/* 电机方向控制 */
void motor_dir(direction para)
{
	TIM_CCxCmd(TIM1,TIM_Channel_1,TIM_CCx_Disable);		// 关闭CH1通道PWM输出
	TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Disable); 	// 关闭CH1N通道PWM输出

    if (para == stright)                /* 正转 */
    {
		TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Enable);     // 开启互补通道输出
    } 
    else if (para == invert)           /* 反转 */
    {
		TIM_CCxCmd(TIM1,TIM_Channel_1,TIM_CCx_Enable);     // 开启CCX通道输出 
    }
}


/*电机状态初始化（正确性修复）
 *上电后必须保持H桥关闭（PA2拉低）、PWM通道关闭、比较值为0，
 *禁止在初始化阶段使能功率级；H桥只能由电机控制任务在需要运行时
 *通过motor_start()使能，避免上电瞬间电机意外转动 */
void motor_init(void)
{
	TIM_SetCompare1(TIM1, 0);		/* PWM比较值清零 */
	motor_dir(stright);			/* 默认正转方向 */
	motor_stop();				/* 关闭PWM通道并禁止H桥（PA2拉低） */
}


/* 电机速度调节 */
void motor_speed(u16 ccr)
{
	if(ccr<=1000)	/* 限幅 */
	{
		TIM_SetCompare1(TIM1,ccr);	// 设置占空比
	}
}




/*电机控制*/
void motor_pwm_set(float para)
{
    int val = (int)para;

    if (val >= 0) 
    {
        motor_dir(stright);           /* 正转 */
        motor_speed(val);
    } 
    else 
    {
        motor_dir(invert);           /* 反转 */
        motor_speed(-val);
    }
}

/******************************************** 以下为编码器测速部分代码 *****************************************/
/* TIM2编码器功能初始化 */
void TIM2_encode_init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef  TIM_ICInitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
	
	/* 使能TIM2时钟和GPIO */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	// 使能定时器2时钟
	io_set(MOTOR_ENC_A_GPIO_PORT, MOTOR_ENC_A_GPIO_PIN, GPIO_Mode_IPD);		// 初始化TIM2_CH1
	io_set(MOTOR_ENC_B_GPIO_PORT, MOTOR_ENC_B_GPIO_PIN, GPIO_Mode_IPD);		// 初始化TIM2_CH2
	
	/* 配置定时器参数 */
	TIM_TimeBaseStructure.TIM_Period = arr; 				//占空比
    TIM_TimeBaseStructure.TIM_Prescaler = psc;       		//分频系数
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//时钟不分割
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	/* 配置编码器，此处选择4倍频，需要TI1(A相)和TI2(B相)进行计数 */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;				// 选择CH1			
	TIM_ICInitStructure.TIM_ICFilter = 10;							// 输入滤波器
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 	// 选择是否反相，不反相
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	// 选择输入通道，对应寄存器CCIS[1:0]
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			// 不分频，每一个边沿触发一次捕获
	TIM_ICInit(TIM2,&TIM_ICInitStructure);							// 初始化TIM2输入捕获接口
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;				// 选择CH2			
	TIM_ICInitStructure.TIM_ICFilter = 10;							// 输入滤波器
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 	// 选择是否反相，不反相
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	// 选择输入通道，对应寄存器CCIS[1:0]
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			// 不分频，每一个边沿触发一次捕获
	TIM_ICInit(TIM2,&TIM_ICInitStructure);							// 初始化TIM2输入捕获接口
	
	/* 配置编码器接口，选择TI1和TI2同时计数 */
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	/* 配置TIM2中断，因为计数器为65536，可能会溢出所以配置中断 */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  				// TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;  		// 抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  			// 子优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 				// IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  								// 根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); 						// 使能指定的TIM2中断，允许更新中断

	TIM_Cmd(TIM2, ENABLE);  										// 使能TIM2
	
}



/* 定时器Timer4初始化，用于计算转速 */
void TIM4_init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 			// 时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; 						// 设置在下一个更新事件装入自动重装载寄存器周期的值	 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 						// 设置用来作为TIMx时钟频率除数的预分频值  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 					// 设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	// TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 				// 根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); 						// 使能指定的TIM4中断，允许更新中断
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  				// TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;  		// 抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  			// 子优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 				// IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  								// 根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM4, ENABLE);  										//使能定时器						 
}


/* 获取编码器计数值（无锁一致快照，正确性修复）
 * 竞态说明：overflow 由TIM2溢出中断更新，counter 由硬件计数器更新。
 * 若读取counter前后overflow发生变化，说明快照不一致，必须重读，
 * 否则拼出的32位计数值会偏差65536，导致RPM计算出现尖刺 */
int get_encoder_value(void)
{
	u16 cnt;
	int ovf_before, ovf_after;

	do
	{
		ovf_before = overflow;			/* 第一次读取溢出计数 */
		cnt = TIM_GetCounter(TIM2);		/* 读取16位硬件计数器 */
		ovf_after = overflow;			/* 第二次读取溢出计数 */
	} while (ovf_before != ovf_after);	/* 不一致则说明期间发生溢出中断，重读 */

	return (int)cnt + ovf_before * 65536;
}

/* 获取计数方向 */
u8 TIM_GetDirection(TIM_TypeDef* TIMx)
{
    return (TIMx->CR1 & TIM_CR1_DIR) ? 1 : 0;	/*在编码器模式下，定时器的计数方向变为只读*/
}


/* 获取电机转速。ms=50表示每50ms计算一次速度，即采样精度为50ms
 * 【测速滤波升级：滚动窗口 + 启动预热】
 * 原版批次式：攒满10个原始值才滤波一次（500ms才更新一次speed，启动前500ms speed=0）
 * 本版滚动窗口：每50ms新值进、最旧值出，每50ms滤波一次，speed实时更新；
 * 启动预热：样本不足时用已收集样本均值直接输出，消除speed=0真空期；
 * 满窗统计口径与原版一致：去2最大2最小取中间6个平均后再做一阶低通 */
#define SPEED_FILTER_N  10u          /* 滤波窗口：10个原始值 */

float get_speed(int encode_value,u16 ms)
{
	u8 i = 0, j = 0;
    float t = 0.0, raw = 0.0, mid = 0.0, sum = 0.0;
	static float speed=0;									/*需要为静态变量，因为一阶滤波算法需要上次的滤波值 */
    static uint8_t sp_count = 0, n = 0;
    static float raw_arr[SPEED_FILTER_N] = {0.0};           // 原始速度滑动窗口
	static int old_value=0,now_value=0;
    float tmp[SPEED_FILTER_N];                              // 排序副本（任务栈约40B）

    if (sp_count == ms)                                     // 每50ms计算一次速度
    {
		now_value = encode_value;							// 记录当前编码器值
		
        // 计算转速，30为减速比，4倍频，11线
		
		/* 1000/ms指在这个ms时间内获得了xx脉冲变化值，用xx变化值/ms得到1ms的脉冲变化值
		 * 再乘以1000得到1s的变化值 */
        raw = (float)((now_value - old_value) * ((1000 / ms) * 60.0) / 30 / (11*4));
		old_value = now_value;								// 保存当前计数值

        /* 滚动窗口：样本不足直接追加（预热），满了新值入尾、最旧值出 */
        if (n < SPEED_FILTER_N)
        {
            raw_arr[n++] = raw;
        }
        else
        {
            for (i = 0; i < (SPEED_FILTER_N - 1); i++)
            {
                raw_arr[i] = raw_arr[i + 1];
            }
            raw_arr[SPEED_FILTER_N - 1] = raw;
        }

        /* 中值滤波：对窗口副本排序，按样本数分档去首尾取中间平均 */
        for (i = 0; i < n; i++) tmp[i] = raw_arr[i];
        for (i = n; i >= 1; i--)
        {
            for (j = 0; j < (i - 1); j++)
            {
                if (tmp[j] > tmp[j + 1])                    /* 数值比较 */
                {
                    t = tmp[j];                             /* 数值换位 */
                    tmp[j] = tmp[j + 1];
                    tmp[j + 1] = t;
                }
            }
        }

        sum = 0.0f;
        if (n >= 7)                                         /* 满窗：去2最大2最小，中间n-4个平均 */
        {
            for (i = 2; i < (n - 2); i++) sum += tmp[i];
            mid = sum / (n - 4);
        }
        else if (n >= 5)                                    /* 过渡：去1最大1最小 */
        {
            for (i = 1; i < (n - 1); i++) sum += tmp[i];
            mid = sum / (n - 2);
        }
        else                                                /* 预热：全平均直接输出，不喂0 */
        {
            for (i = 0; i < n; i++) sum += tmp[i];
            mid = sum / n;
        }

        /* 一阶低通：预热期直接取mid（消除speed=0真空），满窗后平滑 */
        if (n < SPEED_FILTER_N)
        {
            speed = mid;
        }
        else
        {
            speed = (float)(0.48f * mid + (speed * 0.52f));
        }
        sp_count = 0;
    }
    sp_count ++;
	return speed;
}


/* 注意：TIM2_IRQHandler和TIM4_IRQHandler已移至app_tasks.c中
 * 原因：中断服务程序需要访问FreeRTOS信号量，统一放在应用层文件中管理 */

