/*
 * 按键驱动源文件（状态机消抖）
 * 按键1: PB1 - 模式切换（短按切换模式）
 * 按键2: PB12 - 档位切换/风机开关（短按切换档位，长按开关风机）
 * 
 * 特点：
 * 1. 非阻塞状态机消抖，不使用延时函数
 * 2. 支持短按和长按检测
 * 3. 长按持续触发功能（用于蜂鸣器持续鸣叫）
 *
 */
#include "key.h"


/* 按键实例 */
static Key_t g_key1;
static Key_t g_key2;

/* 读取按键1电平 */
static u8 Key1_ReadPin(void)
{
    return (KEY1_READ() == KEY_PRESSED_LEVEL) ? 1 : 0;
}

/* 读取按键2电平（KEY2_ENABLED=0 时编译剔除：PB12 归 TIM1_BKIN 刹车预留，
 * 双重所有权说明见 CONFIG/board_pins.h） */
#if KEY2_ENABLED
static u8 Key2_ReadPin(void)
{
    return (KEY2_READ() == KEY_PRESSED_LEVEL) ? 1 : 0;
}
#endif

/**
 * @brief 初始化按键GPIO
 */
void Key_Init(void)
{
    
	/*初始化按键KEY1*/
	io_set(KEY1_GPIO_PORT,KEY1_GPIO_PIN,GPIO_Mode_IPU);

	/*初始化按键KEY2（KEY2_ENABLED=0 时跳过：不触碰 PB12，引脚归刹车预留）*/
#if KEY2_ENABLED
	io_set(KEY2_GPIO_PORT,KEY2_GPIO_PIN,GPIO_Mode_IPU);
#endif

    /* 初始化按键1结构体 */
    g_key1.state = KEY_STATE_IDLE;
    g_key1.pressStartTick = 0;
    g_key1.readPin = Key1_ReadPin;
    g_key1.event = KEY_EVENT_NONE;
    g_key1.longPressTriggered = 0;

#if KEY2_ENABLED
    /* 初始化按键2结构体（禁用时保持静态零初始化：状态 IDLE、事件 NONE，
     * Key2_GetEvent() 恒返回 KEY_EVENT_NONE，应用层无感）*/
    g_key2.state = KEY_STATE_IDLE;
    g_key2.pressStartTick = 0;
    g_key2.readPin = Key2_ReadPin;
    g_key2.event = KEY_EVENT_NONE;
    g_key2.longPressTriggered = 0;
#endif
}

/**
 * @brief 按键状态机处理（单个按键）
 * @param key: 按键结构体指针
 * @note 使用xTaskGetTickCount()实现精确时间判断，不受调用周期影响
 */
static void Key_StateMachine(Key_t *key)
{
    u8 keyPressed = key->readPin();
    TickType_t currentTick = xTaskGetTickCount();//记录rtos时钟节拍数，每1ms记录一次，此时时钟节拍数加1
    TickType_t elapsedTime;
    
    switch (key->state)
    {
        case KEY_STATE_IDLE:
            /* 空闲状态：检测到按下则进入消抖状态 */
            if (keyPressed)
            {
                key->state = KEY_STATE_DEBOUNCE;
                key->pressStartTick = currentTick;  /* 记录按下开始时间 */
            }
            break;
            
        case KEY_STATE_DEBOUNCE:
            /* 消抖状态：等待消抖时间到达后再检测按键状态 */
            elapsedTime = currentTick - key->pressStartTick;
            if (elapsedTime >= pdMS_TO_TICKS(KEY_DEBOUNCE_TIME_MS))
            {
                /* 消抖时间已到，此时再判断按键状态 */
                if (keyPressed)
                {
                    /* 消抖完成，确认按下 */
                    key->state = KEY_STATE_PRESSED;
                    key->pressStartTick = currentTick;  /* 重新记录时间，用于长按判断 */
                }
                else
                {
                    /* 消抖后按键未按下，视为抖动，返回空闲状态 */
                    key->state = KEY_STATE_IDLE;
                }
            }
            /* 消抖时间未到，忽略当前读数，继续等待 */
            break;
            
        case KEY_STATE_PRESSED:
            /* 按下确认状态：等待长按或释放 */
            if (keyPressed)
            {
                elapsedTime = currentTick - key->pressStartTick;
                if (elapsedTime > pdMS_TO_TICKS(KEY_LONG_PRESS_TIME_MS))
                {
                    /* 达到长按时间 */
                    key->state = KEY_STATE_LONG_PRESS;
                    key->event = KEY_EVENT_LONG_PRESS;
                    key->longPressTriggered = 1;	/*标记一次长按*/
                }
            }
            else
            {
                /* 释放按键 */
                if (!key->longPressTriggered)
                {
                    /* 未触发长按，则为短按 */
                    key->event = KEY_EVENT_SHORT_PRESS;
                }
                key->state = KEY_STATE_IDLE;
                key->longPressTriggered = 0;
            }
            break;
            
        case KEY_STATE_LONG_PRESS:
            /* 长按状态：持续检测，产生长按持续事件 */
            if (keyPressed)
            {
                /* 长按持续中，每个周期都产生持续事件 */
                key->event = KEY_EVENT_LONG_PRESSING;
            }
            else
            {
                /* 释放按键 */
                key->event = KEY_EVENT_RELEASE;
                key->state = KEY_STATE_IDLE;
                key->longPressTriggered = 0;
            }
            break;
            
        default:
            key->state = KEY_STATE_IDLE;
            break;
    }
}

/**
 * @brief 按键扫描函数（需周期性调用）
 * @note 
 */
void Key_Scan(void)
{
    Key_StateMachine(&g_key1);
#if KEY2_ENABLED
    Key_StateMachine(&g_key2);
#endif
}

/**
 * @brief 获取按键1事件
 * @return 按键事件类型
 */
KeyEvent_t Key1_GetEvent(void)
{
    return g_key1.event;
}

/**
 * @brief 获取按键2事件
 * @return 按键事件类型
 */
KeyEvent_t Key2_GetEvent(void)
{
    return g_key2.event;
}

/**
 * @brief 清除按键1事件
 */
void Key1_ClearEvent(void)
{
    g_key1.event = KEY_EVENT_NONE;
}

/**
 * @brief 清除按键2事件
 */
void Key2_ClearEvent(void)
{
    g_key2.event = KEY_EVENT_NONE;
}

/**
 * @brief 检查按键1是否处于按下状态
 * @return 1:按下; 0:未按下
 */
u8 Key1_IsPressed(void)
{
    return (g_key1.state == KEY_STATE_PRESSED || 
            g_key1.state == KEY_STATE_LONG_PRESS) ? 1 : 0;
}

/**
 * @brief 按键1状态机处理中（非 IDLE：消抖/按下/长按判定进行时）
 * @return 1:处理中; 0:空闲
 * @note KeyScanTask 待机态阻塞策略的豁免信号：
 *       处理中必须保持 10ms 有限超时推进消抖/长按计时，
 *       否则待机 + portMAX_DELAY 会把按键卡死在消抖状态。
 */
u8 Key1_InProgress(void)
{
    return (g_key1.state != KEY_STATE_IDLE) ? 1 : 0;
}

/**
 * @brief 检查按键2是否处于按下状态
 * @return 1:按下; 0:未按下
 */
u8 Key2_IsPressed(void)
{
    return (g_key2.state == KEY_STATE_PRESSED || 
            g_key2.state == KEY_STATE_LONG_PRESS) ? 1 : 0;
}
