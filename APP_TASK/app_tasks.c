/*
 * 应用层任务源文件
 * 基于FreeRTOS的油烟机控制系统
 *
 */
#include "app_tasks.h"
#include "delay.h"
#include "gpiox.h"
#include "board_pins.h"
#include "lcd.h"
#include "gui.h"
#include "key.h"
#include "key_exti.h"   /* EXTI 唤醒通道 + 扫描阻塞策略 */
#include "beep.h"
#include "dht11.h"
#include "mq2.h"
#include "motor.h"
#include "pid.h"
#include "wind_speed.h"
#include "crc32.h"
#include "iap_request.h"     /* 升级请求（安全停机->BKP_DR2->复位）*/
#include "standby_policy.h"  /* 待机阻塞决策 + 事件位单一来源 */
#include "watchdog.h"        /* Watchdog_ServiceRequiredNow 绑定 */
#include "lowpower.h"        /* 三路 Tickless/STOP wrapper + 快照源绑定 */
#include <stdio.h>
#include <string.h>
#include "usart.h"


/*-----------------------------------------------------------
 * 全局变量定义
 *----------------------------------------------------------*/
SystemState_t g_systemState;                /* 系统状态 */
SemaphoreHandle_t g_dataMutex = NULL;       /* 数据互斥信号量 */
SemaphoreHandle_t g_speedCalcSemaphore = NULL;  /* 速度计算二值信号量 */

/* 电机命令队列（电机硬件单一所有者：所有任务通过队列请求电机动作）*/
QueueHandle_t g_motorCmdQueue = NULL;
volatile u32 g_motorQueueFullCount = 0;     /* 队列满丢弃/挤占计数（诊断证据）*/

/* 状态事件队列（StateManager单一写者：其他任务经队列请求状态变化）*/
QueueHandle_t g_stateEventQueue = NULL;
volatile u32 g_stateEventFullCount = 0;     /* 事件队列满丢弃计数（诊断证据）*/

/* ISR统计计数（ISR内禁止printf，只允许清标志/计数/FromISR API；
 * 计数由DiagnosticTask周期打印，作为ISR存活与频率的证据）*/
volatile u32 g_tim2OverflowIrqCount = 0;    /* TIM2编码器溢出中断次数 */
volatile u32 g_tim4IrqCount = 0;            /* TIM4周期中断次数（速度计算触发）*/

/* 蜂鸣器命令队列（KeyScan/StateManager只发命令，
 * 阻塞式鸣叫时序由低优先级BuzzerTask异步执行）*/
QueueHandle_t g_buzzerCmdQueue = NULL;
volatile u32 g_buzzerQueueFullCount = 0;    /* 命令队列满丢弃计数（诊断证据）*/

/* 待机广播事件组：StateManager 在模式边沿
 * 置位（进待机 STANDBY_EVT_STANDBY_BIT / 退待机 ACTIVE_BIT + 四个
 * 周期任务独立唤醒位，位定义单一来源 standby_policy.h）；周期任务
 * 待机阻塞在各自唤醒位上，消除待机态全部周期唤醒源。
 * 堆开销约 90B（10K 堆余量已确认）；创建失败走既有
 * vApplicationMallocFailedHook FATAL 路径（heap_4 分配失败回调）*/
EventGroupHandle_t g_standbyEventGroup = NULL;

/* 任务句柄 */
static TaskHandle_t xStartTaskHandle = NULL;
static TaskHandle_t xKeyScanTaskHandle = NULL;
static TaskHandle_t xSensorTaskHandle = NULL;
static TaskHandle_t xWindSpeedTaskHandle = NULL;
static TaskHandle_t xMotorControlTaskHandle = NULL;
static TaskHandle_t xStateManagerTaskHandle = NULL;
static TaskHandle_t xUIDisplayTaskHandle = NULL;
static TaskHandle_t xSpeedCalcTaskHandle = NULL;
static TaskHandle_t xDiagnosticTaskHandle = NULL;    /* EXIT 广播通知目标 */

/* KeyScanTask 句柄访问器——EXTI1（stm32f10x_it.c）与
 * USART1 RX（usart.c）ISR 的通知目标；任务创建前返回 NULL，调用方判空 */
TaskHandle_t App_GetKeyScanTaskHandle(void)
{
    return xKeyScanTaskHandle;
}


/* PID控制器 */
extern PID_TypeDef g_speedPID;

/* 前置声明：SensorTask位于定义之前使用（同文件内部接口）*/
static void State_SendEvent(StateEventType_t type);


/* 自动模式状态机状态 */
typedef enum {
    AUTO_STATE_STARTUP,             /* 启动阶段（等待cooking event） */
    AUTO_STATE_COOKING,             /* Cooking Event激活 */
    AUTO_STATE_DELAY_OFF            /* 延时关闭阶段 */
} AutoModeState_t;

static AutoModeState_t g_autoModeState = AUTO_STATE_STARTUP;

/*-----------------------------------------------------------
 * 模式名称字符串
 *----------------------------------------------------------*/
static const char* ModeNames[] = {
    "Standby",
    "Manual ",
    "Auto   ",
    "Anti-BF"
};

static const char* SpeedLevelNames[] = {
    "LOW ",
    "HIGH"
};

static const char* AutoStateNames[] = {
    "Startup ",
    "Cooking ",
    "DelayOff"
};

/*-----------------------------------------------------------
 * 低功耗快照源适配器（责任表见 lowpower.h）
 * 全部运行时只读真实硬件状态，不缓存软件标志——深睡门槛必须反映
 * 物理世界（IdleTask 上下文经 wrapper 查询，绑定早于调度器启动）
 *----------------------------------------------------------*/
static u8 App_LpStandbyMode(void *self)
{
    (void)self;
    return (g_systemState.currentMode == MODE_STANDBY) ? 1u : 0u;
}

static u8 App_LpMotorPhysicallyOff(void *self)
{
    (void)self;
    /* motor_stop() 的物理后效验证：TIM1 CH1/CH1N 输出使能位清零
     * 且 MOTOR_EN 引脚拉低（H桥禁止）——软件 motorRunning 标志
     * 不能证明功率级已断开 */
    if ((TIM1->CCER & (TIM_CCER_CC1E | TIM_CCER_CC1NE)) != (uint16_t)0u)
    {
        return 0u;
    }
    if ((MOTOR_EN_GPIO_PORT->ODR & MOTOR_EN_GPIO_PIN) != (uint16_t)0u)
    {
        return 0u;
    }
    return 1u;
}

static u8 App_LpBuzzerIdle(void *self)
{
    u8 queue_empty;
    u8 output_active;

    (void)self;
    /* StandbyBuzzer_StopPermitted 纯函数：命令队列空
     * 且输出无效（有源蜂鸣器高电平响，读 bep 输出数据寄存器）*/
    queue_empty = (uxQueueMessagesWaiting(g_buzzerCmdQueue) == 0u) ? 1u : 0u;
    output_active = ((bep.port->ODR & bep.pin) != (uint16_t)0u) ? 1u : 0u;
    return StandbyBuzzer_StopPermitted(queue_empty, output_active);
}

static u8 App_LpIapIdle(void *self)
{
    (void)self;
    /* APP 侧无 IAP 会话：升级请求即安全停机->写 BKP_DR2->复位进
     * Boot（App_PollIapCommand），不会挂起在 APP 等待——恒空闲 */
    return 1u;
}

static u8 App_LpUartIdle(void *self)
{
    (void)self;
    /* USART1 TC=1：printf 重定向通道发送完成（fputc 亦等待 TC）*/
    return ((USART1->SR & USART_SR_TC) != (uint16_t)0u) ? 1u : 0u;
}

static u8 App_LpKeyExtiAvailable(void *self)
{
    (void)self;
    /* KEY EXTI 线已配置（KeyExti_TargetInit 使能 EXTI1 IMR 位）：
     * 深睡唤醒通道必须在场，否则 STOP 后无唤醒源 */
    return ((EXTI->IMR & KEY_EXTI_LINE_MASK) != 0u) ? 1u : 0u;
}

/* 低功耗快照源绑定表（self=NULL：适配器只读全局状态，无需上下文）*/
static const LowPower_Sources_t s_lpSources = {
    NULL,
    App_LpStandbyMode,
    App_LpMotorPhysicallyOff,
    App_LpBuzzerIdle,
    App_LpIapIdle,
    App_LpUartIdle,
    App_LpKeyExtiAvailable
};

/*-----------------------------------------------------------
 * 系统初始化
 *----------------------------------------------------------*/
void System_Init(void)
{
    /* 初始化系统状态 */
    g_systemState.currentMode = MODE_STANDBY;
    g_systemState.speedLevel = SPEED_LOW;
    g_systemState.motorRunning = 0;
    g_systemState.temperature = 0;
    g_systemState.humidity = 0;
    g_systemState.gasConcentration = 0.0f;
    g_systemState.windSpeedPWM = 0.0f;
    g_systemState.actualRPM = 0.0f;
    g_systemState.targetRPM = 0;
    g_systemState.cookingEventActive = 0;
    g_systemState.antiBackflowActive = 0;
    g_systemState.gasThreshold = GAS_THRESHOLD_NORMAL;
    g_systemState.autoModeCounter = 0;
    g_systemState.cookingEventCounter = 0;
    g_systemState.fault = FAULT_NONE;

    /* 创建互斥信号量 */
    g_dataMutex = xSemaphoreCreateMutex();

    /* 创建速度计算二值信号量 */
    g_speedCalcSemaphore = xSemaphoreCreateBinary();

    /* 创建电机命令队列（长度4，电机硬件单一所有者通信通道）*/
    g_motorCmdQueue = xQueueCreate(MOTOR_CMD_QUEUE_LEN, sizeof(MotorCommand_t));

    /* 创建状态事件队列（长度8，StateManager单一写者通信通道）*/
    g_stateEventQueue = xQueueCreate(STATE_EVENT_QUEUE_LEN, sizeof(StateEvent_t));

    /* 蜂鸣器命令队列（必须在BuzzerTask运行前创建）*/
    g_buzzerCmdQueue = xQueueCreate(BUZZER_CMD_QUEUE_LEN, sizeof(BuzzerCmd_t));

    /* 待机广播事件组（必须在各任务运行前创建，
     * StateManagerTask 边沿置位 / 周期任务与 HealthTask 待机阻塞）*/
    g_standbyEventGroup = xEventGroupCreate();

#if defined(TASK_HEALTH_ENABLED) && TASK_HEALTH_ENABLED
    /* HealthTask 注入待机事件组（模式边沿驱动 mask 切换）
     * 与 IWDG 服务查询（软件启动状态 + option byte 硬件狗位，
     * 决定待机态是否允许无限期阻塞）*/
    TaskHealth_BindStandbyEvents(g_standbyEventGroup);
    TaskHealth_BindIwdgQuery(Watchdog_ServiceRequiredNow);
#endif

    /* 低功耗快照源绑定（深睡门槛只读真实硬件状态；
     * 未绑定绝不深睡——wrapper 保持官方浅睡 fallback）*/
    LowPower_BindSources(&s_lpSources);

}

/*-----------------------------------------------------------
 * 创建开始任务
 *----------------------------------------------------------*/
void StartTask_Create(void)
{
    xTaskCreate(StartTask, "StartTask", TASK_START_STK_SIZE, NULL, 
                TASK_START_PRIORITY, &xStartTaskHandle);
}

/*-----------------------------------------------------------
 * 开始任务 - 创建其他任务
 *----------------------------------------------------------*/
void StartTask(void *pvParameters)
{
    taskENTER_CRITICAL();   /* 进入临界区 */
    
    /* 创建按键扫描任务 */
    xTaskCreate(KeyScanTask, "KeyScan", TASK_KEY_STK_SIZE, NULL,
                TASK_KEY_PRIORITY, &xKeyScanTaskHandle);
    
    /* 创建传感器采集任务 */
    xTaskCreate(SensorTask, "Sensor", TASK_SENSOR_STK_SIZE, NULL,
                TASK_SENSOR_PRIORITY, &xSensorTaskHandle);
    
    /* 创建风速计算任务 */
    xTaskCreate(WindSpeedTask, "WindSpeed", TASK_WIND_SPEED_STK_SIZE, NULL,
                TASK_WIND_SPEED_PRIORITY, &xWindSpeedTaskHandle);
    
    /* 创建状态管理任务（控制平面单一写者）*/
    xTaskCreate(StateManagerTask, "StateMgr", TASK_STATE_MANAGER_STK_SIZE, NULL,
                TASK_STATE_MANAGER_PRIORITY, &xStateManagerTaskHandle);

    /* 创建电机控制任务 */
    xTaskCreate(MotorControlTask, "Motor", TASK_MOTOR_STK_SIZE, NULL,
                TASK_MOTOR_PRIORITY, &xMotorControlTaskHandle);
    
    /* 创建UI显示任务 */
    xTaskCreate(UIDisplayTask, "UI", TASK_UI_STK_SIZE, NULL,
                TASK_UI_PRIORITY, &xUIDisplayTaskHandle);

    /* 创建速度计算任务 */
    xTaskCreate(SpeedCalcTask, "SpeedCalc", TASK_SPEED_CALC_STK_SIZE, NULL,
                TASK_SPEED_CALC_PRIORITY, &xSpeedCalcTaskHandle);

    /* 创建诊断统计任务（任务表/栈高水位周期输出；
     * 句柄记录为待机 EXIT 广播的通知目标之一）*/
    xTaskCreate(DiagnosticTask, "Diag", TASK_DIAGNOSTIC_STK_SIZE, NULL,
                TASK_DIAGNOSTIC_PRIORITY, &xDiagnosticTaskHandle);

    /* 创建健康监督任务（2s窗口心跳判定 + 唯一喂狗点）*/
    xTaskCreate(HealthTask, "Health", TASK_HEALTH_STK_SIZE, NULL,
                TASK_HEALTH_PRIORITY, NULL);

    /* 创建蜂鸣器任务（最低优先级，异步执行鸣叫时序）*/
    xTaskCreate(BuzzerTask, "Buzzer", TASK_BUZZER_STK_SIZE, NULL,
                TASK_BUZZER_PRIORITY, NULL);

    /* 初始化TIM4定时器（必须在信号量创建后调用，避免HardFault）
     * 参数：5-1=4, 14400-1=14399
     * 定时器频率 = 72MHz / 14400 = 5kHz
     * 中断周期 = 5 / 5kHz = 1ms */
    TIM4_init(5-1, 14400-1);
    
    taskEXIT_CRITICAL();    /* 退出临界区 */
    
    /* 装配 KEY EXTI 唤醒通道（必须在 KeyScanTask 创建后：
     * ISR 通知目标句柄已存在；EXTI1/NVIC 真实配置见 stm32f10x_it.c）*/
    KeyExti_TargetInit();

    /* 低功耗目标侧装配（ExtiStop 后端 IO 适配 + LSE 板型
     * RTC 时间基；必须在 KeyExti_TargetInit 之后——EXTI 唤醒线已在场，
     * 深睡序列才安全）*/
    LowPower_TargetInit();

    /* 删除开始任务 */
    vTaskDelete(xStartTaskHandle);
}

/*-----------------------------------------------------------
 * IAP 升级命令轮询
 * 串口命令行由 usart.c 的 RXNE 中断组装（"IAP\r\n" 结尾置位
 * USART_RX_STA bit15），KeyScanTask 以 10ms 周期在此消费：
 * 命中 "IAP" 即安全停机->写 BKP_DR2 请求->复位进 Boot。
 * 触发方：PC 端 tools/iap_uart_sender.py --kick（发 "IAP\r\n" 后
 * 等 2s 再开始 START 握手）；非法行直接丢弃重新组行。
 *----------------------------------------------------------*/
static void App_PollIapCommand(void)
{
    if ((USART_RX_STA & 0x8000u) != 0u)           /* 收到以\r\n结尾的完整行 */
    {
        u16 len = USART_RX_STA & 0x3FFFu;
        if ((len == 3u)
            && (USART_RX_BUF[0] == 'I')
            && (USART_RX_BUF[1] == 'A')
            && (USART_RX_BUF[2] == 'P'))
        {
            printf("IAP: update requested, rebooting to bootloader...\r\n");
            IAP_Request_Update();                /* 不返回：复位进 Boot */
        }
        USART_RX_STA = 0u;                        /* 丢弃本行，重新组行 */
    }
}

/*-----------------------------------------------------------
 * 按键扫描任务 - 周期10ms
 *----------------------------------------------------------*/
void KeyScanTask(void *pvParameters)
{
    KeyEvent_t key1Event, key2Event;
    /* 长按鸣叫边沿去重状态——LONG_PRESSING每10ms重复触发，
     * ON/OFF命令只在状态翻转的边沿发送一次，避免队列被重复命令灌满 */
    u8 key1Buzzing = 0;
    u8 key2Buzzing = 0;

    while (1)
    {
        DWT_EXEC_BEGIN(DWT_SLOT_KEY_SCAN);   /* DWT begin */
        /* 扫描按键 */
        Key_Scan();

        /* 获取按键1事件（模式切换） */
        key1Event = Key1_GetEvent();
        if (key1Event == KEY_EVENT_SHORT_PRESS)
        {
            System_SwitchMode();
            Buzzer_SendCommand(BUZZER_CMD_BEEP, 100);   /* 短按提示音（异步，不阻塞本任务）*/
            Key1_ClearEvent();
        }
        else if (key1Event == KEY_EVENT_LONG_PRESSING)
        {
            if (!key1Buzzing)    /* 边沿触发：只在进入长按时发一次ON */
            {
                key1Buzzing = 1;
                Buzzer_SendCommand(BUZZER_CMD_ON, 0);   /* 长按持续鸣叫 */
            }
        }
        else if (key1Event == KEY_EVENT_RELEASE)
        {
            if (key1Buzzing)     /* 边沿触发：只在鸣叫中发OFF */
            {
                key1Buzzing = 0;
                Buzzer_SendCommand(BUZZER_CMD_OFF, 0);  /* 释放时停止鸣叫 */
            }
            Key1_ClearEvent();
        }

        /* 获取按键2事件（档位切换/风机开关） */
        key2Event = Key2_GetEvent();
        if (key2Event == KEY_EVENT_SHORT_PRESS)
        {
            System_SwitchSpeedLevel();
            Buzzer_SendCommand(BUZZER_CMD_BEEP, 100);   /* 短按提示音（异步）*/
            Key2_ClearEvent();
        }
        else if (key2Event == KEY_EVENT_LONG_PRESS)
        {
            System_ToggleMotor();         /*长按时关闭或者打开电机，无论电机在何种模式下*/

        }
        else if (key2Event == KEY_EVENT_LONG_PRESSING)
        {
            if (!key2Buzzing)    /* 边沿触发 */
            {
                key2Buzzing = 1;
                Buzzer_SendCommand(BUZZER_CMD_ON, 0);   /* 长按持续鸣叫 */
            }
        }
        else if (key2Event == KEY_EVENT_RELEASE)
        {
            if (key2Buzzing)     /* 边沿触发 */
            {
                key2Buzzing = 0;
                Buzzer_SendCommand(BUZZER_CMD_OFF, 0);  /* 释放时停止鸣叫 */
            }
            Key2_ClearEvent();
        }
        DWT_EXEC_END(DWT_SLOT_KEY_SCAN);     /* DWT end */

        App_PollIapCommand();   /* 串口 "IAP" 升级命令轮询 */

        /* 扫描阻塞策略（KeyExti_ScanWaitTicks 纯函数）：
         *   活动态/按键处理中 -> 10ms 有限超时推进消抖与长按计时；
         *   待机且无按键处理中 -> portMAX_DELAY，只由按键 EXTI 通知
         *（或 IAP 串口行完成通知，usart.c）唤醒——消除待机周期
         *   唤醒源，为 eNoTasksWaitingTimeout 的 STOP 深睡铺路（Task 15）*/
        ulTaskNotifyTake(pdTRUE, KeyExti_ScanWaitTicks(
            (g_systemState.currentMode == MODE_STANDBY) ? 1u : 0u,
            Key1_InProgress()));

    }
}

/*-----------------------------------------------------------
 * 传感器采集任务 - 周期1000ms（DHT11规格要求≥1s）
 *----------------------------------------------------------*/
void SensorTask(void *pvParameters)
{
    u8 temp,humi;
    float gasValue;
    u8 ret;
    MQ2_Status_t mq2Status;
    u8 failCount = 0;           /* 传感器组连续失败计数（DHT11+MQ2任一失败）*/
    u8 okCount = 0;             /* 超时后连续成功计数（恢复判据）*/
    u8 timeoutActive = 0;       /* 本地超时上报状态（边沿触发，避免重复上报）*/

    while (1)
    {
        DWT_EXEC_BEGIN(DWT_SLOT_SENSOR);   /* DWT begin */
        /* 读取DHT11数据（引脚统一取自 CONFIG/board_pins.h：PC13，
         * 板型 BOARD_C8_LSE 冻结后 PC14/PC15 归 LSE，DHT11 自 PC14 迁入） */
        ret = DHT_Read_Data(&temp,&humi,DHT11_GPIO_PORT, DHT11_GPIO_PIN, &dht);
        if (ret == 1)/*如果校验成功*/
        {
            /*如果此处获取2个传感器数据使用同一把锁，会导致占用资源时间过长，
            更容易带来优先级反转问题；因此此处采用两把锁*/
            if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE)
            {
                g_systemState.temperature = temp;
                g_systemState.humidity = humi;
                xSemaphoreGive(g_dataMutex);
            }
        }

        /* 读取MQ2数据（带状态码，失败保留上一有效值——
         * 不写0污染控制平面，自动模式决策继续用旧浓度，偏差有限）*/
        mq2Status = MQ2_ReadGasConcentration(&gasValue);
        if (mq2Status == MQ2_OK)
        {
            if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE)
            {
                g_systemState.gasConcentration = gasValue;
                xSemaphoreGive(g_dataMutex);
            }
        }

        /* 传感器组健康判定（抽纯函数 Sensor_HealthCheck，
         * 判定与执行分层——纯逻辑只返回事件位图，硬件副作用留在任务上下文）*/
        {
            SensorHealthCtx_t sh;
            SensorHealthEvt_t evts;
            sh.dhtOk         = (ret == 1) ? 1 : 0;
            sh.mq2Ok         = (mq2Status == MQ2_OK) ? 1 : 0;
            sh.currentFault  = g_systemState.fault;
            sh.failCount     = failCount;
            sh.okCount       = okCount;
            sh.timeoutActive = timeoutActive;
            evts = Sensor_HealthCheck(&sh);
            failCount     = sh.failCount;
            okCount       = sh.okCount;
            timeoutActive = sh.timeoutActive;
            /* 按位顺序发送：先 TIMEOUT/RECOVERED，再 HOLD 重发（与原内联
             * 块逐行等价——首确认与保持可在同一周期同时触发两次 TIMEOUT）*/
            if (evts & SENSOR_EVT_TIMEOUT)
                State_SendEvent(STATE_EVENT_SENSOR_TIMEOUT);
            if (evts & SENSOR_EVT_RECOVERED)
                State_SendEvent(STATE_EVENT_SENSOR_RECOVERED);
            if (evts & SENSOR_EVT_TIMEOUT_HOLD)
                State_SendEvent(STATE_EVENT_SENSOR_TIMEOUT);
        }
        DWT_EXEC_END(DWT_SLOT_SENSOR);     /* DWT end */
#if SENSOR_DEBUG    /*传感器数据调试，右键点击该宏--goto definition--将0改成1即可使用*/
        printf("Temp: %d, Humi: %d, Gas: %.2f\r\n", temp, humi, gasValue);
#endif
        /* 任务心跳——无条件上报（传感器读取失败照常心跳，
         * 心跳只证明任务在跑，业务降级由StateManager故障状态机负责）*/
        TaskHealth_Heartbeat(TASK_HEALTH_BIT_SENSOR);
        /* 待机阻塞在事件组 SENSOR 独立唤醒位
         *（xClearOnExit 消费），StateManager 退出待机边沿广播唤醒；
         * 消除待机态 1000ms 采集周期唤醒。待机期间无心跳（HealthTask
         * 待机判据为空集，一致）*/
        if (StandbyPeriodic_ShouldBlock(
                (g_systemState.currentMode == MODE_STANDBY) ? 1u : 0u))
        {
            (void)xEventGroupWaitBits(g_standbyEventGroup,
                                      STANDBY_EVT_SENSOR_BIT,
                                      pdTRUE, pdFALSE, portMAX_DELAY);
        }
        else
        {
            delay_ms(1000);  /* 1000ms采集周期：DHT11数据手册要求读取间隔≥1s，
                                MQ2气体浓度变化缓慢，1s采样完全够用 */
        }
    }
}

/*-----------------------------------------------------------
 * 风速计算任务 - 周期100ms
 *----------------------------------------------------------*/
void WindSpeedTask(void *pvParameters)
{
    u8 temp, humidity;
    float gas;
    
    /*如果此处获取传感器和更新系统状态使用同一把锁，会导致占用资源时间过长，更容易带来优先级反转问题*/
    while (1)
    {
        DWT_EXEC_BEGIN(DWT_SLOT_WIND_SPEED);   /* DWT begin */
        /* 获取传感器数据 */
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE)
        {
            temp = g_systemState.temperature;
            humidity = g_systemState.humidity;
            gas = (float)g_systemState.gasConcentration;
            xSemaphoreGive(g_dataMutex);
        }
        
        /* 更新风速计算 */
        WindSpeed_Update(temp, humidity, gas);
        
        /* 更新系统状态 */
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE)
        {
            g_systemState.windSpeedPWM = WindSpeed_GetPWM();
            g_systemState.cookingEventActive = WindSpeed_IsCookingEvent();
            xSemaphoreGive(g_dataMutex);
        }
        DWT_EXEC_END(DWT_SLOT_WIND_SPEED);     /* DWT end */
        
        /* 待机阻塞在事件组 WIND 独立唤醒位，
         * 退出待机广播唤醒；消除待机态 100ms 计算周期唤醒 */
        if (StandbyPeriodic_ShouldBlock(
                (g_systemState.currentMode == MODE_STANDBY) ? 1u : 0u))
        {
            (void)xEventGroupWaitBits(g_standbyEventGroup,
                                      STANDBY_EVT_WIND_BIT,
                                      pdTRUE, pdFALSE, portMAX_DELAY);
        }
        else
        {
            delay_ms(100);  /* 100ms计算周期 */
        }
    }
}

/*-----------------------------------------------------------
 * 电机命令发送接口（电机硬件单一所有者）
 * 其他任务通过本函数请求电机动作，禁止直接调用motor_*
 * 策略说明：
 *  - 普通命令(RUN_RPM)：入队失败即计数丢弃（周期性命令下一周期会重发）；
 *  - 安全停机(STOP)：插队到队首优先处理；队列满时挤掉队尾最旧命令，
 *    保证STOP绝不因队列满而丢失，且不静默丢弃（记录计数）
 *----------------------------------------------------------*/
BaseType_t Motor_SendCommand(MotorCommandType_t type, int16_t value)
{
    MotorCommand_t cmd;
    BaseType_t ok = pdFALSE;

    cmd.type = type;
    cmd.value = value;
    cmd.timestamp = xTaskGetTickCount();

    /* 队列满策略经纯函数 Queue_FullPolicy/ResolveFullAction 统一描述，
     * STOP=安全关键(EVICT_RETRY)，其余=DROP_COUNT；行为与原内联块逐行等价 */
    {
        u8 isCritical = (type == MOTOR_CMD_STOP) ? 1 : 0;
        QueueFullPolicy_t pol = Queue_FullPolicy(isCritical);

        if (isCritical)
        {
            /* 安全停机：插到队首保证优先执行 */
            ok = xQueueSendToFront(g_motorCmdQueue, &cmd, 0);
            if (ok != pdTRUE)
            {
                /* 队列满：丢弃最旧的一条命令腾出空间，STOP必须送达 */
                MotorCommand_t oldest;
                u8 evictOk = (xQueueReceive(g_motorCmdQueue, &oldest, 0) == pdTRUE) ? 1 : 0;
                u8 inc = 0;
                QueueFullAction_t act = Queue_ResolveFullAction(pol, 0, evictOk, &inc);
                g_motorQueueFullCount += inc;   /* 挤占成功才计数（诊断证据）*/
                if (act == QF_ACT_EVICT_RETRY)
                    ok = xQueueSendToFront(g_motorCmdQueue, &cmd, 0);
            }
        }
        else
        {
            ok = xQueueSend(g_motorCmdQueue, &cmd, 0);
            if (ok != pdTRUE)
            {
                u8 inc = 0;
                Queue_ResolveFullAction(pol, 0, 0, &inc);
                g_motorQueueFullCount += inc;   /* 队列满计数（诊断证据）*/
            }
        }
    }

    return ok;
}

/*-----------------------------------------------------------
 * 蜂鸣器命令发送接口（异步蜂鸣器）
 * 发送方（KeyScanTask 10ms / StateManagerTask 50ms周期）不再
 * 直接调用阻塞式Buzzer_Beep（100ms占空比恶化到1000%/200%），
 * 改为向长度2的命令队列投递，由最低优先级BuzzerTask执行鸣叫时序。
 * 队列满策略：丢弃+计数（蜂鸣为UI反馈非安全关键，与电机STOP
 * 的挤占保证不同）；ON/OFF为边沿触发命令，正常流量队列不满
 *----------------------------------------------------------*/
BaseType_t Buzzer_SendCommand(BuzzerCmdType_t type, u16 duration_ms)
{
    BuzzerCmd_t cmd;
    BaseType_t ok;

    cmd.type = type;
    cmd.duration_ms = duration_ms;

    /* 蜂鸣器为UI反馈类，非安全关键，统一走 DROP_COUNT 策略 */
    {
        u8 isCritical = 0;
        QueueFullPolicy_t pol = Queue_FullPolicy(isCritical);
        ok = xQueueSend(g_buzzerCmdQueue, &cmd, 0);
        if (ok != pdTRUE)
        {
            u8 inc = 0;
            Queue_ResolveFullAction(pol, 0, 0, &inc);
            g_buzzerQueueFullCount += inc;   /* 队列满计数（诊断证据）*/
        }
    }
    return ok;
}

/*-----------------------------------------------------------
 * 蜂鸣器任务（最低优先级，异步执行鸣叫时序）
 * BEEP: on→vTaskDelay(duration)→off（阻塞发生在本任务，与发送方无关）
 * ON/OFF: 直接置GPIO（长按鸣叫的开始/结束）
 * 注：本任务不加入TASK_HEALTH必需心跳位——蜂鸣器是UI反馈类，
 * 卡死不应触发IWDG整机复位（与控制/传感任务的安全等级不同）
 *----------------------------------------------------------*/
void BuzzerTask(void *pvParameters)
{
    BuzzerCmd_t cmd;

    (void)pvParameters;

    while (1)
    {
        if (xQueueReceive(g_buzzerCmdQueue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            switch (cmd.type)
            {
                case BUZZER_CMD_BEEP:
                    Beep_on(&bep);
                    vTaskDelay(pdMS_TO_TICKS(cmd.duration_ms));
                    Beep_off(&bep);
                    break;

                case BUZZER_CMD_ON:
                    Beep_on(&bep);
                    break;

                case BUZZER_CMD_OFF:
                default:
                    Beep_off(&bep);
                    break;
            }
        }
    }
}

/*-----------------------------------------------------------
 * 状态事件发送接口（StateManager单一写者）
 * 模式/档位/电机开关请求一律入队，由StateManagerTask统一决策，
 * 键盘任务等不再直接修改控制平面状态
 *----------------------------------------------------------*/
static void State_SendEvent(StateEventType_t type)
{
    StateEvent_t evt;
    /* FAULT_STALL=安全关键(EVICT_RETRY)，其余=DROP_COUNT；
     * 策略与 Motor_SendCommand 的 STOP 命令一致 */
    u8 isCritical = (type == STATE_EVENT_FAULT_STALL) ? 1 : 0;
    QueueFullPolicy_t pol = Queue_FullPolicy(isCritical);

    evt.type = type;

    if (xQueueSend(g_stateEventQueue, &evt, 0) != pdTRUE)
    {
        if (isCritical)
        {
            /* 堵转上报是安全关键事件，绝不因队列满而静默丢弃：
             * 挤掉最旧的普通事件（按键请求可容忍丢失）后重试入队 */
            StateEvent_t oldest;
            u8 evictOk = (xQueueReceive(g_stateEventQueue, &oldest, 0) == pdTRUE) ? 1 : 0;
            u8 inc = 0;
            QueueFullAction_t act = Queue_ResolveFullAction(pol, 0, evictOk, &inc);
            g_stateEventFullCount += inc;   /* 挤占成功才计数（不静默丢弃）*/
            if (act == QF_ACT_EVICT_RETRY)
                xQueueSend(g_stateEventQueue, &evt, 0);
        }
        else
        {
            u8 inc = 0;
            Queue_ResolveFullAction(pol, 0, 0, &inc);
            g_stateEventFullCount += inc;   /* 队列满计数（诊断证据），按键事件低频可容忍 */
        }
    }
}

/*-----------------------------------------------------------
 * 状态管理任务 - 周期50ms（控制平面单一写者）
 * 职责：
 *  1. 消费按键事件（模式/档位/电机开关请求）；
 *  2. 执行自动模式状态机（启动等待/Cooking/延时关闭）；
 *  3. 执行防回流滞回逻辑（原AntiBackflowTask并入）；
 *  4. 周期性向MotorTask下发电机命令（目标刷新，最大延迟一个周期）；
 *  唯一写者字段：currentMode/speedLevel/targetRPM/autoModeCounter/
 *  cookingEventCounter/antiBackflowActive/gasThreshold
 *----------------------------------------------------------*/
void StateManagerTask(void *pvParameters)
{
    StateEvent_t evt;
    u16 targetRPM;
    u16 pwmCompare;
    u8 isDetected;
    TickType_t xLastWakeTime;                       /* 确定周期调度的基准时刻 */
    const TickType_t xPeriod = pdMS_TO_TICKS(50);   /* 50ms决策周期（tick=1ms）*/
    u8 prevStandby = 1u;   /* 上周期待机标志（System_Init 置
                            * MODE_STANDBY，启动即待机稳态，无边沿广播）*/
    u8 stopSent = 0u;      /* 待机 STOP 已下发标志（退出待机复位）*/

    /* 以当前时刻为基准初始化，之后每次唤醒时刻=上次+50ms，
     * 周期不受本轮处理耗时影响（自动模式计时器精度依赖此特性）*/
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        /* 待机稳态（STOP 已下发且仍在待机）
         * ——阻塞在状态事件队列上（portMAX_DELAY），唯一唤醒源是状态
         * 事件（按键请求/故障上报），消除待机态 50ms 决策周期唤醒。
         * 取到的事件放回队首，由下方统一消费路径处理（避免复制事件
         * switch）；阻塞位于 DWT 区间外，等待时长不污染执行/周期统计；
         * 唤醒后重置 vTaskDelayUntil 基准，避免陈旧基准导致恢复后
         * 首周期连跑（决策周期抖动）*/
        if (StandbyMgr_ShouldBlockOnEvent(
                (g_systemState.currentMode == MODE_STANDBY) ? 1u : 0u, stopSent))
        {
            StateEvent_t wake;
            if (xQueueReceive(g_stateEventQueue, &wake, portMAX_DELAY) == pdTRUE)
            {
                (void)xQueueSendToFront(g_stateEventQueue, &wake, 0);
            }
            xLastWakeTime = xTaskGetTickCount();
        }

        DWT_EXEC_BEGIN(DWT_SLOT_STATE_MGR);   /* DWT begin */
        /* 1. 消费本周期积压的全部状态事件 */
        while (xQueueReceive(g_stateEventQueue, &evt, 0) == pdTRUE)
        {
            switch (evt.type)
            {
                case STATE_EVENT_KEY_MODE:
                    /* 模式循环切换：待机 -> 手动 -> 自动 -> 防回流 -> 待机 */
                    switch (g_systemState.currentMode)
                    {
                        case MODE_STANDBY:
                            g_systemState.currentMode = MODE_MANUAL;
                            break;
                        case MODE_MANUAL:
                            g_systemState.currentMode = MODE_AUTO;
                            g_autoModeState = AUTO_STATE_STARTUP;
                            g_systemState.autoModeCounter = 0;
                            g_systemState.cookingEventCounter = 0;
                            break;
                        case MODE_AUTO:
                            g_systemState.currentMode = MODE_ANTI_BACKFLOW;
                            /* 进入防回流模式：复位防回流状态，等待气体触发 */
                            g_systemState.antiBackflowActive = 0;
                            g_systemState.gasThreshold = GAS_THRESHOLD_NORMAL;
                            break;
                        case MODE_ANTI_BACKFLOW:
                            g_systemState.currentMode = MODE_STANDBY;
                            break;
                    }

                    /* 切换模式即请求电机停机（除自动模式外），
                     * 新模式的运行命令由下方周期性决策下发 */
                    if (g_systemState.currentMode != MODE_AUTO)
                    {
                        Motor_SendCommand(MOTOR_CMD_STOP, 0);
                    }
                    break;

                case STATE_EVENT_KEY_SPEED:
                    /* 档位循环切换：LOW -> HIGH -> LOW */
                    switch (g_systemState.speedLevel)
                    {
                        case SPEED_LOW:
                            g_systemState.speedLevel = SPEED_HIGH;
                            break;
                        case SPEED_HIGH:
                            g_systemState.speedLevel = SPEED_LOW;
                            break;
                    }
                    break;

                case STATE_EVENT_KEY_MOTOR:
                    if (g_systemState.fault == FAULT_MOTOR_STALL)
                    {
                        /* 堵转故障锁存：用户长按确认清除（禁止自动重启），
                         * 解除电机命令锁存并回到待机，由用户重新起动 */
                        g_systemState.fault = FAULT_NONE;
                        Motor_SendCommand(MOTOR_CMD_CLEAR_FAULT, 0);
                        g_systemState.currentMode = MODE_STANDBY;
                    }
                    else if (g_systemState.motorRunning)
                    {
                        /* 关闭：强制待机，防止周期性决策重新拉起电机 */
                        g_systemState.currentMode = MODE_STANDBY;
                    }
                    else
                    {
                        /* 开启：强制手动模式，周期性决策持续下发RUN_RPM */
                        g_systemState.currentMode = MODE_MANUAL;
                    }
                    break;

                case STATE_EVENT_FAULT_STALL:
                    /* MotorTask已本地安全停机（硬件所有者即时动作），
                     * 此处置系统故障态并强制待机，停止一切RUN命令下发 */
                    g_systemState.fault = FAULT_MOTOR_STALL;
                    g_systemState.currentMode = MODE_STANDBY;
                    g_autoModeState = AUTO_STATE_STARTUP;
                    g_systemState.autoModeCounter = 0;
                    g_systemState.cookingEventCounter = 0;
                    /* 蜂鸣器报警（改发命令，本50ms周期任务
                     * 不再被100ms阻塞式鸣叫拖慢两个周期）*/
                    Buzzer_SendCommand(BUZZER_CMD_BEEP, 100);
                    break;

                case STATE_EVENT_SENSOR_TIMEOUT:
                    /* 传感器数据失效：置故障态；自动模式决策处降级为固定安全风量。
                     * 故障码字段单一，堵转优先级更高：堵转锁存期间不覆盖
                     *（此期间模式已强制待机，降级无意义；传感器仍超时的
                     * 情况由SensorTask在故障码空出后重发事件恢复降级）*/
                    if (g_systemState.fault == FAULT_NONE)
                    {
                        g_systemState.fault = FAULT_SENSOR_TIMEOUT;
                    }
                    break;

                case STATE_EVENT_SENSOR_RECOVERED:
                    /* 传感器恢复：仅清除传感器故障，不触碰堵转锁存 */
                    if (g_systemState.fault == FAULT_SENSOR_TIMEOUT)
                    {
                        g_systemState.fault = FAULT_NONE;
                    }
                    break;

                default:
                    break;
            }
        }

        /* 模式边沿检测 + 待机广播。
         * 只在活动<->待机转换的边沿置位（稳态不重复广播，防粘滞位
         * 让等待方误判二次进待机）：
         *   ENTER -> STANDBY 位（HealthTask 切待机监督语义）；
         *   EXIT  -> ACTIVE 位（HealthTask 切活动监督）+ 四个周期任务
         *            独立唤醒位（各自 xClearOnExit 消费清零，一次
         *            广播唤醒全部，位定义单一来源 standby_policy.h）*/
        {
            u8 nowStandby = (g_systemState.currentMode == MODE_STANDBY) ? 1u : 0u;
            StandbyEdge_t edge = StandbyMgr_DetectEdge(prevStandby, nowStandby);

            if (edge == STANDBY_EDGE_ENTER)
            {
                (void)xEventGroupSetBits(g_standbyEventGroup,
                                         STANDBY_EVT_STANDBY_BIT);
            }
            else if (edge == STANDBY_EDGE_EXIT)
            {
                (void)xEventGroupSetBits(g_standbyEventGroup,
                        STANDBY_EVT_ACTIVE_BIT | STANDBY_EVT_SENSOR_BIT |
                        STANDBY_EVT_WIND_BIT | STANDBY_EVT_UI_BIT |
                        STANDBY_EVT_DIAG_BIT);
                stopSent = 0u;      /* 退出待机：STOP 已发标志复位 */
            }
            prevStandby = nowStandby;
        }

        /* 2. 按当前模式决策并下发电机命令（每周期刷新目标/开环PWM）*/
        switch (g_systemState.currentMode)
        {
            case MODE_STANDBY:
                /* 待机模式：电机 STOP 只在进入待机后的
                 * 首个决策周期下发一次（stopSent 标志，决策纯函数
                 * StandbyMgr_ShouldSendStop），之后本任务阻塞在状态
                 * 事件队列（循环顶部），不再每 50ms 重发 STOP——消除
                 * 待机态周期唤醒源。STOP 命令幂等（motor_stop+比较值
                 * 清零），KEY 路径已发过一次时重复无害 */
                if (StandbyMgr_ShouldSendStop(1u, stopSent))
                {
                    Motor_SendCommand(MOTOR_CMD_STOP, 0);
                    stopSent = 1u;
                }
                g_systemState.autoModeCounter = 0;
                g_systemState.cookingEventCounter = 0;
                g_autoModeState = AUTO_STATE_STARTUP;
                break;

            case MODE_MANUAL:
                /* 手动模式：按档位闭环PID调速 */
                targetRPM = WindSpeed_GetTargetRPM(g_systemState.speedLevel);
                g_systemState.targetRPM = targetRPM;
                Motor_SendCommand(MOTOR_CMD_RUN_RPM, (int16_t)targetRPM);
                break;

            case MODE_AUTO:
                /* 自动模式状态机（开环：风速融合PWM直接驱动）*/
                switch (g_autoModeState)
                {
                    case AUTO_STATE_STARTUP:
                        /* 启动阶段：以最小转速运行，等待Cooking Event */
                        Motor_SendCommand(MOTOR_CMD_RUN_PWM, (int16_t)(PWM_MIN*10));

                        g_systemState.autoModeCounter += 50;

                        if (g_systemState.cookingEventActive)
                        {
                            /* 检测到Cooking Event */
                            g_autoModeState = AUTO_STATE_COOKING;
                            g_systemState.cookingEventCounter = 0;
                        }
                        else if (g_systemState.autoModeCounter >= AUTO_MODE_STARTUP_TIME)
                        {
                            /* 60秒内无Cooking Event，关闭自动模式 */
                            g_systemState.currentMode = MODE_STANDBY;
                        }
                        break;

                    case AUTO_STATE_COOKING:
                        /* Cooking Event激活：根据传感器自动调节。
                         * 传感器超时降级：融合算法输入已失效，弃用其输出，
                         * 固定输出安全风量（30%）维持排烟基本功能 */
                        if (g_systemState.fault == FAULT_SENSOR_TIMEOUT)
                        {
                            pwmCompare = AUTO_SAFE_PWM_COMPARE;
                        }
                        else
                        {
                            pwmCompare = WindSpeed_GetPWMCompare(MAXCCR);
                        }
                        Motor_SendCommand(MOTOR_CMD_RUN_PWM, (int16_t)pwmCompare);

                        g_systemState.cookingEventCounter += 50;

                        if (!g_systemState.cookingEventActive)
                        {
                            /* Cooking Event结束 */
                            g_autoModeState = AUTO_STATE_DELAY_OFF;
                            g_systemState.cookingEventCounter = 0;
                        }
                        else if (g_systemState.cookingEventCounter >= COOKING_EVENT_TIMEOUT)
                        {
                            /* Cooking Event持续超过60秒，关闭自动模式 */
                            g_systemState.currentMode = MODE_STANDBY;
                        }
                        break;

                    case AUTO_STATE_DELAY_OFF:
                        /* 延时关闭阶段（传感器超时同样降级为固定安全风量）*/
                        if (g_systemState.fault == FAULT_SENSOR_TIMEOUT)
                        {
                            pwmCompare = AUTO_SAFE_PWM_COMPARE;
                        }
                        else
                        {
                            pwmCompare = WindSpeed_GetPWMCompare(MAXCCR);
                        }
                        Motor_SendCommand(MOTOR_CMD_RUN_PWM, (int16_t)pwmCompare);

                        g_systemState.cookingEventCounter += 50;

                        if (g_systemState.cookingEventActive)
                        {
                            /* 又检测到Cooking Event */
                            g_autoModeState = AUTO_STATE_COOKING;
                            g_systemState.cookingEventCounter = 0;
                        }
                        else if (g_systemState.cookingEventCounter >= COOKING_EVENT_DELAY_OFF)
                        {
                            /* 10秒延时结束，关闭自动模式 */
                            g_systemState.currentMode = MODE_STANDBY;
                        }
                        break;
                }
                break;

            case MODE_ANTI_BACKFLOW:
                /* 防回流滞回逻辑（双阈值防抖动）*/
                isDetected = (g_systemState.gasConcentration >= g_systemState.gasThreshold) ? 1 : 0;

                if (isDetected)
                {
                    if (g_systemState.gasConcentration < GAS_THRESHOLD_HIGH)/*防止在极端情况下会重启风机*/
                    {
                        /* 气体超过NORMAL阈值：启动风机并立即切换到HIGH阈值防重复触发 */
                        g_systemState.antiBackflowActive = 1;
                        g_systemState.gasThreshold = GAS_THRESHOLD_HIGH;
                    }
                }
                else if (g_systemState.gasConcentration < GAS_THRESHOLD_NORMAL)
                {
                    /* 气体浓度降到NORMAL阈值以下：停止风机并恢复NORMAL阈值 */
                    g_systemState.antiBackflowActive = 0;
                    g_systemState.gasThreshold = GAS_THRESHOLD_NORMAL;
                }

                if (g_systemState.antiBackflowActive)
                {
                    /* 使用用户设定的档位转速闭环运行 */
                    targetRPM = WindSpeed_GetTargetRPM(g_systemState.speedLevel);
                    g_systemState.targetRPM = targetRPM;
                    Motor_SendCommand(MOTOR_CMD_RUN_RPM, (int16_t)targetRPM);
                }
                else
                {
                    Motor_SendCommand(MOTOR_CMD_STOP, 0);
                }
                break;
        }

        DWT_EXEC_END(DWT_SLOT_STATE_MGR);     /* DWT end */
        /* 完成一个决策周期，上报任务心跳 */
        TaskHealth_Heartbeat(TASK_HEALTH_BIT_STATE_MGR);
        /* vTaskDelayUntil：绝对唤醒时刻，确定周期。
         * 相比vTaskDelay（相对延时，实际周期=处理耗时+50ms，存在累积漂移），
         * vTaskDelayUntil保证长期平均周期恒为50ms */
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

/*-----------------------------------------------------------
 * 电机控制任务 - 周期50ms（纯执行器）
 * 电机硬件（PWM/方向/H桥使能）的唯一所有者：
 *  1. 每周期非阻塞清空命令队列，执行最新命令语义；
 *  2. 闭环命令(RUN_RPM)按目标转速做PID；开环命令(RUN_PWM)直接输出比较值；
 *  3. 堵转检测：命令值高于门限且实际转速低于门限持续500ms即安全停机，
 *     本地锁存故障拒绝RUN命令（禁止自动重启），并上报StateManager；
 *  4. 本任务不再读取/决策工作模式，只忠实执行StateManager下发的命令
 *----------------------------------------------------------*/
void MotorControlTask(void *pvParameters)
{
    MotorCommand_t cmd;
    float pidOutput;
    u8 motorRunning = 0;        /* 电机运行状态（MotorTask私有的真实硬件状态）*/
    u8 pidActive = 0;           /* 1=闭环RPM控制，0=开环PWM控制 */
    u16 openLoopPwm = 0;        /* 开环PWM比较值 */
    u16 commandedTarget = 0;    /* 最近下发的闭环目标转速（堵转判据用）*/
    u8 faultLatched = 0;        /* 堵转故障锁存：置位后拒绝RUN命令，仅CLEAR_FAULT清除 */
    u16 stallCounter = 0;       /* 堵转条件连续满足时长(ms) */
    u16 startGrace = 0;         /* 起转宽限期剩余(ms)，宽限期内不做堵转判定 */
    TickType_t xLastWakeTime;                       /* 确定周期调度的基准时刻 */
    const TickType_t xPeriod = pdMS_TO_TICKS(50);   /* 50ms控制周期（tick=1ms）*/

    /* 初始化唤醒基准：PID采样周期恒为50ms，不受单轮处理耗时影响 */
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        /* 已停且命令队列空（决策纯函数
         * StandbyMotor_ShouldBlock）——阻塞在命令队列上
         *（portMAX_DELAY）：StateManager 待机只发一次 STOP，之后
         * 两任务均在各自队列上挂起，消除待机态 50ms 控制周期唤醒；
         * 运行态（或队列有待消费命令）保持 50ms PID/堵转检测周期。
         * 取到的命令放回队首，由下方统一消费路径处理（避免复制
         * 命令 switch）；阻塞位于 DWT 区间外；唤醒后重置
         * vTaskDelayUntil 基准（PID 的固定 50ms 采样周期假设不被
         * 陈旧基准破坏）*/
        if (StandbyMotor_ShouldBlock(motorRunning,
                (uxQueueMessagesWaiting(g_motorCmdQueue) == 0u) ? 1u : 0u))
        {
            MotorCommand_t wake;
            if (xQueueReceive(g_motorCmdQueue, &wake, portMAX_DELAY) == pdTRUE)
            {
                (void)xQueueSendToFront(g_motorCmdQueue, &wake, 0);
            }
            xLastWakeTime = xTaskGetTickCount();
        }

        DWT_EXEC_BEGIN(DWT_SLOT_MOTOR);   /* DWT begin */
        /* 获取实际转速并发布（测量数据平面）*/
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE)
        {
            g_systemState.actualRPM = speed;
            xSemaphoreGive(g_dataMutex);
        }

        /* 1. 非阻塞清空命令队列：StateManager每周期刷新命令，
         *    只需保留每条命令的最新语义（后到覆盖先到）*/
        while (xQueueReceive(g_motorCmdQueue, &cmd, 0) == pdTRUE)
        {
            switch (cmd.type)
            {
                case MOTOR_CMD_STOP:
                    motor_stop();                        /* PWM通道关闭+H桥禁止 */
                    TIM_SetCompare1(TIM1, 0);           /* 比较值清零 */
                    PID_Reset(&g_speedPID);             /* 清积分，避免残留输出突变 */
                    motorRunning = 0;
                    pidActive = 0;
                    openLoopPwm = 0;
                    break;

                case MOTOR_CMD_RUN_RPM:
                    if (faultLatched)
                    {
                        break;  /* 堵转锁存期间拒绝运行命令（禁止自动重启）*/
                    }
                    if (!motorRunning)
                    {
                        PID_Reset(&g_speedPID);         /* 起转前清积分 */
                        motor_start();                  /* 使能H桥 */
                        motorRunning = 1;
                        startGrace = MOTOR_START_GRACE_MS; /* 起转宽限期开始 */
                        stallCounter = 0;
                    }
                    pidActive = 1;
                    commandedTarget = (u16)cmd.value;
                    PID_SetTarget(&g_speedPID, (float)cmd.value);
                    break;

                case MOTOR_CMD_RUN_PWM:
                    if (faultLatched)
                    {
                        break;  /* 堵转锁存期间拒绝运行命令（禁止自动重启）*/
                    }
                    if (!motorRunning)
                    {
                        PID_Reset(&g_speedPID);
                        motor_start();                  /* 使能H桥 */
                        motorRunning = 1;
                        startGrace = MOTOR_START_GRACE_MS; /* 起转宽限期开始 */
                        stallCounter = 0;
                    }
                    pidActive = 0;
                    openLoopPwm = (u16)cmd.value;
                    break;

                case MOTOR_CMD_CLEAR_FAULT:
                    /* 用户确认清除堵转故障：解除命令锁存，
                     * 电机保持停止，等待StateManager下发新的运行命令 */
                    faultLatched = 0;
                    stallCounter = 0;
                    break;

                default:
                    break;
            }
        }

        /* 2. 堵转检测（硬件所有者的本地安全职责，任务6 + 
         * 抽纯函数 Motor_StallCheck，判定与执行分层——纯逻辑只返回
         * NONE/PENDING/CONFIRMED，硬件副作用留在任务上下文执行）
         * 判据：命令值高于门限（闭环目标RPM/开环比较值）且实际转速
         * 低于门限，连续满足500ms即确认堵转；
         * 起转后1000ms为宽限期，避免正常起动过程被误判 */
        {
            StallCheckCtx_t sc;
            StallVerdict_t verdict;
            sc.motorRunning    = motorRunning;
            sc.faultLatched    = faultLatched;
            sc.startGrace      = startGrace;
            sc.pidActive       = pidActive;
            sc.commandedTarget = commandedTarget;
            sc.speed           = speed;
            sc.openLoopPwm     = openLoopPwm;
            sc.stallCounter    = stallCounter;
            verdict = Motor_StallCheck(&sc);
            startGrace   = sc.startGrace;
            stallCounter = sc.stallCounter;
            if (verdict == STALL_CONFIRMED)
            {
                /* 堵转确认：立即本地安全停机（不等StateManager往返，
                 * 硬件所有者对安全负最终责任），锁存故障并上报 */
                motor_stop();
                TIM_SetCompare1(TIM1, 0);
                PID_Reset(&g_speedPID);
                motorRunning = 0;
                pidActive = 0;
                openLoopPwm = 0;
                faultLatched = 1;       /* 拒绝后续RUN命令，禁止自动重启 */
                State_SendEvent(STATE_EVENT_FAULT_STALL);  /* 上报StateManager */
            }
        }

        /* 3. 同步真实运行状态到系统状态（motorRunning的唯一权威写者）*/
        g_systemState.motorRunning = motorRunning;

        /* 4. 执行控制律 */
        if (motorRunning)
        {
            if (pidActive)
            {
                pidOutput = PID_Calculate(&g_speedPID, speed);
                motor_pwm_set(pidOutput);
            }
            else
            {
                motor_pwm_set((float)openLoopPwm);
            }
        }

        /* vTaskDelayUntil：绝对唤醒时刻，PID采样周期恒为50ms。
         * vTaskDelay会因单轮处理耗时不稳定导致采样周期抖动，
         * 影响PID中积分/微分项的dt假设（此处声明固定50ms）*/
        DWT_EXEC_END(DWT_SLOT_MOTOR);     /* DWT end */
        /* 完成一个控制周期，上报任务心跳 */
        TaskHealth_Heartbeat(TASK_HEALTH_BIT_MOTOR);
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

/*-----------------------------------------------------------
 * UI显示任务 - 周期200ms
 * 通过System_GetSnapshot()获取状态快照拷贝，
 * 不直接持有可变全局指针（单一写者架构下的只读消费者）
 *----------------------------------------------------------*/
void UIDisplayTask(void *pvParameters)
{
    char dispBuf[32];
    SystemSnapshot_t snap;

    /* 清屏 */
    LCD_Clear(WHITE);

    while (1)
    {
        DWT_EXEC_BEGIN(DWT_SLOT_UI);   /* DWT begin */
        /* 获取系统状态快照（互斥保护下的拷贝）*/
        System_GetSnapshot(&snap);

        /* 显示当前模式 */
        /*该函数用于数字转字符串，或者是构建复杂字符串，如最后一个参数值是manual,则dispBuf的结果为Mode:manual*/
        sprintf(dispBuf, "Mode:%s", ModeNames[snap.currentMode]);
        Show_Str(0, 20, BLUE, WHITE, (u8*)dispBuf, 16, 0);

        /* 显示档位 */
        sprintf(dispBuf, "Level:%s", SpeedLevelNames[snap.speedLevel]);
        Show_Str(0, 40, BLUE, WHITE, (u8*)dispBuf, 16, 0);

        /* 显示风速PWM */
        sprintf(dispBuf, "WIND:%.1f%%  ", snap.windSpeedPWM);
        Show_Str(0, 60, BLUE, WHITE, (u8*)dispBuf, 16, 0);

        /* 显示实际转速 */
        sprintf(dispBuf, "RPM:%.0f    ", snap.actualRPM);
        Show_Str(0, 80, BLUE, WHITE, (u8*)dispBuf, 16, 0);

        /* 显示自动模式状态 */
        sprintf(dispBuf, "Auto:%s", AutoStateNames[snap.autoModeState]);
        Show_Str(0, 100, BLUE, WHITE, (u8*)dispBuf, 16, 0);

        /* 显示自动模式启动计时 */
        sprintf(dispBuf, "AMCnt:%ds  ", snap.autoModeCounter / 1000);
        Show_Str(0, 120, BLUE, WHITE, (u8*)dispBuf, 16, 0);

        /* 显示Cooking Event计时 */
        sprintf(dispBuf, "CECnt:%ds  ", snap.cookingEventCounter / 1000);
        Show_Str(0, 140, BLUE, WHITE, (u8*)dispBuf, 16, 0);

        /* 显示故障状态 */
        if (snap.fault == FAULT_MOTOR_STALL)
        {
            Show_Str(0, 160, RED, WHITE, "FAULT:STALL", 16, 0);
        }
        else if (snap.fault == FAULT_SENSOR_TIMEOUT)
        {
            Show_Str(0, 160, RED, WHITE, "FAULT:SENSOR", 16, 0);
        }
        else
        {
            Show_Str(0, 160, BLUE, WHITE, "FAULT:NONE  ", 16, 0);
        }
        DWT_EXEC_END(DWT_SLOT_UI);     /* DWT end */

        /* 待机阻塞在事件组 UI 独立唤醒位，
         * 退出待机广播唤醒；消除待机态 200ms 刷新周期唤醒
         *（待机显示保持静态画面，唤醒后首轮刷新）*/
        if (StandbyPeriodic_ShouldBlock(
                (g_systemState.currentMode == MODE_STANDBY) ? 1u : 0u))
        {
            (void)xEventGroupWaitBits(g_standbyEventGroup,
                                      STANDBY_EVT_UI_BIT,
                                      pdTRUE, pdFALSE, portMAX_DELAY);
        }
        else
        {
            delay_ms(200);  /* 200ms刷新周期 */
        }
    }
}

/*-----------------------------------------------------------
 * 切换工作模式（发送事件请求，由StateManagerTask单一写者决策执行）
 *----------------------------------------------------------*/
void System_SwitchMode(void)
{
    State_SendEvent(STATE_EVENT_KEY_MODE);
}

/*-----------------------------------------------------------
 * 切换档位（发送事件请求，由StateManagerTask单一写者决策执行）
 *----------------------------------------------------------*/
void System_SwitchSpeedLevel(void)
{
    State_SendEvent(STATE_EVENT_KEY_SPEED);
}

/*-----------------------------------------------------------
 * 切换电机开关（发送事件请求，由StateManagerTask单一写者决策执行）
 * 此处仅提供即时的LCD反馈，真实状态以MotorTask同步的为准
 *----------------------------------------------------------*/
void System_ToggleMotor(void)
{
    if (g_systemState.motorRunning)
    {
        Show_Str(0, 0, BLUE, WHITE, "Motor Stopped", 16, 0);
    }
    else
    {
        Show_Str(0, 0, BLUE, WHITE, "Motor Started", 16, 0);
    }
    State_SendEvent(STATE_EVENT_KEY_MOTOR);
}

/*-----------------------------------------------------------
 * 获取系统状态快照（互斥保护下的只读拷贝）
 * UI等消费者通过快照读取，不持有可变全局指针
 *----------------------------------------------------------*/
void System_GetSnapshot(SystemSnapshot_t *snap)
{
    if (snap == NULL)
    {
        return;
    }

    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE)
    {
        snap->currentMode = g_systemState.currentMode;
        snap->speedLevel = g_systemState.speedLevel;
        snap->motorRunning = g_systemState.motorRunning;
        snap->fault = g_systemState.fault;

        snap->temperature = g_systemState.temperature;
        snap->humidity = g_systemState.humidity;
        snap->gasConcentration = g_systemState.gasConcentration;

        snap->windSpeedPWM = g_systemState.windSpeedPWM;
        snap->actualRPM = g_systemState.actualRPM;
        snap->targetRPM = g_systemState.targetRPM;

        snap->cookingEventActive = g_systemState.cookingEventActive;
        snap->antiBackflowActive = g_systemState.antiBackflowActive;

        snap->autoModeCounter = g_systemState.autoModeCounter;
        snap->cookingEventCounter = g_systemState.cookingEventCounter;
        snap->autoModeState = (u8)g_autoModeState;

        xSemaphoreGive(g_dataMutex);
    }
}

/*-----------------------------------------------------------
 * 速度计算任务 - 由TIM4中断触发
 *----------------------------------------------------------*/
void SpeedCalcTask(void *pvParameters)
{
    int encoderCount;
    
    while (1)
    {
        /* 等待TIM4中断发送的信号量 */
        if (xSemaphoreTake(g_speedCalcSemaphore, portMAX_DELAY) == pdTRUE)
        {
            DWT_EXEC_BEGIN(DWT_SLOT_SPEED_CALC);   /* DWT begin */
            /* 获取编码器计数值 */
            encoderCount = get_encoder_value();

            /* 计算电机转速（50ms计算一次） */
            DWT_EXEC_END(DWT_SLOT_SPEED_CALC);     /* DWT end */
            speed = get_speed(encoderCount, 50);
            /* 完成一次转速计算，上报任务心跳 */
            TaskHealth_Heartbeat(TASK_HEALTH_BIT_SPEED_CALC);
        }
    }
}

/*-----------------------------------------------------------
 * TIM2中断服务程序 - 编码器溢出处理
 *----------------------------------------------------------*/
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        g_tim2OverflowIrqCount++;   /* ISR计数（诊断证据）*/

        /* 根据计数方向更新溢出计数器 */
        if (TIM_GetDirection(TIM2))
        {
            overflow--;     /* 递减计数 */
        }
        else
        {
            overflow++;     /* 递增计数 */
        }
    }
}

/*-----------------------------------------------------------
 * TIM4中断服务程序 - 触发速度计算任务
 *----------------------------------------------------------*/
void TIM4_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

        g_tim4IrqCount++;   /* ISR计数（诊断证据）*/

        /* 发送信号量通知SpeedCalcTask执行速度计算 */
        xSemaphoreGiveFromISR(g_speedCalcSemaphore, &xHigherPriorityTaskWoken);

        /* 如果需要，触发上下文切换 */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*-----------------------------------------------------------
 * 诊断统计任务 - 周期10秒（任务8：任务表/栈高水位统计输出）
 * 证据链支撑：串口周期输出各任务栈高水位（历史最小剩余，单位word）、
 * 命令/事件队列满丢弃计数、堆剩余/历史最小、故障码——
 * 用于证明每个任务的栈配给合理（高水位>0且留有裕量），
 * 以及队列容量与堆预算在长时间运行下未被击穿
 *----------------------------------------------------------*/
void DiagnosticTask(void *pvParameters)
{
    while (1)
    {
        DWT_EXEC_BEGIN(DWT_SLOT_DIAG);   /* DWT begin */
        printf("---- Diagnostics ----\r\n");

        /* 任务表：栈高水位=自任务创建以来栈的最小剩余空间(word)，
         * 该值>0说明从未溢出，越小说明越接近极限 */
        printf("StateMgr  stkfree=%u\r\n", (unsigned)uxTaskGetStackHighWaterMark(xStateManagerTaskHandle));
        printf("Motor     stkfree=%u\r\n", (unsigned)uxTaskGetStackHighWaterMark(xMotorControlTaskHandle));
        printf("KeyScan   stkfree=%u\r\n", (unsigned)uxTaskGetStackHighWaterMark(xKeyScanTaskHandle));
        printf("Sensor    stkfree=%u\r\n", (unsigned)uxTaskGetStackHighWaterMark(xSensorTaskHandle));
        printf("WindSpd   stkfree=%u\r\n", (unsigned)uxTaskGetStackHighWaterMark(xWindSpeedTaskHandle));
        printf("UI        stkfree=%u\r\n", (unsigned)uxTaskGetStackHighWaterMark(xUIDisplayTaskHandle));
        printf("SpeedCalc stkfree=%u\r\n", (unsigned)uxTaskGetStackHighWaterMark(xSpeedCalcTaskHandle));
        printf("Diag      stkfree=%u\r\n", (unsigned)uxTaskGetStackHighWaterMark(NULL));   /* NULL=当前任务 */

        /* 队列满丢弃计数（应为0；非0说明队列容量不足或消费过慢）*/
        printf("QFull: motor=%u state=%u buzz=%u\r\n",
               (unsigned)g_motorQueueFullCount,
               (unsigned)g_stateEventFullCount,
               (unsigned)g_buzzerQueueFullCount);

        /* 堆统计：当前剩余与历史最小剩余（最小值反映最坏情况）*/
        printf("Heap: free=%u min=%u\r\n",
               (unsigned)xPortGetFreeHeapSize(),
               (unsigned)xPortGetMinimumEverFreeHeapSize());

        /* 系统健康摘要 */
        printf("Fault=%u Mode=%u RPM=%.0f\r\n",
               (unsigned)g_systemState.fault,
               (unsigned)g_systemState.currentMode,
               g_systemState.actualRPM);

        /* ISR统计计数（ISR内只计数不打印，此处集中输出）。
         * 增量为0且外设已使能说明中断链路故障（NVIC/外设配置丢失）*/
        printf("ISR: tim2ovf=%lu tim4=%lu\r\n",
               (unsigned long)g_tim2OverflowIrqCount,
               (unsigned long)g_tim4IrqCount);

        /* 低功耗统计（三路 Tickless/STOP 路径
         * 计数、唤醒源分布与时钟恢复失败；拒绝原因分计数不折叠成
         * 单一失败值——LpRefuse_t 单一来源。待机阻塞唤醒后输出，
         * 不为打印周期唤醒系统）*/
        {
            static const char* LpRefuseNames[LP_REFUSE_COUNT] = {
                "none", "not-standby", "motor-on", "buzzer-busy",
                "iap-busy", "uart-busy", "clock-bad", "tasks-waiting",
                "iwdg-svc", "no-exti"
            };
            LowPower_Stats_t lp;
            unsigned ri;

            LowPower_SnapshotStats(&lp);
            printf("LP: shallow=%lu rtc=%lu exti=%lu degrade=%lu\r\n",
                   (unsigned long)lp.shallow_count,
                   (unsigned long)lp.rtc_stop_count,
                   (unsigned long)lp.exti_stop_count,
                   (unsigned long)lp.rtc_degrade_count);
            printf("LP: wake key=%lu alarm=%lu other=%lu none=%lu "
                   "clkfail=%lu clamp=%lu\r\n",
                   (unsigned long)lp.wake_key,
                   (unsigned long)lp.wake_alarm,
                   (unsigned long)lp.wake_other,
                   (unsigned long)lp.wake_none,
                   (unsigned long)lp.clk_resume_fail,
                   (unsigned long)lp.tick_clamp_count);
            for (ri = 1u; ri < (unsigned)LP_REFUSE_COUNT; ri++)
            {
                if (lp.refuse_counts[ri] != 0u)
                {
                    printf("LP: refuse %s=%lu\r\n",
                           LpRefuseNames[ri],
                           (unsigned long)lp.refuse_counts[ri]);
                }
            }
        }

#if DWT_PROFILER_ENABLED
        /* deferred DWT stats output (snapshot first, print later).
         * Measured tasks only write RAM via DWT macros - no printf in hot path. */
        {
            DwtSnapshot_t snap;
            unsigned si;
            DWT_SnapshotAndReset(&snap);
            printf("-- DWT snapshot (MHz=%lu) --\r\n",
                   (unsigned long)(snap.cpu_hz / 1000000uL));
            for (si = 0; si < (unsigned)DWT_SLOT_MAX; si++) {
                const DwtSlotSnapshot_t *p = &snap.slots[si];
                if (p->exec_count == 0u && p->period_count == 0u) continue;
                printf("%-9s exec n=%lu min=%lu max=%lu sum=%lu | period n=%lu min=%lu max=%lu (cyc)\r\n",
                       DWT_SlotName((DwtSlotId_t)si),
                       (unsigned long)p->exec_count,
                       (unsigned long)p->exec_cycles_min,
                       (unsigned long)p->exec_cycles_max,
                       (unsigned long)p->exec_cycles_sum,
                       (unsigned long)p->period_count,
                       (unsigned long)p->period_cycles_min,
                       (unsigned long)p->period_cycles_max);
            }
        }
#endif /* DWT_PROFILER_ENABLED */
        DWT_EXEC_END(DWT_SLOT_DIAG);     /* DWT end */
        /* 待机阻塞在事件组 DIAG 独立唤醒位，
         * 退出待机广播唤醒；消除待机态 10s 诊断打印周期唤醒
         *（设计：诊断计数在唤醒后输出，不为打印周期唤醒系统）*/
        if (StandbyPeriodic_ShouldBlock(
                (g_systemState.currentMode == MODE_STANDBY) ? 1u : 0u))
        {
            (void)xEventGroupWaitBits(g_standbyEventGroup,
                                      STANDBY_EVT_DIAG_BIT,
                                      pdTRUE, pdFALSE, portMAX_DELAY);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(10000));   /* 10秒统计周期 */
        }
    }
}

/*-----------------------------------------------------------
 * FreeRTOS诊断钩子（任务5：RTOS诊断）
 * 内核配置见FreeRTOSConfig.h：
 *  - configCHECK_FOR_STACK_OVERFLOW=2  上下文切换时检查栈溢出
 *  - configUSE_MALLOC_FAILED_HOOK=1    堆分配失败回调
 *  - configASSERT                      内核断言（printf报告文件/行号）
 * 说明：钩子在临界上下文执行，必须短小且不允许返回。
 * 栈溢出/堆耗尽均为不可恢复的致命错误：安全停机后挂起系统，
 * 由看门狗（若启用）或人工复位恢复。
 *----------------------------------------------------------*/

/* 栈溢出钩子：内核在任务切换时检测到某任务栈越界（模式2：
 * 校验栈底魔数0xA5A5A5A5被破坏，能发现已经发生的溢出）*/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;

    /* 故障安全：直接断开电机功率级。
     * 例外说明：此处绕过电机命令队列直接操作硬件是故意的——
     * 系统栈已损坏，队列/调度器不可信，唯一安全的动作是
     * 寄存器级断电（PWM通道关闭+H桥禁止）*/
    motor_stop();
    TIM_SetCompare1(TIM1, 0);

    /* 可听见报警：FATAL状态持续鸣叫（例外——Hook内
     * 调度器/队列不可信，绕过命令队列直接操作GPIO是故意的）*/
    Beep_on(&bep);

    printf("[FATAL] Stack overflow in task: %s\r\n", pcTaskName);

    taskDISABLE_INTERRUPTS();
    for (;;)
    {
        /* 挂起等待复位：禁止返回（返回后内核状态未定义）*/
    }
}

/* 堆分配失败钩子：xTaskCreate/xQueueCreate等内核对象
 * 分配失败时回调（FreeRTOS V9无失败返回路径，任务创建
 * 静默失败，此钩子是唯一可见的诊断手段）*/
void vApplicationMallocFailedHook(void)
{
    /* 故障安全：电机停机（理由同上，系统资源已耗尽）*/
    motor_stop();
    TIM_SetCompare1(TIM1, 0);

    /* 可听见报警：FATAL状态持续鸣叫（Hook例外，直呼GPIO）*/
    Beep_on(&bep);

    printf("[FATAL] Heap exhausted, free=%u bytes\r\n",
           (unsigned int)xPortGetFreeHeapSize());

    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}
