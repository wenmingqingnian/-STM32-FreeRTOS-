/*
 * 应用层任务头文件
 * 基于FreeRTOS的油烟机控制系统
 *
 */
#ifndef __APP_TASKS_H
#define __APP_TASKS_H

#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"   /* 待机/活动广播事件组 */
/* DWT profiler. Build switch defaults live in BSP/DIAG headers
 * (dwt_profiler.h / prio_inversion.h) so every translation unit agrees. */
#include "dwt_profiler.h"
/* task heartbeat + health supervision (HealthTask + IWDG).
 * Build switch defaults live in APP_TASK/task_health.h and BSP/WATCHDOG/watchdog.h. */
#include "task_health.h"
/* 故障判定纯函数 + 故障码/堵转/传感器阈值集中于此
 *（单一来源，主机 mock 可测）。原内联判定已抽为纯函数，
 * app_tasks.c 改为调用，行为等价（Keil 0错误0警告 + mock 全绿为证）。 */
#include "fault_decision.h"

/*-----------------------------------------------------------
 * 工作模式定义
 *----------------------------------------------------------*/
typedef enum {
    MODE_STANDBY = 0,       /* 待机模式 */
    MODE_MANUAL,            /* 手动模式 */
    MODE_AUTO,              /* 自动模式 */
    MODE_ANTI_BACKFLOW      /* 防回流模式 */
} WorkMode_t;

/*-----------------------------------------------------------
 * 手动模式档位定义
 *----------------------------------------------------------*/
typedef enum {
    SPEED_LOW = 0,          /* 低档 */
    SPEED_HIGH              /* 高档 */
} SpeedLevel_t;

/*-----------------------------------------------------------
 * 电机命令定义（电机硬件单一所有者设计）
 * MotorControlTask是PWM/方向/H桥的唯一硬件所有者，
 * 其他任务禁止直接调用motor_start/stop/dir/pwm_set，
 * 只能通过g_motorCmdQueue发送命令请求电机动作
 *----------------------------------------------------------*/
typedef enum {
    MOTOR_CMD_STOP = 0,     /* 安全停机：PWM清零+关闭H桥+PID清积分 */
    MOTOR_CMD_RUN_RPM,      /* 闭环运行到指定目标转速（value=目标RPM）*/
    MOTOR_CMD_RUN_PWM,      /* 开环运行到指定PWM比较值（value=CCR，自动模式用）*/
    MOTOR_CMD_CLEAR_FAULT   /* 清除故障锁存（故障处理任务实现）*/
} MotorCommandType_t;

typedef struct {
    MotorCommandType_t type;    /* 命令类型 */
    int16_t value;              /* 命令参数：RUN_RPM为目标转速，RUN_PWM为PWM比较值 */
    TickType_t timestamp;       /* 命令产生时刻（诊断用）*/
} MotorCommand_t;

#define MOTOR_CMD_QUEUE_LEN     4       /* 命令队列长度 */

/*-----------------------------------------------------------
 * 状态事件定义（StateManager单一写者）
 * 其他任务通过g_stateEventQueue请求状态变化，
 * 禁止在StateManagerTask之外直接写控制平面字段
 *（currentMode/speedLevel/targetRPM/自动模式计时器/防回流状态）
 *----------------------------------------------------------*/
typedef enum {
    STATE_EVENT_KEY_MODE = 0,       /* 按键1短按：切换工作模式请求 */
    STATE_EVENT_KEY_SPEED,          /* 按键2短按：切换档位请求 */
    STATE_EVENT_KEY_MOTOR,          /* 按键2长按：电机开关请求 */
    STATE_EVENT_FAULT_STALL,        /* MotorTask堵转检测上报（硬件所有者本地已安全停机）*/
    STATE_EVENT_SENSOR_TIMEOUT,     /* SensorTask连续读取失败上报（数据失效）*/
    STATE_EVENT_SENSOR_RECOVERED    /* SensorTask连续读取成功上报（数据恢复）*/
} StateEventType_t;

typedef struct {
    StateEventType_t type;      /* 事件类型 */
} StateEvent_t;

#define STATE_EVENT_QUEUE_LEN   8       /* 状态事件队列长度 */

/*-----------------------------------------------------------
 * 蜂鸣器命令定义（异步蜂鸣器）
 * KeyScanTask/StateManagerTask只发命令不阻塞等待鸣叫完成，
 * 实际鸣叫时序由低优先级BuzzerTask执行——
 * 发送方（10ms/50ms周期任务）不再被Buzzer_Beep的100ms阻塞拖慢
 *----------------------------------------------------------*/
typedef enum {
    BUZZER_CMD_BEEP = 0,   /* 定时鸣叫：on→delay(duration_ms)→off */
    BUZZER_CMD_ON,         /* 持续鸣叫开始（长按提示，需边沿触发）*/
    BUZZER_CMD_OFF         /* 持续鸣叫结束（按键释放，需边沿触发）*/
} BuzzerCmdType_t;

typedef struct {
    BuzzerCmdType_t type;      /* 命令类型 */
    u16 duration_ms;           /* BEEP命令的鸣叫时长（ON/OFF忽略）*/
} BuzzerCmd_t;

#define BUZZER_CMD_QUEUE_LEN    2       /* 命令队列长度（plan规定）*/

/*-----------------------------------------------------------
 * 系统状态结构体
 *----------------------------------------------------------*/
typedef struct {
    WorkMode_t currentMode;         /* 当前工作模式 */
    SpeedLevel_t speedLevel;        /* 当前档位 */
    u8 motorRunning;                /* 电机运行状态,0表示停止，1表示运行 */
    
    u8 temperature;                 /* 温度值 */
    u8 humidity;                    /* 湿度值 */
    float gasConcentration;         /* 气体浓度值 */
    
    float windSpeedPWM;             /* 风速PWM值(%) */
    float actualRPM;                /* 实际转速 */
    u16 targetRPM;                  /* 目标转速 */
    
    u8 cookingEventActive;          /* Cooking Event激活标志 */
    
    u8 antiBackflowActive;          /* 防回流激活标志 */
    float gasThreshold;             /* 当前气体浓度阈值 */
    
    u32 autoModeCounter;            /* 自动模式启动阶段计时器(ms) */
    u32 cookingEventCounter;        /* Cooking Event计时器(ms) */

    u8 fault;                       /* 故障码，0=无故障（故障处理任务写入）*/
} SystemState_t;

/* 故障码/堵转/传感器阈值已移入 fault_decision.h（单一来源，主机 mock 可测）。
 * 此处通过上方 #include "fault_decision.h" 间接引入：
 *   FAULT_NONE / FAULT_MOTOR_STALL / FAULT_SENSOR_TIMEOUT
 *   MOTOR_STALL_RPM_THRESHOLD / MOTOR_STALL_PWM_THRESHOLD /
 *   MOTOR_STALL_WINDOW_MS / MOTOR_START_GRACE_MS
 *   SENSOR_TIMEOUT_FAILS / SENSOR_RECOVER_SUCCESSES
 *（抽纯函数不改阈值数值） */

/* 传感器超时降级风量：自动模式弃用融合算法，固定输出安全比较值（30%）*/
#define AUTO_SAFE_PWM_COMPARE       300

/*-----------------------------------------------------------
 * 系统状态快照（UI等只读消费者使用拷贝，不持有可变全局指针）
 *----------------------------------------------------------*/
typedef struct {
    WorkMode_t currentMode;         /* 当前工作模式 */
    SpeedLevel_t speedLevel;        /* 当前档位 */
    u8 motorRunning;                /* 电机运行状态 */
    u8 fault;                       /* 故障码 */

    u8 temperature;                 /* 温度值 */
    u8 humidity;                    /* 湿度值 */
    float gasConcentration;         /* 气体浓度值 */

    float windSpeedPWM;             /* 风速PWM值(%) */
    float actualRPM;                /* 实际转速 */
    u16 targetRPM;                  /* 目标转速 */

    u8 cookingEventActive;          /* Cooking Event激活标志 */
    u8 antiBackflowActive;          /* 防回流激活标志 */

    u32 autoModeCounter;            /* 自动模式启动阶段计时器(ms) */
    u32 cookingEventCounter;        /* Cooking Event计时器(ms) */
    u8 autoModeState;               /* 自动模式状态机状态 */
} SystemSnapshot_t;

/*-----------------------------------------------------------
 * 时序参数定义(ms)
 *----------------------------------------------------------*/
#define AUTO_MODE_STARTUP_TIME      60000   /* 自动模式启动等待时间：60秒 */
#define COOKING_EVENT_TIMEOUT       60000   /* Cooking Event超时时间：60秒 */
#define COOKING_EVENT_DELAY_OFF     10000   /* Cooking Event结束后延时关闭：10秒 */


/*-----------------------------------------------------------
 * 任务优先级定义（数值越大优先级越高）
 * 设计依据：电机闭环(5) > 状态决策(4) > 按键(3) > 传感器/风量(2) > UI(1)，
 * 速度计算(6)最高保证及时响应编码器采样
 *----------------------------------------------------------*/
#define TASK_START_PRIORITY         1       /* 开始任务优先级 */
#define TASK_KEY_PRIORITY           3       /* 按键扫描任务优先级 */
#define TASK_SENSOR_PRIORITY        2       /* 传感器采集任务优先级 */
#define TASK_WIND_SPEED_PRIORITY    2       /* 风速计算任务优先级 */
#define TASK_STATE_MANAGER_PRIORITY 4       /* 状态管理任务优先级（控制平面单一写者）*/
#define TASK_MOTOR_PRIORITY         5       /* 电机控制任务优先级 */
#define TASK_UI_PRIORITY            1       /* UI显示任务优先级 */
#define TASK_SPEED_CALC_PRIORITY    6       /* 速度计算任务优先级（最高，保证及时响应中断） */
#define TASK_DIAGNOSTIC_PRIORITY    1       /* 诊断统计任务优先级（最低，后台输出）*/
#define TASK_HEALTH_PRIORITY        1       /* 健康监督任务优先级（低：监督者不需抢占业务）*/
#define TASK_BUZZER_PRIORITY        1       /* 蜂鸣器任务优先级（最低：UI反馈类，可被任何业务任务抢占）*/

/*-----------------------------------------------------------
 * 任务栈大小定义
 *----------------------------------------------------------*/
#define TASK_START_STK_SIZE         64     /* 开始任务栈大小 */
#define TASK_KEY_STK_SIZE           64     /* 按键扫描任务栈大小 */
#define TASK_SENSOR_STK_SIZE        128    /* 传感器采集任务栈大小 */
#define TASK_WIND_SPEED_STK_SIZE    64     /* 风速计算任务栈大小 */
#define TASK_STATE_MANAGER_STK_SIZE 128    /* 状态管理任务栈大小 */
#define TASK_MOTOR_STK_SIZE         256    /* 电机控制任务栈大小 */
#define TASK_UI_STK_SIZE            256    /* UI显示任务栈大小 */
#define TASK_SPEED_CALC_STK_SIZE    128    /* 速度计算任务栈大小 */
#define TASK_DIAGNOSTIC_STK_SIZE    128    /* 诊断统计任务栈大小 */
#define TASK_HEALTH_STK_SIZE        128    /* 健康监督任务栈大小 */
#define TASK_BUZZER_STK_SIZE        64     /* 蜂鸣器任务栈大小（GPIO+vTaskDelay，无printf）*/


/*-----------------------------------------------------------
 * 函数声明
 *----------------------------------------------------------*/
/* 系统初始化 */
void System_Init(void);

/* 创建开始任务 */
void StartTask_Create(void);

/* 任务函数声明 */
void StartTask(void *pvParameters);         /* 开始任务 */
void KeyScanTask(void *pvParameters);       /* 按键扫描任务 */
TaskHandle_t App_GetKeyScanTaskHandle(void); /* 按键扫描任务句柄访问器（EXTI1/USART1 RX ISR 通知目标）*/
void SensorTask(void *pvParameters);        /* 传感器采集任务 */
void WindSpeedTask(void *pvParameters);     /* 风速计算任务 */
void StateManagerTask(void *pvParameters);  /* 状态管理任务（控制平面单一写者）*/
void MotorControlTask(void *pvParameters);  /* 电机控制任务 */
void UIDisplayTask(void *pvParameters);     /* UI显示任务 */
void SpeedCalcTask(void *pvParameters);     /* 速度计算任务（由TIM4中断触发） */
void DiagnosticTask(void *pvParameters);    /* 诊断统计任务（任务表/栈高水位输出） */
void BuzzerTask(void *pvParameters);        /* 蜂鸣器任务（异步执行鸣叫时序） */

/* 模式切换函数（已改为向StateManager发送事件请求，不再直接写状态）*/
void System_SwitchMode(void);               /* 切换工作模式 */
void System_SwitchSpeedLevel(void);         /* 切换档位 */
void System_ToggleMotor(void);              /* 切换电机开关 */

/* 获取系统状态快照（只读拷贝）*/
void System_GetSnapshot(SystemSnapshot_t *snap);

/* 电机命令发送接口（供外部模块间接控制电机，禁止直接调motor_*）*/
BaseType_t Motor_SendCommand(MotorCommandType_t type, int16_t value);

/* 蜂鸣器命令发送接口（非阻塞，鸣叫时序由BuzzerTask执行）*/
BaseType_t Buzzer_SendCommand(BuzzerCmdType_t type, u16 duration_ms);

/* 电机命令队列与诊断计数（供诊断任务输出）*/
extern QueueHandle_t g_motorCmdQueue;
extern volatile u32 g_motorQueueFullCount;      /* 命令队列满丢弃计数 */

/* 状态事件队列与诊断计数（供诊断任务输出）*/
extern QueueHandle_t g_stateEventQueue;
extern volatile u32 g_stateEventFullCount;      /* 状态事件队列满丢弃计数 */

/* ISR统计计数（ISR内只计数不打印，由DiagnosticTask输出）*/
extern volatile u32 g_tim2OverflowIrqCount;    /* TIM2编码器溢出中断次数 */
extern volatile u32 g_tim4IrqCount;            /* TIM4周期中断次数 */

/* 蜂鸣器命令队列与诊断计数 */
extern QueueHandle_t g_buzzerCmdQueue;
extern volatile u32 g_buzzerQueueFullCount;    /* 蜂鸣器命令队列满丢弃计数 */

/* 待机广播事件组：
 * StateManagerTask 在模式边沿置位（进待机 STANDBY_EVT_STANDBY_BIT /
 * 退出待机 STANDBY_EVT_ACTIVE_BIT，位定义单一来源 standby_policy.h）；
 * Sensor/WindSpeed/UI/Diagnostic/HealthTask 待机阻塞在事件组上，
 * 消除待机态全部周期唤醒源 */
extern EventGroupHandle_t g_standbyEventGroup;

#endif /* __APP_TASKS_H */
