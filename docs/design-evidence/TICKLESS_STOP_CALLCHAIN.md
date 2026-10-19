# FreeRTOS 三路 Tickless/STOP 调用链（plan Task 15）

> 证据等级：L1（源码审计）+ L2（Keil 0错误0警告构建）。
> 决策纯函数（LpPolicy\_Decide/RefuseReason）、RTC 换算（LpMath）、
> EXTI-only 后端序列（ExtiStop）、RTC 时间基（RtcTimebase）均另有
> host mock 覆盖（test\_lowpower\_policy / test\_lowpower\_math /
> test\_exti\_stop\_backend / test\_rtc\_timebase，共 900+ checks）。
> 无L3板端数据：深睡进出计数需实机运行方可引用，当前仅声明机制。

## 1. 调用链总览（内核 → wrapper → 三路分发 → 恢复）

```
prvIdleTask (FreeRTOS/tasks.c:3197, configUSE_TICKLESS_IDLE=1)
  │
  ├─ xExpectedIdleTime = prvGetExpectedIdleTime()      tasks.c:3206（未挂起预检）
  ├─ if (xExpectedIdleTime >= configEXPECTED_IDLE_TIME_BEFORE_SLEEP)  [默认2]
  │    │
  │    ├─ vTaskSuspendAll()                            tasks.c:3210（调度器挂起）
  │    ├─ xExpectedIdleTime = prvGetExpectedIdleTime() tasks.c:3216（挂起后复采）
  │    │
  │    ├─ portSUPPRESS_TICKS_AND_SLEEP(xExpectedIdleTime)  tasks.c:3221
  │    │      │
  │    │      │  FreeRTOS/portable/RVDS/ARM_CM3/portmacro.h:163
  │    │      │  #define portSUPPRESS_TICKS_AND_SLEEP(x) LowPower_SuppressTicksAndSleep(x)
  │    │      ▼
  │    │  LowPower_SuppressTicksAndSleep               BSP/LOWPOWER/lowpower.c:631
  │    │      │
  │    │      ├─ 未绑定/未装配 → 官方浅睡 vPortSuppressTicksAndSleep（lowpower.c:640-645）
  │    │      ├─ __disable_irq + DSB/ISB（PRIMASK 屏蔽，非 BASEPRI）   lowpower.c:649
  │    │      ├─ eTaskConfirmSleepModeStatus()          lowpower.c:654
  │    │      │    eAbortSleep → 恢复 PRIMASK → 官方浅睡（657-660）
  │    │      ├─ prvAssembleSnapshot（真实硬件状态快照）  lowpower.c:665
  │    │      ├─ LpPolicy_RefuseReason → refuse_counts[] 分计数  lowpower.c:666-670
  │    │      ├─ LpPolicy_Decide → 三态决策              lowpower.c:671
  │    │      │
  │    │      ├─ LP_STOP_RTC      → prvStopRtcPath      lowpower.c:675 → 509
  │    │      ├─ LP_STOP_EXTI_ONLY→ prvStopExtiOnlyPath lowpower.c:684 → 472
  │    │      └─ LP_SHALLOW_TICKLESS（兜底/拒绝/降级）
  │    │            → 恢复 PRIMASK → vPortSuppressTicksAndSleep  lowpower.c:694-696
  │    │
  │    └─ xTaskResumeAll()                              tasks.c:3229（调度器恢复）
```

要点：**wrapper 永不返回失败**——每条路径退出时 PRIMASK/外设/SysTick
均已恢复，浅睡兜底保证无"scheduler suspended 后死机"路径
。

## 2. 三路决策表（LpPolicy\_Decide 单一来源）

| 路径                    | 触发条件                                                     | tick 处理                               | 唤醒源                      |
| --------------------- | -------------------------------------------------------- | ------------------------------------- | ------------------------ |
| LP\_STOP\_RTC         | 公共门槛全过 + LSE 板型 RTC ready + idle > guard                 | RTC 差值补偿 vTaskStepTick（§6.5，clamp 计数） | RTC Alarm（EXTI17）优先，按键次之 |
| LP\_STOP\_EXTI\_ONLY  | 公共门槛全过 + eNoTasksWaitingTimeout + IWDG 无需服务 + 按键 EXTI 在场 | **tick 冻结不补偿**（不调 vTaskStepTick）      | 按键 EXTI（PB1），其他 EXTI 线计数 |
| LP\_SHALLOW\_TICKLESS | 其余一切（未绑定/拒绝/深睡序列降级）                                      | 官方 SysTick 重载算法                       | 任意中断                     |

公共门槛（任一不过即拒深睡，拒绝原因**分别计数**不折叠）：
MODE\_STANDBY / 电机物理关断 / 蜂鸣器空闲 / IAP 空闲 / UART 发送完成 /
系统时钟健康；EXTI 前提：无任务等待超时 + IWDG 无周期服务 + EXTI 线在场。

本板（BOARD\_C8\_LSE=0）实际可达深睡路径仅 EXTI-only；RTC 路径完整编译
（L2 验证）但被板型门禁拦住（rtc\_timebase stub → ready 恒 0）。

## 3. EXTI-only STOP 序列（本板唯一深睡路径）

进入（ExtiStop\_Enter，Task 12 后端 + wrapper 决策前置）：

1. 决策层确认 eNoTasksWaitingTimeout + IWDG 无需服务（决策与后端双重校验）
2. 停 SysTick → 外设静默（TIM2/TIM4 停止+清 pending、USART1 RXNE 屏蔽、
   ADC 断电、LCD 背光保存熄灭）→ PDDS=0 LPDS=1（STOP 模式）
3. 保留按键 EXTI 线 → **清旧 EXTI pending → DSB/ISB → WFI**
（按键在 clear 与 WFI 间到达形成 pending，WFI 立即返回——无 sleep race）

唤醒后（仍处于 PRIMASK 屏蔽内）：

1. 唤醒源分类（wake\_key / wake\_other / wake\_none 分计数）
2. 72MHz 时钟恢复（HSE→PLL，有界超时）

   * 失败 → fail-safe 复位（外设按 72MHz 配置，不得在 8MHz HSI 下运行）
3. 外设恢复 → SysTick 恢复 → 退出 PRIMASK（按键 ISR 通知 KeyScanTask）

tick 冻结语义：待机态下周期任务全部阻塞在事件组（Task 14），无绝对
时间依赖，tick 停走不影响正确性；唤醒由按键 EXTI 驱动状态机恢复。

## 4. 快照源绑定（APP 层适配，app\_tasks.c:128-196）

| 快照字段                   | 适配器                       | 读取的真实状态                                         |
| ---------------------- | ------------------------- | ----------------------------------------------- |
| standby\_mode          | App\_LpStandbyMode        | g\_systemState.currentMode == MODE\_STANDBY     |
| motor\_physically\_off | App\_LpMotorPhysicallyOff | TIM1 CCER CC1E/CC1NE==0 且 MOTOR\_EN(PA2) ODR==0 |
| buzzer\_idle           | App\_LpBuzzerIdle         | StandbyBuzzer\_StopPermitted(队列空, bep 输出电平)     |
| iap\_idle              | App\_LpIapIdle            | 恒 1（升级请求即复位进 Boot，无挂起会话）                        |
| uart\_dma\_idle        | App\_LpUartIdle           | USART1 SR.TC==1（printf 通道发送完成）                  |
| key\_exti\_available   | App\_LpKeyExtiAvailable   | EXTI IMR & KEY\_EXTI\_LINE\_MASK（EXTI1）         |

绑定时机：System\_Init 末尾（app\_tasks.c:249，未绑定=绝不深睡）；
后端装配：StartTask 中 KeyExti\_TargetInit 之后（app\_tasks.c:325，
唤醒线在场后深睡序列才安全）。

## 5. 诊断统计（DiagnosticTask 唤醒后输出，app\_tasks.c:1528-1564）

```
LP: shallow=<n> rtc=<n> exti=<n> degrade=<n>
LP: wake key=<n> alarm=<n> other=<n> none=<n> clkfail=<n> clamp=<n>
LP: refuse not-standby=<n> motor-on=<n> ...（仅非零项）
```

读取走 LowPower\_SnapshotStats（BASEPRI 临界区快照防撕裂，
DWT\_SnapshotAndReset 同惯例）；拒绝原因按 LpRefuse\_t 独立计数，
不折叠成单一失败值。

## 6. 构建证据（L2）

* `USER/project.uvprojx`（target project）：0 Error / 0 Warning，
  Code=47456 RO=7064 RW=444 ZI=13244

* `USER/app_audit.uvprojx`（target App\_SizeAudit）：0 Error / 0 Warning

* `USER/app_c8_provisional.uvprojx`（target App\_C8\_Provisional；Task 16 已随 CB 迁移重命名为 `app_cb_provisional.uvprojx`）：
  **L6406E 链接失败——APP 需 54964 B 超出临时容量 51200 B**（12 KiB
  Boot 临时布局）。属容量问题非代码问题，为 Task 16 布局收敛的直接
  输入：C8（64 KiB Flash，最大 APP\_CAPACITY=55296 B）扣除 2 KiB 硬
  余量后已无法容纳当前 APP（54964+2048=57012 > 55296），Task 16 按
  计划生成 CB 布局（不靠缩减硬余量/删除错误处理过关）。
  **Task 16 已收口**：CB（128 KiB）冻结布局 `BOOT_PART=0x3800/
  APP_BASE=0x08003800/APP_CAPACITY=0x1C000`，App bin 54620 B ≤
  112640 B（余量 58020 B），`Boot_CB_Provisional`/`App_CB_Provisional`
  双 0E0W 且两遍构建收敛一致（`logs/iap-stop-build.log`）。

## 7. 关键纪律备忘

* 深睡入口二次检查用 **PRIMASK**（\_\_disable\_irq），BASEPRI 临界区
  屏蔽不了数值 0-2 的高优先级中断（FreeRTOSConfig.h
  configLIBRARY\_MAX\_SYSCALL\_INTERRUPT\_PRIORITY=3）

* 浅睡路径在 PRIMASK **恢复后**才进入官方 weak（官方自带临界区管理）

* RTC 路径 vTaskStepTick 步进被 clamp（不越过下一个解阻塞点）时
  tick\_clamp\_count 计数，不静默吞掉

* 时钟恢复失败统一 fail-safe 复位：不留下"外设配置与实际时钟不匹配
  还在跑"的活锁态
