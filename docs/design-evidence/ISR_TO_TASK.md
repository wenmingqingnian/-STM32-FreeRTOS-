# ISR到任务层的数据通路与中断纪律（plan Task 4）

> 证据等级：L1（源码审计）+ L2（Keil 0错误0警告构建）。
> 本文所有行号对应 `APP_TASK/app_tasks.c`（commit见git log，Task 4提交）。
> 无L3板端数据：ISR计数打印需实机运行方可引用，当前仅声明机制。

## 1. 中断源清单与职责

| ISR | 触发源 | 周期/频率 | 职责 | 通知对象 |
|-----|--------|-----------|------|----------|
| `DMA1_Channel5_IRQHandler` (app_tasks.c:1169) | DMA传输完成（IAP固件接收） | 每次DMA块传输1次 | 清标志、计数、释放信号量 | IAP任务（`g_iapSemaphore`） |
| `TIM2_IRQHandler` (app_tasks.c:1285) | TIM2更新事件（编码器溢出） | 编码器计数回绕时 | 清标志、计数、维护`overflow` | 无（数据由SpeedCalcTask按需读取） |
| `TIM4_IRQHandler` (app_tasks.c:1308) | TIM4更新事件（速度计算节拍） | 1ms固定周期（72MHz/14400/5） | 清标志、计数、释放信号量 | SpeedCalcTask（`g_speedCalcSemaphore`） |
| `USART1_IRQHandler` (SYSTEM/usart/usart.c:118) | 串口接收 | 波特率相关 | 纯接收状态机（原厂代码） | 无（轮询读取`USART_RX_STA`） |

## 2. ISR内允许的操作纪律

整改后所有ISR仅包含三类操作（FreeRTOS中断纪律）：

1. **清中断标志**——`DMA_ClearITPendingBit` / `TIM_ClearITPendingBit`
2. **统计计数**——`g_dmaIrqCount++` 等volatile计数（新增，替代printf）
3. **FromISR API**——`xSemaphoreGiveFromISR` + `portYIELD_FROM_ISR`

**已删除**：`DMA1_Channel5_IRQHandler` 内的 `#if DEBUG printf("CNDTR: %d\n", ...)`。
理由：printf重入非线程安全、执行时长不可控（毫秒级），在ISR上下文中
既阻塞同优先级以下中断，又可能破坏被其他上下文占用的UART状态。

ISR存活证据改由计数承担：`DiagnosticTask`（10秒周期）集中打印

```
ISR: dma=<n> tim2ovf=<n> tim4=<n>
```

（app_tasks.c:1368）。计数增量持续为0而外设已使能，即可判定中断链路
故障（NVIC配置丢失/外设时钟关闭），无需在ISR内打印即可定位。

## 3. NVIC优先级审计（FromISR合法性）

NVIC分组：`NVIC_PriorityGroup_4`（main.c:39）——4位全抢占，0位子优先级。

FreeRTOS约束（FreeRTOSConfig.h:186）：

```
configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 3
```

即：**抢占优先级数值 0~2 的中断禁止调用任何FromISR API**（内核屏蔽
无法覆盖它们，调用会破坏内核临界区）；3~15 可安全调用。

| 中断 | 抢占优先级 | 调用FromISR？ | 数值≥3？ | 结论 |
|------|-----------|--------------|----------|------|
| DMA1_Ch5 | 4（dma.c:42） | 是 | 4>3 | 合规 |
| TIM4 | 5（motor.c:261） | 是 | 5>3 | 合规 |
| TIM2 | 6（motor.c:232） | 否（纯计数） | 6>3 | 合规 |
| USART1 | 未显式配置（默认0） | 否（纯标志/缓冲） | — | 不适用：未调用任何内核API |

已知边界：USART1抢占优先级为复位默认值0，但它不调用任何FromISR API，
纯缓冲区写入，不违反FreeRTOS约束；如未来需在串口ISR中通知任务，
必须先将其优先级改到3~15。

## 4. ISR→任务数据通路图

```
TIM4更新(1ms节拍)              DMA1_Ch5传输完成            TIM2计数回绕
      │                              │                        │
      │ 清标志+计数                   │ 清标志+计数             │ 清标志+计数
      ▼                              ▼                        ▼
xSemaphoreGiveFromISR         xSemaphoreGiveFromISR       overflow++/--
      │                       (g_iapSemaphore)                │
      ▼                              │                        ▼
SpeedCalcTask被唤醒                  ▼（无直接通知，
读TIM2->CNT + overflow         IAP任务被唤醒           SpeedCalc下次
get_speed(50ms窗口)换算RPM     校验/提交固件            采样时读取）
      │                              │                        │
      └──────────────┬───────────────┴────────────────────────┘
                     ▼
        DiagnosticTask（10s周期）：打印ISR计数/栈水位/队列统计
        —— ISR内零打印，所有观测数据延迟到任务层输出
```

## 5. 计数器定义（诊断证据锚点）

| 变量 | 定义位置 | 语义 |
|------|---------|------|
| `g_dmaIrqCount` | app_tasks.c:49 | DMA1_Ch5传输完成中断累计次数 |
| `g_tim2OverflowIrqCount` | app_tasks.c:50 | TIM2编码器溢出累计次数（方向信息在`overflow`变量） |
| `g_tim4IrqCount` | app_tasks.c:51 | TIM4速度节拍累计次数（1ms周期，理论每10秒打印窗口增加10000次） |

均为 `volatile u32`，单写者（各自ISR）单读者（DiagnosticTask打印），
u32对齐读写在Cortex-M3上原子，无需额外保护。

## 6. 设计要点

**Q: 为什么ISR里不能printf？**
A: 三个原因——(1) printf非重入，UART外设状态可能被任务上下文占用，ISR内
调用会损坏其状态机；(2) 执行时长不可控（格式化+波特率发送，115200下
一个20字符printf约1.7ms），期间同优先级以下中断全部被阻塞；
(3) 违反ISR最短路径原则。替代方案：ISR只写volatile计数，诊断任务
延迟集中打印（与DWT profiler的"测量热路径零打印"同一纪律）。

**Q: 怎么验证ISR优先级配置是对的？**
A: 两条规则——(1) 调用FromISR API的中断，其抢占优先级数值必须大于
等于configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY（本项目=3），否则
内核临界区（basepri屏蔽）盖不住它；(2) NVIC分组必须与FreeRTOS假设
一致——本项目Group_4全抢占，子优先级不参与屏蔽判断。三者（DMA=4、
TIM4=5、TIM2=6）均满足。

**Q: volatile计数在ISR和任务间需要临界区吗？**
A: 本场景不需要。单写者（一个ISR）单读者（诊断任务），Cortex-M3上
对齐的32位读写是单指令原子操作；volatile仅禁止编译器缓存/重排。
若多写者（多个ISR写同一计数）就需要`taskENTER_CRITICAL_FROM_ISR`
或改用原子操作。
