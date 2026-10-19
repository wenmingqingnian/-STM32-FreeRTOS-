# 智能油烟机控制系统（STM32F103 + FreeRTOS）

基于 STM32F103RCT6（高密度 F1，256 KiB Flash / 2 KiB 页）与 FreeRTOS 的嵌入式实时控制系统：多任务架构、多传感器加权融合、电机 PID 闭环调速、故障降级与双分区 IAP 固件升级。

## 功能特性

- **多任务实时架构**：11 个任务按实时性分级，队列 / 事件组 / 二值信号量完成同步与数据传递；任务心跳监控 + IWDG 看门狗兜底（`WATCHDOG_ENABLED` 构建开关，默认关闭）
- **传感器与信号处理**：DHT11 温湿度、MQ2 气体浓度、按键输入；多级软件滤波；温度 / 湿度 / 气体**加权融合**（权重 0.2 / 0.2 / 0.6）自动调节风量
- **电机闭环控制**：TIM2 编码器 4 倍频测速（截尾均值 + 一阶低通）+ 位置式 PID 调速；TIM1 1 kHz 带死区 PWM 驱动 H 桥；堵转检测与故障锁存
- **防回流**：气体浓度双阈值滞回，抑制风机频繁启停震荡
- **故障降级**：传感器失效自动降级为固定风量，条件满足后自动恢复
- **IAP 固件升级**：串口 DMA 分帧协议 + CRC32 双重校验；A/B 双分区 + 双页元数据交替写入，实现断电安全升级；多重校验后无栈跳转（VTOR 重映射）
- **低功耗**：空闲 Tickless 浅睡；待机 STOP 深睡（RTC 闹钟 / 按键 EXTI 唤醒）与 RTC tick 补偿
- **主机侧测试**：FreeRTOS mock 测试套件，纯函数抽取 + 故障注入（堵转 / 传感器超时 / 队列满）

## 任务架构

| 任务 | 优先级 | 节拍 | 职责 |
| --- | --- | --- | --- |
| SpeedCalc | 6 | TIM4 中断驱动，50 ms | 编码器测速与三重滤波 |
| Motor | 5 | 50 ms | PID 闭环调速、堵转检测 |
| StateMgr | 4 | 50 ms | 模式状态机（控制平面单一写者）、降级/恢复 |
| KeyScan | 3 | 10 ms | 按键扫描、IAP 串口命令轮询 |
| Sensor | 2 | 1000 ms | DHT11 / MQ2 采集（DHT11 手册要求读取间隔 ≥ 1 s） |
| WindSpeed | 2 | 100 ms | 加权融合与风量映射 |
| UI | 1 | 200 ms | SPI LCD 显示 |
| Diag | 1 | 10 s | 运行指标统计 |
| Health | 1 | 周期 | 任务心跳监控（喂狗权唯一） |
| Buzzer | 1 | 事件 | 异步蜂鸣器命令队列 |
| Start | 1 | 一次性 | 初始化后自删除 |

```
 按键 ─▶ KeyScan ──┐
                   ├─▶ StateMgr ──▶ WindSpeed ──▶ Motor ──▶ PWM / H 桥
 DHT11 ─▶ Sensor ──┘  (模式状态机)    (加权融合)    (PID 闭环)
 MQ2 ────┘                                          ▲        │
                                                    │ 编码器  │
 UI ◀── 状态 / 风量显示                               │        │
                              SpeedCalc（TIM2 4 倍频）┴────────┘
```

## 目录结构

```
├── APP_TASK/          应用层任务：业务逻辑、状态机、心跳、故障决策、待机策略
├── BOOT/              Bootloader（裸机程序，不含 FreeRTOS）：协议 / Flash / 元数据 / 升级引擎 / UART DMA / 无栈跳转
├── BSP/               板级驱动：DHT11 / MQ2 / KEY / MOTOR / PID / WIND / LCD / SPI / IAP / LOWPOWER / WATCHDOG ...
├── CONFIG/            板级配置：引脚所有权表、板型 Profile、Flash 分区布局（分区地址唯一权威来源）
├── CORE/              Cortex-M3 内核与启动文件
├── FreeRTOS/          FreeRTOS 内核
├── STM32F10x_FWLib/   ST 标准外设库
├── SYSTEM/            串口 / 延时等系统驱动
├── USER/              Keil 工程与 main
├── mock/              主机侧 FreeRTOS mock 测试套件
├── tools/             尺寸审计 / IAP 打包与发送 / 分区校验等脚本
└── docs/              设计证据与测试日志（调用链、失败矩阵、分区布局、mock/Keil 原始输出）
```

## 快速开始

**环境**：Keil MDK-ARM 5.x（ARMCC V5）+ J-Link

| 工程 | Target | 用途 |
| --- | --- | --- |
| `USER/project.uvprojx` | project | APP 主工程 |
| `USER/boot_audit.uvprojx` | Boot_SizeAudit | Bootloader |
| `USER/app_audit.uvprojx` | App_SlotA/B_SizeAudit | 双槽尺寸审计 |

**硬件**：STM32F103RCT6 最小系统；H 桥电机驱动；TIM2 接正交编码器；DHT11 / MQ2 / 按键 / 蜂鸣器 / SPI LCD。

**IAP 升级**（主机侧）：

```bash
# 打包固件并附 CRC32（--layout 从 CONFIG/memory_layout.h 读取分区地址）
python tools/iap_pack.py --input OBJ_APP/range_hood_app.bin \
    --output OBJ_APP/range_hood_app.iap --layout CONFIG/memory_layout.h

# 串口发送（stop-and-wait，带重传预算）
python tools/iap_uart_sender.py --input OBJ_APP/range_hood_app.iap --port COM3

# 单元测试
python -m unittest tools.test_iap_tools -v
```

**主机侧 mock 测试**：

```powershell
mock/freertos/run-mock-iap-stop.ps1   # IAP / STOP / 低功耗 / 任务心跳等套件
```

## 文档

`docs/design-evidence/` 下为设计与测试证据：

- `TICKLESS_STOP_CALLCHAIN.md` —— 低功耗进入链全流程
- `IAP_FAILURE_MATRIX.md` —— 升级失败矩阵（双页版）
- `IAP_STOP_BASELINE.md` —— 改造前基线冻结
- `MEMORY_LAYOUT.md` / `ISR_TO_TASK.md` / `ENCODER_WRAP_CASE.md` 等
- `logs/` —— Keil 构建与 mock 套件原始输出

## 第三方组件与许可

本仓库包含下列第三方组件。其版权归各自原作者所有，适用其各自的许可条款，**不在本仓库的 MIT 许可范围内**：

| 组件 | 目录 | 版权 | 许可 |
| --- | --- | --- | --- |
| STM32 标准外设库 | `STM32F10x_FWLib/`、`USER/stm32f10x*.h`、`USER/system_stm32f10x.c` 等 | STMicroelectronics © 2011（文件署名 MCD Application Team） | ST 标准外设库许可：允许在 STM32 器件开发中复制、修改、分发。**非 MIT，不可按 MIT 再授权** |
| FreeRTOS 内核 V9.0.0 | `FreeRTOS/` | Real Time Engineers Ltd. © 2016 | **GPL v2 + FreeRTOS exception**。FreeRTOS 内核自 V10.0.0 起才改为 MIT；本仓库用的是 V9.0.0 |
| CMSIS-Core | `CORE/` | ARM Limited © 2009（All rights reserved） | ARM CMSIS 许可（Apache-2.0 的前身） |
| LCD 点阵字库与驱动 | `BSP/LCD/`（`FONT.H`、`lcd.c/.h`、`GUI.c/.h`） | 第三方来源，本仓库仅作芯片平台移植与引脚适配 | **未随仓库提供原始许可文本**；引用/二次分发前需自行确认授权范围 |

除上述第三方组件外，`APP_TASK/`、`BOOT/`、`BSP/`（不含 `BSP/LCD/`）、`CONFIG/`、`tools/`、`mock/`、`docs/` 及 `USER/main.c` 由 丁鲁 编写，采用 MIT 许可。

> GPL v2 要求随分发附带许可证正文：`FreeRTOS/License/license.txt` 已包含 FreeRTOS GPL Exception 与 GPL v2 全文（取自 FSF 官方 GPL v2 正文）。

## License

自写代码采用 MIT，见 [LICENSE](LICENSE)。第三方组件适用其各自许可，见上表。
