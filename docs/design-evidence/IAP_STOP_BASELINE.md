# IAP + STOP 低功耗改造：改造前基线冻结

日期：2026-09-04
依据：本仓库 `CONFIG/memory_layout.h` 与 `CONFIG/board_profile.h` 的最终布局
证据源：Keil MDK-ARM 5.39 / ARMCC V5.06 update 6 (build 750) 单 Target `project`，0 Error / 0 Warning

---

## 1. 本文档的作用

- 冻结**改造前**的单体镜像尺寸证据，供后续 Task 2/3/16 计算 Boot/APP 拆分增量对比；
- 冻结**板级硬件事实**（C8 边界、无表晶、引脚所有权），后续所有 Task 不得与本文冲突；
- **明确边界**：本文中的尺寸是"Boot/APP 尚未拆分"的单体数据，只用于比较增量，
  **不得**用于冻结最终分区（最终分区以 Task 16 `FINAL_AUDIT` 的独立 Boot/App map/bin 为准）。

## 2. C8 官方边界（芯片事实）

| 项 | 值 |
| --- | --- |
| Flash 区间 | `[0x08000000, 0x08010000)` = 64 KiB |
| SRAM 区间 | `[0x20000000, 0x20005000)` = 20 KiB |
| Flash 页大小 | 0x400（1 KiB，页擦除粒度） |
| Flash 编程粒度 | 半字（16 bit） |

芯片事实常量集中在 `CONFIG/board_profile.h`；分区地址常量（Boot/App/Meta 边界）
归 `CONFIG/memory_layout.h`（Task 3 建立），本文不定义任何地址宏。

## 3. 改造前单体镜像证据（Keil map 导出）

| 指标 | 值 | 十进制 |
| --- | --- | --- |
| Load Region LR_IROM1 Base | `0x08000000` | — |
| Load Region LR_IROM1 Size | `0xBBFC` | 48060 B |
| Total RO（Code+RO Data） | — | 47764 B |
| Total RW（RW Data+ZI Data） | — | 13496 B |
| Total ROM（Code+RO+RW，链接器压缩后） | — | 47832 B |

注 1：本表数值于 2026-09-04 改动**前**从 Keil map 导出捕获；map 文件
会被后续构建覆盖，本文即为其唯一存档。
注 2：计划头部引用的"48124 B"与本 map 实测不一致（LR 实测 48060 B / ROM 47832 B），
后续一律以 map 实测为准。

### Task 1 改动后的复测增量（供增量对比链路校验）

改动后全量 Rebuild（`logs/task1-keil-rebuild.log`，0 Error / 0 Warning）：

| 指标 | 改动前 | 改动后 | 增量 |
| --- | --- | --- | --- |
| LR_IROM1 Size | 48060 B（0xBBFC） | 48052 B（0xBBB4） | −8 B |
| Total ROM（压缩后） | 47832 B | 47760 B | −72 B |
| Total RW | 13496 B | 13496 B | 0 |

增量来源：KEY2 初始化/扫描/读脚代码被 `KEY2_ENABLED=0` 编译剔除；引脚宏替换
本身不产生代码。

## 4. 板级硬件事实（用户板端确认）

| 事实 | 结论 | 影响 |
| --- | --- | --- |
| 板载无 32.768k 表晶 | 已确认（2026-09-04） | LSE/RTC 路线废弃；PC14/PC15 不保留给 LSE；STOP 深睡只走按键 EXTI 事件唤醒，tick 冻结不补偿 |
| PC14 | 归 DHT11 唯一使用（app_tasks.c SensorTask） | 不触碰 backup domain，不用 EXTI17/RTC Alarm |
| PB12 | 双重所有权已修复（见 §5） | KEY2 编译开关默认禁用 |

## 5. 引脚所有权表（CONFIG/board_pins.h 为唯一权威来源）

| 引脚 | Owner | 宏 | 备注 |
| --- | --- | --- | --- |
| PB1 | KEY1 按键（模式切换） | `KEY1_GPIO_PORT/PIN` | 兼作 STOP 深睡 EXTI 唤醒源 |
| PB12 | TIM1_BKIN 刹车输入（默认预留） | `MOTOR_BRAKE_GPIO_PORT/PIN` | 厂商示例为 `#if 0` 死代码，未启用 |
| PB12 | KEY2 按键（编译开关，**默认禁用**） | `KEY2_GPIO_PORT/PIN` | `KEY2_ENABLED=0`；原理图确认后置 1 或迁移引脚 |
| PA8 | 电机 PWM（TIM1_CH1） | `MOTOR_PWM_GPIO_PORT/PIN` | |
| PB13 | 电机互补输出（TIM1_CH1N） | `MOTOR_CH1N_GPIO_PORT/PIN` | |
| PA2 | H 桥使能 | `MOTOR_EN_GPIO_PORT/PIN` | |
| PA0/PA1 | 编码器 A/B 相（TIM2_CH1/CH2） | `MOTOR_ENC_A/B_GPIO_PORT/PIN` | |
| PC14 | DHT11 数据线 | `DHT11_GPIO_PORT/PIN` | 板载无表晶，不与 LSE 冲突 |

### PB12 双重所有权修复记录（本 Task 的 Gate）

- **缺陷**：key.h:25 声明 `KEY2=PB12`（运行时初始化为上拉输入），motor.c 厂商刹车
  示例（`#if 0` 死代码）把 PB12 初始化为 TIM1_BKIN 刹车输入——同一引脚两套语义。
- **修复**：`CONFIG/board_pins.h` 声明 PB12 默认 owner 为刹车输入预留；KEY2 退为编译
  开关 `KEY2_ENABLED`（默认 0，key.c 中初始化/扫描/读脚全部编译剔除，
  `Key2_GetEvent()` 恒返回 `KEY_EVENT_NONE`，应用层无感）；编译期冲突检查覆盖
  PORTA/PORTB 组内互异与 DHT11 跨组检查。
- **后续**：待原理图确认接线后，置 `KEY2_ENABLED=1`（PB12 归按键，刹车路径保持
  禁用）或迁移 KEY2 到空闲引脚；两者永远互斥。

## 6. 审计记录（Task 1 步骤）

```text
$ rg -n "GPIO_Pin_14|KEY2_GPIO_PIN|GPIO_Pin_12" BSP APP_TASK CONFIG
BSP\KEY\key.h:31:  #define KEY2_READ()  GPIO_ReadInputDataBit(KEY2_GPIO_PORT, KEY2_GPIO_PIN)
BSP\KEY\key.c:49:  io_set(KEY2_GPIO_PORT,KEY2_GPIO_PIN,GPIO_Mode_IPU);   # KEY2_ENABLED 门控内
CONFIG\board_pins.h:48:  #define KEY2_GPIO_PIN  BOARD_PIN_MASK(12)
```

结论：`GPIO_Pin_14`/`GPIO_Pin_12` 字面量零残留；`KEY2_GPIO_PIN` 仅存于统一配置
及其引用处，无绕过 `board_pins.h` 的旧引用。

## 7. 验证等级（本 Task）

| 项 | 等级 |
| --- | --- |
| board_profile.h / board_pins.h / KEY2 编译开关 / 引脚集中化 | `SOURCE_IMPLEMENTED` + `BUILD_VERIFIED`（Keil 0 Error 0 Warning，见 Task 1 提交） |
| PB12 双重所有权修复 | `SOURCE_IMPLEMENTED` + `BUILD_VERIFIED` + `STATIC_CHECKED`（编译期 #if 冲突检查） |
| 引脚实际电平/按键功能/DHT11 采集 | `NOT_BOARD_TESTED`（全项目声明边界，见 CLAIM_MATRIX.md） |

## 8. 对后续 Task 的约束传递

1.；PB12 所有权未确认前
   不得引用 KEY2 wake 配置；
2.，任何源码不得写死分区地址；
3. 本基线的 48060 B LR 是"拆分前"数据：Boot 裸机 + APP FreeRTOS 拆分后，
   APP 不再包含 Boot 侧代码，两独立镜像之和才是最终容量判断依据（Task 16）。
