# 最终 Flash 分区布局与容量预算（RCT6 / FINAL 冻结）

日期：2026-09-06（布局冻结）
依据：`CONFIG/memory_layout.h` 与 `tools/check_memory_layout.py` 的实测校验结论
构建环境：Keil MDK-ARM 5.39 / ARMCC V5.06 update 6 (build 750)

***

## 1. 本文档的作用与证据边界

* 记录**最终冻结**（FINAL）的 CB 分区布局推导、构建证据与验收结果；
  `LAYOUT_PROVISIONAL=0`，布局不再随功能增长漂移；

* **C8→CB 迁移是计划内决策**：Task 15 全功能 APP 链接需求 54964 B，
  C8（64 KiB）任意可行 Boot 尺寸下最大 APP_CAPACITY=55296 B，扣除
  2 KiB 硬余量后（54964+2048=57012 > 55296）放不下；按，生成 CB 布局"分支迁移 CB（128 KiB），**没有**删除
  错误处理、**没有**缩减硬余量；

* 本文档数值的**运行验证级别为 L2（编译/链接/静态校验）**：所有尺寸、
  对齐、向量、区间不重叠结论来自 map/bin 与 `check_memory_layout.py --final`
  交叉校验；板上烧写与升级演练属 L3，未在本任务范围内。

## 2. 最终布局常量（CONFIG/memory\_layout.h 为唯一权威来源）

| 常量                   | 值                  | 推导/说明                                                    |
| -------------------- | ------------------ | -------------------------------------------------------- |
| `FLASH_BASE`         | `0x08000000`       | 芯片事实                                                     |
| `FLASH_END_EXCL`     | `0x08020000`       | **CB = 128 KiB**（C8 迁移点）                              |
| `FLASH_PAGE_SIZE`    | `0x400`            | 1 KiB 页擦除粒度                                              |
| `META_A_BASE`        | `0x0801F800`       | page 126，双页事务元数据 A                                    |
| `META_B_BASE`        | `0x0801FC00`       | page 127，双页事务元数据 B（Flash 末页）                      |
| `APP_LIMIT_EXCL`     | `0x0801F800`       | APP 上界 = META\_A\_BASE                                   |
| `BOOT_PART_SIZE`     | `0x3800`（14336 B） | `align_up(B_final + 2048, 1024)`，见 §3                    |
| `APP_BASE`           | `0x08003800`       | `FLASH_BASE + BOOT_PART_SIZE`；VTOR 偏移 0x3800 是 0x200 的倍数 |
| `APP_CAPACITY`       | `0x1C000`（114688 B） | `APP_LIMIT_EXCL − APP_BASE`                              |
| `LAYOUT_PROVISIONAL` | `0`                | **已冻结**（Task 16）                                       |

## 3. BOOT\_PART\_SIZE 最终推导

```
B_final            = 11468 B（Task 16 FINAL_AUDIT Boot bin：Task 8 的 10916 B
                               + Task 9 镜像检查/无栈跳转等增量）
BOOT_PART_SIZE     = align_up(11468 + 2048, 1024) = align_up(13516, 1024)
                    = 14336 B = 0x3800
APP_BASE           = 0x08000000 + 0x3800 = 0x08003800
APP_CAPACITY       = 0x0801F800 - 0x08003800 = 0x1C000 = 114688 B
```

2 KiB 硬余量按计划保留在 Boot 分区尾部与 APP 容量校验两侧，未为容纳
APP 而压缩。

## 4. 冻结构建证据（全部 0 Error / 0 Warning）

工程与 Target（`tools/build-iap-stop.ps1` 全流水线，
`logs/iap-stop-build.log`）：

| Target（工程文件）                                                | IROM                 | IRAM                | 关键宏                          |
| ----------------------------------------------------------- | -------------------- | ------------------ | ---------------------------- |
| `Boot_SizeAudit`（`USER/boot_audit.uvprojx`）                | `0x08000000/0x3800`  | `0x20000000/0x5000` | —                            |
| `App_SizeAudit`（`USER/app_audit.uvprojx`）                  | `0x08003800/0x1C000` | `0x20000000/0x5000` | `VECT_TAB_OFFSET=0x3800`     |
| `Boot_CB_Provisional`（`USER/boot_cb_provisional.uvprojx`）  | `0x08000000/0x3800`  | `0x20000000/0x5000` | —                            |
| `App_CB_Provisional`（`USER/app_cb_provisional.uvprojx`）    | `0x08003800/0x1C000` | `0x20000000/0x5000` | `VECT_TAB_OFFSET=0x3800`     |
| `project`（`USER/project.uvprojx`，日常开发）                 | CB 全域              | `0x20000000/0x5000` | —                            |

审计与最终布局 Target 的关键尺寸（FINAL\_AUDIT 与 PROV\_PASS1/PASS2
三次一致）：

| 指标                    | Boot                     | App                            | 预算                                    | 结论      |
| --------------------- | ------------------------ | ------------------------------ | ------------------------------------- | ------- |
| LR\_IROM1 Base        | `0x08000000`             | `0x08003800`                   | —                                     | 与分区基址一致 |
| LR\_IROM1 Size        | 11468 B                  | 54964 B                        | Boot ≤ 14336 B / APP ≤ 114688 B       | PASS    |
| bin 长度（fromelf --bin） | 11468 B                  | 54620 B                        | 同上                                    | PASS    |
| Total RW              | 3456 B                   | 13688 B                        | Boot ≤ 4096 B / APP ≤ 18432 B（设计 §4） | PASS    |
| map/bin 同源            | bin == Total ROM Size    | bin == Total ROM Size          | —                                     | PASS    |

注 1（RW 压缩与同源判据）：App LR Size（54964 B）大于 bin（54620 B），
是 armlink 对 RW 加载副本压缩（map 标记 COMPRESSED）所致。同源判据按
"bin 实长 == map Total ROM Size（Code+RO+RW）且 ≤ LR Size"执行
（`tools/iap_stop_gates.py` audit-gates），不把压缩镜像误判为不同构建。

注 2（布局收敛）：PROV\_PASS1 与 PROV\_PASS2 两遍全量重建的
`LAYOUT_COMPUTED boot_part=0x3800 app_base=0x08003800
app_capacity=0x1C000 boot_len=11468 app_len=54620` **逐字相同**，
满足计划"连续两次完全相同"的收敛门槛。

## 5. 容量与硬余量（最终）

```
A_final = 54620 B <= APP_CAPACITY - 2048 = 112640 B（余量 58020 B）
B_final = 11468 B <= BOOT_PART_SIZE - 2048 = 12288 B（余量 820 B，分区总空隙 2868 B）
```

## 6. 静态验收（check\_memory\_layout.py --final，`logs/iap-stop-layout.log`）

* Boot 最高 load address `0x08002CCC < APP_BASE` → PASS；
* APP 区间 `[0x08003800, 0x08010D5C) < META_A_BASE` → PASS；
* boot/app/meta 三段 Flash 区间互不重叠，metadata 两页不在任一 bin → PASS；
* 向量表：boot MSP `0x20000D80`（SRAM 内）/ PC `0x08000189`（Thumb，
  Boot 区间内）；app MSP `0x20003578` / PC `0x08003A91`（APP 区间内）→ PASS；
* Boot/App Total RW 各 3456/13688 B ≤ 4096/18432 B（20 KiB SRAM 预算）→ PASS；
* 所有分区页/半字/VTOR 对齐，两个 bin 半字对齐（11468/54620 B 均为偶数）→ PASS；
* `LAYOUT_PROVISIONAL == 0` → PASS。

## 7. .iap 升级包验证（Task 16 终点）

```
APP_BASE      = 0x08003800
image_size    = 54620 B        image_crc32 = 0xB907C17E
image_version = 0x00000100
frames        = 109（START 1 + DATA 107 + END 1，512 B 分片）
output        = OBJ_APP_PROV/range_hood_app.iap（57252 B）
round-trip    : Python 解包重组 54620 B 与原 bin 字节级一致 → PASS
```

复现：`python tools/iap_pack.py --input OBJ_APP_PROV\range_hood_app.bin
--output OBJ_APP_PROV\range_hood_app.iap --layout CONFIG\memory_layout.h`
+ `python tools/iap_stop_gates.py roundtrip ...`（或直接跑
`tools/build-iap-stop.ps1` Stage 8）。

## 8. 一致性校验工具与复现方式

`tools/check_memory_layout.py`（77 个单元测试全过）+
`tools/iap_stop_gates.py`（audit-gates / roundtrip / stale-scan，
26 个单元测试全过）。

复现（仓库根目录，一次覆盖 Stage 0–8）：

```powershell
powershell -ExecutionPolicy Bypass -File tools\build-iap-stop.ps1
```

或手工单跑布局终检：

```powershell
& D:\Keilv5\UV4\UV4.exe -b USER\boot_cb_provisional.uvprojx -t Boot_CB_Provisional -j0
& D:\Keilv5\UV4\UV4.exe -b USER\app_cb_provisional.uvprojx  -t App_CB_Provisional  -j0
python tools/check_memory_layout.py --final `
  --layout CONFIG/memory_layout.h `
  --boot-proj USER/boot_cb_provisional.uvprojx `
  --boot-map OBJ_BOOT_PROV/boot.map --boot-bin OBJ_BOOT_PROV/boot.bin `
  --app-proj USER/app_cb_provisional.uvprojx `
  --app-map OBJ_APP_PROV/range_hood_app.map `
  --app-bin OBJ_APP_PROV/range_hood_app.bin `
  --app-audit-bin OBJ_APP_AUDIT/range_hood_app.bin
```

实测输出（2026-09-06 Task 16 冻结，overall PASS，退出码 0）：

```text
layout: FINAL
flash_end_exclusive: 0x08020000
layout_provisional: 0
final gate: LAYOUT_PROVISIONAL == 0 -> PASS
meta_a/meta_b: page aligned, non-overlapping
current capacity warning: PASS (app audit bin 54620 B <= APP_CAPACITY 114688 B)
boot target 'Boot_CB_Provisional': IROM 0x08000000/0x3800, IRAM 0x20000000/0x5000, VECT_TAB_OFFSET=none -> PASS
app target 'App_CB_Provisional': IROM 0x08003800/0x1C000, IRAM 0x20000000/0x5000, VECT_TAB_OFFSET=0x3800 -> PASS
boot map: LR_IROM1 base 0x08000000 size 11468 B, Total RW 3456 B <= 4096 B -> PASS
app map: LR_IROM1 base 0x08003800 size 54964 B, Total RW 13688 B <= 18432 B -> PASS
boot bin 11468 B <= boot capacity 14336 B -> PASS
app bin 54620 B <= app capacity 114688 B -> PASS
boot vector: MSP 0x20000D80 in SRAM, PC 0x08000189 thumb in [0x08000000,0x08003800) -> PASS
app vector: MSP 0x20003578 in SRAM, PC 0x08003A91 thumb in [0x08003800,0x0801F800) -> PASS
flash intervals: boot [0x08000000,0x08002CCC), app [0x08003800,0x08010D5C), meta [0x0801F800,0x08020000) disjoint -> PASS
boot bin halfword aligned (11468 B) -> PASS
app bin halfword aligned (54620 B) -> PASS
tight window (hard margin 2048 B): boot 12 KiB -> APP_CAPACITY 116736 B, A_max 114688 B; boot 16 KiB -> APP_CAPACITY 112640 B, A_max 110592 B
overall: PASS
```

## 9. 烧写与使用边界（冻结后）

* Keil Flash Download 有效范围为 **CB 官方 `0x08000000/0x20000`**
（`STM32F10x_128.FLM -FS08000000 -FL020000`），不得向 CB 官方边界
  之外写入；C8 时期"范围覆盖 128 KiB 但板子只有 64 KiB"的风险随
  板型迁移一并消除；

* 日常开发仍用主工程 `USER/project.uvprojx`；`*_cb_provisional`
  与 `*_audit` 工程是布局/尺寸证据工程，产物不用于烧写业务固件；

* 旧绝对地址（`0x0800F000/0x0800EC00/0x20004000` 等 C8 时期散落值）
  已由 `iap_stop_gates.py stale-scan`（98 个源文件）确认清零，
  布局事实只存在于 `memory_layout.h`。

***

## 历史注（C8 时期中间态，git 历史口径）

* **Task 3**（2026-09-04）：临时布局 BOOT\_PART=0x1C00（7 KiB）/
  APP\_BASE=0x08001C00/CAPACITY=0xDC00，`LAYOUT_PROVISIONAL=1`；
* **Task 5**（2026-09-04）：退役旧 `iap.c`/`iap.h` 后 App LR
  48052→46416 B（−1636 B）；
* **Task 8**（2026-09-05）：全功能 Boot（协议+Flash+元数据+UART DMA）
  10916 B，Boot 分区 7 KiB→12 KiB（0x3000）复调，APP\_BASE=0x08003000/
  CAPACITY=0xC800；同时清理 APP 侧旧 IAP/DMA/`stmflash.c` 死代码，
  App bin 46112 B；
* **Task 15**（2026-09-06）：Tickless/STOP 接线后 App LR 54964 B，
  12 KiB Boot 临时布局下 L6406E 链接失败（54964 > 51200），且 C8 任意
  可行布局最大 APP\_CAPACITY=55296 B 也放不下（+2 KiB 硬余量超限），
  触发计划内 C8→CB 迁移决策（证据 `TICKLESS_STOP_CALLCHAIN.md` §6）。
