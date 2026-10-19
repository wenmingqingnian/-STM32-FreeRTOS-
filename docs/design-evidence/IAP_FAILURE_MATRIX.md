# IAP 升级失败矩阵：双页元数据事务、commit 提交点与 39 点断电注入（双页版）

> 证据等级：L1（源码审计）+ L2（Keil 四工程 0 错误 0 警告）+ L2（主机 mock：
> meta 172/172、boot-image 31/31、boot-protocol 159/159、boot-flash 135/135、
> crc32-stream 85/85，`logs/iap-stop-*.log`）。
> 本文件为**双页版**。
> 旧单页设计（0x1111/0x2222 状态字、T0-T5 时刻矩阵、iap.c/iap.h）已整体
> 退役删除，旧版文档仅存 git 历史，不得再被任何声明引用。
> 无 L3 板端断电数据：39 点注入为 fake_flash（1->0 物理模型）模拟结论。

## 1. 分区布局与记录格式（CB 128KB 冻结布局）

| 区域 | 地址 | 大小 | 说明 |
|------|------|------|------|
| Bootloader | 0x08000000-0x080037FF | 14336 B（BOOT_PART_SIZE=0x3800） | 升级决策+校验+无栈跳转 |
| APP 分区 | 0x08003800-0x0801F7FF | 114688 B（APP_CAPACITY=0x1C000） | 单分区固件（无 A/B 槽） |
| 元数据页 A | 0x0801F800 | 1 KB，页首 1 条记录 | 双页交替写入 |
| 元数据页 B | 0x0801FC00 | 1 KB，页首 1 条记录 | 双页交替写入 |

记录布局（`BSP/IAP/iap_meta.h:37-52`，显式小端 36 B = 18 半字，禁止结构体 memcpy）：

```
off  size  field            约束
 0    4    magic            0x31415049（"IPA1" 小端）
 4    2    format_version   0x0001
 6    2    header_size      36
 8    4    generation       无符号环绕递增（见 §3）
12    2    state            RECEIVING=0x5AA5 / VALID=0x3CC3
14    4    app_base         必须等于 APP_BASE
18    4    image_size       8..APP_CAPACITY
22    4    image_crc32       整镜像 CRC32
26    4    image_version    仅记录，不作防回滚门禁
30    4    record_crc32     覆盖 [0,30) 前缀（本字段置零后计算）
34    2    commit_marker    0x33CC，整条记录最后一次半字编程
```

状态字设计（iap_meta.h:60-63）：`ERASED=0xFFFF` / `RECEIVING=0x5AA5` / `VALID=0x3CC3`。
**两状态互不位包含**——0x5AA5 -> 0x3CC3 无法用 Flash 1->0 特性原地改写，
这是刻意设计：状态跃迁必须走"另一页写新记录"，强制双页交替，杜绝
"原地改状态字"带来的半提交窗口。

## 2. 升级事务时序（declare-before-erase，双页交替）

会话由分帧协议驱动（boot_update.c，事件 START/DATA/END）：

```
START（bu_start，boot_update.c:255）
  ① 工厂容量门禁（低于布局要求拒绝一切擦写，防克隆片）
  ② 最新页判定（IAP_MetaSelectLatest）
  ③ RECEIVING（gen = 最新+1）写到【非最新页】
     擦页 -> 17 个字段半字 -> commit 半字最后编程
  ④ 擦 APP 分区 -> ACK
DATA
  ⑤ 流式半字写 APP（越界检查"先减后比"防回绕；奇数长度补 0xFF，
     CRC 只算真实字节）
END（bu_end，boot_update.c:347）
  ⑥ 整镜像 CRC 从 Flash 重算（不信任 RAM）
  ⑦ VALID（gen = RECEIVING+1）写到【RECEIVING 之外的另一页】
     ——即旧 VALID 所在页，本事务覆盖，双页交替
  ⑧ ACK 发送完成（等 TC）-> 复位会话
```

**提交点语义**：一条记录"可解码"蕴含 commit 已编程；commit 之前所有
34 字节已全部落盘。任何断电只要发生在 commit 编程之前，该记录解码
即为无效（magic/版本/record_crc/commit 任一不满足）——Flash 半字
编程的物理时序实现了逻辑原子提交，无需页擦除参与提交动作
（这是与旧单页"擦除+重写整页提交"的本质区别）。

**一旦 RECEIVING 已提交，不再退回**（设计 §5.1）：此后任何断电，
最新记录均为 RECEIVING，Boot 拒绝跳转，停留在升级模式等待重传。
## 3. 断电注入矩阵（39 点，19/19/1 分布）

完整升级会话的元数据 Flash 操作 = 2 页擦除 + 36 半字编程
（RECEIVING 18 半字 + VALID 18 半字）= 38 个操作。fake_flash 在
**每个操作前**注入一次断电（38 点）+ 终态验证（1 点）= 39 点
（`logs/iap-stop-meta-mock.log`：`injection points=39, continue-old=19,
reject=19, continue-new=1`）。

| 断电位置 | 操作数 | 落盘最新记录 | 重启决策 | 归类 |
|----------|--------|--------------|----------|------|
| RECEIVING 页：擦页/17 字段半字/commit 半字编程中 | 19 点 | 旧 VALID（完好未动） | CONTINUE 旧 APP | continue-old=19 |
| RECEIVING 已提交后；VALID 页：擦页/17 字段半字/commit 半字编程中 | 19 点 | RECEIVING | REJECT 停留 Boot | reject=19 |
| 终态（VALID 已提交） | 1 点 | 新 VALID | CONTINUE 新 APP | continue-new=1 |

每点断电后重启，断言启动决策属于
{继续旧固件, 拒绝, 继续新固件} 且**绝不启动部分写入的记录**。

**部分写入记录不可能被启动的三重防线**：

1. commit 未编程 -> 记录解码 invalid（commit_marker != 0x33CC）；
2. record_crc32 覆盖 [0,30) -> 字段损坏 -> 解码 invalid；
3. 即使记录完好但 state=RECEIVING -> IAP_BootDecision 一律 REJECT。

**generation 无符号环绕比较**（iap_meta.h:136-140）：diff = a-b，
diff < 0x80000000 -> a 更新；diff > 0x80000000 -> b 更新；
**恰等于 0x80000000 -> 歧义 -> 保守拒绝（AMBIGUOUS，停 Boot）**。
环绕序列 0xFFFFFFFE->0xFFFFFFFF->0->1 严格递增由测试证明
（meta-generation-wrap 套件 13 检查，含歧义双向）。

**连续两次升级**（meta-double-update 套件）：update 1 完成 -> CONTINUE
（gen7）；update 2 进行中（RECEIVING gen8 最新）-> REJECT；update 2
完成 -> CONTINUE（gen9）。双页交替跨会话保持"最新者胜"。

## 4. Boot 复位决策优先级与握手竞态（boot_main.c，设计 §5.1）

| 序 | 输入 | 行为 | 依据 |
|----|------|------|------|
| 1 | 恢复键（KEY1/PB1）复位时保持按下 | 强制升级模式，**无握手超时回退** | 用户物理动作不可能"陈旧"，可信度高于任何软件状态 |
| 2 | BKP_DR2 升级请求（APP 侧写入） | 进入升级模式；**握手超时不直接跳 APP，而是重读双页元数据** | 超时瞬间升级可能恰好已开写：RECEIVING"开始"记录可能已提交，直接跳=跳进半擦写镜像。一次重读消除竞态窗口 |
| 3 | 双页元数据决策 | CONTINUE -> 校验链+跳转；REJECT（RECEIVING/无记录/歧义/字段非法）-> 停留 Boot | §3 三重防线 |

BKP 请求握手的超时重读是本设计里最容易被追问的竞态点：
"超时"与"对侧开写"是异步事件，读一次元数据把两个事件合并成
一个一致视图。

## 5. 镜像校验链与七步跳转（Task 9）

校验链顺序（`BOOT/boot_image.c`）：**state -> size -> base -> MSP ->
PC -> CRC**，先便宜后贵——状态解码是几次比较，CRC 遍历整镜像
（54620 B）放最后，前序任何一道拦下的损坏镜像都省掉整镜像 CRC。
测试用"前一道通过、后一道拦截"逐对用例证明顺序真实生效
（boot-image 31 检查）。

七步跳转（`Boot_JumpToApp`，boot_main.c:85；无栈尾跳 boot_jump.s）：

| 步 | 操作 | 若省略的后果 |
|----|------|--------------|
| 1 | 镜像头防御性复检（MSP 在 SRAM、复位向量在 APP 分区内） | 跳入垃圾镜像（纵深防御） |
| 2 | `__disable_irq()` 关全局中断 | 跳转过程被中断打断，上下文混乱 |
| 3 | 停 SysTick（CTRL/LOAD/VAL 清零） | APP 未重设 VTOR 前发生 SysTick 异常 -> 按 Boot 向量表跳入已失效处理函数 |
| 4 | 失能 Boot 外设（DMA/USART）+ 清标志 + 关 NVIC 通道 | APP 初始化同外设时残留中断互相干扰 |
| 5 | 清 PendSV/SysTick 挂起标志（SCB->ICSR） | APP 开中断后补触发 -> 跳回 Boot 的 PendSV_Handler，破坏 APP 栈 |
| 6 | 重定位 VTOR 到 APP 基地址 | APP 中断按 Boot 向量表分发 |
| 7 | 取复位向量 -> 设 MSP -> 清 BASEPRI/FAULTMASK -> 尾跳 | APP 复位流程不清屏蔽位，Boot 必须代为恢复 |

顺序不可颠倒：步骤 2 必须先于 3-5（清理过程不可被打断）；步骤 5
必须在开中断前（挂起标志补触发）。校验放在 Boot_JumpToApp 而非
main 决策处：跳转前一刻重读元数据，与决策点之间不存在擦写，双保险。
## 6. 单分区固件边界（诚实边界，需主动说明）

1. **无固件级回滚**：RECEIVING 提交后旧镜像在流式写入中被破坏，
   断电只能"重新升级"恢复，不存在回退旧版本路径。双页设计改善的是
   **元数据一致性窗口**，不是固件回滚：RECEIVING 落盘**前**断电，
   旧 VALID 记录完好 -> 仍可启动旧 APP（continue-old 19 点的由来）；
   RECEIVING 落盘**后**无路可退（reject 19 点）。A/B 双槽方案可回滚，
   代价是 128 KB Flash 装不下两份 54620 B 镜像——CB 容量下的物理不可行。
2. **升级信道**：分帧协议 seq/offset 双字段校验，跳号 NACK 带期望
   seq/offset，重复帧幂等重 ACK 不二次编程；任意分片喂入解码与整帧
   一致（159 检查）。协议层具备按期望 offset 续传的能力；上层工具
   的断点续传策略不属于本矩阵声明范围。
3. **页粒度隔离**：元数据页在 Flash 末端（0x0801F800/0x0801FC00），
   擦 APP 分区的页擦除物理上不可能误擦元数据——状态字可信的前提。
4. **无防回滚版本门禁**：image_version 仅记录不校验。受控串口线
   威胁模型下回滚攻击不在模型内；若需 anti-rollback 要加单调版本
   比较并处理 generation 语义冲突（声明为未做的安全特性）。

## 7. 验证证据

- 主机 mock（`logs/iap-stop-meta-mock.log`，172/172）：10 套件——
  encode-layout（字节级布局）/ field-corruption / generation-wrap（含
  歧义双向）/ select-latest / boot-decision / fake-flash-physics
（1->0、已编程拒绝、非对齐拒绝、越界拒绝）/ write-record /
  write-power-cut / **power-cut-injection（39 点 19/19/1）** /
  double-update（gen7 -> RECEIVING8 REJECT -> gen9）；
- 链路配套 mock：boot-protocol 159、boot-flash 135、boot-image 31、
  crc32-stream 85（`logs/iap-stop-{protocol,flash,image,crc32}-mock.log`）；
- Keil 四工程（Boot/APP/审计/provisional）0 错误 0 警告，布局三次
  收敛逐字一致（`logs/iap-stop-build.log` Stage 2-6）；
- L3 断电注入未执行：需实机+可编程电源。fake_flash 以 1->0 半字
  物理模型近似真实 Flash 编程时序；状态机与提交点逻辑已被 39 点
  穷举覆盖，Flash 电气边界依赖器件手册保证。

## 8. 一句话总结

"升级事务是双页元数据的两段声明：先在对侧页提交 RECEIVING（declare
-before-erase，此后断电一律停 Boot），镜像从 Flash 重算 CRC 通过后，
在另一页提交 VALID（generation+1，commit 半字最后编程实现原子提交）；
39 个断电注入点验证三重防线——commit 未写、record_crc 损坏、RECEIVING
状态——任何一条都拦住部分写入的记录；单分区的代价是没有固件级回滚，
双页换来的是'开始记录落盘前旧系统始终可启动'的一致性窗口。"
