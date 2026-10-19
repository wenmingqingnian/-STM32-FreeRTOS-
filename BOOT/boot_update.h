/*
 * Boot 升级引擎
 *
 * 职责——把 Task 4/5/6/7 的纯逻辑模块接线成完整的升级会话：
 *   1. 目标侧 Flash IO 后端（BOOTFLASH 与 IAP_Meta 的 SPL 适配：解锁/
 *      擦页/半字编程/读半字；擦页内安全喂 IWDG）；
 *   2. 双页元数据读取 + 启动决策（Boot_Update_MetaDecision，供 main 与
 *      Task 9 跳转链复用）；
 *   3. 升级会话引擎：DMA 环消费 -> 解析器 -> 会话事件 -> Flash 写入 ->
 *      ACK/NACK 回复；START 时先提交 RECEIVING 再擦【目标槽】
 *（declare-before-erase），END 后整镜像 CRC 通过才提交 VALID，
 *      ACK 后系统复位；
 *
 * 双槽（A/B）要点：
 *   - 目标槽 = 当前活动槽（最新 VALID 记录指向的槽）的另一槽，
 *     由 IAP_SlotOther 计算；首次烧录无活动槽时默认 Slot A；
 *   - 元数据 slot_id/app_base 全程携带目标槽身份，Boot 决策按槽校验；
 *   - 升级过程中旧活动槽始终保持可回退，直至新槽 VALID 提交+复位。
 *   4. 超时治理：握手超时（仅 BKP 请求入口可回退旧 APP）与帧间超时
 *（只丢半截帧，会话状态保持，主机整帧重发）。
 *
 * Boot->主机 ACK/NACK 帧编码（与主机方向帧同构，24 B 头，小端）：
 *   ACK ：type=0x04，sequence/image_offset = 被确认帧的回显，payload_len=0；
 *   NACK：type=0x05，sequence/image_offset = 期望主机重发的 sequence/offset
 *（会话级失败后二者为 0：主机应整会话重启、重新发 START），
 *         payload 4 B = nack_reason（u32 小端，编码见 BOOT_UPDATE_NACK_*）。
 *   两者的 frame_crc32 均覆盖 [4,20) + payload（Task 10 的
 *   iap_uart_sender.py 必须按本编码实现）。
 *
 * 主机端 NACK 处理约定：
 *   - reason < 0x40（BootProto_NackReason_t）：按头字段期望值重发对应帧；
 *   - reason >= 0x40（本文件 BOOT_UPDATE_NACK_*）：Boot 已复位会话，
 *     主机应放弃当前镜像进度，从 START 重新开始整个升级。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#ifndef __BOOT_UPDATE_H__
#define __BOOT_UPDATE_H__

#include "sys.h"
#include "iap_meta.h"

/*-----------------------------------------------------------
 * Boot 侧 NACK 扩展原因码（>=0x40；协议级原因见 BootProto_NackReason_t）
 *----------------------------------------------------------*/
#define BOOT_UPDATE_NACK_FLASH_SIZE   0x40u  /* 工厂容量寄存器低于布局要求 */
#define BOOT_UPDATE_NACK_META_WRITE   0x41u  /* RECEIVING/VALID 元数据事务失败 */
#define BOOT_UPDATE_NACK_ERASE        0x42u  /* APP 分区页擦除失败 */
#define BOOT_UPDATE_NACK_PROGRAM      0x43u  /* DATA 半字编程失败 */
#define BOOT_UPDATE_NACK_READBACK     0x44u  /* DATA 包级回读不一致 */
#define BOOT_UPDATE_NACK_IMAGE_CRC    0x45u  /* END 后整镜像 CRC 不一致 */
/* 帧级解析错误（onError 路径）映射 */
#define BOOT_UPDATE_NACK_FRAME_VERSION 0x50u /* protocol_version 非法 */
#define BOOT_UPDATE_NACK_FRAME_FORMAT  0x51u /* header_len/reserved/type/payload_len 非法 */
#define BOOT_UPDATE_NACK_FRAME_CRC     0x52u /* 帧 CRC32 校验失败 */

/*-----------------------------------------------------------
 * 超时参数（SysTick 1 ms 时基）
 *----------------------------------------------------------*/
#define BOOT_UPDATE_HANDSHAKE_TIMEOUT_MS  5000u  /* 等待 START 握手窗口 */
#define BOOT_UPDATE_FRAME_TIMEOUT_MS     1000u  /* 会话内帧间超时 */

/*-----------------------------------------------------------
 * 升级模式进入原因（设计 §5.1 决策优先级）
 *----------------------------------------------------------*/
typedef enum
{
    BOOT_UPDATE_ENTRY_RECOVERY_KEY = 0,  /* 恢复键保持按下：无握手超时回退 */
    BOOT_UPDATE_ENTRY_BKP_REQUEST  = 1,  /* BKP_DR2 请求：超时可回退旧 APP */
    BOOT_UPDATE_ENTRY_FORCED       = 2   /* 元数据非 VALID：强制停留升级模式 */
} Boot_UpdateEntry_t;

/* SessionPoll 返回值 */
typedef enum
{
    BOOT_UPDATE_RUN_ACTIVE = 0,           /* 会话进行中，继续轮询 */
    BOOT_UPDATE_RUN_HANDSHAKE_TIMEOUT     /* 握手超时（仅 BKP 入口会返回；
                                              引擎已重启握手窗口） */
} Boot_UpdateRun_t;

/*-----------------------------------------------------------
 * API
 *----------------------------------------------------------*/

/* 时基（SysTick 1 ms）+ Flash IO 接线 + 工厂容量寄存器检查 +
 * 解析器/会话/写入器上下文初始化。main 最先调用（HSE/PLL 已由
 * startup 的 SystemInit 配好）。 */
void Boot_Update_Init(void);

/* 双页元数据读取 + 启动决策（IAP_BootDecision 语义：最新记录 VALID
 * 且 slot_id/app_base 合法、8<=image_size<=SLOT_CAPACITY 才 CONTINUE）。
 * latestOut 可为 NULL；存在有效记录时回填该记录副本（可能为 RECEIVING
 * 态，以返回值为准），无有效记录时清零。
 *
 * 双槽版签名：内部固定注入 SLOT_A_BASE/SLOT_B_BASE/SLOT_CAPACITY，
 * 调用方无需（也不应）再传分区常量。 */
IAP_BootAction_t Boot_Update_MetaDecision(IAP_MetaRecord_t *latestOut);

/* 当前活动槽基址（最新 VALID 记录指向的槽；无有效记录返回 0）。
 * boot_main 的跳转链用它把"该跳哪个槽"注入 Boot_ImageConfig_t，并据此
 * 设置 VTOR/取向量。与 Boot_Update_MetaDecision 读同一份元数据，二者
 * 之间不存擦写，故一致。 */
u32 Boot_Update_ActiveSlot(void);

/* 工厂容量寄存器 >= 布局要求（FLASH_END_EXCL-FLASH_BASE）：1 可擦写。
 * 低于要求时拒绝一切擦写（C8 克隆片隐藏 128 KiB 不得使用）。 */
u8 Boot_Update_FlashSizeOk(void);

/* 以指定原因进入升级会话（复位解析器/会话并排空接收环）。 */
void Boot_Update_SessionBegin(Boot_UpdateEntry_t entry);

/* 主循环轮询：消费 DMA 环、驱动解析器/会话/Flash 写入与 ACK/NACK。
 * 升级完成（VALID 提交 + 终帧 ACK）在函数内部直接系统复位，不返回。 */
Boot_UpdateRun_t Boot_Update_SessionPoll(void);

/* 会话是否处于 RECEIVING（START 已接受且未完成）：main 用于阶段观测。 */
u8 Boot_Update_IsReceiving(void);

#endif /* __BOOT_UPDATE_H__ */
