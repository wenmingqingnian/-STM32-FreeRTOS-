/*
 * Boot UART DMA 环形接收
 *
 * 职责：
 *   1. USART1（PA9/PA10，115200-8N1）+ DMA1_Channel5 循环模式接收：
 *      1024 B、4 字节对齐静态环形缓冲，替代旧版固定地址接收区（旧 C8 时代 SRAM 末尾预留区，已随布局冻结废弃）；
 *      容量下界依据：≥ 单帧线上最大长度 536 B（24 B 帧头 + 512 B 负载，
 *      见 boot_protocol.h），保证 stop-and-wait 下一帧必定整帧放得下；
 *      同时取 2 的幂以便环内取模用掩码。为什么选 DMA 而非 RXNE 中断：
 *      擦写期间 Flash 总线停摆、ISR 代码无法取指，中断方案在擦除窗口内失效，
 *      而 DMA 走 USART(APB2)->SRAM 不碰 Flash，可继续收。
 *   2. USART IDLE / DMA ISR 只做标志清除与计数（producer 位置由主循环与
 *      擦页后端 查询 NDTR 推导，ISR 不解析协议、不打印、不加锁操作 Ring 状态）；
 *   3. 主循环消费接口：Pending() 返回 [消费者, 生产者) 的连续跨度（跨环
 *      尾自动拆两段），Consume() 推进消费者；
 *   4. 轮询式发送（ACK/NACK 出口）：逐字节等 TXE，帧尾等 TC，不用
 *      printf、不用 DMA TX。
 *
 * 生产者追踪与溢出检测（设计 §5.2 stop-and-wait 前提）：
 *   - producer 绝对位置由 Boot_UartDma_Poll() 用【环内位置回跳】累加更新：
 *     环内位置 = 1024 - NDTR，新位置小于上次快照即判定一次环绕，绝对位置
 *     按 (1024 - 上次快照) + 新位置 进一整圈；两次 Poll 之间位置恰好前进
 *     整圈时判为"无变化"（等号歧义），故依赖下列调用密度；
 *   - 该回跳法只能分辨【一圈】：两次 Poll 之间到达超过 1024 B 会少计。
 *     调用方必须保证 Poll 间隔覆盖的到达量 < 1024 B。当前两个调用点：
 *       (a) 主循环每轮 —— Boot_Update_SessionPoll()（boot_update.c）；
 *       (b) 擦页后端逐页 —— tio_erase_page()（boot_update.c，与逐页喂狗同一
 *           位置），把擦除窗口的 Poll 间隔压到"一页"，115200 波特率下
 *           40 ms 页擦除最多到达约 460 B < 1024 B，回跳法由此成立。
 *           ⚠️ 该调用点不可删：擦写期间对 Flash 的读访问让总线停住，ISR
 *           代码在 Flash 中无法取指，擦除窗口内没有别处能采样进度；
 *   - DMA1_Channel5 的 TC 中断开着并计入 s_tcCount，但【不参与位置推导】，
 *     目前只作证据（s_tcCount ≤ 环绕次数：擦写期间多次环绕只会让 TC 标志
 *     与 NVIC pending 保持"置位一次"，ISR 只执行一次→计数少计）。因此
 *     中断计数【不能】替代轮询；若要利用它，只能做交叉校验
 *（需另加 GetTcCount 类接口读取，本模块当前未提供）；
 *   - 溢出（生产者套圈消费者）置粘滞 overrun 标志：主机违反 stop-and-wait
 *     或 Boot 长时间阻塞导致。溢出后调用方应 ResetRx() 排空环形缓冲并
 *     复位解析器，靠帧 CRC + 主机重传收敛；
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#ifndef __BOOT_UART_DMA_H__
#define __BOOT_UART_DMA_H__

#include "stm32f10x.h"
#include "boot_minimal.h"   /* BOOT_UART_BAUDRATE */

#define BOOT_UART_DMA_RING_SIZE   1024u   /* 2 的幂：环形取模用掩码 */

void Boot_UartDma_Init(void);

/* 更新生产者绝对位置（主循环与逐页擦除间隙调用；内部做环绕检测） */
void Boot_UartDma_Poll(void);

/* 待消费跨度：返回总字节数；span0/span1 为两段连续区（跨环尾时两段都有效） */
u16  Boot_UartDma_Pending(const u8 **span0, u16 *len0,
                          const u8 **span1, u16 *len1);

/* 消费 n 字节（n <= Pending() 返回值；只有消费方调用） */
void Boot_UartDma_Consume(u16 n);

u8   Boot_UartDma_Overrun(void);      /* 粘滞溢出标志（Poll 检测） */
void Boot_UartDma_ResetRx(void);      /* 排空环形缓冲，清 overrun 标志 */

u32  Boot_UartDma_GetIdleCount(void); /* USART IDLE 中断计数（证据） */
u32  Boot_UartDma_GetErrorCount(void);/* 接收错误计数（ORE/NE/FE/PE/DMA TE） */

/* 轮询发送（ACK/NACK 帧）；len=0 只等 TC 清空移位寄存器 */
void Boot_UartDma_Send(const u8 *data, u16 len);

#endif /* __BOOT_UART_DMA_H__ */
