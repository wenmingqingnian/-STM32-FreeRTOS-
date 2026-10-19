/*
 * Boot 镜像合法性校验
 *
 * 职责——在元数据决策（IAP_BootDecision，Task 5）通过之后、无栈跳转
 *（boot_jump.s，Task 9）之前，对 Flash 里的 APP 镜像做最后一道校验：
 *
 *   1. 记录状态：最新记录必须可解码且为 VALID——RECEIVING（升级写入
 *      中断电）即使向量/CRC 全部正确也拒绝；
 *   2. 尺寸：8 <= image_size <= cfg->appCapacity（= SLOT_CAPACITY，<8 连向量表
 *      都不完整）；
 *   3. 基址：元数据 app_base 必须与 cfg->appBase（本次目标槽）严格一致
 *（防错位烧写）；调用方在进入本链前已由 IAP_BootDecision 保证
 *      slot_id 合法且 app_base 属于某个已知槽，本模块只做"是否等于
 *      本次目标槽"的单点比较，不需要重新分叉槽逻辑；
 *   4. 初始 MSP：完整 SRAM 范围内且 8 B 对齐——上界取"含"语义
 *（0x20005000 = SRAM 顶端，APP 复位态标准初值），
 *      不能误用 < sramLimit 排除合法值；
 *   5. Reset Handler：bit0=1（Thumb），清 bit0 后落在
 *      [appBase, appBase+image_size)——范围判定必须用清位后的值；
 *   6. 整镜像 CRC：从 cfg->appBase 流式重算（含向量表），奇数 image_size
 *      的 0xFF 填充半字节不计入。
 *
 * 校验链顺序（设计 §5.5，测试证明逐级短路）：
 *   PARAM -> STATE -> SIZE -> BASE -> MSP -> PC -> CRC
 *
 * 本文件只含纯逻辑 + 经注入的只读后端，不访问任何寄存器，便于主机
 * fake_flash 测试；地址常量一律由调用方从 CONFIG/memory_layout.h
 * 与板级事实（设计 §4 SRAM 边界）填入，本模块不硬编码任何地址。
 * 只读契约由链接集证明：本模块不引用任何擦写后端。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#ifndef __BOOT_IMAGE_H__
#define __BOOT_IMAGE_H__

#include <stddef.h>   /* NULL — ARMCC 与 host gcc 通用 */
#include "sys.h"
#include "iap_meta.h"  /* IAP_MetaRecord_t / IAP_META_STATE_VALID */

/*-----------------------------------------------------------
 * 只读后端（依赖注入）：
 *   host 测试侧接 fake_flash.c（绝对地址->偏移换算 wrapper）；
 *   目标侧适配内存映射直读（*(volatile u16 *)addr，见 boot_main.c
 *   Task 9 接线），与 BootFlash_Io_t::read_halfword 同语义。
 *----------------------------------------------------------*/
typedef struct {
    void *self;
    u16 (*read_halfword)(void *self, u32 addr);
} Boot_ImageIo_t;

/*-----------------------------------------------------------
 * 校验配置（调用方注入；Flash 常量来自 CONFIG/memory_layout.h，
 * SRAM 边界为芯片事实：0x20000000/0x20005000，见设计 §4）
 *
 * 双槽改造：appBase 为【本次要启动的影子槽基址】——Boot_JumpToApp 决策
 * 出活动槽后填入（IAP_SlotBaseFromRecord），校验链的 BASE 级即校验
 * "元数据 app_base == 该槽规定基址"，与槽无关的代码零分叉。
 *----------------------------------------------------------*/
typedef struct {
    u32 appBase;     /* 目标槽基址（运行期活动槽，非编译期常量） */
    u32 appCapacity; /* 槽容量（SLOT_CAPACITY） */
    u32 sramBase;    /* SRAM_BASE = 0x20000000 */
    u32 sramLimit;   /* SRAM 顶端：MSP 可等于该值（含上界，0x20005000）*/
} Boot_ImageConfig_t;

/*-----------------------------------------------------------
 * 状态码（具体失败原因，供决策与诊断）
 *----------------------------------------------------------*/
typedef enum {
    BOOT_IMAGE_OK = 0,
    BOOT_IMAGE_ERR_PARAM,  /* NULL 参数 / 读后端缺函数指针 */
    BOOT_IMAGE_ERR_STATE,  /* 记录不可解码 / 非 VALID（RECEIVING 等）*/
    BOOT_IMAGE_ERR_SIZE,   /* image_size < 8 或 > cfg->appCapacity */
    BOOT_IMAGE_ERR_BASE,   /* 元数据 app_base != cfg->appBase（目标槽） */
    BOOT_IMAGE_ERR_MSP,    /* 初始 MSP 越界 / 未 8 B 对齐 */
    BOOT_IMAGE_ERR_PC,     /* Reset Handler 非 Thumb / 清位后越界 */
    BOOT_IMAGE_ERR_CRC     /* 整镜像 CRC 与元数据不一致 */
} Boot_ImageStatus_t;

/*-----------------------------------------------------------
 * 镜像校验链入口
 *
 * rec：双页元数据解码视图（Boot_Update_MetaDecision 回填的最新记录）。
 * 返回 BOOT_IMAGE_OK 才允许进入跳转准备；任何非 OK 均停留升级模式。
 * 只读：本函数不做任何擦写，对 Flash 的访问仅 read_halfword。
 *----------------------------------------------------------*/
Boot_ImageStatus_t Boot_ImageValidate(const IAP_MetaRecord_t *rec,
                                      const Boot_ImageIo_t *io,
                                      const Boot_ImageConfig_t *cfg);

#endif /* __BOOT_IMAGE_H__ */
