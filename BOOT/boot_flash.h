/*
 * Boot 专用 Flash 写入器
 *
 * 职责：
 *   1. 严格有界的目标槽写入：擦除/编程/回读/整镜像 CRC 只允许
 *      [appBase, appLimitExcl)；Boot 区与双页元数据（page 126/127）
 *      一律拒绝——越界判定先减后比，禁止可被 u32 回绕绕过的
 *      addr+len <= limit 写法；
 *   2. 半字编程流：DATA 包按半字逐个写入（每次检查后端状态，任一
 *      失败立即停止并保留错误码与 failAddr），奇数末字节以 0xFF 补
 *      高字节，镜像 CRC / 回读只覆盖真实字节；
 *   3. 元数据页独立 API：只能擦 A/B 两页（EraseMetaPage），记录写入
 *      事务由 BSP/IAP/iap_meta.c 承担（commit 半字最后编程）；
 *   4. 解锁纪律：每次写操作（擦除/编程/元数据擦除）解锁一次，
 *      全部返回路径（含错误路径）统一上锁；只读操作（回读/CRC）
 *      与参数校验失败不解锁。
 *
 * 本文件不含任何 2 KiB 页镜像（不复用旧 STMFLASH_Write 的整页读改写），
 * 也不直接访问寄存器：真实 FLASH_ErasePage/FLASH_ProgramHalfWord 适配
 *（含 FLASH_Status 逐次检查）由 Task 8 的目标侧后端提供，host mock
 * 接 fake_flash.c（页擦除 / 1->0 编程 / 断电注入）。
 *
 * 地址常量不硬编码：分区配置由调用方从 CONFIG/memory_layout.h 填入
 * BootFlash_Config_t（与 iap_meta 的地址注入约定一致）。
 *
 */
#ifndef __BOOT_FLASH_H__
#define __BOOT_FLASH_H__

#include <stddef.h>   /* NULL — ARMCC 与 host gcc 通用 */
#include "sys.h"

/*-----------------------------------------------------------
 * 状态码（同时保留在 BootFlash_t.status 直至下一次操作）
 *----------------------------------------------------------*/
typedef enum {
    BOOTFLASH_OK = 0,
    BOOTFLASH_ERR_PARAM,     /* NULL 参数 / 长度 0 / 元数据选择子非法 */
    BOOTFLASH_ERR_BOUNDS,   /* 越界：Boot 区 / 元数据页 / APP 终点外 / 长度回绕 */
    BOOTFLASH_ERR_ALIGN,    /* erase 非 2 KiB 页对齐 / program 非 2 B 对齐 */
    BOOTFLASH_ERR_ERASE,    /* 后端擦除失败（立即停止） */
    BOOTFLASH_ERR_PROGRAM,  /* 后端半字编程失败（立即停止） */
    BOOTFLASH_ERR_READBACK, /* 包级回读不一致（Task 8 拒绝 ACK 的依据） */
    BOOTFLASH_ERR_CRC       /* 整镜像 CRC 不一致（Task 8 拒绝提交 VALID 的依据） */
} BootFlash_Status_t;

/*-----------------------------------------------------------
 * 元数据页选择子（EraseMetaPage 专用）
 *----------------------------------------------------------*/
#define BOOTFLASH_META_A   0u
#define BOOTFLASH_META_B   1u

/*-----------------------------------------------------------
 * Flash 后端接口（依赖注入）：
 *   host 测试侧接 fake_flash.c（经绝对地址->偏移换算的 wrapper）；
 *   目标侧适配 FLASH_Unlock/FLASH_Lock/FLASH_ErasePage/
 *   FLASH_ProgramHalfWord/直接读半字（Task 8 提供），
 *   erase_page/program_halfword 返回 0 成功、非 0 失败
 *（FLASH_Status != FLASH_COMPLETE 的映射）。
 *   unlock/lock 可为 NULL（无锁后端）；契约：调用本模块写 API 时
 *   各操作指针必须非 NULL。
 *----------------------------------------------------------*/
typedef struct {
    void *self;
    void (*unlock)(void *self);          /* 可 NULL：读操作不调用 */
    void (*lock)(void *self);           /* 可 NULL：读操作不调用 */
    int  (*erase_page)(void *self, u32 page_addr);
    int  (*program_halfword)(void *self, u32 addr, u16 value);
    u16  (*read_halfword)(void *self, u32 addr);
} BootFlash_Io_t;

/*-----------------------------------------------------------
 * 分区配置（目标侧从 CONFIG/memory_layout.h 填充；pageSize 为 2 的幂）
 *
 * 双槽改造：appBase 不再是编译期常量 APP_BASE，而是【本次会话要写入
 * 的目标槽基址】——Boot_Update_Init 时取"当前活动槽的另一槽"
 *（IAP_SlotOther），升级写完 VALID 复位后由 Boot 重新决策。因此
 * appBase 仍为运行期字段（本模块无任何编译期槽假设），只是其来源从
 * 常量改为运行期计算，其余语义（边界/对齐/先减后比）一概不变。
 *----------------------------------------------------------*/
typedef struct {
    u32 appBase;        /* 目标槽基址（运行期活动槽的另一槽；非编译期常量） */
    u32 appLimitExcl;  /* APP_LIMIT_EXCL（两槽合计上界，含元数据页前，不含） */
    u32 metaABase;      /* META_A_BASE（page 126） */
    u32 metaBBase;      /* META_B_BASE（page 127） */
    u32 pageSize;       /* FLASH_PAGE_SIZE = 2 KiB（高密度 F1） */
} BootFlash_Config_t;

/*-----------------------------------------------------------
 * 写入器上下文
 *----------------------------------------------------------*/
typedef struct {
    const BootFlash_Io_t *io;
    BootFlash_Config_t cfg;
    u8  status;    /* 最近一次操作结果（BootFlash_Status_t，保留至下次操作） */
    u32 failAddr;  /* 首个失败/不一致的绝对地址（诊断与定位坏包用）；
                      CRC 失败置 0（整镜像级错误无单一地址） */
} BootFlash_t;

void BootFlash_Init(BootFlash_t *f, const BootFlash_Io_t *io,
                    const BootFlash_Config_t *cfg);

/*-----------------------------------------------------------
 * APP 分区擦除：擦除 startAddr 起 numPages 页。
 *   - startAddr 必须 2 KiB 页对齐且在 [appBase, appLimitExcl) 内；
 *   - numPages >= 1，且 numPages <= (appLimitExcl - startAddr)/pageSize
 *（先减后比：numPages=0xFFFFFFFF 类回绕输入必须拒绝）。
 * image_size == APP_CAPACITY 恰可整区擦除；+1 字节对应的页数越界拒绝。
 *----------------------------------------------------------*/
BootFlash_Status_t BootFlash_ErasePages(BootFlash_t *f, u32 startAddr, u32 numPages);

/*-----------------------------------------------------------
 * 元数据页擦除（独立 API，只能擦 META_A/META_B 两页之一）：
 * 记录写入（擦页+编码+commit 最后编程）由 iap_meta 承担。
 *----------------------------------------------------------*/
BootFlash_Status_t BootFlash_EraseMetaPage(BootFlash_t *f, u8 metaSel);

/*-----------------------------------------------------------
 * DATA 包半字编程：len 字节写入绝对地址 addr。
 *   - addr 必须 2 B 对齐且在 [appBase, appLimitExcl) 内；
 *   - len <= appLimitExcl - addr（先减后比防溢出）；
 *   - len 奇数时末字节以 0xFF 补高字节编程；
 *   - 每个半字编程后端状态逐次检查，失败立即停止。
 *----------------------------------------------------------*/
BootFlash_Status_t BootFlash_ProgramData(BootFlash_t *f, u32 addr,
                                         const u8 *data, u32 len);

/*-----------------------------------------------------------
 * 包级回读（编程后、ACK 前）：逐半字比较 Flash 内容与发送数据，
 * 奇数末字节只比较低字节（补位 0xFF 不参与）。
 * 只读操作：不解锁/上锁。
 *----------------------------------------------------------*/
BootFlash_Status_t BootFlash_ReadbackVerify(BootFlash_t *f, u32 addr,
                                            const u8 *data, u32 len);

/*-----------------------------------------------------------
 * 整镜像 CRC 校验（END 后、提交 VALID 前）：从 addr 流式读取 size
 * 字节计算 CRC32（复用 Task 4 流式接口），与 expectedCrc 比较。
 * addr 必须 2 B 对齐且 [addr, addr+size) 在 APP 区内；
 * 奇数 size 的末字节只取半字低字节（补位 0xFF 不计入 CRC）。
 * 只读操作：不解锁/上锁。
 *----------------------------------------------------------*/
BootFlash_Status_t BootFlash_VerifyCrc(BootFlash_t *f, u32 addr, u32 size,
                                       u32 expectedCrc);

#endif /* __BOOT_FLASH_H__ */
