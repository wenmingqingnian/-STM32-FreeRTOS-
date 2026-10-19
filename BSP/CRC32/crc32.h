#ifndef __CRC32_H__
#define __CRC32_H__
#include "sys.h"

/*************************************************************************
功能：用于IAP固件校验


**************************************************************************/

/**
 * @brief 流式CRC32计算上下文（IAP计划Task 4）
 * @note  只保存运行余数状态，不复制查找表，不引入动态内存；
 *        上下文可放调用者任意存储（栈/静态），同一上下文串行使用
 */
typedef struct {
    u32 state;  /* 运行余数（初始0xFFFFFFFF，Final前不异或0xFFFFFFFF） */
} CRC32_StreamCtx_t;

/**
 * @brief 初始化流式CRC32上下文
 * @param ctx: 上下文指针
 */
void CRC32_Init(CRC32_StreamCtx_t *ctx);

/**
 * @brief 流式输入数据（可多次调用，分块长度任意）
 * @param ctx: 上下文指针
 * @param data: 数据指针（NULL仅当length为0时允许）
 * @param length: 数据长度（字节）
 * @note  分块计算结果与一次性计算完全一致（UART DMA分包写入路径）
 */
void CRC32_Update(CRC32_StreamCtx_t *ctx, const u8 *data, u32 length);

/**
 * @brief 结束流式计算并返回CRC32校验值（只读，可重复调用）
 * @param ctx: 上下文指针
 * @return CRC32校验值
 */
u32 CRC32_Final(const CRC32_StreamCtx_t *ctx);

/**
 * @brief 计算数据的CRC32校验值
 * @param data: 数据指针
 * @param length: 数据长度（字节）
 * @return CRC32校验值
 * @note 兼容包装：内部走Init/Update/Final，与流式结果一致
 */
u32 CRC32_Calculate(u8 *data, u32 length);

/**
 * @brief 校验固件CRC32
 * @param data: 固件数据指针（包含末尾4字节CRC）
 * @param totalLength: 总长度（固件+4字节CRC）
 * @return 1: 校验通过, 0: 校验失败
 */
u8 CRC32_VerifyFirmware(u8 *data, u32 totalLength);

#endif
