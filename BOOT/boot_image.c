/*
 * Boot 镜像合法性校验实现
 *
 * 实现要点：
 *   - 校验链顺序 PARAM -> STATE -> SIZE -> BASE -> MSP -> PC -> CRC，
 *     每级失败立即返回具体错误码（短路由 mock 测试的链序用例证明）；
 *   - 向量字经两次半字读拼装（与后端读粒度一致），不依赖对齐假设；
 *   - 整镜像 CRC 走流式接口逐半字输入：奇数 image_size 的末半字只取
 *     低字节（真实末字节），0xFF 填充高字节不计入——与
 *     BootFlash_ProgramData 的奇补写入规则和 boot_update.c END 校验
 *     口径一致；
 *   - 只读契约：本文件不引用任何擦写后端/锁操作。
 *
 * 依据：docs/design-evidence/IAP_STOP_BASELINE.md
 */
#include "boot_image.h"
#include "crc32.h"

/*-----------------------------------------------------------
 * 内部工具：读 4 B 向量字（小端拼装，两半字）
 *----------------------------------------------------------*/
static u32 bi_read_word(const Boot_ImageIo_t *io, u32 addr)
{
    u32 lo = (u32)io->read_halfword(io->self, addr);
    u32 hi = (u32)io->read_halfword(io->self, addr + 2u);
    return lo | (hi << 16);
}

/*-----------------------------------------------------------
 * 整镜像流式 CRC（只计真实字节；奇数末字节的 0xFF 填充除外）
 *----------------------------------------------------------*/
static u32 bi_image_crc(const Boot_ImageIo_t *io, u32 base, u32 len)
{
    CRC32_StreamCtx_t ctx;
    u32 i;
    u32 hw;
    u8 b[2];

    CRC32_Init(&ctx);
    for (i = 0u; i < (len >> 1u); i++)
    {
        hw = (u32)io->read_halfword(io->self, base + 2u * i);
        b[0] = (u8)(hw & 0xFFu);
        b[1] = (u8)((hw >> 8) & 0xFFu);
        CRC32_Update(&ctx, b, 2u);
    }
    if ((len & 1u) != 0u)
    {
        hw = (u32)io->read_halfword(io->self, base + 2u * (len >> 1u));
        b[0] = (u8)(hw & 0xFFu);      /* 真实末字节 */
        CRC32_Update(&ctx, b, 1u);    /* 0xFF 填充高字节不入 CRC */
    }
    return CRC32_Final(&ctx);
}

/*-----------------------------------------------------------
 * 校验链
 *----------------------------------------------------------*/
Boot_ImageStatus_t Boot_ImageValidate(const IAP_MetaRecord_t *rec,
                                      const Boot_ImageIo_t *io,
                                      const Boot_ImageConfig_t *cfg)
{
    u32 msp;
    u32 pc;

    /* 0. PARAM */
    if ((rec == NULL) || (io == NULL) || (cfg == NULL) ||
        (io->read_halfword == NULL))
    {
        return BOOT_IMAGE_ERR_PARAM;
    }

    /* 1. STATE：记录可解码且为 VALID（RECEIVING 拒绝）*/
    if ((rec->valid == 0u) || (rec->stateWord != IAP_META_STATE_VALID))
    {
        return BOOT_IMAGE_ERR_STATE;
    }

    /* 2. SIZE：8 <= image_size <= appCapacity（调用方注入 = SLOT_CAPACITY） */
    if ((rec->imageSize < 8u) || (rec->imageSize > cfg->appCapacity))
    {
        return BOOT_IMAGE_ERR_SIZE;
    }

    /* 3. BASE：元数据基址与编译时布局严格一致 */
    if (rec->appBase != cfg->appBase)
    {
        return BOOT_IMAGE_ERR_BASE;
    }

    /* 4. MSP：完整 SRAM 范围内（上界含）且 8 B 对齐 */
    msp = bi_read_word(io, cfg->appBase);
    if ((msp & 0x7u) != 0u)
    {
        return BOOT_IMAGE_ERR_MSP;
    }
    if ((msp < cfg->sramBase) || (msp > cfg->sramLimit))
    {
        return BOOT_IMAGE_ERR_MSP;
    }

    /* 5. PC：Thumb(bit0=1)，清位后落在 [appBase, appBase+image_size) */
    pc = bi_read_word(io, cfg->appBase + 4u);
    if ((pc & 1u) == 0u)
    {
        return BOOT_IMAGE_ERR_PC;
    }
    pc &= ~((u32)1u);
    if ((pc < cfg->appBase) || (pc >= (cfg->appBase + rec->imageSize)))
    {
        return BOOT_IMAGE_ERR_PC;
    }

    /* 6. CRC：整镜像重算与元数据一致 */
    if (bi_image_crc(io, cfg->appBase, rec->imageSize) != rec->imageCrc)
    {
        return BOOT_IMAGE_ERR_CRC;
    }

    return BOOT_IMAGE_OK;
}
