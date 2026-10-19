;*******************************************************************************
;* File Name          : boot_jump.s
;* Description        : IAP 无栈尾跳（IAP，设计文档 §5.5）
;*
;* 职责：完成 C 侧 NVIC/SCB 清理之后的最终跳转序列——
;*       清 BASEPRI/FAULTMASK -> 切回 privileged MSP -> 写 APP 初始 MSP
;*       -> CPSIE i -> BX reset_handler。
;*
;* 硬性约束：
;*   1. MSR MSP 之后不得出现任何压栈指令（PUSH/POP/BL/LDM/STM）——
;*      MSP 已指向 APP 栈，任何压栈都会破坏 APP 复位现场；
;*   2. 本函数不返回：BX 之后没有后继指令；
;*   3. AAPCS 入参：R0 = APP 初始 MSP，R1 = APP Reset Handler（Thumb，
;*      bit0=1，已由 Boot_ImageValidate 保证）。
;*
;* 为什么这段必须在汇编里（原理要点）：
;*   C 编译器随时可能在任意语句间插入栈帧维护；只有汇编能向静态审查
;*   证明"写 MSP 与 BX 之间零栈操作"。BL 进入本函数发生在 MSR MSP
;*   之前（C 侧），不影响本约束。
;*
;* 依据：docs/design-evidence/IAP_FAILURE_MATRIX.md（无栈跳转契约）
;*******************************************************************************

        AREA    |.text|, CODE, READONLY

        THUMB

        EXPORT  Boot_JumpTail

;------------------------------------------------------------------------------
; void Boot_JumpTail(u32 msp, u32 resetHandler)
;------------------------------------------------------------------------------
Boot_JumpTail   PROC
        ; R0 = APP 初始 MSP（已校验：SRAM 内且 8B 对齐）
        ; R1 = APP Reset Handler（已校验：Thumb 且镜像内）

        ; 1. 清 BASEPRI / FAULTMASK：解除升级路径可能设置的优先级屏蔽
        MOVS    R2, #0
        MSR     BASEPRI, R2
        MSR     FAULTMASK, R2

        ; 2. 切回 privileged / MSP（清 CONTROL.SPSEL；nPRIV 保持 0）。
        ;    Boot 裸机从未用 PSP，此处显式清零属冗余防御；CONTROL 写后
        ;    必须 ISB 才保证后续取指使用正确上下文
        MRS     R2, CONTROL
        BICS    R2, R2, #2
        MSR     CONTROL, R2
        ISB

        ; 3. 写 APP 初始 MSP——此后不得再出现任何栈操作（见文件头约束 1）
        MSR     MSP, R0

        ; 4. 开全局中断：APP 复位态期望 PRIMASK=0；VTOR 已在 C 侧指向
        ;    活动槽基址（Slot A 或 Slot B），此处开中断后可能的异常入口
        ;    取的都是正确槽的向量（本函数与槽无关：R0/R1 由调用方按
        ;    活动槽传入，Boot_JumpTail 不引用任何地址常量）
        CPSIE   i

        ; 5. 尾跳 APP Reset Handler（bit0=1 维持 Thumb 态；不返回）
        BX      R1

        ALIGN
        ENDP

        ALIGN
        END
