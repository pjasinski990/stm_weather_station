# 1 "Middlewares/Third_Party/CMSIS_5/CMSIS/RTOS2/RTX/Source/GCC/irq_armv7m.S"
# 1 "<built-in>"
# 1 "<command-line>"
# 31 "<command-line>"
# 1 "/usr/include/stdc-predef.h" 1 3 4
# 32 "<command-line>" 2
# 1 "Middlewares/Third_Party/CMSIS_5/CMSIS/RTOS2/RTX/Source/GCC/irq_armv7m.S"
# 27 "Middlewares/Third_Party/CMSIS_5/CMSIS/RTOS2/RTX/Source/GCC/irq_armv7m.S"
        .syntax unified

# 1 "Middlewares/Third_Party/CMSIS_5/CMSIS/RTOS2/RTX/Include/rtx_def.h" 1
# 32 "Middlewares/Third_Party/CMSIS_5/CMSIS/RTOS2/RTX/Include/rtx_def.h"
# 1 "Middlewares/Third_Party/CMSIS_5/CMSIS/RTOS2/RTX/Library/RTX_Config.h" 1
# 33 "Middlewares/Third_Party/CMSIS_5/CMSIS/RTOS2/RTX/Include/rtx_def.h" 2
# 30 "Middlewares/Third_Party/CMSIS_5/CMSIS/RTOS2/RTX/Source/GCC/irq_armv7m.S" 2




        .equ FPU_USED, 0


        .equ I_T_RUN_OFS, 20
        .equ TCB_SP_OFS, 56
        .equ TCB_SF_OFS, 34
        .equ TCB_ZONE_OFS, 68

        .equ FPCCR, 0xE000EF34

        .equ osRtxErrorStackOverflow, 1
        .equ osRtxErrorSVC, 6

        .section ".rodata"
        .global irqRtxLib
irqRtxLib:
        .byte 0


        .thumb
        .section ".text"
        .align 2
        .eabi_attribute Tag_ABI_align_preserved, 1


        .thumb_func
        .type SVC_Handler, %function
        .global SVC_Handler
        .fnstart
        .cantunwind
SVC_Handler:

        tst lr,#0x04
        ite eq
        mrseq r0,msp
        mrsne r0,psp

        ldr r1,[r0,#24]
        ldrb r1,[r1,#-2]
        cmp r1,#0
        bne SVC_User
# 100 "Middlewares/Third_Party/CMSIS_5/CMSIS/RTOS2/RTX/Source/GCC/irq_armv7m.S"
        push {r0,lr}
        ldm r0,{r0-r3,r12}
        blx r12
        pop {r12,lr}
        str r0,[r12]

SVC_Context:
        ldr r3,=osRtxInfo+I_T_RUN_OFS
        ldm r3,{r1,r2}
        cmp r1,r2
        it eq
        bxeq lr

        str r2,[r3]

      .if (FPU_USED != 0)
        cbnz r1,SVC_ContextSave
SVC_FP_LazyState:
        tst lr,#0x10
        bne SVC_ContextRestore
        ldr r3,=FPCCR
        ldr r0,[r3]
        bic r0,r0,#1
        str r0,[r3]
        b SVC_ContextRestore
      .else
        cbz r1,SVC_ContextRestore
      .endif

SVC_ContextSave:
# 172 "Middlewares/Third_Party/CMSIS_5/CMSIS/RTOS2/RTX/Source/GCC/irq_armv7m.S"
        stmdb r12!,{r4-r11}
      .if (FPU_USED != 0)
        tst lr,#0x10
        it eq
        vstmdbeq r12!,{s16-s31}
        strb lr, [r1,#TCB_SF_OFS]
      .endif
        str r12,[r1,#TCB_SP_OFS]


SVC_ContextRestore:
        movs r4,r2
# 195 "Middlewares/Third_Party/CMSIS_5/CMSIS/RTOS2/RTX/Source/GCC/irq_armv7m.S"
        ldr r0,[r4,#TCB_SP_OFS]
      .if (FPU_USED != 0)
        ldrsb lr,[r4,#TCB_SF_OFS]
        tst lr,#0x10
        it eq
        vldmiaeq r0!,{s16-s31}
      .else
        mvn lr,#~0xFFFFFFFD
      .endif
        ldmia r0!,{r4-r11}
        msr psp,r0

SVC_Exit:
        bx lr

SVC_User:
        ldr r2,=osRtxUserSVC
        ldr r3,[r2]
        cmp r1,r3
        bhi SVC_Exit

        push {r0,lr}
        ldr r12,[r2,r1,lsl #2]
        ldm r0,{r0-r3}
        blx r12
        pop {r12,lr}
        str r0,[r12]

        bx lr

        .fnend
        .size SVC_Handler, .-SVC_Handler


        .thumb_func
        .type PendSV_Handler, %function
        .global PendSV_Handler
        .fnstart
        .cantunwind
PendSV_Handler:

        push {r0,lr}
        bl osRtxPendSV_Handler
        pop {r0,lr}
        mrs r12,psp
        b SVC_Context

        .fnend
        .size PendSV_Handler, .-PendSV_Handler


        .thumb_func
        .type SysTick_Handler, %function
        .global SysTick_Handler
        .fnstart
        .cantunwind
SysTick_Handler:

        push {r0,lr}
        bl osRtxTick_Handler
        pop {r0,lr}
        mrs r12,psp
        b SVC_Context

        .fnend
        .size SysTick_Handler, .-SysTick_Handler
# 281 "Middlewares/Third_Party/CMSIS_5/CMSIS/RTOS2/RTX/Source/GCC/irq_armv7m.S"
        .end
