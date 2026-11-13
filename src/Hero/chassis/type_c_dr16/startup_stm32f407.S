.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.section .isr_vector,"a",%progbits
.balign 512
.global vector_table
vector_table:
    .word _estack
    .word Reset_Handler
    .word Default_Handler              /* NMI */
    .word Default_Handler              /* HardFault */
    .word Default_Handler              /* MemManage */
    .word Default_Handler              /* BusFault */
    .word Default_Handler              /* UsageFault */
    .word 0, 0, 0, 0
    .word Default_Handler              /* SVC */
    .word Default_Handler              /* DebugMon */
    .word 0
    .word Default_Handler              /* PendSV */
    .word SysTick_Handler
    .rept 12                           /* F407 external IRQs 0..11 */
    .word Default_Handler
    .endr
    .word DMA1_Stream1_IRQHandler       /* IRQ 12 */
    .rept 26                           /* IRQs 13..38 */
    .word Default_Handler
    .endr
    .word USART3_IRQHandler             /* IRQ 39 */
    .rept 42                           /* IRQs 40..81 */
    .word Default_Handler
    .endr

.section .text.Reset_Handler,"ax",%progbits
.global Reset_Handler
.type Reset_Handler,%function
.thumb_func
Reset_Handler:
    cpsid i
    ldr r0, =0xe000ed88                /* CPACR: CP10/11 full access. */
    ldr r1, [r0]
    ldr r2, =0x00f00000
    orrs r1, r1, r2
    str r1, [r0]
    dsb
    isb
    ldr r0, =_sidata
    ldr r1, =_sdata
    ldr r2, =_edata
1:
    cmp r1, r2
    bcs 2f
    ldr r3, [r0], #4
    str r3, [r1], #4
    b 1b
2:
    ldr r1, =_sbss
    ldr r2, =_ebss
    movs r3, #0
3:
    cmp r1, r2
    bcs 4f
    str r3, [r1], #4
    b 3b
4:
    ldr r0, =0xe000ed08                /* VTOR uses this firmware vector. */
    ldr r1, =vector_table
    str r1, [r0]
    dsb
    isb
    cpsie i
    bl main
    b Default_Handler
.size Reset_Handler, .-Reset_Handler

.section .note.GNU-stack,"",%progbits
