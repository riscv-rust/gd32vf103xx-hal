#if __riscv_xlen == 64
# define STORE    sd
# define LOAD     ld
# define LOG_REGBYTES 3
#else
# define STORE    sw
# define LOAD     lw
# define LOG_REGBYTES 2
#endif
#define REGBYTES (1 << LOG_REGBYTES)

#define CSR_MSTATUS     0x300
#define CSR_MTVT        0x307
#define CSR_MEPC        0x341
#define CSR_MCAUSE      0x342
#define CSR_MTVT2       0x7EC
#define CSR_JALMNXTI    0x7ED
#define CSR_PUSHMCAUSE  0x7EE
#define CSR_PUSHMEPC    0x7EF
#define CSR_PUSHMSUBM   0x7EB
#define CSR_MMISC_CTL   0x7d0
#define CSR_MSUBM       0x7c4

#define MSTATUS_MIE         0x00000008

.macro DISABLE_MIE
  csrc CSR_MSTATUS, MSTATUS_MIE
.endm

.macro SAVE_CONTEXT
#ifdef __riscv_flen
  #if (__riscv_flen==64 )
    addi sp, sp, -20*REGBYTES - 20*FPREGBYTES
  #else
    addi sp, sp, -20*REGBYTES
  #endif
#else
  addi sp, sp, -20*REGBYTES
#endif
  STORE x1, 0*REGBYTES(sp)
	STORE x4, 1*REGBYTES(sp)
	STORE x5, 2*REGBYTES(sp)
	STORE x6, 3*REGBYTES(sp)
	STORE x7, 4*REGBYTES(sp)
	STORE x10, 5*REGBYTES(sp)
	STORE x11, 6*REGBYTES(sp)
	STORE x12, 7*REGBYTES(sp)
	STORE x13, 8*REGBYTES(sp)
	STORE x14, 9*REGBYTES(sp)
	STORE x15, 10*REGBYTES(sp)
#ifndef __riscv_32e
    STORE x16, 11*REGBYTES(sp)
	STORE x17, 12*REGBYTES(sp)
	STORE x28, 13*REGBYTES(sp)
	STORE x29, 14*REGBYTES(sp)
	STORE x30, 15*REGBYTES(sp)
	STORE x31, 16*REGBYTES(sp)
#endif

#ifdef __riscv_flen
  #if (__riscv_flen == 64)
  FPSTORE f0, (20*REGBYTES + 0*FPREGBYTES)(sp)
	FPSTORE f1, (20*REGBYTES + 1*FPREGBYTES)(sp)
	FPSTORE f2, (20*REGBYTES + 2*FPREGBYTES)(sp)
	FPSTORE f3, (20*REGBYTES + 3*FPREGBYTES)(sp)
	FPSTORE f4, (20*REGBYTES + 4*FPREGBYTES)(sp)
	FPSTORE f5, (20*REGBYTES + 5*FPREGBYTES)(sp)
	FPSTORE f6, (20*REGBYTES + 6*FPREGBYTES)(sp)
	FPSTORE f7, (20*REGBYTES + 7*FPREGBYTES)(sp)
	FPSTORE f10, (20*REGBYTES + 8*FPREGBYTES)(sp)
	FPSTORE f11, (20*REGBYTES + 9*FPREGBYTES)(sp)
	FPSTORE f12, (20*REGBYTES + 10*FPREGBYTES)(sp)
	FPSTORE f13, (20*REGBYTES + 11*FPREGBYTES)(sp)
	FPSTORE f14, (20*REGBYTES + 12*FPREGBYTES)(sp)
	FPSTORE f15, (20*REGBYTES + 13*FPREGBYTES)(sp)
	FPSTORE f16, (20*REGBYTES + 14*FPREGBYTES)(sp)
	FPSTORE f17, (20*REGBYTES + 15*FPREGBYTES)(sp)
	FPSTORE f28, (20*REGBYTES + 16*FPREGBYTES)(sp)
	FPSTORE f29, (20*REGBYTES + 17*FPREGBYTES)(sp)
	FPSTORE f30, (20*REGBYTES + 18*FPREGBYTES)(sp)
	FPSTORE f31, (20*REGBYTES + 19*FPREGBYTES)(sp)
  #endif
#endif
.endm

.macro RESTORE_CONTEXT
    LOAD x1, 0*REGBYTES(sp)
	LOAD x4, 1*REGBYTES(sp)
	LOAD x5, 2*REGBYTES(sp)
	LOAD x6, 3*REGBYTES(sp)
	LOAD x7, 4*REGBYTES(sp)
	LOAD x10, 5*REGBYTES(sp)
	LOAD x11, 6*REGBYTES(sp)
	LOAD x12, 7*REGBYTES(sp)
	LOAD x13, 8*REGBYTES(sp)
	LOAD x14, 9*REGBYTES(sp)
	LOAD x15, 10*REGBYTES(sp)
#ifndef __riscv_32e
  LOAD x16, 11*REGBYTES(sp)
	LOAD x17, 12*REGBYTES(sp)
	LOAD x28, 13*REGBYTES(sp)
	LOAD x29, 14*REGBYTES(sp)
	LOAD x30, 15*REGBYTES(sp)
	LOAD x31, 16*REGBYTES(sp)
#endif

#ifdef __riscv_flen
  #if (__riscv_flen==64)
/* Restore fp caller registers */
	FPLOAD f0, (20*REGBYTES + 0*FPREGBYTES)(sp)
	FPLOAD f1, (20*REGBYTES + 1*FPREGBYTES)(sp)
	FPLOAD f2, (20*REGBYTES + 2*FPREGBYTES)(sp)
	FPLOAD f3, (20*REGBYTES + 3*FPREGBYTES)(sp)
	FPLOAD f4, (20*REGBYTES + 4*FPREGBYTES)(sp)
	FPLOAD f5, (20*REGBYTES + 5*FPREGBYTES)(sp)
	FPLOAD f6, (20*REGBYTES + 6*FPREGBYTES)(sp)
	FPLOAD f7, (20*REGBYTES + 7*FPREGBYTES)(sp)
	FPLOAD f10, (20*REGBYTES + 8*FPREGBYTES)(sp)
	FPLOAD f11, (20*REGBYTES + 9*FPREGBYTES)(sp)
	FPLOAD f12, (20*REGBYTES + 10*FPREGBYTES)(sp)
	FPLOAD f13, (20*REGBYTES + 11*FPREGBYTES)(sp)
	FPLOAD f14, (20*REGBYTES + 12*FPREGBYTES)(sp)
	FPLOAD f15, (20*REGBYTES + 13*FPREGBYTES)(sp)
	FPLOAD f16, (20*REGBYTES + 14*FPREGBYTES)(sp)
	FPLOAD f17, (20*REGBYTES + 15*FPREGBYTES)(sp)
	FPLOAD f28, (20*REGBYTES + 16*FPREGBYTES)(sp)
	FPLOAD f29, (20*REGBYTES + 17*FPREGBYTES)(sp)
	FPLOAD f30, (20*REGBYTES + 18*FPREGBYTES)(sp)
	FPLOAD f31, (20*REGBYTES + 19*FPREGBYTES)(sp)
  #endif
#endif

#ifdef __riscv_flen
  #if(__riscv_flen == 64 )
    addi sp, sp, 20*REGBYTES  + 20*FPREGBYTES
  #else
    addi sp, sp, 20*REGBYTES
  #endif
#else
// De-allocate the stack space
 addi sp, sp, 20*REGBYTES
#endif
.endm

/*
    Entry point of all programs (_start).

    It initializes DWARF call frame information, the stack pointer, the
    frame pointer (needed for closures to work in start_rust) and the global
    pointer. Then it calls _start_rust.
*/

// IRQ entry point
.section      .text.irq
.align 2
.global irq_entry
.weak irq_entry
irq_entry:
  // Allocate the stack space

  SAVE_CONTEXT// Save 16 regs

  //------This special CSR read operation, which is actually use mcause as operand to directly store it to memory
  csrrwi  x0, CSR_PUSHMCAUSE, 17
  //------This special CSR read operation, which is actually use mepc as operand to directly store it to memory
  csrrwi  x0, CSR_PUSHMEPC, 18
  //------This special CSR read operation, which is actually use Msubm as operand to directly store it to memory
  csrrwi  x0, CSR_PUSHMSUBM, 19

service_loop:
  //------This special CSR read/write operation, which is actually Claim the CLIC to find its pending highest
  // ID, if the ID is not 0, then automatically enable the mstatus.MIE, and jump to its vector-entry-label, and
  // update the link register
  csrrw ra, CSR_JALMNXTI, ra

  //RESTORE_CONTEXT_EXCPT_X5

  #---- Critical section with interrupts disabled -----------------------
  DISABLE_MIE # Disable interrupts

  LOAD x5,  19*REGBYTES(sp)
  csrw CSR_MSUBM, x5
  LOAD x5,  18*REGBYTES(sp)
  csrw CSR_MEPC, x5
  LOAD x5,  17*REGBYTES(sp)
  csrw CSR_MCAUSE, x5

  RESTORE_CONTEXT

  mret

.section .init, "ax"
    .weak  eclic_msip_handler
    .weak  eclic_mtip_handler
    .weak  eclic_bwei_handler
    .weak  eclic_pmovi_handler
    .weak  WWDGT
    .weak  LVD
    .weak  TAMPER
    .weak  RTC
    .weak  FMC
    .weak  RCU
    .weak  EXTI0
    .weak  EXTI1
    .weak  EXTI2
    .weak  EXTI3
    .weak  EXTI4
    .weak  DMA0_Channel0
    .weak  DMA0_Channel1
    .weak  DMA0_Channel2
    .weak  DMA0_Channel3
    .weak  DMA0_Channel4
    .weak  DMA0_Channel5
    .weak  DMA0_Channel6
    .weak  ADC0_1
    .weak  CAN0_TX
    .weak  CAN0_RX0
    .weak  CAN0_RX1
    .weak  CAN0_EWMC
    .weak  EXTI5_9
    .weak  TIMER0_BRK
    .weak  TIMER0_UP
    .weak  TIMER0_TRG_CMT
    .weak  TIMER0_Channel
    .weak  TIMER1
    .weak  TIMER2
    .weak  TIMER3
    .weak  I2C0_EV
    .weak  I2C0_ER
    .weak  I2C1_EV
    .weak  I2C1_ER
    .weak  SPI0
    .weak  SPI1
    .weak  USART0
    .weak  USART1
    .weak  USART2
    .weak  EXTI10_15
    .weak  RTC_Alarm
    .weak  USBFS_WKUP
    .weak  EXMC
    .weak  TIMER4
    .weak  SPI2
    .weak  UART3
    .weak  UART4
    .weak  TIMER5
    .weak  TIMER6
    .weak  DMA1_Channel0
    .weak  DMA1_Channel1
    .weak  DMA1_Channel2
    .weak  DMA1_Channel3
    .weak  DMA1_Channel4
    .weak  CAN1_TX
    .weak  CAN1_RX0
    .weak  CAN1_RX1
    .weak  CAN1_EWMC
    .weak  USBFS
vector_base:
    j _start
    .align    2
    .word     0
    .word     0
    .word     eclic_msip_handler
    .word     0
    .word     0
    .word  	  0
    .word  	  eclic_mtip_handler
    .word  	  0
    .word  	  0
    .word  	  0
    .word  	  0
    .word  	  0
    .word  	  0
    .word  	  0
    .word 	  0
    .word  	  0
    .word  	  eclic_bwei_handler
    .word  	  eclic_pmovi_handler
    .word  	  WWDGT
    .word     LVD
    .word     TAMPER
    .word     RTC
    .word     FMC
    .word     RCU
    .word     EXTI0
    .word     EXTI1
    .word     EXTI2
    .word     EXTI3
    .word     EXTI4
    .word     DMA0_Channel0
    .word     DMA0_Channel1
    .word     DMA0_Channel2
    .word     DMA0_Channel3
    .word     DMA0_Channel4
    .word     DMA0_Channel5
    .word     DMA0_Channel6
    .word     ADC0_1
    .word     CAN0_TX
    .word     CAN0_RX0
    .word     CAN0_RX1
    .word     CAN0_EWMC
    .word     EXTI5_9
    .word     TIMER0_BRK
    .word     TIMER0_UP
    .word     TIMER0_TRG_CMT
    .word     TIMER0_Channel
    .word     TIMER1
    .word     TIMER2
    .word     TIMER3
    .word     I2C0_EV
    .word     I2C0_ER
    .word     I2C1_EV
    .word     I2C1_ER
    .word     SPI0
    .word     SPI1
    .word     USART0
    .word     USART1
    .word     USART2
    .word     EXTI10_15
    .word     RTC_Alarm
    .word     USBFS_WKUP
    .word     0
    .word     0
    .word     0
    .word     0
    .word     0
    .word     EXMC
    .word     0
    .word     TIMER4
    .word     SPI2
    .word     UART3
    .word     UART4
    .word     TIMER5
    .word     TIMER6
    .word     DMA1_Channel0
    .word     DMA1_Channel1
    .word     DMA1_Channel2
    .word     DMA1_Channel3
    .word     DMA1_Channel4
    .word     0
    .word     0
    .word     CAN1_TX
    .word     CAN1_RX0
    .word     CAN1_RX1
    .word     CAN1_EWMC
    .word     USBFS
.global _start
_start:
    /* Jump to the absolute address defined by the linker script. */
    // for 32bit
    .if __riscv_xlen == 32
    lui ra, %hi(_abs_start)
    jr %lo(_abs_start)(ra)
    .endif

    // for 64bit
    .if __riscv_xlen == 64
1:
    auipc ra, %pcrel_hi(1f)
    ld ra, %pcrel_lo(1b)(ra)
    jr ra
    .align  3
1:
    .dword _abs_start
    .endif

.global _abs_start
_abs_start:
    .cfi_startproc
    .cfi_undefined ra

    csrw mie, 0
    csrw mip, 0

    li  x1, 0
    li  x2, 0
    li  x3, 0
    li  x4, 0
    li  x5, 0
    li  x6, 0
    li  x7, 0
    li  x8, 0
    li  x9, 0
    li  x10,0
    li  x11,0
    li  x12,0
    li  x13,0
    li  x14,0
    li  x15,0
    li  x16,0
    li  x17,0
    li  x18,0
    li  x19,0
    li  x20,0
    li  x21,0
    li  x22,0
    li  x23,0
    li  x24,0
    li  x25,0
    li  x26,0
    li  x27,0
    li  x28,0
    li  x29,0
    li  x30,0
    li  x31,0

    .option push
    .option norelax
    la gp, __global_pointer$
    .option pop

    // Check hart id
    csrr a2, mhartid
    lui t0, %hi(_max_hart_id)
    add t0, t0, %lo(_max_hart_id)
    bgtu a2, t0, abort

    // Allocate stacks
    la sp, _stack_start
    lui t0, %hi(_hart_stack_size)
    add t0, t0, %lo(_hart_stack_size)
#ifdef __riscv_mul
    mul t0, a2, t0
#else
    beqz a2, 2f  // Jump if single-hart
    mv t1, a2
    mv t2, t0
1:
    add t0, t0, t2
    addi t1, t1, -1
    bnez t1, 1b
2:
#endif
    sub sp, sp, t0

    // Set frame pointer
    add s0, sp, zero

    // Set the the NMI base to share with mtvec by setting CSR_MMISC_CTL
    li t0, 0x200
    csrs CSR_MMISC_CTL, t0

	// Set the mtvt
    la t0, vector_base
    csrw CSR_MTVT, t0

	// Set the mtvt2 and enable it
    la t0, irq_entry
    csrw CSR_MTVT2, t0
    csrs CSR_MTVT2, 0x1

    // Enable ECLIC and set trap handler
    la t0, _start_trap
    andi t0, t0, -64
    ori t0, t0, 3
    csrw mtvec, t0

    jal zero, _start_rust

    .cfi_endproc


/*
    Trap entry point (_start_trap)

    Saves caller saved registers ra, t0..6, a0..7, calls _start_trap_rust,
    restores caller saved registers and then returns.
*/
.section .trap, "ax"
.align 6
.global _start_trap

_start_trap:
    addi sp, sp, -16*REGBYTES

    STORE ra, 0*REGBYTES(sp)
    STORE t0, 1*REGBYTES(sp)
    STORE t1, 2*REGBYTES(sp)
    STORE t2, 3*REGBYTES(sp)
    STORE t3, 4*REGBYTES(sp)
    STORE t4, 5*REGBYTES(sp)
    STORE t5, 6*REGBYTES(sp)
    STORE t6, 7*REGBYTES(sp)
    STORE a0, 8*REGBYTES(sp)
    STORE a1, 9*REGBYTES(sp)
    STORE a2, 10*REGBYTES(sp)
    STORE a3, 11*REGBYTES(sp)
    STORE a4, 12*REGBYTES(sp)
    STORE a5, 13*REGBYTES(sp)
    STORE a6, 14*REGBYTES(sp)
    STORE a7, 15*REGBYTES(sp)

    add a0, sp, zero
    jal ra, _start_trap_rust

    LOAD ra, 0*REGBYTES(sp)
    LOAD t0, 1*REGBYTES(sp)
    LOAD t1, 2*REGBYTES(sp)
    LOAD t2, 3*REGBYTES(sp)
    LOAD t3, 4*REGBYTES(sp)
    LOAD t4, 5*REGBYTES(sp)
    LOAD t5, 6*REGBYTES(sp)
    LOAD t6, 7*REGBYTES(sp)
    LOAD a0, 8*REGBYTES(sp)
    LOAD a1, 9*REGBYTES(sp)
    LOAD a2, 10*REGBYTES(sp)
    LOAD a3, 11*REGBYTES(sp)
    LOAD a4, 12*REGBYTES(sp)
    LOAD a5, 13*REGBYTES(sp)
    LOAD a6, 14*REGBYTES(sp)
    LOAD a7, 15*REGBYTES(sp)

    addi sp, sp, 16*REGBYTES
    mret
