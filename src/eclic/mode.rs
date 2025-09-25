#![allow(named_asm_labels)]

use core::arch::global_asm;

global_asm!(
    ".equ LOG_REGBYTES, 2",
    ".equ REGBYTES, 1 << LOG_REGBYTES",
    ".equ MSTATUS_MIE, 0x00000008",
    //
    ".macro DISABLE_MIE",
        "csrc mstatus, MSTATUS_MIE",
    ".endm",
    //
    ".macro SAVE_CONTEXT",
        //
        "addi sp, sp, -20*REGBYTES",
        //
        "sw x1, 0*REGBYTES(sp)",
        "sw x4, 1*REGBYTES(sp)",
        "sw x5, 2*REGBYTES(sp)",
        "sw x6, 3*REGBYTES(sp)",
        "sw x7, 4*REGBYTES(sp)",
        "sw x10, 5*REGBYTES(sp)",
        "sw x11, 6*REGBYTES(sp)",
        "sw x12, 7*REGBYTES(sp)",
        "sw x13, 8*REGBYTES(sp)",
        "sw x14, 9*REGBYTES(sp)",
        "sw x15, 10*REGBYTES(sp)",
        "sw x16, 11*REGBYTES(sp)",
        "sw x17, 12*REGBYTES(sp)",
        "sw x28, 13*REGBYTES(sp)",
        "sw x29, 14*REGBYTES(sp)",
        "sw x30, 15*REGBYTES(sp)",
        "sw x31, 16*REGBYTES(sp)",
    ".endm",
    //
    ".macro RESTORE_CONTEXT",
        "lw x1, 0*REGBYTES(sp)",
        "lw x4, 1*REGBYTES(sp)",
        "lw x5, 2*REGBYTES(sp)",
        "lw x6, 3*REGBYTES(sp)",
        "lw x7, 4*REGBYTES(sp)",
        "lw x10, 5*REGBYTES(sp)",
        "lw x11, 6*REGBYTES(sp)",
        "lw x12, 7*REGBYTES(sp)",
        "lw x13, 8*REGBYTES(sp)",
        "lw x14, 9*REGBYTES(sp)",
        "lw x15, 10*REGBYTES(sp)",
        "lw x16, 11*REGBYTES(sp)",
        "lw x17, 12*REGBYTES(sp)",
        "lw x28, 13*REGBYTES(sp)",
        "lw x29, 14*REGBYTES(sp)",
        "lw x30, 15*REGBYTES(sp)",
        "lw x31, 16*REGBYTES(sp)",
        // De-allocate the stack space
        "addi sp, sp, 20*REGBYTES",
    ".endm",
    
    //
    ".section .text.irq",
    ".option push",
    ".option norelax",
    ".align 2",
    ".option pop",
    ".global _irq_handler",
    "_irq_handler:",
        "SAVE_CONTEXT",
        //
        // The special CSR read operation, which actually uses mcause as operand
        // to directly store it to memory
        "csrrwi x0, 0x7ee, 17",
        // The special CSR read operation, which actually uses mepc as operand
        // to directly store it to memory
        "csrrwi x0, 0x7ef, 18",
        // The special CSR read operation, which actually uses msubm as operand
        // to directly store it to memory
        "csrrwi x0, 0x7eb, 19",
        //
        // The special CSR read/write operation, which is actually Claim the CLIC to
        // find its pending highest ID, if the ID is not 0, then automatically enable
        // the mstatus.MIE, and jump to its vector-entry-label, and update the link register.
        "csrrw ra, 0x7ed, ra",
        //
        "DISABLE_MIE",
        //
        "lw x5, 19*REGBYTES(sp)",
        // Load x5 value into MSUBM system status register
        "csrw 0x7c4, x5",
        "lw x5, 18*REGBYTES(sp)",
        "csrw mepc, x5",
        "lw x5, 17*REGBYTES(sp)",
        "csrw mcause, x5",
        //
        "RESTORE_CONTEXT",
        //
        "mret",
    //
        
    ".section .trap, \"ax\"",
    ".option push",
    ".option norelax",
    ".align 6",
    ".option pop",
    ".global _start_trap",
    "_start_trap:",
        "addi sp, sp, -16*REGBYTES",
        //
        "sw ra, 0*REGBYTES(sp)",
        "sw t0, 1*REGBYTES(sp)",
        "sw t1, 2*REGBYTES(sp)",
        "sw t2, 3*REGBYTES(sp)",
        "sw t3, 4*REGBYTES(sp)",
        "sw t4, 5*REGBYTES(sp)",
        "sw t5, 6*REGBYTES(sp)",
        "sw t6, 7*REGBYTES(sp)",
        "sw a0, 8*REGBYTES(sp)",
        "sw a1, 9*REGBYTES(sp)",
        "sw a2, 10*REGBYTES(sp)",
        "sw a3, 11*REGBYTES(sp)",
        "sw a4, 12*REGBYTES(sp)",
        "sw a5, 13*REGBYTES(sp)",
        "sw a6, 14*REGBYTES(sp)",
        "sw a7, 15*REGBYTES(sp)",
        //
        "add a0, sp, zero",
        "jal ra, _start_trap_rust",
        //
        "lw ra, 0*REGBYTES(sp)",
        "lw t0, 1*REGBYTES(sp)",
        "lw t1, 2*REGBYTES(sp)",
        "lw t2, 3*REGBYTES(sp)",
        "lw t3, 4*REGBYTES(sp)",
        "lw t4, 5*REGBYTES(sp)",
        "lw t5, 6*REGBYTES(sp)",
        "lw t6, 7*REGBYTES(sp)",
        "lw a0, 8*REGBYTES(sp)",
        "lw a1, 9*REGBYTES(sp)",
        "lw a2, 10*REGBYTES(sp)",
        "lw a3, 11*REGBYTES(sp)",
        "lw a4, 12*REGBYTES(sp)",
        "lw a5, 13*REGBYTES(sp)",
        "lw a6, 14*REGBYTES(sp)",
        "lw a7, 15*REGBYTES(sp)",
        //
        "addi sp, sp, 16*REGBYTES",
        "mret",
    //
        
    ".section .text",
    ".global _setup_interrupts",
    "_setup_interrupts:",
        // Set the the NMI base to share with mtvec by setting CSR_MMISC_CTL
        "li t0, 0x200",
        "csrs 0x7d0, t0",
        //
        // Set the mtvt
        "la t0, vectors",
        "csrw 0x307, t0",
        //
        // Set the mtvt2 and enable it
        "la t0, _irq_handler",
        "csrw 0x7ec, t0",
        "csrs 0x7ec, 0x1",
        //
        // Enable ECLIC and set trap handler
        "la t0, _start_trap",
        "andi t0, t0, -64",
        "ori t0, t0, 3",
        "csrw mtvec, t0",
        //
        "ret",
    options(raw),
);

unsafe extern "C" {
    fn INT_SFT();
    fn INT_TMR();
    fn INT_BWEI();
    fn INT_PMOVI();
    fn WWDGT();
    fn EXTI_LVD();
    fn TAMPER();
    fn RTC();
    fn FMC();
    fn RCU();
    fn EXTI_LINE0();
    fn EXTI_LINE1();
    fn EXTI_LINE2();
    fn EXTI_LINE3();
    fn EXTI_LINE4();
    fn DMA0_CHANNEL0();
    fn DMA0_CHANNEL1();
    fn DMA0_CHANNEL2();
    fn DMA0_CHANNEL3();
    fn DMA0_CHANNEL4();
    fn DMA0_CHANNEL5();
    fn DMA0_CHANNEL6();
    fn ADC0_1();
    fn CAN0_TX();
    fn CAN0_RX0();
    fn CAN0_RX1();
    fn CAN0_EWMC();
    fn EXTI_LINE9_5();
    fn TIMER0_BRK();
    fn TIMER0_UP();
    fn TIMER0_TRG_CMT();
    fn TIMER0_CHANNEL();
    fn TIMER1();
    fn TIMER2();
    fn TIMER3();
    fn I2C0_EV();
    fn I2C0_ER();
    fn I2C1_EV();
    fn I2C1_ER();
    fn SPI0();
    fn SPI1();
    fn USART0();
    fn USART1();
    fn USART2();
    fn EXTI_LINE15_10();
    fn RTC_ALARM();
    fn USBFS_WKUP();
    fn EXMC(); // not present in Reference Manual but present in vendor HAL
    fn TIMER4();
    fn SPI2();
    fn UART3();
    fn UART4();
    fn TIMER5();
    fn TIMER6();
    fn DMA1_CHANNEL0();
    fn DMA1_CHANNEL1();
    fn DMA1_CHANNEL2();
    fn DMA1_CHANNEL3();
    fn DMA1_CHANNEL4();
    fn CAN1_TX();
    fn CAN1_RX0();
    fn CAN1_RX1();
    fn CAN1_EWMC();
    fn USBFS();
}
#[doc(hidden)]
#[repr(C)]
pub union Vector {
    _handler: unsafe extern "C" fn(),
    _reserved: u32,
}

#[doc(hidden)]
#[repr(C)]
#[repr(align(512))]
pub struct Align512<T>(T);

#[doc(hidden)]
#[unsafe(link_section = ".text.vectors")]
#[unsafe(no_mangle)]
pub static vectors: Align512<[Vector; 87]> = Align512([
        Vector { _reserved: 0 },
        Vector { _reserved: 0 },
        Vector { _reserved: 0 },
        Vector { _handler: INT_SFT },
        Vector { _reserved: 0 },
        Vector { _reserved: 0 },
        Vector { _reserved: 0 },
        Vector { _handler: INT_TMR },
        Vector { _reserved: 0 },
        Vector { _reserved: 0 },
        Vector { _reserved: 0 },
        Vector { _reserved: 0 },
        Vector { _reserved: 0 },
        Vector { _reserved: 0 },
        Vector { _reserved: 0 },
        Vector { _reserved: 0 },
        Vector { _reserved: 0 },
        Vector { _handler: INT_BWEI },
        Vector { _handler: INT_PMOVI },
        Vector { _handler: WWDGT },
        Vector { _handler: EXTI_LVD },
        Vector { _handler: TAMPER },
        Vector { _handler: RTC },
        Vector { _handler: FMC },
        Vector { _handler: RCU },
        Vector { _handler: EXTI_LINE0 },
        Vector { _handler: EXTI_LINE1 },
        Vector { _handler: EXTI_LINE2 },
        Vector { _handler: EXTI_LINE3 },
        Vector { _handler: EXTI_LINE4 },
        Vector { _handler: DMA0_CHANNEL0 },
        Vector { _handler: DMA0_CHANNEL1 },
        Vector { _handler: DMA0_CHANNEL2 },
        Vector { _handler: DMA0_CHANNEL3 },
        Vector { _handler: DMA0_CHANNEL4 },
        Vector { _handler: DMA0_CHANNEL5 },
        Vector { _handler: DMA0_CHANNEL6 },
        Vector { _handler: ADC0_1 },
        Vector { _handler: CAN0_TX },
        Vector { _handler: CAN0_RX0 },
        Vector { _handler: CAN0_RX1 },
        Vector { _handler: CAN0_EWMC },
        Vector { _handler: EXTI_LINE9_5 },
        Vector { _handler: TIMER0_BRK },
        Vector { _handler: TIMER0_UP },
        Vector { _handler: TIMER0_TRG_CMT },
        Vector { _handler: TIMER0_CHANNEL },
        Vector { _handler: TIMER1 },
        Vector { _handler: TIMER2 },
        Vector { _handler: TIMER3 },
        Vector { _handler: I2C0_EV },
        Vector { _handler: I2C0_ER },
        Vector { _handler: I2C1_EV },
        Vector { _handler: I2C1_ER },
        Vector { _handler: SPI0 },
        Vector { _handler: SPI1 },
        Vector { _handler: USART0 },
        Vector { _handler: USART1 },
        Vector { _handler: USART2 },
        Vector { _handler: EXTI_LINE15_10 },
        Vector { _handler: RTC_ALARM },
        Vector { _handler: USBFS_WKUP },
        Vector { _reserved: 0 },
        Vector { _reserved: 0 },
        Vector { _reserved: 0 },
        Vector { _reserved: 0 },
        Vector { _reserved: 0 },
        Vector { _handler: EXMC }, // not present in Reference Manual but present in vendor HAL
        Vector { _reserved: 0 },
        Vector { _handler: TIMER4 },
        Vector { _handler: SPI2 },
        Vector { _handler: UART3 },
        Vector { _handler: UART4 },
        Vector { _handler: TIMER5 },
        Vector { _handler: TIMER6 },
        Vector { _handler: DMA1_CHANNEL0 },
        Vector { _handler: DMA1_CHANNEL1 },
        Vector { _handler: DMA1_CHANNEL2 },
        Vector { _handler: DMA1_CHANNEL3 },
        Vector { _handler: DMA1_CHANNEL4 },
        Vector { _reserved: 0 },
        Vector { _reserved: 0 },
        Vector { _handler: CAN1_TX },
        Vector { _handler: CAN1_RX0 },
        Vector { _handler: CAN1_RX1 },
        Vector { _handler: CAN1_EWMC },
        Vector { _handler: USBFS },
]);