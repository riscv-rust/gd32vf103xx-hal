.section .text, "ax"
.global _mp_hook
_mp_hook:
    la      a0, _mp_hook
    li      a1, 1
    slli    a1, a1, 29  // 0x20000000
    bleu    a1, a0, 1f  // jump if running from RAM
    srli    a1, a1, 2   // 0x08000000
    bleu    a1, a0, 1f  // jump if running from flash
    jr      a1  // jump to the start of flash

1:
    // return true
    li  a0, 1
    ret
