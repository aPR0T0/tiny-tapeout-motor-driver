/* Startup: stack at the top of on-chip RAM, copy .data, clear .bss, run main.
   Returning from main executes ECALL, which halts the core (cause 1). */
    .section .text.start
    .globl _start
_start:
    la    sp, _stack_top
    la    a0, _data_load
    la    a1, _data_start
    la    a2, _data_end
1:  bgeu  a1, a2, 2f
    lw    a3, 0(a0)
    sw    a3, 0(a1)
    addi  a0, a0, 4
    addi  a1, a1, 4
    j     1b
2:  la    a1, _bss_start
    la    a2, _bss_end
3:  bgeu  a1, a2, 4f
    sw    zero, 0(a1)
    addi  a1, a1, 4
    j     3b
4:  call  main
    ecall
5:  j     5b
