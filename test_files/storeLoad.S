.text
 .globl _start
_start:

lui sp, 0x1
lw a5, x
sw a5, 0(sp)
lw a4, 0(sp)
nop
nop
nop

.data
x: .word 0x3

