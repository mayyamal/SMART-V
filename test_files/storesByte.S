.text
 .globl _start
_start:

li a5, 0xfeca
li a4, 0x1111
li sp, 0xff8
sw a5, 8(sp)
sb a4, 8(sp)
nop
nop
nop
nop
nop


