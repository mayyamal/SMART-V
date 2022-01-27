.text
 .globl _start
_start:

lui sp, 0x1
lw a5, x
sw a5, 0(sp)
lw a4, 0(sp)

lui a6, %hi(8388868) #GPIO_OUT = 0x800104
addi a6, a6, %lo(8388868)
sw a5, 0(a6)
nop
nop
nop

.globl user_mode	
user_mode:
	
	lw a7, 0(a6) 
	nop
	nop
	nop
	
.globl vector_user	
vector_user:
	li  a6, 0xfff
	nop
	nop
	nop
	nop	
	mret	
	

.data
x: .word 0x3
