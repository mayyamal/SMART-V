.text
 .globl _start
_start:
	
	
	la a5, user_mode
	csrw mepc, a5	
	
	
	li a5, 0 #generates an exception
	#li a5, 2; 
	csrw 0x3b6, a5  #`CSR_PMPADDR6
	
		
	li a5, 0x37f
	#li a6, 4080 #illegal address
	lui a6, %hi(8388868) #GPIO_OUT = 0x800104
	addi a6, a6, %lo(8388868)
	sw a5, 0(a6)
	
	
	#make gpio accessible to the task
	csrw 0x3b6, 0x1
	
	mret

	nop
	nop
	nop
	nop
	
	
.globl user_mode	
user_mode:
	
	lw a4, 0(a6) #should cause an exception
	li a5, 0xfff
	sw a5, 0(a6)
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
	
	
	
