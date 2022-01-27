

.section .vector_user
	j	vector_u
.text


 .globl _start
_start:
	li t0, 0x1c0
	csrrw zero, mtvec, t0
	li t0, 0x100
	csrrw zero, 0x005, t0 #utvec
	la a5, user_mode
	#csrw 0x41, a5 #uepc
	csrw mepc, a5
	li a4, 25
	li a5, 0	
	csrw 0x322, a5
	csrw 0x321, a4 #write to mtimecmp
	li a4, 0x88
	csrw mie, a4
	li a4, 0xe
	csrw mstatus, a4

	#uret	
	mret
	li a6, 7
	li a7, 3
	add a6, a6, a7
	add a7, a7, a6
	nop
	nop

	#ecall
	
.globl vector_u	
vector_u:
	li  a4, 0x80
	csrc mip, a4	
	li a6, 7
	li a7, 3
	add a6, a6, a7
	add a7, a7, a6
	nop
	nop
	uret
	



.globl user_mode	
user_mode:
	
	nop	
	nop	
	nop
	nop
	nop
	nop	
	li a5, 0x0
	addi a5, a5, 0x3
	li a6, 0x7	
	nop	
	ecall



	
