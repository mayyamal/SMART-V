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
	csrw mepc, a5

	li a4, 0x08
	csrw mstatus, a4


	#MPU_CODE
	la a5, user_mode	
	csrw 0x3b0, a5  #`CSR_PMPADDR0
	
	li a5, 0x304
	csrw 0x3b1, a5 #`CSR_PMPADDR1

	#MPU_DATA
	lui a5, %hi(4194304)
	addi a5, a5, %lo(4194304)
	csrw 0x3b2, a5  #`CSR_PMPADDR2
	
	lui a5, %hi(4194312)
	addi a5, a5, %lo(4194312)
	csrw 0x3b3, a5 #`CSR_PMPADDR3
	
	li a5, 0x37f
	li a6, 4194312 #legal address
	sw a5, 0(a6)


	#lw t3, x
	
	#lw t4, y
	#add t5, t3, t4
	#lb t3, z
	
	mret

	nop
	nop
	nop
	nop
	

	
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
	mret
	nop
	nop
	nop
	



.globl user_mode	
user_mode:
	
	nop	
	nop	
	lw a4, 0(a6) #should NOT cause an exception


	la a5, vector_u
	jalr a5

	lui a6, %hi(4194316)
	addi a6, a6, %lo(4194316)

	sw a4, 0(a6) #SHOULD cause an exception


	nop
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
	nop
	nop
	nop
	nop
	
	
.data
x: .word 0x2
y: .word 0x5
z: .word 0x4