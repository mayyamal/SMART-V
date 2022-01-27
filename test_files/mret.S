////////////////////////////////////////////////////////////////////////////////
// Author:         Maja Malenko - malenko.maya@gmail.com                      //
//                                                                            //
// Design Name:    SMART-V Protection Unit                                    //
// Project Name:   SMART-V                                                    //
// Language:       RISC-V Assembly                                            //
//                                                                            //
// Description:    Simple program showing mret instruction                    //
////////////////////////////////////////////////////////////////////////////////

.section .vector_user #jump here after exception 
	j	vector_u


.section .vector_timer_int
	j	timer_isr
	
	
.section .gpio_int
	j	gpio_isr

	
.text	
 .globl _start
_start:
	li t0, 0x100
	csrw  mtvec, t0		
	la t0, user_mode #where to go after mret	 
	csrw mepc, t0
	
	#timer-related instructions:		
	li a4, 25
	li a5, 0		
	csrw 0x322, a5
	csrw 0x321, a4 #write to mtimecmp
	li a4, 0x80
	csrs mie, a4 #enable timer interrupt		
		
		
	mret #jump to the address in mepc
	
		
	nop
	
.globl user_mode	
user_mode:			
	ecall  #jump to the address in mtvec
	nop
		
.globl vector_u	
vector_u:			
	mret
	nop

.globl timer_isr
timer_isr:
	lui a4, %hi(8388868) #GPIO_IN = 0x00800104 = 8,388,868 dec
	addi a4, a4, %lo(8388868) 	
	lui a5, %hi(8388872) #GPIO_IE = 0x00800108 = 8,388,872 dec
	addi a5, a5, %lo(8388872) 
	li t0, 1
	sw t0, 0(a5)
	#li t0, 1
	sw t0, 0(a5)
	
	
	csrr t0, 0xb01  #mtime
	add t0, t0, 15
	csrw 0x321, t0 #add several cycles
	
	mret

.globl gpio_isr
gpio_isr:
	lui a5, %hi(8388872) #GPIO_IE = 0x00800108 = 8,388,872 dec
	addi a5, a5, %lo(8388872) 
	li t0, 0
	sw t0, 0(a5)
	#li t2, 7
	#beq x0, t0, loop	
	mret




	
