////////////////////////////////////////////////////////////////////////////////
// Author:         Maja Malenko - malenko.maya@gmail.com                      //
//                                                                            //
// Design Name:    SMART-V Protection Unit                                    //
// Project Name:   SMART-V                                                    //
// Language:       RISC-V Assembly                                            //
//                                                                            //
// Description:    Simple program demonstrating gpio_in interrupt             //
//				                                                              //
//				                                                              //
////////////////////////////////////////////////////////////////////////////////
.section .gpio_int
	j	gpio_isr
	
	
.text
 .globl _start
_start:


	lui a6, %hi(8388864) #GPIO_OUT = 0x00800100 = 8,388,864 dec
	addi a6, a6, %lo(8388864) 	

	lui a4, %hi(8388868) #GPIO_IN = 0x00800104 = 8,388,868 dec
	addi a4, a4, %lo(8388868) 
	
	lui a5, %hi(8388872) #GPIO_IE = 0x00800108 = 8,388,872 dec
	addi a5, a5, %lo(8388872) 
	li t0, 1
	sw t0, 0(a5)

	li t0, 0
	loop:
	lw t1, 0(a4)
	sw t1, 0(a6)  
	beq x0, t0, loop


.globl gpio_isr
gpio_isr:
	li t0, 0
	sw t0, 0(a5)
	li t2, 7
	beq x0, t0, loop
	