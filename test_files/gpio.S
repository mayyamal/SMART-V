////////////////////////////////////////////////////////////////////////////////
// Author:         Maja Malenko - malenko.maya@gmail.com                      //
//                                                                            //
// Design Name:    SMART-V Protection Unit                                    //
// Project Name:   SMART-V                                                    //
// Language:       RISC-V Assembly                                            //
//                                                                            //
// Description:    Simple program demonstrating IN (switches)                 //
//				   and OUT (LEDs) ports of a                                  //
//				   memory-mapped LED peripheral                               //
////////////////////////////////////////////////////////////////////////////////

.text
 .globl _start
_start:


	#li a5, 0x18

	lui a6, %hi(8388864) #GPIO_OUT = 0x00800100 = 8,388,864 dec
	addi a6, a6, %lo(8388864) 
	#sw a5, 0(a6)

	lui a4, %hi(8388868) #GPIO_IN = 0x00800104 = 8,388,868 dec
	addi a4, a4, %lo(8388868) 

	li t0, 0
	loop:
	lw a5, 0(a4)
	sw a5, 0(a6)  
	beq x0, t0, loop

/*
li t0, 5
loop:
addi t0, t0, -1
bne x0, t0, loop

xor a5, a5, 0x01
sw a5, 0(a6)

li t0, 5

jal loop
*/



/*li a5, 0x37f
lui a6, %hi(8388876) #GPIO_IE = 0x80010c = 8399976dec
addi a6, a6, %lo(8388876)
sw a5, 0(a6)

lw a7,  0(a6)
nop
nop 
*/