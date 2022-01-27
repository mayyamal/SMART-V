////////////////////////////////////////////////////////////////////////////////
// Author:         Maja Malenko - malenko.maya@gmail.com                      //
//                                                                            //
// Design Name:    SMART-V Protection Unit                                    //
// Project Name:   SMART-V                                                    //
// Language:       RISC-V Assembly                                            //
//                                                                            //
// Description:    Simple program showing 2 load instructions:                //
//				   load word and load byte                                    //
////////////////////////////////////////////////////////////////////////////////


.text
 .globl _start
_start:
	lw a4, x	
	lw a5, y
	add a3, a4, a5
	lb a4, z #a4 stores 0x23
	nop

.section .rodata 
x: .word 0x2
y: .word 0x5
z: .word 0x123

