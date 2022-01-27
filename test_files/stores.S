////////////////////////////////////////////////////////////////////////////////
// Author:         Maja Malenko - malenko.maya@gmail.com                      //
//                                                                            //
// Design Name:    SMART-V Protection Unit                                    //
// Project Name:   SMART-V                                                    //
// Language:       RISC-V Assembly                                            //
//                                                                            //
// Description:    Simple program showing 2 load after 2 store instructions   //
////////////////////////////////////////////////////////////////////////////////


.text
 .globl _start
_start:
	li a5, 2
	li a4, 3
	la a3, x	
	sw a5, 0(a3)
	sw a4, 4(a3)
	lw a5, 4(a3)
	lw a4, 0(a3)
	nop

.data 
x: .word 
