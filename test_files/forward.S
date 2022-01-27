////////////////////////////////////////////////////////////////////////////////
// Author:         Maja Malenko - malenko.maya@gmail.com                      //
//                                                                            //
// Design Name:    SMART-V Protection Unit                                    //
// Project Name:   SMART-V                                                    //
// Language:       RISC-V Assembly                                            //
//                                                                            //
// Description:    Simple program showing forwarding of a5 from the           //
//                 first add to the sub                                       //   
//                 and load-use hazard which stalls the pipeline              //
////////////////////////////////////////////////////////////////////////////////



.text
 .globl _start
_start:
	li a5, 5
	li a4, 3	
	add a5, a5, a4
	sub a4, a5, a4 #a5 forwarded from EX	 
	nop
	lw a5, x
	add a4, a4, a5 #load-use data hazard
	nop

.section .rodata 
	x: .word 0x2
