////////////////////////////////////////////////////////////////////////////////
// Author:         Maja Malenko - malenko.maya@gmail.com                      //
//                                                                            //
// Design Name:    SMART-V Protection Unit                                    //
// Project Name:   SMART-V                                                    //
// Language:       RISC-V Assembly                                            //
//                                                                            //
// Description:    Simple program showing function call and return            //
//				   using jalr instruction                                     //
////////////////////////////////////////////////////////////////////////////////


.text
 .globl _start
_start:
	jal ra, fun	
	li a4, 9
	nop
	nop
	nop
	
fun:
	addi a5, x0, 7
	jalr x0, ra #return
	addi a5, x0, 15
	nop
	


