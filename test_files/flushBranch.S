////////////////////////////////////////////////////////////////////////////////
// Author:         Maja Malenko - malenko.maya@gmail.com                      //
//                                                                            //
// Design Name:    SMART-V Protection Unit                                    //
// Project Name:   SMART-V                                                    //
// Language:       RISC-V Assembly                                            //
//                                                                            //
// Description:    Simple program showing pipeline flush when the branch      //
//				   is taken                                                   //
////////////////////////////////////////////////////////////////////////////////

.text
 .globl _start
_start:
	li a5, 0	
	beq x0, a5, success #branch taken
	addi a4, x0, 3
	addi a4, x0, 5
	addi a4, x0, 7	
	
success:
	addi a5, x0, 15
	beq x0, a5, success #branch not taken
	nop
	
	

