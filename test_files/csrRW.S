////////////////////////////////////////////////////////////////////////////////
// Author:         Maja Malenko - malenko.maya@gmail.com                      //
//                                                                            //
// Design Name:    SMART-V Protection Unit                                    //
// Project Name:   SMART-V                                                    //
// Language:       RISC-V Assembly                                            //
//                                                                            //
// Description:    Simple program showing CSR operations                      //
////////////////////////////////////////////////////////////////////////////////


.text
 .globl _start
_start:	
	csrr a5, mstatus  #0x300
	csrr a4, 0xb01  #mtime	
	li t0, 0x100
	csrw mtvec, t0
	li t0, 0x200
	csrw 0x005, t0 #utvec 
	csrr a4, 0xb01  #mtime
	nop
	
