.text
 .globl _start
_start:

	li a4, 0xffff	
	csrr a5, 0x300  #mstatus	
	nop
	nop
	nop	
	csrw 0x300, a4
	nop
	nop
	nop
	nop

