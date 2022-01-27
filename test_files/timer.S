.text
 .globl _start
_start:

	rdcycleh a5 #, 0xb41 #mtimeH
	rdcycle a4  #, 0xb01  #mtime
	rdcycleh a6 #, 0xb41 #mtimeH
	
	beq a5, a5, exit
	rdcycle a4 #, 0xb01  #mtime
	nop
	nop
	nop
	nop

exit:
	li sp, 7
