////////////////////////////////////////////////////////////////////////////////
// Author:         Maja Malenko - malenko.maya@gmail.com                      //
//                                                                            //
// Design Name:    SMART-V Protection Unit                                    //
// Project Name:   SMART-V                                                    //
// Language:       RISC-V Assembly                                            //
//                                                                            //
// Description:    Simple program demonstrating configuring and sending       //
//		           a character over a memory-mapped UART peripheral           //
////////////////////////////////////////////////////////////////////////////////

.text
 .globl _start
_start:

	#UART_CTRL =  BAUD_DIV << 16;
	lui	a5,0x800
	addi a5,a5,512 # 800200 -> UART_CTRL

	#0x1b10  => baud rate
	lui	a4,0x1b10
	sw	a4,0(a5)


	#while(UART_STATUS & 0x0200);	
	#0x200 = 001000000000 
	#while (tx_full);


	loop: lui a5,0x800
	addi a5,a5,516 # 800204 -> 

	lw	a5,0(a5)
	andi a5,a5,512 #512 = 0x200
	bnez a5,loop

	# char c = 'M';
	# UART_DATA = (unsigned char)c;

	lui	a5,0x800
	addi a5,a5,520 # 800208 -> UART_DATA

	li	a4,77 #77 = 'M' = 0x4d
	sw	a4,0(a5)




