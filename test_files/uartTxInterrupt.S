.text
 .globl _start
_start:

#UART_CTRL =  BAUD_DIV << 16;
#0x00310003 => rxie = txie = 1

lui	a5,0x800
addi a5,a5,512 # 800200 -> UART_CTRL

#a4 = 0x1b1 0000 => baud rate
lui	a4,0x01 #MM: Changed the baud rate just for testing purposes
addi a4, a4, 3
sw	a4,0(a5)


#while(UART_STATUS & 0x0200);	
#0x200 = 001000000000 
#while (tx_full);

nop

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
nop
nop
nop



