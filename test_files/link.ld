OUTPUT_ARCH ("riscv")
ENTRY(_start)
SECTIONS
{

.text :
{
	. = 0x100;
	 *(.vector_user)
	 
	 . = 0x11c;
	*(.vector_timer_int)
	
	. = 0x140;
	*(.gpio_int)
	  
	. = 0x1C0; 
	*(.vector_machine)	
	
	. = 0x200;
	*(.start)
	
	

	*(.text)
}

.rodata :
{
	. = 0x1000;	
	*(.rodata)	
}

.data :
{
	/*. = 0x400000;	*/
	. = 0x4000;
	*(.data)
}

}