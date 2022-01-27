`ifndef _vscale_peripherals_vh_
`define _vscale_peripherals_vh_



`define PERIPHERAL_SIZE          32'h00000100

///////////////////////
//  ___ ___ ___ ___  //
// / __| _ \_ _/ _ \ //
//| (_ |  _/| | (_) |//
// \___|_| |___\___/ //
///////////////////////

`define GPIO_REG_WIDTH           5
`define GPIO_REG_IN		         `GPIO_REG_WIDTH'b00000
`define GPIO_REG_OUT	         `GPIO_REG_WIDTH'b00100
`define GPIO_REG_DIR	         `GPIO_REG_WIDTH'b01000
`define GPIO_REG_IE		         `GPIO_REG_WIDTH'b01100
`define GPIO_REG_ICFG	         `GPIO_REG_WIDTH'b10000
`define GPIO_REG_IS		         `GPIO_REG_WIDTH'b10100
`define GPIO_REG_LAST            `GPIO_REG_IS

//////////////////////////
// _   _  _   ___ _____ //
//| | | |/_\ | _ \_   _|//
//| |_| / _ \|   / | |  //
// \___/_/ \_\_|_\ |_|  //
//////////////////////////                      

`define GPIO_BASE			     32'h00800100
`define GPIO_IDX                 5'h0

`define UART_REG_WIDTH           4
`define UART_REG_CTRL	         `UART_REG_WIDTH'b0000
`define UART_REG_STATUS	         `UART_REG_WIDTH'b0100
`define UART_REG_DATA	         `UART_REG_WIDTH'b1000
`define UART_REG_LAST            `UART_REG_DATA


`define UART_BASE			     32'h00800200
`define UART_IDX                 5'h1


////////////////////////////////////////////////////////
// ___ ___ _____    ___   _   _  _ ___ ___ _  _  ___  //
//| _ )_ _|_   _|__| _ ) /_\ | \| |   \_ _| \| |/ __| //
//| _ \| |  | ||___| _ \/ _ \| .` | |) | || .` | (_ | // 
//|___/___| |_|    |___/_/ \_\_|\_|___/___|_|\_|\___| //                 
////////////////////////////////////////////////////////  

`define BITBAND_SMARTV_REGS 2

`define BITFIELD_INDEX 5

// BITBAND_PERI_BASE is the base address of bit-band alias region for peripherals.


`define PERIPH_BASE 32'h00800100  //PERI_BASE is the base address of bit-band region for peripherals.

`define BITBAND_PERIPH_BASE 32'h00800300  //BITBAND_PERI_BASE is the base address of bit-band alias region for peripherals.
 
`define BITBAND_PERIPH_LAST (`BITBAND_PERIPH_BASE + (32*4*`BITBAND_SMARTV_REGS))




`endif