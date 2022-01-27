 // Copyright 2021 TU Graz.

////////////////////////////////////////////////////////////////////////////////
// Author:         Maja Malenko - malenko.maya@gmail.com                      //
//                                                                            //
// Design Name:    SMART-V Peripheral Controller                              //
// Project Name:   SMART-V                                                    //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Peripheral Controller for on-chip peripherals              //
//                 Implements peripheral protection concepts                  //   
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
 
 `include "smartv_peripherals.vh"

module peripheral_controller 
#(
	parameter							PERIPHERAL_NO       = 2,                  
	parameter [31:0]					PERIPHERAL_OFFSET   = 32'h0080_0000
	 
	// GPIO_BASE: 0x00800100, UART_BASE: 0x00800200
)
(	
	input						data_req_i,
	input						data_we_i,             
	input [31:0]				data_addr_i, 
            
	output [PERIPHERAL_NO-1:0]	periph_en_o,	         
	output [31:0]				periph_addr_o,
	input logic [PERIPHERAL_NO-1:0]   periph_en_i`ifdef  SMARTV_EN ,
	input 				is_bitBanded_i,
	output [31:0]     	bb_addr_data 
	`endif
	//input 				bbError_i	

/*          
            input [PERIPHERAL_NO-1:0]                 en_per_mpu, 
            input    								  bitBandingEn,
            input [`XPR_LEN-1:0]   					  p0_bbwmask	 
*/			
);
  
logic gpio_en;
logic uart_en;


//logic  [32-1:0] bb_addr_data;
`ifdef  SMARTV_EN 
assign  bb_addr_data = (data_addr_i - `BITBAND_PERIPH_BASE)/32 + `PERIPH_BASE;
`endif
   //assign bus_en = en_data; 

localparam [31:0] PERIPHERALS_OFFSETS [PERIPHERAL_NO - 1:0] = {32'h0000_0200, 32'h0000_0100}; // {GPIO, UART}
localparam GPIO_BASE = PERIPHERAL_OFFSET + PERIPHERALS_OFFSETS[`GPIO_IDX];
localparam UART_BASE = PERIPHERAL_OFFSET + PERIPHERALS_OFFSETS[`UART_IDX];

always @(*)
begin
  gpio_en <= 1'b0;
  uart_en <= 1'b0;    
  
  // In conjuction with SmartOS memory layout
  `ifdef  SMARTV_EN 
  if ( (data_addr_i >= GPIO_BASE && data_addr_i < GPIO_BASE + `PERIPHERAL_SIZE) || (is_bitBanded_i && (bb_addr_data >= GPIO_BASE && bb_addr_data < GPIO_BASE + `PERIPHERAL_SIZE))) gpio_en  <= 1'b1; 
  `else
  if (data_addr_i >= GPIO_BASE && data_addr_i < GPIO_BASE + `PERIPHERAL_SIZE) gpio_en  <= 1'b1; 
  `endif// 64 32-bit regs
  if ( data_addr_i >= UART_BASE && data_addr_i < UART_BASE + `PERIPHERAL_SIZE) uart_en  <= 1'b1; // 64 32-bit regs
  
  
end
  
assign  periph_en_o = ({gpio_en, uart_en}) & periph_en_i;
`ifdef  SMARTV_EN 
assign  periph_addr_o = (is_bitBanded_i) ? bb_addr_data : data_addr_i;
`else
assign  periph_addr_o  = data_addr_i;
`endif
	
endmodule	
