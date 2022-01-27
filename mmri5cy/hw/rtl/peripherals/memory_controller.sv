 // Copyright 2021 TU Graz.

////////////////////////////////////////////////////////////////////////////////
// Author:         Maja Malenko - malenko.maya@gmail.com                      //
//                                                                            //
// Design Name:    SMART-V Memory Controller                                  //
// Project Name:   SMART-V                                                    //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Memory Controller for ROM and RAM memories                 //
//                 Based on Tobias Scheipel tsri5cy Project implementation    //   
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
`include "smartv_peripherals.vh"
 
module memory_controller
#(
  parameter ROM_FILE = ""  
)
(
  // Clock and Reset
  input logic				clk_i,
  input logic				rst_n,  
  
  // instruction memory signals  
  input logic [31:0]		instr_addr_i,
  input logic				instr_req_i,
  output logic [31:0]		instr_rdata_o,
  output logic				instr_gnt_o,
  output logic				instr_rvalid_o,
  
  // data memory signals
  input logic [31:0]		data_addr_i,
  input logic [31:0]		data_wdata_i,
  input logic 				data_we_i,
  input logic 				data_req_i,
  input logic [3:0]			data_be_i,
  
  output logic [31:0]		data_rdata_o,
  output logic				data_gnt_o,
  output logic				data_rvalid_o, 
  
  /////// I/O signals forwarded to top  
  // LEDs
  output logic [15:0] 	leds_o, 
  output logic 			gpio_irq,
  // switches
  input logic [15:0] 	switches_i,  
  // UART
  output logic 			uart_tx_o,
  input logic [2-1:0]   periph_en_i `ifdef  SMARTV_EN ,
  input 				is_bitBanded_i,
  input [31:0]			p0_bbwmask_i
  `endif
  //input 				bbError_i	 
);

logic			rom_en;
logic			data_rom_gnt; 
logic			data_rom_rvalid;
logic [31:0]	data_rom_rdata;



logic			ram_en;
logic			data_ram_gnt; 
logic			data_ram_rvalid;
logic [31:0]	data_ram_rdata;


logic			gpio_en;
logic			data_gpio_gnt;
logic			data_gpio_rvalid;
logic [31:0]	data_gpio_rdata;
//logic 			gpio_irq;

logic			uart_en;
logic			data_uart_gnt;
logic			data_uart_rvalid;
logic [31:0]	data_uart_rdata;



localparam ROM_OFFSET = 32'h0000_0000;
localparam RAM_OFFSET = 32'h0040_0000;
localparam PERIPHERAL_NO = 2;
localparam PERIPHERAL_OFFSET = 32'h0080_0000;




// multiplexer for selecting peripheral
always @(*)
begin
  rom_en <= 1'b0;
  ram_en <= 1'b0;  
  
  
  // In conjuction with SmartOS memory layout
  if ( data_addr_i >= ROM_OFFSET && data_addr_i < RAM_OFFSET) rom_en  <= 1'b1; // 4MB
  if ( data_addr_i >= RAM_OFFSET && data_addr_i < PERIPHERAL_OFFSET) ram_en  <= 1'b1; // 4MB

end


// demultiplexer for read data
always @(*)
begin
  data_rdata_o = 0;
  if (data_rom_rvalid)  data_rdata_o = data_rom_rdata;
  if (data_ram_rvalid)  data_rdata_o = data_ram_rdata;
  if (data_uart_rvalid) data_rdata_o = data_uart_rdata;
  if (data_gpio_rvalid) data_rdata_o = data_gpio_rdata;
end

assign data_gnt_o = 	data_rom_gnt  | 
						data_ram_gnt  |
						data_uart_gnt |
						data_gpio_gnt;

assign data_rvalid_o =  	data_rom_rvalid   | 
							data_ram_rvalid   |
							data_gpio_rvalid  |
							data_uart_rvalid;

// Instruction Memory Instance

instruction_memory
#(
	.ROM_FILE ( ROM_FILE ),
	.MEM_SIZE( 32768 ),
	.DATA_WIDTH ( 32 )
)
instruction_memory_dut
(
	.clk_i ( clk_i ),
	.instr_req_i     ( instr_req_i       ),
	.instr_gnt_o     ( instr_gnt_o       ),
	.instr_rvalid_o  ( instr_rvalid_o    ),
	.instr_addr_i    ( instr_addr_i      ),
	.instr_rdata_o   ( instr_rdata_o     ),

	.data_en_i       ( rom_en          ),
	.data_req_i      ( data_req_i      ),
	.data_gnt_o      ( data_rom_gnt    ),
	.data_rvalid_o   ( data_rom_rvalid ),
	.data_addr_i     ( data_addr_i     ),
	.data_rdata_o    ( data_rom_rdata  )	
);

data_memory 
#( //	.ROM_FILE ( ), //MM: Remove this, only for the simple testbenches!!!
	.MEM_SIZE( 32768 ),
	.DATA_WIDTH ( 32 )
)
data_memory_dut
(
	.clk_i         ( clk_i           ) ,	
	.data_en_i     ( ram_en          ) ,	
	.data_req_i    ( data_req_i      ) ,
	.data_gnt_o    ( data_ram_gnt    ) ,
	.data_rvalid_o ( data_ram_rvalid ) ,
	.data_rdata_o  ( data_ram_rdata  ) ,

	.data_we_i     ( data_we_i       ) ,
	.data_be_i     ( data_be_i       ) ,
	.data_addr_i   ( data_addr_i     ) ,
	.data_wdata_i  ( data_wdata_i    ) 
	
    	
);

////////////////////////////////////////////////////////////////////////
// ___ ___ ___ ___ ___ _  _ ___ ___    _   _       ___ _____ ___ _    // 
//| _ \ __| _ \_ _| _ \ || | __| _ \  /_\ | |     / __|_   _| _ \ |   //  
//|  _/ _||   /| ||  _/ __ | _||   / / _ \| |__  | (__  | | |   / |__ // 
//|_| |___|_|_\___|_| |_||_|___|_|_\/_/ \_\____|  \___| |_| |_|_\____|//
////////////////////////////////////////////////////////////////////////
logic [31:0] periph_addr;


peripheral_controller 
#(
	.PERIPHERAL_NO             (PERIPHERAL_NO),   	 
	.PERIPHERAL_OFFSET         (PERIPHERAL_OFFSET)	
) 
peripheral_controller_dut 
(
	.data_req_i		(data_req_i),
    .data_we_i		(data_we_i), 
    .data_addr_i	(data_addr_i),  
	.periph_en_o	({gpio_en, uart_en}),	           
	.periph_addr_o	(periph_addr),
	.periph_en_i    (periph_en_i) `ifdef  SMARTV_EN ,
	.is_bitBanded_i (is_bitBanded_i),
	.bb_addr_data   (bb_addr_data)	
	`endif
  	//.bbError_i		(bbError_i)
);


gpio gpio_0
(
	.clk_i         ( clk_i             ),
	.rstn_i        ( rst_n             ),
	.en_i          ( gpio_en           ),

	.data_req_i    ( data_req_i        ),
	.data_gnt_o    ( data_gpio_gnt     ),
	.data_rvalid_o ( data_gpio_rvalid  ),
	.data_rdata_o  ( data_gpio_rdata   ),
	
	.data_we_i     ( data_we_i         ),
	.data_be_i     ( data_be_i         ),
	.data_addr_i   ( periph_addr[`GPIO_REG_WIDTH-1:0] ),
	.data_wdata_i  ( data_wdata_i      ),

	.leds_o        ( leds_o            ),
	.switches_i    ( switches_i        ),
	.gpio_irq      ( gpio_irq		   )`ifdef  SMARTV_EN ,
	.p0_bbwmask_i  ( p0_bbwmask_i),
	.is_bitBanded_i (is_bitBanded_i),
	.bb_addr_data   (bb_addr_data)	
	`endif
);

uart_wrap uart_0
(
	.clk_i         ( clk_i            ),
	.rstn_i        ( rst_n            ),
	.en_i          ( uart_en          ),

	.data_req_i    ( data_req_i       ),
	.data_gnt_o    ( data_uart_gnt    ),
	.data_rvalid_o ( data_uart_rvalid ),
	.data_rdata_o  ( data_uart_rdata  ),

	.data_we_i     ( data_we_i        ),
	.data_be_i     ( data_be_i        ),
	.data_addr_i   ( periph_addr[`UART_REG_WIDTH-1:0] ),
	.data_wdata_i  ( data_wdata_i     ),
	
	.tx_o          ( uart_tx_o        ),
	.rx_i          (                  )

);

endmodule