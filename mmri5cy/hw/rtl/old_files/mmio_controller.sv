/*****************************************************************************************
 *          Copyright Notice and Disclaimer of the tsri5cy Project
 *        Copyright (c) 2017-2020 Tobias Scheipel. All rights reserved.
 *
 * THIS FILE WAS RELEASED FOR EDUCATIONAL USE WITHIN
 *    a) THE INSTITUTE OF TECHNICAL INFORMATICS, GRAZ UNIVERSITY OF TECHNOLOGY, AUSTRIA
 * DISTRIBUTION IN SOURCE AND BINARY FORM OF THE RELATED SOFTWARE IS PROHIBITED.
 * THIS COPYRIGHT MAY NOT BE REMOVED, MODIFIED OR RELOCATED WITHIN THIS FILE.
 *****************************************************************************************/
 
 
module mmio_controller
#(
  parameter ROM_FILE = ""  
)
(
  // Clock and Reset
  input logic				clk_i,
  input logic				rst_n,
  
  // data signals
  input logic [31:0]		data_addr_i,
  input logic [31:0]		data_wdata_i,
  input logic 				data_we_i,
  input logic 				data_req_i,
  input logic [3:0]			data_be_i,
  
  output logic [31:0]		data_rdata_o,
  output logic				data_gnt_o,
  output logic				data_rvalid_o,
  
  // instruction signals  
  input logic [31:0]		instr_addr_i,
  input logic				instr_req_i,
  output logic [31:0]		instr_rdata_o,
  output logic				instr_gnt_o,
  output logic				instr_rvalid_o,
  
  
  /////// I/O signals forwarded to top
  
  // LEDs
  output logic [15:0] 	leds_o,
  
  // switches
  input logic [15:0] 	switches_i,
  
  // UART
  output logic 			uart_tx_o
  
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

logic			uart_en;
logic			data_uart_gnt;
logic			data_uart_rvalid;
logic [31:0]	data_uart_rdata;


// multiplexer for selecting peripheral
always @(*)
begin
  rom_en <= 1'b0;
  ram_en <= 1'b0; //MM :Change this later!!!!
  gpio_en <= 1'b0;
  uart_en <= 1'b0;
  
  
  // memory mapped I/O regions
  if ( data_addr_i >= 'h0000_0000 && data_addr_i < 'h0040_0000) rom_en  <= 1'b1;
  if ( data_addr_i >= 'h0040_0000 && data_addr_i < 'h8000_0000) ram_en  <= 1'b1;
  if ( data_addr_i >= 'h8000_0000 && data_addr_i < 'h8000_FFFF) gpio_en <= 1'b1;
  if ( data_addr_i >= 'hC000_0000 && data_addr_i < 'hC000_FFFF) uart_en <= 1'b1;

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

// PERIPHERALS INSTANCES
rom
#(
	.ROM_FILE ( ROM_FILE ),
	.MEM_SIZE( 32768 )
)
rom_dut
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

ram 
#(
	.MEM_SIZE( 32768 )
)
ram_inst
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

gpio gpio_inst
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
	.data_addr_i   ( data_addr_i[11:0] ),
	.data_wdata_i  ( data_wdata_i      ),

	.leds_o        ( leds_o            ),
	.switches_i    ( switches_i        )
);

uart_wrap uart_wrap_inst
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
	.data_addr_i   ( data_addr_i[3:0] ),
	.data_wdata_i  ( data_wdata_i     ),
	
	.tx_o          ( uart_tx_o        ),
	.rx_i          (                  )

);

endmodule