 // Copyright 2021 TU Graz.

////////////////////////////////////////////////////////////////////////////////
// Author:         Maja Malenko - malenko.maya@gmail.com                      //
//                                                                            //
// Design Name:    SMART-V Data Memory                                        //
// Project Name:   SMART-V                                                    //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Data Memory (RAM) for byte RW access to data               //
//                 Based on Tobias Scheipel tsri5cy project implementation    //   
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module byte_data_memory 
#(
  //parameter ROM_FILE = "",
  parameter ADDR_WIDTH = 8,
  parameter DATA_WIDTH = 32
)
(
  input   logic  clk_i,

  input   logic                     en_i,
  input   logic                     wr_i,
  input   logic  [ADDR_WIDTH-1:0]   addr_i,
  input   logic  [DATA_WIDTH-1:0]   wdata_i,
  output  logic  [DATA_WIDTH-1:0]   rdata_o 
   
);

localparam SIZE = 2**ADDR_WIDTH;

(*ram_style = "block" *) reg [DATA_WIDTH-1:0] mem[0:SIZE-1];

logic [DATA_WIDTH-1:0] rdata;

/*
initial
begin
  if (ROM_FILE != "")
    $readmemh(ROM_FILE, mem);
end
*/
always @(posedge clk_i)
begin
  // port a
  if (en_i)
  begin
    if (wr_i)
		/*if(is_bitBanded_i) begin
			mem[addr_i] <= (mem[addr_i] & ~(p0_bbwmask_i)) | p0_bbwmask_i;
		end
		else begin	*/
			mem[addr_i] <= wdata_i;
		//end	
    rdata <= mem[addr_i];
  end 
end 

assign rdata_o = rdata;



endmodule
