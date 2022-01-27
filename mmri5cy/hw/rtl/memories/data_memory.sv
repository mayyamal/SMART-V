 // Copyright 2021 TU Graz.

////////////////////////////////////////////////////////////////////////////////
// Author:         Maja Malenko - malenko.maya@gmail.com                      //
//                                                                            //
// Design Name:    SMART-V Data Memory                                        //
// Project Name:   SMART-V                                                    //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Data Memory (RAM) for RW access to data                    //
//                 Based on Tobias Scheipel tsri5cy project implementation    //   
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


module data_memory 
#(
  //parameter ROM_FILE = "",	
  parameter MEM_SIZE = 4096,
  parameter DATA_WIDTH = 32
)
(
  input   logic                   clk_i,
  input   logic                   data_en_i,
  input   logic                   data_req_i,
  output  logic                   data_gnt_o,
  output  logic                   data_rvalid_o,
  input   logic                   data_we_i,
  input   logic [3:0]             data_be_i,
  input   logic [31:0]            data_addr_i,
  input   logic [31:0]            data_wdata_i,
  output  logic [31:0]            data_rdata_o
    
);

	localparam ADDR_WIDTH = $clog2(MEM_SIZE/4);	 
	logic [3:0] data_wr;
	logic [ADDR_WIDTH-1:0] data_addr;

	logic [31:0] dbg_ram[0:MEM_SIZE/4-1];
	
	logic data_en;
	
assign data_en = data_en_i & data_req_i;


//assign data_addr = data_addr_i[ADDR_WIDTH-1+2:2];

assign data_addr = data_addr_i[ADDR_WIDTH-1+2:2];


genvar i;
generate
  for (i = 0; i < 4; i++) begin 

    assign data_wr[i] = data_we_i & data_be_i[i];    

    byte_data_memory
      #(//.ROM_FILE(ROM_FILE),
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(8)
      )
    byte_data_memory_dut
      (
        .clk_i ( clk_i ),
        .en_i( data_en ), //data_en
        .wr_i( data_wr[i] ),
        .addr_i( data_addr ),
        .wdata_i( data_wdata_i[ (i*8)+:8] ),
        .rdata_o( data_rdata_o[(i*8+7):(i*8)] )
      );
    end
endgenerate

assign data_gnt_o = data_en; // the request is accepted

always @(posedge clk_i)
begin
  data_rvalid_o <= data_en; // valid and data signals are high for one cycle, at the same time when data is available
end



`ifndef SYNTHESIS
  // contiguous memory for simulation purposes
  int n;
  initial 
  begin
    for(n = 0; n < MEM_SIZE / 4; n++)
    begin
      dbg_ram[n] = 'X;
    end
  end

  always @(posedge clk_i)
  begin
    begin
      for(n = 0; n < 4; n++)
      begin 
        if (data_en && data_wr[n])   
			dbg_ram[data_addr][(n*8)+:8] <= data_wdata_i[(n*8)+:8];        
      end
    end 
  end 

`endif

endmodule // data_memory
