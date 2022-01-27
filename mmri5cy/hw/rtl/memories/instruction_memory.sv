 // Copyright 2021 TU Graz.

////////////////////////////////////////////////////////////////////////////////
// Author:         Maja Malenko - malenko.maya@gmail.com                      //
//                                                                            //
// Design Name:    SMART-V Instruction Memory                                 //
// Project Name:   SMART-V                                                    //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Instruction Memory (ROM) for read-only access              //
//   			   to instructions           				                  //
//                 Based on Tobias Scheipel tsri5cy project implementation    //   
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module instruction_memory
#(
  parameter ROM_FILE = "",
  parameter MEM_SIZE = 4096, // KB 2^12
  parameter DATA_WIDTH = 32
)
( 
  input   logic         clk_i,
  input   logic         instr_req_i,
  output  logic         instr_gnt_o,
  output  logic         instr_rvalid_o,
  input   logic [31:0]  instr_addr_i,
  output  logic [31:0]  instr_rdata_o,
  //RO Data
  input   logic         data_en_i,
  input   logic         data_req_i,
  output  logic         data_gnt_o,
  output  logic         data_rvalid_o,
  input   logic [31:0]  data_addr_i,
  output  logic [31:0]  data_rdata_o  
);

  localparam ADDR_WIDTH = $clog2(MEM_SIZE/4); 
  logic [ADDR_WIDTH-1:0] instr_addr; 
  logic [ADDR_WIDTH-1:0] data_addr;
  logic [DATA_WIDTH-1:0] rdata_instr;
  logic [DATA_WIDTH-1:0] rdata_rodata;
  
  (*rom_style = "block" *) reg [DATA_WIDTH-1:0] mem[0:MEM_SIZE-1];
  
assign instr_addr = instr_addr_i[ADDR_WIDTH-1+2:2]; // 4B index: 0x0->0x0, 0x4->0x1, ...,  0x200 -> 0x80, ...
assign data_addr = data_addr_i[ADDR_WIDTH-1+2:2];
    
initial
begin
	if (ROM_FILE != "")
    $readmemh(ROM_FILE, mem);
end  
  
always_ff @(posedge clk_i)
begin
  // instruction request is issued
  if (instr_req_i)
  begin    
    rdata_instr <= mem[instr_addr];
  end 
end 

assign instr_rdata_o = rdata_instr;

assign instr_gnt_o = instr_req_i; // the request is accepted

always_ff @(posedge clk_i)
begin
  instr_rvalid_o <= instr_req_i; // valid and data signals are high for one cycle, at the same time when data is available
end



assign data_gnt_o = data_req_i & data_en_i;

always @(posedge clk_i)
begin
  data_rvalid_o <= data_req_i & data_en_i;  
  if (data_en_i)  
    rdata_rodata <= mem[data_addr];
end

assign data_rdata_o = rdata_rodata;

endmodule // instruction_memory
