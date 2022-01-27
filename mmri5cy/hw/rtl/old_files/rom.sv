module rom
#(
  parameter ROM_FILE = "",
  parameter MEM_SIZE = 4096
)
(
  input   logic         clk_i,
  input   logic         instr_req_i,
  output  logic         instr_gnt_o,
  output  logic         instr_rvalid_o,
  input   logic [31:0]  instr_addr_i,
  output  logic [31:0]  instr_rdata_o,
  input   logic         data_en_i,
  input   logic         data_req_i,
  output  logic         data_gnt_o,
  output  logic         data_rvalid_o,
  input   logic [31:0]  data_addr_i,
  output  logic [31:0]  data_rdata_o
);

  localparam ADDR_WIDTH = $clog2(MEM_SIZE/4);

  logic [ADDR_WIDTH-1:0] data_addr;
  logic [ADDR_WIDTH-1:0] instr_addr;

  assign data_addr = data_addr_i[ADDR_WIDTH-1+2:2];
  assign instr_addr = instr_addr_i[ADDR_WIDTH-1+2:2];

  dpram 
    #(
      .ROM_FILE (ROM_FILE),
      .ADDR_WIDTH( ADDR_WIDTH ),
      .DATA_WIDTH ( 32 )
    )
  dpram_inst
    (
      .clk_i ( clk_i ),
      .en_a_i( instr_req_i ),
      //.wr_a_i( 0 ),
      .addr_a_i( instr_addr ),
      //.wdata_a_i( 0 ),
      .rdata_a_o( instr_rdata_o ),

      .en_b_i( data_en_i ),
      //.wr_b_i( 0 ),
      .addr_b_i( data_addr ),
      //.wdata_b_i( 0 ),
      .rdata_b_o( data_rdata_o )
    );

assign instr_gnt_o = instr_req_i;
always @(posedge clk_i)
begin
  instr_rvalid_o <= instr_req_i;
end

assign data_gnt_o = data_req_i & data_en_i;

always @(posedge clk_i)
begin
  data_rvalid_o <= data_req_i & data_en_i;
end

endmodule
