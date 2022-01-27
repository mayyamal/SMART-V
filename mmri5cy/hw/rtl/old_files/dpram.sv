module dpram 
#(
  parameter ROM_FILE = "",
  parameter ADDR_WIDTH = 8,
  parameter DATA_WIDTH = 32
)
(
  input   logic  clk_i,

  input   logic                     en_a_i,
  input   logic                     wr_a_i,
  input   logic  [ADDR_WIDTH-1:0]   addr_a_i,
  input   logic  [DATA_WIDTH-1:0]   wdata_a_i,
  output  logic  [DATA_WIDTH-1:0]   rdata_a_o,

  input   logic                     en_b_i,
  input   logic                     wr_b_i,
  input   logic  [ADDR_WIDTH-1:0]   addr_b_i,
  input   logic  [DATA_WIDTH-1:0]   wdata_b_i,
  output  logic  [DATA_WIDTH-1:0]   rdata_b_o
);

localparam SIZE = 2**ADDR_WIDTH;

(*ram_style = "block" *) reg [DATA_WIDTH-1:0] mem[0:SIZE-1];

logic [DATA_WIDTH-1:0] rdata_a;
logic [DATA_WIDTH-1:0] rdata_b;

initial
begin
  if (ROM_FILE != "")
    $readmemh(ROM_FILE, mem);
end

always @(posedge clk_i)
begin
  // port a
  if (en_a_i)
  begin
    if (wr_a_i)
      mem[addr_a_i] <= wdata_a_i;
    rdata_a <= mem[addr_a_i];
  end 
end 

assign rdata_a_o = rdata_a;

always @(posedge clk_i)
begin
  // port b
  if (en_b_i)
  begin
    if (wr_b_i)
      mem[addr_b_i] <= wdata_b_i;
    rdata_b <= mem[addr_b_i];
  end 
end

assign rdata_b_o = rdata_b;

endmodule
