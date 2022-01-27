module ram 
#(
  parameter MEM_SIZE = 4096
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
  output  logic [31:0]            data_rdata_o,

  input   logic                   osmem_en_i,
  input   logic                   osmem_req_i,
  output  logic                   osmem_gnt_o,
  output  logic                   osmem_rvalid_o,
  input   logic                   osmem_we_i,
  input   logic [3:0]             osmem_be_i,
  input   logic [31:0]            osmem_addr_i,
  input   logic [31:0]            osmem_wdata_i,
  output  logic [31:0]            osmem_rdata_o
);

localparam ADDR_WIDTH = $clog2(MEM_SIZE/4);

logic data_en;
logic [3:0] data_wr;
logic [ADDR_WIDTH-1:0] data_addr;

logic osmem_en;
logic [3:0] osmem_wr;
logic [ADDR_WIDTH-1:0] osmem_addr;

logic [31:0] dbg_ram[0:MEM_SIZE/4-1];

assign data_en = data_en_i & data_req_i;
assign data_addr = data_addr_i[ADDR_WIDTH-1+2:2];

assign osmem_en = osmem_en_i & osmem_req_i;
assign osmem_addr = osmem_addr_i[ADDR_WIDTH-1+2:2];

genvar i;
generate
  for (i = 0; i < 4; i++) begin 

    assign data_wr[i] = data_we_i & data_be_i[i];
    assign osmem_wr[i] = osmem_we_i & osmem_be_i[i];

    dpram
      #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(8)
      )
    dpram_inst
      (
        .clk_i ( clk_i ),

        .en_a_i( data_en ),
        .wr_a_i( data_wr[i] ),
        .addr_a_i( data_addr ),
        .wdata_a_i( data_wdata_i[ (i*8)+:8] ),
        .rdata_a_o( data_rdata_o[(i*8+7):(i*8)] ),

        .en_b_i( osmem_en ),
        .wr_b_i( osmem_wr[i] ),
        .addr_b_i( osmem_addr ),
        .wdata_b_i( osmem_wdata_i[ (i*8)+:8] ),
        .rdata_b_o( osmem_rdata_o[(i*8+7):(i*8)] )

      );
    end
endgenerate

assign data_gnt_o = data_req_i & data_en_i;

always @(posedge clk_i)
begin
  data_rvalid_o <= data_req_i & data_en_i;
end

assign osmem_gnt_o = osmem_req_i & osmem_en_i;

always @(posedge clk_i)
begin
  osmem_rvalid_o <= osmem_req_i & osmem_en_i;
end

`ifndef SYNTHESIS

  // contiguous memory for simulation purposes

  int n;
  initial 
  begin
    for(n=0; n<MEM_SIZE/4; n++)
    begin
      dbg_ram[n] = 'X;
    end
  end

  always @(posedge clk_i)
  begin
    begin
      for(n=0; n<4; n++)
      begin
        if (data_en && data_wr[n])   dbg_ram[data_addr][(n*8)+:8] <= data_wdata_i[(n*8)+:8];
        if (osmem_en && osmem_wr[n]) dbg_ram[osmem_addr][(n*8)+:8] <= osmem_wdata_i[(n*8)+:8];
      end
    end 
  end 

`endif

endmodule
