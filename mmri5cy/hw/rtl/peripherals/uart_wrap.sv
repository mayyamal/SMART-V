module uart_wrap
#(
  parameter ADDR_WIDTH = 4
)
(
  input  logic        			clk_i,
  input  logic        			rstn_i,
  input  logic        			en_i,

  input logic         			data_req_i,
  output  logic       			data_gnt_o,
  output  logic       			data_rvalid_o,

  input logic         			data_we_i,
  input logic [3:0]   			data_be_i,

  input logic [ADDR_WIDTH-1:0]  data_addr_i,
  input logic [31:0]            data_wdata_i,
  output logic [31:0]           data_rdata_o,

  output logic 					tx_o,
  input logic  					rx_i

);

  logic uart_en;
  logic [31:0] uart_data_reg;

  assign uart_en = data_req_i & en_i;

  always @(posedge clk_i)
  begin
    if ( uart_en ) begin
      uart_data_reg <= data_wdata_i;
    end
  end

  assign data_gnt_o = uart_en;

  always @(posedge clk_i)
  begin
    data_rvalid_o <= uart_en;
  end

  uart uart_inst
    (
      .clk ( clk_i ),
      .reset ( !rstn_i ),
      .addr ( data_addr_i[3:0] ),
      .wdata ( uart_data_reg ),
      .rdata ( data_rdata_o ) ,
      .en ( uart_en ),
      .wen( data_we_i ),
      .badmem ( ),
      .bus_wait ( ),

      .irq_tx ( ),
      .irq_rx ( ),
      .irq_err ( ),
      .txd ( tx_o ),
      .rxd ( 0 )
    );

endmodule
