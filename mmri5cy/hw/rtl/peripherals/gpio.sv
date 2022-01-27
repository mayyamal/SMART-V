`include "smartv_peripherals.vh"

module gpio
#(
  parameter ADDR_WIDTH = 12
)
(
  input logic                   clk_i,
  input  logic                  en_i,
  input logic 					rstn_i,
  input logic                   data_req_i,
  output  logic                 data_rvalid_o,
  output  logic                 data_gnt_o,
  input logic                   data_we_i,
  input logic [3:0]             data_be_i,
  input logic [`GPIO_REG_WIDTH-1:0]  data_addr_i,
  input logic [31:0]            data_wdata_i,
  output logic [31:0]           data_rdata_o,
  
  output logic [15:0]           leds_o,
  input logic [15:0]            switches_i,
  output                        gpio_irq `ifdef  SMARTV_EN ,
  input   logic [31:0]			p0_bbwmask_i ,
  input   logic					is_bitBanded_i,
  input   logic [31:0]     		bb_addr_data 
`endif  
);

logic w_en;
logic [31:0] data_rdata;


logic [15:0] gpio_ie;

assign w_en = en_i & data_req_i & data_we_i;
assign data_rdata_o = data_rdata;

//assign data_rdata_o = {16'h0} & leds_o;


always @(posedge clk_i)
begin
	//data_rdata_o <= 32'h0;

    if (data_addr_i == 12'h00) data_rdata <= {16'h0, leds_o};
    if (data_addr_i == 12'h04) data_rdata <= {16'h0, switches_i};	  
	if (data_addr_i == 12'h08) data_rdata <= {16'h0, gpio_ie};
	  
end

always @(posedge clk_i)
begin
  if (rstn_i == 1'b0) begin
	leds_o  <= 16'h0;
	gpio_ie <= 16'h0;
  end else begin
  
	if (w_en && data_addr_i == 12'h00) begin
	`ifdef  SMARTV_EN
		if(is_bitBanded_i) begin
			leds_o <= (leds_o & ~(p0_bbwmask_i)) | p0_bbwmask_i;
		end
		else
	`endif
		leds_o <= data_wdata_i[15:0];
	
	end
	if (w_en && data_addr_i == 12'h08) gpio_ie <= data_wdata_i[15:0];

  end
  //if (leds_en)
//	leds_o <= data_wdata_i[15:0];
end


/****************  Interrupts ***************/
assign gpio_irq = |(gpio_ie & switches_i);



assign data_gnt_o = data_req_i & en_i;

always @(posedge clk_i)
begin
  data_rvalid_o <= data_req_i & en_i;
end


endmodule
