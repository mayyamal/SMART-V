module top
#(
  parameter ROM_FILE = ""  
)
(
	input logic clk_i,
	input logic cpu_resetn, 

	output logic [15:0] leds_o,
	input logic  [15:0] sw,
	output logic 		uart_tx_o,

	output logic [7:0]   JA,
	output logic [7:0]   JB,
	output logic [7:0]   JC,
	output logic [7:0]   JXADC
	
	//-------------MM-----------------	 
	//input logic irq_i,
	//output logic irq_ack_o
	//-------------MM-----------------	 
	
  
);

  assign JA[0] = leds_o[15];

  logic         		clk;
  logic         		locked;
  logic 			  	rstn;

  logic 	     		instr_req;
  logic 	     		instr_gnt;
  logic 	      		instr_rvalid;
  logic [31:0]  		instr_addr;
  logic [31:0]  		instr_rdata;

  logic 		   		data_req;
  logic 		    	data_gnt;
  logic 		    	data_rvalid;
  logic [31:0]  		data_addr;
  logic 		    	data_we;
  logic [3:0]   		data_be;
  logic [31:0]  		data_rdata;
  logic [31:0]  		data_wdata;



  logic         		sec_lvl;
  //logic irq;
  logic [4:0] 			irq_id;
  //logic irq_ack;
  logic 				irq_sec;
  //logic 				irq_i = 'd0;
  logic 				irq_i;
  logic 				irq_ack_o;
  logic 				gpio_irq;
  
  
  logic [2-1:0]  	    periph_en_tmp;
  
  logic					s_bitBanded_tmp;	
  logic 			    bbError_tmp;
  logic [31:0] p0_bbwmask_tmp;

  //assign 				irq_id = 'd4;
  assign 				irq_id = 'd16; // [MM] Local Interrupt 0 = GPIO
  
  
`ifdef SYNTHESIS

  clk_gen clk_gen_inst
  (
    .clk_i ( clk_i ),
    .locked ( locked ),
    .clk_50_o ( clk )
  );
  assign rstn = locked & ~cpu_resetn;

`else

  assign clk = clk_i;

  // TODO refactor rstn
  initial
  begin
    rstn = 0;
    repeat(2) @(posedge clk); 
    rstn = 1;
  end

`endif


riscv_core
  #(
    .INSTR_RDATA_WIDTH ( 32 )
 
  )
  riscv_core_i
  (
    .clk_i                  ( clk          ) ,
    .rst_ni                 ( rstn         ) ,
    .clock_en_i             ( '1           ) ,
    .test_en_i              ( '1           ) ,    
    .boot_addr_i            ( 'h200    ) ,
    .core_id_i              ( 4'h0         ) ,
    .cluster_id_i           ( 6'h0         ) ,

    .instr_addr_o           ( instr_addr   ) ,
    .instr_req_o            ( instr_req    ) ,
    .instr_rdata_i          ( instr_rdata  ) ,
    .instr_gnt_i            ( instr_gnt    ) ,
    .instr_rvalid_i         ( instr_rvalid ) ,

    .data_addr_o            ( data_addr    ) ,
    .data_wdata_o           ( data_wdata   ) ,
    .data_we_o              ( data_we      ) ,
    .data_req_o             ( data_req     ) ,
    .data_be_o              ( data_be      ) ,
    .data_rdata_i           ( data_rdata   ) ,
    .data_gnt_i             ( data_gnt     ) ,
    .data_rvalid_i          ( data_rvalid  ) ,

   

    .irq_i                  ( gpio_irq     ) ,
    .irq_id_i               ( irq_id       ) ,
    .irq_ack_o              ( irq_ack_o    ) ,
    .irq_id_o               (              ) ,
    .irq_sec_i              ( irq_sec      ) ,

    .sec_lvl_o              ( sec_lvl      ) ,


    .fetch_enable_i         ( 1'b1         ) ,
    .core_busy_o            (              ) ,
    .periph_en_o            (periph_en_tmp ) `ifdef  SMARTV_EN ,
	.is_bitBanded_o 		(is_bitBanded_tmp),
	.p0_bbwmask_o			(p0_bbwmask_tmp)
    //.bbError			    (bbError_tmp)	
	`endif
  );
  
  
memory_controller
  #(
    .ROM_FILE ( ROM_FILE )
  )
 memory_controller_i
  (
    .clk_i                  ( clk          ) ,
    .rst_n                  ( rstn         ) ,
	
    .data_addr_i            ( data_addr    ) ,
    .data_wdata_i           ( data_wdata   ) ,
    .data_we_i              ( data_we      ) ,
    .data_req_i             ( data_req     ) ,
    .data_be_i              ( data_be      ) ,
	
    .data_rdata_o           ( data_rdata   ) ,
    .data_gnt_o             ( data_gnt     ) ,
    .data_rvalid_o          ( data_rvalid  ) ,	
	
    .instr_addr_i           ( instr_addr   ) ,
    .instr_req_i            ( instr_req    ) ,
    .instr_rdata_o          ( instr_rdata  ) ,
    .instr_gnt_o            ( instr_gnt    ) ,
    .instr_rvalid_o         ( instr_rvalid ) ,
	
    .leds_o                 ( leds_o       ) ,
	.gpio_irq				( gpio_irq	   ) ,
    .switches_i             ( sw   ) ,
    .uart_tx_o              ( uart_tx_o    ) ,
	.periph_en_i            ( periph_en_tmp) `ifdef  SMARTV_EN  ,
	.is_bitBanded_i 		(is_bitBanded_tmp),
	.p0_bbwmask_i			(p0_bbwmask_tmp)
	 `endif
    //.bbError_i			    (bbError_tmp)	
    
  );


 endmodule	// top
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
