// Copyright 2021 TU Graz.

////////////////////////////////////////////////////////////////////////////////
// Author:         Maja Malenko - malenko.maya@gmail.com                      //
//                                                                            //
// Design Name:    SMART-V Protection Unit                                    //
// Project Name:   SMART-V                                                    //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Memory Protection of Tasks and Peripherals                 //
//                 Based on RI5CY PMP implementation                          //   
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


import riscv_defines::*;

module riscv_smartv
#(
   parameter N_PMP_ENTRIES = `SMARTV_N_PMP_ENTRIES,
   parameter PERIPHERAL_NO = 2
)
(
   input logic                             clk,
   input logic                             rst_n,

   input PrivLvl_t                         pmp_privil_mode_i,
   input logic  [N_PMP_ENTRIES-1:0] [31:0] pmp_addr_i,   
   input logic  [31:0] [31:0] 			   drv_addr_i,
   
   
   // from IF stage
   input  logic                            instr_req_i,
   input  logic [31:0]                     instr_addr_i,
   output logic                            instr_gnt_o,
   // to Instruction Memory
   output logic                            instr_req_o,
   output logic [31:0]                     instr_addr_o,
   input  logic                            instr_gnt_i,
   output logic                            instr_fault_o, 
   
    // data side : if TO pipeline
   input  logic                            data_req_i,
   input  logic [31:0]                     data_addr_i,
   input  logic                            data_we_i,
   output logic                            data_gnt_o,
   // if to Memory
   output logic                            data_req_o,
   input  logic                            data_gnt_i,
   output logic [31:0]                     data_addr_o,
   output logic                            data_err_o,
   input  logic                            data_err_ack_i,
   
   input logic [6:0] 					   opcodeData,
   input logic [6:0] 					   opcodeCode,

   
   input  logic							   is_interrupt,
   output logic [PERIPHERAL_NO-1:0]  	   periph_en_o, 
   output logic								   is_bitBanded_o,
   output [31:0] p0_bbwmask
    	
   
);
	
	localparam [32-1:0] PERIPHERAL_SIZE 	= 32'h00000100;
	localparam [32-1:0] PERIPHERAL_BASE 	= 32'h00800100;
	//localparam			PERIPHERAL_NO       = 2;
	
	
	localparam N_SMARTV_ADDRESS_REG = 3; // CODE, DATA, LIB
	localparam N_BITBAND = 2; //GPIO_IN, GPIO_OUT
	/*
	localparam N_DRIVERS = 16;
	
	logic [N_SE_ADDRESS-1:0][31:0] start_addr;
	logic [N_SE_ADDRESS-1:0][31:0] end_addr;	
		
	*/
	
	
	logic [PERIPHERAL_NO-1:0]  	   periph_en; // [MM] Be careful about this one, it is set within an `ifdef
	//logic is_bitBanded;
	//logic bbError;
	
	`ifdef  SMARTV_PERIPHERAL_EN 
			logic [31:0] SMARTV_PERIPH;
			logic [PERIPHERAL_NO-1 :0] periph_index;
			logic peripheral_access_fault;
						
	`endif 
	
	
	`ifdef  SMARTV_BITBANDING_EN 
			logic [N_BITBAND -1:0][31:0] bitband_reg;
			logic [31:0] SMARTV_PERIPH_BB;
			logic isBitBandedAccess;
			logic bb_access_fault; 

			logic [`BITFIELD_INDEX - 1:0] bbIndex;   
			logic [`BITFIELD_INDEX - 1:0] bbRegIndex; 

			logic [PERIPHERAL_NO-1 :0] periph_index_bb;	
			logic [N_BITBAND -1:0] whichPeriph;
	`endif
			
		
	//`ifdef  SMARTV_EN 
	
		logic [N_SMARTV_ADDRESS_REG-1:0][31:0] start_addr;
		logic [N_SMARTV_ADDRESS_REG-1:0][31:0] end_addr;
		logic               data_err_int;
		
		
						
		always_comb
        begin
			start_addr	[`SMARTV_CAR] = pmp_addr_i[`PMP_CODE_START]; 	// CSR addr = 3B0
			end_addr	[`SMARTV_CAR] = pmp_addr_i[`PMP_CODE_END]; 		// CSR addr = 3B1
			
			start_addr	[`SMARTV_DAR] = pmp_addr_i[`PMP_DATA_START];	// CSR addr = 3B2	
			end_addr	[`SMARTV_DAR] = pmp_addr_i[`PMP_DATA_END]; 		// CSR addr = 3B3
			
			start_addr	[`SMARTV_LAR] = pmp_addr_i[`PMP_LIB_START]; 	// CSR addr = 3B4
			end_addr	[`SMARTV_LAR] = pmp_addr_i[`PMP_LIB_END]; 		// CSR addr = 3B5
			
		`ifdef  SMARTV_PERIPHERAL_EN 
			SMARTV_PERIPH			  = pmp_addr_i[`PMP_PERIPH]; 		// CSR addr = 3B6
		`endif 
		
		
	`ifdef  SMARTV_BITBANDING_EN 
			
			SMARTV_PERIPH_BB 		 = pmp_addr_i[`PMP_PERIPH_BB]; 		// CSR addr = 3B7
			bitband_reg[`SMARTV_PERIPH_0]			 = pmp_addr_i[`PMP_PERIPH_BB_0];   // GPIO_BB CSR addr = 3B8
			bitband_reg[`SMARTV_PERIPH_1]			 = pmp_addr_i[`PMP_PERIPH_BB_1];
			// UART_BB CSR addr = 3B9			
			
			whichPeriph      = {N_BITBAND{1'b0}}; // [MM] both GPIO_IN, GPIO_OUT belong to peripheral 0 = GPIO
	`endif
		
		
			
			/*
			start_addr	[`SMARTV_ROAR] = pmp_addr_i[`PMP_RO_START]; 	// CSR addr = 3B6
			end_addr	[`SMARTV_ROAR] = pmp_addr_i[`PMP_RO_END]; 		// CSR addr = 3B7
			
			start_addr	[`SMARTV_SAR] = pmp_addr_i[`PMP_STC_START]; 	// CSR addr = 3B6
			end_addr	[`SMARTV_SAR] = pmp_addr_i[`PMP_STC_END]; 		// CSR addr = 3B7	
		
			
			*/
		end	
		
	
		//--------------------------- Data Access Fault -------------------------		
		//logic data_address_out_of_bounds;
		logic store_access_fault;         // exception code = 7
		logic load_access_fault;		  // exception code = 5
		logic data_access_fault;	
		logic valid_data_address;
		//logic data_addr_valid;
		
		
		logic isGPIO;
		logic isUART;
		logic isPeripheral;		
		
		
		//--------------------------- Code Access Fault -------------------------	
		logic instr_access_fault;		 // exception code = 1
		logic valid_instruction_address;
		logic valid_lib_address;
		//logic valid_lib_address_data; // [MM] In printf there is a function poiter to streamWriter!?
		
//	`endif 
 


/////////////////////////////////////////////////////////////////////////
// ___   _ _____ _     ___ ___  ___ _____ ___ ___ _____ ___ ___  _  _  //
//|   \ /_\_   _/_\   | _ \ _ \/ _ \_   _| __/ __|_   _|_ _/ _ \| \| | //
//| |) / _ \| |/ _ \  |  _/   / (_) || | | _| (__  | |  | | (_) |  ` | //
//|___/_/ \_\_/_/ \_\ |_| |_|_\\___/ |_| |___\___| |_| |___\___/|_|\_| //
//                                                                     //    
// http://www.network-science.de/ascii/, small                         // 
/////////////////////////////////////////////////////////////////////////


// The next code portion only forwards the Data Address signals between LSU and Data Memory 
// Just for testing, when no protection is needed
/*	
    assign data_addr_o  = data_addr_i;	
	assign data_req_o   = data_req_i;
    assign data_gnt_o   = data_gnt_i;
    //assign data_err_int = 1'b0;
	assign data_err_o   = 1'b0;	
	assign periph_en_o 	   = {PERIPHERAL_NO{1'b1}};
	assign is_bitBanded	   = 1'b0;
	assign  bbError		   = 1'b0;
*/ 
 
//`ifdef  SMARTV_EN 
 //[MM To Do] Use parameters! 
assign isGPIO = (( data_addr_i >= 'h0080_0100 && data_addr_i < 'h0080_0200));
assign isUART = (( data_addr_i >= 'h0080_0200 && data_addr_i < 'h0080_0300));

`ifdef  SMARTV_BITBANDING_EN 
assign isPeripheral = isGPIO || isUART || isBitBandedAccess;
`else 
assign isPeripheral = isGPIO || isUART;
`endif

assign valid_data_address = ((data_addr_i >= start_addr[`SMARTV_DAR]) && (data_addr_i <= end_addr[`SMARTV_DAR]));


//assign data_addr_valid  =  (valid_data_address || isPeripheral);// || valid_lib_address);
assign store_access_fault = !(valid_data_address || isPeripheral) && (opcodeData == OPCODE_STORE); 
assign load_access_fault  = !(valid_data_address || isPeripheral) && (opcodeData == OPCODE_LOAD); 

`ifdef  SMARTV_BITBANDING_EN 
	assign data_access_fault = store_access_fault || load_access_fault || (peripheral_access_fault && !isBitBandedAccess) || bb_access_fault;

`elsif   SMARTV_PERIPHERAL_EN 
	assign data_access_fault = store_access_fault || load_access_fault || peripheral_access_fault;

`else
	assign data_access_fault = store_access_fault || load_access_fault;
`endif

assign data_addr_o  =  data_addr_i;
		
	always_comb
    begin
      if(pmp_privil_mode_i == PRIV_LVL_M)
      begin
         data_req_o   = data_req_i;
         data_gnt_o   = data_gnt_i;
         data_err_int   = 1'b0;

      end
      else
      begin		
		if(data_access_fault == 1'b1)
            begin
               data_req_o   = 1'b0;
               data_err_int   =  data_req_i; 
               data_gnt_o   = 1'b0;
            end
            else
            begin 
               data_req_o   =  data_req_i;
               data_err_int =  1'b0;
               data_gnt_o   =  data_gnt_i;
            end
      end
    end
	
	
   enum logic {IDLE, GIVE_ERROR} data_err_state_q, data_err_state_n;

   always_comb
   begin
      data_err_o       = 1'b0;
	  periph_en_o 	   = {PERIPHERAL_NO{1'b1}};
      data_err_state_n = data_err_state_q;
	  //is_bitBanded_o     = 1'b0;
	  //bbError		   = 1'b0;
	  
	  
      unique case(data_err_state_q)
         IDLE:
         begin
            if(data_err_int)
               data_err_state_n = GIVE_ERROR;			   
         end

         GIVE_ERROR:
         begin
            data_err_o = 1'b1; // [MM] 
			
			`ifdef  SMARTV_PERIPHERAL_EN 
			periph_en_o = periph_en; // [MM] !!!
			`endif
			
			
            if(data_err_ack_i)
               data_err_state_n = IDLE;
         end
      endcase
   end
   
   always_ff @(posedge clk or negedge rst_n) begin
      if(~rst_n) begin
          data_err_state_q <= IDLE;
      end else begin
          data_err_state_q <= data_err_state_n;
      end
   end
	
//`endif 

  
//////////////////////////////////////////////////////////////////////////
//  ___ ___  ___  ___   ___ ___  ___ _____ ___ ___ _____ ___ ___  _  _  //
// / __/ _ \|   \| __| | _ \ _ \/ _ \_   _| __/ __|_   _|_ _/ _ \| \| | //
//| (_| (_) | |) | _|  |  _/   / (_) || | | _| (__  | |  | | (_) |  ` | //
// \___\___/|___/|___| |_| |_|_\\___/ |_| |___\___| |_| |___\___/|_|\_| //
//                                                                      // 
//////////////////////////////////////////////////////////////////////////
   
// The next code portion only forwards the Instruction Address signals between Prefetch Buffer and Instruction Memory 
// Just for testing, when no protection is needed
/*
   assign instr_addr_o  = instr_addr_i;      
   assign instr_req_o   = instr_req_i;
   assign instr_gnt_o   = instr_gnt_i;
   assign instr_fault_o   = 1'b0;
   assign periph_en_o 	   = {PERIPHERAL_NO{1'b1}};
*/
   
   
   
//`ifdef  SMARTV_EN 
 assign valid_instruction_address = ((instr_addr_i >= start_addr[`SMARTV_CAR]) && (instr_addr_i <= end_addr[`SMARTV_CAR]));
 assign valid_lib_address = ((instr_addr_i >= start_addr[`SMARTV_LAR]) && (instr_addr_i <= end_addr[`SMARTV_LAR]));
   
 assign instr_access_fault = !(valid_instruction_address || valid_lib_address) && ((opcodeCode == OPCODE_BRANCH) || (opcodeCode == OPCODE_JALR) || (opcodeCode == OPCODE_JAL));

//assign instr_access_fault = !(valid_instruction_address || valid_lib_address);
assign instr_addr_o = instr_addr_i;
   
   always_comb
   begin
      if(pmp_privil_mode_i == PRIV_LVL_M)
      begin
         instr_req_o   = instr_req_i;
         instr_gnt_o   = instr_gnt_i;
         instr_fault_o   = 1'b0;

      end
      else
      begin
            if(instr_access_fault == 1'b1)
            begin
               instr_req_o   = 1'b0;
               instr_fault_o   = instr_req_i;
               instr_gnt_o   = 1'b0;
            end
            else
            begin
               instr_req_o   =  instr_req_i;
               instr_fault_o   =  1'b0;
               instr_gnt_o   =  instr_gnt_i;
            end
      end
   end
	
//`endif 

   
////////////////////////////////////////////////////////////////////////////////////////////////////
// ___ ___ ___ ___ ___ _  _ ___ ___    _   _      ___ ___  ___ _____ ___ ___ _____ ___ ___  _  _  //
//| _ \ __| _ \_ _| _ \ || | __| _ \  /_\ | |    | _ \ _ \/ _ \_   _| __/ __|_   _|_ _/ _ \| \| | //
//|  _/ _||   /| ||  _/ __ | _||   / / _ \| |__  |  _/   / (_) || | | _| (__  | |  | | (_) |  ` | //
//|_| |___|_|_\___|_| |_||_|___|_|_\/_/ \_\____| |_| |_|_\\___/ |_| |___\___| |_| |___\___/|_|\_| //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////   
   
   
   
`ifdef  SMARTV_PERIPHERAL_EN 

	assign periph_index = (data_addr_i - PERIPHERAL_BASE) / PERIPHERAL_SIZE;
	assign peripheral_access_fault = (SMARTV_PERIPH[periph_index] == 0) && isPeripheral && ((opcodeData == OPCODE_STORE) || (opcodeData == OPCODE_LOAD));
	
	
	`ifdef  SMARTV_BITBANDING_EN 
		assign periph_en = (isBitBandedAccess == 1 && bb_access_fault == 1)? {PERIPHERAL_NO{1'b0}} : SMARTV_PERIPH_BB[0]; //[MM] Change it!!!!!
	`else
		assign periph_en = SMARTV_PERIPH[periph_index];
	`endif
		 
		
`endif 
  
//assign periph_en_o = periph_en;   
   
   
   
   
   
////////////////////////////////////////////////////////
// ___ ___ _____    ___   _   _  _ ___ ___ _  _  ___  //
//| _ )_ _|_   _|__| _ ) /_\ | \| |   \_ _| \| |/ __| //
//| _ \| |  | ||___| _ \/ _ \| .` | |) | || .` | (_ | // 
//|___/___| |_|    |___/_/ \_\_|\_|___/___|_|\_|\___| //                 
////////////////////////////////////////////////////////  



`ifdef  SMARTV_BITBANDING_EN  
   
     
	assign bbIndex = ((data_addr_i - `BITBAND_PERIPH_BASE)/4)%32; //the bit inside the SSMARTV_PERIPH_BB register
	assign bbRegIndex = ((data_addr_i - `BITBAND_PERIPH_BASE)/4)/32; //which of N_BITBAND bitband_reg registers should be used
		
	logic [32-1:0] firtsBB = `BITBAND_PERIPH_BASE; 
    logic [32-1:0] lastBB = `BITBAND_PERIPH_LAST; 
	
	//logic [31:0]  p0_bbwmask;
      
    assign isBitBandedAccess = ((data_addr_i >= firtsBB) && (data_addr_i <= lastBB));
	
	assign bb_access_fault = (isBitBandedAccess && bitband_reg[bbRegIndex][bbIndex] == 0) && ((opcodeData == OPCODE_STORE) || (opcodeData == OPCODE_LOAD)); 
	
	assign periph_index_bb = bbRegIndex;
	
	assign p0_bbwmask = {32{1 << bbIndex}}; 
	
	
`endif

`ifdef  SMARTV_BITBANDING_EN 
	assign is_bitBanded_o = isBitBandedAccess; // [MM] !!!
`else	
	assign is_bitBanded_o = 1'b0;
`endif

///////////////////////////////////////
// _____ _   ___  ___ ___ _  _  ___  //
//|_   _/_\ / __|/ __|_ _| \| |/ __| //
//  | |/ _ \ (_ | (_ || || .` | (_ | //
//  |_/_/ \_\___|\___|___|_|\_|\___| //                                             
///////////////////////////////////////
   
   
   
`ifdef  SMARTV_TAGGING_EN 
	
`endif 
   
 
   
   
//////////////////////////////////////////////////////////////////////////////////
// ___  ___ _____   _____ ___   ___ ___  ___ _____ ___ ___ _____ ___ ___  _  _  //
//|   \| _ \_ _\ \ / / __| _ \ | _ \ _ \/ _ \_   _| __/ __|_   _|_ _/ _ \| \| | //
//| |) |   /| | \ V /| _||   / |  _/   / (_) || | | _| (__  | |  | | (_) | .` | //
//|___/|_|_\___| \_/ |___|_|_\ |_| |_|_\\___/ |_| |___\___| |_| |___\___/|_|\_| // 
//////////////////////////////////////////////////////////////////////////////////      
   
`ifdef  SMARTV_DRIVER_EN 
	
`endif 




endmodule // riscv_smartv