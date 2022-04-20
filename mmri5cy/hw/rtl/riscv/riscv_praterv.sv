// Copyright 2021 TU Graz.

////////////////////////////////////////////////////////////////////////////////
// Author:         Maja Malenko - malenko.maya@gmail.com                      //
//                                                                            //
// Design Name:    PRATER-V Protection Unit                                   //
// Project Name:   PRATER-V                                                   //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Memory Protection and Relocation                           //
//                 Based on RI5CY PMP implementation                          //   
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

import riscv_defines::*;

module riscv_praterv
#(
   parameter N_PMP_ENTRIES = `PRATERV_N_PMP_ENTRIES,
   parameter PERIPHERAL_NO = 2
)
(
   input logic                             clk,
   input logic                             rst_n,

   input PrivLvl_t                         pmp_privil_mode_i,

   input logic  [N_PMP_ENTRIES-1:0] [31:0] pmp_addr_i,
   
   input logic  [31:0] [31:0]              drv_addr_i,
   
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


   // fetch side : if TO pipeline
   input  logic                            instr_req_i,
   input  logic [31:0]                     instr_addr_i,
   output logic                            instr_gnt_o,
   // fetch to PF buffer
   output logic                            instr_req_o,
   input  logic                            instr_gnt_i,
   output logic [31:0]                     instr_addr_o,
   output logic                            instr_err_o, 
   
   input  logic							   is_interrupt,
   output logic [PERIPHERAL_NO-1:0]  	   periph_en_o

   
);

	localparam N_DRIVERS = 31;
	localparam N_SMARTV_ADDRESS_REG = 3; // CODE, DATA, LIB
	localparam TAGDRIVER_LENGTH = 5;
	localparam TAGFUNCTION_LENGTH = 5;
	
	localparam [32-1:0] PERIPHERAL_SIZE 	= 32'h00000100;
	//MM: Use underscores: 32'h0000_0100;
	localparam [32-1:0] PERIPHERAL_BASE 	= 32'h00800100;
	//MM: Use underscores: 32'h0080_0100;
			
	// ******** ADDRESS START/END REGISTERS ********
	logic [N_SMARTV_ADDRESS_REG-1:0][31:0] start_addr;
	logic [N_SMARTV_ADDRESS_REG-1:0][31:0] end_addr;	
	logic data_err_int;
		
	// ******** PERIPHERAL BITFIELD REGISTER ********
	//logic [31:0] PRATERV_PERIPH;
	
	// ******** ADDRESS RELOCATION REGISTERS ********
	//logic [N_SMARTV_ADDRESS_REG-1:0][31:0] reloc_addr;
	logic [31:0] PRATERV_CRR;
	logic [31:0] PRATERV_DRR;
	//logic [31:0] PRATERV_DRRTMP;
	
	
	//----------------------IIT/Middleware -----------------------
	
	
	// The starting address can always be calculated from the VA (+Tags), no need to check, cause the 
	// middleware is already compiled with it, and the checks will be successful only for MW VAs
	//logic [N_DRIVERS-1:0][31:0] drv_start_addr_code;
	logic [N_DRIVERS-1:0][31:0] drv_stop_addr_code;
	logic [N_DRIVERS-1:0][31:0] drv_rel_addr_code;
	
	//logic [N_DRIVERS-1:0][31:0] drv_start_addr_data;
	// [MM] We activated these regs because of the new benchmarks which had BSS section
	//logic [N_DRIVERS-1:0][31:0] drv_stop_addr_data; 
	logic [N_DRIVERS-1:0][31:0] drv_rel_addr_data;
	
	/*logic [31:0] active_start_addr_code;
	logic [31:0] active_end_addr_code;
	logic [31:0] active_rel_addr_code;
	
	
	logic [31:0] active_start_addr_data;
	logic [31:0] active_end_addr_data;
	logic [31:0] active_rel_addr_data;
	
	logic [3:0] whichDriver;
	*/
	
    always_comb
        begin
			start_addr[`PRATERV_CAR] 	 	= pmp_addr_i[`PMP_CODE_START]; //3B0
			end_addr[`PRATERV_CAR] 			= pmp_addr_i[`PMP_CODE_END]; //3B1
			
			start_addr[`PRATERV_DAR]  		= pmp_addr_i[`PMP_DATA_START]; //3B2	
			end_addr[`PRATERV_DAR] 			= pmp_addr_i[`PMP_DATA_END]; //3B3
			
			start_addr[`PRATERV_LAR]  		= pmp_addr_i[`PMP_LIB_START]; //3B4
			end_addr[`PRATERV_LAR] 			= pmp_addr_i[`PMP_LIB_END]; //3B5
			
			//PRATERV_PERIPH 					= pmp_addr_i[`PMP_PERIPH]; //3B6
			
			PRATERV_CRR						= pmp_addr_i[`PMP_CODE_RELOC]; //3B7
			PRATERV_DRR						= pmp_addr_i[`PMP_DATA_RELOC]; //3B8
			
			 			
			//drv_start_addr_code[`PMP_DRV_ID] = pmp_addr_i[`PMP_DRV_CODE_START]; //3B9	
			drv_stop_addr_code[pmp_addr_i[`PMP_DRV_ID]] = pmp_addr_i[`PMP_DRV_CODE_END];   //3BA
			drv_rel_addr_code[pmp_addr_i[`PMP_DRV_ID]] 	= pmp_addr_i[`PMP_DRV_CODE_RELOC]; //3BB
			
			
			// [MM] We activated these regs because of the new benchmarks which had BSS section
			drv_rel_addr_data[pmp_addr_i[`PMP_DRV_ID]] 	= pmp_addr_i[`PMP_DRV_DATA_RELOC]; //3BD
									
		end
		
			
		
    //--------------------------- Data Access Fault -------------------------	
	logic store_access_fault;       //exception code = 7
    logic load_access_fault;		//exception code = 5
	logic data_access_fault;
	logic valid_data_address;
	
	
	logic isGPIO;
	logic isUART;
	logic isPeripheral;
	
	
	logic peripheral_access_fault;
	logic [PERIPHERAL_NO-1 :0] periph_index;
	logic [PERIPHERAL_NO-1:0]  periph_en; // [MM] Be careful about this one, it is set within an `ifdef
	
	//--------------------------- Code Access Fault -------------------------
	
	logic instr_access_fault; 		//exception code = 1
	logic valid_mono_instruction_address;
    logic valid_mono_lib_address;
	
   //--------------------------- Tagging -------------------------
	
	logic [TAGDRIVER_LENGTH -1: 0] tagDriver; 
	logic tagMode;
	logic tagMemory;
	
	// [MM] The virtuual address of the trampolines for the drivers
	logic [TAGDRIVER_LENGTH - 1: 0] tagDriverTramp; 
	logic [TAGFUNCTION_LENGTH - 1 : 0] tagFunctionTramp;
	logic tagModeTramp;
	logic tagMemoryTramp;
	
	logic [31 : 0] driverBaseAddress;
	logic [31 : 0] driverIITEndAddress;
	

	
/////////////////////////////////////////////////////////////////////////
// ___   _ _____ _     ___ ___  ___ _____ ___ ___ _____ ___ ___  _  _  //
//|   \ /_\_   _/_\   | _ \ _ \/ _ \_   _| __/ __|_   _|_ _/ _ \| \| | //
//| |) / _ \| |/ _ \  |  _/   / (_) || | | _| (__  | |  | | (_) |  ` | //
//|___/_/ \_\_/_/ \_\ |_| |_|_\\___/ |_| |___\___| |_| |___\___/|_|\_| //
//                                                                     //    
// http://www.network-science.de/ascii/, small                         // 
/////////////////////////////////////////////////////////////////////////

////////////////////////////////////// 
//  _____              _            //         
//|_   _|_ _ __ _ __ _(_)_ _  __ _  //
//  | |/ _` / _` / _` | | ' \/ _` | //
//  |_|\__,_\__, \__, |_|_||_\__, | //
//          |___/|___/       |___/  //
////////////////////////////////////// 


	// Interface Indirection Table (IIT)
	assign tagModeTramp 		= data_addr_i[31];
	assign tagMemoryTramp		= data_addr_i[30];
	assign tagDriverTramp 		= data_addr_i[29:25];
	assign tagFunctionTramp 	= 5'b00011;//drv_stop_addr_code[tagDriverTramp][24:20]
	// [MM] Change it later, when it is programmed by the kernel! For now we have 3 functions per driver!
	 
			
	logic [31:0] relocate_data;
	logic isDrvData;
	logic isAppData;
	logic isKernelData;
	
	always_comb
	begin
		isDrvData = 1'b0;
		isAppData = 1'b0;
		isKernelData = 1'b0;
		driverBaseAddress = {tagModeTramp, tagMemoryTramp, tagDriverTramp, 25'b0 };
		driverIITEndAddress = driverBaseAddress + 4*(tagFunctionTramp - 1);
		if(tagModeTramp == 1'b1) // user space
		begin
			if(tagDriverTramp != 5'b00000)			//if MWM module, the IIT!
				begin		
					/* Let's check whether we access a data belonging to a middleware module from a
					   different middleware module or from an application module.
					   If so, we must land into the IIT!
					*/
					if(tagDriver != tagDriverTramp) // If we access the IIT (tagDriverTramp), while our PC is still in another module (tagDriver)
					//Check against the size of the IIT
						begin
							//if (/*(data_addr_i >= drv_start_addr_code[tagDriverTramp]) && */(data_addr_i <= drv_stop_addr_code[tagDriverTramp]))
							if (data_addr_i <= driverIITEndAddress)
								begin
									relocate_data = drv_rel_addr_code[tagDriverTramp];
									isDrvData = 1'b1;
									//allDrv |= 1 << tagDriver;
								end	
							else	
								begin
									isDrvData = 1'b0;
								end	
						end
					else //New for the benchmarks BSS
						begin
							if (tagMemoryTramp == 1'b1)// check tagMemoryTramp!!!!!!!!!!!!!!!!!!!!!!!!!
								relocate_data = drv_rel_addr_data[tagDriverTramp];
							else 
								relocate_data = drv_rel_addr_code[tagDriverTramp];
							isDrvData = 1'b1;
						end
											
				end // if MWM module	
// [MM] To Do: check the data S/E as well! Fow now, the IIT is the only data section!
			else 	//if APP module				
					begin
						if(/*(data_addr_i >= start_addr[`PRATERV_CAR]) && */(data_addr_i <= end_addr[`PRATERV_CAR]))		
							begin
								relocate_data = PRATERV_CRR;
								isAppData = 1'b1;
							end	
						else if((data_addr_i >= start_addr[`PRATERV_DAR]) && (data_addr_i <= end_addr[`PRATERV_DAR]))	
							begin
								relocate_data = PRATERV_DRR;
								isAppData = 1'b1;
							end	
					end	
		end	
			else //if kernel space or monolithic tasks!		
				begin
					relocate_data = 32'b0;					
					isKernelData = 1'b1;
				end					
	end

// The next code portion only forwards the Data Address signals between LSU and Data Memory 
// Just for testing, when no protection is needed
/*	
    assign data_addr_o  = data_addr_i;	
	assign data_req_o   = data_req_i;
    assign data_gnt_o   = data_gnt_i;
    //assign data_err_int = 1'b0;
	assign data_err_o   = 1'b0;	
	assign periph_en_o 	   = {PERIPHERAL_NO{1'b1}};
*/ 	
    //[MM To Do] Use parameters!  
	assign isGPIO = (( data_addr_i >= 'h0080_0100 && data_addr_i < 'h0080_0200));
	assign isUART = (( data_addr_i >= 'h0080_0200 && data_addr_i < 'h0080_0300));
	assign isPeripheral = isGPIO || isUART;
	
	
//	assign isMET =  (( data_addr_i >= met_start && data_addr_i < met_end));
//	assign whichDriver = (data_addr_i - met_start)/4;
	
	logic   isprintfAddr;
    assign 	isprintfAddr = (( data_addr_i >= 'h0000_0000 && data_addr_i < 'h0040_1000));

	assign valid_data_address = ((data_addr_i >= start_addr[`SMARTV_DAR_MONOLITHIC]) && (data_addr_i <= end_addr[`SMARTV_DAR_MONOLITHIC])) || (valid_mono_lib_address == 1'b1) || isDrvData || isAppData; // [MM] the valid_lib_address is a current workaround for that stupid printfx

	
	assign store_access_fault = !(valid_data_address || isPeripheral) && (opcodeData == OPCODE_STORE); 
	assign load_access_fault  = !(valid_data_address || isPeripheral) && (opcodeData == OPCODE_LOAD); 
	
	assign data_access_fault = store_access_fault || load_access_fault; //|| peripheral_access_fault; [MM] When getResource is implemented
	//data_address_out_of_bounds && !isPeripheral && !isMET;
	
	
    //assign data_addr_o  = isROV_daddr ? data_addr_i + relocate_ROdata : ((pmp_privil_mode_i == PRIV_LVL_M) || isPeripheral || valid_lib_address ) ? data_addr_i : data_addr_i + PRATERV_DRR;
	
	//assign data_addr_o  = isROV_daddr ? data_addr_i + relocate_ROdata : (isPeripheral || valid_lib_address ) ? data_addr_i : data_addr_i + PRATERV_DRR;
	//assign data_addr_o  = isROV_daddr ? data_addr_i + relocate_ROdata : (isPeripheral  || isprintfAddr) ? data_addr_i : data_addr_i + PRATERV_DRR;
	assign data_addr_o  =  data_addr_i + relocate_data;
	// 1. isROV_daddr relocates the RO data with the code relocation register of APP or MWM
	// 2. static lib functions are not relocated
	// 3. relocate other data with the data relocation register
	

	//==========MET
	/*assign active_start_addr_data = isMET ? drv_start_addr_data[whichDriver] : start_addr[`DATA];
	assign active_end_addr_data = isMET ? drv_stop_addr_data[whichDriver] : stop_addr[`DATA];
	assign active_rel_addr_data = isMET ? drv_rel_addr_data[whichDriver] : reloc_addr[`DATA];
	*/
	
	
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
		if(data_access_fault == 1'b1 )
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
      data_err_state_n = data_err_state_q;
	  periph_en_o 	   = {PERIPHERAL_NO{1'b1}}; // [MM] not yet fully implemented, always 1
	  
      unique case(data_err_state_q)
         IDLE:
         begin
            if(data_err_int)
               data_err_state_n = GIVE_ERROR;
         end

         GIVE_ERROR:
         begin
            data_err_o = 1'b1; 
			periph_en_o = periph_en; // [MM]!!!
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
   //=======MET
/*
	assign active_start_addr_code = isMET ? drv_start_addr_code[whichDriver] : start_addr[`CODE];
	assign active_end_addr_code = isMET ? drv_stop_addr_code[whichDriver] : stop_addr[`CODE];
	assign active_rel_addr_code = isMET ? drv_rel_addr_code[whichDriver] : reloc_addr[`CODE];
 */  
 
 ////////////////////////////////////// 
//  _____              _            //         
//|_   _|_ _ __ _ __ _(_)_ _  __ _  //
//  | |/ _` / _` / _` | | ' \/ _` | //
//  |_|\__,_\__, \__, |_|_||_\__, | //
//          |___/|___/       |___/  //
////////////////////////////////////// 

	assign tagDriver	 = instr_addr_i[29:25];
	assign tagMode 		 = instr_addr_i[31];
	assign tagMemory	 = instr_addr_i[30];
	
	logic [31:0] relocate_code;
	logic isDrvCode;
	logic isAppCode;
	logic isKernelCode;
	
	always_comb
	begin
	    	isDrvCode = 1'b0;
		isAppCode = 1'b0;
		isKernelCode = 1'b0;
		
		//MM: Check the pmp_privil_mode_i as well!!!
		if(tagMode == 1'b1) // user space 
			begin
				if(tagDriver != 5'b00000) //if MWM module
					begin
						//First check if PC and instr tags are different
						//if Yes, only if flag = 1 check the DataLATag (tagDriverTramp) with the tagDriver; flag = 0
						//also if Yes it can be a return.
						if (/*(instr_addr_i >= drv_start_addr_code[tagDriver]) && */(instr_addr_i <= drv_stop_addr_code[tagDriver]) /* (allDrv &= 1 << tagDriver) != '0' */)
								begin
									relocate_code = drv_rel_addr_code[tagDriver];
									isDrvCode = 1'b1; 									
								end	
					end // if MWM module	
			    else  // APP module 
					if((instr_addr_i >= start_addr[`PRATERV_CAR]) && (instr_addr_i <= end_addr[`PRATERV_CAR]))		
						begin
							relocate_code = PRATERV_CRR;
							isAppCode = 1'b1;
						end	
			end
		//MM: Check the pmp_privil_mode_i as well!!!
		else //if(!= 1'b1) // kernel space or monolithic!		
			begin
			    relocate_code = 32'b0;				
			    isKernelCode = 1'b1;
			end					
	end // always_comb

 

   
   assign valid_mono_instruction_address = ((instr_addr_i >= start_addr[`PRATERV_CAR]) && (instr_addr_i <= end_addr[`PRATERV_CAR]));
   assign valid_mono_lib_address = ((instr_addr_i >= start_addr[`PRATERV_LAR]) && (instr_addr_i <= end_addr[`PRATERV_LAR]));
   
   assign instr_access_fault = !(valid_mono_instruction_address || isDrvCode || valid_mono_lib_address || isAppCode ) && ((opcodeCode == OPCODE_BRANCH) || (opcodeCode == OPCODE_JALR) || (opcodeCode == OPCODE_JAL));
   //MM: Check the opcode for branch, I think I will receive it in a later clock cycle, just as the opcodeData!? 
   
   //assign instr_addr_o = isdrv_caddr ? instr_addr_i + relocate_drvcode : ((pmp_privil_mode_i == PRIV_LVL_M) || is_interrupt == 1 || valid_lib_address ) ? instr_addr_i : instr_addr_i + relocate_drvcode;
   
   //assign instr_addr_o = ((pmp_privil_mode_i == PRIV_LVL_M) || is_interrupt == 1 || valid_lib_address ) ? instr_addr_i : instr_addr_i + relocate_drvcode;
   
   assign instr_addr_o =  instr_addr_i + relocate_code;
   
   


 

   always_comb
   begin
      if(pmp_privil_mode_i == PRIV_LVL_M)
      begin
         instr_req_o   = instr_req_i;
         instr_gnt_o   = instr_gnt_i;
         instr_err_o   = 1'b0;

      end
      else
      begin
            if(instr_access_fault == 1'b1)
            begin
               instr_req_o   = 1'b0;
               instr_err_o   = instr_req_i;
               instr_gnt_o   = 1'b0;
            end
            else
            begin
               instr_req_o   =  instr_req_i;
               instr_err_o   =  1'b0;
               instr_gnt_o   =  instr_gnt_i;
            end
      end
   end
   
   // [MM Begin] The next peripheral protection code is not yet active!
	assign periph_index = (data_addr_i - PERIPHERAL_BASE) / PERIPHERAL_SIZE;
	assign peripheral_access_fault = /*(PRATERV_PERIPH[periph_index] == 0) && */ isPeripheral && ((opcodeData == OPCODE_STORE) || (opcodeData == OPCODE_LOAD));	
	assign periph_en = 1'b1; //SMARTV_PERIPH[periph_index]; 
  // [MM End] When getResource is implemented with peripheral protection
 
endmodule
