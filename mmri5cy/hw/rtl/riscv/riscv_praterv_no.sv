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

module riscv_praterv_no
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

	

// The next code portion only forwards the Data Address signals between LSU and Data Memory 
// Just for testing, when no protection is needed
	
    assign data_addr_o  = data_addr_i;	
	assign data_req_o   = data_req_i;
    assign data_gnt_o   = data_gnt_i;
    //assign data_err_int = 1'b0;
	assign data_err_o   = 1'b0;	
	//assign periph_en_o 	   = {PERIPHERAL_NO{1'b1}};
 	
  
// The next code portion only forwards the Instruction Address signals between Prefetch Buffer and Instruction Memory 
// Just for testing, when no protection is needed

   assign instr_addr_o  = instr_addr_i;      
   assign instr_req_o   = instr_req_i;
   assign instr_gnt_o   = instr_gnt_i;
   assign instr_err_o   = 1'b0;
   assign periph_en_o 	   = {PERIPHERAL_NO{1'b1}};


endmodule