
// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Matthias Baer - baermatt@student.ethz.ch                   //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Andreas Traber - atraber@student.ethz.ch                   //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                 Michael Gautschi - gautschi@iis.ee.ethz.ch                 //
//                 Davide Schiavone - pschiavo@iis.ee.ethz.ch                 //
//                                                                            //
// Design Name:    Top level module                                           //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Top level module of the RISC-V core.                       //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

import riscv_defines::*;

module riscv_core
#( `ifdef  SMARTV_EN 
  parameter N_PMP_ENTRIES = `SMARTV_N_PMP_ENTRIES,  // code, data, lib, periph, periph_bb, bb
	`elsif PRATERV_EN
  parameter N_PMP_ENTRIES = `PRATERV_N_PMP_ENTRIES,  // code, data, lib, periph, periph_bb, bb
	`else
  parameter N_PMP_ENTRIES = 16,
	`endif
  parameter INSTR_RDATA_WIDTH   = 32,  
  parameter SHARED_DSP_MULT     =  0,
  parameter SHARED_INT_DIV      =  0,
  parameter PERIPHERAL_NO		=  2)
(
  // Clock and Reset
  input  logic                         clk_i         ,
  input  logic                         rst_ni        ,
  input  logic                         clock_en_i    ,     // enable clock, otherwise it is gated
  input  logic                         test_en_i     ,     // enable all clock gates for testing

  // Core ID, Cluster ID and boot address are considered more or less static
  input  logic [31:0] boot_addr_i,
  input  logic [ 3:0] core_id_i,
  input  logic [ 5:0] cluster_id_i,

  // Instruction memory interface
  output logic                         instr_req_o   ,
  input  logic                         instr_gnt_i   ,
  input  logic                         instr_rvalid_i,
  output logic [31:0]                  instr_addr_o  ,
  input  logic [INSTR_RDATA_WIDTH-1:0] instr_rdata_i ,

  // Data memory interface
  output logic                         data_req_o    ,
  input  logic                         data_gnt_i    ,
  input  logic                         data_rvalid_i ,
  output logic                         data_we_o     ,
  output logic [3:0]                   data_be_o     ,
  output logic [31:0]                  data_addr_o   ,
  output logic [31:0]                  data_wdata_o  ,
  input  logic [31:0]                  data_rdata_i  ,

  // Interrupt inputs
  input  logic                         irq_i        ,  // [MM] For now it is only gpio_ie
  input  logic [4:0]                   irq_id_i     ,
  output logic                         irq_ack_o    ,
  output logic [4:0]                   irq_id_o     ,
  input  logic                         irq_sec_i    ,

  output logic                         sec_lvl_o    ,

  // CPU Control Signals
  input  logic        				  fetch_enable_i,
  output logic        				  core_busy_o   ,

  output logic [PERIPHERAL_NO-1:0]    periph_en_o `ifdef  SMARTV_EN ,
  output							  is_bitBanded_o,
  output [31:0] p0_bbwmask_o
  `endif
  //output 						      bbError		 	  

  //input  logic [N_EXT_PERF_COUNTERS-1:0] ext_perf_counters_i
);

  // IF/ID signals
  logic              instr_valid_id;
  logic [31:0]       instr_rdata_id;    // Instruction sampled inside IF stage
  logic              is_compressed_id;
  logic              is_fetch_failed_id;
  logic              illegal_c_insn_id; // Illegal compressed instruction sent to ID stage
  logic [31:0]       pc_if;             // Program counter in IF stage
  logic [31:0]       pc_id;             // Program counter in ID stage

  logic              clear_instr_valid;
  logic              pc_set;
  logic [2:0]        pc_mux_id;     // Mux selector for next PC
  logic [2:0]        exc_pc_mux_id; // Mux selector for exception PC
  logic [5:0]        exc_cause;
  logic              trap_addr_mux;
  logic              lsu_load_err;
  logic              lsu_store_err;

  // ID performance counter signals
  logic        is_decoding;

  logic        useincr_addr_ex;   // Active when post increment
  logic        data_misaligned;

  logic        mult_multicycle;

  // Jump and branch target and decision (EX->IF)
  logic [31:0] jump_target_id, jump_target_ex;
  logic        branch_in_ex;
  logic        branch_decision;

  logic        ctrl_busy;
  logic        if_busy;
  logic        lsu_busy;

  logic [31:0] pc_ex; // PC of last executed branch or p.elw

  // ALU Control
  logic        alu_en_ex;
  logic [ALU_OP_WIDTH-1:0] alu_operator_ex;
  logic [31:0] alu_operand_a_ex;
  logic [31:0] alu_operand_b_ex;
  logic [31:0] alu_operand_c_ex;
  logic [ 4:0] bmask_a_ex;
  logic [ 4:0] bmask_b_ex;
  logic [ 1:0] imm_vec_ext_ex;
  logic [ 1:0] alu_vec_mode_ex;

  // Multiplier Control
  logic [ 2:0] mult_operator_ex;
  logic [31:0] mult_operand_a_ex;
  logic [31:0] mult_operand_b_ex;
  logic [31:0] mult_operand_c_ex;
  logic        mult_en_ex;
  logic        mult_sel_subword_ex;
  logic [ 1:0] mult_signed_mode_ex;
  logic [ 4:0] mult_imm_ex;
  logic [31:0] mult_dot_op_a_ex;
  logic [31:0] mult_dot_op_b_ex;
  logic [31:0] mult_dot_op_c_ex;
  logic [ 1:0] mult_dot_signed_ex;
  
  // Register Write Control
  logic [5:0]  regfile_waddr_ex;
  logic        regfile_we_ex;
  logic [5:0]  regfile_waddr_fw_wb_o;        // From WB to ID
  logic        regfile_we_wb;
  logic [31:0] regfile_wdata;

  logic [5:0]  regfile_alu_waddr_ex;
  logic        regfile_alu_we_ex;

  logic [5:0]  regfile_alu_waddr_fw;
  logic        regfile_alu_we_fw;
  logic [31:0] regfile_alu_wdata_fw;

  // CSR control
  logic        csr_access_ex;
  logic  [1:0] csr_op_ex;
  logic [23:0] mtvec, utvec;

  logic        csr_access;
  logic  [1:0] csr_op;
  logic [11:0] csr_addr;
  logic [11:0] csr_addr_int;
  logic [31:0] csr_rdata;
  logic [31:0] csr_wdata;
  PrivLvl_t    current_priv_lvl;

  // Data Memory Control:  From ID stage (id-ex pipe) <--> load store unit
  logic        data_we_ex;
  logic [1:0]  data_type_ex;
  logic        data_sign_ext_ex;
  logic [1:0]  data_reg_offset_ex;
  logic        data_req_ex;
  logic        data_load_event_ex;
  logic        data_misaligned_ex;

  logic [31:0] lsu_rdata;

  // stall control
  logic        halt_if;
  logic        id_ready;
  logic        ex_ready;

  logic        id_valid;
  logic        ex_valid;
  logic        wb_valid;

  logic        lsu_ready_ex;
  logic        lsu_ready_wb;

  // Signals between instruction core interface and pipe (if and id stages)
  logic        instr_req_int;    // Id stage asserts a req to instruction core interface

  // Interrupts
  logic        m_irq_enable, u_irq_enable;
  logic        csr_irq_sec;
  logic [31:0] mepc, uepc;

  logic        csr_save_cause;
  logic        csr_save_if;
  logic        csr_save_id;
  logic        csr_save_ex;
  logic [5:0]  csr_cause;
  logic        csr_restore_mret_id;
  //logic 	   [PERIPHERAL_NO-1:0]  	   periph_en_tmp;
  //logic        csr_restore_uret_id;


  //core busy signals
  logic        core_ctrl_firstfetch, core_busy_int, core_busy_q;

  // internal pending interrupt csr register
  logic [31:0]     mip;



//-----------------------MM---------------------------

  //pmp signals
`ifdef  PRATERV_EN
  logic  [N_PMP_ENTRIES-1:0] [31:0] pmp_addr;  
  logic  [15:0] [31:0] drv_addr;
`endif 


`ifdef  SMARTV_EN
  logic  [N_PMP_ENTRIES-1:0] [31:0] pmp_addr;  
`endif 
  
  

  logic                             data_req_pmp;
  logic [31:0]                      data_addr_pmp;
  logic                             data_we_pmp;
  logic                             data_gnt_pmp;
  
  logic                             data_err_pmp;
  logic                             data_err_ack;
  logic                             instr_req_pmp;
  logic                             instr_gnt_pmp;
  logic [31:0]                      instr_addr_pmp;
  logic                             instr_err_pmp;


  logic [6:0] opcode_tmp_delayed;
  logic [6:0] opcode_tmp;
  
  always_ff @(posedge clk_i, negedge rst_ni)
  begin
    if (rst_ni == 1'b0) begin
      opcode_tmp_delayed <= 1'b0;
    end else begin
      opcode_tmp_delayed <= opcode_tmp;
    end
  end



  //Simchecker signal
  logic is_interrupt;
  assign is_interrupt = (pc_mux_id == PC_EXCEPTION) && (exc_pc_mux_id == EXC_PC_IRQ);

  //////////////////////////////////////////////////////////////////////////////////////////////
  //   ____ _            _      __  __                                                   _    //
  //  / ___| | ___   ___| | __ |  \/  | __ _ _ __   __ _  __ _  ___ _ __ ___   ___ _ __ | |_  //
  // | |   | |/ _ \ / __| |/ / | |\/| |/ _` | '_ \ / _` |/ _` |/ _ \ '_ ` _ \ / _ \ '_ \| __| //
  // | |___| | (_) | (__|   <  | |  | | (_| | | | | (_| | (_| |  __/ | | | | |  __/ | | | |_  //
  //  \____|_|\___/ \___|_|\_\ |_|  |_|\__,_|_| |_|\__,_|\__, |\___|_| |_| |_|\___|_| |_|\__| //
  //                                                     |___/                                //
  //////////////////////////////////////////////////////////////////////////////////////////////

  logic        clk;

  logic        clock_en;


  logic        sleeping;

  assign core_busy_o = core_ctrl_firstfetch ? 1'b1 : core_busy_q;

  // if we are sleeping on a barrier let's just wait on the instruction
  // interface to finish loading instructions
  assign core_busy_int = (data_load_event_ex & data_req_o) ? (if_busy) : (if_busy | ctrl_busy | lsu_busy);

  assign clock_en      =  irq_i | core_busy_o;

  assign sleeping      = ~core_busy_o;


  always_ff @(posedge clk_i, negedge rst_ni)
  begin
    if (rst_ni == 1'b0) begin
      core_busy_q <= 1'b0;
    end else begin
      core_busy_q <= core_busy_int;
    end
  end

  // cluster_clock_gating had no function inside
  assign clk = clk_i ;

  //////////////////////////////////////////////////
  //   ___ _____   ____ _____  _    ____ _____    //
  //  |_ _|  ___| / ___|_   _|/ \  / ___| ____|   //
  //   | || |_    \___ \ | | / _ \| |  _|  _|     //
  //   | ||  _|    ___) || |/ ___ \ |_| | |___    //
  //  |___|_|     |____/ |_/_/   \_\____|_____|   //
  //                                              //
  //////////////////////////////////////////////////
  riscv_if_stage if_stage_i
  (
    .clk                 ( clk               ),
    .rst_n               ( rst_ni            ),
	
    // boot address
    .boot_addr_i         ( boot_addr_i[31:1] ),

    // trap vector location
    .m_trap_base_addr_i  ( mtvec             ),
    .u_trap_base_addr_i  ( utvec             ),
    .trap_addr_mux_i     ( trap_addr_mux     ),

    // instruction request control
    .req_i               ( instr_req_int     ),
	
	//-----------------MM---------------
	 // instruction cache interface PMP
	 
    .instr_req_o         ( instr_req_pmp     ),
    .instr_addr_o        ( instr_addr_pmp    ),
    .instr_gnt_i         ( instr_gnt_pmp     ),
    .instr_rvalid_i      ( instr_rvalid_i    ),
    .instr_rdata_i       ( instr_rdata_i     ),
    .instr_err_pmp_i     ( instr_err_pmp     ),
	
   //-----------------MM---------------

    // instruction cache interface
	/*
    .instr_req_o         ( instr_req_o       ),
    .instr_addr_o        ( instr_addr_o      ),
    .instr_gnt_i         ( instr_gnt_i       ),
    .instr_rvalid_i      ( instr_rvalid_i    ),
    .instr_rdata_i       ( instr_rdata_i     ),
	*/

    // outputs to ID stage
    .instr_valid_id_o    ( instr_valid_id    ),
    .instr_rdata_id_o    ( instr_rdata_id    ),
    .is_compressed_id_o  ( is_compressed_id  ),
    .illegal_c_insn_id_o ( illegal_c_insn_id ),
    .pc_if_o             ( pc_if             ),
    .pc_id_o             ( pc_id             ),
    .is_fetch_failed_o   ( is_fetch_failed_id ),

    // control signals
    .clear_instr_valid_i ( clear_instr_valid ),
    .pc_set_i            ( pc_set            ),

    .mepc_i              ( mepc              ), // exception return address
    .uepc_i              ( uepc              ), // exception return address

    .pc_mux_i            ( pc_mux_id         ), // sel for pc multiplexer
    .exc_pc_mux_i        ( exc_pc_mux_id     ),
    .exc_vec_pc_mux_i    ( exc_cause[4:0]    ),

    // Jump targets
    .jump_target_id_i    ( jump_target_id    ),
    .jump_target_ex_i    ( jump_target_ex    ),

    // pipeline stalls
    .halt_if_i           ( halt_if           ),
    .id_ready_i          ( id_ready          ),

    .if_busy_o           ( if_busy           ),
    .perf_imiss_o        ( perf_imiss        )
  );


  /////////////////////////////////////////////////
  //   ___ ____    ____ _____  _    ____ _____   //
  //  |_ _|  _ \  / ___|_   _|/ \  / ___| ____|  //
  //   | || | | | \___ \ | | / _ \| |  _|  _|    //
  //   | || |_| |  ___) || |/ ___ \ |_| | |___   //
  //  |___|____/  |____/ |_/_/   \_\____|_____|  //
  //                                             //
  /////////////////////////////////////////////////
  //                                             // 
  //  Instruction Decode Stage                   //
  //                                             //
  /////////////////////////////////////////////////
  
  riscv_id_stage id_stage_i
  (
    .clk                          ( clk                  ),
    .rst_n                        ( rst_ni               ),
    .test_en_i                    ( test_en_i            ),

    // Processor Enable
    .fetch_enable_i               ( fetch_enable_i       ),
    .ctrl_busy_o                  ( ctrl_busy            ),
    .core_ctrl_firstfetch_o       ( core_ctrl_firstfetch ),
    .is_decoding_o                ( is_decoding          ),

    // Interface to instruction memory
    .instr_valid_i                ( instr_valid_id       ),
    .instr_rdata_i                ( instr_rdata_id       ),
    .instr_req_o                  ( instr_req_int        ),

    // Jumps and branches
    .branch_in_ex_o               ( branch_in_ex         ),
    .branch_decision_i            ( branch_decision      ),
    .jump_target_o                ( jump_target_id       ),

    // IF and ID control signals
    .clear_instr_valid_o          ( clear_instr_valid    ),
    .pc_set_o                     ( pc_set               ),
    .pc_mux_o                     ( pc_mux_id            ),
    .exc_pc_mux_o                 ( exc_pc_mux_id        ),
    .exc_cause_o                  ( exc_cause            ),
    .trap_addr_mux_o              ( trap_addr_mux        ),
    .illegal_c_insn_i             ( illegal_c_insn_id    ),
    .is_compressed_i              ( is_compressed_id     ),
    .is_fetch_failed_i            ( is_fetch_failed_id   ),

    .pc_if_i                      ( pc_if                ),
    .pc_id_i                      ( pc_id                ),

    // Stalls
    .halt_if_o                    ( halt_if              ),

    .id_ready_o                   ( id_ready             ),
    .ex_ready_i                   ( ex_ready             ),
    .wb_ready_i                   ( lsu_ready_wb         ),

    .id_valid_o                   ( id_valid             ),
    .ex_valid_i                   ( ex_valid             ),

    // From the Pipeline ID/EX
    .pc_ex_o                      ( pc_ex                ),

    .alu_en_ex_o                  ( alu_en_ex            ),
    .alu_operator_ex_o            ( alu_operator_ex      ),
    .alu_operand_a_ex_o           ( alu_operand_a_ex     ),
    .alu_operand_b_ex_o           ( alu_operand_b_ex     ),
    .alu_operand_c_ex_o           ( alu_operand_c_ex     ),
    .bmask_a_ex_o                 ( bmask_a_ex           ),
    .bmask_b_ex_o                 ( bmask_b_ex           ),

    .regfile_waddr_ex_o           ( regfile_waddr_ex     ),
    .regfile_we_ex_o              ( regfile_we_ex        ),

    .regfile_alu_we_ex_o          ( regfile_alu_we_ex    ),
    .regfile_alu_waddr_ex_o       ( regfile_alu_waddr_ex ),

    // MUL
    .mult_operator_ex_o           ( mult_operator_ex     ), // from ID to EX stage
    .mult_en_ex_o                 ( mult_en_ex           ), // from ID to EX stage
    .mult_sel_subword_ex_o        ( mult_sel_subword_ex  ), // from ID to EX stage
    .mult_signed_mode_ex_o        ( mult_signed_mode_ex  ), // from ID to EX stage
    .mult_operand_a_ex_o          ( mult_operand_a_ex    ), // from ID to EX stage
    .mult_operand_b_ex_o          ( mult_operand_b_ex    ), // from ID to EX stage
    .mult_operand_c_ex_o          ( mult_operand_c_ex    ), // from ID to EX stage
    .mult_imm_ex_o                ( mult_imm_ex          ), // from ID to EX stage
	
  

    // CSR ID/EX
    .csr_access_ex_o              ( csr_access_ex        ),
    .csr_op_ex_o                  ( csr_op_ex            ),
    .current_priv_lvl_i           ( current_priv_lvl     ),
    .csr_irq_sec_o                ( csr_irq_sec          ),
    .csr_cause_o                  ( csr_cause            ),
    .csr_save_if_o                ( csr_save_if          ), // control signal to save pc
    .csr_save_id_o                ( csr_save_id          ), // control signal to save pc
    .csr_save_ex_o                ( csr_save_ex          ), // control signal to save pc
    .csr_restore_mret_id_o        ( csr_restore_mret_id  ), // control signal to restore pc
    //.csr_restore_uret_id_o        ( csr_restore_uret_id  ), // control signal to restore pc
    .csr_save_cause_o             ( csr_save_cause       ),

    // LSU
    .data_req_ex_o                ( data_req_ex          ), // to load store unit
    .data_we_ex_o                 ( data_we_ex           ), // to load store unit
    .data_type_ex_o               ( data_type_ex         ), // to load store unit
    .data_sign_ext_ex_o           ( data_sign_ext_ex     ), // to load store unit
    .data_reg_offset_ex_o         ( data_reg_offset_ex   ), // to load store unit
    .data_load_event_ex_o         ( data_load_event_ex   ), // to load store unit

    .data_misaligned_ex_o         ( data_misaligned_ex   ), // to load store unit

    .prepost_useincr_ex_o         ( useincr_addr_ex      ),
    .data_misaligned_i            ( data_misaligned      ),
	
	//-----------------MM---------------
	
    .data_err_i                   ( data_err_pmp         ),
    .data_err_ack_o               ( data_err_ack         ),	
	
	//-----------------MM---------------
	
	
    // Interrupt Signals
    .irq_i                        ( irq_i                ), // incoming interrupts
    .irq_sec_i                    ( 1'b0 ),
    .irq_id_i                     ( irq_id_i             ),
    .m_irq_enable_i               ( m_irq_enable         ),
    .u_irq_enable_i               ( u_irq_enable         ),
    .irq_ack_o                    ( irq_ack_o            ),
    .irq_id_o                     ( irq_id_o             ),

    // Forward Signals
    .regfile_waddr_wb_i           ( regfile_waddr_fw_wb_o),  // Write address ex-wb pipeline
    .regfile_we_wb_i              ( regfile_we_wb        ),  // write enable for the register file
    .regfile_wdata_wb_i           ( regfile_wdata        ),  // write data to commit in the register file

    .regfile_alu_waddr_fw_i       ( regfile_alu_waddr_fw ),
    .regfile_alu_we_fw_i          ( regfile_alu_we_fw    ),
    .regfile_alu_wdata_fw_i       ( regfile_alu_wdata_fw ),

    // from ALU
    .mult_multicycle_i            ( mult_multicycle      ),

    .mip_i                        ( mip              ),
	.opcode 					  (opcode_tmp)
  );


  /////////////////////////////////////////////////////
  //   _______  __  ____ _____  _    ____ _____      //
  //  | ____\ \/ / / ___|_   _|/ \  / ___| ____|     //
  //  |  _|  \  /  \___ \ | | / _ \| |  _|  _|       //
  //  | |___ /  \   ___) || |/ ___ \ |_| | |___      //
  //  |_____/_/\_\ |____/ |_/_/   \_\____|_____|     //
  //                                                 //
  /////////////////////////////////////////////////////
  riscv_ex_stage
  #(
   .SHARED_INT_DIV   ( SHARED_INT_DIV     )
  )
  ex_stage_i
  (
    // Global signals: Clock and active low asynchronous reset
    .clk                        ( clk                          ),
    .rst_n                      ( rst_ni                       ),

    // Alu signals from ID stage
    .alu_en_i                   ( alu_en_ex                    ),
    .alu_operator_i             ( alu_operator_ex              ), // from ID/EX pipe registers
    .alu_operand_a_i            ( alu_operand_a_ex             ), // from ID/EX pipe registers
    .alu_operand_b_i            ( alu_operand_b_ex             ), // from ID/EX pipe registers
    .alu_operand_c_i            ( alu_operand_c_ex             ), // from ID/EX pipe registers
    .bmask_a_i                  ( bmask_a_ex                   ), // from ID/EX pipe registers
    .bmask_b_i                  ( bmask_b_ex                   ), // from ID/EX pipe registers
    .imm_vec_ext_i              ( imm_vec_ext_ex               ), // from ID/EX pipe registers
    .alu_vec_mode_i             ( alu_vec_mode_ex              ), // from ID/EX pipe registers

    // Multipler
    .mult_operator_i            ( mult_operator_ex             ), // from ID/EX pipe registers
    .mult_operand_a_i           ( mult_operand_a_ex            ), // from ID/EX pipe registers
    .mult_operand_b_i           ( mult_operand_b_ex            ), // from ID/EX pipe registers
    .mult_operand_c_i           ( mult_operand_c_ex            ), // from ID/EX pipe registers
    .mult_en_i                  ( mult_en_ex                   ), // from ID/EX pipe registers
    .mult_sel_subword_i         ( mult_sel_subword_ex          ), // from ID/EX pipe registers
    .mult_signed_mode_i         ( mult_signed_mode_ex          ), // from ID/EX pipe registers
    .mult_imm_i                 ( mult_imm_ex                  ), // from ID/EX pipe registers

    .mult_multicycle_o          ( mult_multicycle              ), // to ID/EX pipe registers
	
	
   

    .lsu_en_i                   ( data_req_ex                  ),
    .lsu_rdata_i                ( lsu_rdata                    ),

    // interface with CSRs
    .csr_access_i               ( csr_access_ex                ),
    .csr_rdata_i                ( csr_rdata                    ),

    // From ID Stage: Regfile control signals
    .branch_in_ex_i             ( branch_in_ex                 ),
    .regfile_alu_waddr_i        ( regfile_alu_waddr_ex         ),
    .regfile_alu_we_i           ( regfile_alu_we_ex            ),

    .regfile_waddr_i            ( regfile_waddr_ex             ),
    .regfile_we_i               ( regfile_we_ex                ),

    // Output of ex stage pipeline
    .regfile_waddr_wb_o         ( regfile_waddr_fw_wb_o        ),
    .regfile_we_wb_o            ( regfile_we_wb                ),
    .regfile_wdata_wb_o         ( regfile_wdata                ),

    // To IF: Jump and branch target and decision
    .jump_target_o              ( jump_target_ex               ),
    .branch_decision_o          ( branch_decision              ),

    // To ID stage: Forwarding signals
    .regfile_alu_waddr_fw_o     ( regfile_alu_waddr_fw         ),
    .regfile_alu_we_fw_o        ( regfile_alu_we_fw            ),
    .regfile_alu_wdata_fw_o     ( regfile_alu_wdata_fw         ),

    // stall control
    .lsu_ready_ex_i             ( lsu_ready_ex                 ),
	
	//-----------------MM---------------
	/*
    .lsu_err_i                  ( data_err_pmp                 ),
	*/
	//-----------------MM---------------
	

    .ex_ready_o                 ( ex_ready                     ),
    .ex_valid_o                 ( ex_valid                     ),
    .wb_ready_i                 ( lsu_ready_wb                 )
  );


  ////////////////////////////////////////////////////////////////////////////////////////
  //    _     ___    _    ____    ____ _____ ___  ____  _____   _   _ _   _ ___ _____   //
  //   | |   / _ \  / \  |  _ \  / ___|_   _/ _ \|  _ \| ____| | | | | \ | |_ _|_   _|  //
  //   | |  | | | |/ _ \ | | | | \___ \ | || | | | |_) |  _|   | | | |  \| || |  | |    //
  //   | |__| |_| / ___ \| |_| |  ___) || || |_| |  _ <| |___  | |_| | |\  || |  | |    //
  //   |_____\___/_/   \_\____/  |____/ |_| \___/|_| \_\_____|  \___/|_| \_|___| |_|    //
  //                                                                                    //
  ////////////////////////////////////////////////////////////////////////////////////////

  riscv_load_store_unit  load_store_unit_i
  (
    .clk                   ( clk                ),
    .rst_n                 ( rst_ni             ),
	
	//-----------------MM------------------
	//output to data memory PMP
	
    .data_req_o            ( data_req_pmp       ),
    .data_gnt_i            ( data_gnt_pmp       ),
    .data_rvalid_i         ( data_rvalid_i      ),
    .data_err_i            ( data_err_pmp       ),

    .data_addr_o           ( data_addr_pmp      ),
    .data_we_o             ( data_we_o          ),
    .data_be_o             ( data_be_o          ),
    .data_wdata_o          ( data_wdata_o       ),
    .data_rdata_i          ( data_rdata_i       ),
	
	//-----------------MM------------------

    //output to data memory
	/* Remove this comment if an MPU is not there!!!
    .data_req_o            ( data_req_o         ),
    .data_gnt_i            ( data_gnt_i         ),
    .data_rvalid_i         ( data_rvalid_i      ),

    .data_addr_o           ( data_addr_o        ),
    .data_we_o             ( data_we_o          ),
    .data_be_o             ( data_be_o          ),
    .data_wdata_o          ( data_wdata_o       ),
    .data_rdata_i          ( data_rdata_i       ),
	*/
    // signal from ex stage
    .data_we_ex_i          ( data_we_ex         ),
    .data_type_ex_i        ( data_type_ex       ),
    .data_wdata_ex_i       ( alu_operand_c_ex   ),
    .data_reg_offset_ex_i  ( data_reg_offset_ex ),
    .data_sign_ext_ex_i    ( data_sign_ext_ex   ),  // sign extension

    .data_rdata_ex_o       ( lsu_rdata          ),
    .data_req_ex_i         ( data_req_ex        ),
    .operand_a_ex_i        ( alu_operand_a_ex   ),
    .operand_b_ex_i        ( alu_operand_b_ex   ),
    .addr_useincr_ex_i     ( useincr_addr_ex    ),

    .data_misaligned_ex_i  ( data_misaligned_ex ), // from ID/EX pipeline
    .data_misaligned_o     ( data_misaligned    ),

    // control signals
    .lsu_ready_ex_o        ( lsu_ready_ex       ),
    .lsu_ready_wb_o        ( lsu_ready_wb       ),

    .ex_valid_i            ( ex_valid           ),
    .busy_o                ( lsu_busy           )
  );

  assign wb_valid = lsu_ready_wb;

  //////////////////////////////////////
  //        ____ ____  ____           //
  //       / ___/ ___||  _ \ ___      //
  //      | |   \___ \| |_) / __|     //
  //      | |___ ___) |  _ <\__ \     //
  //       \____|____/|_| \_\___/     //
  //                                  //
  //   Control and Status Registers   //
  //////////////////////////////////////

  riscv_cs_registers  
  cs_registers_i
  (
    .clk                     ( clk                ),
    .rst_n                   ( rst_ni             ),

    // Core and Cluster ID from outside
    .core_id_i               ( core_id_i          ),
    .cluster_id_i            ( cluster_id_i       ),
    .mtvec_o                 ( mtvec              ),
    .utvec_o                 ( utvec              ),
    // boot address
    .boot_addr_i             ( boot_addr_i[31:1]  ),
    // Interface to CSRs (SRAM like)
    .csr_access_i            ( csr_access         ),
    .csr_addr_i              ( csr_addr           ),
    .csr_wdata_i             ( csr_wdata          ),
    .csr_op_i                ( csr_op             ),
    .csr_rdata_o             ( csr_rdata          ),

    // Interrupt related control signals
    .m_irq_enable_o          ( m_irq_enable       ),
    .u_irq_enable_o          ( u_irq_enable       ),
    .csr_irq_sec_i           ( csr_irq_sec        ),
    .sec_lvl_o               ( sec_lvl_o          ),
    .mepc_o                  ( mepc               ),
    .uepc_o                  ( uepc               ),
    .priv_lvl_o              ( current_priv_lvl   ),

    .mip_o                   ( mip                ),
	
	//---------------------MM---------------------------
	
	.pmp_addr_o              ( pmp_addr           ),    
	.drv_addr_o 			 (/*drv_addr*/ 		  ),
	
	//---------------------MM---------------------------

    .pc_if_i                 ( pc_if              ),
    .pc_id_i                 ( pc_id              ),
    .pc_ex_i                 ( pc_ex              ),

    .csr_save_if_i           ( csr_save_if        ),
    .csr_save_id_i           ( csr_save_id        ),
    .csr_save_ex_i           ( csr_save_ex        ),
    .csr_restore_mret_i      ( csr_restore_mret_id ),
    //.csr_restore_uret_i      ( csr_restore_uret_id ),
    .csr_cause_i             ( csr_cause          ),
    .csr_save_cause_i        ( csr_save_cause     )

    // performance counter related signals
    /*.id_valid_i              ( id_valid           ),
    .is_compressed_i         ( is_compressed_id   ),
    .is_decoding_i           ( is_decoding        ),

    .imiss_i                 ( perf_imiss         ),
    .pc_set_i                ( pc_set             ),
    .jump_i                  ( perf_jump          ),
    .branch_i                ( branch_in_ex       ),
    .branch_taken_i          ( branch_decision    ),
    .ld_stall_i              ( perf_ld_stall      ),
    .jr_stall_i              ( perf_jr_stall      ),
    .pipeline_stall_i        ( perf_pipeline_stall ),

    .mem_load_i              ( data_req_o & data_gnt_i & (~data_we_o) ),
    .mem_store_i             ( data_req_o & data_gnt_i & data_we_o    ),

    .ext_counters_i          ( ext_perf_counters_i                    )*/
  );
  
  
  
  ///////////////////////////
  //   ____  __  __ ____   //
  //  |  _ \|  \/  |  _ \  //
  //  | |_) | |\/| | |_) | //
  //  |  __/| |  | |  __/  //
  //  |_|   |_|  |_|_|     //
  //                       //
  ///////////////////////////


`ifdef  PRATERV_EN
//OR riscv_praterv
  riscv_praterv_no
  #(
  
     .N_PMP_ENTRIES(N_PMP_ENTRIES),
	 .PERIPHERAL_NO(PERIPHERAL_NO)
  )
	riscv_praterv_i
  (
    .clk                     ( clk                ),
    .rst_n                   ( rst_ni             ),

    .pmp_privil_mode_i       ( current_priv_lvl   ),

    .pmp_addr_i              ( pmp_addr           ),
	.drv_addr_i              ( drv_addr           ),	
    

    .data_req_i              ( data_req_pmp       ),
    .data_addr_i             ( data_addr_pmp      ),
    .data_we_i               ( data_we_o          ),
    .data_gnt_o              ( data_gnt_pmp       ),

    .data_req_o              ( data_req_o         ),
    .data_gnt_i              ( data_gnt_i         ),
    .data_addr_o             ( data_addr_o        ),
    .data_err_o              ( data_err_pmp       ),
    .data_err_ack_i          ( data_err_ack       ),
	.opcodeData				 ( opcode_tmp_delayed ),
	.opcodeCode 			 ( opcode_tmp),

    .instr_req_i             ( instr_req_pmp      ),
    .instr_addr_i            ( instr_addr_pmp     ),
    .instr_gnt_o             ( instr_gnt_pmp      ),

    .instr_req_o             ( instr_req_o        ),
    .instr_gnt_i             ( instr_gnt_i        ),
    .instr_addr_o            ( instr_addr_o       ),
    .instr_err_o             ( instr_err_pmp      ),
	.is_interrupt            ( is_interrupt		  ),     
	.periph_en_o			 ( periph_en_o        )		 	
  );
`endif 

 
`ifdef SMARTV_EN 
 riscv_smartv
  #(
     .N_PMP_ENTRIES(N_PMP_ENTRIES),
	 .PERIPHERAL_NO(PERIPHERAL_NO)
  )
	riscv_smartv_i
  (
    .clk                     ( clk                ),
    .rst_n                   ( rst_ni             ),

    .pmp_privil_mode_i       ( current_priv_lvl   ),

    .pmp_addr_i              ( pmp_addr           ),
	.drv_addr_i              ( /*drv_addr*/       ),	    

    .data_req_i              ( data_req_pmp       ),
    .data_addr_i             ( data_addr_pmp      ),
    .data_we_i               ( data_we_o          ),
    .data_gnt_o              ( data_gnt_pmp       ),

    .data_req_o              ( data_req_o         ),
    .data_gnt_i              ( data_gnt_i         ),
    .data_addr_o             ( data_addr_o        ),
    .data_err_o              ( data_err_pmp       ),
    .data_err_ack_i          ( data_err_ack       ),
	
	.opcodeData				 ( opcode_tmp_delayed ),
	.opcodeCode 			 ( opcode_tmp),

    .instr_req_i             ( instr_req_pmp      ),
    .instr_addr_i            ( instr_addr_pmp     ),
    .instr_gnt_o             ( instr_gnt_pmp      ),

    .instr_req_o             ( instr_req_o        ),
    .instr_gnt_i             ( instr_gnt_i        ),
    .instr_addr_o            ( instr_addr_o       ),
    .instr_fault_o           ( instr_err_pmp      ),
	.is_interrupt            ( is_interrupt		  ),
	.periph_en_o			 ( periph_en_o        ),
 	.is_bitBanded_o 		 ( is_bitBanded_o	  ),
	.p0_bbwmask				 ( p0_bbwmask_o		  )
    //.bbError			     ( bbError		 	  )		
  );
 `endif 

  // Mux for CSR access through Debug Unit
  assign csr_access   = csr_access_ex    ;
  assign csr_addr     = csr_addr_int     ;
  assign csr_wdata    = alu_operand_a_ex ;
  assign csr_op       = csr_op_ex         ;
  assign csr_addr_int = csr_access_ex ? alu_operand_b_ex[11:0] : '0;

  
 endmodule
