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
// Engineer:       Renzo Andri - andrire@student.ethz.ch                      //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Andreas Traber - atraber@student.ethz.ch                   //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                                                                            //
// Design Name:    Instruction Fetch Stage                                    //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Instruction fetch unit: Selection of the next PC, and      //
//                 buffering (sampling) of the read instruction               //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


import riscv_defines::*;

module riscv_if_stage
(
    input  logic        clk,
    input  logic        rst_n,

    // Used to calculate the exception offsets
    input  logic [23:0] m_trap_base_addr_i,
    input  logic [23:0] u_trap_base_addr_i,
    input  logic        trap_addr_mux_i,
    // Used for boot address
    input  logic [30:0] boot_addr_i,

    // instruction request control
    input  logic        req_i,

    // instruction cache interface
    output logic                   instr_req_o,
    output logic            [31:0] instr_addr_o,
    input  logic                   instr_gnt_i,
    input  logic                   instr_rvalid_i,
    input  logic [31:0] 		   instr_rdata_i,
	//-----------------MM---------------
    input  logic                   instr_err_pmp_i,
	//-----------------MM---------------  
    

    // Output of IF Pipeline stage   
    output logic              instr_valid_id_o,      // instruction in IF/ID pipeline is valid
    output logic       [31:0] instr_rdata_id_o,      // read instruction is sampled and sent to ID stage for decoding
    output logic              is_compressed_id_o,    // compressed decoder thinks this is a compressed instruction
    output logic              illegal_c_insn_id_o,   // compressed decoder thinks this is an invalid instruction
    output logic       [31:0] pc_if_o,
    output logic       [31:0] pc_id_o,
    output logic              is_fetch_failed_o,

    // Forwarding ports - control signals
    input  logic        clear_instr_valid_i,   // clear instruction valid bit in IF/ID pipe
    input  logic        pc_set_i,              // set the program counter to a new value
    input  logic [31:0] mepc_i,    // address used to restore PC when the interrupt/exception is served
    input  logic [31:0] uepc_i,    // address used to restore PC when the interrupt/exception is served
    input  logic  [2:0] pc_mux_i,              // sel for pc multiplexer
    input  logic  [2:0] exc_pc_mux_i,          // selects ISR address
    input  logic  [4:0] exc_vec_pc_mux_i,      // selects ISR address for vectorized interrupt lines

    // jump and branch target and decision
    input  logic [31:0] jump_target_id_i,      // jump target address
    input  logic [31:0] jump_target_ex_i,      // jump target address

  
    // pipeline stall
    input  logic        halt_if_i,
    input  logic        id_ready_i,

    // misc signals
    output logic        if_busy_o,             // is the IF stage busy fetching instructions?
    output logic        perf_imiss_o           // Instruction Fetch Miss
);

  // offset FSM
  enum logic[0:0] {WAIT, IDLE } offset_fsm_cs, offset_fsm_ns;

  logic              if_valid, if_ready;
  logic              valid;

  // prefetch buffer related signals
  logic              prefetch_busy;
  logic              branch_req;
  logic       [31:0] fetch_addr_n;

  logic              fetch_valid;
  logic              fetch_ready;
  logic       [31:0] fetch_rdata;
  logic       [31:0] fetch_addr;
 

  logic       [31:0] exc_pc;


  logic [23:0]       trap_base_addr;
  logic              fetch_failed;


  // exception PC selection mux
  always_comb
  begin : EXC_PC_MUX
    exc_pc = '0;

    unique case (trap_addr_mux_i)
      TRAP_MACHINE: trap_base_addr = m_trap_base_addr_i;
      TRAP_USER:    trap_base_addr = u_trap_base_addr_i;
      default:;
    endcase

    unique case (exc_pc_mux_i)
      EXC_PC_EXCEPTION:            exc_pc = { trap_base_addr, 8'h0 }; //Spec: '1.10 all the exceptions go to base address'
	  
								  // MM: since we have vector_user to jump to when exceptions happen while in U-mode
								  // and vector_machine to jump to when exception happens while in M-mode, 
								  // e.g., accidentally calling ecall, illegal instruction, etc
								  // I modified this 256-byte alignment!!!	
								  // exc_pc = { trap_base_addr, current_priv_lvl_i << 5};
								  
								  /*
									. = 0x100;
										*(.vector_user)
	  
									. = 0x1C0;   // == trap_base_addr, current_priv_lvl_i << 6 = 'b111000000
									    *(.vector_machine)
								  
								  */
								  
								  
      EXC_PC_IRQ:                 exc_pc = { trap_base_addr, 1'b0, exc_vec_pc_mux_i[4:0], 2'b0 }; // MM: interrupts are vectored, thus base address + 4 x interruptID
      default:;
    endcase
  end

  // fetch address selection
  always_comb
  begin
    fetch_addr_n = '0;

    unique case (pc_mux_i)
      PC_BOOT:      fetch_addr_n = {boot_addr_i, 1'b0};
      PC_JUMP:      fetch_addr_n = jump_target_id_i;
      PC_BRANCH:    fetch_addr_n = jump_target_ex_i;
      PC_EXCEPTION: fetch_addr_n = exc_pc;             // set PC to exception handler
      PC_MRET:      fetch_addr_n = mepc_i; // PC is restored when returning from IRQ/exception
      PC_URET:      fetch_addr_n = uepc_i; // PC is restored when returning from IRQ/exception
     

      default:;
    endcase
  end

  generate
     begin : prefetch_32
      // prefetch buffer, caches a fixed number of instructions
      riscv_prefetch_buffer prefetch_buffer_i
      (
        .clk               ( clk                         ),
        .rst_n             ( rst_n                       ),

        .req_i             ( req_i                       ),

        .branch_i          ( branch_req                  ),
        .addr_i            ( {fetch_addr_n[31:1], 1'b0}  ),

        
        .ready_i           ( fetch_ready                 ),
        .valid_o           ( fetch_valid                 ),
        .rdata_o           ( fetch_rdata                 ),
        .addr_o            ( fetch_addr                  ),
       
        // goes to instruction memory / instruction cache
        .instr_req_o       ( instr_req_o                 ),
        .instr_addr_o      ( instr_addr_o                ),
        .instr_gnt_i       ( instr_gnt_i                 ),
        .instr_rvalid_i    ( instr_rvalid_i              ),
		
		//-----------------MM---------------
        .instr_err_pmp_i   ( instr_err_pmp_i             ),
		//-----------------MM---------------
		
        .fetch_failed_o    ( fetch_failed                ),
        .instr_rdata_i     ( instr_rdata_i               ),

        // Prefetch Buffer Status
        .busy_o            ( prefetch_busy               )
      );
   
    end
  endgenerate

  // offset FSM state
  always_ff @(posedge clk, negedge rst_n)
  begin
    if (rst_n == 1'b0) begin
      offset_fsm_cs     <= IDLE;
    end else begin
      offset_fsm_cs     <= offset_fsm_ns;
    end
  end

  // offset FSM state transition logic
  always_comb
  begin
    offset_fsm_ns = offset_fsm_cs;

    fetch_ready   = 1'b0;
    branch_req    = 1'b0;
    valid         = 1'b0;

    unique case (offset_fsm_cs)
      // no valid instruction data for ID stage
      // assume aligned
      IDLE: begin
        if (req_i) begin
          branch_req    = 1'b1;
          offset_fsm_ns = WAIT;
        end
      end

      // serving aligned 32 bit or 16 bit instruction, we don't know yet
      WAIT: begin
        if (fetch_valid) begin
          valid   = 1'b1; // an instruction is ready for ID stage

          if (req_i && if_valid) begin
            fetch_ready   = 1'b1;
            offset_fsm_ns = WAIT;
          end
        end
      end

      default: begin
        offset_fsm_ns = IDLE;
      end
    endcase


    // take care of jumps and branches
    if (pc_set_i) begin
      valid = 1'b0;
      // switch to new PC from ID stage
      branch_req    = 1'b1;
      offset_fsm_ns = WAIT;
    end   
  end



  assign pc_if_o         = fetch_addr;

  assign if_busy_o       = prefetch_busy;

  assign perf_imiss_o    = (~fetch_valid) | branch_req;


  // compressed instruction decoding, or more precisely compressed instruction
  // expander
  //
  // since it does not matter where we decompress instructions, we do it here
  // to ease timing closure
  
  
  logic [31:0] instr_decompressed;
  logic        illegal_c_insn;
  logic        instr_compressed_int;

  riscv_compressed_decoder
  compressed_decoder_i
  (
    .instr_i         ( fetch_rdata          ),
    .instr_o         ( instr_decompressed   ),
    .is_compressed_o ( instr_compressed_int ),
    .illegal_instr_o ( illegal_c_insn       )
  );
  
  


  // IF-ID pipeline registers, frozen when the ID stage is stalled
  always_ff @(posedge clk, negedge rst_n)
  begin : IF_ID_PIPE_REGISTERS
    if (rst_n == 1'b0)
    begin
      instr_valid_id_o      <= 1'b0;
      instr_rdata_id_o      <= '0;
      illegal_c_insn_id_o   <= 1'b0;
      is_compressed_id_o    <= 1'b0;
      pc_id_o               <= '0;     
      is_fetch_failed_o     <= 1'b0;

    end
    else
    begin

      if (if_valid)
      begin
        instr_valid_id_o    <= 1'b1;
        instr_rdata_id_o    <= instr_decompressed;
        illegal_c_insn_id_o <= illegal_c_insn;
        is_compressed_id_o  <= instr_compressed_int;
        pc_id_o             <= pc_if_o;       
        is_fetch_failed_o   <= 1'b0;      

      end else if (clear_instr_valid_i) begin
        instr_valid_id_o    <= 1'b0;
        is_fetch_failed_o   <= fetch_failed;
      end

    end
  end

  assign if_ready = valid & id_ready_i;
  assign if_valid = (~halt_if_i) & if_ready;

  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------
  `ifndef VERILATOR
    // there should never be a grant when there is no request
    assert property (
      @(posedge clk) (instr_gnt_i) |-> (instr_req_o) )
      else $warning("There was a grant without a request");
  `endif
endmodule
