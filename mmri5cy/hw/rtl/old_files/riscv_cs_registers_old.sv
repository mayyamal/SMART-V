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
// Engineer:       Sven Stucki - svstucki@student.ethz.ch                     //
//                                                                            //
// Additional contributions by:                                               //
//                 Andreas Traber - atraber@iis.ee.ethz.ch                    //
//                 Michael Gautschi - gautschi@iis.ee.ethz.ch                 //
//                 Davide Schiavone - pschiavo@iis.ee.ethz.ch                 //
//                                                                            //
// Design Name:    Control and Status Registers                               //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Control and Status Registers (CSRs) loosely following the  //
//                 RiscV draft priviledged instruction set spec (v1.9)        //
//                 Added Floating point support                               //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

import riscv_defines::*;

module riscv_cs_registers
#(
  parameter N_EXT_CNT     = 0,
  parameter PULP_SECURE   = 0,
//-----------------MM---------------  
parameter N_PMP_ENTRIES = 16
//-----------------MM---------------

)
(
  // Clock and Reset
  input  logic            clk,
  input  logic            rst_n,

  // Core and Cluster ID
  input  logic  [3:0]     core_id_i,
  input  logic  [5:0]     cluster_id_i,
  output logic [23:0]     mtvec_o,
  output logic [23:0]     utvec_o,

  // Used for boot address
  input  logic [30:0]     boot_addr_i,

  // Interface to registers (SRAM like)
  input  logic            csr_access_i,
  input  logic [11:0]     csr_addr_i,
  input  logic [31:0]     csr_wdata_i,
  input  logic  [1:0]     csr_op_i,
  output logic [31:0]     csr_rdata_o,

  // Interrupts
  output logic            m_irq_enable_o,
  output logic            u_irq_enable_o,
  //csr_irq_sec_i is always 0 if PULP_SECURE is zero
  input  logic            csr_irq_sec_i,
  output logic            sec_lvl_o,
  output logic [31:0]     mepc_o,
  output logic [31:0]     uepc_o,

  output logic [31:0]     mip_o,


  //-----------------MM---------------

 output logic  [N_PMP_ENTRIES-1:0] [31:0] pmp_addr_o,
  output logic  [15:0] [31:0] drv_addr_o,
// output logic  [N_PMP_ENTRIES-1:0] [7:0]  pmp_cfg_o,

 //-----------------MM---------------

 output PrivLvl_t        priv_lvl_o,

  input  logic [31:0]     pc_if_i,
  input  logic [31:0]     pc_id_i,
  input  logic [31:0]     pc_ex_i,

  input  logic            csr_save_if_i,
  input  logic            csr_save_id_i,
  input  logic            csr_save_ex_i,

  input  logic            csr_restore_mret_i,
  input  logic            csr_restore_uret_i,
  //coming from controller
  input  logic [5:0]      csr_cause_i,
  //coming from controller
  input  logic            csr_save_cause_i,

  // Performance Counters
  input  logic                 id_valid_i,        // ID stage is done
  input  logic                 is_compressed_i,   // compressed instruction in ID
  input  logic                 is_decoding_i,     // controller is in DECODE state

  input  logic                 imiss_i,           // instruction fetch
  input  logic                 pc_set_i,          // pc was set to a new value
  input  logic                 jump_i,            // jump instruction seen   (j, jr, jal, jalr)
  input  logic                 branch_i,          // branch instruction seen (bf, bnf)
  input  logic                 branch_taken_i,    // branch was taken
  input  logic                 ld_stall_i,        // load use hazard
  input  logic                 jr_stall_i,        // jump register use hazard
  input  logic                 pipeline_stall_i,  // extra cycles from elw

  input  logic                 mem_load_i,        // load from memory in this cycle
  input  logic                 mem_store_i,       // store to memory in this cycle

  input  logic [N_EXT_CNT-1:0] ext_counters_i
);

  localparam N_PERF_COUNTERS = 12 + N_EXT_CNT;

  localparam PERF_EXT_ID     = 12;
  localparam MTVEC_MODE      = 2'b01;


 //-----------------MM---------------


// localparam MAX_N_PMP_ENTRIES = 16;
// localparam MAX_N_PMP_CFG     =  4;
// localparam N_PMP_CFG         = N_PMP_ENTRIES % 4 == 0 ? N_PMP_ENTRIES/4 : N_PMP_ENTRIES/4 + 1;
//-----------------MM---------------



`ifdef ASIC_SYNTHESIS
  localparam N_PERF_REGS     = 1;
`else
  localparam N_PERF_REGS     = N_PERF_COUNTERS;
`endif

  `define MSTATUS_UIE_BITS        0
  `define MSTATUS_SIE_BITS        1
  `define MSTATUS_MIE_BITS        3
  `define MSTATUS_UPIE_BITS       4
  `define MSTATUS_SPIE_BITS       5
  `define MSTATUS_MPIE_BITS       7
  `define MSTATUS_SPP_BITS        8
  `define MSTATUS_MPP_BITS    12:11
  `define MSTATUS_MPRV_BITS      17


  typedef struct packed {
    logic uie;
    // logic sie;      - unimplemented, hardwired to '0
    // logic hie;      - unimplemented, hardwired to '0
    logic mie;
    logic upie;
    // logic spie;     - unimplemented, hardwired to '0
    // logic hpie;     - unimplemented, hardwired to '0
    logic mpie;
    // logic spp;      - unimplemented, hardwired to '0
    // logic[1:0] hpp; - unimplemented, hardwired to '0
    PrivLvl_t mpp;
    logic mprv;
  } Status_t;
//-----------------MM---------------  

typedef struct packed {
   logic  [N_PMP_ENTRIES-1:0] [31:0] pmpaddr;
  // logic  [MAX_N_PMP_CFG-1:0]     [31:0] pmpcfg_packed;
  // logic  [MAX_N_PMP_ENTRIES-1:0] [ 7:0] pmpcfg;
  } Pmp_t;

//-----------------MM---------------
   logic  [15:0] [31:0] drvaddr;

  // CSR update logic
  logic [31:0] csr_wdata_int;
  logic [31:0] csr_rdata_int;
  logic        csr_we_int;
  logic [C_RM-1:0]     frm_q, frm_n;
  logic [C_FFLAG-1:0]  fflags_q, fflags_n;
  logic [C_PC-1:0]     fprec_q, fprec_n;

  // Interrupt control signals
  logic [31:0] mepc_q, mepc_n;
  logic [31:0] uepc_q, uepc_n;
  logic [31:0] exception_pc;
  Status_t mstatus_q, mstatus_n;
  logic [ 5:0] mcause_q, mcause_n;
  logic [ 5:0] ucause_q, ucause_n;
  //not implemented yet
  logic [23:0] mtvec_n, mtvec_q;
  logic [23:0] utvec_n, utvec_q;

  logic is_irq;
  PrivLvl_t priv_lvl_n, priv_lvl_q, priv_lvl_reg_q;
//-----------------MM---------------

  Pmp_t pmp_reg_q, pmp_reg_n;
  //clock gating for pmp regs
  logic [N_PMP_ENTRIES-1:0] pmpaddr_we;
 // logic [MAX_N_PMP_ENTRIES-1:0] pmpcfg_we;

//-----------------MM---------------

 

  // Performance Counter Signals
  logic                          id_valid_q;
  logic [N_PERF_COUNTERS-1:0]    PCCR_in;  // input signals for each counter category
  logic [N_PERF_COUNTERS-1:0]    PCCR_inc, PCCR_inc_q; // should the counter be increased?

  logic [N_PERF_REGS-1:0] [31:0] PCCR_q, PCCR_n; // performance counters counter register
  logic [1:0]                    PCMR_n, PCMR_q; // mode register, controls saturation and global enable
  logic [N_PERF_COUNTERS-1:0]    PCER_n, PCER_q; // selected counter input

  logic [31:0]                   perf_rdata;
  logic [4:0]                    pccr_index;
  logic                          pccr_all_sel;
  logic                          is_pccr;
  logic                          is_pcer;
  logic                          is_pcmr;
  
  // TODO REPLACE
  logic [63:0] mcycle;
  logic [63:0] mtime;
  logic [63:0] mtimecmp;
  logic [31:0] mip;
  logic [31:0] mie;

  logic mtimer_expired;
  `define MIE_MTIE_BITS        7
  `define MIP_MTIP_BITS        7

  assign is_irq = csr_cause_i[5];

  ////////////////////////////////////////////
  //   ____ ____  ____    ____              //
  //  / ___/ ___||  _ \  |  _ \ ___  __ _   //
  // | |   \___ \| |_) | | |_) / _ \/ _` |  //
  // | |___ ___) |  _ <  |  _ <  __/ (_| |  //
  //  \____|____/|_| \_\ |_| \_\___|\__, |  //
  //                                |___/   //
  ////////////////////////////////////////////


   genvar j;


if(PULP_SECURE==1) begin
  // read logic
  always_comb
  begin
    case (csr_addr_i)
      // fcsr: Floating-Point Control and Status Register (frm + fflags).
      12'h001: csr_rdata_int = '0;
      12'h002: csr_rdata_int = '0;
      12'h003: csr_rdata_int = '0;
      12'h006: csr_rdata_int = '0; // Optional precision control for FP DIV/SQRT Unit
      // mstatus
      12'h300: csr_rdata_int = {
                                  14'b0,
                                  mstatus_q.mprv,
                                  4'b0,
                                  mstatus_q.mpp,
                                  3'b0,
                                  mstatus_q.mpie,
                                  2'h0,
                                  mstatus_q.upie,
                                  mstatus_q.mie,
                                  2'h0,
                                  mstatus_q.uie
                                };
      // mtvec: machine trap-handler base address
      12'h305: csr_rdata_int = {mtvec_q, 6'h0, MTVEC_MODE};
      // mepc: exception program counter
      12'h341: csr_rdata_int = mepc_q;
      // mcause: exception cause
      12'h342: csr_rdata_int = {mcause_q[5], 26'b0, mcause_q[4:0]};
      // mhartid: unique hardware thread id
      12'hF14: csr_rdata_int = {21'b0, cluster_id_i[5:0], 1'b0, core_id_i[3:0]};
      //-----------------MM---------------


      // PMP config registers
	  /*
      12'h3A0: csr_rdata_int = pmp_reg_q.pmpcfg_packed[0];
      12'h3A1: csr_rdata_int = pmp_reg_q.pmpcfg_packed[1];
      12'h3A2: csr_rdata_int = pmp_reg_q.pmpcfg_packed[2];
      12'h3A3: csr_rdata_int = pmp_reg_q.pmpcfg_packed[3];
	  */
	  `CSR_PMPADDR0: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_CODE_START];//3B0				
	  `CSR_PMPADDR1: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_CODE_END]; //3B1		
	  `CSR_PMPADDR2: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_DATA_START]; //3B2		
	  `CSR_PMPADDR3: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_DATA_END]; //3B3
	  `CSR_PMPADDR4: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_LIB_START]; //3B4		
	  `CSR_PMPADDR5: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_LIB_END]; //3B5
      `CSR_PMPADDR6: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_PERIPH]; //3B6
	  `CSR_PMPADDR7: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_CODE_RELOC]; //3B7
	  `CSR_PMPADDR8: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_DATA_RELOC]; //3B8
	 
      //12'h3Bx: csr_rdata_int = pmp_reg_q.pmpaddr[csr_addr_i[3:0]];
	  
	  
	  
	   `CSR_DRV0: csr_rdata_int = drvaddr[0];  //pmpaddr_we[`PMP_DATA_RELOC] = 1'b1; end //3B8
	  `CSR_DRV1: csr_rdata_int =drvaddr[1];
	  `CSR_DRV2: csr_rdata_int = drvaddr[2] ;
	  `CSR_DRV3: csr_rdata_int = drvaddr[3] ; 
	  `CSR_DRV4: csr_rdata_int = drvaddr[4];
	  `CSR_DRV5: csr_rdata_int = drvaddr[5] ;
	  `CSR_DRV6: csr_rdata_int = drvaddr[6] ;
	  `CSR_DRV7: csr_rdata_int = drvaddr[7] ;
	  `CSR_DRV8: csr_rdata_int = drvaddr[8] ;
	  `CSR_DRV9: csr_rdata_int = drvaddr[9] ;
	  `CSR_DRV10: csr_rdata_int = drvaddr[10] ;
	  `CSR_DRV11: csr_rdata_int = drvaddr[11] ;
	  `CSR_DRV12: csr_rdata_int = drvaddr[12] ;
	  `CSR_DRV13: csr_rdata_int = drvaddr[13] ;
	  `CSR_DRV14: csr_rdata_int = drvaddr[14] ;
	  `CSR_DRV15: csr_rdata_int = drvaddr[15] ;

//-----------------MM---------------

      // machine counter / timer
      12'hB00: csr_rdata_int = mcycle[31:0];
      12'hB80: csr_rdata_int = mcycle[63:32];
      12'hb01: csr_rdata_int = mtime[31:0];
      12'hb41: csr_rdata_int = mtime[63:32];
      12'h321: csr_rdata_int = mtimecmp[31:0]; 
      12'h322: csr_rdata_int = mtimecmp[63:32];
      12'h304: csr_rdata_int = mie;

      /* USER CSR */
      // ustatus
      12'h000: csr_rdata_int = {
                                  27'b0,
                                  mstatus_q.upie,
                                  3'h0,
                                  mstatus_q.uie
                                };
      // utvec: user trap-handler base address
      12'h005: csr_rdata_int = {utvec_q, 6'h0, MTVEC_MODE};
      // dublicated mhartid: unique hardware thread id (not official)
      12'h014: csr_rdata_int = {21'b0, cluster_id_i[5:0], 1'b0, core_id_i[3:0]};
      // uepc: exception program counter
      12'h041: csr_rdata_int = uepc_q;
      // ucause: exception cause
      12'h042: csr_rdata_int = {ucause_q[5], 26'h0, ucause_q[4:0]};
      // user counter / timer
      12'hC00: csr_rdata_int = mcycle[31:0];
      12'hC80: csr_rdata_int = mcycle[63:32];
      // current priv level (not official)
      12'hC10: csr_rdata_int = {30'h0, priv_lvl_q};
      default:
        csr_rdata_int = '0;
    endcase
  end
end else begin //PULP_SECURE == 0
  // read logic
  always_comb
  begin

    case (csr_addr_i)
      // fcsr: Floating-Point Control and Status Register (frm + fflags).
      12'h001: csr_rdata_int = '0;
      12'h002: csr_rdata_int = '0;
      12'h003: csr_rdata_int = '0;
      12'h006: csr_rdata_int = '0; // Optional precision control for FP DIV/SQRT Unit
      // mstatus: always M-mode, contains IE bit
      12'h300: csr_rdata_int = {
                                  14'b0,
                                  mstatus_q.mprv,
                                  4'b0,
                                  mstatus_q.mpp,
                                  3'b0,
                                  mstatus_q.mpie,
                                  2'h0,
                                  mstatus_q.upie,
                                  mstatus_q.mie,
                                  2'h0,
                                  mstatus_q.uie
                                };
      //misa: (no allocated ID yet)
      12'h301: csr_rdata_int = 32'h0;
      // mtvec: machine trap-handler base address
      12'h305: csr_rdata_int = {mtvec_q, 6'h0, MTVEC_MODE};
      // mepc: exception program counter
      12'h341: csr_rdata_int = mepc_q;
      // mcause: exception cause
      12'h342: csr_rdata_int = {mcause_q[5], 26'b0, mcause_q[4:0]};
      // mhartid: unique hardware thread id
      12'hF14: csr_rdata_int = {21'b0, cluster_id_i[5:0], 1'b0, core_id_i[3:0]};

      // counter / timer
      12'hB00: csr_rdata_int = mcycle[31:0];
      12'hB80: csr_rdata_int = mcycle[63:32];
      12'hb01: csr_rdata_int = mtime[31:0];
      12'hb41: csr_rdata_int = mtime[63:32];
      12'h321: csr_rdata_int = mtimecmp[31:0]; 
      12'h322: csr_rdata_int = mtimecmp[63:32];
      12'h304: csr_rdata_int = mie;

      /* USER CSR */
      // dublicated mhartid: unique hardware thread id (not official)
      12'h014: csr_rdata_int = {21'b0, cluster_id_i[5:0], 1'b0, core_id_i[3:0]};
      // current priv level (not official)
      12'hC10: csr_rdata_int = {30'h0, priv_lvl_q};
      default:
        csr_rdata_int = '0;
    endcase
  end
end //PULP_SECURE

assign mtimer_expired = (mtime >= mtimecmp);

always @(posedge clk, negedge rst_n) begin
  if (rst_n == 1'b0) begin
    mcycle    <= 64'h0;
    mtime     <= 64'h0;
    //mtimecmp  <= 64'h0;
    mie       <= 32'h0;
    mip       <= 32'h0;
  end else begin
    mcycle <= mcycle + 1;
    mtime  <= mtime + 1;
    if (csr_we_int) begin
      case (csr_addr_i)
        12'hb00: mcycle[31:0]    <= csr_wdata_int;
        12'hb80: mcycle[63:32]   <= csr_wdata_int;
        12'hb01: mtime[31:0]     <= csr_wdata_int;
        12'hb41: mtime[63:32]    <= csr_wdata_int;
        12'h321: mtimecmp[31:0]  <= csr_wdata_int;
        12'h322: mtimecmp[63:32] <= csr_wdata_int;
        12'h304: mie             <= csr_wdata_int;
        default : ;
      endcase 
    end 

    if(csr_we_int && (csr_addr_i == 12'h321 || csr_addr_i == 12'h322)) begin
      mip[`MIP_MTIP_BITS] <= 1'b0;
    end else if (mtimer_expired && mstatus_q.mie && mie[`MIE_MTIE_BITS]) begin
      mip[`MIP_MTIP_BITS] <= 1'b1;
    end
  end
end 


if(PULP_SECURE==1) begin
  // write logic
  always_comb
  begin
    mepc_n                   = mepc_q;
    uepc_n                   = uepc_q;
    mstatus_n                = mstatus_q;
    mcause_n                 = mcause_q;
    ucause_n                 = ucause_q;
    exception_pc             = pc_id_i;
    priv_lvl_n               = priv_lvl_q;
    mtvec_n                  = mtvec_q;
    utvec_n                  = utvec_q;
//-----------------MM---------------

    pmp_reg_n.pmpaddr        = pmp_reg_q.pmpaddr;
   // pmp_reg_n.pmpcfg_packed  = pmp_reg_q.pmpcfg_packed;
    pmpaddr_we               = '0;
   // pmpcfg_we                = '0;

//-----------------MM---------------

    

    casex (csr_addr_i)
      // mstatus: IE bit
      12'h300: if (csr_we_int) begin
        mstatus_n = '{
          uie:  csr_wdata_int[`MSTATUS_UIE_BITS],
          mie:  csr_wdata_int[`MSTATUS_MIE_BITS],
          upie: csr_wdata_int[`MSTATUS_UPIE_BITS],
          mpie: csr_wdata_int[`MSTATUS_MPIE_BITS],
          mpp:  PrivLvl_t'(csr_wdata_int[`MSTATUS_MPP_BITS]),
          mprv: csr_wdata_int[`MSTATUS_MPRV_BITS]
        };
      end
      // mtvec: machine trap-handler base address
      12'h305: if (csr_we_int) begin
        mtvec_n    = csr_wdata_int[31:8];
      end
      // mepc: exception program counter
      12'h341: if (csr_we_int) begin
        mepc_n       = csr_wdata_int;
      end
      // mcause
      12'h342: if (csr_we_int) mcause_n = {csr_wdata_int[31], csr_wdata_int[4:0]};
//-----------------MM---------------

// PMP config registers
/*
      12'h3A0: if (csr_we_int) begin pmp_reg_n.pmpcfg_packed[0] = csr_wdata_int; pmpcfg_we[3:0]   = 4'b1111; end
      12'h3A1: if (csr_we_int) begin pmp_reg_n.pmpcfg_packed[1] = csr_wdata_int; pmpcfg_we[7:4]   = 4'b1111; end
      12'h3A2: if (csr_we_int) begin pmp_reg_n.pmpcfg_packed[2] = csr_wdata_int; pmpcfg_we[11:8]  = 4'b1111; end
      12'h3A3: if (csr_we_int) begin pmp_reg_n.pmpcfg_packed[3] = csr_wdata_int; pmpcfg_we[15:12] = 4'b1111; end

      12'h3BX: if (csr_we_int) begin pmp_reg_n.pmpaddr[csr_addr_i[3:0]]   = csr_wdata_int; pmpaddr_we[csr_addr_i[3:0]] = 1'b1;  end
*/
	  `CSR_PMPADDR0: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_CODE_START] = csr_wdata_int; pmpaddr_we[`PMP_CODE_START] = 1'b1; end //3B0				
	  `CSR_PMPADDR1: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_CODE_END] = csr_wdata_int; pmpaddr_we[`PMP_CODE_END] = 1'b1; end //3B1		
	  `CSR_PMPADDR2: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_DATA_START] = csr_wdata_int; pmpaddr_we[`PMP_DATA_START] = 1'b1; end  //3B2		
	  `CSR_PMPADDR3: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_DATA_END] = csr_wdata_int; pmpaddr_we[`PMP_DATA_END] = 1'b1; end //3B3
	  `CSR_PMPADDR4: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_LIB_START] = csr_wdata_int; pmpaddr_we[`PMP_LIB_START] = 1'b1; end //3B4		
	  `CSR_PMPADDR5: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_LIB_END] = csr_wdata_int;  pmpaddr_we[`PMP_LIB_END] = 1'b1; end //3B5
	  
	  `CSR_PMPADDR6: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_PERIPH] = csr_wdata_int;  pmpaddr_we[`PMP_PERIPH] = 1'b1; end //3B6
	  
	  `CSR_PMPADDR7: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_CODE_RELOC] = csr_wdata_int;  pmpaddr_we[`PMP_CODE_RELOC] = 1'b1; end //3B7
	  
	  `CSR_PMPADDR8: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_DATA_RELOC] = csr_wdata_int;  pmpaddr_we[`PMP_DATA_RELOC] = 1'b1; end //3B8
	  
	  
	  
	  //Drivers
	  `CSR_DRV0: if (csr_we_int) begin drvaddr[0] = csr_wdata_int; end  //pmpaddr_we[`PMP_DATA_RELOC] = 1'b1; end //3B8
	  `CSR_DRV1: if (csr_we_int) begin drvaddr[1] = csr_wdata_int; end 
	  `CSR_DRV2: if (csr_we_int) begin drvaddr[2] = csr_wdata_int; end 
	  `CSR_DRV3: if (csr_we_int) begin drvaddr[3] = csr_wdata_int; end 
	  `CSR_DRV4: if (csr_we_int) begin drvaddr[4] = csr_wdata_int; end 
	  `CSR_DRV5: if (csr_we_int) begin drvaddr[5] = csr_wdata_int; end 
	  `CSR_DRV6: if (csr_we_int) begin drvaddr[6] = csr_wdata_int; end 
	  `CSR_DRV7: if (csr_we_int) begin drvaddr[7] = csr_wdata_int; end 
	  `CSR_DRV8: if (csr_we_int) begin drvaddr[8] = csr_wdata_int; end 
	  `CSR_DRV9: if (csr_we_int) begin drvaddr[9] = csr_wdata_int; end 
	  `CSR_DRV10: if (csr_we_int) begin drvaddr[10] = csr_wdata_int; end 
	  `CSR_DRV11: if (csr_we_int) begin drvaddr[11] = csr_wdata_int; end 
	  `CSR_DRV12: if (csr_we_int) begin drvaddr[12] = csr_wdata_int; end 
	  `CSR_DRV13: if (csr_we_int) begin drvaddr[13] = csr_wdata_int; end 
	  `CSR_DRV14: if (csr_we_int) begin drvaddr[14] = csr_wdata_int; end 
	  `CSR_DRV15: if (csr_we_int) begin drvaddr[15] = csr_wdata_int; end 
	  
	  
//-----------------MM---------------


      /* USER CSR */
      // ucause: exception cause
      12'h000: if (csr_we_int) begin
        mstatus_n = '{
          uie:  csr_wdata_int[`MSTATUS_UIE_BITS],
          mie:  mstatus_q.mie,
          upie: csr_wdata_int[`MSTATUS_UPIE_BITS],
          mpie: mstatus_q.mpie,
          mpp:  mstatus_q.mpp,
          mprv: mstatus_q.mprv
        };
      end
      // utvec: user trap-handler base address
      12'h005: if (csr_we_int) begin
        utvec_n    = csr_wdata_int[31:8];
      end
      // uepc: exception program counter
      12'h041: if (csr_we_int) begin
        uepc_n     = csr_wdata_int;
      end
      // ucause: exception cause
      12'h042: if (csr_we_int) ucause_n = {csr_wdata_int[31], csr_wdata_int[4:0]};
    endcase

    // exception controller gets priority over other writes
    unique case (1'b1)

      csr_save_cause_i: begin

        unique case (1'b1)
          csr_save_if_i:
            exception_pc = pc_if_i;
          csr_save_id_i:
            exception_pc = pc_id_i;
          csr_save_ex_i:
            exception_pc = pc_ex_i;
          default:;
        endcase

        unique case (priv_lvl_q)

          PRIV_LVL_U: begin
            if(~is_irq) begin
              //Exceptions, Ecall U --> M
              priv_lvl_n     = PRIV_LVL_M;
              mstatus_n.mpie = mstatus_q.uie;
              mstatus_n.mie  = 1'b0;
              mstatus_n.mpp  = PRIV_LVL_U;
              mepc_n         = exception_pc;
              mcause_n       = csr_cause_i;
            end
            else begin
              if(~csr_irq_sec_i) begin
              //U --> U
                priv_lvl_n     = PRIV_LVL_U;
                mstatus_n.upie = mstatus_q.uie;
                mstatus_n.uie  = 1'b0;
                uepc_n         = exception_pc;
                ucause_n       = csr_cause_i;
              end else begin
              //U --> M
                priv_lvl_n     = PRIV_LVL_M;
                mstatus_n.mpie = mstatus_q.uie;
                mstatus_n.mie  = 1'b0;
                mstatus_n.mpp  = PRIV_LVL_U;
                mepc_n         = exception_pc;
                mcause_n       = csr_cause_i;
              end
            end
          end //PRIV_LVL_U

          PRIV_LVL_M: begin
            //Exceptions or Interrupts from PRIV_LVL_M always do M --> M
            priv_lvl_n     = PRIV_LVL_M;
            mstatus_n.mpie = mstatus_q.mie;
            mstatus_n.mie  = 1'b0;
            mstatus_n.mpp  = PRIV_LVL_M;
            mepc_n         = exception_pc;
            mcause_n       = csr_cause_i;
          end //PRIV_LVL_M
          default:;

        endcase

      end //csr_save_cause_i

      csr_restore_uret_i: begin //URET
        //mstatus_q.upp is implicitly 0, i.e PRIV_LVL_U
        mstatus_n.uie  = mstatus_q.upie;
        priv_lvl_n     = PRIV_LVL_U;
        mstatus_n.upie = 1'b1;
      end //csr_restore_uret_i

      csr_restore_mret_i: begin //MRET
        unique case (mstatus_q.mpp)
          PRIV_LVL_U: begin
            mstatus_n.uie  = mstatus_q.mpie;
            priv_lvl_n     = PRIV_LVL_U;
            mstatus_n.mpie = 1'b1;
            mstatus_n.mpp  = PRIV_LVL_U;
          end
          PRIV_LVL_M: begin
            mstatus_n.mie  = mstatus_q.mpie;
            priv_lvl_n     = PRIV_LVL_M;
            mstatus_n.mpie = 1'b1;
            mstatus_n.mpp  = PRIV_LVL_U;
          end
          default:;
        endcase
      end //csr_restore_mret_i
      default:;
    endcase
  end
end else begin //PULP_SECURE == 0
  // write logic
  always_comb
  begin
    fflags_n                 = fflags_q;
    frm_n                    = frm_q;
    fprec_n                  = fprec_q;
    mepc_n                   = mepc_q;
    mstatus_n                = mstatus_q;
    mcause_n                 = mcause_q;
    exception_pc             = pc_id_i;
    priv_lvl_n               = priv_lvl_q;
    mtvec_n                  = mtvec_q;
//-----------------MM---------------

    pmp_reg_n.pmpaddr        = pmp_reg_q.pmpaddr;
   // pmp_reg_n.pmpcfg_packed  = pmp_reg_q.pmpcfg_packed;
    pmpaddr_we               = '0;
   // pmpcfg_we                = '0;



   //-----------------MM---------------
      // mstatus: IE bit
	 case (csr_addr_i)
      12'h300: if (csr_we_int) begin
        mstatus_n = '{
          uie:  csr_wdata_int[`MSTATUS_UIE_BITS],
          mie:  csr_wdata_int[`MSTATUS_MIE_BITS],
          upie: csr_wdata_int[`MSTATUS_UPIE_BITS],
          mpie: csr_wdata_int[`MSTATUS_MPIE_BITS],
          mpp:  PrivLvl_t'(csr_wdata_int[`MSTATUS_MPP_BITS]),
          mprv: csr_wdata_int[`MSTATUS_MPRV_BITS]
        };
      end
      // mepc: exception program counter
      12'h341: if (csr_we_int) begin
        mepc_n       = csr_wdata_int;
      end
      // mcause
      12'h342: if (csr_we_int) mcause_n = {csr_wdata_int[31], csr_wdata_int[4:0]};

    endcase

    // exception controller gets priority over other writes
    unique case (1'b1)

      csr_save_cause_i: begin

        unique case (1'b1)
          csr_save_if_i:
            exception_pc = pc_if_i;
          csr_save_id_i:
            exception_pc = pc_id_i;
          default:;
        endcase

        priv_lvl_n     = PRIV_LVL_M;
        mstatus_n.mpie = mstatus_q.mie;
        mstatus_n.mie  = 1'b0;
        mstatus_n.mpp  = PRIV_LVL_M;
        mepc_n         = exception_pc;
        mcause_n       = csr_cause_i;
      end //csr_save_cause_i

      csr_restore_mret_i: begin //MRET
        mstatus_n.mie  = mstatus_q.mpie;
        priv_lvl_n     = PRIV_LVL_M;
        mstatus_n.mpie = 1'b1;
        mstatus_n.mpp  = PRIV_LVL_M;
      end //csr_restore_mret_i
      default:;
    endcase
  end
end //PULP_SECURE

  // CSR operation logic
  always_comb
  begin
    csr_wdata_int = csr_wdata_i;
    csr_we_int    = 1'b1;

    unique case (csr_op_i)
      CSR_OP_WRITE: csr_wdata_int = csr_wdata_i;
      CSR_OP_SET:   csr_wdata_int = csr_wdata_i | csr_rdata_o;
      CSR_OP_CLEAR: csr_wdata_int = (~csr_wdata_i) & csr_rdata_o;

      CSR_OP_NONE: begin
        csr_wdata_int = csr_wdata_i;
        csr_we_int    = 1'b0;
      end

      default:;
    endcase
  end


  // output mux
  always_comb
  begin
    csr_rdata_o = csr_rdata_int;

    // performance counters
    if (is_pccr || is_pcer || is_pcmr)
      csr_rdata_o = perf_rdata;
  end


  // directly output some registers
  assign m_irq_enable_o  = mstatus_q.mie & priv_lvl_q == PRIV_LVL_M;
  assign u_irq_enable_o  = mstatus_q.uie & priv_lvl_q == PRIV_LVL_U;
  assign priv_lvl_o      = priv_lvl_q;
  assign sec_lvl_o       = priv_lvl_q[0];

  assign mtvec_o         = mtvec_q;
  assign utvec_o         = utvec_q;

  assign mepc_o          = mepc_q;
  assign uepc_o          = uepc_q;

  assign mip_o           = mip;
//-----------------MM---------------

  assign pmp_addr_o     = pmp_reg_q.pmpaddr;
  assign drv_addr_o     = drvaddr;
 // assign pmp_cfg_o      = pmp_reg_q.pmpcfg;
//-----------------MM---------------


  generate
  if (PULP_SECURE == 1)
  begin
/*
    for(j=0;j<N_PMP_ENTRIES;j++)
    begin : CS_PMP_CFG
      assign pmp_reg_n.pmpcfg[j]                                 = pmp_reg_n.pmpcfg_packed[j/4][8*((j%4)+1)-1:8*(j%4)];
      assign pmp_reg_q.pmpcfg_packed[j/4][8*((j%4)+1)-1:8*(j%4)] = pmp_reg_q.pmpcfg[j];
    end
*/
    for(j=0;j<N_PMP_ENTRIES;j++)
    begin : CS_PMP_REGS_FF
      always_ff @(posedge clk, negedge rst_n)
      begin
          if (rst_n == 1'b0)
          begin
            //pmp_reg_q.pmpcfg[j]   <= '0;
            pmp_reg_q.pmpaddr[j]  <= '0;
          end
          else
          begin
           // if(pmpcfg_we[j])
            //  pmp_reg_q.pmpcfg[j]    <= pmp_reg_n.pmpcfg[j];
            if(pmpaddr_we[j])
              pmp_reg_q.pmpaddr[j]  <=  pmp_reg_n.pmpaddr[j];
          end
        end
      end //CS_PMP_REGS_FF


      always_ff @(posedge clk, negedge rst_n)
      begin
          if (rst_n == 1'b0)
          begin
            uepc_q         <= '0;
            ucause_q       <= '0;
            mtvec_q        <= '0;
            utvec_q        <= '0;
            priv_lvl_q     <= PRIV_LVL_M;

          end
          else
          begin
            uepc_q         <= uepc_n;
            ucause_q       <= ucause_n;
            mtvec_q        <= mtvec_n;
            utvec_q        <= utvec_n;
            priv_lvl_q     <= priv_lvl_n;
          end
        end

  end
  else begin

        assign uepc_q       = '0;
        assign ucause_q     = '0;
        assign mtvec_q      = {boot_addr_i, 1'b0};
        assign utvec_q      = '0;
        assign priv_lvl_q   = PRIV_LVL_M;

  end
  endgenerate


  // actual registers
  always_ff @(posedge clk, negedge rst_n)
  begin
    if (rst_n == 1'b0)
    begin
      mstatus_q  <= '{
              uie:  1'b0,
              mie:  1'b0,
              upie: 1'b0,
              mpie: 1'b0,
              mpp:  PRIV_LVL_M,
              mprv: 1'b0
            };
      mepc_q      <= '0;
      mcause_q    <= '0;
    end
    else
    begin
      if (PULP_SECURE == 1) begin
        mstatus_q      <= mstatus_n ;
      end else begin
        mstatus_q  <= '{
                uie:  1'b0,
                mie:  mstatus_n.mie,
                upie: 1'b0,
                mpie: mstatus_n.mpie,
                mpp:  PRIV_LVL_M,
                mprv: 1'b0
              };
      end
      mepc_q     <= mepc_n    ;
      mcause_q   <= mcause_n  ;
    end
  end

  /////////////////////////////////////////////////////////////////
  //   ____            __     ____                  _            //
  // |  _ \ ___ _ __ / _|   / ___|___  _   _ _ __ | |_ ___ _ __  //
  // | |_) / _ \ '__| |_   | |   / _ \| | | | '_ \| __/ _ \ '__| //
  // |  __/  __/ |  |  _|  | |__| (_) | |_| | | | | ||  __/ |    //
  // |_|   \___|_|  |_|(_)  \____\___/ \__,_|_| |_|\__\___|_|    //
  //                                                             //
  /////////////////////////////////////////////////////////////////

  assign PCCR_in[0]  = 1'b1;                                          // cycle counter
  assign PCCR_in[1]  = id_valid_i & is_decoding_i;                    // instruction counter
  assign PCCR_in[2]  = ld_stall_i & id_valid_q;                       // nr of load use hazards
  assign PCCR_in[3]  = jr_stall_i & id_valid_q;                       // nr of jump register hazards
  assign PCCR_in[4]  = imiss_i & (~pc_set_i);                         // cycles waiting for instruction fetches, excluding jumps and branches
  assign PCCR_in[5]  = mem_load_i;                                    // nr of loads
  assign PCCR_in[6]  = mem_store_i;                                   // nr of stores
  assign PCCR_in[7]  = jump_i                     & id_valid_q;       // nr of jumps (unconditional)
  assign PCCR_in[8]  = branch_i                   & id_valid_q;       // nr of branches (conditional)
  assign PCCR_in[9]  = branch_i & branch_taken_i  & id_valid_q;       // nr of taken branches (conditional)
  assign PCCR_in[10] = id_valid_i & is_decoding_i & is_compressed_i;  // compressed instruction counter
  assign PCCR_in[11] = pipeline_stall_i;                              //extra cycles from elw

  // assign external performance counters
  generate
    genvar i;
    for(i = 0; i < N_EXT_CNT; i++)
    begin
      assign PCCR_in[PERF_EXT_ID + i] = ext_counters_i[i];
    end
  endgenerate

  // address decoder for performance counter registers
  always_comb
  begin
    is_pccr      = 1'b0;
    is_pcmr      = 1'b0;
    is_pcer      = 1'b0;
    pccr_all_sel = 1'b0;
    pccr_index   = '0;
    perf_rdata   = '0;

    // only perform csr access if we actually care about the read data
    if (csr_access_i) begin
      unique case (csr_addr_i)
        12'h7A0: begin
          is_pcer = 1'b1;
          perf_rdata[N_PERF_COUNTERS-1:0] = PCER_q;
        end
        12'h7A1: begin
          is_pcmr = 1'b1;
          perf_rdata[1:0] = PCMR_q;
        end
        12'h79F: begin
          is_pccr = 1'b1;
          pccr_all_sel = 1'b1;
        end
        default:;
      endcase

      // look for 780 to 79F, Performance Counter Counter Registers
      if (csr_addr_i[11:5] == 7'b0111100) begin
        is_pccr     = 1'b1;

        pccr_index = csr_addr_i[4:0];
`ifdef  ASIC_SYNTHESIS
        perf_rdata = PCCR_q[0];
`else
        perf_rdata = csr_addr_i[4:0] < N_PERF_COUNTERS ? PCCR_q[csr_addr_i[4:0]] : '0;
`endif
      end
    end
  end


  // performance counter counter update logic
`ifdef ASIC_SYNTHESIS
  // for synthesis we just have one performance counter register
  assign PCCR_inc[0] = (|(PCCR_in & PCER_q)) & PCMR_q[0];

  always_comb
  begin
    PCCR_n[0]   = PCCR_q[0];

    if ((PCCR_inc_q[0] == 1'b1) && ((PCCR_q[0] != 32'hFFFFFFFF) || (PCMR_q[1] == 1'b0)))
      PCCR_n[0] = PCCR_q[0] + 1;

    if (is_pccr == 1'b1) begin
      unique case (csr_op_i)
        CSR_OP_NONE:   ;
        CSR_OP_WRITE:  PCCR_n[0] = csr_wdata_i;
        CSR_OP_SET:    PCCR_n[0] = csr_wdata_i | PCCR_q[0];
        CSR_OP_CLEAR:  PCCR_n[0] = csr_wdata_i & ~(PCCR_q[0]);
      endcase
    end
  end
`else
  always_comb
  begin
    for(int i = 0; i < N_PERF_COUNTERS; i++)
    begin : PERF_CNT_INC
      PCCR_inc[i] = PCCR_in[i] & PCER_q[i] & PCMR_q[0];

      PCCR_n[i]   = PCCR_q[i];

      if ((PCCR_inc_q[i] == 1'b1) && ((PCCR_q[i] != 32'hFFFFFFFF) || (PCMR_q[1] == 1'b0)))
        PCCR_n[i] = PCCR_q[i] + 1;

      if (is_pccr == 1'b1 && (pccr_all_sel == 1'b1 || pccr_index == i)) begin
        unique case (csr_op_i)
          CSR_OP_NONE:   ;
          CSR_OP_WRITE:  PCCR_n[i] = csr_wdata_i;
          CSR_OP_SET:    PCCR_n[i] = csr_wdata_i | PCCR_q[i];
          CSR_OP_CLEAR:  PCCR_n[i] = csr_wdata_i & ~(PCCR_q[i]);
        endcase
      end
    end
  end
`endif

  // update PCMR and PCER
  always_comb
  begin
    PCMR_n = PCMR_q;
    PCER_n = PCER_q;

    if (is_pcmr) begin
      unique case (csr_op_i)
        CSR_OP_NONE:   ;
        CSR_OP_WRITE:  PCMR_n = csr_wdata_i[1:0];
        CSR_OP_SET:    PCMR_n = csr_wdata_i[1:0] | PCMR_q;
        CSR_OP_CLEAR:  PCMR_n = csr_wdata_i[1:0] & ~(PCMR_q);
      endcase
    end

    if (is_pcer) begin
      unique case (csr_op_i)
        CSR_OP_NONE:   ;
        CSR_OP_WRITE:  PCER_n = csr_wdata_i[N_PERF_COUNTERS-1:0];
        CSR_OP_SET:    PCER_n = csr_wdata_i[N_PERF_COUNTERS-1:0] | PCER_q;
        CSR_OP_CLEAR:  PCER_n = csr_wdata_i[N_PERF_COUNTERS-1:0] & ~(PCER_q);
      endcase
    end
  end

  // Performance Counter Registers
  always_ff @(posedge clk, negedge rst_n)
  begin
    if (rst_n == 1'b0)
    begin
      id_valid_q <= 1'b0;

      PCER_q <= '0;
      PCMR_q <= 2'h3;

      for(int i = 0; i < N_PERF_REGS; i++)
      begin
        PCCR_q[i]     <= '0;
        PCCR_inc_q[i] <= '0;
      end
    end
    else
    begin
      id_valid_q <= id_valid_i;

      PCER_q <= PCER_n;
      PCMR_q <= PCMR_n;

      for(int i = 0; i < N_PERF_REGS; i++)
      begin
        PCCR_q[i]     <= PCCR_n[i];
        PCCR_inc_q[i] <= PCCR_inc[i];
      end

    end
  end

endmodule
