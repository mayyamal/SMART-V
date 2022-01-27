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
// Modified by:    Maja Malenko - malenko.maya@gmail.com                      //
//                 exception control modifications                            //
//				   pmp modifications                                          //  	
//                                                                            // 
// Design Name:    Control and Status Registers                               //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Control and Status Registers (CSRs) loosely following the  //
//                 RiscV draft priviledged instruction set spec (v1.9)        //
//                 PULP_SECURE = 1                                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

import riscv_defines::*;

module riscv_cs_registers
#(  
  //-----------------MM---------------  
  parameter N_PMP_ENTRIES = `SMARTV_N_PMP_ENTRIES // code, data, lib
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
  //input  logic            csr_restore_uret_i,
  //coming from controller
  input  logic [5:0]      csr_cause_i,
  //coming from controller
  input  logic            csr_save_cause_i  
);

  
  localparam MTVEC_MODE      = 2'b01;

 //-----------------MM---------------
 // localparam MAX_N_PMP_ENTRIES = 16;
 // localparam MAX_N_PMP_CFG     =  4;
 // localparam N_PMP_CFG         = N_PMP_ENTRIES % 4 == 0 ? N_PMP_ENTRIES/4 : N_PMP_ENTRIES/4 + 1;
 //-----------------MM---------------

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
  
  
	  /***********************************************************
	  ********************** PRATER-V BASIC ***********************
	  ***********************************************************/
	`ifdef  PRATERV_EN 
		typedef struct packed {
			logic  [N_PMP_ENTRIES-1:0] [31:0] pmpaddr; 
		} Pmp_t;
		//logic  [15:0] [31:0] drvaddr;
		
		Pmp_t pmp_reg_q, pmp_reg_n;
		// clock gating for pmp regs
		logic [N_PMP_ENTRIES-1:0] pmpaddr_we;
		
	`endif 
	
  
	  /***********************************************************
	  ********************** SMART-V  ****************************
	  ***********************************************************/
	  
	`ifdef  SMARTV_EN 
		typedef struct packed {
			logic  [N_PMP_ENTRIES-1:0] [31:0] pmpaddr; 
		} Pmp_t;
		//logic  [15:0] [31:0] drvaddr;
		
		Pmp_t pmp_reg_q, pmp_reg_n;
		// clock gating for pmp regs
		logic [N_PMP_ENTRIES-1:0] pmpaddr_we;
		
	`endif 
	  
	  

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
  
  
  
/////////////////////////////////////////  
//  ___ ___   _   ___     ___ ___ ___  // 
// | _ \ __| /_\ |   \   / __/ __| _ \ // 
// |   / _| / _ \| |) | | (__\__ \   / //
// |_|_\___/_/ \_\___/   \___|___/_|_\ //
//                                     //
/////////////////////////////////////////


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
	  
	  
	  
	  /***********************************************************
	  *********************** PRATER-V ***************************
	  ***********************************************************/
	  	  
	`ifdef  PRATERV_EN 
		`CSR_PMPADDR0: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_CODE_START];//3B0				
		`CSR_PMPADDR1: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_CODE_END]; //3B1		
		`CSR_PMPADDR2: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_DATA_START]; //3B2		
		`CSR_PMPADDR3: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_DATA_END]; //3B3
		`CSR_PMPADDR4: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_LIB_START]; //3B4		
		`CSR_PMPADDR5: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_LIB_END]; //3B5	
		`CSR_PMPADDR6: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_PERIPH]; //3B6
		
		`CSR_PMPADDR7: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_CODE_RELOC]; //3B7
		`CSR_PMPADDR8: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_DATA_RELOC]; //3B8
		
		`CSR_PMPADDR9: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_DRV_CODE_START]; //3B9
		`CSR_PMPADDR10: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_DRV_CODE_END]; //3BA
		`CSR_PMPADDR11: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_DRV_CODE_RELOC]; //3BB
		`CSR_PMPADDR12: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_DRV_ID]; //3BC
		`CSR_PMPADDR13: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_DRV_DATA_RELOC]; //3BD
		
		// User registers, workaround
		// [MM] rewrite internal CSR/PRATER-V registers!
		`CSR_USER_CODE_RELOC: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_CODE_RELOC];
		`CSR_USER_DATA_RELOC: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_DATA_RELOC];
		`CSR_USER_DRV_RELOC:  csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_DRV_CODE_RELOC]; 
		`CSR_USER_END_DATA:   csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_DATA_END];
	`endif 
	 
	  /*	  
	  `CSR_DRV0: csr_rdata_int = drvaddr[0];  //pmpaddr_we[`PMP_DATA_RELOC] = 1'b1; end //3B8
	  `CSR_DRV1: csr_rdata_int = drvaddr[1];
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
	  */
	  
	  
	  /***********************************************************
	  ********************** SMART-V  ****************************
	  ***********************************************************/
	  
	`ifdef  SMARTV_EN 
		`CSR_PMPADDR0: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_CODE_START];//3B0				
		`CSR_PMPADDR1: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_CODE_END]; //3B1		
		`CSR_PMPADDR2: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_DATA_START]; //3B2		
		`CSR_PMPADDR3: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_DATA_END]; //3B3
		`CSR_PMPADDR4: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_LIB_START]; //3B4		
		`CSR_PMPADDR5: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_LIB_END]; //3B5	
		
		`ifdef  SMARTV_PERIPHERAL_EN 
		`CSR_PMPADDR6: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_PERIPH]; //3B6	
		`endif 
		
		
		`ifdef  SMARTV_BITBANDING_EN 
		`CSR_PMPADDR7: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_PERIPH_BB]; //3B7
		`CSR_PMPADDR8: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_PERIPH_BB_0]; //3B8
		`CSR_PMPADDR9: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_PERIPH_BB_1]; //3B9
		`endif 
		
		/*
		`CSR_PMPADDR6: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_RO_START]; //3B6		
		`CSR_PMPADDR7: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_RO_END]; //3B7
		
		`CSR_PMPADDR8: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_STC_START]; //3B8		
		`CSR_PMPADDR9: csr_rdata_int = pmp_reg_q.pmpaddr[`PMP_STC_END]; //3B9		
		
		*/
	`endif   
 
    
	
	  
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



  
//////////////////////////////////////////////////  
// __      _____ ___ _____ ___    ___ ___ ___   // 
// \ \    / / _ \_ _|_   _| __|  / __/ __| _ \  //
//  \ \/\/ /|   /| |  | | | _|  | (__\__ \   /  //
//   \_/\_/ |_|_\___| |_| |___|  \___|___/_|_\  //
//                                              //
//////////////////////////////////////////////////



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
    //pmp_reg_n.pmpaddr        = pmp_reg_q.pmpaddr;
   // pmp_reg_n.pmpcfg_packed  = pmp_reg_q.pmpcfg_packed;
   // pmpaddr_we               = '0;
   // pmpcfg_we                = '0;
   //-----------------MM---------------


	`ifdef  PRATERV_EN 
		pmp_reg_n.pmpaddr        = pmp_reg_q.pmpaddr;		
		pmpaddr_we               = '0;
		
	`endif 
	
	
	`ifdef  SMARTV_EN 
		pmp_reg_n.pmpaddr        = pmp_reg_q.pmpaddr;		
		pmpaddr_we               = '0;		
	`endif 
    

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
		
		
	  /***********************************************************
	  ********************** PRATER-V ****************************
	  ***********************************************************/	
		
	`ifdef  PRATERV_EN 
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
	`CSR_PMPADDR9: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_DRV_CODE_START]= csr_wdata_int;  pmpaddr_we[`PMP_DRV_CODE_START] = 1'b1; end //3B9		
	`CSR_PMPADDR10: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_DRV_CODE_END]= csr_wdata_int;  pmpaddr_we[`PMP_DRV_CODE_END] = 1'b1; end //3BA	
	`CSR_PMPADDR11: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_DRV_CODE_RELOC]= csr_wdata_int;  pmpaddr_we[`PMP_DRV_CODE_RELOC] = 1'b1; end //3BB		
	`CSR_PMPADDR12: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_DRV_ID]= csr_wdata_int;  pmpaddr_we[`PMP_DRV_ID] = 1'b1; end //3BB	
	// New for benchmark BSS
	`CSR_PMPADDR13: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_DRV_DATA_RELOC]= csr_wdata_int;  pmpaddr_we[`PMP_DRV_DATA_RELOC] = 1'b1; end //3BD
	
	
	// User CSRs workaround	
	`CSR_USER_CODE_RELOC: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_CODE_RELOC]= csr_wdata_int;  pmpaddr_we[`PMP_CODE_RELOC] = 1'b1; end		
	`CSR_USER_DATA_RELOC: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_DATA_RELOC]= csr_wdata_int;  pmpaddr_we[`PMP_DATA_RELOC] = 1'b1; end	
	`CSR_USER_DRV_RELOC: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_DRV_CODE_RELOC]= csr_wdata_int;  pmpaddr_we[`PMP_DRV_CODE_RELOC] = 1'b1; end 	
	`CSR_USER_END_DATA: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_DRV_CODE_END]= csr_wdata_int;  pmpaddr_we[`PMP_DRV_CODE_END] = 1'b1; end
	
	`endif 	

	//Drivers
	 /* `CSR_DRV0: if (csr_we_int) begin drvaddr[0] = csr_wdata_int; end  //pmpaddr_we[`PMP_DATA_RELOC] = 1'b1; end //3B8
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
	  */
		
	  /***********************************************************
	  ********************** SMART-V BASIC ***********************
	  ***********************************************************/
	  
	  
	 `ifdef  SMARTV_EN 
	  `CSR_PMPADDR0: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_CODE_START] = csr_wdata_int; pmpaddr_we[`PMP_CODE_START] = 1'b1; end //3B0				
	  `CSR_PMPADDR1: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_CODE_END] = csr_wdata_int; pmpaddr_we[`PMP_CODE_END] = 1'b1; end //3B1		
	  `CSR_PMPADDR2: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_DATA_START] = csr_wdata_int; pmpaddr_we[`PMP_DATA_START] = 1'b1; end  //3B2		
	  `CSR_PMPADDR3: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_DATA_END] = csr_wdata_int; pmpaddr_we[`PMP_DATA_END] = 1'b1; end //3B3
	  `CSR_PMPADDR4: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_LIB_START] = csr_wdata_int; pmpaddr_we[`PMP_LIB_START] = 1'b1; end //3B4		
	  `CSR_PMPADDR5: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_LIB_END] = csr_wdata_int;  pmpaddr_we[`PMP_LIB_END] = 1'b1; end //3B5
	  
	  `ifdef  SMARTV_PERIPHERAL_EN 
		`CSR_PMPADDR6: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_PERIPH] = csr_wdata_int;  pmpaddr_we[`PMP_PERIPH] = 1'b1; end //3B6	
	  `endif 
	  
	  
	  `ifdef  SMARTV_BITBANDING_EN 
		`CSR_PMPADDR7: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_PERIPH_BB] = csr_wdata_int;  pmpaddr_we[`PMP_PERIPH_BB] = 1'b1; end //3B7		
		`CSR_PMPADDR8: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_PERIPH_BB_0] = csr_wdata_int;  pmpaddr_we[`PMP_PERIPH_BB_0] = 1'b1; end //3B8		
		`CSR_PMPADDR9: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_PERIPH_BB_1] = csr_wdata_int;  pmpaddr_we[`PMP_PERIPH_BB_1] = 1'b1; end //3B9
	  `endif 
	  	  
	`endif 	  
	  /*
	  `CSR_PMPADDR6: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_RO_START] = csr_wdata_int; pmpaddr_we[`PMP_RO_START] = 1'b1; end //3B6		
	  `CSR_PMPADDR7: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_RO_END] = csr_wdata_int;  pmpaddr_we[`PMP_RO_END] = 1'b1; end //3B7
	  
	  `CSR_PMPADDR8: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_STC_START] = csr_wdata_int; pmpaddr_we[`PMP_STC_START] = 1'b1; end //3B8		
	  `CSR_PMPADDR9: if (csr_we_int) begin pmp_reg_n.pmpaddr[`PMP_STC_END] = csr_wdata_int;  pmpaddr_we[`PMP_STC_END] = 1'b1; end //3B9
	  */
		
	 

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
    endcase // casex(csr_addr_i)
	
	
	
//////////////////////////////////////////////////////////////////////  
//  _____  _____ ___ ___ _____ ___ ___  _  _    ___ _____ ___ _     //
// | __\ \/ / __| __| _ \_   _|_ _/ _ \| \| |  / __|_   _| _ \ |    //  
// | _| >  < (__| _||  _/ | |  | | (_) | .` | | (__  | | |   / |__  // 
// |___/_/\_\___|___|_|   |_| |___\___/|_|\_|  \___| |_| |_|_\____| //
//                                                                  //
//////////////////////////////////////////////////////////////////////
	
	// exception controller gets priority over other writes
    unique case (1'b1)
      csr_save_cause_i: begin // MM: From ctrl. It is '1' on exceptions: PMP error, illegal, ecall
        unique case (1'b1)
          csr_save_if_i:
            exception_pc = pc_if_i;
          csr_save_id_i:
            exception_pc = pc_id_i;
          csr_save_ex_i:
            exception_pc = pc_ex_i;
          default:;
        endcase

		 priv_lvl_n     = PRIV_LVL_M;             
		 mstatus_n.mpie = mstatus_q.mie;
         mstatus_n.mie  = 1'b0;			
         mstatus_n.mpp  = PRIV_LVL_U; // MM: think about putting priv_lvl_q!
         mepc_n         = exception_pc;
         mcause_n       = csr_cause_i;            
      
      end //csr_save_cause_i

      csr_restore_mret_i: begin //MRET 
			mstatus_n.mie  = mstatus_q.mpie; // MM: for mret from M-mode, mpp should hold U-mode, mpie should be 1
            priv_lvl_n     = PRIV_LVL_U;
            mstatus_n.mpie = 1'b1; // mpie always holds the interrupt enable bits prior to the trap 
            mstatus_n.mpp  = PRIV_LVL_U;
	   end            
      default:;
    endcase
  end
 

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
  //assign pmp_addr_o     = pmp_reg_q.pmpaddr;
  //assign drv_addr_o     = drvaddr;
 // assign pmp_cfg_o      = pmp_reg_q.pmpcfg;
 //-----------------MM---------------
 
 
 	`ifdef  PRATERV_EN 
		assign pmp_addr_o     = pmp_reg_q.pmpaddr;
		//assign drv_addr_o     = drvaddr;
		//assign pmp_cfg_o      = pmp_reg_q.pmpcfg;
	`endif 
	
	
	`ifdef  SMARTV_EN 
		assign pmp_addr_o     = pmp_reg_q.pmpaddr;		
	`endif 

  genvar j;
  generate  
  
  
  
  	`ifdef  PRATERV_EN 
		for(j=0;j<N_PMP_ENTRIES;j++)
			begin : CS_PMP_REGS_FF
				always_ff @(posedge clk, negedge rst_n)
				begin
					if (rst_n == 1'b0)
						begin           
						pmp_reg_q.pmpaddr[j]  <= '0;
						end
					else
						begin
						if(pmpaddr_we[j])
						pmp_reg_q.pmpaddr[j]  <=  pmp_reg_n.pmpaddr[j];
						end
				end
			end //CS_PMP_REGS_FF
	`endif 
	
	
	
	`ifdef  SMARTV_EN 
		for(j=0;j<N_PMP_ENTRIES;j++)
			begin : CS_PMP_REGS_FF
				always_ff @(posedge clk, negedge rst_n)
				begin
					if (rst_n == 1'b0)
						begin           
						pmp_reg_q.pmpaddr[j]  <= '0;
						end
					else
						begin
						if(pmpaddr_we[j])
						pmp_reg_q.pmpaddr[j]  <=  pmp_reg_n.pmpaddr[j];
						end
				end
			end //CS_PMP_REGS_FF
	`endif 


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
              mpie: 1'b1, 
              mpp:  PRIV_LVL_U,
              mprv: 1'b0
            };
      mepc_q      <= '0;
      mcause_q    <= '0;
    end
    else
    begin      
      mstatus_q  <= mstatus_n ;     
      mepc_q     <= mepc_n    ;
      mcause_q   <= mcause_n  ;
    end
  end



endmodule
