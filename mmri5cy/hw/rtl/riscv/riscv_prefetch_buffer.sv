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
// Engineer:       Andreas Traber - atraber@iis.ee.ethz.ch                    //
//                                                                            //
// Design Name:    Prefetcher Buffer for 32 bit memory interface              //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Prefetch Buffer that caches instructions. This cuts overly //
//                 long critical paths to the instruction cache               //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

// input port: send address one cycle before the data
// clear_i clears the FIFO for the following cycle. in_addr_i can be sent in
// this cycle already

module riscv_prefetch_buffer
(
  input  logic        clk,
  input  logic        rst_n,

  input  logic        req_i,

  input  logic        branch_i,
  input  logic [31:0] addr_i,

 

  input  logic        ready_i,
  output logic        valid_o,
  output logic [31:0] rdata_o,
  output logic [31:0] addr_o,
  

  // goes to instruction memory / instruction cache
  output logic        instr_req_o,
  input  logic        instr_gnt_i,
  output logic [31:0] instr_addr_o,
  input  logic [31:0] instr_rdata_i,
  input  logic        instr_rvalid_i,
  
  //-----------------MM---------------
  input  logic        instr_err_pmp_i,
  //-----------------MM---------------
  output logic        fetch_failed_o,

  // Prefetch Buffer Status
  output logic        busy_o
);

  enum logic [2:0] {IDLE, WAIT_GNT, WAIT_RVALID, WAIT_ABORTED, WAIT_JUMP } CS, NS;

  logic [31:0] instr_addr_q, fetch_addr;

  logic        addr_valid;

  logic        fifo_valid;
  logic        fifo_ready;
  logic        fifo_clear;


  logic        valid_stored;
  
  logic        unaligned_is_compressed;


  //////////////////////////////////////////////////////////////////////////////
  // prefetch buffer status
  //////////////////////////////////////////////////////////////////////////////

  assign busy_o = (CS != IDLE) || instr_req_o;

  //////////////////////////////////////////////////////////////////////////////
  // fetch fifo
  // consumes addresses and rdata
  //////////////////////////////////////////////////////////////////////////////

  riscv_fetch_fifo fifo_i
  (
    .clk                   ( clk               ),
    .rst_n                 ( rst_n             ),
    .clear_i               ( fifo_clear        ),

    .in_addr_i             ( instr_addr_q      ),
    .in_rdata_i            ( instr_rdata_i     ),
    .in_valid_i            ( fifo_valid        ),
    .in_ready_o            ( fifo_ready        ),

    .out_valid_o           ( valid_o           ),
    .out_ready_i           ( ready_i           ),
    .out_rdata_o           ( rdata_o           ),
    .out_addr_o            ( addr_o            ),
    .unaligned_is_compressed_o ( unaligned_is_compressed ),
    .out_valid_stored_o    ( valid_stored      )
  );


  //////////////////////////////////////////////////////////////////////////////
  // fetch addr
  //////////////////////////////////////////////////////////////////////////////

  assign fetch_addr    = {instr_addr_q[31:2], 2'b00} + 32'd4;



  always_comb
  begin    
    fifo_clear         = 1'b0;   
    if (branch_i) begin     
      fifo_clear = 1'b1;
    end
  end

  //////////////////////////////////////////////////////////////////////////////
  // instruction fetch FSM
  // deals with instruction memory / instruction cache
  //////////////////////////////////////////////////////////////////////////////

  always_comb
  begin
    instr_req_o   = 1'b0;
    instr_addr_o  = fetch_addr;
    fifo_valid    = 1'b0;
    addr_valid    = 1'b0;    
    fetch_failed_o = 1'b0;
    NS            = CS;

    unique case(CS)
      // default state, not waiting for requested data
      IDLE:
      begin
        instr_addr_o = fetch_addr;
        instr_req_o  = 1'b0;

        if (branch_i )
          instr_addr_o = branch_i ? addr_i : instr_addr_q;
       

        if (req_i & (fifo_ready | branch_i)) begin
          instr_req_o = 1'b1;
          addr_valid  = 1'b1;        

          if(instr_gnt_i) //~>  granted request
            NS = WAIT_RVALID;
          else begin //~> got a request but no grant
            NS = WAIT_GNT;
          end
//-----------------MM---------------		
          if(instr_err_pmp_i)
            NS = WAIT_JUMP;		
//-----------------MM---------------
        end
      end // case: IDLE


      WAIT_JUMP:
      begin

        instr_req_o  = 1'b0;
        fetch_failed_o = valid_o == 1'b0; //MM!!!!!

        if (branch_i) begin
          instr_addr_o = addr_i;
          addr_valid   = 1'b1;
          instr_req_o  = 1'b1;
          fetch_failed_o = 1'b0;

          if(instr_gnt_i)
            NS = WAIT_RVALID;
          else
            NS = WAIT_GNT;
        end
      end


      // we sent a request but did not yet get a grant
      WAIT_GNT:
      begin
        instr_addr_o = instr_addr_q;
        instr_req_o  = 1'b1;

        if (branch_i) begin
          instr_addr_o = branch_i ? addr_i : instr_addr_q;
          addr_valid   = 1'b1;
        end
        if(instr_gnt_i)
          NS = WAIT_RVALID;
        else
          NS = WAIT_GNT;

        //-----------------MM---------------		
        if(instr_err_pmp_i)
          NS = WAIT_JUMP;		
		//-----------------MM---------------
      end // case: WAIT_GNT

      // we wait for rvalid, after that we are ready to serve a new request
      WAIT_RVALID: begin
        instr_addr_o = fetch_addr;

        if (branch_i)
          instr_addr_o = branch_i ? addr_i : instr_addr_q;
       

        if (req_i & (fifo_ready | branch_i )) begin
          // prepare for next request

          if (instr_rvalid_i) begin
            instr_req_o = 1'b1;
            fifo_valid  = 1'b1;
            addr_valid  = 1'b1;
           
            if (instr_gnt_i) begin
              NS = WAIT_RVALID;
            end else begin
              NS = WAIT_GNT;
            end
           //-----------------MM---------------
			
			if(instr_err_pmp_i)
              NS = WAIT_JUMP;
			
			//-----------------MM---------------

          end else begin
            // we are requested to abort our current request
            // we didn't get an rvalid yet, so wait for it
            if (branch_i ) begin
              addr_valid = 1'b1;
              NS  = WAIT_ABORTED;
            
            end
          end
        end else begin
          // just wait for rvalid and go back to IDLE, no new request

          if (instr_rvalid_i) begin
            fifo_valid = 1'b1;
            NS         = IDLE;
          end
        end
      end // case: WAIT_RVALID

      // our last request was aborted, but we didn't yet get a rvalid and
      // there was no new request sent yet
      // we assume that req_i is set to high
      WAIT_ABORTED: begin
        instr_addr_o = instr_addr_q;

        if (branch_i ) begin
          instr_addr_o = branch_i ? addr_i : instr_addr_q;
          addr_valid   = 1'b1;
        end

        if (instr_rvalid_i) begin
          instr_req_o  = 1'b1;
          // no need to send address, already done in WAIT_RVALID

          if (instr_gnt_i) begin
            NS = WAIT_RVALID;
          end else begin
            NS = WAIT_GNT;
          end
          //-----------------MM---------------
			
          if(instr_err_pmp_i)
            NS = WAIT_JUMP;
			
		//-----------------MM---------------
        end
      end

      default:
      begin
        NS          = IDLE;
        instr_req_o = 1'b0;
      end
    endcase
  end

  //////////////////////////////////////////////////////////////////////////////
  // registers
  //////////////////////////////////////////////////////////////////////////////

  always_ff @(posedge clk, negedge rst_n)
  begin
    if(rst_n == 1'b0)
    begin
      CS              <= IDLE;     
      instr_addr_q    <= '0;
    end
    else
    begin
      CS              <= NS;
      if (addr_valid) begin
        instr_addr_q    <=  instr_addr_o;
      end
    end
  end

endmodule
