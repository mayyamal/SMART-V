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
// Engineer:       Francesco Conti - f.conti@unibo.it                         //
//                                                                            //
// Additional contributions by:                                               //
//                 Michael Gautschi - gautschi@iis.ee.ethz.ch                 //
//                                                                            //
// Design Name:    RISC-V register file                                       //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Register file with 31x 32 bit wide registers. Register 0   //
//                 is fixed to 0. This register file is based on flip-flops.  //
//                 Also supports the fp-register file now if FPU=1            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module riscv_register_file
#(
    parameter ADDR_WIDTH    = 5,
    parameter DATA_WIDTH    = 32
)
(
    // Clock and Reset
    input  logic         clk,
    input  logic         rst_n,

  

    //Read port R1
    input  logic [ADDR_WIDTH-1:0]  raddr_a_i,
    output logic [DATA_WIDTH-1:0]  rdata_a_o,

    //Read port R2
    input  logic [ADDR_WIDTH-1:0]  raddr_b_i,
    output logic [DATA_WIDTH-1:0]  rdata_b_o,

    //Read port R3
    input  logic [ADDR_WIDTH-1:0]  raddr_c_i,
    output logic [DATA_WIDTH-1:0]  rdata_c_o,

    // Write port W1
    input logic [ADDR_WIDTH-1:0]   waddr_a_i,
    input logic [DATA_WIDTH-1:0]   wdata_a_i,
    input logic                    we_a_i,

    // Write port W2
    input logic [ADDR_WIDTH-1:0]   waddr_b_i,
    input logic [DATA_WIDTH-1:0]   wdata_b_i,
    input logic                    we_b_i
);

  // number of integer registers
  localparam    NUM_WORDS     = 2**(ADDR_WIDTH-1);
  

  // integer register file
  logic [NUM_WORDS-1:0][DATA_WIDTH-1:0]     mem;

 

  //-----------------------------------------------------------------------------
  //-- READ : Read address decoder RAD
  //-----------------------------------------------------------------------------
    assign rdata_a_o = mem[raddr_a_i[ADDR_WIDTH-1:0]];
    assign rdata_b_o = mem[raddr_b_i[ADDR_WIDTH-1:0]];
    assign rdata_c_o = mem[raddr_c_i[ADDR_WIDTH-1:0]];
  

    //-----------------------------------------------------------------------------
    //-- WRITE : Write operation
    //-----------------------------------------------------------------------------
    // R0 is nil
   
    // R0 is nil
    assign mem[0] = {DATA_WIDTH{1'b0}};

    genvar i;
    
    generate

        // loop from 1 to NUM_WORDS-1 as R0 is nil
        for (i = 1; i < NUM_WORDS; i++) 
        begin : rf_gen

            always_ff @(posedge clk, negedge rst_n)
            begin : register_write_behavioral
        
                if (rst_n==1'b0) 
                begin
                    mem[i] <= {DATA_WIDTH{1'b0}};
                end else 
                begin
                    if(waddr_b_i == i && we_b_i == 1'b1)
                        mem[i] <= wdata_b_i;
                    else if(waddr_a_i == i && we_a_i == 1'b1)
                        mem[i] <= wdata_a_i;
                end
            end
        end

    endgenerate

endmodule