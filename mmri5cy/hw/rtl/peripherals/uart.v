//------------------------------------------------------------------------------
// Copyright (c) 2016-2018, Fabian Mauroner
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// - Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// 
// - Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// 
// - Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//------------------------------------------------------------------------------
// FILE_NAME         : uart.v
// AUTHOR            : Fabian Mauroner
// AUTHOR'S EMAIL    : mauroner@tugraz.at
//----------------------------------------------------------------------------
// RELEASE HISTORY
// VERSION  DATE        AUTHOR               DESCRIPTION
// 1.0      2017-05-05  Fabian Mauroner      initial version
//----------------------------------------------------------------------------
// KEYWORDS : uart, peripheral
//----------------------------------------------------------------------------
// PURPOSE  : UART peripheral with interrupt support
//----------------------------------------------------------------------------
// PARAMETERS
//    PARAMETER NAME       RANGE    DEFAULT  UNITS       DESCRITPION
//    N/A
//----------------------------------------------------------------------------
// REUSE ISSUES
//    Reset Strategy    : Synchronous, active high
//    Clock domains     : clk
//    Critical Timing   : N/A
//    Asynchronous I/F  : None
//    Synthesizable     : Y
//    Other             : N/A
//----------------------------------------------------------------------------

`define XPR_LEN 32

`define UART_REG_WIDTH           4
`define UART_REG_CTRL          `UART_REG_WIDTH'b0000
`define UART_REG_STATUS        `UART_REG_WIDTH'b0100
`define UART_REG_DATA          `UART_REG_WIDTH'b1000                                                                                                                           
`define UART_REG_LAST            `UART_REG_DATA

module uart(
		input								clk,
		input								reset,

        input [`UART_REG_WIDTH-1:0]			addr,
        input [`XPR_LEN-1:0]		   		wdata,
        output [`XPR_LEN-1:0]				rdata,
		input								en,
		input								wen,
        output reg                			badmem,
        output                    			bus_wait,

		output								irq_tx,
		output								irq_rx,
		output								irq_err,
		output								txd,
		input								rxd
);


reg [`UART_REG_WIDTH-1:0]		  reg_addr;
reg				               	  write;
reg				              	  en_dly;
wire			                  tx_start;
wire			                  tx_done;
wire			                  tx_bit_nxt;
wire			                  tx_fifo_empty;
wire			                  tx_fifo_full;

reg				                  rx_parity_valid;
wire			                  rx_fifo_empty;
wire			                  rx_fifo_full;
wire			                  rx_fifo_ov;
reg				                  rx_stop_valid;




always @(posedge clk)
	if (reset)		   reg_addr <= `UART_REG_WIDTH'h0;
	else if(en)		   reg_addr <= addr;
	else if(en_dly)	reg_addr <= reg_addr + 'd1;

always @(posedge clk)
	if (reset)		write <= 1'b0;
	else if (en)	write <= wen;

always @(posedge clk)
	if (reset)	en_dly <= 1'b0;
	else		   en_dly <= en;

/*****************************************************************************
 * Register
 ****************************************************************************/

// CTRL Register
//  --------------------------------------------------------------------------------
//  |  -  | bdiv | rxfs | txfs | - | pc | pe | tsb | - | txic | erie | rxie | txie |
//  --------------------------------------------------------------------------------
//	31:30   29:16  15:14  13:12  11  10   9     8   7:4   3      2      1      0
//
reg [31:0]	ctrl;
wire [13:0]	ctrl_bdiv			= ctrl[29:16];		// baud = clk / (div + 1)
wire [1:0]	ctrl_rxfs			= ctrl[15:14];		// transmit fifo size
wire [1:0]	ctrl_txfs			= ctrl[13:12];		// transmit fifo size
wire		   ctrl_pc				= ctrl[10];			// parity control
wire		   ctrl_pe				= ctrl[9];			// parity enable
wire		   ctrl_tsb			   = ctrl[8];			// two stop bit
wire		   ctrl_txic			= ctrl[3];			// transmit interrupt control
wire		   ctrl_erie			= ctrl[2];			// error interrupt enable
wire		   ctrl_rxie			= ctrl[1];			// receive interrupt enable
wire		   ctrl_txie			= ctrl[0];			// transmitt interrupt enable

wire en_ctrl = en_dly & (reg_addr == `UART_REG_CTRL);
wire wr_ctrl = en_ctrl & write;
wire rd_ctrl = en_ctrl & ~write;
always @(posedge clk)
	if(reset)			ctrl <= 'h0;
	else if (wr_ctrl)	ctrl <= wdata;


// STATUS Register
//
wire en_status = en_dly & (reg_addr == `UART_REG_STATUS);
wire wr_status = en_status & write;
wire rd_status = en_status & ~write;
wire [31:0] status;
reg  [31:0] status_clr;
always @(posedge clk)
	if (reset)			   status_clr <= 32'h0;
	else if (wr_status)	status_clr <= ~wdata;
	else 				      status_clr <= 32'h0;

// DATA Register
//
reg [7:0] tx_data;
wire en_data = en_dly & (reg_addr == `UART_REG_DATA);
wire wr_data = en_data & write;
wire rd_data = en_data & ~write;
always @(posedge clk)
	if (reset)				tx_data <= 'h0;
	else if (wr_data)		tx_data <= wdata;


/*****************************************************************************
 * Transmitter
 ****************************************************************************/

// Tx Fifo
//
reg [7:0]	tx_fifo [3:0];
reg [1:0]	tx_fifo_rd;
reg [1:0]	tx_fifo_wr;
reg [2:0]	tx_fifo_cnt;

always @(posedge clk)
	if (reset)				tx_fifo_rd <= 2'b00;
	else if (tx_start)	tx_fifo_rd <= tx_fifo_rd + 2'b01;

always @(posedge clk)
	if (reset)				tx_fifo_wr <= 2'b00;
	else if (wr_data)		tx_fifo_wr <= tx_fifo_wr + 2'b01;

always @(posedge clk)
	if (wr_data)			tx_fifo[tx_fifo_wr]	<= wdata[7:0];

always @(posedge clk)
	if (reset)										         tx_fifo_cnt	<= 3'h0;
	else if (wr_data & ~tx_start & ~tx_fifo_full)	tx_fifo_cnt <= tx_fifo_cnt + 3'b001;
	else if (tx_start & ~wr_data)					      tx_fifo_cnt <= tx_fifo_cnt + 3'b111;

assign tx_fifo_empty	= (tx_fifo_cnt == 3'b000);
assign tx_fifo_full	= (tx_fifo_cnt == {1'b0, ctrl_txfs} + 3'h1);


// Send counter
//
reg [3:0]	tx_bit_cnt;
reg [13:0]	tx_cnt;

assign tx_start	= tx_done & ~tx_fifo_empty;
assign tx_bit_nxt = (tx_cnt == 14'h0) && (tx_bit_cnt != 4'h0);
assign tx_done	   = (tx_cnt == 14'h0) && (tx_bit_cnt == 4'h0);


always @(posedge clk)
	if (reset)				tx_cnt <= 14'h0;
	else if (tx_start)	tx_cnt <= ctrl_bdiv;
	else if (tx_bit_nxt)	tx_cnt <= ctrl_bdiv;
	else if (|tx_cnt)		tx_cnt <= tx_cnt + 14'h3fff;


always @(posedge clk)
	if (reset)				tx_bit_cnt <= 4'h0;
	else if (tx_start)	tx_bit_cnt <= 4'd10 + ctrl_pe + ctrl_tsb;
	else if (tx_bit_nxt)	tx_bit_cnt <= tx_bit_cnt + 4'b1111;


// Send fifo item
//
reg [9:0]	tx_buf;
wire		tx_parity_even	= ^wdata[7:0];
wire		tx_parity_odd	= ~tx_parity_even;
wire		tx_parity		= (~ctrl_pe) ? 1 : 
						(ctrl_pc) ? tx_parity_even : tx_parity_odd;
always @(posedge clk)
	if (reset)				tx_buf <= 10'h3FF;
	else if (tx_start)	tx_buf <= {tx_parity ,tx_fifo[tx_fifo_rd][7:0], 1'b0};
	else if (tx_bit_nxt)	tx_buf <= {1'b1, tx_buf[9:1]};


assign txd = tx_buf[0];

/*****************************************************************************
 * Receiver
 ****************************************************************************/

// Two of three encowdatag
//
reg			rxd_maj;
reg [1:0]	rxd_buf;

always @(posedge clk)
	if (reset)	rxd_buf <= 2'b11;
	else		rxd_buf <= {rxd_buf[0], rxd};

wire [1:0] rxd_maj_cnt = {1'b0, rxd}
						+ {1'b0, rxd_buf[0]}
						+ {1'b0, rxd_buf[1]};
wire       rxd_maj_nxt = (rxd_maj_cnt>=2'b10);

always @(posedge clk)
	if (reset)	rxd_maj <= 1'b1;
	else		rxd_maj <= rxd_maj_nxt;

wire rxd_fe = rxd_maj & ~rxd_maj_nxt;


// Receive counter
//
reg [3:0]	rx_bit_cnt;
reg [13:0]	rx_cnt;

wire rx_done	   = (rx_bit_cnt == 4'd10 + ctrl_pe + ctrl_tsb) & (rx_cnt == ctrl_bdiv[13:4]);   // aroung 1/16 finished before
wire rx_start	   = (rx_bit_cnt == 14'h0) & rxd_fe;
wire rx_bit_nxt   = (rx_bit_cnt != 14'h0) & (rx_bit_cnt != 4'd10 + ctrl_pe + ctrl_tsb) & (rx_cnt == 14'h0);
wire rx_sample	   = (rx_cnt == {2'b00, ctrl_bdiv[13:2]});								// sample around 3/4

always @(posedge clk)
	if (reset)				rx_cnt <= 14'h0;
	else if (rx_start)	rx_cnt <= ctrl_bdiv;
	else if (rx_bit_nxt)	rx_cnt <= ctrl_bdiv;
	else if (|rx_cnt)		rx_cnt <= rx_cnt + 14'h3fff;


always @(posedge clk)
	if (reset)				rx_bit_cnt <= 4'h0;
	else if (rx_start)	rx_bit_cnt <= 4'h1;
	else if (rx_done)		rx_bit_cnt <= 4'h0;
	else if (rx_bit_nxt)	rx_bit_cnt <= rx_bit_cnt + 4'h1;

// Receive buffer
//
reg [11:0]	rx_buf;
always @(posedge clk)
	if(reset)				rx_buf <= 12'hfff;
	else if (rx_start)	rx_buf <= 12'hfff;
	else if (rx_sample)	rx_buf[rx_bit_cnt-1] <= rxd_maj;


reg   rx_data_valid;
wire	rx_data_valid_nxt = (rx_buf[0] == 1'b0) & rx_parity_valid & rx_stop_valid;
always @ (*)
	begin
		rx_parity_valid = 1'b1;
		rx_stop_valid = 1'b1;
		if(ctrl_pe)	// parity enable
		  begin
			rx_parity_valid = ((ctrl_pc) ? ^rx_buf[8:1] : ~^rx_buf[8:1]);
			rx_stop_valid = (rx_buf[10] == 1'b1);
			if(ctrl_tsb) rx_stop_valid = rx_stop_valid & (rx_buf[11] == 1'b1);
		  end
		else
		  begin
			rx_stop_valid = (rx_buf[9] == 1'b1);
			if(ctrl_tsb) rx_stop_valid = rx_stop_valid & (rx_buf[10] == 1'b1);
		  end	
	end

always @(posedge clk)
	if (reset)			rx_data_valid <= 1'b1;
	else if (rx_done)	rx_data_valid <= rx_data_valid_nxt;
					 


// Rx Fifo
//
reg [7:0]	rx_fifo [3:0];
reg [1:0]	rx_fifo_rd;
reg [1:0]	rx_fifo_wr;
reg [2:0]	rx_fifo_cnt;

wire rx_fifo_ins = rx_done & ~rx_fifo_full & rx_data_valid_nxt;

always @(posedge clk)
	if (reset)							   rx_fifo_rd <= 2'b00;
	else if (rd_data)					   rx_fifo_rd <= rx_fifo_rd + 2'b01;

always @(posedge clk)
	if (reset)							   rx_fifo_wr <= 2'b00;
	else if (rx_fifo_ins)			   rx_fifo_wr <= rx_fifo_wr + 2'b01;

always @(posedge clk)
	if (rx_fifo_ins)					   rx_fifo[rx_fifo_wr]	<= rx_buf[8:1];

always @(posedge clk)
	if (reset)							   rx_fifo_cnt	<= 3'h0;
	else if (rx_fifo_ins & ~rd_data)	rx_fifo_cnt <= rx_fifo_cnt + 3'b001;
	else if (rd_data & ~rx_fifo_ins)	rx_fifo_cnt <= rx_fifo_cnt + 3'b111;

// Rx status
//
assign rx_fifo_empty	= (rx_fifo_cnt == 3'h0);
assign rx_fifo_full	= (rx_fifo_cnt == {1'b0, ctrl_rxfs} + 3'h1);
assign rx_fifo_ov		= (rx_fifo_full & rx_done);


/*****************************************************************************
 * Output
 ****************************************************************************/
// STATUS
//  -------------------------------------------------------------------------------
//  |           | tx_cnt | tx_full | tx_empty |      | rx_cnt | rx_full | rx_empty |
//  -------------------------------------------------------------------------------
//     31:12      12:10       9          8      7:6      5:2        1          0

assign status =  {16'h0, 
					3'h0, tx_fifo_cnt, tx_fifo_full, tx_fifo_empty,
					3'h0, rx_fifo_cnt, rx_fifo_full, rx_fifo_empty};

assign rdata = ({32{rd_ctrl}}		& ctrl)
			| ({32{rd_status}}		& status)
			| ({32{rd_data}}		& {24'h0, rx_fifo[rx_fifo_rd]});


always @(posedge clk)
   if(reset)   badmem <= 1'b0;
   else        badmem <= en & (`UART_REG_LAST < addr);

assign bus_wait = 1'b0;

/*****************************************************************************
 * Interrupt
 ****************************************************************************/
// A transmit interrupt can be configured to raise by a empty buffer
// or by the last sent byte
assign irq_tx	= ctrl_txie & tx_fifo_empty		// tx fifo is empty
					& (ctrl_txic ? 1'b1 : tx_done);	// tx fifo empty and finish sent

assign irq_rx	= ctrl_rxie & rx_fifo_full;		// rx fifo full

assign irq_err	= ctrl_erie & 
					(rx_done & ~rx_data_valid		// start, parity or stopbit failure
					| rx_fifo_full & rx_done);		// overflow


endmodule //uart
