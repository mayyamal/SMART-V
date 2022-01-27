module top_tb
#(parameter ROM_FILE_SIM = "")
();

logic clk;
logic [15:0] leds;
logic [15:0] switches;
logic uart_tx;
logic irq;
logic irq_ack;

initial
begin
  clk = 0;
  irq = 0;
  //repeat(1000) @(posedge clk);
  //$display("SIMULATION END"); 
  //$finish;
end 

always
begin
  clk = 1;
  #10;  /// 10ns = 100MHz
  clk = 0;
  #10;
 // switches = 16'hA500;
end

top
  #(
	.ROM_FILE( ROM_FILE_SIM )
  )
dut 
  (
    .clk_i ( clk ),
    .leds_o ( leds ),
    .uart_tx_o ( uart_tx ),
	.sw (switches)
    //.irq_i ( irq ), 
    //.irq_ack_o( irq_ack )
  );



always @(posedge clk) begin    

	
	 //if(dut.riscv_core_i.pc_id == 32'hcf0)
	 //$stop;	
	 /*
	 //CASE 1
	 if(dut.riscv_core_i.pc_id == 32'h80000020) //localVar
	 $stop;	 
	 if(dut.riscv_core_i.pc_id == 32'hf80) //moveMData csrw
	 $stop;
	 
	 //CASE 2
	 if(dut.riscv_core_i.pc_id == 32'h80000064) //localFunc
	 $stop;
	 if(dut.riscv_core_i.pc_id == 32'he28) //moveMCode csrw
	 $stop;
	 */
	 //CASE 5	 
	 // if(dut.riscv_core_i.pc_id == 32'h80000148) //LEDSetState
	 // $stop;
	 //if(dut.riscv_core_i.pc_id == 32'hfb0) //updateEM csrw
	 //$stop;
	 
	 
	 //if(dut.riscv_core_i.pc_id == 32'hba000090)
	 //stop;	
	 
	 //LEDsetState(LED_01, LED_ON);
     //updateEM();
	 /*
	 if(dut.riscv_core_i.pc_id == 32'h80000074) //LEDSetState
	 $stop;
	 if(dut.riscv_core_i.pc_id == 32'h80000088) //LEDSetState
	 $stop;
	 if(dut.riscv_core_i.pc_id == 32'hBA00000C) //LEDSetState
	 $stop;
	 //if(dut.riscv_core_i.pc_id == 32'h00001018) //LEDSetState
	 //$stop;
	 if(dut.riscv_core_i.pc_id == 32'h8000009C) //LEDSetState
	 $stop;
	 if(dut.riscv_core_i.pc_id == 32'h800000a8) //LEDSetState
	 $stop;
	 if(dut.riscv_core_i.pc_id == 32'hBA00008C) //LEDSetState
	 $stop;
	 
	 if(dut.riscv_core_i.pc_id == 32'h800000BC) //LEDSetState
	 $stop;
	 if(dut.riscv_core_i.pc_id == 32'h800000C8) //LEDSetState
	 $stop;
	 if(dut.riscv_core_i.pc_id == 32'hBA00008C) //LEDSetState
	 $stop;
	 */
	 //if(dut.riscv_core_i.pc_id == 32'h800005e0) //
	 //$stop;
	// if(dut.riscv_core_i.pc_id == 32'h80000048) //setEvent MTask
//	 $stop;
	// if(dut.riscv_core_i.pc_id == 32'h80000060) //waitEventUntil MTask
	// $stop;
	// if(dut.riscv_core_i.pc_id == 32'h80000b28) //benchmark_mont64
	// $stop;
	 // if(dut.riscv_core_i.pc_id == 32'h1af8) //benchmark_mont64
	// $stop;
	//if(dut.riscv_core_i.pc_id == 32'h80000b1c) //benchmark_mont64
	// $stop;
	// if(dut.riscv_core_i.pc_id == 32'h80000fa4) //benchmark_mont64
	//// $stop;
	 //if(dut.riscv_core_i.pc_id == 32'h8000145c) //benchmark_mont64
	// $stop;
	 
	 //if(dut.riscv_core_i.pc_id == 32'h80000f3c) //benchmark_mont64
	 //$stop;
	 if(dut.riscv_core_i.pc_id == 32'h00003620) //benchmark_mont64
	 $stop;
	 if(dut.riscv_core_i.pc_id == 32'h0000364c) //benchmark_mont64	 
	 $stop;
	 if(dut.riscv_core_i.pc_id == 32'h0000365c) //benchmark_mont64	 
	 $stop;
	 
	 
	
	 
	 
end	



//*********************** MM TO USE *****************************
//***************************************************************
//***************************************************************



// $display() // displays once every time it is executed
// $monitor("At time %t, d = %h", $time, var) // displays every time one of its parameters changes



/*wire [31:0] exc_t_0 = dut.mmio_controller_i.ram_inst.dbg_ram[202];
wire [31:0] exc_t_1 = dut.mmio_controller_i.ram_inst.dbg_ram[203];
wire [31:0] exc_t_2 = dut.mmio_controller_i.ram_inst.dbg_ram[204];
wire [31:0] exc_t_3 = dut.mmio_controller_i.ram_inst.dbg_ram[205];
wire [31:0] exc_t_4 = dut.mmio_controller_i.ram_inst.dbg_ram[206];
wire [31:0] exc_t_5 = dut.mmio_controller_i.ram_inst.dbg_ram[207];
wire [31:0] exc_t_6 = dut.mmio_controller_i.ram_inst.dbg_ram[208];
wire [31:0] exc_t_7 = dut.mmio_controller_i.ram_inst.dbg_ram[209];
wire [31:0] exc_t_8 = dut.mmio_controller_i.ram_inst.dbg_ram[210];
wire [31:0] exc_t_9 = dut.mmio_controller_i.ram_inst.dbg_ram[211];*/
//----------- Instruction Address and Data -----------/
//wire [31:0] if_stage_instr_addr_o = dut.riscv_core_i.if_stage_i.instr_addr_o;
//wire [31:0] mmio_instr_addr_i = dut.mmio_controller_i.instr_addr_i;



//wire [31:0] IF_pc_if_o = dut.riscv_core_i.if_stage_i.pc_if_o;
//wire [31:0] IF_pc_id_o = dut.riscv_core_i.if_stage_i.pc_id_o;

/*

wire [31:0] IF_fetch_addr = dut.riscv_core_i.if_stage_i.prefetch_32.prefetch_buffer_i.fetch_addr;
wire [31:0] IF_branch_i = dut.riscv_core_i.if_stage_i.prefetch_32.prefetch_buffer_i.branch_i;
wire [31:0] IF_addr_i = dut.riscv_core_i.if_stage_i.prefetch_32.prefetch_buffer_i.addr_i;
wire [31:0] IF_instr_addr_q = dut.riscv_core_i.if_stage_i.prefetch_32.prefetch_buffer_i.instr_addr_q;

*/
//Important Varuables:
/*
wire [31:0] os_readyQueue = dut.mmio_controller_i.ram_inst.dbg_ram[688];

wire [31:0] os_timeoutQueue = dut.mmio_controller_i.ram_inst.dbg_ram[687];

wire [31:0] os_runningTask = dut.mmio_controller_i.ram_inst.dbg_ram[703];

wire [31:0] os_stack_end  = dut.mmio_controller_i.ram_inst.dbg_ram[705];

wire [31:0] os_taskStacks_end = dut.mmio_controller_i.ram_inst.dbg_ram[704];

wire [31:0] os_tcbs_end = dut.mmio_controller_i.ram_inst.dbg_ram[679];
*/

//Task Constructors - constant location
/*
wire [31:0] taskConstr1 =  dut.mmio_controller_i.rom_dut.dpram_inst.mem[5120];
wire [31:0] taskConstr1Entry =  dut.mmio_controller_i.rom_dut.dpram_inst.mem[5121];
wire [31:0] taskConstr1Prio =  dut.mmio_controller_i.rom_dut.dpram_inst.mem[5122];
wire [31:0] taskConstr1Stack =  dut.mmio_controller_i.rom_dut.dpram_inst.mem[5123];
wire [31:0] taskConstr1Periph =  dut.mmio_controller_i.rom_dut.dpram_inst.mem[5124];

wire [31:0] taskConstrIdle =  dut.mmio_controller_i.rom_dut.dpram_inst.mem[5126];
wire [31:0] taskConstrIdleEntry =  dut.mmio_controller_i.rom_dut.dpram_inst.mem[5127];
wire [31:0] taskConstrIdlePrio =  dut.mmio_controller_i.rom_dut.dpram_inst.mem[5128];
wire [31:0] taskConstrIdleStack =  dut.mmio_controller_i.rom_dut.dpram_inst.mem[5129];
wire [31:0] taskConstrIdlePeriph =  dut.mmio_controller_i.rom_dut.dpram_inst.mem[5130];
*/
//TCBs

wire  [31:0] localVarsUP [300:0] = dut.memory_controller_i.data_memory_dut.dbg_ram[96:396];

wire [31:0] localVarOld = dut.memory_controller_i.data_memory_dut.dbg_ram[396];
wire [31:0] localVarNew3up = dut.memory_controller_i.data_memory_dut.dbg_ram[649];
wire [31:0] localVarNew2up = dut.memory_controller_i.data_memory_dut.dbg_ram[650];
wire [31:0] localVarNew1up = dut.memory_controller_i.data_memory_dut.dbg_ram[651];
wire [31:0] localVarNew = dut.memory_controller_i.data_memory_dut.dbg_ram[652];
wire [31:0] localVarNew1down = dut.memory_controller_i.data_memory_dut.dbg_ram[653];
wire [31:0] localVarNew2down = dut.memory_controller_i.data_memory_dut.dbg_ram[654];
wire [31:0] localVarNew3down = dut.memory_controller_i.data_memory_dut.dbg_ram[655];
/*
*/

wire  [31:0] localVarsdown [350:0] = dut.memory_controller_i.data_memory_dut.dbg_ram[656:1006];


wire [31:0] t1_ctx = dut.memory_controller_i.data_memory_dut.dbg_ram[0];
wire [31:0] t1_name = dut.memory_controller_i.data_memory_dut.dbg_ram[1];
wire [31:0] t1_stackHA = dut.memory_controller_i.data_memory_dut.dbg_ram[2];
wire [31:0] t1_stackLA = dut.memory_controller_i.data_memory_dut.dbg_ram[3];
wire [31:0] t1_basePrio = dut.memory_controller_i.data_memory_dut.dbg_ram[4];
wire [63:0] t1_timeout = dut.memory_controller_i.data_memory_dut.dbg_ram[5];
wire [31:0] t1_X = dut.memory_controller_i.data_memory_dut.dbg_ram[7];
wire [31:0] t1_entry = dut.memory_controller_i.data_memory_dut.dbg_ram[8];
wire [31:0] t1_prioListPrev = dut.memory_controller_i.data_memory_dut.dbg_ram[9];
wire [31:0] t1_prioListnext = dut.memory_controller_i.data_memory_dut.dbg_ram[10];
wire [63:0] t1_timeoutList = dut.memory_controller_i.data_memory_dut.dbg_ram[11];
wire [31:0] t1_id = dut.memory_controller_i.data_memory_dut.dbg_ram[13];
wire [31:0] t1_mpu_CS = dut.memory_controller_i.data_memory_dut.dbg_ram[14];
wire [31:0] t1_mpu_CE = dut.memory_controller_i.data_memory_dut.dbg_ram[15];
wire [31:0] t1_mpu_DS = dut.memory_controller_i.data_memory_dut.dbg_ram[16];
wire [31:0] t1_mpu_DE = dut.memory_controller_i.data_memory_dut.dbg_ram[17];
wire [31:0] t1_mpu_LS = dut.memory_controller_i.data_memory_dut.dbg_ram[18];
wire [31:0] t1_mpu_LE = dut.memory_controller_i.data_memory_dut.dbg_ram[19];

wire [31:0] t1_mpu_periph = dut.memory_controller_i.data_memory_dut.dbg_ram[20];

wire [31:0] t1_relocation_code = dut.memory_controller_i.data_memory_dut.dbg_ram[21];
wire [31:0] t1_relocation_data = dut.memory_controller_i.data_memory_dut.dbg_ram[22];
wire [31:0] t1_relocation_lib = dut.memory_controller_i.data_memory_dut.dbg_ram[23];
wire [31:0] t1_owned_resources = dut.memory_controller_i.data_memory_dut.dbg_ram[24];


wire [31:0] t2_ctx = dut.memory_controller_i.data_memory_dut.dbg_ram[26];
wire [31:0] t2_name = dut.memory_controller_i.data_memory_dut.dbg_ram[27];
wire [31:0] t2_stackHA = dut.memory_controller_i.data_memory_dut.dbg_ram[28];
wire [31:0] t2_stackLA = dut.memory_controller_i.data_memory_dut.dbg_ram[29];
wire [31:0] t2_basePrio = dut.memory_controller_i.data_memory_dut.dbg_ram[30];
wire [63:0] t2_timeout = dut.memory_controller_i.data_memory_dut.dbg_ram[31];
wire [31:0] t2_X = dut.memory_controller_i.data_memory_dut.dbg_ram[33];
wire [31:0] t2_entry = dut.memory_controller_i.data_memory_dut.dbg_ram[34];
wire [31:0] t2_prioListPrev = dut.memory_controller_i.data_memory_dut.dbg_ram[35];
wire [31:0] t2_prioListnext = dut.memory_controller_i.data_memory_dut.dbg_ram[36];
wire [63:0] t2_timeoutList = dut.memory_controller_i.data_memory_dut.dbg_ram[37];
wire [31:0] t2_id = dut.memory_controller_i.data_memory_dut.dbg_ram[39];
wire [31:0] t2_mpu_CS = dut.memory_controller_i.data_memory_dut.dbg_ram[40];
wire [31:0] t2_mpu_CE = dut.memory_controller_i.data_memory_dut.dbg_ram[41];
wire [31:0] t2_mpu_DS = dut.memory_controller_i.data_memory_dut.dbg_ram[42];
wire [31:0] t2_mpu_DE = dut.memory_controller_i.data_memory_dut.dbg_ram[43];
wire [31:0] t2_mpu_LS = dut.memory_controller_i.data_memory_dut.dbg_ram[44];
wire [31:0] t2_mpu_LE = dut.memory_controller_i.data_memory_dut.dbg_ram[44];

wire [31:0] t2_mpu_periph = dut.memory_controller_i.data_memory_dut.dbg_ram[46];

wire [31:0] t2_relocation_code = dut.memory_controller_i.data_memory_dut.dbg_ram[47];
wire [31:0] t2_relocation_data = dut.memory_controller_i.data_memory_dut.dbg_ram[48];
wire [31:0] t2_relocation_lib = dut.memory_controller_i.data_memory_dut.dbg_ram[49];

wire [31:0] t2_owned_resources = dut.memory_controller_i.data_memory_dut.dbg_ram[50];


wire [31:0] t3_ctx = dut.memory_controller_i.data_memory_dut.dbg_ram[52];
wire [31:0] t3_name = dut.memory_controller_i.data_memory_dut.dbg_ram[53];
wire [31:0] t3_stackHA = dut.memory_controller_i.data_memory_dut.dbg_ram[54];
wire [31:0] t3_stackLA = dut.memory_controller_i.data_memory_dut.dbg_ram[55];
wire [31:0] t3_basePrio = dut.memory_controller_i.data_memory_dut.dbg_ram[56];
wire [63:0] t3_timeout = dut.memory_controller_i.data_memory_dut.dbg_ram[57];
wire [31:0] t3_X = dut.memory_controller_i.data_memory_dut.dbg_ram[59];
wire [31:0] t3_entry = dut.memory_controller_i.data_memory_dut.dbg_ram[60];
wire [31:0] t3_prioListPrev = dut.memory_controller_i.data_memory_dut.dbg_ram[61];
wire [31:0] t3_prioListnext = dut.memory_controller_i.data_memory_dut.dbg_ram[62];
wire [63:0] t3_timeoutList = dut.memory_controller_i.data_memory_dut.dbg_ram[63];
wire [31:0] t3_id = dut.memory_controller_i.data_memory_dut.dbg_ram[65];
wire [31:0] t3_mpu_CS = dut.memory_controller_i.data_memory_dut.dbg_ram[66];
wire [31:0] t3_mpu_CE = dut.memory_controller_i.data_memory_dut.dbg_ram[67];
wire [31:0] t3_mpu_DS = dut.memory_controller_i.data_memory_dut.dbg_ram[68];
wire [31:0] t3_mpu_DE = dut.memory_controller_i.data_memory_dut.dbg_ram[69];
wire [31:0] t3_mpu_LS = dut.memory_controller_i.data_memory_dut.dbg_ram[70];
wire [31:0] t3_mpu_LE = dut.memory_controller_i.data_memory_dut.dbg_ram[71];

wire [31:0] t3_mpu_periph = dut.memory_controller_i.data_memory_dut.dbg_ram[72];

wire [31:0] t3_relocation_code = dut.memory_controller_i.data_memory_dut.dbg_ram[73];
wire [31:0] t3_relocation_data = dut.memory_controller_i.data_memory_dut.dbg_ram[74];
wire [31:0] t3_relocation_lib = dut.memory_controller_i.data_memory_dut.dbg_ram[75];

wire [31:0] t3_owned_resources = dut.memory_controller_i.data_memory_dut.dbg_ram[76];

/*


wire [31:0] t4_ctx = dut.mmio_controller_i.ram_inst.dbg_ram[78];
wire [31:0] t4_name = dut.mmio_controller_i.ram_inst.dbg_ram[79];
wire [31:0] t4_stackHA = dut.mmio_controller_i.ram_inst.dbg_ram[80];
wire [31:0] t4_stackLA = dut.mmio_controller_i.ram_inst.dbg_ram[81];
wire [31:0] t4_basePrio = dut.mmio_controller_i.ram_inst.dbg_ram[82];
wire [63:0] t4_timeout = dut.mmio_controller_i.ram_inst.dbg_ram[83];
wire [31:0] t4_X = dut.mmio_controller_i.ram_inst.dbg_ram[85];
wire [31:0] t4_entry = dut.mmio_controller_i.ram_inst.dbg_ram[86];
wire [31:0] t4_prioListPrev = dut.mmio_controller_i.ram_inst.dbg_ram[87];
wire [31:0] t4_prioListnext = dut.mmio_controller_i.ram_inst.dbg_ram[88];
wire [63:0] t4_timeoutList = dut.mmio_controller_i.ram_inst.dbg_ram[89];
wire [31:0] t4_id = dut.mmio_controller_i.ram_inst.dbg_ram[91];
wire [31:0] t4_mpu_CS = dut.mmio_controller_i.ram_inst.dbg_ram[92];
wire [31:0] t4_mpu_CE = dut.mmio_controller_i.ram_inst.dbg_ram[93];
wire [31:0] t4_mpu_DS = dut.mmio_controller_i.ram_inst.dbg_ram[94];
wire [31:0] t4_mpu_DE = dut.mmio_controller_i.ram_inst.dbg_ram[95];
wire [31:0] t4_mpu_LS = dut.mmio_controller_i.ram_inst.dbg_ram[96];
wire [31:0] t4_mpu_LE = dut.mmio_controller_i.ram_inst.dbg_ram[97];

wire [31:0] t4_mpu_periph = dut.mmio_controller_i.ram_inst.dbg_ram[98];

wire [31:0] t4_relocation_code = dut.mmio_controller_i.ram_inst.dbg_ram[99];
wire [31:0] t4_relocation_data = dut.mmio_controller_i.ram_inst.dbg_ram[100];
wire [31:0] t4_relocation_lib = dut.mmio_controller_i.ram_inst.dbg_ram[101];

wire [31:0] t4_owned_resources = dut.mmio_controller_i.ram_inst.dbg_ram[102];




wire [31:0] GPIO_counter = dut.mmio_controller_i.ram_inst.dbg_ram[320];
wire [31:0] GPIO_owner = dut.mmio_controller_i.ram_inst.dbg_ram[321];
wire [31:0] GPIO_prio = dut.mmio_controller_i.ram_inst.dbg_ram[322];
wire [31:0] UART_counter= dut.mmio_controller_i.ram_inst.dbg_ram[323];
wire [31:0] UART_owner = dut.mmio_controller_i.ram_inst.dbg_ram[324];
wire [31:0] UART_prio= dut.mmio_controller_i.ram_inst.dbg_ram[325];

*/
/*
wire [31:0] taskConstr2 =  dut.hasti_mem.mem[3475];
wire [31:0] taskConstr2Entry =  dut.hasti_mem.mem[3476];
wire [31:0] taskConstr2Prio =  dut.hasti_mem.mem[3477];
wire [31:0] taskConstr2Stack =  dut.hasti_mem.mem[3478];
wire [31:0] taskConstr2Periph =  dut.hasti_mem.mem[3479];
wire [31:0] taskConstr3 =  dut.hasti_mem.mem[3480];
wire [31:0] taskConstr3Entry =  dut.hasti_mem.mem[3481];
wire [31:0] taskConstr3Prio =  dut.hasti_mem.mem[3482];
wire [31:0] taskConstr3Stack =  dut.hasti_mem.mem[3483];
wire [31:0] taskConstr3Periph =  dut.hasti_mem.mem[3484];
*/
//----------- Register File -----------/
wire [31:0] ra = dut.riscv_core_i.id_stage_i.registers_i.mem[1];
wire [31:0] sp = dut.riscv_core_i.id_stage_i.registers_i.mem[2];
wire [31:0] gp = dut.riscv_core_i.id_stage_i.registers_i.mem[3];
wire [31:0] tp = dut.riscv_core_i.id_stage_i.registers_i.mem[4];


wire [31:0] a4 = dut.riscv_core_i.id_stage_i.registers_i.mem[14]; 
wire [31:0] a5 = dut.riscv_core_i.id_stage_i.registers_i.mem[15];
wire [31:0] a6 = dut.riscv_core_i.id_stage_i.registers_i.mem[16];
wire [31:0] a7 = dut.riscv_core_i.id_stage_i.registers_i.mem[17];

wire [31:0] t3 = dut.riscv_core_i.id_stage_i.registers_i.mem[28];

wire [31:0] t4 = dut.riscv_core_i.id_stage_i.registers_i.mem[29];
wire [31:0] t5 = dut.riscv_core_i.id_stage_i.registers_i.mem[30];
wire [31:0] t6 = dut.riscv_core_i.id_stage_i.registers_i.mem[31];   


wire [31:0] t0 = dut.riscv_core_i.id_stage_i.registers_i.mem[5];
wire [31:0] t1 = dut.riscv_core_i.id_stage_i.registers_i.mem[6];
wire [31:0] t2 = dut.riscv_core_i.id_stage_i.registers_i.mem[7];

wire [31:0] s0 = dut.riscv_core_i.id_stage_i.registers_i.mem[8];
wire [31:0] s1 = dut.riscv_core_i.id_stage_i.registers_i.mem[9];
wire [31:0] a0 = dut.riscv_core_i.id_stage_i.registers_i.mem[10];
wire [31:0] a1 = dut.riscv_core_i.id_stage_i.registers_i.mem[11];
wire [31:0] a2 = dut.riscv_core_i.id_stage_i.registers_i.mem[12];
wire [31:0] a3 = dut.riscv_core_i.id_stage_i.registers_i.mem[13];
 

wire [31:0] s2 = dut.riscv_core_i.id_stage_i.registers_i.mem[18];
wire [31:0] s3 = dut.riscv_core_i.id_stage_i.registers_i.mem[19];


wire [31:0] s4 = dut.riscv_core_i.id_stage_i.registers_i.mem[20];
wire [31:0] s5 = dut.riscv_core_i.id_stage_i.registers_i.mem[21];

wire [31:0] s6 = dut.riscv_core_i.id_stage_i.registers_i.mem[22];
wire [31:0] s7 = dut.riscv_core_i.id_stage_i.registers_i.mem[23];
wire [31:0] s8 = dut.riscv_core_i.id_stage_i.registers_i.mem[24];
wire [31:0] s9 = dut.riscv_core_i.id_stage_i.registers_i.mem[25];
wire [31:0] s10 = dut.riscv_core_i.id_stage_i.registers_i.mem[26];
wire [31:0] s11 = dut.riscv_core_i.id_stage_i.registers_i.mem[27];




endmodule	// top


