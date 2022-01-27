SIM_TOP = top_tb
SIM_VSRC = top_tb.sv

VSRC := 
VSRC +=				 ../rtl/peripherals/smartv_peripherals.vh      		
 
 
VSRC +=              ../rtl/peripherals/clk_gen.sv 
VSRC +=				 ../rtl/peripherals/gpio.sv 
VSRC +=				 ../rtl/peripherals/uart_wrap.sv
VSRC +=				 ../rtl/peripherals/uart.v  
VSRC +=				 ../rtl/memories/byte_data_memory.sv  
VSRC +=				 ../rtl/memories/data_memory.sv  
VSRC +=				 ../rtl/memories/instruction_memory.sv 			 
VSRC +=		 		 ../rtl/peripherals/memory_controller.sv  


VSRC +=				 ../rtl/riscv/include/riscv_defines.sv             
VSRC +=				 ../rtl/riscv/include/riscv_tracer_defines.sv   

VSRC +=				 ../rtl/peripherals/peripheral_controller.sv	

VSRC +=				 ../rtl/riscv/riscv_alu.sv                         
VSRC +=				 ../rtl/riscv/riscv_alu_div.sv                     
VSRC +=				 ../rtl/riscv/riscv_compressed_decoder.sv          
VSRC +=				 ../rtl/riscv/riscv_controller.sv                  
VSRC +=				 ../rtl/riscv/riscv_cs_registers.sv                
VSRC +=				 ../rtl/riscv/riscv_decoder.sv                     
VSRC +=				 ../rtl/riscv/riscv_int_controller.sv              
VSRC +=				 ../rtl/riscv/riscv_ex_stage.sv                    
VSRC +=				 ../rtl/riscv/riscv_id_stage.sv                    
VSRC +=				 ../rtl/riscv/riscv_fetch_fifo.sv                  
VSRC +=				 ../rtl/riscv/riscv_if_stage.sv                    
VSRC +=				 ../rtl/riscv/riscv_load_store_unit.sv             
VSRC +=				 ../rtl/riscv/riscv_mult.sv                        
VSRC +=				 ../rtl/riscv/riscv_prefetch_buffer.sv             
VSRC +=				 ../rtl/riscv/riscv_register_file.sv               
VSRC +=				 ../rtl/riscv/riscv_core.sv                        
#VSRC +=				 ../rtl/riscv/riscv_smartv.sv	
VSRC +=				 ../rtl/riscv/riscv_praterv_no.sv		
#VSRC +=				 ../rtl/riscv/riscv_praterv.sv		   
VSRC +=				 ../rtl/top.sv									   

			 

VINC = ../rtl/riscv/include 

SPACE :=
SPACE +=


BOARD=basys3
PART=xc7a35tcpg236-1




MEMFILE=../../../test_files/mret_init.mem
MEMFILE_SIM=../../../test_files/mret_init.mem
ELFFILE=../../../test_files/gpioIE.elf


#export BOARD
#export PART
#export MEMFILE
#export MEMFILE_SIM
#export ELFFILE

.PHONY: all fpga elab sim sim-run-gui clean

all: fpga

fpga:
	cd work; vivado -mode tcl -source ../bin/build.tcl -tclargs "$(VSRC)" "$(VINC)" "$(BOARD)" "$(PART)" "$(MEMFILE)"; cd ..

elab:	
	@echo "$(MEMFILE)"
	@echo $(INC)
	@echo "Parsing HDL"
	@cd sim && xvlog --sv $(VSRC) $(SIM_VSRC) $(VINC:%=-i$(SPACE)%)
	@echo "Elaborating design"
	@cd sim && xelab $(SIM_TOP) --incr -L unisim --debug all --timescale 1ns/1ps  --generic_top "ROM_FILE_SIM=$(MEMFILE_SIM)"
	
sim:
	make elab
	@echo "Starting simulation (gui)"
	@cd sim && xsim $(SIM_TOP) --gui -t sim.tcl

update_rom:
	@echo "UPDATE ROM"		
	./bin/update_bit_file.sh 

clean:
	rm -fr work/*.jou work/*.log work/*.html work/*.xml work/.Xil work/build/*
	rm -fr sim/*.jou sim/*.log sim/*.html sim/*.xml sim/*.pb sim/*.wdb sim/*.zip sim/.Xil sim/xsim.dir
