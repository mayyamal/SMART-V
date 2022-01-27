default: all

CC_PREFIX ?= /mnt/c/ubuntu_playgrounds/riscv/bin/riscv32-unknown-elf-
GCC = $(CC_PREFIX)gcc
AS  = $(CC_PREFIX)gcc
AR  = $(CC_PREFIX)ar
LD  = $(CC_PREFIX)ld
SZ  = $(CC_PREFIX)size
NM  = $(CC_PREFIX)nm
OC  = $(CC_PREFIX)objcopy
OD  = $(CC_PREFIX)objdump
ST  = $(CC_PREFIX)strip

RISCV_GCC_OPTS ?= -nostartfiles -nostdlib -Tlink.ld -march=rv32im


SRCS = 	$(wildcard *.c) $(wildcard *.S)

SRCC = $(wildcard *.c)
SRCA = $(wildcard *.S)


PROGS = $(patsubst %.S,%.elf,$(SRCA)) $(patsubst %.c,%.elf,$(SRCC))

HEX = $(patsubst %.S,%.hex,$(SRCA)) $(patsubst %.c,%.hex,$(SRCC))

VHEX = $(patsubst %.S,%.vhex,$(SRCA)) $(patsubst %.c,%.vhex,$(SRCC))

DUMPS =	$(patsubst %.S,%.dump,$(SRCA)) $(patsubst %.c,%.dump,$(SRCC))

BINNAMES = $(patsubst %.S,%.bin,$(SRCA)) $(patsubst %.c,%.bin,$(SRCC))

MEMNAMES = $(patsubst %.S,%.mem,$(SRCA)) $(patsubst %.c,%.mem,$(SRCC))

INITMEMNAMES = $(patsubst %.S,%_init.mem,$(SRCA)) $(patsubst %.c,%_init.mem,$(SRCC))

#BINNAME_$(1)  = $$(NAME).$(1).bin
#INITMEMNAME$(1)  = $$(NAME).$(1)_init.mem
#MEMNAME$(1)  = $$(NAME).$(1).mem


## RI5CY section
	#@echo ">> Creating bin from elf: [$$(BINNAME_$(1))]"
	#@$$(OC_$(1)) -O binary $$@ $$(BINNAME_$(1))
	
	#@echo ">> Creating _init.mem from bin: [$$(INITMEMNAME$(1))]"
	#@$$(BIN2MEM) $$(BINNAME_$(1)) > $$(INITMEMNAME$(1))
		
	#@echo ">> Creating mem from bin: [$$(MEMNAME$(1))]"
	#@$$(BIN2MEM) $$(BINNAME_$(1)) -r > $$(MEMNAME$(1))

all: $(SRCS) $(PROGS) $(HEX) $(DUMPS) $(BINNAMES) $(MEMNAMES) $(INITMEMNAMES) #$(VHEX)


	@echo 'Srcs'
	@echo $(SRCS)
	@echo 'Progs'	
	@echo $(PROGS) 
	@echo 'Dumps'
	@echo $(DUMPS)
	@echo 'Hexs'
	@echo $(HEX)
	@echo 'Vhexs'	
	@echo $(VHEX) 
	@echo 'Bins'	
	@echo $(BINNAMES)
	@echo 'Mems'	
	@echo $(MEMNAMES)  
	

%.elf: %.S   
	$(AS) $(RISCV_GCC_OPTS) -o $@ $<
	
%.elf: %.c	
	$(GCC)  $(RISCV_GCC_OPTS) -o $@ $<
	
%.hex: %.elf
	#$(OC) -O ihex --pad-to 0x2000 $^ $@
	$(OC) -O ihex  $^ $@ 
	
#%.vhex: %.hex
	#./hex2vhex.tcl -input $^ -output $@ -offset 0 -width 4 -endianswap 1
		
%.dump: %.elf
	$(OD) -d -r  $< > $@


%.bin: %.elf
	@echo ">> Creating bin from elf:"
	#$(OC) -O binary --pad-to 0x2000 $< $@
	$(OC) -O binary  $< $@

%.mem: %.bin	
	@echo ">> Creating mem from bin:"
	../sw/buildenv/util/bin2mem/bin2mem	$^ -r > $@

%_init.mem: %.bin	
	@echo ">> Creating _init.mem from bin"
	../sw/buildenv/util/bin2mem/bin2mem	$^  > $@	

	
clean: 
	rm -r $(PROGS) $(HEX)  $(DUMPS) $(BINNAMES) $(MEMNAMES) $(INITMEMNAMES) #$(VHEX)
