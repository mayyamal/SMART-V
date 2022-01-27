## Switching between SMART-V and PRATER-V

For now the hardware for SMART-V, PRATER-V, and no MPU is created as a separate module in this branch.

The enabling/disabling is done in the following files:
1. In `makefile`:
```shell
	VSRC += ../rtl/riscv/riscv_smartv.sv	 OR
	VSRC +=	../rtl/riscv/riscv_praterv_no.sv OR		
	VSRC +=	../rtl/riscv/riscv_praterv.sv	
```
2. In `riscv_defines` choose either
```shell
	`define PRATERV_EN 1  OR
	`define SMARTV_EN 1
```

3.  In `riscv_core` leave everything as it is, except if using no MPU, change the instantiation name from `riscv_praterv` to `riscv_praterv_no`

That's basically it.
