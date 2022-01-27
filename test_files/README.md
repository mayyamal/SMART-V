# Test Files

This directory contains simple asm files to test the pipeline of RI5CY.
The linker script corresponds to the current memory layout.

## Quick Start

Don't forget to initialize your environment as described in `../mmri5cy/README.md`

Compile the asm files
```bash
make all
```

The `.mem` file of the code you want to run on the cpu should be entered as 
`MEMFILE`, `MEMFILE_SIM`, while the `.elf` file as `ELFFILE` in `mmri5cy-V/hw/makefile`

Then run 
```bash
make sim
```
from `mmri5cy/hw`


