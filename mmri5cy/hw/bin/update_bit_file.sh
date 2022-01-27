#!/bin/sh

BUILD_DIR=work/build
SW_DIR=../../../sw
HW_DIR=../..

cd $BUILD_DIR


updatemem -force -meminfo memory_layout.mmi -data ../${ELFFILE} -bit fpga_${BOARD}.bit -proc top -out fpga_${BOARD}.bit
#updatemem -force -meminfo memory_layout.mmi -data ../${MEMFILE} -bit fpga_${BOARD}.bit -proc top -out fpga_${BOARD}.bit

cd ..