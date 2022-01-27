#Created by stephenm@xilinx.com. This is not supported by WTS
#Adapted by mauroner@tugraz.at
#The mem_name is the name of the Block RAM in the BD.
#This works with a memory range 0K - 1M and supports only one Bus block
#This only supports data width of 32 bits.

#Check UG898 for more informations
#https://forums.xilinx.com/t5/Embedded-Development-Tools/MEM-file-in-vivado/td-p/652370
#https://www.xilinx.com/support/answers/63041.html

# =====================
# Memory Layout
#
# byte_num   3        2         1       0
#									reg1 reg0
#	
#       31 ... 24|23 .. 16|15 ..  8|7  ..   0
#   Add	------------------------------------
#	0x0	|  mem3  |  mem2  |  mem1  |  mem0  |
#		------------------------------------
#	0x4	|  mem3  |  mem2  |  mem1  |  mem0  |
#		------------------------------------
#	0x8	|  mem3  |  mem2  |  mem1  |  mem0  |
#		------------------------------------
#	 .					.
#	 .					.
#	    ------------------------------------
#		|  mem3  |  mem2  |  mem1  |  mem0  |
#		------------------------------------
#	END	|  mem3  |  mem2  |  mem1  |  mem0  |
#		------------------------------------


# write_mmi rom vscale_pipeline Little
proc write_mmi {mem_name cpu_name endianess part {offset 0} {folder .} } {
	set proj [current_project]
	
	#set part [get_property PART [current_project ]]
	puts "Device part is $part"
	
	set filename $folder/memory_layout.mmi
	set fileout [open $filename "w"]

	# count number of CPUs
	#set proc_found 0	
	#set inst_path [split [get_cells -hierarchical -filter "REF_NAME =~  *$cpu_name*" ] " "]
	#if {$inst_path == ""} {
#		puts "Warning: No Processor found"
#	} else {
#		set proc_found [llength $inst_path]
#	}
		
	puts $fileout "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
	puts $fileout "<MemInfo Version=\"1\" Minor=\"0\">"

	# create for each cpu an address space	
	#for {set p 0} {$p < $proc_found} {incr p} {

		#set pattern [format ".*\\\[$p\\\].*$mem_name.*|^$mem_name.*"]

		#set pattern [format ".*\\\[t\\\].*$mem_name.*|^$mem_name.*"]
		#set pattern [format ".*\\\.*$mem_name.*|^$mem_name.*"]
		set cell_name_bram [split [get_cells -hierarchical -filter { PRIMITIVE_TYPE =~ BMEM.bram.* } -regexp $pattern] " "]
		#set cell_name_bram [split [get_cells -hierarchical -filter { PRIMITIVE_TYPE =~ BMEM.bram.* }] " "]
		
		puts "Cell is $cell_name_bram"
		
		set bram_range 0
		
		for {set i 0} {$i < [llength $cell_name_bram]} {incr i} {
			set bram_type [get_property REF_NAME [get_cells [lindex $cell_name_bram $i]]]
			if {$bram_type == "RAMB36E1"} {
				set bram_range [expr {$bram_range + 4096}]	
			} elseif {$bram_type == "RAMB18E1"} {
				set bram_range [expr {$bram_range + 2048}]	
			}

		}
		
		puts $fileout "  <Processor Endianness=\"$endianess\" InstPath=\"[format %s $cpu_name]\">"
		puts $fileout "  <AddressSpace Name=\"$mem_name\" Begin=\"[expr $offset] \" End=\"[expr {$offset + $bram_range - 1}]\">"
		puts $fileout "      <BusBlock>"


		set bram_cnt [llength $cell_name_bram]
		
		puts "BRam count $bram_cnt"
		
		set bram_size [expr {32768 / (32 / $bram_cnt)}]
		
		puts "BRAM size $bram_size"

		#updatedmem requires these sequences for the msb
		#totally stupid, because the MSB & LSB are not considered
		set bram [llength $cell_name_bram]
		
		if {$bram == 32} {
			if {[string compare -nocase $endianess "little"]==0} {
				set sequence "7,6,5,4,3,2,1,0,15,14,13,12,11,10,9,8,23,22,21,20,19,18,17,16,31,30,29,28,27,26,25,24"
			} else {
				set sequence "31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0"
			}
		} elseif {$bram >= 16 && $bram < 32} {
			if {[string compare -nocase $endianess "little"]==0} {
				set sequence "7,5,3,1,15,13,11,9,23,21,19,17,31,29,27,25"
			} else {
				set sequence "31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1"
			}
		}  elseif {$bram >= 8 && $bram < 16} {
			if {[string compare -nocase $endianess "little"]==0} {
				set sequence "7,3,15,11,23,19,31,27"
			} else {
				set sequence "31,27,23,19,15,11,7,3"
			}		
		} elseif {$bram >= 4 && $bram < 8} {
			if {[string compare -nocase $endianess "little"]==0} {
				set sequence "7,15,23,31"
			} else {
				set sequence "31,23,15,7"
			}
		} else {
			return -error "Error: Size is not supported"
		}
		set sequence [split $sequence ","]
		
		puts "Sequence $sequence"
			
		for {set i 0} {$i < [llength $sequence]} {incr i} {
			for {set j 0} {$j < $bram_cnt} {incr j} {
				
				puts [lindex $cell_name_bram $j]
				
				set byte_num 0
				set pattern ".*?mem_reg_(\\d)"
				regexp $pattern [lindex $cell_name_bram $j] match bit_block
				
				puts $bit_block
				
				set bmm_lsb			[expr {8 * $byte_num + 32 / $bram_cnt * $bit_block}]
				set bmm_msb			[expr {8 * $byte_num + 32 / $bram_cnt * $bit_block + 32 / $bram_cnt-1}]
				set bmm_range_begin	0
				set bmm_range_end	[expr {$bram_size-1}]
				set bram_type		[get_property REF_NAME [get_cells [lindex $cell_name_bram $j]]]			


				if {[lindex $sequence $i] == $bmm_msb} {
					if {$bram_type == "RAMB36E1"} {
						set bram_type "RAMB32"
					} elseif {$bram_type == "RAMB18E1"} {
						set bram_type "RAMB16"
					}

					set status [get_property STATUS [get_cells [lindex $cell_name_bram $j]]]																		
					if {$status == "UNPLACED"} {
						set placed "X0Y0"
					} else {
						set placed [get_property LOC [get_cells [lindex $cell_name_bram $j]]]
						set placed_list [split $placed "_"]
						set placed [lindex $placed_list 1]
					}
														

					puts $fileout "        <BitLane MemType=\"$bram_type\" Placement=\"$placed\">"
					puts $fileout "          <DataWidth MSB=\"$bmm_msb\" LSB=\"$bmm_lsb\"/>"
					puts $fileout "          <AddressRange Begin=\"$bmm_range_begin\" End=\"$bmm_range_end\"/>"
					puts $fileout "          <Parity ON=\"false\" NumBits=\"0\"/>"
					puts $fileout "        </BitLane>"
				}
			}
		}
		puts $fileout "      </BusBlock>"

		puts $fileout "    </AddressSpace>"
		puts $fileout "  </Processor>"
	#}
	puts $fileout "<Config>"
	puts $fileout "  <Option Name=\"Part\" Val=\"$part\"/>"
  	puts $fileout "</Config>"
  	puts $fileout "</MemInfo>"
	close $fileout
	puts "MMI file ($filename) created successfully."
	puts "To run Updatemem, use the command line below after write_bitstream:"
	puts "updatemem -force --meminfo $filename --data <data_file>.elf/mem --bit <bit_file>.bit --proc [format %s%s $cpu_name _x] --out <output_bit_file>.bit"
}

