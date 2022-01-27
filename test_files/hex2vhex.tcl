#!/usr/bin/tclsh
#****************************************************************************
#		Copyright Notice and Disclaimer for the mosartMCU Project.
# Copyright (C) 2016-2018 Fabian Mauroner. All rights reserved.
# 
# This file was released for educational use within the 
#             Embedded Automotive Systems Group, 
#             Institute of Technical Informatics,
#           Graz University of Technology, Austria
# This copyright may not be removed, modified or relocated within this file.
#****************************************************************************


package require cmdline

#==============================================================================
#							  Arguments
#==============================================================================
set parameters {
    {input.arg   ""      "Input Hex file"}
    {output.arg  ""      "Output file for Verilog"}
	{offset.arg  0       "Memory Start address <hex>"}
	{width.arg   4      "Size of the memory <byte>"}
	{endianswap.arg 0			"swap the endian"}
}

set usage ": -input <input> -output <output> -offset <offset> -width <addr_width> \[-endianswao\]"
if {[catch {array set options [cmdline::getoptions ::argv $parameters $usage]}]} {
    puts [cmdline::usage $parameters $usage]
	exit 1
} else {
#    parray options
}

# Verify required parameters
set requiredParameters {input output}
foreach parameter $requiredParameters {
    if {$options($parameter) == ""} {
        puts stderr "Missing required parameter: -$parameter"
        exit 1
    }
}


#==============================================================================
#							Parsing	
#==============================================================================
set rfile             [open $options(input) r]
set wfile		      [open $options(output) w]
set address_old       $options(offset)
set addr_width_byte   $options(width)
set wordswap		  $options(endianswap)


# convert hex string to integer
proc hex2dec { val } {
	set val [format "%u" 0x[string trimleft $val]]
	return $val
}

# check line beginning and checksum
proc checkLine { line } {

	if {[string index $line 0] != ":"} {
		return false
	}	

	set sum 0	
	set line_lenght [hex2dec [string range $line 1 2]]
	# sum + checksum has to be zero
	for {set i 0} {$i <2*$line_lenght+2} {incr i 2} {
		set sum [expr $sum + [hex2dec [string range $line [expr $i+9] [expr $i+9+3]]]]
	}

	return $sum == 0
}

#------------------------------------------------------------------------------
#							Parse hex file
#------------------------------------------------------------------------------
set extended_linear_address 0
set extended_segment_address 0

while {[gets $rfile record] >= 0} {


	if {[checkLine $record] == false} {
		puts stderr [concat "ERROR in line:" $record]
		break
	}

	set line_lenght [hex2dec [string range $record 1 2]]
	set line_addr   [expr [hex2dec [string range $record 3 4]] << 8 | [hex2dec [string range $record 5 6]]]
	set line_type   [string range $record 7 8]


	switch -exact -- $line_type {
		COMMENT {#Data record}
		00 { 	
			set add [expr [hex2dec [string range $record 3 4]] << 8 | [hex2dec [string range $record 5 6]]]
			set add [expr $add | $extended_segment_address | $extended_linear_address]

			#fill up
			for {set i 0} {$i < [expr {$add-$address_old}] } {incr i $addr_width_byte} {
				set tmp_write "00"
				for {set j 1} {$j < $addr_width_byte} {incr j} {
					append tmp_write "00"
				}
				puts $wfile $tmp_write
			}

			#copy data
			for {set i 0} {$i < 2*$line_lenght} {incr i [expr {2*$addr_width_byte}]} {
				set code ""

				for {set j 0} {$j < [expr 2*$addr_width_byte]} {incr j 2} {

					set byte [string range $record [expr $i+$j+9] [expr {$i+$j+10}]]
					if {$wordswap} {
						set code $byte$code
					} else {
						set code $code$byte
					}
				}
				puts $wfile $code
			}

			set address_old [expr {$add + $line_lenght}]
		}

		COMMENT {# Extended segment address record}
		02 { 
			set data [expr [hex2dec [string range $record 9 10]] << 8 | [hex2dec [string range $record 11 12]]]
			set extended_segment_address [expr $data << 4]
		}
		COMMENT {# Extended linear address record}
		04 { 
			set data [expr [hex2dec [string range $record 9 10]] << 8 | [hex2dec [string range $record 11 12]]]
			set extended_linear_address [expr $data << 16]
		}
		COMMENT {# End of file record}
		01 { }
		COMMENT {# start segment address record (x86 CS:IP registers)}
		03 { }
		COMMENT {# Start Linear Address Record (x86 EIP register)}
		05 { }
		NONE -
		default {puts {wrong file}}
	}
	#puts $record
}


close $rfile
close $wfile
exit 0



