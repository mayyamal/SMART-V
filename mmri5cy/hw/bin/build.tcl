set outputDir ./build
file mkdir $outputDir

if {$argc != 5} {
  puts "ERROR: missing parameter"
  exit
}

set vsrc [lindex $argv 0]
set vinc [lindex $argv 1]
set board [lindex $argv 2]
set part [lindex $argv 3]
set memfile [lindex $argv 4]

puts "source files:"
puts $vsrc

puts "includes dirs:"
puts $vinc
set vinc_param [set lst {}; foreach i $vinc {lappend lst "-include_dirs $i"}; join $lst]

source -notrace ../bin/write_mmi.tcl


read_verilog -sv $vsrc
read_xdc ../board_$board.xdc

synth_design -top top -part $part -verilog_define SYNTHESIS=1 -generic ROM_FILE=$memfile -include_dirs $vinc -flatten_hierarchy none -keep_equivalent_registers
write_checkpoint -force $outputDir/post_synth
report_timing_summary -file $outputDir/post_synth_timing_summary.rpt

#opt_design
place_design
phys_opt_design
write_checkpoint -force $outputDir/post_place
report_timing_summary -file $outputDir/post_place_timing_summary.rpt

route_design
write_checkpoint -force $outputDir/post_route
report_timing_summary -file $outputDir/post_route_timing_summary.rpt
report_timing -sort_by group -max_paths 100 -path_type summary -file $outputDir/post_route_timing.rpt
report_clock_utilization -file $outputDir/clock_util.rpt
report_utilization -file $outputDir/post_route_util.rpt
report_utilization -hierarchical -file $outputDir/post_route_util_hierarch.rpt


write_bitstream -force -bin $outputDir/fpga_$board.bit

#write_mmi rom top Little $part 0x00000000 $outputDir 


quit
