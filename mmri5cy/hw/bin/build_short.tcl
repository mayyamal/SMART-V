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

source -notrace ../bin/write_mmi.tcl


write_mmi rom top Little $part 0x00000000 $outputDir 


quit
