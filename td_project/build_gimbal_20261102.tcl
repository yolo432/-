import_device ph1_35p.db -package PH1P35MDG324 -speed 3
open_project camera_to_dsi_display.al
elaborate -top design_top_wrapper
read_adc ../user_source/constraints_source/pin.adc
read_ip_adc -ip ddr2 -file ../user_source/hdl_source/ph1p35_ddr/ddr2/src/adc/ddr2.adc
read_sdc camera_to_dsi_display.sdc
read_sdc -ip ddr2 ../user_source/hdl_source/ph1p35_ddr/ddr2/src/sdc/ddr2.sdc
read_sdc -ip w128_d512_fifo ../user_source/ip_source/w128_d512_fifo/w128_d512_fifo.tcl
optimize_rtl
write_verilog camera_gimbal_20261102.rtl.v
set rtl_file [open camera_gimbal_20261102.rtl.v r]
set rtl_text [read $rtl_file]
close $rtl_file
foreach output {O_buzzer O_gimbal_pan O_gimbal_tilt} {
    if {[regexp "assign\\s+$output\\s*=\\s*1'b\[01\]" $rtl_text]} {
        error "$output is constant in the optimized RTL"
    }
}
unset rtl_text
optimize_gate -packarea camera_gimbal_20261102.area
legalize_phy_inst
place
route
update_timing -mode final
report_timing_summary -file camera_gimbal_20261102.timing
report_timing_status -file camera_gimbal_20261102.ts
report_qor -step route -file camera_gimbal_20261102.route.qor
bitgen -bit camera_eth_gimbal_20261102.bit
exit
