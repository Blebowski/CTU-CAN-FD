


##########################################################
#Adds the basic waveforms which are common for all tests
##########################################################
proc add_common_waves {} {
	add wave -noupdate -divider -height 20 "Test details"
	add wave status
	add wave run
	add wave errors
	add wave test_comp/iterations
	add wave test_comp/log_level
	add wave test_comp/error_beh
}
