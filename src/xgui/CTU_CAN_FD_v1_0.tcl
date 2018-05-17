# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "logger_size" -parent ${Page_0}
  ipgui::add_param $IPINST -name "rx_buffer_size" -parent ${Page_0}
  ipgui::add_param $IPINST -name "sup_be" -parent ${Page_0}
  ipgui::add_param $IPINST -name "sup_filtA" -parent ${Page_0}
  ipgui::add_param $IPINST -name "sup_filtB" -parent ${Page_0}
  ipgui::add_param $IPINST -name "sup_filtC" -parent ${Page_0}
  ipgui::add_param $IPINST -name "sup_range" -parent ${Page_0}
  ipgui::add_param $IPINST -name "tx_time_sup" -parent ${Page_0}
  ipgui::add_param $IPINST -name "use_logger" -parent ${Page_0}
  ipgui::add_param $IPINST -name "use_sync" -parent ${Page_0}


}

proc update_PARAM_VALUE.logger_size { PARAM_VALUE.logger_size } {
	# Procedure called to update logger_size when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.logger_size { PARAM_VALUE.logger_size } {
	# Procedure called to validate logger_size
	return true
}

proc update_PARAM_VALUE.rx_buffer_size { PARAM_VALUE.rx_buffer_size } {
	# Procedure called to update rx_buffer_size when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.rx_buffer_size { PARAM_VALUE.rx_buffer_size } {
	# Procedure called to validate rx_buffer_size
	return true
}

proc update_PARAM_VALUE.sup_be { PARAM_VALUE.sup_be } {
	# Procedure called to update sup_be when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.sup_be { PARAM_VALUE.sup_be } {
	# Procedure called to validate sup_be
	return true
}

proc update_PARAM_VALUE.sup_filtA { PARAM_VALUE.sup_filtA } {
	# Procedure called to update sup_filtA when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.sup_filtA { PARAM_VALUE.sup_filtA } {
	# Procedure called to validate sup_filtA
	return true
}

proc update_PARAM_VALUE.sup_filtB { PARAM_VALUE.sup_filtB } {
	# Procedure called to update sup_filtB when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.sup_filtB { PARAM_VALUE.sup_filtB } {
	# Procedure called to validate sup_filtB
	return true
}

proc update_PARAM_VALUE.sup_filtC { PARAM_VALUE.sup_filtC } {
	# Procedure called to update sup_filtC when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.sup_filtC { PARAM_VALUE.sup_filtC } {
	# Procedure called to validate sup_filtC
	return true
}

proc update_PARAM_VALUE.sup_range { PARAM_VALUE.sup_range } {
	# Procedure called to update sup_range when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.sup_range { PARAM_VALUE.sup_range } {
	# Procedure called to validate sup_range
	return true
}

proc update_PARAM_VALUE.tx_time_sup { PARAM_VALUE.tx_time_sup } {
	# Procedure called to update tx_time_sup when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.tx_time_sup { PARAM_VALUE.tx_time_sup } {
	# Procedure called to validate tx_time_sup
	return true
}

proc update_PARAM_VALUE.use_logger { PARAM_VALUE.use_logger } {
	# Procedure called to update use_logger when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.use_logger { PARAM_VALUE.use_logger } {
	# Procedure called to validate use_logger
	return true
}

proc update_PARAM_VALUE.use_sync { PARAM_VALUE.use_sync } {
	# Procedure called to update use_sync when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.use_sync { PARAM_VALUE.use_sync } {
	# Procedure called to validate use_sync
	return true
}


proc update_MODELPARAM_VALUE.use_logger { MODELPARAM_VALUE.use_logger PARAM_VALUE.use_logger } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.use_logger}] ${MODELPARAM_VALUE.use_logger}
}

proc update_MODELPARAM_VALUE.rx_buffer_size { MODELPARAM_VALUE.rx_buffer_size PARAM_VALUE.rx_buffer_size } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.rx_buffer_size}] ${MODELPARAM_VALUE.rx_buffer_size}
}

proc update_MODELPARAM_VALUE.use_sync { MODELPARAM_VALUE.use_sync PARAM_VALUE.use_sync } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.use_sync}] ${MODELPARAM_VALUE.use_sync}
}

proc update_MODELPARAM_VALUE.sup_filtA { MODELPARAM_VALUE.sup_filtA PARAM_VALUE.sup_filtA } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.sup_filtA}] ${MODELPARAM_VALUE.sup_filtA}
}

proc update_MODELPARAM_VALUE.sup_filtB { MODELPARAM_VALUE.sup_filtB PARAM_VALUE.sup_filtB } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.sup_filtB}] ${MODELPARAM_VALUE.sup_filtB}
}

proc update_MODELPARAM_VALUE.sup_filtC { MODELPARAM_VALUE.sup_filtC PARAM_VALUE.sup_filtC } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.sup_filtC}] ${MODELPARAM_VALUE.sup_filtC}
}

proc update_MODELPARAM_VALUE.sup_range { MODELPARAM_VALUE.sup_range PARAM_VALUE.sup_range } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.sup_range}] ${MODELPARAM_VALUE.sup_range}
}

proc update_MODELPARAM_VALUE.tx_time_sup { MODELPARAM_VALUE.tx_time_sup PARAM_VALUE.tx_time_sup } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.tx_time_sup}] ${MODELPARAM_VALUE.tx_time_sup}
}

proc update_MODELPARAM_VALUE.sup_be { MODELPARAM_VALUE.sup_be PARAM_VALUE.sup_be } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.sup_be}] ${MODELPARAM_VALUE.sup_be}
}

proc update_MODELPARAM_VALUE.logger_size { MODELPARAM_VALUE.logger_size PARAM_VALUE.logger_size } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.logger_size}] ${MODELPARAM_VALUE.logger_size}
}

