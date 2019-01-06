################################################################################
## 
## CAN with Flexible Data-Rate IP Core 
## 
## Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
## 
## Project advisor: Jiri Novak <jnovak@fel.cvut.cz>
## Department of Measurement         (http://meas.fel.cvut.cz/)
## Faculty of Electrical Engineering (http://www.fel.cvut.cz)
## Czech Technical University        (http://www.cvut.cz/)
## 
## Permission is hereby granted, free of charge, to any person obtaining a copy 
## of this VHDL component and associated documentation files (the "Component"), 
## to deal in the Component without restriction, including without limitation 
## the rights to use, copy, modify, merge, publish, distribute, sublicense, 
## and/or sell copies of the Component, and to permit persons to whom the 
## Component is furnished to do so, subject to the following conditions:
## 
## The above copyright notice and this permission notice shall be included in 
## all copies or substantial portions of the Component.
## 
## THE COMPONENT IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
## AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
## FROM, OUT OF OR IN CONNECTION WITH THE COMPONENT OR THE USE OR OTHER DEALINGS 
## IN THE COMPONENT.
## 
## The CAN protocol is developed by Robert Bosch GmbH and protected by patents. 
## Anybody who wants to implement this IP core on silicon has to obtain a CAN 
## protocol license from Bosch.
## 
################################################################################

################################################################################
## Description:
##   File with waves definition for all memory registers to simplify debugging.
##   Core must have configured Full Frame filter support and Event Logger.
################################################################################


################################################################################
# Path to test component
################################################################################
global TCOMP


################################################################################
# Local path to Core instance. For now hard-coded will be moved to global path
# later on. At the moment usable for feature test.
################################################################################
set INST_PATH g_inst__1/CAN_inst


################################################################################
# Adds waves of writable register
################################################################################
proc add_w_reg_waves {path name} {
    add wave -group $name \
        -label "Data width" $path/data_width \
        -label "Data mask" $path/data_mask \
        -label "Reset polarity" $path/reset_polarity \
        -label "Reset value" $path/reset_value \
        -label "Auto clear" $path/auto_clear \
        -label "Clock" $path/clk_sys \
        -label "Input Data" $path/data_in \
        -label "Write" $path/write \
        -label "Chip-select" $path/cs \
        -label "Byte enable" $path/w_be \
        -label "Register value" $path/reg_value
}



################################################################################
################################################################################
# Adding Control registers waves
################################################################################
################################################################################

# Configure path to generated control regisster
set RM_PATH $TCOMP/$INST_PATH/memory_registers_comp/control_registers_reg_map_comp

# Add Divider
add wave -noupdate -divider -height 30 "Control registers - writable"


################################################################################
# Add writable register ports.
################################################################################

add_w_reg_waves $RM_PATH/mode_reg_comp "MODE"
add_w_reg_waves $RM_PATH/command_reg_comp "COMMAND"
add_w_reg_waves $RM_PATH/settings_reg_comp "SETTINGS"

add_w_reg_waves $RM_PATH/int_stat_reg_comp "INT_STAT"
add_w_reg_waves $RM_PATH/int_ena_set_reg_comp "INT_ENA_SET"
add_w_reg_waves $RM_PATH/int_ena_clr_reg_comp "INT_ENA_CLEAR"
add_w_reg_waves $RM_PATH/int_mask_set_reg_comp "INT_MASK_SET"
add_w_reg_waves $RM_PATH/int_mask_clr_reg_comp "INT_MASK_CLEAR"

add_w_reg_waves $RM_PATH/btr_reg_comp "BTR"
add_w_reg_waves $RM_PATH/btr_fd_reg_comp "BTR_FD"

add_w_reg_waves $RM_PATH/ewl_reg_comp "EWL"
add_w_reg_waves $RM_PATH/erp_reg_comp "ERP"
add_w_reg_waves $RM_PATH/ctr_pres_reg_comp "CTR_PRES"

add_w_reg_waves $RM_PATH/FILTER_A_MASK_present_gen_t/filter_a_mask_reg_comp "FILTER_A_MASK"
add_w_reg_waves $RM_PATH/FILTER_A_VAL_present_gen_t/filter_a_val_reg_comp "FILTER_A_VAL"

add_w_reg_waves $RM_PATH/FILTER_B_MASK_present_gen_t/filter_b_mask_reg_comp "FILTER_B_MASK"
add_w_reg_waves $RM_PATH/FILTER_B_VAL_present_gen_t/filter_b_val_reg_comp "FILTER_B_VAL"

add_w_reg_waves $RM_PATH/FILTER_C_MASK_present_gen_t/filter_c_mask_reg_comp "FILTER_C_MASK"
add_w_reg_waves $RM_PATH/FILTER_C_VAL_present_gen_t/filter_c_val_reg_comp "FILTER_C_VAL"

add_w_reg_waves $RM_PATH/FILTER_RAN_LOW_present_gen_t/filter_ran_low_reg_comp "FILTER_RAN_LOW"
add_w_reg_waves $RM_PATH/FILTER_RAN_HIGH_present_gen_t/filter_ran_high_reg_comp "FILTER_RAN_HIGH"


add_w_reg_waves $RM_PATH/filter_control_reg_comp "FILTER_CONTROL"
add_w_reg_waves $RM_PATH/rx_settings_reg_comp "RX_SETTINGS"

add_w_reg_waves $RM_PATH/tx_command_reg_comp "TX_COMMAND"
add_w_reg_waves $RM_PATH/tx_priority_reg_comp "TX_PRIORITY"

add_w_reg_waves $RM_PATH/ssp_cfg_reg_comp "SSP_CFG"


################################################################################
# Add Readable register - Adding input record is enough here since there
# are NO components.
################################################################################

# Add Divider
add wave -noupdate -divider -height 30 "Control registers - readable"

add wave -label "Control Registers - Read record" -expand $RM_PATH/control_registers_in


################################################################################
################################################################################
# Adding Event Logger waves
################################################################################
################################################################################

# Configure path to generated control regisster
set RM_PATH $TCOMP/$INST_PATH/memory_registers_comp/log_pres_gen/event_logger_reg_map_comp

# Add Divider
add wave -noupdate -divider -height 30 "Event Logger registers - writable"

add_w_reg_waves $RM_PATH/log_trig_config_reg_comp "LOG_TRIG"
add_w_reg_waves $RM_PATH/log_capt_config_reg_comp "LOG_CAPT"
add_w_reg_waves $RM_PATH/log_command_reg_comp "LOG_COMMAND"

# Add Divider
add wave -noupdate -divider -height 30 "Event Logger registers - readable"

add wave -label "Event Loger Registers - Read record" -expand $RM_PATH/event_logger_in

