--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core 
-- Copyright (C) 2021-present Ondrej Ille
-- 
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to use, copy, modify, merge, publish, distribute the Component for
-- educational, research, evaluation, self-interest purposes. Using the
-- Component for commercial purposes is forbidden unless previously agreed with
-- Copyright holder.
-- 
-- The above copyright notice and this permission notice shall be included in
-- all copies or substantial portions of the Component.
-- 
-- THE COMPONENT IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
-- FROM, OUT OF OR IN CONNECTION WITH THE COMPONENT OR THE USE OR OTHER DEALINGS
-- IN THE COMPONENT.
-- 
-- The CAN protocol is developed by Robert Bosch GmbH and protected by patents.
-- Anybody who wants to implement this IP core on silicon has to obtain a CAN
-- protocol license from Bosch.
-- 
-- -------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core 
-- Copyright (C) 2015-2020 MIT License
-- 
-- Authors:
--     Ondrej Ille <ondrej.ille@gmail.com>
--     Martin Jerabek <martin.jerabek01@gmail.com>
-- 
-- Project advisors: 
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 
-- Department of Measurement         (http://meas.fel.cvut.cz/)
-- Faculty of Electrical Engineering (http://www.fel.cvut.cz)
-- Czech Technical University        (http://www.cvut.cz/)
-- 
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to deal in the Component without restriction, including without limitation
-- the rights to use, copy, modify, merge, publish, distribute, sublicense,
-- and/or sell copies of the Component, and to permit persons to whom the
-- Component is furnished to do so, subject to the following conditions:
-- 
-- The above copyright notice and this permission notice shall be included in
-- all copies or substantial portions of the Component.
-- 
-- THE COMPONENT IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
-- FROM, OUT OF OR IN CONNECTION WITH THE COMPONENT OR THE USE OR OTHER DEALINGS
-- IN THE COMPONENT.
-- 
-- The CAN protocol is developed by Robert Bosch GmbH and protected by patents.
-- Anybody who wants to implement this IP core on silicon has to obtain a CAN
-- protocol license from Bosch.
-- 
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
--  Purpose:
--    Lists all feature tests and provides feature test exec function.
--
--    Replacement for previous automated solution with Python. With manual
--    solution, we have to list each and every feature test here, however,
--    we dont need to include any third-party modules. It was decided to stick
--    with this solution as it minimizes dependencies of the TB!
--
--    If VHDL had some sort of intro-spection or reflection, it would be
--    possible to write this code much more nicely!
--
--------------------------------------------------------------------------------
-- Revision History:
--    12.3.2021     Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;

-- Feature test packages
use ctu_can_fd_tb.alc_base_id_ftest.all;
use ctu_can_fd_tb.alc_ide_ftest.all;
use ctu_can_fd_tb.alc_id_extension_ftest.all;
use ctu_can_fd_tb.alc_rtr_ext_id_ftest.all;
use ctu_can_fd_tb.alc_rtr_r0_ftest.all;
use ctu_can_fd_tb.alc_srr_rtr_ftest.all;
use ctu_can_fd_tb.alc_srr_rtr_2_ftest.all;

use ctu_can_fd_tb.device_id_ftest.all;
use ctu_can_fd_tb.dlc_can20_8_64_bytes_ftest.all;

use ctu_can_fd_tb.err_capt_ack_ack_ftest.all;
use ctu_can_fd_tb.err_capt_arb_bit_ftest.all;
use ctu_can_fd_tb.err_capt_arb_stuff_ftest.all;
use ctu_can_fd_tb.err_capt_crc_bit_ftest.all;
use ctu_can_fd_tb.err_capt_crc_err_ftest.all;
use ctu_can_fd_tb.err_capt_ctrl_bit_ftest.all;
use ctu_can_fd_tb.err_capt_ctrl_form_ftest.all;
use ctu_can_fd_tb.err_capt_data_bit_ftest.all;
use ctu_can_fd_tb.err_capt_eof_ftest.all;
use ctu_can_fd_tb.err_capt_err_frm_ftest.all;
use ctu_can_fd_tb.err_capt_ovr_frm_ftest.all;
use ctu_can_fd_tb.err_capt_sof_ftest.all;

use ctu_can_fd_tb.fault_state_ftest.all;
use ctu_can_fd_tb.glitch_filtering_ftest.all;

use ctu_can_fd_tb.mode_bus_monitoring_ftest.all;
use ctu_can_fd_tb.mode_fd_enable_ftest.all;
use ctu_can_fd_tb.mode_loopback_ftest.all;
use ctu_can_fd_tb.mode_fdrf_ftest.all;
use ctu_can_fd_tb.mode_pex_ftest.all;
use ctu_can_fd_tb.mode_restr_op_ftest.all;
use ctu_can_fd_tb.mode_test_ftest.all;
use ctu_can_fd_tb.mode_self_test_ftest.all;
use ctu_can_fd_tb.mode_frame_filters_ftest.all;

use ctu_can_fd_tb.one_shot_ftest.all;
use ctu_can_fd_tb.rec_saturation_ftest.all;
use ctu_can_fd_tb.rx_counter_ftest.all;

use ctu_can_fd_tb.status_eft_ftest.all;
use ctu_can_fd_tb.status_ewl_ftest.all;
use ctu_can_fd_tb.status_idle_ftest.all;
use ctu_can_fd_tb.status_rxne_ftest.all;
use ctu_can_fd_tb.status_rxs_ftest.all;
use ctu_can_fd_tb.status_txnf_ftest.all;
use ctu_can_fd_tb.status_txs_ftest.all;

use ctu_can_fd_tb.tx_cmd_set_abort_ftest.all;
use ctu_can_fd_tb.tx_cmd_set_empty_ftest.all;
use ctu_can_fd_tb.tx_cmd_set_ready_ftest.all;
use ctu_can_fd_tb.tx_counter_ftest.all;
use ctu_can_fd_tb.tx_from_intermission_ftest.all;
use ctu_can_fd_tb.tx_priority_change_ftest.all;
use ctu_can_fd_tb.tx_priority_ftest.all;
use ctu_can_fd_tb.tx_status_ftest.all;
use ctu_can_fd_tb.timestamp_low_high_ftest.all;
use ctu_can_fd_tb.txt_buffer_hazard_ftest.all;


package feature_test_list_pkg is
    
    procedure exec_feature_test(
        constant test_name    : in     string;
        signal   channel      : inout  t_com_channel
    );

end package;


package body feature_test_list_pkg is

    procedure exec_feature_test(
        constant test_name    : in     string;
        signal   channel      : inout  t_com_channel
    ) is
    begin
        if (test_name = "alc_base_id") then
            alc_base_id_ftest_exec(channel);
        elsif (test_name = "alc_ide") then
            alc_ide_ftest_exec(channel);
        elsif (test_name = "alc_id_extension") then
            alc_id_extension_ftest_exec(channel);
        elsif (test_name = "alc_rtr_ext_id") then
            alc_rtr_ext_id_ftest_exec(channel);
        elsif (test_name = "alc_rtr_r0") then
            alc_rtr_r0_ftest_exec(channel);
        elsif (test_name = "alc_srr_rtr") then
            alc_srr_rtr_ftest_exec(channel);
        elsif (test_name = "alc_srr_rtr_2") then
            alc_srr_rtr_2_ftest_exec(channel);
             
        elsif (test_name = "device_id") then
            device_id_ftest_exec(channel);
        elsif (test_name = "dlc_can20_8_64_bytes") then
            dlc_can20_8_64_bytes_ftest_exec(channel);
        elsif (test_name = "glitch_filtering") then
            glitch_filtering_ftest_exec(channel);

        elsif (test_name = "err_capt_ack_ack") then
            err_capt_ack_ack_ftest_exec(channel);            
        elsif (test_name = "err_capt_arb_bit") then
            err_capt_arb_bit_ftest_exec(channel);
        elsif (test_name = "err_capt_arb_stuff") then
            err_capt_arb_stuff_ftest_exec(channel);
        elsif (test_name = "err_capt_crc_bit") then
            err_capt_crc_bit_ftest_exec(channel);
        elsif (test_name = "err_capt_crc_err") then
            err_capt_crc_err_ftest_exec(channel);
        elsif (test_name = "err_capt_ctrl_bit") then
            err_capt_ctrl_bit_ftest_exec(channel);
        elsif (test_name = "err_capt_ctrl_form") then
            err_capt_ctrl_form_ftest_exec(channel);
        elsif (test_name = "err_capt_data_bit") then
            err_capt_data_bit_ftest_exec(channel);
        elsif (test_name = "err_capt_eof") then
            err_capt_eof_ftest_exec(channel);
        elsif (test_name = "err_capt_err_frm") then
            err_capt_err_frm_ftest_exec(channel);
        elsif (test_name = "err_capt_ovr_frm") then
            err_capt_ovr_frm_ftest_exec(channel);
        elsif (test_name = "err_capt_sof") then
            err_capt_sof_ftest_exec(channel);
                                                      
        elsif (test_name = "fault_state") then
            fault_state_ftest_exec(channel);
                    
        elsif (test_name = "mode_bus_monitoring") then
            mode_bus_monitoring_ftest_exec(channel);
        elsif (test_name = "mode_fd_enable") then
            mode_fd_enable_ftest_exec(channel);
        elsif (test_name = "mode_loopback") then
            mode_loopback_ftest_exec(channel);
        elsif (test_name = "mode_fdrf") then
            mode_fdrf_ftest_exec(channel);
        elsif (test_name = "mode_pex") then
            mode_pex_ftest_exec(channel);
        elsif (test_name = "mode_restr_op") then
            mode_restr_op_ftest_exec(channel);
        elsif (test_name = "mode_test") then
            mode_test_ftest_exec(channel);
        elsif (test_name = "mode_self_test") then
            mode_self_test_ftest_exec(channel);
        elsif (test_name = "mode_frame_filters") then
            mode_frame_filters_ftest_exec(channel);

        elsif (test_name = "one_shot") then
            one_shot_ftest_exec(channel);
        elsif (test_name = "rec_saturation") then
            rec_saturation_ftest_exec(channel);
        elsif (test_name = "rx_counter") then
            rx_counter_ftest_exec(channel);
        elsif (test_name = "status_eft") then
            status_eft_ftest_exec(channel);
        elsif (test_name = "status_ewl") then
            status_ewl_ftest_exec(channel);
        elsif (test_name = "status_idle") then
            status_idle_ftest_exec(channel);
        elsif (test_name = "status_rxne") then
            status_rxne_ftest_exec(channel);
        elsif (test_name = "status_rxs") then
            status_rxs_ftest_exec(channel);
        elsif (test_name = "status_txnf") then
            status_txnf_ftest_exec(channel);
        elsif (test_name = "status_txs") then
            status_txs_ftest_exec(channel);

        elsif (test_name = "tx_cmd_set_abort") then
            tx_cmd_set_abort_ftest_exec(channel);
        elsif (test_name = "tx_cmd_set_empty") then
            tx_cmd_set_empty_ftest_exec(channel);
        elsif (test_name = "tx_cmd_set_ready") then
            tx_cmd_set_ready_ftest_exec(channel);
        elsif (test_name = "tx_counter") then
            tx_counter_ftest_exec(channel);
        elsif (test_name = "tx_from_intermission") then
            tx_from_intermission_ftest_exec(channel);
        elsif (test_name = "tx_priority_change") then
            tx_priority_change_ftest_exec(channel);
        elsif (test_name = "tx_priority") then
            tx_priority_ftest_exec(channel);
        elsif (test_name = "tx_status") then
            tx_status_ftest_exec(channel);
        elsif (test_name = "timestamp_low_high") then
            timestamp_low_high_ftest_exec(channel);
        elsif (test_name = "txt_buffer_hazard") then
            txt_buffer_hazard_ftest_exec(channel);
            
        else
            error_m("TODO: Implement calling feature test function based on test name!!");
        end if;
    end procedure;

end package body;
