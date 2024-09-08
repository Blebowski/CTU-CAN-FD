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
-- @TestInfoStart
--
-- @Purpose:
--  RX Error logging feature test 8
--
-- @Verifies:
--  @1. When CTU CAN FD is error passive, FRAME_FORMAT_W[ERF_ERP] = 1.
--  @2. When CTU CAN FD is error active, FRAME_FORMAT_W[ERF_ERP] = 1.
--
-- @Test sequence:
--  @1. Configure Test Node to ACK forbidden mode. Configure DUT to
--      MODE[ERFM] = 1 and to Test Mode.
--  @2. Iterate over scenarios: Error Active, Error Passive.
--      @2.1 Set DUT to either Error Active or Error Passive.
--      @2.2 Generate CAN frame and send it byt DUT Node. Wait until bus is
--           idle.
--      @2.3 Check DUT node has a single frame in RX Buffer. Read this frame,
--           and check this is an error frame. Check FRAME_FORMAT_W[ERF_ERP] = 1
--           when DUT was error passive. Check FRAME_FORMAT_W[ERF_ERP] = 0
--           when DUT was error active.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    8.9.2024   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package rx_err_log_8_ftest is
    procedure rx_err_log_8_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body rx_err_log_8_ftest is

    procedure rx_err_log_8_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable mode_1             : SW_mode := SW_mode_rst_val;
        variable mode_2             : SW_mode := SW_mode_rst_val;

        variable CAN_frame          : SW_CAN_frame_type;
        variable err_frame          : SW_CAN_frame_type;

        variable frames_match       : boolean;
        variable frame_sent         : boolean;

        variable rx_buf_info        : SW_RX_Buffer_info;

        variable err_counters       : SW_error_counters;

    begin

        -------------------------------------------------------------------------------------------
        -- @1. Configure Test Node to ACK forbidden mode. Configure DUT to
        --     MODE[ERFM] = 1 and to Test Mode.
        -------------------------------------------------------------------------------------------
        info_m("Step 1");

        mode_1.error_logging := true;
        mode_1.test := true;
        set_core_mode(mode_1, DUT_NODE, chn);

        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, TEST_NODE, chn);

        -------------------------------------------------------------------------------------------
        --  @2. Iterate over scenarios: Error Active, Error Passive.
        -------------------------------------------------------------------------------------------
        info_m("Step 2");

        for i in 0 to 5 loop
        for fault_state in fc_error_active to fc_error_passive loop

            ---------------------------------------------------------------------------------------
            -- @2.1 Set DUT to either Error Active or Error Passive.
            ---------------------------------------------------------------------------------------
            info_m("Step 2.1");

            if (fault_state = fc_error_passive) then
                err_counters.rx_counter := 150;
                set_error_counters(err_counters, DUT_NODE, chn);
            else
                err_counters.rx_counter := 10;
                set_error_counters(err_counters, DUT_NODE, chn);
            end if;

            ---------------------------------------------------------------------------------------
            -- @2.2 Generate CAN frame and send it byt DUT Node. Wait until bus is idle.
            ---------------------------------------------------------------------------------------
            info_m("Step 2.2");

            CAN_generate_frame(CAN_frame);
            CAN_send_frame(CAN_frame, 1, DUT_NODE, chn, frame_sent);
            CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);
            CAN_wait_bus_idle(DUT_NODE, chn);

            ---------------------------------------------------------------------------------------
            --  @2.3 Check DUT node has a single frame in RX Buffer. Read this frame,
            --       and check this is an error frame. Check FRAME_FORMAT_W[ERF_ERP] = 1
            --       when DUT was error passive. Check FRAME_FORMAT_W[ERF_ERP] = 0
            --       when DUT was error active.
            ---------------------------------------------------------------------------------------
            info_m("Step 2.3");

            get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
            check_m(rx_buf_info.rx_frame_count = 1, "Single Error frame in RX Buffer!");

            CAN_read_frame(err_frame, DUT_NODE, chn);
            check_m(err_frame.erf = '1', "FRAME_FORMAT_W[ERF] = 1");

            if (fault_state = fc_error_passive) then
                check_m(err_frame.erf_erp = '1', "FRAME_FORMAT_W[ERF_TYPE] = 1");
            else
                check_m(err_frame.erf_erp = '0', "FRAME_FORMAT_W[ERF_TYPE] = 0");
            end if;

        end loop;
        end loop;

    end procedure;

end package body;
