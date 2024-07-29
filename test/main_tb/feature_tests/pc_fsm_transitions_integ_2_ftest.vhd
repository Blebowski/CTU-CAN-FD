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
--  Corner-case PC FSM transitions 2
--
-- @Verifies:
--  @1. Corner-case transitions of Protocol Control FSM to Integrating from
--      ACK Delimiter.
--
-- @Test sequence:
--  @1. Set DUT to Restricted operation mode.
--  @2. Iterate over both CAN 2.0 frame and CAN FD frame:
--      @2.1 Send CAN frame by Test Node. Wait until ACK in DUT node.
--           Force bus to Recessive. Wait until sample point. Release bus level.
--      @2.2 DUT should detect bit error in ACK bit. Due to pipelined
--           processing of error request, the error will become valid in
--           s_pc_ack_delim or s_pc_ack_fd_2. Wait until DUT integrates to the bus.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    29.7.2024   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package pc_fsm_transitions_integ_2_ftest is
    procedure pc_fsm_transitions_integ_2_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body pc_fsm_transitions_integ_2_ftest is

    procedure pc_fsm_transitions_integ_2_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable mode               :       SW_mode := SW_mode_rst_val;
        variable CAN_TX_frame       :       SW_CAN_frame_type;
        variable status             :       SW_status;
        variable err_capt           :       SW_error_capture;

    begin

        -------------------------------------------------------------------------------------------
        -- @1. Set DUT to Restricted operation mode.
        -------------------------------------------------------------------------------------------
        info_m("Step 1: Set DUT to Restricted operation mode");

        mode.restricted_operation := true;
        set_core_mode(mode, DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @2. Iterate over both CAN 2.0 frame and CAN FD frame:
        -------------------------------------------------------------------------------------------
        info_m("Step 2: Send CAN 2.0 and CAN FD frame");

        for frame_format in NORMAL_CAN to FD_CAN loop
            ---------------------------------------------------------------------------------------
            -- @2.1 Send CAN frame by Test Node. Wait until ACK in DUT node.
            --      Force bus to Recessive. Wait until sample point. Release bus level.
            ---------------------------------------------------------------------------------------
            info_m("Step 2.1");

            CAN_generate_frame(CAN_TX_frame);
            CAN_TX_frame.frame_format := frame_format;

            CAN_insert_TX_frame(CAN_TX_frame, 1, TEST_NODE, chn);
            send_TXT_buf_cmd(buf_set_ready, 1, TEST_NODE, chn);

            CAN_wait_pc_state(pc_deb_ack, DUT_NODE, chn);
            wait for 20 ns;
            force_bus_level(RECESSIVE, chn);
            CAN_wait_sample_point(DUT_NODE, chn);
            release_bus_level(chn);
            wait for 100 ns;

            ---------------------------------------------------------------------------------------
            -- @2.2 DUT should detect bit error in ACK bit. Due to pipelined processing of error
            --      request, the error will become valid in s_pc_ack_delim or s_pc_ack_fd_2.
            --      Wait until DUT integrates to the bus.
            ---------------------------------------------------------------------------------------
            info_m("Step 2.2");

            CAN_read_error_code_capture(err_capt, DUT_NODE, chn);
            check_m(err_capt.err_type = can_err_bit, "Bit Error");
            check_m(err_capt.err_pos = err_pos_ack, "Error in CRC Delim, ACK or ACK delim");

            wait for 10000 ns;

        end loop;

  end procedure;

end package body;
