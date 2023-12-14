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
--  TX Arbitration consistency feature test 2.
--
-- @Verifies:
--  @1. That if during TXT Buffer validation process (during TX Arbitration)
--      a highest priority TXT Buffer changes, new TXT Buffer will be used
--      for transmission.
--
-- @Test sequence:
--  @1.1 Loop with incrementing wait time:
--      @1. Generate two random frames and insert them to TXT Buffer 1 and 2.
--          Configure priority of TXT Buffer 1 higher than TXT Buffer 2.
--      @2. Wait until sample point and issue Set ready command to TXT Buffer 2.
--      @3. Wait number of cycles given by iteration, and issue Set Ready Command
--          to TXT Buffer 1.
--      @4. Wait until frame is sent, and verify that frame from TXT Buffer 1
--          was succesfully sent (wait times are shorter than bit time!)
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--      29.11.2020   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.clk_gen_agent_pkg.all;

package tx_arb_consistency_2_ftest is
    procedure tx_arb_consistency_2_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body tx_arb_consistency_2_ftest is

    procedure tx_arb_consistency_2_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame_rx_1     :       SW_CAN_frame_type;
        variable CAN_frame_rx_2     :       SW_CAN_frame_type;
        variable CAN_frame_tx_1     :       SW_CAN_frame_type;
        variable CAN_frame_tx_2     :       SW_CAN_frame_type;

        variable frame_equal        :       boolean := false;
        variable tmp_int            :       natural := 0;

        variable bus_timing         :       bit_time_config_type;

        variable frames_equal_1     :       boolean;
        variable frames_equal_2     :       boolean;
    begin

        -----------------------------------------------------------------------
        --  @1 Loop with incrementing wait time:
        -----------------------------------------------------------------------
        for wait_cycles in 0 to 10 loop

            -----------------------------------------------------------------------
            -- @1.1 Generate two random frames and insert them to TXT Buffer 1 and
            --      2. Configure priority of TXT Buffer 1 higher than TXT Buffer 2.
            -----------------------------------------------------------------------
            info_m("Step 1");

            CAN_generate_frame(CAN_frame_tx_1);
            CAN_generate_frame(CAN_frame_tx_2);

            CAN_insert_TX_frame(CAN_frame_tx_1, 1, DUT_NODE, chn);
            CAN_insert_TX_frame(CAN_frame_tx_2, 2, DUT_NODE, chn);

            CAN_configure_tx_priority(1, 5, DUT_NODE, chn);
            CAN_configure_tx_priority(2, 3, DUT_NODE, chn);

            CAN_read_timing_v(bus_timing, DUT_NODE, chn);

            -----------------------------------------------------------------------
            -- @2. Wait until sample point and issue Set ready command to TXT
            --     Buffer 2.
            -----------------------------------------------------------------------
            info_m("Step 2");

            CAN_wait_sample_point(DUT_NODE, chn, false);
            send_TXT_buf_cmd(buf_set_ready, 2, DUT_NODE, chn);

            -----------------------------------------------------------------------
            -- @3. Wait number of cycles given by iteration, and issue Set Ready
            --     Command to TXT Buffer 1.
            -----------------------------------------------------------------------
            info_m("Step 3");

            if (wait_cycles > 0) then
                for i in 1 to wait_cycles loop
                    clk_agent_wait_cycle(chn);
                end loop;
            end if;

            send_TXT_buf_cmd(buf_set_ready, 1, DUT_NODE, chn);

            -----------------------------------------------------------------------
            -- @4. Wait until frame is sent, and verify that frame from TXT Buffer 1
            --      was succesfully sent (wait times are shorter than bit time!)
            -----------------------------------------------------------------------
            CAN_wait_frame_sent(TEST_NODE, chn);

            CAN_read_frame(CAN_frame_rx_1, TEST_NODE, chn);
            CAN_compare_frames(CAN_frame_rx_1, CAN_frame_tx_1, false, frames_equal_1);

            check_m(frames_equal_1, "First frame was properly received!");

            CAN_wait_frame_sent(TEST_NODE, chn);
            CAN_read_frame(CAN_frame_rx_2, TEST_NODE, chn);
            CAN_compare_frames(CAN_frame_rx_2, CAN_frame_tx_2, false, frames_equal_2);

            check_m(frames_equal_2, "Second frame was properly received!");

            CAN_wait_bus_idle(DUT_NODE, chn);
            CAN_wait_bus_idle(TEST_NODE, chn);

        end loop;

    end procedure;
end package body;
