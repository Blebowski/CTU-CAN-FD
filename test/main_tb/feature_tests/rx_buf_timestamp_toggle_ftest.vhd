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
--  Feature test for Timestamp toggle coverage in RX Buffer.
--
-- @Verifies:
--  @1. Bits 63:16 of timestamp are stored correctly in RX Buffer.
--
-- @Test sequence:
--  @1. Set bits 63:16 to 1 in TB (timestamp agent)
--  @2. Generate CAN frame and send it by Test Node. Wait until the frame
--      is received by DUT.
--  @3. Read frame from DUT and check highest bits of timestamp are set to 1.
--  @4. Drive all timestamp to 0.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    29.7.2024     Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.timestamp_agent_pkg.all;

package rx_buf_timestamp_toggle_ftest is
    procedure rx_buf_timestamp_toggle_ftest_exec(
        signal      chn             : inout  t_com_channel
	);
end package;


package body rx_buf_timestamp_toggle_ftest is
    procedure rx_buf_timestamp_toggle_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable ts_val             :        std_logic_vector(63 downto 0);
        variable tx_can_frame       :        SW_CAN_frame_type;
        variable rx_can_frame       :        SW_CAN_frame_type;
        variable frame_sent         :        boolean;
    begin

        -----------------------------------------------------------------------
        -- @1. Set bits 63:16 to 1 in TB (timestamp agent)
        -----------------------------------------------------------------------
        info_m("Step 1");

        -- LSB 16 bits at zero provide 2^16 cycles until timestamp overflow.
        -- This is sufficient reserve, and overflow will not occur even with
        -- 64 byte long frame, and thus bits 64:16 will always stay at 0xFF.
        ts_val := x"FFFFFFFFFFFF0000";
        info_m("Forcing start timestamp in DUT to: " & to_hstring(ts_val));
        ftr_tb_set_timestamp(ts_val, chn);

        -----------------------------------------------------------------------
        -- @2. Generate CAN frame and send it by Test Node. Wait until the
        --     frame is received by DUT.
        -----------------------------------------------------------------------
        info_m("Step 2");

        CAN_generate_frame(tx_can_frame);

        CAN_send_frame(tx_can_frame, 1, TEST_NODE, chn, frame_sent);
        CAN_wait_frame_sent(DUT_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @3. Read frame from DUT and check highest bits of timestamp are set
        --     to 1.
        -----------------------------------------------------------------------
        info_m("Step 3");

        CAN_read_frame(rx_can_frame, DUT_NODE, chn);
        check_m(rx_can_frame.timestamp(63 downto 16) = x"FFFFFFFFFFFF",
                    "Highest RX Buffer timestamp bits all 0xFFF");

        -----------------------------------------------------------------------
        -- @4. Drive all timestamp to 0.
        -----------------------------------------------------------------------
        info_m("Step 4");

        ftr_tb_set_timestamp(x"0000000000000000", chn);

    end procedure;

end package body;
