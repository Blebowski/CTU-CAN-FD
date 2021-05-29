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
--  Feature test which disables node while it is transmitting frame. Test
--  implemented to achieve full coverage of operation control
--
-- @Verifies:
--  @1. Device can be disabled while transmitting.
--
-- @Test sequence:
--  @1. Generate random frame and send it by DUT.
--  @2. Wait until frame starts in DUT, and disable DUT.
--  @3. Wait until bus is idle in Test Node (Test Node will transmitt error 
--      frame).
--  @4. Enable DUT again, and transmitt another frame. Wait until it is send,
--      and check it is received by Test Node.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    29.05.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package disable_in_tx_ftest is
    procedure disable_in_tx_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body disable_in_tx_ftest is

    procedure disable_in_tx_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable r_data     : std_logic_vector(31 downto 0) := (OTHERS => '0');
        variable CAN_frame  : SW_CAN_frame_type;
        variable CAN_frame_2: SW_CAN_frame_type;
        variable outcome    : boolean;
        variable frame_sent : boolean;
    begin

        -----------------------------------------------------------------------
        -- @1. Generate random frame and send it by DUT.
        -----------------------------------------------------------------------
        info_m("Step 1");

        CAN_generate_frame(CAN_frame);
        CAN_send_frame(CAN_frame, 1, DUT_NODE, chn, frame_sent);
        
        -----------------------------------------------------------------------
        -- @2. Wait until frame starts in DUT, and disable DUT.
        -----------------------------------------------------------------------
        info_m("Step 2");

        CAN_wait_pc_state(pc_deb_control, DUT_NODE, chn);
        CAN_turn_controller(false, DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @3. Wait until bus is idle in Test Node (Test Node will transmitt
        --     error frame).
        -----------------------------------------------------------------------
        info_m("Step 3");
        
        CAN_wait_bus_idle(TEST_NODE, chn);
        wait for 1000 ns;

        -----------------------------------------------------------------------
        -- @4. Enable DUT again, and transmitt another frame. Wait until it is
        --     send, and check it is received by Test Node.
        -----------------------------------------------------------------------
        info_m("Step 4");
        
        CAN_turn_controller(true, DUT_NODE, chn);
        CAN_wait_bus_on(DUT_NODE, chn);

        CAN_generate_frame(CAN_frame);
        CAN_send_frame(CAN_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_frame_sent(DUT_NODE, chn);

        CAN_wait_bus_idle(TEST_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);

        CAN_read_frame(CAN_frame_2, TEST_NODE, chn);
        CAN_compare_frames(CAN_frame, CAN_frame_2, false, outcome);

        check_m(outcome, "TX/RX frames equal");

  end procedure;

end package body;
