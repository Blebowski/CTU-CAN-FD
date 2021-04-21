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
--  Feature test for frame transmittion with invalid combination of CAN frame
--  configurations.
--
-- @Verifies:
--  @1. When CAN FD frame with RTR bit is sent, RTR bit is ignored and CAN FD
--      frame without RTR bit set is received!
--  @2. When CAN 2.0 frame with BRS bit is sent, BRS bit is ignored and CAN 2.0
--      frame without BRS bit is received!
--
-- @Test sequence:
--  @1. Send CAN FD frame with RTR bit set and check it is received without RTR.
--  @2. Send CAN 2.0 frame with BRS bit set and check it is received without BRS.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    30.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--     12.6.2018  Modified to use CAN Test lib instead of direct register
--                access functions.
--    11.11.2019  Modified to have new common format of header.
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package invalid_frames_ftest is
    procedure invalid_frames_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;

package body invalid_frames_ftest is
    procedure invalid_frames_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable tx_frame           :       SW_CAN_frame_type;
        variable rx_frame           :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
    begin

        ------------------------------------------------------------------------
        -- @1. Send CAN FD frame with RTR bit set and check it is received
        --     without RTR.
        ------------------------------------------------------------------------
        info_m("Step 1");

        CAN_generate_frame(tx_frame);
        tx_frame.frame_format := FD_CAN;
        tx_frame.rtr := RTR_FRAME;
        CAN_send_frame(tx_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_frame_sent(TEST_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);
        
        CAN_read_frame(rx_frame, TEST_NODE, chn);
        check_m(rx_frame.frame_format = FD_CAN, "FD frame received");
        check_m(rx_frame.rtr = NO_RTR_FRAME, "NO RTR received");
        
        ------------------------------------------------------------------------
        -- @2. Send CAN 2.0 frame with BRS bit set and check it is received
        --     without BRS.
        ------------------------------------------------------------------------
        info_m("Step 2");
        
        CAN_generate_frame(tx_frame);
        tx_frame.frame_format := NORMAL_CAN;
        tx_frame.brs := BR_SHIFT;
        CAN_send_frame(tx_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_frame_sent(TEST_NODE, chn);        
        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);
        
        CAN_read_frame(rx_frame, TEST_NODE, chn);
        check_m(rx_frame.brs = BR_NO_SHIFT, "Frame with no BRS received!");
        check_m(rx_frame.frame_format = NORMAL_CAN, "CAN 2.0 frame received!");        

    end procedure;
end package body;