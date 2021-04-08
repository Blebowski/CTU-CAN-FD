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
--  Overload frame feature test 
--
-- @Verifies:
--  @1. Overload frame is transmitted as a result of sampling dominant during
--      first and second bit of intermission.
--  @2. Overload frame is transmitted as a result of sampling dominant during
--      last bit of end of frame by receiver.
--  @3. Error frame is transmitted as a result of sampling dominant bit during
--      last bit of end of frame by transmitter.
--
-- @Test sequence:
--  @1. Send frame by DUT. Wait until first bit of intermission in DUT and
--      force bus level Dominant. Check that DUT transmitts Overload frame.
--      Wait until the end of overload frame in DUT.
--  @2. Check that we are in "Intermission field now". Wait until second bit of
--      Intermission and force bus low. Wait until sample point and check that
--      Node has transmitted Overload frame. Wait until the end of Overload frame.
--      Wait until bus is idle for both Nodes.
--  @3. Send frame by DUT. Wait until last bit of End of Frame field of DUT.
--      Force bus low. Check that DUT reacts with Error frame. Check that
--      Test node reacts with Overload frame.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    22.11.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package overload_ftest is
    procedure overload_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body overload_ftest is
    procedure overload_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;

        variable pc_dbg             :     SW_PC_Debug;
        variable frame_sent         :     boolean;
        variable command            :     SW_command := SW_command_rst_val;
        variable status             :     SW_status;
    begin
        
        -----------------------------------------------------------------------
        -- @1. Send frame by DUT. Wait until first bit of intermission in
        --     DUT and force bus level Dominant. Check that DUT transmitts
        --     Overload frame. Wait until the end of overload frame in DUT.
        -----------------------------------------------------------------------
        info_m("Step 1");
        
        CAN_generate_frame(frame_1);
        CAN_send_frame(frame_1, 1, DUT_NODE, chn, frame_sent);
        
        CAN_wait_pc_state(pc_deb_intermission, DUT_NODE, chn);
        wait for 15 ns;

        force_bus_level(DOMINANT, chn);
        CAN_wait_sample_point(DUT_NODE, chn, false);
        wait for 15 ns; -- To be sure sample point was processed!
        release_bus_level(chn);

        CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);

        -- Now we check for whole duration of overload flag! Check that
        -- dominant bit is transmitted!
        for i in 0 to 5 loop
            info_m("Overload flag index: " & integer'image(i));
            check_m(pc_dbg = pc_deb_overload, "Overload frame transmitted!");
            CAN_wait_sample_point(DUT_NODE, chn);
            check_can_tx(DOMINANT, DUT_NODE, "Dominant Overload flag transmitted!", chn);
        end loop;

        CAN_wait_not_pc_state(pc_deb_overload, DUT_NODE, chn); 

        -----------------------------------------------------------------------
        -- @2. Check that we are in "Intermission field now". Wait until second
        --     bit of Intermission and force bus low. Wait until sample point
        --     and check that Node has transmitted Overload frame. Wait until
        --     the end of Overload frame. Wait until bus is idle for both Nodes.
        -----------------------------------------------------------------------
        info_m("Step 2");
        
        CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
        check_m(pc_dbg = pc_deb_intermission, "Intermission after Overload frame");

        CAN_wait_sample_point(DUT_NODE, chn, false);

        -- Now we are beyond sample point in first bit of intermission!
        force_bus_level(DOMINANT, chn);
        CAN_wait_sample_point(DUT_NODE, chn, false);
        wait for 15 ns; -- To be sure sample point was processed!
        release_bus_level(chn);

        -- Now we check for whole duration of overload flag! Check that
        -- dominant bit is transmitted!
        for i in 0 to 5 loop
            info_m("Overload flag index: " & integer'image(i));
            CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
            check_m(pc_dbg = pc_deb_overload, "Overload frame transmitted!");
            CAN_wait_sample_point(DUT_NODE, chn, false);
            check_can_tx(DOMINANT, DUT_NODE, "Dominant Overload flag transmitted!", chn);
        end loop;

        CAN_wait_not_pc_state(pc_deb_overload, DUT_NODE, chn); 
        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);

        -----------------------------------------------------------------------
        -- @3. Send frame by DUT. Wait until last bit of End of Frame field
        --     of DUT. Force bus low. Check that DUT reacts with Error
        --     frame. Check that Test node reacts with Overload frame.
        -----------------------------------------------------------------------
        info_m("Step 3");
        CAN_generate_frame(frame_1);
        CAN_send_frame(frame_1, 1, DUT_NODE, chn, frame_sent);

        CAN_wait_pc_state(pc_deb_eof, DUT_NODE, chn);
        for i in 0 to 5 loop
            CAN_wait_sample_point(DUT_NODE, chn, false);
        end loop;
        
        -- This is to cover possibility that sample point of one bit before end
        -- of EOF in Test node did not pass yet! We want to be also in last bit
        -- of EOF of Test node, because only then we get Overload frame!
        wait for 400 ns;

        -- Now we should be in one bit before the end of EOF!
        force_bus_level(DOMINANT, chn);
        CAN_wait_sample_point(DUT_NODE, chn, false);
        CAN_wait_sample_point(DUT_NODE, chn, false);
        wait for 25 ns; -- To be sure sample point was processed!
        release_bus_level(chn);

        get_controller_status(status, DUT_NODE, chn);
        check_m(status.error_transmission,
            "Transmitter sends error frame due to dominant bit in last bit of EOF!");

        -- Test node is transmitting overload as a result of last bit of EOF
        -- dominant during EOF! 
        CAN_wait_sample_point(DUT_NODE, chn, false);
        CAN_read_pc_debug_m(pc_dbg, TEST_NODE, chn);
        check_m(pc_dbg = pc_deb_overload,
            "Receiver sends overload frame due to dominant bit in last bit of EOF!");

        CAN_wait_bus_idle(DUT_NODE, chn);

  end procedure;

end package body;