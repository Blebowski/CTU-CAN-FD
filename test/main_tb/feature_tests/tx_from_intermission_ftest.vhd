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
--  Start of transmission from Intermission 
--
-- @Verifies:
--  @1. Transmission is started when Node detects Dominant bit during third
--      bit of intermission and it has frame for transmission available.
--  @2. Reception is started when Node detectes Dominant bit during third bit
--      of Intermission and it has no frame for transmission available.
--
-- @Test sequence:
--  @1. Insert CAN frame for transmission into Test node. Wait until transmission
--      will be started. Insert CAN frame to DUT during transmission of frame
--      from Test node and wait until Intermission.
--  @2. Wait for two sample points and force the bus dominant during third bit
--      of intermission for DUT. Wait until sample point, and check that Node
--      1 started transmitting. Check that DUT is in "arbitration" phase.
--      Check that DUT is NOT in SOF. Wait until frame is sent, and check
--      it is properly receieved by Test node (Test node should have turned receiver).
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

package tx_from_intermission_ftest is
    procedure tx_from_intermission_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body tx_from_intermission_ftest is
    procedure tx_from_intermission_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;
        variable frame_2            :     SW_CAN_frame_type;
        variable frame_rx           :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;
        variable frame_sent         :     boolean;
        variable frame_equal        :     boolean;
    begin

        -----------------------------------------------------------------------
        -- @1. Insert CAN frame for transmission into Test node. Wait until
        --    transmission will be started. Insert CAN frame to DUT during 
        --    transmission of frame from Test node and wait until Intermission.
        -----------------------------------------------------------------------
        info_m("Step 1");

        CAN_generate_frame(frame_1);
        CAN_send_frame(frame_1, 1, TEST_NODE, chn, frame_sent);

        CAN_wait_tx_rx_start(true, false, TEST_NODE, chn);
        wait for 5000 ns; -- To be sure DUT started reception!

        CAN_generate_frame(frame_2);
        CAN_send_frame(frame_2, 1, DUT_NODE, chn, frame_sent);

        -- We must wait for Intermission of DUT! Only that way we are sure
        -- we properly measure two bits of its intermission!
        CAN_wait_pc_state(pc_deb_intermission, DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @2. Wait for two sample points and force the bus dominant during
        --     third bit of intermission for DUT. Wait until sample point,
        --     and check that DUT started transmitting. Check that DUT is
        --     in "arbitration" phase. Check that DUT is NOT in SOF. Wait 
        --     until frame is sent, and check it is properly receieved by Test
        --     node (Test node should have turned receiver).
        -----------------------------------------------------------------------
        info_m("Step 2");

        CAN_wait_sample_point(DUT_NODE, chn, false);
        CAN_wait_sample_point(DUT_NODE, chn, false);

        -- This is needed to be sure that Test node also reached second sample
        -- point of intermission. Otherwise, it would interpret this as
        -- overload condition, and it would not turn reciever!
        wait for 100 ns;

        force_bus_level(DOMINANT, chn);
        CAN_wait_sample_point(DUT_NODE, chn, false);
        wait for 15 ns; -- To be sure sample point was processed!
        release_bus_level(chn);
        
        -- Now DUT thinks that third bit of its intermission was dominant.
        -- It should start transmitting with first bit of Base ID. Test node should
        -- also interpret this as SOF and start receiving.
        -- Hopefully, Test node clock will not be too slow so that it will still
        -- catch this as second bit of Intermission and Interpret this as
        -- Overload frame ...

        CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);     
        check_m(pc_dbg = pc_deb_arbitration, "Test node in arbitration");
        check_false_m(pc_dbg = pc_deb_sof, "Test node NOT in SOF!");

        get_controller_status(stat_1, DUT_NODE, chn);
        check_m(stat_1.transmitter, "DUT transmitter!");

        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);

        CAN_read_frame(frame_rx, TEST_NODE, chn);
        CAN_compare_frames(frame_rx, frame_2, false, frame_equal);

        check_m(frame_equal, "TX/RX frame match");

  end procedure;

end package body;
