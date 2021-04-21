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
--  No Start of Frame feature test - frame transmission!
--
-- @Verifies:
--  @1. When a dominant bit is sampled in Bus idle and a frame is available for
--      transmission, its transmission is started without transmitting SOF bit.
--  @2. When CTU CAN FD joins transmission without transmitting SOF bit, it
--      accounts SOF bit as transmitted dominant bit in number of equal conse-
--      cutive bits.
--
-- @Test sequence:
--  @1. Configure both Nodes to one-shot mode.
--  @2. Insert CAN frames which have first 5 bits of identifier equal to zero to
--      both nodes. Check both nodes are Idle. Wait till Sample point in DUT.
--  @3. Send Set ready command to both nodes. Wait until DUT is not in Bus
--      idle state. Check it is transmitting Base Identifier (NOT SOF)!
--  @4. Wait until sample point 5 times (5th bit of idetifier) in DUT. Check
--      DUT is transmitting Recessive bit (Stuff bit).
--  @5. Wait until frame is over. Check frame was received OK, read it from 
--      receiving node and verify it was received OK!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    07.10.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package no_sof_tx_ftest is
    procedure no_sof_tx_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body no_sof_tx_ftest is
    procedure no_sof_tx_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;
        variable frame_2            :     SW_CAN_frame_type;
        variable frame_rx           :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;
        
        variable txt_buf_state      :     SW_TXT_Buffer_state_type;
        variable rx_buf_info        :     SW_RX_Buffer_info;
        variable frames_equal       :     boolean := false;
                 
        variable pc_state           :     SW_PC_Debug;
    begin

        ------------------------------------------------------------------------
        -- @1. Configure both Nodes to one-shot mode.
        ------------------------------------------------------------------------
        info_m("Step 1: Configure one -shot mode");

        CAN_enable_retr_limit(true, 0, DUT_NODE, chn);
        CAN_enable_retr_limit(true, 0, TEST_NODE, chn);

        ------------------------------------------------------------------------
        -- @2. Insert CAN frames which have first 5 bits of identifier equal to
        --    zero to both nodes. Check both nodes are Idle. Wait till Sample
        --    point in Test node.
        ------------------------------------------------------------------------
        info_m("Step 2: Insert CAN frames!");

        CAN_generate_frame(frame_1);
        CAN_generate_frame(frame_2);
        frame_1.ident_type := BASE;
        frame_2.ident_type := BASE;
        frame_1.identifier := 1;
        frame_2.identifier := 2;
        -- Use FD can frames, they contain stuff count!!
        frame_1.frame_format := FD_CAN;
        frame_2.frame_format := FD_CAN;
        CAN_insert_TX_frame(frame_1, 1, DUT_NODE, chn);
        CAN_insert_TX_frame(frame_2, 1, TEST_NODE, chn);
        CAN_wait_sample_point(TEST_NODE, chn);
        
        ------------------------------------------------------------------------
        -- @3. Send Set ready command to both nodes. Wait until DUT is not in
        --    Bus idle state. Check it is transmitting Base Identifier (NOT SOF)!
        ------------------------------------------------------------------------
        info_m("Step 3");

        send_TXT_buf_cmd(buf_set_ready, 1, TEST_NODE, chn);
        CAN_wait_sample_point(TEST_NODE, chn);
        send_TXT_buf_cmd(buf_set_ready, 1, DUT_NODE, chn);
        
        -- Wait until bus is not idle by DUT!
        get_controller_status(stat_1, DUT_NODE, chn);
        while (stat_1.bus_status) loop
            get_controller_status(stat_1, DUT_NODE, chn);
        end loop;

        CAN_read_pc_debug_m(pc_state, DUT_NODE, chn);
        check_m(pc_state = pc_deb_arbitration, "DUT did not transmitt SOF!");
        wait for 20 ns;

        ------------------------------------------------------------------------
        -- @4. Wait until sample point 5 times (5th bit of idetifier) in DUT.
        --    Check DUT is transmitting Recessive bit (Stuff bit).
        ------------------------------------------------------------------------
        info_m("Step 4");

        for i in 0 to 4 loop
            CAN_wait_sample_point(DUT_NODE, chn, skip_stuff_bits => false);
        end loop;
        check_can_tx(RECESSIVE, DUT_NODE, "Stuff bit inserted!", chn);

        ------------------------------------------------------------------------
        -- @5. Wait until frame is over. Check frame was received OK, read it 
        --    from receiving node and verify it was received OK!
        ------------------------------------------------------------------------
        info_m("Step 5");

        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);

        get_tx_buf_state(1, txt_buf_state, DUT_NODE, chn);
        check_m(txt_buf_state = buf_done, "Frame transmitted OK!");
        
        get_rx_buf_state(rx_buf_info, TEST_NODE, chn);
        check_m(rx_buf_info.rx_frame_count = 1, "Frame received OK!");

        CAN_read_frame(frame_rx, TEST_NODE, chn);
        CAN_compare_frames(frame_rx, frame_1, false, frames_equal);
        check_m(frames_equal, "TX vs. RX frames match!");

    end procedure;
end package body;
