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
--  TX Status feature test.
--
-- @Verifies:
--  @1. TXT Buffer is Empty after reset
--  @2. TXT Buffer is OK after successfull transmission.
--  @2. TXT Buffer is Failed after transmission fails from this buffer!
--  @3. TXT Buffer is Aborted after Set Abort command was issued during
--      transmission.
--
-- @Test sequence:
--  @1. Reset Node, Enable it, and wait until it integrates. Pick random TXT
--      Buffer which will be used during this test. Check that TXT Buffer is
--      empty.
--  @2. Transmitt CAN Frame by DUT and wait until it is received in Test node.
--      Check that TXT Buffer is OK.
--  @3. Set ACK Forbidden mode in Test node. Set One shot mode in DUT. Send frame
--      by DUT and wait until it is sent. Check that TXT Buffer is in TX
--      Failed.
--  @4. Send CAN frame and when it starts, issue Set Abort Command. Wait until
--      frame is sent and check that TXT Buffer is in Aborted.
--
-- Note:
--  Ready, TX in Progress and Abort in Progress are not tested here as they are
--  checked in tx_cmd_set_ready/empty/abort test case.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--     22.11.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package tx_status_ftest is
    procedure tx_status_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;

package body tx_status_ftest is

    procedure tx_status_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is        
        variable CAN_frame_tx       :       SW_CAN_frame_type;        
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        variable frame_sent         :       boolean := false;
        variable bus_timing         :       bit_time_config_type;
        variable txt_state          :       SW_TXT_Buffer_state_type;
        variable txt_buf_num        :       natural;
    begin

        -----------------------------------------------------------------------
        -- @1. Reset Node, Enable it, and wait until it integrates. Pick random
        --     TXT Buffer which will be used during this test. Check that TXT
        --     Buffer is empty.
        -----------------------------------------------------------------------
        info_m("Step 1");
        
        pick_random_txt_buffer(txt_buf_num, DUT_NODE, chn);

        CAN_read_timing_v(bus_timing, DUT_NODE, chn);
        exec_SW_reset(DUT_NODE, chn);
        exec_SW_reset(TEST_NODE, chn);
        CAN_configure_timing(bus_timing, DUT_NODE, chn);
        CAN_configure_timing(bus_timing, TEST_NODE, chn);

        CAN_turn_controller(true, DUT_NODE, chn);
        CAN_turn_controller(true, TEST_NODE, chn);

        -- Wait till integration is over!
        CAN_wait_bus_on(DUT_NODE, chn);
        CAN_wait_bus_on(TEST_NODE, chn);

        get_tx_buf_state(txt_buf_num, txt_state, DUT_NODE, chn);
        check_m(txt_state = buf_empty, "TX Empty after reset!");

        -----------------------------------------------------------------------
        -- @2. Transmitt CAN Frame by DUT and wait until it is received in
        --     Test node. Check that TXT Buffer is OK.
        -----------------------------------------------------------------------
        info_m("Step 2");
        
        CAN_generate_frame(CAN_frame_tx);
        CAN_send_frame(CAN_frame_tx, txt_buf_num, DUT_NODE, chn, frame_sent);

        CAN_wait_frame_sent(DUT_NODE, chn);

        get_tx_buf_state(txt_buf_num, txt_state, DUT_NODE, chn);
        check_m(txt_state = buf_done, "TX OK after frame sent!");

        -----------------------------------------------------------------------
        -- @3. Set ACK Forbidden mode in Test node. Set One shot mode in DUT. 
        --     Send frame by DUT and wait until it is sent. Check that TXT
        --     Buffer is in TX Failed.
        -----------------------------------------------------------------------
        info_m("Step 3");

        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, TEST_NODE, chn);

        CAN_enable_retr_limit(true, 0, DUT_NODE, chn);

        CAN_generate_frame(CAN_frame_tx);
        CAN_send_frame(CAN_frame_tx, txt_buf_num, DUT_NODE, chn, frame_sent);
        CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);
        
        get_tx_buf_state(txt_buf_num, txt_state, DUT_NODE, chn);
        check_m(txt_state = buf_failed, "TX Failed after error!");

        -----------------------------------------------------------------------
        -- @4. Send CAN frame and when it starts, issue Set Abort Command. Wait
        --     until frame is sent and check that TXT Buffer is in Aborted.
        -----------------------------------------------------------------------
        info_m("Step 4");

        -- One shot must be disabled now, otherwise this would lead to
        -- TX Failed. We need transmission to be failed, but retransmitt limit
        -- not reached!
        CAN_enable_retr_limit(false, 5, DUT_NODE, chn);

        CAN_generate_frame(CAN_frame_tx);
        CAN_send_frame(CAN_frame_tx, txt_buf_num, DUT_NODE, chn, frame_sent);
        
        CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);
        send_TXT_buf_cmd(buf_set_abort, txt_buf_num, DUT_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);

        get_tx_buf_state(txt_buf_num, txt_state, DUT_NODE, chn);
        check_m(txt_state = buf_aborted, "TX Aborted after Set Abort!");

    end procedure;
end package body;
