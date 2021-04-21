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
--  Loopback mode - feature test.
--
-- @Verifies:
--  @1. Transmitted CAN frame will be also received in Loopback mode when frame
--      is transmitted successfully.
--  @2. Transmitted CAN frame will not be received in Loopback mode when error
--      frame occurs.
--  @3. Transmitted CAN frame will not be received be node itself when Loopback
--      mode was disabled.
--
-- @Test sequence:
--  @1. Configure Loopback mode in DUT.
--  @2. Generate random CAN frame and send it by DUT.
--  @3. Wait until frame is received. Check that DUT has 1 frame in RX Buffer.
--  @4. Read CAN frame from DUT. Check it is the same as transmitted frame.
--      Check that there are 0 frames in RX Buffer of DUT. Read also frame
--      from Test node not to leave it hanging there!
--  @5. Set Test node to Acknowledge forbidden mode. Set DUT to one shot mode.
--  @6. Generate random CAN frame and send it by DUT.
--  @7. Wait until transmission is over. Check that TXT Buffer used for transmi-
--      ssion is in TX failed. Check that RX Buffer in DUT has no frame.
--  @8. Disable Loopback mode in DUT. Disable Acknowledge forbidden mode in
--      Test node.
--  @9. Send CAN frame by DUT. Wait until frame is over.
-- @10. Check that RX Buffer of DUT has no CAN frame received. Check that
--      RX Buffer of Test node has frame received.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    18.9.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package mode_loopback_ftest is
    procedure mode_loopback_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body mode_loopback_ftest is
    procedure mode_loopback_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_TX_frame       :       SW_CAN_frame_type;
        variable CAN_RX_frame       :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;

        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        variable txt_buf_state      :       SW_TXT_Buffer_state_type;
        variable rx_buf_state       :       SW_RX_Buffer_info;
        variable frames_equal       :       boolean := false;
    begin

        ------------------------------------------------------------------------
        -- @1. Configure Loopback mode in DUT.
        ------------------------------------------------------------------------
        info_m("Step 1: Configuring Loopback mode in DUT");
        
        mode_1.internal_loopback := true;
        set_core_mode(mode_1, DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- @2. Generate random CAN frame and send it by DUT.
        ------------------------------------------------------------------------
        info_m("Step 2: Sending frame by DUT");
        
        CAN_generate_frame(CAN_TX_frame);
        CAN_send_frame(CAN_TX_frame, 1, DUT_NODE, chn, frame_sent);
        
        ------------------------------------------------------------------------
        -- @3. Wait until frame is received. Check that DUT has 1 frame in 
        --     RX Buffer.
        ------------------------------------------------------------------------
        info_m("Step 3: Waiting until frame is sent");
        
        CAN_wait_frame_sent(DUT_NODE, chn);
        get_rx_buf_state(rx_buf_state, DUT_NODE, chn);
        check_m(rx_buf_state.rx_frame_count = 1, "Own frame in Loopback received");

        ------------------------------------------------------------------------
        -- @4. Read CAN frame from DUT. Check it is the same as transmitted 
        --    frame. Check that there are 0 frames in RX Buffer of DUT.
        ------------------------------------------------------------------------
        info_m("Step 4: Read own transmitted frame from RX Buffer");
        
        CAN_read_frame(CAN_RX_frame, DUT_NODE, chn);
        CAN_compare_frames(CAN_RX_frame, CAN_TX_frame, false, frames_equal);
        check_m(frames_equal, "Own frame in Loopback is the same as sent!");
        
        get_rx_buf_state(rx_buf_state, DUT_NODE, chn);
        check_m(rx_buf_state.rx_frame_count = 0, "Own frame read from RX Buffer");
        
        CAN_read_frame(CAN_RX_frame, TEST_NODE, chn);
        
        ------------------------------------------------------------------------
        -- @5. Set Test node to Acknowledge forbidden mode. Set DUT to one shot
        --    mode.
        ------------------------------------------------------------------------
        info_m("Step 5: Configure Test node to ACF, DUT to One shot mode");
        
        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, TEST_NODE, chn);
        CAN_enable_retr_limit(true, 0, DUT_NODE, chn);
        
        ------------------------------------------------------------------------
        -- @6. Generate random CAN frame and send it by DUT.
        ------------------------------------------------------------------------
        info_m("Step 6: Send frame by DUT!");
        
        CAN_generate_frame(CAN_TX_frame);
        CAN_send_frame(CAN_TX_frame, 1, DUT_NODE, chn, frame_sent);
        
        ------------------------------------------------------------------------
        -- @7. Wait until transmission is over. Check that TXT Buffer used for 
        --    transmission is in TX failed. Check that RX Buffer in DUT has 
        --    no frame.
        ------------------------------------------------------------------------
        info_m("Step 7: Check no own frame received on Error frame!");
        
        CAN_wait_error_frame(DUT_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);
        
        get_tx_buf_state(1, txt_buf_state, DUT_NODE, chn);
        check_m(txt_buf_state = buf_failed, "TXT Buffer failed");
        
        get_rx_buf_state(rx_buf_state, DUT_NODE, chn);
        check_m(rx_buf_state.rx_frame_count = 0,
            "No own frame received when error frame was received!");

        ------------------------------------------------------------------------
        -- @8. Disable Loopback mode in DUT. Disable Acknowledge forbidden 
        --    mode in Test node.
        ------------------------------------------------------------------------
        info_m("Step 8: Disable Loopback in DUT. Disable ACF in Test node!");
        
        mode_1.internal_loopback := false;
        set_core_mode(mode_1, DUT_NODE, chn);
        
        mode_2.acknowledge_forbidden := false;
        set_core_mode(mode_2, TEST_NODE, chn);
        
        ------------------------------------------------------------------------
        -- @9. Send CAN frame by DUT. Wait until frame is over.
        ------------------------------------------------------------------------
        info_m("Step 9: Send CAN frame by DUT.");
        
        CAN_generate_frame(CAN_TX_frame);
        CAN_send_frame(CAN_TX_frame, 1, DUT_NODE, chn, frame_sent);
        
        CAN_wait_frame_sent(DUT_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);
        
        ------------------------------------------------------------------------
        -- @10. Check that RX Buffer of DUT has no CAN frame received. Check 
        --     that RX Buffer of Test node has frame received.
        ------------------------------------------------------------------------
        info_m("Step 10: Check own frame not received when Loopback is disabled");
        
        get_rx_buf_state(rx_buf_state, DUT_NODE, chn);
        check_m(rx_buf_state.rx_frame_count = 0,
            "Own frame not received when Loopback mode is disabled!");
        
        get_rx_buf_state(rx_buf_state, TEST_NODE, chn);
        check_m(rx_buf_state.rx_frame_count = 1,
            "Frame received in Test node!");
        
        CAN_read_frame(CAN_RX_frame, TEST_NODE, chn);
        CAN_compare_frames(CAN_RX_frame, CAN_TX_frame, false, frames_equal);
        check_m(frames_equal, "TX vs. RX frame matching!");

        wait for 1000 ns;
        
  end procedure;

end package body;