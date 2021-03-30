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
--  One shot mode feature test (Retransmitt limit = 0).
--
-- @Verifies:
--  @1. One shot mode - Retransmitt limit enabled and set to 0. Verifies there is
--      only one atempt to transmitt a frame in one shot mode
--  @2. When One shot mode is not set (retransmit limit = 0, but disabled),
--      core does not stop re-transmitting after retransmitt limit number of
--      retransmissions was reached (retransmitts indefinitely).
--  @3. When transmission fails as result of Error frame, device in One shot mode
--      does not transmitt anymore!
--  @4. When transmission fails as result of Arbitration loss, device in One shot
--      mode does not transmitt anymore!
--
-- @Test sequence:
--  @1. Set retransmitt limit to 0 in DUT. Enable retransmitt limitations.
--      Set Acknowledge forbidden mode in Test node (to produce ACK errors). Turn
--      on Test mode in DUT (to manipulate error counters).
--  @2. Generate frame and start sending the frame by DUT. Wait until
--      error frame occurs and transmission is over.
--  @3. Check transmission failed and transmitting TXT Buffer is "TX Error".
--  @4. Disable retransmitt limitions in DUT. Start sending a frame by DUT.
--      Wait until error frame and check that transmitting TXT Buffer is "Ready"
--      again (hitting current retransmitt limit did not cause stopping
--      retransmissions when retransmitt limit is disabled).
--  @5. Abort transmission by DUT. Wait until transmission was aborted.
--  @6. Insert frames for transmission to DUT and Test node simultaneously
--      to invoke arbitration. ID of frame in DUT is higher than the one in
--      Test node (to loose arbitration). Wait until node 1 is in Control field of
--      a frame. Check that DUT is receiver (arbitration was really lost) and
--      TXT Buffer in DUT ended up in "TX Error" state.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    06.7.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package one_shot_ftest is
    procedure one_shot_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body one_shot_ftest is
    procedure one_shot_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        variable buf_state          :       SW_TXT_Buffer_state_type;
        variable status             :       SW_status;
    begin

        ------------------------------------------------------------------------
        -- @1. Set retransmitt limit to 0 in DUT. Enable retransmitt 
        --     limitations. Set Acknowledge forbidden mode in Test node (to 
        --     produce ACK errors). Turn on Test mode in DUT (to manipulate  
        --     error counters).
        ------------------------------------------------------------------------
        info_m("Step 1: Configuring One shot Mode (DUT), ACF (Test node)");
        
        CAN_enable_retr_limit(true, 0, DUT_NODE, chn);
        
        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, TEST_NODE, chn);
        
        mode_1.test := true;
        set_core_mode(mode_1, DUT_NODE, chn);
        
        ------------------------------------------------------------------------
        -- @2. Generate frame and start sending the frame by DUT. Wait until
        --     error frame occurs and transmission is over.
        ------------------------------------------------------------------------
        info_m("Step 2: Sending frame by DUT");
        
        CAN_generate_frame(CAN_frame);
        CAN_frame.rtr := RTR_FRAME; -- Use RTR frame to save simulation time
        CAN_frame.frame_format := NORMAL_CAN;
        
        CAN_send_frame(CAN_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_error_frame(DUT_NODE, chn);
        
        CAN_wait_bus_idle(DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- @3. Check transmission failed and transmitting TXT Buffer is
        --     "TX Error".
        ------------------------------------------------------------------------
        info_m("Step 3: Checking transmission failed.");
        
        get_tx_buf_state(1, buf_state, DUT_NODE, chn);
        check_m(buf_state = buf_failed, "TXT Buffer failed!");
        
        ------------------------------------------------------------------------
        -- @4. Disable retransmitt limitions in DUT. Start sending a frame by
        --     DUT. Wait until error frame and check that transmitting TXT
        --     Buffer is "Ready" again (hitting current retransmitt limit did not
        --     cause stopping retransmissions when retransmitt limit is disabled).
        ------------------------------------------------------------------------
        info_m("Step 4: Testing disabled One shot mode");
        
        CAN_enable_retr_limit(false, 0, DUT_NODE, chn);
        CAN_send_frame(CAN_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_error_frame(DUT_NODE, chn);
        
        get_tx_buf_state(1, buf_state, DUT_NODE, chn);
        check_m(buf_state = buf_ready, "TXT Buffer ready!");
        
        ------------------------------------------------------------------------
        -- @5. Abort transmission by DUT. Wait until transmission was aborted.
        ------------------------------------------------------------------------
        info_m("Step 5: Aborting transmission");
        
        send_TXT_buf_cmd(buf_set_abort, 1, DUT_NODE, chn);
        get_tx_buf_state(1, buf_state, DUT_NODE, chn);
        while (buf_state /= buf_aborted) loop
            get_tx_buf_state(1, buf_state, DUT_NODE, chn);
        end loop;        
        CAN_wait_bus_idle(DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- @6. Insert frames for transmission to DUT and Test node simultaneously
        --     to invoke arbitration. ID of frame in DUT is higher than the
        --     one in Test node (to loose arbitration). Wait until node 1 is in 
        --     Control field of a frame. Check that DUT is receiver 
        --     (arbitration was really lost) and TXT Buffer in DUT ended up
        --     in "TX Error" state.
        ------------------------------------------------------------------------
        info_m("Step 6: Testing One shot due to arbitration loss!");
        
        CAN_enable_retr_limit(true, 0, DUT_NODE, chn);
        
        CAN_frame.ident_type := BASE;
        CAN_frame.identifier := 10;
        CAN_insert_TX_frame(CAN_frame, 1, DUT_NODE, chn);
        
        CAN_frame.identifier := 9;
        CAN_insert_TX_frame(CAN_frame, 1, TEST_NODE, chn);
        
        -- TODO: Use atomic procedure after priority test is merged to be sure!
        send_TXT_buf_cmd(buf_set_ready, 1, DUT_NODE, chn);
        send_TXT_buf_cmd(buf_set_ready, 1, TEST_NODE, chn);
        
        CAN_wait_pc_state(pc_deb_control, DUT_NODE, chn);
        
        get_controller_status(status, DUT_NODE, chn);
        check_m(status.receiver, "DUT lost arbitration");
        
        get_tx_buf_state(1, buf_state, DUT_NODE, chn);
        check_m(buf_state = buf_failed, "TXT Buffer failed");
        CAN_wait_bus_idle(DUT_NODE, chn);
        
  end procedure;

end package body;