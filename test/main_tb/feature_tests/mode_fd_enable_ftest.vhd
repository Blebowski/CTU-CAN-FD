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
--  Flexible data-rate enable feature test!
--
-- @Verifies:
--  @1. CAN FD frame (EDL bit recessive) is received OK when Flexible data-rate
--      mode is enabled (as default).
--  @2. Receiving CAN FD frame results in Error frame when Flexible data-rate
--      mode is disabled.
--  @3. Transmitting CAN FD frame when Flexible data-rate mode is disabled
--      results in transmission of CAN 2.0 frame only!
--
-- @Test sequence:
--  @1. Send CAN FD frame by Test node. Wait till frame is sent. Read it from
--      DUT and compare it with send frame.
--  @2. Disable Flexible data-rate mode in DUT. Send CAN frame by Test node.
--      Wait till Control field of DUT. Set both nodes to One-shot mode.
--  @3. Wait till DUT is not in Control field. Check that it is transmitting
--      error frame. Read Error code capture and check that it shows Form Error
--      during Control field. Wait till the frame is transmitted.
--  @4. Set Test node to Acknowledge forbidden mode. Transmitt frame by DUT.
--      Wait till it is sent, read Error code capture and check it is NOT equal
--      to Form error (this is just to achieve change in Error code capture).  
--  @5. Unset ACK forbidden in Test node. Send CAN FD frame by DUT.
--  @6. Wait until frame is sent and check that it is received OK in Test node
--      Make sure it CAN 2.0 frame.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    22.9.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package mode_fd_enable_ftest is
    procedure mode_fd_enable_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body mode_fd_enable_ftest is
    procedure mode_fd_enable_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_TX_frame       :       SW_CAN_frame_type;
        variable CAN_RX_frame       :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;

        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        variable status             :       SW_status;
        variable frames_equal       :       boolean := false;
        variable err_capt           :       SW_error_capture;
    begin

        ------------------------------------------------------------------------
        -- @1. Send CAN FD frame by Test node. Wait till frame is sent. Read it
        --     from DUT and compare it with send frame.
        ------------------------------------------------------------------------
        info_m("Step 1: Sending CAN FD frame when FD mode enabled!");
        
        CAN_generate_frame(CAN_TX_frame);
        CAN_TX_frame.frame_format := FD_CAN;
        CAN_send_frame(CAN_TX_frame, 1, TEST_NODE, chn, frame_sent);
        CAN_wait_frame_sent(DUT_NODE, chn);
        
        CAN_read_frame(CAN_RX_frame, DUT_NODE, chn);
        CAN_compare_frames(CAN_RX_frame, CAN_TX_frame, false, frames_equal);
        check_m(frames_equal, "TX - RX frames matching!");

        ------------------------------------------------------------------------
        -- @2. Disable Flexible data-rate mode in DUT. Send CAN frame by 
        --     Test node. Wait till Control field of DUT. Set both nodes to
        --     One-shot mode.
        ------------------------------------------------------------------------
        info_m("Step 2: Disable FD mode, send frame!");
        
        mode_1.flexible_data_rate := false;
        set_core_mode(mode_1, DUT_NODE, chn);
        
        CAN_enable_retr_limit(true, 0, TEST_NODE, chn);
        CAN_enable_retr_limit(true, 0, DUT_NODE, chn);
        
        CAN_send_frame(CAN_TX_frame, 1, TEST_NODE, chn, frame_sent);
        CAN_wait_pc_state(pc_deb_control, DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- @3. Wait till DUT is not in Control field. Check that it is 
        --     transmitting error frame. Read Error code capture and check that
        --     it shows Form Error during Control field. Wait till the frame is
        --     transmitted.
        ------------------------------------------------------------------------
        info_m("Step 3: Check error frame is transmitted, Form error occurs!");
        
        CAN_wait_not_pc_state(pc_deb_control, DUT_NODE, chn);
        get_controller_status(status, DUT_NODE, chn);
        check_m(status.error_transmission,
            "Error frame transmitted as response to CAN FD frame!");
        
        CAN_read_error_code_capture(err_capt, DUT_NODE, chn);
        check_m(err_capt.err_type = can_err_form,
            "Error type: " & SW_error_type'image(err_capt.err_type));
        check_m(err_capt.err_pos = err_pos_ctrl,
            "Error in :" & SW_error_position'image(err_capt.err_pos));
        
        CAN_wait_bus_idle(DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- @4. Set Test node to Acknowledge forbidden mode. Transmitt frame by 
        --     DUT. Wait till it is sent, read Error code capture and check it
        --     is NOT equal to Form error (this is just to achieve change in 
        --     Error code capture).
        ------------------------------------------------------------------------
        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, TEST_NODE, chn);
        
        CAN_TX_frame.frame_format := NORMAL_CAN;
        CAN_send_frame(CAN_TX_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_TX_frame.frame_format := FD_CAN;
        
        CAN_wait_frame_sent(DUT_NODE, chn);
        
        CAN_read_error_code_capture(err_capt, DUT_NODE, chn);
        check_false_m(err_capt.err_type = can_err_form, "Error type changed!");
        
        CAN_wait_bus_idle(DUT_NODE, chn);
        
        ------------------------------------------------------------------------
        -- @5. Unset ACK forbidden in Test node. Send frame by DUT.
        ------------------------------------------------------------------------
        info_m("Step 4: Send frame by node with FD disabled");
        
        wait for 20000 ns;
        mode_2.acknowledge_forbidden := false;
        set_core_mode(mode_2, TEST_NODE, chn);
        
        CAN_TX_frame.frame_format := FD_CAN;
        CAN_send_frame(CAN_TX_frame, 1, DUT_NODE, chn, frame_sent);

        ------------------------------------------------------------------------
        -- @6. Wait until frame is sent and check that it is received OK in
        --     Test node. Make sure it CAN 2.0 frame.
        ------------------------------------------------------------------------
        info_m("Step 5: Check Test node receives CAN 2.0 frame!");
        
        CAN_wait_frame_sent(TEST_NODE, chn);
        CAN_read_frame(CAN_RX_frame, TEST_NODE, chn);
    
        check_m(CAN_RX_frame.frame_format = NORMAL_CAN, "CAN 2.0 frame received");
        check_m(CAN_RX_frame.dlc = CAN_TX_frame.dlc, "TX/RX DLC matching");
    
        CAN_wait_bus_idle(DUT_NODE, chn);
        
  end procedure;

end package body;