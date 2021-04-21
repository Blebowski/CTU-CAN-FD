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
--  FDRF (Filter drop remote frame) feature test.  
--
-- @Verifies:
--  @1. When SETTINGS[FDRF] is set, CTU CAN FD drops filters out RTR frames.
--  @1. When SETTINGS[FDRF] is NOT set, CTU CAN FD accepts RTR frames.
--
-- @Test sequence:
--  @1. Configure frame filters in DUT. Enable SETTINGS[FDRF]. Configure
--      filter A to accept any identifier (mask = all zeroes).4
--  @2. Send RTR frame by Test node. Wait until frame is sent and check that there
--      is no frame received by DUT.
--  @3. Disable SETTINGS[FDRF] in DUT.
--  @4. Send RTR frame by Test node. Wait until frame is sent and check that frame
--      is received by DUT.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    31.10.2020   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package mode_fdrf_ftest is
    procedure mode_fdrf_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body mode_fdrf_ftest is
    procedure mode_fdrf_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_TX_frame       :       SW_CAN_frame_type;
        variable CAN_RX_frame       :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        
        variable mode_1             :       SW_mode := SW_mode_rst_val;
        
        variable rx_buf_state       :       SW_RX_Buffer_info;
        variable frames_equal       :       boolean := false;
        variable filt_A_cfg         :       SW_CAN_mask_filter_config;
        variable filt_B_C_cfg       :       SW_CAN_mask_filter_config;
        variable filter_range_cfg   :       SW_CAN_range_filter_config;
    begin

        ------------------------------------------------------------------------
        -- @1. Configure frame filters in DUT. Enable SETTINGS[FDRF].
        --     Configure filter A to accept any identifier (mask = all zeroes).
        ------------------------------------------------------------------------
        info_m("Step 1: Configure frame filters");
        
        mode_1.acceptance_filter := true;
        mode_1.fdrf := true;
        set_core_mode(mode_1, DUT_NODE, chn);

        -- Filter A (set mask to 0 - accept all frames)
        filt_A_cfg.ID_value := 0;
        filt_A_cfg.ID_mask := 0;
        filt_A_cfg.ident_type := BASE;
        filt_A_cfg.acc_CAN_2_0 := true;
        filt_A_cfg.acc_CAN_FD := true;
        CAN_set_mask_filter(filter_A, filt_A_cfg, DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- @2. Send RTR frame by Test node. Wait until frame is sent and check 
        --     that there is no frame received by DUT.
        ------------------------------------------------------------------------
        info_m("Step 2: Check that RTR frame is filtered when FDRF=1.");
        
        CAN_generate_frame(CAN_TX_frame);
        CAN_TX_frame.ident_type := BASE;
        CAN_TX_frame.frame_format := NORMAL_CAN;
        CAN_TX_frame.identifier := CAN_TX_frame.identifier mod 2048;
        CAN_TX_frame.rtr := RTR_FRAME;
        
        CAN_send_frame(CAN_TX_frame, 1, TEST_NODE, chn, frame_sent);
        CAN_wait_frame_sent(DUT_NODE, chn);
        
        get_rx_buf_state(rx_buf_state, DUT_NODE, chn);
        check_m(rx_buf_state.rx_frame_count = 0, "Frame filtered out!");

        ------------------------------------------------------------------------
        -- @3. Disable SETTINGS[FDRF] in DUT.
        ------------------------------------------------------------------------
        info_m("Step 3: Disable SETTINGS[FDRF]");
        
        mode_1.fdrf := false;
        set_core_mode(mode_1, DUT_NODE, chn);
        
        ------------------------------------------------------------------------
        -- @4. Send RTR frame by Test node. Wait until frame is sent and check 
        --     that frame is received by DUT.
        ------------------------------------------------------------------------
        info_m("Step 4: Check that RTR frame is NOT filtered when FDRF=0.");
        
        CAN_generate_frame(CAN_TX_frame);
        CAN_TX_frame.ident_type := BASE;
        CAN_TX_frame.identifier := CAN_TX_frame.identifier mod 2048;
        CAN_TX_frame.rtr := RTR_FRAME;
        
        CAN_send_frame(CAN_TX_frame, 1, TEST_NODE, chn, frame_sent);
        CAN_wait_frame_sent(DUT_NODE, chn);
        
        get_rx_buf_state(rx_buf_state, DUT_NODE, chn);
        check_m(rx_buf_state.rx_frame_count = 1, "Frame NOT filtered out!");
        
  end procedure;

end package body;