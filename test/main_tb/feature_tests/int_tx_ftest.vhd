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
--  Interrupt TX feature test.
--
-- @Verifies:
--  @1. TX Interrupt is set when frame is transmitted OK in EOF field.
--      Note: This test is not testing that TX frame is validated in correct
--            moment!
--  @2. TX Interrupt causes INT to go high when it is enabled.
--  @3. TX Interrupt causes INT to go low when it is disabled.
--  @4. TX Interrupt is cleared by write to INT_STATUS register.
--  @5. TX Interrupt is not set when Error frame occurs.
--  @6. TX Interrupt is not set when it is masked.
--  @7. TX Interrupt enable is manipulated properly by INT_ENA_SET and
--      INT_ENA_CLEAR.
--  @8. TX Interrupt mask is manipulated properly by INT_MASk_SET and
--      INT_MASK_CLEAR.
--  @9. TX Interrupt is not set when frame was transmitted.

-- @Test sequence:
--  @1. Unmask and enable TX Interrupt, disable and mask all other interrupts on
--      DUT.
--  @2. Set Retransmitt limit to 0 on DUT (One shot-mode). Enable Retransmitt
--      limitations on DUT. Send frame by DUT.
--  @3. Monitor DUT frame, check that in the beginning of EOF, Interrupt is
--      Not set, after EOF it is set.
--  @4. Check that INT pin is high. Disable TX Interrupt and check that INT pin
--      goes low. Enable Interrupt and check it goes high again.
--  @5. Clear TX Interrupt, check it is cleared and INT pin goes low.
--  @6. Send Frame by DUT. Force bus level during ACK to recessive, check that
--      error frame is transmitted by both nodes. Wait till bus is idle, check
--      that no TX Interrupt was set, INT pin is low.
--  @7. Mask TX Interrupt. Send frame by DUT. Check that after frame TX
--      Interrupt was not set. Check that INT pin is low.
--  @8. Unmask TX Interrupt. Send frame by Test node. Check that after frame TX
--      Interrupt was set. Check INT pin is high.
--  @9. Disable TX Interrupt, check that TX interrupt was disabled.
-- @10. Enable TX Interrupt, check that TX interrupt was enabled.
-- @11. Mask TX Interrupt, check TX Interrupt is Masked.
-- @12. Un-Mask TX Interrupt, check TX Interrupt is Un-masked.
-- @13. Send frame by DUT. Check that after frame TX Interrupt is not set and
--      Interrupt pin remains low.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    22.6.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.interrupt_agent_pkg.all;

package int_tx_ftest is
    procedure int_tx_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;

package body int_tx_ftest is
    procedure int_tx_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :     SW_CAN_frame_type;
        variable CAN_frame_rx       :     SW_CAN_frame_type;
        variable frame_sent         :     boolean := false;
        variable frames_equal       :     boolean := false;

        variable int_mask           :     SW_interrupts := SW_interrupts_rst_val;
        variable int_ena            :     SW_interrupts := SW_interrupts_rst_val;
        variable int_stat           :     SW_interrupts := SW_interrupts_rst_val;
        variable mode               :     SW_mode := SW_mode_rst_val;
        variable pc_dbg             :     SW_PC_Debug;  
    begin

        -----------------------------------------------------------------------
        -- @1. Unmask and enable TX Interrupt, disable and mask all other 
        --     interrupts on DUT.
        -----------------------------------------------------------------------
        info_m("Step 1: Setting TX Interrupt");

        int_mask.transmitt_int := false;
        write_int_mask(int_mask, DUT_NODE, chn);

        int_ena.transmitt_int := true;
        write_int_enable(int_ena, DUT_NODE, chn);
        
        -----------------------------------------------------------------------
        --  @2. Set Retransmitt limit to 0 on DUT (One shot-mode). Enable 
        --      Retransmitt limitations on DUT. Send frame by DUT.
        -----------------------------------------------------------------------
        info_m("Step 2: Sending frame");

        CAN_enable_retr_limit(true, 0, DUT_NODE, chn);
        CAN_generate_frame(CAN_frame);
        CAN_send_frame(CAN_frame, 1, DUT_NODE, chn, frame_sent);
        
        -----------------------------------------------------------------------
        --  @3. Monitor DUT frame, check that in the beginning of EOF, 
        --      Interrupt is Not set, after EOF it is set.
        -----------------------------------------------------------------------  
        info_m("Step 3: Check TX Interrupt is set in EOF!");
        
        CAN_wait_pc_state(pc_deb_eof, DUT_NODE, chn);
        read_int_status(int_stat, DUT_NODE, chn);
        check_false_m(int_stat.transmitt_int,
            "TX Interrupt not set in beginning of EOF");
        
        CAN_wait_not_pc_state(pc_deb_eof, DUT_NODE, chn);
        read_int_status(int_stat, DUT_NODE, chn);
        check_m(int_stat.transmitt_int, "TX Interrupt set at the end of EOF");
        
        -- Wait till bus is idle, read-out received frame so that there is
        -- nothing in RX Buffer of Test node.
        CAN_wait_bus_idle(TEST_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_read_frame(CAN_frame_rx, TEST_NODE, chn);
        
        CAN_compare_frames(CAN_frame, CAN_frame_rx, false, frames_equal);
        check_m(frames_equal, "TX, RX frames should be equal!");
        
        -----------------------------------------------------------------------
        --  @4. Check that INT pin is high. Disable TX Interrupt and check that
        --      INT pin goes low. Enable Interrupt and check it goes high again.
        -----------------------------------------------------------------------
        info_m("Step 4: Check INT pin toggles");

        interrupt_agent_check_asserted(chn);
        int_ena.transmitt_int := false;
        write_int_enable(int_ena, DUT_NODE, chn);
        wait for 10 ns;
        
        interrupt_agent_check_not_asserted(chn);
        
        int_ena.transmitt_int := true;
        write_int_enable(int_ena, DUT_NODE, chn);
        wait for 10 ns;
        
        interrupt_agent_check_asserted(chn);

        -----------------------------------------------------------------------
        --  @5. Clear TX Interrupt, check it is cleared and INT pin goes low.
        -----------------------------------------------------------------------
        info_m("Step 4: Clear TX Interrupt, Check INT pin toggles");

        int_stat.transmitt_int := true;
        clear_int_status(int_stat, DUT_NODE, chn);
        read_int_status(int_stat, DUT_NODE, chn);
        
        check_false_m(int_mask.transmitt_int, "TX Interrupt status should be 0!");
        interrupt_agent_check_not_asserted(chn);
        
        -----------------------------------------------------------------------
        --  @6. Send Frame by DUT. Force bus level during ACK to recessive, 
        --      check that error frame is transmitted by both nodes. Wait till
        --      bus is idle, check.
        -----------------------------------------------------------------------
        info_m("Step 6: Check TX Interrupt is not set upon Error Frame!");

        CAN_generate_frame(CAN_frame);
        -- CAN 2.0 frame is needed! In FD frame, ACK can be prolonged so it
        -- is not enough to force it recessive for one bit!!!
        CAN_frame.frame_format := NORMAL_CAN; 
        CAN_send_frame(CAN_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_pc_state(pc_deb_ack, DUT_NODE, chn);
        
        force_bus_level(RECESSIVE, chn);
        CAN_wait_not_pc_state(pc_deb_ack, DUT_NODE, chn);
        CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
        release_bus_level(chn);
        
        CAN_wait_bus_idle(TEST_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);

        read_int_status(int_stat, DUT_NODE, chn);
        
        check_false_m(int_stat.transmitt_int, "TX Interrupt status should be 0!");
        interrupt_agent_check_not_asserted(chn);

        -----------------------------------------------------------------------
        --  @7. Mask TX Interrupt. Send frame by DUT. Check that after frame
        --     TX Interrupt was not set. Check that INT pin is low.
        -----------------------------------------------------------------------
        info_m("Step 7: Check Masked TX Interrupt is not captured");

        int_mask.transmitt_int := true;
        int_ena.transmitt_int := true;
        write_int_mask(int_mask, DUT_NODE, chn);

        CAN_generate_frame(CAN_frame);
        CAN_send_frame(CAN_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_frame_sent(DUT_NODE, chn);

        CAN_read_frame(CAN_frame_rx, TEST_NODE, chn);
        CAN_compare_frames(CAN_frame, CAN_frame_rx, false, frames_equal);
        check_m(frames_equal, "TX, RX frames should be equal!");

        read_int_status(int_stat, DUT_NODE, chn);
        check_false_m(int_stat.transmitt_int, "TX Interrupt status should be 0!");
        interrupt_agent_check_not_asserted(chn);

        -----------------------------------------------------------------------
        --  @8. Unmask TX Interrupt. Send frame by DUT. Check that after 
        --     frame TX Interrupt was set. Check INT pin is high.
        -----------------------------------------------------------------------
        info_m("Step 8: Check Un-Masked TX Interrupt is captured");

        int_mask.transmitt_int := false;
        int_ena.transmitt_int := true;
        write_int_mask(int_mask, DUT_NODE, chn);

        CAN_generate_frame(CAN_frame);
        CAN_send_frame(CAN_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_frame_sent(DUT_NODE, chn);

        CAN_read_frame(CAN_frame_rx, TEST_NODE, chn);
        CAN_compare_frames(CAN_frame, CAN_frame_rx, false, frames_equal);
        check_m(frames_equal, "TX, RX frames should be equal!");

        read_int_status(int_stat, DUT_NODE, chn);        
        check_m(int_stat.transmitt_int, "TX Interrupt status should be 1!");
        interrupt_agent_check_asserted(chn);
        
        -----------------------------------------------------------------------
        --  @9. Disable TX Interrupt, check that TX interrupt was disabled.
        -----------------------------------------------------------------------
        info_m("Step 9: Check TX Interrupt Enable Set");

        int_ena.transmitt_int := false;
        write_int_enable(int_ena, DUT_NODE, chn);
        int_ena.transmitt_int := true;

        read_int_enable(int_ena, DUT_NODE, chn);
        check_false_m(int_ena.transmitt_int, "TX Interrupt should be disabled!");
        
        -----------------------------------------------------------------------
        -- @10. Enable TX Interrupt, check that TX interrupt was enabled.
        -----------------------------------------------------------------------
        info_m("Step 10: Check TX Interrupt Enable Clear");

        int_ena.transmitt_int := true;
        write_int_enable(int_ena, DUT_NODE, chn);
        int_ena.transmitt_int := false;
        read_int_enable(int_ena, DUT_NODE, chn);
        
        check_m(int_ena.transmitt_int, "TX Interrupt should be enabled!");        
        
        -----------------------------------------------------------------------
        -- @11. Mask TX Interrupt, check TX Interrupt is Masked.
        -----------------------------------------------------------------------
        info_m("Step 11: Check TX Interrupt Mask Set");

        int_mask.transmitt_int := true;
        write_int_mask(int_mask, DUT_NODE, chn);
        int_mask.transmitt_int := false;
        read_int_mask(int_mask, DUT_NODE, chn);
        
        check_m(int_ena.transmitt_int, "TX Interrupt should be masked!");        
        
        -----------------------------------------------------------------------
        -- @12. Un-Mask TX Interrupt, check TX Interrupt is Un-masked.
        -----------------------------------------------------------------------
        info_m("Step 12: Check TX Interrupt Mask Clear");

        int_mask.transmitt_int := false;
        write_int_mask(int_mask, DUT_NODE, chn);
        int_mask.transmitt_int := true;
        read_int_mask(int_mask, DUT_NODE, chn);
        
        check_false_m(int_mask.transmitt_int, "TX Interrupt should be unmasked!");
        
        -----------------------------------------------------------------------
        -- @13. Send frame by Test node. Check that after frame TX Interrupt is 
        --      not set and Interrupt pin remains low.
        -----------------------------------------------------------------------
        info_m("Step 13: Check RX does not cause TX Interrupt to be captured!");
        int_stat.transmitt_int := true;
        clear_int_status(int_stat, DUT_NODE, chn);
        
        int_mask.transmitt_int := false;
        write_int_mask(int_mask, DUT_NODE, chn);
        
        int_ena.transmitt_int := true;
        write_int_enable(int_ena, DUT_NODE, chn);
        
        CAN_generate_frame(CAN_frame);
        CAN_send_frame(CAN_frame, 1, TEST_NODE, chn, frame_sent);
        CAN_wait_frame_sent(TEST_NODE, chn);
        
        CAN_read_frame(CAN_frame_rx, DUT_NODE, chn);
        CAN_compare_frames(CAN_frame, CAN_frame_rx, false, frames_equal);
        
        read_int_status(int_stat, DUT_NODE, chn);
        check_m(frames_equal, "TX, RX frames should be equal!");
        check_false_m(int_stat.transmitt_int, "TX Interrupt should not be set after RX!");
        interrupt_agent_check_not_asserted(chn);
        
        info_m("Finished TX interrupt test");

    end procedure;
end package body;