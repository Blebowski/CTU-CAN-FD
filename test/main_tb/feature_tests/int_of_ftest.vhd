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
--  Interrupt DOI feature test.
--
-- @Verifies:
--  @1. OF Interrupt is set when node transmits overload frame.
--  @2. OF Interrupt is not set when it is masked.
--  @3. OF Interrupt causes INT to go high when it is enabled.
--  @4. OF Interrupt causes INT to go low when it is disabled.
--  @5. OF Interrupt is cleared by write to INT_STATUS register.
--  @6. OF Interrupt enable is manipulated properly by INT_ENA_SET and
--      INT_ENA_CLEAR.
--  @7. OF Interrupt mask is manipulated properly by INT_MASK_SET and
--      INT_MASK_CLEAR.
--
-- @Test sequence:
--  @1. Unmask and enable OF Interrupt, disable and mask all other interrupts on
--      DUT.
--  @2. Send frame by DUT, wait till intermission and check OF interrupt is not
--      set. Force bus to dominant, wait one bit, and check that DUT transmitts
--      overload frame. Check that OF interrupt is set and that INT pin is high
--  @3. Disable OF Interrupt and check INT pin goes low. Enable OF Interrupt
--      and check INT pin goes high.
--  @4. Clear OF Interrupt and check it has been cleared and that INT pin OF low.
--  @5. Mask OF Interrupt. Send again frame by DUT. Wait until Intermission and
--      check that OF is not set. Force bus low, wait bit and check that OF is
--      still not set (interrupt is masked).
--  @6. Disable OF Interrupt and check it was disabled. Enable OF Interrupt and
--      check it was enabled.
--  @7. Mask OF Interrupt and check it was masked. Un-mask OF Interrupt and
--      check it was un-masked.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    1.7.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.interrupt_agent_pkg.all;

package int_of_ftest is
    procedure int_of_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;

package body int_of_ftest is
    procedure int_of_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :     SW_CAN_frame_type;
        variable frame_sent         :     boolean := false;

        variable int_mask           :     SW_interrupts := SW_interrupts_rst_val;
        variable int_ena            :     SW_interrupts := SW_interrupts_rst_val;
        variable int_stat           :     SW_interrupts := SW_interrupts_rst_val;
        variable command            :     SW_command := SW_command_rst_val;
        variable buf_info           :     SW_RX_Buffer_info;
        variable status             :     SW_status;
        
        variable mode_1             :     SW_mode := SW_mode_rst_val;
        variable mode_2             :     SW_mode := SW_mode_rst_val;
    begin

        -----------------------------------------------------------------------
        -- @1. Unmask and enable OF Interrupt, disable and mask all other
        --     interrupts on DUT.
        -----------------------------------------------------------------------
        info_m("Step 1");
        
        int_mask.overload_frame := false;
        int_ena.overload_frame := true;
        write_int_mask(int_mask, DUT_NODE, chn);
        write_int_enable(int_ena, DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @2. Send frame by DUT, wait till intermission and check OF interrupt
        --     is not set. Force bus to dominant, wait one bit, and check that
        --     DUT transmitts overload frame. Check that OF interrupt is set
        --     and that INT pin is high.
        -----------------------------------------------------------------------
        info_m("Step 2");

        CAN_generate_frame(CAN_frame);
        CAN_send_frame(CAN_frame, 1, DUT_NODE, chn, frame_sent);

        CAN_wait_pc_state(pc_deb_intermission, DUT_NODE, chn);
        read_int_status(int_stat, DUT_NODE, chn);
        check_false_m(int_stat.overload_frame,
                      "OF Interrupt not set before overload frame");

        force_bus_level(DOMINANT, chn);
        CAN_wait_sample_point(DUT_NODE, chn);
        wait for 20 ns;
        
        read_int_status(int_stat, DUT_NODE, chn);
        check_m(int_stat.overload_frame,
                "OF Interrupt set by overload frame");

        release_bus_level(chn);
        
        CAN_wait_bus_idle(TEST_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);
        
        -----------------------------------------------------------------------
        -- @3. Disable OF Interrupt and check INT pin goes low. Enable OF
        --     Interrupt and check INT pin goes high.
        -----------------------------------------------------------------------
        info_m("Step 3");
        
        int_ena.overload_frame := false;
        write_int_enable(int_ena, DUT_NODE, chn);
        wait for 10 ns;
        interrupt_agent_check_not_asserted(chn);
        
        int_ena.overload_frame := true;
        write_int_enable(int_ena, DUT_NODE, chn);
        wait for 10 ns;
        interrupt_agent_check_asserted(chn);
        
        -----------------------------------------------------------------------
        -- @4. Clear OF Interrupt and check it has been cleared and that INT
        --     pin is low.
        -----------------------------------------------------------------------
        info_m("Step 4");
        
        int_stat.overload_frame := true;
        clear_int_status(int_stat, DUT_NODE, chn);
        read_int_status(int_stat, DUT_NODE, chn);

        check_false_m(int_stat.overload_frame, "OF Interrupt cleared!");
        interrupt_agent_check_not_asserted(chn);  
        
        -----------------------------------------------------------------------
        -- @5. Mask OF Interrupt. Send again frame by DUT. Wait until ACK and
        --     check OF interrupt is not set. Wait until error frame and check
        --     OF interrupt is still not set.
        -----------------------------------------------------------------------
        info_m("Step 5");

        int_mask.overload_frame := true;
        write_int_mask(int_mask, DUT_NODE, chn);

        CAN_send_frame(CAN_frame, 1, DUT_NODE, chn, frame_sent);

        CAN_wait_pc_state(pc_deb_ack, DUT_NODE, chn);
        read_int_status(int_stat, DUT_NODE, chn);
        check_false_m(int_stat.overload_frame,
                      "OF Interrupt not set before overload frame");

        force_bus_level(DOMINANT, chn);
        CAN_wait_sample_point(DUT_NODE, chn);
        wait for 20 ns;
        
        read_int_status(int_stat, DUT_NODE, chn);
        check_false_m(int_stat.overload_frame,
                      "OF Interrupt not set by overload frame when masked");

        release_bus_level(chn);
        
        CAN_wait_bus_idle(TEST_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @6. Disable OF Interrupt and check it was disabled. Enable OF 
        --     Interrupt and check it was enabled.
        -----------------------------------------------------------------------
        info_m("Step 6");

        int_ena.overload_frame := false;
        write_int_enable(int_ena, DUT_NODE, chn);
        int_ena.overload_frame := true;

        read_int_enable(int_ena, DUT_NODE, chn);
        check_false_m(int_ena.overload_frame, "OF Interrupt Disabled!");

        int_ena.overload_frame := true;
        write_int_enable(int_ena, DUT_NODE, chn);
        int_ena.overload_frame := false;
        read_int_enable(int_ena, DUT_NODE, chn);
        check_m(int_ena.overload_frame, "OF Interrupt Enabled!");
        
        -----------------------------------------------------------------------
        -- @7. Mask OF Interrupt and check it was masked. Un-mask OF Interrupt
        --     and check it was un-masked.
        -----------------------------------------------------------------------
        info_m("Step 7");
        
        int_mask.overload_frame := true;
        write_int_mask(int_mask, DUT_NODE, chn);
        int_mask.overload_frame := false;
        read_int_mask(int_mask, DUT_NODE, chn);
        check_m(int_mask.overload_frame, "OF Interrupt masked!");

        int_mask.overload_frame := false;
        write_int_mask(int_mask, DUT_NODE, chn);
        int_mask.overload_frame := true;
        read_int_mask(int_mask, DUT_NODE, chn);
        check_false_m(int_mask.overload_frame, "OF Interrupt masked!");

        info_m("Finished OF interrupt test");

    end procedure;
end package body;