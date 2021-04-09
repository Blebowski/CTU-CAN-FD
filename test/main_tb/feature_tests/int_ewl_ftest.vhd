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
--  Interrupt EWL feature test.
--
-- @Verifies:
--  @1. EWL Interrupt is set when TX Error counter is more or equal to EWL.
--  @2. EWL Interrupt is set when RX Error counter more or equal to EWL.
--  @3. EWL Interrupt is not set when either TX or RX Interrupt more or equal to
--      EWL, and EWL Interrupt is masked.
--  @4. EWL Interrupt causes INT to go high when it is enabled.
--  @5. EWL Interrupt causes INT to go low when it is disabled.
--  @6. EWL Interrupt is cleared by write to INT_STATUS register.
--  @7. EWL Interrupt enable is manipulated properly by INT_ENA_SET and
--      INT_ENA_CLEAR.
--  @8. EWL Interrupt mask is manipulated properly by INT_MASK_SET and
--      INT_MASK_CLEAR.
--  @9. EWL Interrupt is set when RX Error counter becomes less than or equal
--      to EWL.
-- @10. EWL Interrupt is set when TX Error counter becomes less than or equal
--      to EWL.
--
-- @Test sequence:
--  @1. Unmask and enable EWL Interrupt, disable and mask all other interrupts on
--      DUT. Turn on Test mode (to manipulate error counters via CTR_PRES).
--      Leave default value of EWL (96).
--  @2. Set TX Error counter to 96. Check that EWL Interrupt was set. Check that
--      EWL Interrupt is set and INT pin is high.
--  @3. Disable EWL Interrupt and check INT pin goes low. Enable EWL Interrupt
--      and check INT pin goes high. 
--  @4. Clear EWL Interrupt and check it not set. Set TX Error counter to
--      95. Check EWL is set. Clear EWL Interrupt. Check EWL interrupt is cleared.
--  @5. Set RX Error counter to 96. Check that EWL Interrupt was set. Check that
--      INT pin goes high. 
--  @6. Disable EWL Interrupt and check INT pin goes low. Enable EWL Interrupt
--      and check INT pin goes high again. 
--  @7. Clear EWL Interrupt. Check that EWL Interrupt is not set. Set RX Error
--      counter to 95. Check EWL is set. Check INT pin goes high and Interrupt
--      is set. Clear the interrupt and check.
--  @8. Mask EWL Interrupt. Set TX Error counter to 96. Check that EWL interrupt
--      was not set.
--  @9. Disable EWL Interrupt and check it was disabled. Enable EWL Interrupt and
--      check it was enabled.
-- @10. Mask EWL Interrupt and check it was masked. Un-mask EWL Interrupt and
--      check it was un-masked.
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

package int_ewl_ftest is
    procedure int_ewl_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;

package body int_ewl_ftest is
    procedure int_ewl_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :     SW_CAN_frame_type;
        variable frame_sent         :     boolean := false;

        variable int_mask           :     SW_interrupts := SW_interrupts_rst_val;
        variable int_ena            :     SW_interrupts := SW_interrupts_rst_val;
        variable int_stat           :     SW_interrupts := SW_interrupts_rst_val;
        variable mode               :     SW_mode := SW_mode_rst_val;
        variable buf_info           :     SW_RX_Buffer_info;
        variable err_ctrs           :     SW_error_counters;
    begin

        -----------------------------------------------------------------------
        -- @1. Unmask and enable EWL Interrupt, disable and mask all other 
        --     interrupts on DUT. Turn on Test mode (to manipulate error
        --     counters via CTR_PRES). Leave default value of EWL (96).
        -----------------------------------------------------------------------
        info_m("Step 1: Setting EWL Interrupt");

        int_mask.error_warning_int := false;
        write_int_mask(int_mask, DUT_NODE, chn);

        int_ena.error_warning_int := true;
        write_int_enable(int_ena, DUT_NODE, chn);

        mode.test := true;
        set_core_mode(mode, DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @2. Set TX Error counter to 96. Check that EWL Interrupt was set. 
        --     Check that EWL Interrupt is set and INT pin is high.
        -----------------------------------------------------------------------
        info_m("Step 2: Check EWL from TX Error counter");

        read_error_counters(err_ctrs, DUT_NODE, chn);
        err_ctrs.tx_counter := 96;
        set_error_counters(err_ctrs, DUT_NODE, chn);

        read_int_status(int_stat, DUT_NODE, chn);
        check_m(int_stat.error_warning_int,
                "EWL Interrupt set when RX Error counter >= EWL!");
        interrupt_agent_check_asserted(chn);

        -----------------------------------------------------------------------
        -- @3. Disable EWL Interrupt and check INT pin goes low. Enable EWL 
        --     Interrupt and check INT pin goes high. 
        -----------------------------------------------------------------------
        info_m("Step 3: Check EWL from TX counter toggles INT pin.");

        int_ena.error_warning_int := false;
        write_int_enable(int_ena, DUT_NODE, chn);
        wait for 10 ns;
        interrupt_agent_check_not_asserted(chn);

        int_ena.error_warning_int := true;
        write_int_enable(int_ena, DUT_NODE, chn);
        wait for 10 ns;
        interrupt_agent_check_asserted(chn);

        -----------------------------------------------------------------------
        -- @4. Clear EWL Interrupt and check it not set. Set TX Error
        --     counter to 95. Check EWL is set. Clear EWL Interrupt. Check EWL
        --     interrupt is cleared.
        -----------------------------------------------------------------------
        info_m("Step 4: Check EWL is set again.");

        int_stat.error_warning_int := true;
        clear_int_status(int_stat, DUT_NODE, chn);

        read_int_status(int_stat, DUT_NODE, chn);
        check_false_m(int_stat.error_warning_int,
            "EWL Interrupt set again when RX Error counter >= EWL!");

        err_ctrs.tx_counter := 95;
        set_error_counters(err_ctrs, DUT_NODE, chn);

        read_int_status(int_stat, DUT_NODE, chn);
        check_m(int_stat.error_warning_int,
            "EWL Interrupt set when TX Error counter < EWL!");
        interrupt_agent_check_asserted(chn);
        
        int_stat.error_warning_int := true;
        clear_int_status(int_stat, DUT_NODE, chn);

        read_int_status(int_stat, DUT_NODE, chn);
        check_false_m(int_stat.error_warning_int,
            "EWL Interrupt cleared!");

        -----------------------------------------------------------------------
        -- @5. Set RX Error counter to 96. Check that EWL Interrupt was set. 
        --     Check that INT pin goes high. 
        -----------------------------------------------------------------------
        info_m("Step 5: Check EWL from RX Error counter");

        err_ctrs.rx_counter := 96;
        set_error_counters(err_ctrs, DUT_NODE, chn);

        read_int_status(int_stat, DUT_NODE, chn);
        check_m(int_stat.error_warning_int, "EWL Interrupt set when TX Error counter >= EWL!");
        interrupt_agent_check_asserted(chn);

        -----------------------------------------------------------------------
        -- @6. Disable EWL Interrupt and check INT pin goes low. Enable EWL 
        --     Interrupt and check INT pin goes high again. 
        -----------------------------------------------------------------------
        info_m("Step 6: Check EWL from RX counter toggles INT pin.");

        int_ena.error_warning_int := false;
        write_int_enable(int_ena, DUT_NODE, chn);
        wait for 10 ns;
        interrupt_agent_check_not_asserted(chn);

        int_ena.error_warning_int := true;
        write_int_enable(int_ena, DUT_NODE, chn);
        wait for 10 ns;
        interrupt_agent_check_asserted(chn);

        -----------------------------------------------------------------------
        -- @7. Clear EWL Interrupt. Check that EWL Interrupt is not set. Set RX
        --     Error counter to 95. Check EWL is set. Check INT pin goes high
        --     and Interrupt is set. Clear the interrupt and check.
        -----------------------------------------------------------------------
        info_m("Step 7: Check EWL from RX counter is set again.");

        int_stat.error_warning_int := true;
        clear_int_status(int_stat, DUT_NODE, chn);

        read_int_status(int_stat, DUT_NODE, chn);
        check_false_m(int_stat.error_warning_int,
            "EWL Interrupt not set again when TX Error counter >= EWL!");
        
        err_ctrs.rx_counter := 95;
        set_error_counters(err_ctrs, DUT_NODE, chn);

        read_int_status(int_stat, DUT_NODE, chn);
        check_m(int_stat.error_warning_int,
            "EWL Interrupt set when RX Error counter < EWL!");
        interrupt_agent_check_asserted(chn);

        int_stat.error_warning_int := true;
        clear_int_status(int_stat, DUT_NODE, chn);
        
        read_int_status(int_stat, DUT_NODE, chn);
        check_false_m(int_stat.error_warning_int, "EWL Interrupt cleared!");

        -----------------------------------------------------------------------
        -- @8. Mask EWL Interrupt. Set TX Error counter to 96. Check that EWL 
        --     interrupt was not set.
        -----------------------------------------------------------------------
        info_m("Step 8: Check EWL is not set when masked!");

        int_mask.error_warning_int := true;
        write_int_mask(int_mask, DUT_NODE, chn);

        err_ctrs.tx_counter := 96;
        set_error_counters(err_ctrs, DUT_NODE, chn);

        read_int_status(int_stat, DUT_NODE, chn);
        check_false_m(int_stat.error_warning_int, "EWL Interrupt not set when masked!");
        interrupt_agent_check_not_asserted(chn);

        -----------------------------------------------------------------------
        -- @9. Disable EWL Interrupt and check it was disabled. Enable EWL 
        --     Interrupt and check it was enabled.
        -----------------------------------------------------------------------
        info_m("Step 9: Check EWL Interrupt enable works OK!");

        int_ena.error_warning_int := false;
        write_int_enable(int_ena, DUT_NODE, chn);

        int_ena.error_warning_int := true;
        read_int_enable(int_ena, DUT_NODE, chn);

        check_false_m(int_ena.error_warning_int, "EWL Interrupt enabled!");

        int_ena.error_warning_int := true;
        write_int_enable(int_ena, DUT_NODE, chn);
        int_ena.error_warning_int := false;

        read_int_enable(int_ena, DUT_NODE, chn);
        check_m(int_ena.error_warning_int, "EWL Interrupt disabled!");

        -----------------------------------------------------------------------
        -- @10. Mask EWL Interrupt and check it was masked. Un-mask EWL 
        --      Interrupt and check it was un-masked.
        -----------------------------------------------------------------------
        info_m("Step 10: Check EWL Interrupt mask works OK!");

        int_mask.error_warning_int := true;
        write_int_mask(int_mask, DUT_NODE, chn);
        int_mask.error_warning_int := false;

        read_int_mask(int_mask, DUT_NODE, chn);
        check_m(int_mask.error_warning_int, "EWL Interrupt masked!");

        int_mask.error_warning_int := false;
        write_int_mask(int_mask, DUT_NODE, chn);
        int_mask.error_warning_int := true;

        read_int_mask(int_mask, DUT_NODE, chn);
        check_false_m(int_mask.error_warning_int, "EWL Interrupt masked!");

        info_m("Finished EWL interrupt test");

    end procedure;
end package body;