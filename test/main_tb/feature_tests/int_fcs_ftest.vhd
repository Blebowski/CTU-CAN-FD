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
--  Interrupt EPI feature test.
--
-- @Verifies:
--  @1. FCS Interrupt is set when unit turns from Error Active to Error Passive.
--  @2. FCS Interrupt is set when unit turns from Error Passive to Bus-off.
--  @3. FCS Interrupt is set when unit turns from Error Passive to Error Active.
--  @4. FCS Interrupt is set when unit turns from Bus-off to Error Active.
--  @5. FCS Interrupt is not set when it is masked.
--  @6. FCS Interrupt causes INT to go high when it is enabled.
--  @7. FCS Interrupt causes INT to go low when it is disabled.
--  @8. FCS Interrupt is cleared by write to INT_STATUS register.
--  @9. FCS Interrupt enable is manipulated properly by INT_ENA_SET and
--      INT_ENA_CLEAR.
-- @10. FCS Interrupt mask is manipulated properly by INT_MASK_SET and
--      INT_MASK_CLEAR.
--
-- @Test sequence:
--  @1. Unmask and enable FCS Interrupt, disable and mask all other interrupts on
--      DUT. Enable test mode (to manipulate with Error counter registers).
--      Keep ERP/EWL values default.
--  @2. Check unit is error active. Set TX Error counter to 128. Check unit is
--      Error passive. Check FCS Interrupt is present. Check that INT pin is high.
--  @3. Disable FCS Interrupt and check INT pin goes low. Enable FCS Interrupt
--      and check INT pin goes high.
--  @4. Clear FCS Interrupt. Check it is cleared.
--  @5. Set TX Error counter to 127. Check unit is Error active. Check that
--      FCS Interrupt is set. Check INT pin is high.
--  @6. Clear EPI Interrupt. Check FCS Interrupt is cleared.
--  @7. Set TX Error counter to 196. Check EPI Interrupt is set. Clear FCS
--      Interrupt.
--  @8. Set TX Error counter to 256. Check that unit is Bus-off. Check that FCS
--      Interrupt is set. Check INT pin is high.
--  @9. Clear FCS Interrupt. Check it is cleared. Check INT pin is low.
-- @10. Issue ERCRST command. Wait till unit turns Error Active. This does
--      not test proper duration after which unit turns Error active (128
--      ocurrences of 11 consecutive Recessive bits)!!! Meanwhile check that
--      FCS Interrupt is not set.
-- @11. When Unit turns error active, check that FCS Interrupt is set. Check
--      INT pin is high. Clear FCS Interrupt.
-- @12. Mask FCS Interrupt. Set RX Error counter to 128. Check Unit is Error
--      Passive. Check FCS Interrupt is not set. Check INT pin is low.
-- @13. Disable FCS Interrupt and check it was disabled. Enable FCS Interrupt and
--      check it was enabled.
-- @14. Mask FCS Interrupt and check it was masked. Un-mask FCS Interrupt and
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
use ctu_can_fd_tb.mem_bus_agent_pkg.all;

package int_fcs_ftest is
    procedure int_fcs_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;

package body int_fcs_ftest is
    procedure int_fcs_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :     SW_CAN_frame_type;

        variable int_mask           :     SW_interrupts := SW_interrupts_rst_val;
        variable int_ena            :     SW_interrupts := SW_interrupts_rst_val;
        variable int_stat           :     SW_interrupts := SW_interrupts_rst_val;
        variable command            :     SW_command := SW_command_rst_val;
        variable mode               :     SW_mode := SW_mode_rst_val;
        variable err_ctrs           :     SW_error_counters;
        variable status             :     SW_status;
        variable fault_state        :     SW_fault_state;
    begin

        -----------------------------------------------------------------------
        -- @1. Unmask and enable FCS Interrupt, disable and mask all other
        --     interrupts on DUT. Enable test mode (to manipulate with Error
        --     counter registers). Keep ERP/EWL values default.
        -----------------------------------------------------------------------
        info_m("Step 1: Setting FCS Interrupt");

        int_mask.fcs_changed_int := false;
        write_int_mask(int_mask, DUT_NODE, chn);

        int_ena.fcs_changed_int := true;
        write_int_enable(int_ena, DUT_NODE, chn);

        mode.test := true;
        set_core_mode(mode, DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @2. Check unit is Error Active. Set TX Error counter to 128. Check
        --     unit is Error passive. Check FCS Interrupt is present. Check that
        --     INT pin is high.
        -----------------------------------------------------------------------
        info_m("Step 2: FCS Interrupt - Error Active -> Error Pasive");

        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_error_active, "Unit Error Active");

        read_error_counters(err_ctrs, DUT_NODE, chn);
        err_ctrs.tx_counter := 128;
        set_error_counters(err_ctrs, DUT_NODE, chn);

        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_error_passive, "Unit Error Passive");

        read_int_status(int_stat, DUT_NODE, chn);
        check_m(int_stat.fcs_changed_int,
            "FCS Interrupt set on Error Active -> Error Passive");
        interrupt_agent_check_asserted(chn);
        
        -----------------------------------------------------------------------
        -- @3. Disable FCS Interrupt and check INT pin goes low. Enable FCS 
        --     Interrupt and check INT pin goes high.
        -----------------------------------------------------------------------
        info_m("Step 3: Check FCS Interrupt toggles INT pin");

        int_ena.fcs_changed_int := false;
        write_int_enable(int_ena, DUT_NODE, chn);
        wait for 10 ns;
        interrupt_agent_check_not_asserted(chn);

        int_ena.fcs_changed_int := true;
        write_int_enable(int_ena, DUT_NODE, chn);
        wait for 10 ns;
        interrupt_agent_check_asserted(chn);        

        -----------------------------------------------------------------------
        -- @4. Clear FCS Interrupt. Check it is cleared.
        -----------------------------------------------------------------------
        info_m("Step 4: Clear FCS Interrupt - Check it is cleared");

        int_stat.fcs_changed_int := true;
        clear_int_status(int_stat, DUT_NODE, chn);

        read_int_status(int_stat, DUT_NODE, chn);
        check_false_m(int_stat.fcs_changed_int,
            "FCS Interrupt still set after clear!");
        interrupt_agent_check_not_asserted(chn);             

        -----------------------------------------------------------------------
        -- @5. Set TX Error counter to 127. Check unit is Error active. Check 
        --     that FCS Interrupt is set. Check INT pin is high.
        -----------------------------------------------------------------------
        info_m("Step 5: FCS Interrupt - Error Passive -> Error Active!");

        err_ctrs.tx_counter := 127;
        set_error_counters(err_ctrs, DUT_NODE, chn);

        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_error_active, "Unit Error Active");

        read_int_status(int_stat, DUT_NODE, chn);
        check_m(int_stat.fcs_changed_int,
            "FCS Interrupt set on Error Passive -> Error Active");
        interrupt_agent_check_asserted(chn);

        -----------------------------------------------------------------------
        -- @6. Clear FCS Interrupt. Check it is cleared.
        -----------------------------------------------------------------------
        info_m("Step 6: Clear FCS Interrupt - Check it is cleared");

        int_stat.fcs_changed_int := true;
        clear_int_status(int_stat, DUT_NODE, chn);
        read_int_status(int_stat, DUT_NODE, chn);
        
        check_false_m(int_stat.fcs_changed_int,
            "FCS Interrupt still set after clear!");
        interrupt_agent_check_not_asserted(chn);

        -----------------------------------------------------------------------
        -- @7. Set TX Error counter to 196. Check EPI Interrupt is set. Clear 
        --    FCS Interrupt.
        -----------------------------------------------------------------------
        info_m("Step 7: FCS Interrupt - Moving to Error Passive!");
        err_ctrs.tx_counter := 196;
        set_error_counters(err_ctrs, DUT_NODE, chn);
        read_int_status(int_stat, DUT_NODE, chn);
        check_m(int_stat.fcs_changed_int, "FCS Interrupt set!");
        int_stat.fcs_changed_int := true;
        clear_int_status(int_stat, DUT_NODE, chn);
        read_int_status(int_stat, DUT_NODE, chn);
        check_false_m(int_stat.fcs_changed_int,
            "FCS Interrupt still set after clear!");
        
        -----------------------------------------------------------------------
        -- @8. Set TX Error counter to 256. Check that unit is Bus-off. Check 
        --     that FCS Interrupt is set. Check INT pin is high.
        -----------------------------------------------------------------------
        info_m("Step 8: FCS Interrupt - Error Passive -> Bus off!");

        err_ctrs.tx_counter := 256;
        set_error_counters(err_ctrs, DUT_NODE, chn);

        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_bus_off, "Unit Bus Off");

        read_int_status(int_stat, DUT_NODE, chn);
        check_m(int_stat.fcs_changed_int,
            "FCS Interrupt: Error Passive -> Bus Off");
        interrupt_agent_check_asserted(chn);

        -----------------------------------------------------------------------
        -- @9. Clear FCS Interrupt. Check it is cleared. Check INT pin is low.
        -----------------------------------------------------------------------
        info_m("Step 9: FCS Interrupt clear");

        int_stat.fcs_changed_int := true;
        clear_int_status(int_stat, DUT_NODE, chn);
        
        read_int_status(int_stat, DUT_NODE, chn);
        check_false_m(int_stat.fcs_changed_int,
            "FCS Interrupt still set after clear!");
        interrupt_agent_check_not_asserted(chn);
        
        -----------------------------------------------------------------------
        -- @10. Issue ERCRST command. Wait till unit turns Error Active. This
        --      does not test proper duration after which unit turns Error 
        --      active (128 ocurrences of 11 consecutive Recessive bits)!!!
        --      Meanwhile check that FCS Interrupt is not set.
        -----------------------------------------------------------------------
        info_m("Step 10: Issue ERCRST command, wait till Error Active");
        
        -- Here we have to wait a little bit since this command is processed
        -- in Sample point of Idle!
        wait for 2000 ns;
        command.err_ctrs_rst := true;
        give_controller_command(command, DUT_NODE, chn);
        command.err_ctrs_rst := false;
        
        get_fault_state(fault_state, DUT_NODE, chn);
        mem_bus_agent_disable_transaction_reporting(chn);
        while (fault_state = fc_bus_off) loop
            read_int_status(int_stat, DUT_NODE, chn);
            get_fault_state(fault_state, DUT_NODE, chn);
            if (fault_state = fc_bus_off) then
                check_false_m(int_stat.fcs_changed_int,
                    "FCS Interrupt not set in Bus-off.");
            end if;
            wait for 5000 ns;
        end loop;
        mem_bus_agent_enable_transaction_reporting(chn);
        
        -----------------------------------------------------------------------
        -- @11. When Unit turns Error Active, check that FCS Interrupt is set.
        --      Check INT pin is high. Clear FCS Interrupt.
        -----------------------------------------------------------------------
        info_m("Step 11: FCS Interrupt: Bus-off -> Error Active");

        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_error_active,
            "Reintegration -> Unit turned Error Active");

        read_int_status(int_stat, DUT_NODE, chn);
        check_m(int_stat.fcs_changed_int,
            "FCS Interrupt set: Bus-off -> Error Active");
        interrupt_agent_check_asserted(chn);

        int_stat.fcs_changed_int := true;
        clear_int_status(int_stat, DUT_NODE, chn);

        read_int_status(int_stat, DUT_NODE, chn);
        check_false_m(int_stat.fcs_changed_int,
            "FCS Interrupt still set after clear!");
            
        -----------------------------------------------------------------------
        -- @12. Mask FCS Interrupt. Set TX Error counter to 128. Check Unit is
        --      Error Passive. Check FCS Interrupt is not set. Check INT pin is
        --      low.
        -----------------------------------------------------------------------
        info_m("Step 12: FCS Interrupt not set when masked");

        int_mask.fcs_changed_int := true;
        write_int_mask(int_mask, DUT_NODE, chn);

        err_ctrs.tx_counter := 128;
        set_error_counters(err_ctrs, DUT_NODE, chn);

        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_error_passive, "Unit turned Error Passive");

        read_int_status(int_stat, DUT_NODE, chn);
        check_false_m(int_stat.fcs_changed_int,
            "FCS Interrupt not set when masked!");
        interrupt_agent_check_not_asserted(chn);

        -----------------------------------------------------------------------
        -- @13. Disable FCS Interrupt and check it was disabled. Enable FCS
        --      Interrupt and check it was enabled.
        -----------------------------------------------------------------------
        info_m("Step 13: Check FCS Interrupt enable works OK!");

        int_ena.fcs_changed_int := false;
        write_int_enable(int_ena, DUT_NODE, chn);
        int_ena.fcs_changed_int := true;

        read_int_enable(int_ena, DUT_NODE, chn);
        check_false_m(int_ena.fcs_changed_int, "FCS Interrupt enabled!");

        int_ena.fcs_changed_int := true;
        write_int_enable(int_ena, DUT_NODE, chn);
        int_ena.fcs_changed_int := false;

        read_int_enable(int_ena, DUT_NODE, chn);
        check_m(int_ena.fcs_changed_int, "FCS Interrupt disabled!");
        
        -----------------------------------------------------------------------
        -- @14. Mask FCS Interrupt and check it was masked. Un-mask FCS
        --      Interrupt and check it was un-masked.
        -----------------------------------------------------------------------
        info_m("Step 14: Check FCSI Interrupt mask works OK!");

        int_mask.fcs_changed_int := true;
        write_int_mask(int_mask, DUT_NODE, chn);
        int_mask.fcs_changed_int := false;

        read_int_mask(int_mask, DUT_NODE, chn);
        check_m(int_mask.fcs_changed_int, "FCS Interrupt masked!");

        int_mask.fcs_changed_int := false;
        write_int_mask(int_mask, DUT_NODE, chn);
        int_mask.fcs_changed_int := true;

        read_int_mask(int_mask, DUT_NODE, chn);
        check_false_m(int_mask.fcs_changed_int, "FCS Interrupt masked!");
        
        info_m("Finished FCS interrupt test");

    end procedure;
end package body;