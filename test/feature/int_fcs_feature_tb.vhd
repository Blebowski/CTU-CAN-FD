--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018
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
--      Node 1. Enable test mode (to manipulate with Error counter registers).
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
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package int_fcs_feature is
    procedure int_fcs_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;

package body int_fcs_feature is
    procedure int_fcs_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable CAN_frame          :     SW_CAN_frame_type;
        variable ID_1           	:     natural := 1;
        variable ID_2           	:     natural := 2;

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
        --    interrupts on Node 1. Enable test mode (to manipulate with Error
        --    counter registers). Keep ERP/EWL values default.
        -----------------------------------------------------------------------
        info("Step 1: Setting FCS Interrupt");
        int_mask.fcs_changed_int := false;
        int_ena.fcs_changed_int := true;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        write_int_enable(int_ena, ID_1, mem_bus(1));
        mode.test := true;
        set_core_mode(mode, ID_1, mem_bus(1));

        -----------------------------------------------------------------------
        -- @2. Check unit is Error Active. Set TX Error counter to 128. Check
        --    unit is Error passive. Check FCS Interrupt is present. Check that
        --    INT pin is high.
        -----------------------------------------------------------------------
        info("Step 2: FCS Interrupt - Error Active -> Error Pasive");
        get_fault_state(fault_state, ID_1, mem_bus(1));
        read_error_counters(err_ctrs, ID_1, mem_bus(1));
        check(fault_state = fc_error_active, "Unit Error Active");
        err_ctrs.tx_counter := 128;
        set_error_counters(err_ctrs, ID_1, mem_bus(1));
        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_error_passive, "Unit Error Passive");
        read_int_status(int_stat, ID_1, mem_bus(1));
        check(int_stat.fcs_changed_int,
            "FCS Interrupt set on Error Active -> Error Passive");
        check(iout(1).irq = '1', "INT pin should be high!");
        
        -----------------------------------------------------------------------
        -- @3. Disable FCS Interrupt and check INT pin goes low. Enable FCS 
        --    Interrupt and check INT pin goes high.
        -----------------------------------------------------------------------
        info("Step 3: Check FCS Interrupt toggles INT pin");
        int_ena.fcs_changed_int := false;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        wait for 10 ns;
        check(iout(1).irq = '0', "INT pin should be low!");
        int_ena.fcs_changed_int := true;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        wait for 10 ns;
        check(iout(1).irq = '1', "INT pin should be high!");        

        -----------------------------------------------------------------------
        -- @4. Clear FCS Interrupt. Check it is cleared.
        -----------------------------------------------------------------------
        info("Step 4: Clear FCS Interrupt - Check it is cleared");
        int_stat.fcs_changed_int := true;
        clear_int_status(int_stat, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_1, mem_bus(1));
        check_false(int_stat.fcs_changed_int,
            "FCS Interrupt still set after clear!");
        check(iout(1).irq = '0', "INT pin should be low!");             

        -----------------------------------------------------------------------
        -- @5. Set TX Error counter to 127. Check unit is Error active. Check 
        --    that FCS Interrupt is set. Check INT pin is high.
        -----------------------------------------------------------------------
        info("Step 5: FCS Interrupt - Error Passive -> Error Active!");
        err_ctrs.tx_counter := 127;
        set_error_counters(err_ctrs, ID_1, mem_bus(1));
        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_error_active, "Unit Error Active");
        read_int_status(int_stat, ID_1, mem_bus(1));
        check(int_stat.fcs_changed_int,
            "FCS Interrupt set on Error Passive -> Error Active");
        check(iout(1).irq = '1', "INT pin should be high!");

        -----------------------------------------------------------------------
        -- @6. Clear FCS Interrupt. Check it is cleared.
        -----------------------------------------------------------------------
        info("Step 6: Clear FCS Interrupt - Check it is cleared");
        int_stat.fcs_changed_int := true;
        clear_int_status(int_stat, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_1, mem_bus(1));
        check_false(int_stat.fcs_changed_int,
            "FCS Interrupt still set after clear!");
        check(iout(1).irq = '0', "INT pin should be low!");

        -----------------------------------------------------------------------
        -- @7. Set TX Error counter to 196. Check EPI Interrupt is set. Clear 
        --    FCS Interrupt.
        -----------------------------------------------------------------------
        info("Step 7: FCS Interrupt - Moving to Error Passive!");
        err_ctrs.tx_counter := 196;
        set_error_counters(err_ctrs, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_1, mem_bus(1));
        check(int_stat.fcs_changed_int, "FCS Interrupt set!");
        int_stat.fcs_changed_int := true;
        clear_int_status(int_stat, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_1, mem_bus(1));
        check_false(int_stat.fcs_changed_int,
            "FCS Interrupt still set after clear!");
        
        -----------------------------------------------------------------------
        -- @8. Set TX Error counter to 256. Check that unit is Bus-off. Check 
        --    that FCS Interrupt is set. Check INT pin is high.
        -----------------------------------------------------------------------
        info("Step 8: FCS Interrupt - Error Passive -> Bus off!");
        err_ctrs.tx_counter := 256;
        set_error_counters(err_ctrs, ID_1, mem_bus(1));
        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_bus_off, "Unit Bus Off");
        read_int_status(int_stat, ID_1, mem_bus(1));
        check(int_stat.fcs_changed_int,
            "FCS Interrupt: Error Passive -> Bus Off");
        check(iout(1).irq = '1', "INT pin should be high!");

        -----------------------------------------------------------------------
        -- @9. Clear FCS Interrupt. Check it is cleared. Check INT pin is low.
        -----------------------------------------------------------------------
        info("Step 9: FCS Interrupt clear");
        int_stat.fcs_changed_int := true;
        clear_int_status(int_stat, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_1, mem_bus(1));
        check_false(int_stat.fcs_changed_int,
            "FCS Interrupt still set after clear!");
        check(iout(1).irq = '0', "INT pin should be low!");
        
        -----------------------------------------------------------------------
        -- @10. Issue ERCRST command. Wait till unit turns Error Active. This
        --     does not test proper duration after which unit turns Error 
        --     active (128 ocurrences of 11 consecutive Recessive bits)!!!
        --     Meanwhile check that FCS Interrupt is not set.
        -----------------------------------------------------------------------
        info("Step 10: Issue ERCRST command, wait till Error Active");
        
        -- Here we have to wait a little bit since this command is processed
        -- in Sample point of Idle!
        wait for 2000 ns;
        command.err_ctrs_rst := true;
        give_controller_command(command, ID_1, mem_bus(1));
        command.err_ctrs_rst := false;
        
        get_fault_state(fault_state, ID_1, mem_bus(1));
        while (fault_state = fc_bus_off) loop
            read_int_status(int_stat, ID_1, mem_bus(1));
            get_fault_state(fault_state, ID_1, mem_bus(1));
            if (fault_state = fc_bus_off) then
                check_false(int_stat.fcs_changed_int,
                    "FCS Interrupt not set in Bus-off.");
            end if;
            wait for 500 ns;
        end loop;
        
        -----------------------------------------------------------------------
        -- @11. When Unit turns Error Active, check that FCS Interrupt is set.
        --     Check INT pin is high. Clear FCS Interrupt.
        -----------------------------------------------------------------------
        info("Step 11: FCS Interrupt: Bus-off -> Error Active");
        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_error_active,
            "Reintegration -> Unit turned Error Active");
        read_int_status(int_stat, ID_1, mem_bus(1));
        check(int_stat.fcs_changed_int,
            "FCS Interrupt set: Bus-off -> Error Active");
        check(iout(1).irq = '1', "INT pin should be high!");
        int_stat.fcs_changed_int := true;
        clear_int_status(int_stat, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_1, mem_bus(1));
        check_false(int_stat.fcs_changed_int,
            "FCS Interrupt still set after clear!");
            
        -----------------------------------------------------------------------
        -- @12. Mask FCS Interrupt. Set TX Error counter to 128. Check Unit is
        --     Error Passive. Check FCS Interrupt is not set. Check INT pin is
        --     low.
        -----------------------------------------------------------------------
        info("Step 12: FCS Interrupt not set when masked");
        int_mask.fcs_changed_int := true;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        err_ctrs.tx_counter := 128;
        set_error_counters(err_ctrs, ID_1, mem_bus(1));
        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_error_passive, "Unit turned Error Passive");
        read_int_status(int_stat, ID_1, mem_bus(1));
        check_false(int_stat.fcs_changed_int,
            "FCS Interrupt not set when masked!");
        check(iout(1).irq = '0', "INT pin should be low!");

        -----------------------------------------------------------------------
        -- @13. Disable FCS Interrupt and check it was disabled. Enable FCS
        --     Interrupt and check it was enabled.
        -----------------------------------------------------------------------
        info("Step 13: Check FCS Interrupt enable works OK!");
        int_ena.fcs_changed_int := false;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        int_ena.fcs_changed_int := true;
        read_int_enable(int_ena, ID_1, mem_bus(1));
        check_false(int_ena.fcs_changed_int, "FCS Interrupt enabled!");

        int_ena.fcs_changed_int := true;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        int_ena.fcs_changed_int := false;
        read_int_enable(int_ena, ID_1, mem_bus(1));
        check(int_ena.fcs_changed_int, "FCS Interrupt disabled!");
        
        -----------------------------------------------------------------------
        -- @14. Mask FCS Interrupt and check it was masked. Un-mask FCS
        --     Interrupt and check it was un-masked.
        -----------------------------------------------------------------------
        info("Step 14: Check FCSI Interrupt mask works OK!");
        int_mask.fcs_changed_int := true;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        int_mask.fcs_changed_int := false;
        read_int_mask(int_mask, ID_1, mem_bus(1));
        check(int_mask.fcs_changed_int, "FCS Interrupt masked!");

        int_mask.fcs_changed_int := false;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        int_mask.fcs_changed_int := true;
        read_int_mask(int_mask, ID_1, mem_bus(1));
        check_false(int_mask.fcs_changed_int, "FCS Interrupt masked!");
        
        info("Finished FCS interrupt test");
        wait for 1000 ns;

    end procedure;
end package body;