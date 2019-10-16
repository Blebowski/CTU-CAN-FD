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
-- Purpose:
--  Interrupt EWL feature test.
--
-- Verifies:
--  1. EWL Interrupt is set when TX Error counter is more or equal to EWL.
--  2. EWL Interrupt is set when RX Error counter more or equal to EWL.
--  3. EWL Interrupt is not set when either TX or RX Interrupt more or equal to
--     EWL, and EWL Interrupt is masked.
--  4. EWL Interrupt causes INT to go high when it is enabled.
--  5. EWL Interrupt causes INT to go low when it is disabled.
--  6. EWL Interrupt is cleared by write to INT_STATUS register.
--  7. EWL Interrupt enable is manipulated properly by INT_ENA_SET and
--     INT_ENA_CLEAR.
--  8. EWL Interrupt mask is manipulated properly by INT_MASK_SET and
--     INT_MASK_CLEAR.
--
-- Test sequence:
--  1. Unmask and enable EWL Interrupt, disable and mask all other interrupts on
--     Node 1. Turn on Test mode (to manipulate error counters via CTR_PRES).
--     Leave default value of EWL (96).
--  2. Set TX Error counter to 96. Check that EWL Interrupt was set. Check that
--     EWL Interrupt is set and INT pin is high.
--  3. Disable EWL Interrupt and check INT pin goes low. Enable EWL Interrupt
--     and check INT pin goes high. 
--  4. Clear EWL Interrupt and check it not set. Set TX Error counter to
--     95. Clear EWL Interrupt. Check EWL interrupt is cleared.
--  5. Set RX Error counter to 96. Check that EWL Interrupt was set. Check that
--     INT pin goes high. 
--  6. Disable EWL Interrupt and check INT pin goes low. Enable EWL Interrupt
--     and check INT pin goes high again. 
--  7. Clear EWL Interrupt. Check that EWL Interrupt is not set. Set RX Error
--     counter to 95. Check INT pin goes low and Interrupt is cleared!
--  8. Mask EWL Interrupt. Set TX Error counter to 96. Check that EWL interrupt
--     was not set.
--  9. Disable EWL Interrupt and check it was disabled. Enable EWL Interrupt and
--     check it was enabled.
-- 10. Mask EWL Interrupt and check it was masked. Un-mask EWL Interrupt and
--     check it was un-masked.
--------------------------------------------------------------------------------
-- Revision History:
--    22.6.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package int_ewl_feature is
    procedure int_ewl_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body int_ewl_feature is
    procedure int_ewl_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable r_data             :     std_logic_vector(31 downto 0) :=
                                            (OTHERS => '0');
        variable CAN_frame          :     SW_CAN_frame_type;
        variable CAN_frame_rx       :     SW_CAN_frame_type;
        variable frame_sent         :     boolean := false;
        variable frames_equal       :     boolean := false;
        variable size_of_buf        :     natural;
        variable ctr_1              :     natural;
        variable ctr_2              :     natural;
        variable ID_1           	:     natural := 1;
        variable ID_2           	:     natural := 2;
        variable vect_1             :     std_logic_vector(31 downto 0);
        variable vect_2             :     std_logic_vector(31 downto 0);
        variable mode_prev          :     std_logic_vector(31 downto 0);
        variable mode_prev_2        :     std_logic_vector(31 downto 0);

        variable int_mask           :     SW_interrupts := SW_interrupts_rst_val;
        variable int_ena            :     SW_interrupts := SW_interrupts_rst_val;
        variable int_stat           :     SW_interrupts := SW_interrupts_rst_val;
        variable command            :     SW_command := SW_command_rst_val;
        variable mode               :     SW_mode := SW_mode_rst_val;
        variable buf_info           :     SW_RX_Buffer_info;
        variable pc_dbg             :     SW_PC_Debug;
        variable err_ctrs           :     SW_error_counters;
    begin
        o.outcome := true;

        -----------------------------------------------------------------------
        -- 1. Unmask and enable EWL Interrupt, disable and mask all other 
        --    interrupts on Node 1. Turn on Test mode (to manipulate error
        --    counters via CTR_PRES). Leave default value of EWL (96).
        -----------------------------------------------------------------------
        info("Step 1: Setting EWL Interrupt");
        int_mask.error_warning_int := false;
        int_ena.error_warning_int := true;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        write_int_enable(int_ena, ID_1, mem_bus(1));
        mode.test := true;
        set_core_mode(mode, ID_1, mem_bus(1));

        -----------------------------------------------------------------------
        -- 2. Set TX Error counter to 96. Check that EWL Interrupt was set. 
        -- Check that EWL Interrupt is set and INT pin is high.
        -----------------------------------------------------------------------
        info("Step 2: Check EWL from TX Error counter");
        read_error_counters(err_ctrs, ID_1, mem_bus(1));
        err_ctrs.tx_counter := 96;
        set_error_counters(err_ctrs, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_1, mem_bus(1));
        check(int_stat.error_warning_int, "EWL Interrupt set when RX Error counter >= EWL!");
        check(iout(1).irq = '1', "INT pin should be high!");

        -----------------------------------------------------------------------
        -- 3. Disable EWL Interrupt and check INT pin goes low. Enable EWL 
        --    Interrupt and check INT pin goes high. 
        -----------------------------------------------------------------------
        info("Step 3: Check EWL from TX counter toggles INT pin.");
        int_ena.error_warning_int := false;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        wait for 10 ns;
        check(iout(1).irq = '0', "INT pin should be low!");
        int_ena.error_warning_int := true;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        wait for 10 ns;
        check(iout(1).irq = '1', "INT pin should be high!");

        -----------------------------------------------------------------------
        -- 4. Clear EWL Interrupt and check it not set. Set TX Error
        --    counter to 95. Clear EWL Interrupt. Check EWL interrupt is
        --    cleared.
        -----------------------------------------------------------------------
        info("Step 4: Check EWL is set again.");
        int_stat.error_warning_int := true;
        clear_int_status(int_stat, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_1, mem_bus(1));
        check_false(int_stat.error_warning_int,
            "EWL Interrupt set again when RX Error counter >= EWL!");
        err_ctrs.tx_counter := 95;
        set_error_counters(err_ctrs, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_1, mem_bus(1));
        check_false(int_stat.error_warning_int,
            "EWL Interrupt not set when RX Error counter < EWL!");
        check(iout(1).irq = '0', "INT pin should be low!");

        -----------------------------------------------------------------------
        -- 5. Set RX Error counter to 96. Check that EWL Interrupt was set. 
        -- Check that INT pin goes high. 
        -----------------------------------------------------------------------
        info("Step 5: Check EWL from RX Error counter");
        err_ctrs.rx_counter := 96;
        set_error_counters(err_ctrs, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_1, mem_bus(1));
        check(int_stat.error_warning_int, "EWL Interrupt set when TX Error counter >= EWL!");
        check(iout(1).irq = '1', "INT pin should be high!");

        -----------------------------------------------------------------------
        -- 6. Disable EWL Interrupt and check INT pin goes low. Enable EWL 
        --    Interrupt and check INT pin goes high again. 
        -----------------------------------------------------------------------
        info("Step 6: Check EWL from RX counter toggles INT pin.");
        int_ena.error_warning_int := false;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        wait for 10 ns;
        check(iout(1).irq = '0', "INT pin should be low!");
        int_ena.error_warning_int := true;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        wait for 10 ns;
        check(iout(1).irq = '1', "INT pin should be high!");

        -----------------------------------------------------------------------
        -- 7. Clear EWL Interrupt. Check that EWL Interrupt is not set. Set RX
        --    Error counter to 95. Check INT pin goes low and Interrupt is
        --    cleared!
        -----------------------------------------------------------------------
        info("Step 7: Check EWL from RX counter is set again.");
        int_stat.error_warning_int := true;
        clear_int_status(int_stat, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_1, mem_bus(1));
        check_false(int_stat.error_warning_int,
            "EWL Interrupt not set again when TX Error counter >= EWL!");
        err_ctrs.rx_counter := 95;

        set_error_counters(err_ctrs, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_1, mem_bus(1));
        check_false(int_stat.error_warning_int,
            "EWL Interrupt not set when RX Error counter < EWL!");
        check(iout(1).irq = '0', "INT pin should be low!");

        -----------------------------------------------------------------------
        -- 8. Mask EWL Interrupt. Set TX Error counter to 96. Check that EWL 
        --    interrupt was not set.
        -----------------------------------------------------------------------
        info("Step 8: Check EWL is not set when masked!");
        int_mask.error_warning_int := true;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        err_ctrs.tx_counter := 96;
        set_error_counters(err_ctrs, ID_1, mem_bus(1));
        read_int_status(int_stat, ID_1, mem_bus(1));
        check_false(int_stat.error_warning_int, "EWL Interrupt not set when masked!");
        check(iout(1).irq = '0', "INT pin should be low!");

        -----------------------------------------------------------------------
        -- 9. Disable EWL Interrupt and check it was disabled. Enable EWL 
        -- Interrupt and check it was enabled.
        -----------------------------------------------------------------------
        info("Step 9: Check EWL Interrupt enable works OK!");
        int_ena.error_warning_int := false;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        int_ena.error_warning_int := true;
        read_int_enable(int_ena, ID_1, mem_bus(1));
        check_false(int_ena.error_warning_int, "EWL Interrupt enabled!");

        int_ena.error_warning_int := true;
        write_int_enable(int_ena, ID_1, mem_bus(1));
        int_ena.error_warning_int := false;
        read_int_enable(int_ena, ID_1, mem_bus(1));
        check(int_ena.error_warning_int, "EWL Interrupt disabled!");

        -----------------------------------------------------------------------
        -- 10. Mask EWL Interrupt and check it was masked. Un-mask EWL 
        -- Interrupt and check it was un-masked.
        -----------------------------------------------------------------------
        info("Step 10: Check EWL Interrupt mask works OK!");
        int_mask.error_warning_int := true;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        int_mask.error_warning_int := false;
        read_int_mask(int_mask, ID_1, mem_bus(1));
        check(int_mask.error_warning_int, "EWL Interrupt masked!");

        int_mask.error_warning_int := false;
        write_int_mask(int_mask, ID_1, mem_bus(1));
        int_mask.error_warning_int := true;
        read_int_mask(int_mask, ID_1, mem_bus(1));
        check_false(int_mask.error_warning_int, "EWL Interrupt masked!");

        info("Finished EWL interrupt test");
        wait for 1000 ns;

    end procedure;
end package body;