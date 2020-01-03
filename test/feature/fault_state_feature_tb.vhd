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
--  FAULT_STATE register feature test.  
--
-- Verifies:
--  1. Fault confinement state change from Error Active to Error Passive and
--     vice versa.
--  2. Fault confinement state change from Error Passive to Bus-off.
--  3. Fault confinement state change from Bus-off to Error active (after
--     reintegration).
--
-- Test sequence:
--  1. Turn on Test mode in Node 1. Generate random value of Error counters
--     below default value of ERP and set them in Node 1. Check Fault state is
--     Error active.
--  2. Generate RX counter to be between ERP and 255 included and set it in Node
--     1. Check that Node 1 Fault state is Error Passive.
--  3. Generate also TX counter to be between 128 and 255 included and set it
--     in Node 1. Check that Node 1 Fault state is still Error Passive. Lower
--     RX Counter below ERP and check Node 1 is still Error Passive. Lower also
--     TX Counter below ERP and check that now the node became Error active! 
--  4. Increment RX Error counter to more than 255 and check that Node 1 is 
--     Error Passive (should not go to bus-off by RX Counter).
--  5. Increment TX Counter to go above 255 and check that Node 1 is now bus off.
--  6. Issue COMMAND[ERCRST] and wait for 128*11 recessive bits. Check that
--     Node 1 becomes Error Active.
--
--------------------------------------------------------------------------------
-- Revision History:
--    30.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--    10.11.2019  Re-wrote to cover new Fault confinement implementation!
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package fault_state_feature is
    procedure fault_state_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body fault_state_feature is
    procedure fault_state_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable frame_sent         :       boolean := false;
        variable ID_1           	:       natural := 1;

        variable err_counters       :       SW_error_counters;
        variable fault_state        :       SW_fault_state;

        variable tx_lt_erp          :       natural;
        variable rx_lt_erp          :       natural;
        variable tx_mt_erp          :       natural;
        variable rx_mt_erp          :       natural;
        variable tx_mt_bof          :       natural;
        variable rx_mt_bof          :       natural;
        
        variable command            :       SW_command := SW_command_rst_val;
        variable mode_1             :       SW_mode := SW_mode_rst_val;
    begin

        -----------------------------------------------------------------------
        -- Pre-generate thresholds
        -----------------------------------------------------------------------
        rand_int_v(rand_ctr, 127, tx_lt_erp);
        rand_int_v(rand_ctr, 127, rx_lt_erp);

        rand_int_v(rand_ctr, 127, tx_mt_erp);
        tx_mt_erp := tx_mt_erp + 128;
        rand_int_v(rand_ctr, 127, rx_mt_erp);
        rx_mt_erp := rx_mt_erp + 128;

        rand_int_v(rand_ctr, 50, tx_mt_bof);
        tx_mt_bof := tx_mt_bof + 255;
        rand_int_v(rand_ctr, 50, rx_mt_bof);
        rx_mt_bof := rx_mt_bof + 255;
        
        info("RX less than ERP: " & integer'image(rx_lt_erp));
        info("TX less than ERP: " & integer'image(tx_lt_erp));
        
        info("RX more than ERP: " & integer'image(rx_mt_erp));
        info("TX more than ERP: " & integer'image(tx_mt_erp));
        
        info("RX more than Bus off: " & integer'image(rx_mt_bof));
        info("TX more than Bus off: " & integer'image(tx_mt_bof));
        
        -----------------------------------------------------------------------
        -- 1. Turn on Test mode in Node 1. Generate random value of Error
        --    counters below default value of ERP and set them in Node 1.
        --    Check Fault state is Error active.
        -----------------------------------------------------------------------
        info("Step 1");
        mode_1.test := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));
        err_counters.tx_counter := tx_lt_erp;
        err_counters.rx_counter := rx_lt_erp;
        set_error_counters(err_counters, ID_1, mem_bus(1));

        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_error_active, "Node Error active!");

        -----------------------------------------------------------------------
        -- 2. Generate RX counter to be between ERP and 255 included and set it
        --    in Node 1. Check that Node 1 Fault state is Error Passive.
        -----------------------------------------------------------------------
        info("Step 2");
        err_counters.rx_counter := rx_mt_erp;
        set_error_counters(err_counters, ID_1, mem_bus(1));
        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_error_passive, "Node Error passive!");

        -----------------------------------------------------------------------
        -- 3. Generate also TX counter to be between 128 and 255 included and
        --    set it in Node 1. Check that Node 1 Fault state is still Error
        --    Passive. Lower RX Counter below ERP and check Node 1 is still
        --    Error Passive. Lower also TX Counter below ERP and check that now
        --    the node became Error active! 
        -----------------------------------------------------------------------
        info("Step 3");
        err_counters.tx_counter := tx_mt_erp;
        set_error_counters(err_counters, ID_1, mem_bus(1));
        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_error_passive, "Node Error passive!");       

        err_counters.rx_counter := rx_lt_erp;
        set_error_counters(err_counters, ID_1, mem_bus(1));
        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_error_passive, "Node Error passive!");
        
        err_counters.tx_counter := tx_lt_erp;
        set_error_counters(err_counters, ID_1, mem_bus(1));
        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_error_active, "Node Error active!");        

        -----------------------------------------------------------------------
        -- 4. Increment RX Error counter to more than 255 and check that Node 1
        --    is Error Passive (should not go to bus-off by RX Counter).
        -----------------------------------------------------------------------
        info("Step 4");
        err_counters.rx_counter := rx_mt_bof;
        set_error_counters(err_counters, ID_1, mem_bus(1));
        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_error_passive, "Node Error passive!");

        -----------------------------------------------------------------------
        -- 5. Increment TX Counter to go above 255 and check that Node 1 is now
        --    bus off.
        -----------------------------------------------------------------------
        info("Step 5");
        err_counters.tx_counter := tx_mt_bof;
        set_error_counters(err_counters, ID_1, mem_bus(1));
        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_bus_off, "Node Bus off!");

        -----------------------------------------------------------------------
        -- 6. Issue COMMAND[ERCRST] and wait for 128*11 recessive bits. Check 
        --    that Node 1 becomes Error Active.
        -----------------------------------------------------------------------
        info("Step 6");
        command.err_ctrs_rst := true;
        CAN_wait_sample_point(iout(1).stat_bus, false);
        wait for 20 ns; -- To make sure sample point is missed!
        give_controller_command(command, ID_1, mem_bus(1));

        -- Wait for 127 * 11 + 10 consecutive recessive bits
        for i in 0 to (127 * 11) - 1 loop
            CAN_wait_sample_point(iout(1).stat_bus, false);
        end loop;

        for i in 0 to 10 loop
            CAN_wait_sample_point(iout(1).stat_bus, false);
        end loop;

        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_bus_off,
            "Node not reintegrated before 128 * 11 bits");

        CAN_wait_sample_point(iout(1).stat_bus, false);
        wait for 20 ns;
        
        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_error_active,
            "Node reintegration finished after 128 * 11 bits");

        read_error_counters(err_counters, ID_1, mem_bus(1));
        check(err_counters.tx_counter = 0, "TX Error counter erased!");
        check(err_counters.rx_counter = 0, "RX Error counter erased!");

    end procedure;

end package body;