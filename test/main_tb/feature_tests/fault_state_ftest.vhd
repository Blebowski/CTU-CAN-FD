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
--  FAULT_STATE register feature test.  
--
-- @Verifies:
--  @1. Fault confinement state change from Error Active to Error Passive and
--      vice versa.
--  @2. Fault confinement state change from Error Passive to Bus-off.
--  @3. Fault confinement state change from Bus-off to Error active (after
--      reintegration).
--
-- @Test sequence:
--  @1. Turn on Test mode in DUT. Generate random value of Error counters
--      below default value of ERP and set them in DUT. Check Fault state is
--      Error active.
--  @2. Generate RX counter to be between ERP and 255 included and set it in DUT.
--      Check that DUT Fault state is Error Passive.
--  @3. Generate also TX counter to be between 128 and 255 included and set it
--      in DUT. Check that DUT Fault state is still Error Passive. Lower
--      RX Counter below ERP and check DUT is still Error Passive. Lower also
--      TX Counter below ERP and check that now the node became Error active! 
--  @4. Increment RX Error counter to more than 255 and check that DUT is 
--      Error Passive (should not go to bus-off by RX Counter).
--  @5. Increment TX Counter to go above 255 and check that DUT is now bus off.
--  @6. Issue COMMAND[ERCRST] and wait for 129*11 recessive bits. Check that
--      DUT becomes Error Active.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    30.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--    10.11.2019  Re-wrote to cover new Fault confinement implementation!
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package fault_state_ftest is
    procedure fault_state_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body fault_state_ftest is
    procedure fault_state_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable frame_sent         :       boolean := false;

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
        rand_int_v(127, tx_lt_erp);
        rand_int_v(127, rx_lt_erp);

        rand_int_v(127, tx_mt_erp);
        tx_mt_erp := tx_mt_erp + 128;
        rand_int_v(127, rx_mt_erp);
        rx_mt_erp := rx_mt_erp + 128;

        rand_int_v(50, tx_mt_bof);
        tx_mt_bof := tx_mt_bof + 256;
        rand_int_v(50, rx_mt_bof);
        rx_mt_bof := rx_mt_bof + 256;
        
        info_m("RX less than ERP: " & integer'image(rx_lt_erp));
        info_m("TX less than ERP: " & integer'image(tx_lt_erp));
        
        info_m("RX more than ERP: " & integer'image(rx_mt_erp));
        info_m("TX more than ERP: " & integer'image(tx_mt_erp));
        
        info_m("RX more than Bus off: " & integer'image(rx_mt_bof));
        info_m("TX more than Bus off: " & integer'image(tx_mt_bof));
        
        -----------------------------------------------------------------------
        -- @1. Turn on Test mode in DUT. Generate random value of Error
        --     counters below default value of ERP and set them in DUT.
        --     Check Fault state is Error active.
        -----------------------------------------------------------------------
        info_m("Step 1");
        
        mode_1.test := true;
        set_core_mode(mode_1, DUT_NODE, chn);
        
        err_counters.tx_counter := tx_lt_erp;
        err_counters.rx_counter := rx_lt_erp;
        set_error_counters(err_counters, DUT_NODE, chn);

        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_error_active, "Node Error active!");

        -----------------------------------------------------------------------
        -- @2. Generate RX counter to be between ERP and 255 included and set it
        --     in DUT. Check that DUT Fault state is Error Passive.
        -----------------------------------------------------------------------
        info_m("Step 2");
        
        err_counters.rx_counter := rx_mt_erp;
        set_error_counters(err_counters, DUT_NODE, chn);
        
        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_error_passive, "Node Error passive!");

        -----------------------------------------------------------------------
        -- @3. Generate also TX counter to be between 128 and 255 included and
        --     set it in DUT. Check that DUT Fault state is still Error
        --     Passive. Lower RX Counter below ERP and check DUT is still
        --     Error Passive. Lower also TX Counter below ERP and check that now
        --     the node became Error active! 
        -----------------------------------------------------------------------
        info_m("Step 3");
        
        err_counters.tx_counter := tx_mt_erp;
        set_error_counters(err_counters, DUT_NODE, chn);
        
        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_error_passive, "Node Error passive!");       

        err_counters.rx_counter := rx_lt_erp;
        set_error_counters(err_counters, DUT_NODE, chn);
        
        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_error_passive, "Node Error passive!");
        
        err_counters.tx_counter := tx_lt_erp;
        set_error_counters(err_counters, DUT_NODE, chn);
        
        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_error_active, "Node Error active!");        

        -----------------------------------------------------------------------
        -- @4. Increment RX Error counter to more than 255 and check that DUT
        --     is Error Passive (should not go to bus-off by RX Counter).
        -----------------------------------------------------------------------
        info_m("Step 4");
        
        err_counters.rx_counter := rx_mt_bof;
        set_error_counters(err_counters, DUT_NODE, chn);
        
        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_error_passive, "Node Error passive!");

        -----------------------------------------------------------------------
        -- @5. Increment TX Counter to go above 255 and check that DUT is now
        --     bus off.
        -----------------------------------------------------------------------
        info_m("Step 5");
        
        err_counters.tx_counter := tx_mt_bof;
        set_error_counters(err_counters, DUT_NODE, chn);
        
        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_bus_off, "Node Bus off!");

        -----------------------------------------------------------------------
        -- @6. Issue COMMAND[ERCRST] and wait for 128*11 recessive bits. Check 
        --    that DUT becomes Error Active.
        -----------------------------------------------------------------------
        info_m("Step 6");
        
        command.err_ctrs_rst := true;
        CAN_wait_sample_point(DUT_NODE, chn);
        wait for 20 ns; -- To make sure sample point is missed!
        give_controller_command(command, DUT_NODE, chn);

        -- Wait for 128 * 11 + 10 consecutive recessive bits
        for i in 0 to (128 * 11) - 1 loop
            CAN_wait_sample_point(DUT_NODE, chn);
        end loop;

        for i in 0 to 10 loop
            CAN_wait_sample_point(DUT_NODE, chn);
        end loop;

        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_bus_off,
            "Node not reintegrated before 129 * 11 bits");

        CAN_wait_sample_point(DUT_NODE, chn);
        wait for 20 ns;
        
        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_error_active,
            "Node reintegration finished after 129 * 11 bits");

        read_error_counters(err_counters, DUT_NODE, chn);
        check_m(err_counters.tx_counter = 0, "TX Error counter erased!");
        check_m(err_counters.rx_counter = 0, "RX Error counter erased!");

    end procedure;

end package body;