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
--  Fault confinement rules - rule B - feature test.
--
-- Verifies:
--  1. When a receiver detects a dominant bit as the first bit after sending an
--     error flag, the receive error counter shall be incremented by 8.
--
-- Test sequence:
--  1. Set Node 2 not to accept CAN FD frames. Transmitt CAN FD frame by Node 1
--     and Wait until Error frame in Node 2. Read Error counters of Node 2.
--     Wait for 7 sample points (Error flag + 1st bit post Error flag) of Node 2
--     and check that sampled value is dominant and sent is recessive. Read
--     RX Error counter and check that it was incremented by 8. Check that TX
--     Error counter is the same as before!
--  2. Set Node 1 not to accept CAN FD frames (Node 2 will accept CAN FD frames).
--     Transmitt CAN FD frame by Node 1
--     and Wait until Error frame in Node 2. Read Error counters of Node 2.
--     Wait for 7 sample points (Error flag + 1st bit post Error flag) of Node 2
--     and check that sampled value is dominant and sent is recessive. Read
--     RX Error counter and check that it was incremented by 8. Check that TX
--     Error counter is the same as before!
--------------------------------------------------------------------------------
-- Revision History:
--    26.11.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package error_rules_b_feature is
    procedure error_rules_b_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;

package body error_rules_b_feature is
    procedure error_rules_b_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1               :       natural := 1;
        variable ID_2               :       natural := 2;
        variable CAN_frame          :       SW_CAN_frame_type;
        variable RX_CAN_frame       :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable rand_value         :       natural;
        
        variable status             :       SW_status;
        variable command            :       SW_command := SW_command_rst_val;
        
        variable rx_buf_info        :       SW_RX_Buffer_info;
        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        
        variable err_counters_1     :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_2     :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_3     :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_4     :       SW_error_counters := (0, 0, 0, 0);
    begin

        -----------------------------------------------------------------------
        -- 1. Set Node 2 not to accept CAN FD frames. Transmitt CAN FD frame by
        --    Node 1 and Wait until Error frame in Node 2. Read Error counters
        --    of Node 2. Wait for 7 sample points (Error flag + 1st bit post 
        --    Error flag) of Node 2 and check that sampled value is dominant
        --    and sent is recessive. Read RX Error counter and check that it
        --    was incremented by 8. Check that TX Error counter is the same as
        --    before! 
        -----------------------------------------------------------------------
        info("Step 1");

        CAN_enable_retr_limit(true, 0, ID_1, mem_bus(1));
        
        mode_2.flexible_data_rate := false;
        set_core_mode(mode_2, ID_2, mem_bus(2));

        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_frame.frame_format := FD_CAN;
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);

        CAN_wait_error_frame(ID_2, mem_bus(2));
        wait for 20 ns;
        read_error_counters(err_counters_1, ID_2, mem_bus(2));

        for i in 0 to 6 loop
            CAN_wait_sample_point(iout(2).stat_bus);
        end loop;

        wait for 20 ns;
        read_error_counters(err_counters_2, ID_2, mem_bus(2));

        check(err_counters_2.rx_counter = err_counters_1.rx_counter + 8,
            "RX Error counter inctemented by 8 in receiver!");

        check(err_counters_2.tx_counter = err_counters_1.tx_counter,
            "TX Error counter unchanged in receiver!");

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        -----------------------------------------------------------------------
        -- 2. Set Node 1 not to accept CAN FD frames (Node 2 will accept CAN FD 
        --    frames). Transmitt CAN FD frame by Node 1 and Wait until Error 
        --    frame in Node 2. Read Error counters of Node 2. Wait for 7 sample
        --    points (Error flag + 1st bit post Error flag) of Node 2 and check
        --    that sampled value is dominant and sent is recessive. Read RX Error
        --    counter and check that it was incremented by 8. Check that TX Error
        --    counter is the same as before!
        -----------------------------------------------------------------------
        info("Step 2");

        -- Enable CAN FD frames in Node 2
        mode_2.flexible_data_rate := true;
        set_core_mode(mode_2, ID_2, mem_bus(2));

        -- Disable in Node 1
        mode_1.flexible_data_rate := false;
        set_core_mode(mode_1, ID_1, mem_bus(1));

        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_frame.frame_format := FD_CAN;
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);

        -- Now Node 1 should be the first to transmitt Error frame!
        CAN_wait_error_frame(ID_1, mem_bus(1));
        wait for 20 ns;
        read_error_counters(err_counters_1, ID_1, mem_bus(1));

        for i in 0 to 6 loop
            CAN_wait_sample_point(iout(1).stat_bus);
        end loop;

        wait for 20 ns;
        read_error_counters(err_counters_2, ID_1, mem_bus(1));

        check(err_counters_2.rx_counter = err_counters_1.rx_counter,
            "RX Error counter unchanged in transmitter!");

        check(err_counters_2.tx_counter = err_counters_1.tx_counter,
            "TX Error counter unchanged in transmitter!");

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

    end procedure;

end package body;