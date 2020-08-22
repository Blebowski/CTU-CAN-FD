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
--  Jiri Novak <jnovak@fel.cvut.cz>
--  Pavel Pisa <pisa@cmp.felk.cvut.cz>
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
--  STATUS[EWL] feature test.
--
-- @Verifies:
--  @1. When RX Error counter (REC) is higher than EWL, then STAT[EWL] is set.
--  @2. When TX Error counter (TEC) is higher than EWL, then STAT[EWL] is set.
--  @3. When both REC and TEC are lower than EWL, STAT[EWL] is not set.
--
-- @Test sequence:
--  @1. Set Node 1 in test mode. Generate random EWL and configure it. Generate
--      random REC and TEC and set them in Node 1. Pre-calculate if STAT[EWL]
--      shall be set and check it.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    02.11.2019  Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package status_ewl_feature is
    procedure status_ewl_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body status_ewl_feature is
    procedure status_ewl_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1               :     natural := 1;
        variable ID_2               :     natural := 2;

        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;
        
        variable mode_1             :     SW_mode;
        variable err_counters       :     SW_error_counters;
        variable fault_th           :     SW_fault_thresholds := (0,0);
        variable exp_stat_ewl       :     boolean;
    begin

        -----------------------------------------------------------------------
        -- @1. Set Node 1 in test mode. Generate random EWL and configure it.
        --    Generate random REC and TEC and set them in Node 1. Pre-calculate
        --    if STAT[EWL] shall be set and check it.
        -----------------------------------------------------------------------
        info("Step 1");
        mode_1.test := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));
        
        -- Random REC and TEC
        rand_int_v(rand_ctr, 255, err_counters.rx_counter);
        rand_int_v(rand_ctr, 255, err_counters.tx_counter);
        set_error_counters(err_counters, ID_1, mem_bus(1));

        -- Random EWL
        rand_int_v(rand_ctr, 255, fault_th.ewl);
        set_fault_thresholds(fault_th, ID_1, mem_bus(1));

        -- Calculate expected status
        if (err_counters.tx_counter >= fault_th.ewl or
            err_counters.rx_counter >= fault_th.ewl)
        then
            exp_stat_ewl := true;
        else
            exp_stat_ewl := false;
        end if;

        get_controller_status(stat_1, ID_1, mem_bus(1));
        check(stat_1.error_warning = exp_stat_ewl,
            "STAT[EWL] equals expected value! " &
            " Expected: " & boolean'image(exp_stat_ewl) &
            " Real: " & boolean'image(stat_1.error_warning) &
            " REC: " & integer'image(err_counters.rx_counter) &
            " TEC: " & integer'image(err_counters.tx_counter) &
            " EWL: " & integer'image(fault_th.ewl)); 

        wait for 100 ns;

  end procedure;

end package body;
