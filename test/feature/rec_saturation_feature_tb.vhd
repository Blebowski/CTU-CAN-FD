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
--  REC (RX Error counter) saturation feature test.
--
-- Verifies:
--  1. REC is saturated at its width (9 bits) and further errors in receiver
--     will not cause its overflow.
--
-- Test sequence:
--  1. Set Test mode in Node 1. Disable CAN FD support in Node 1 and Set REC to
--     510 in Node 1. Check that REC is 510.
--  2. Send CAN FD frame by Node 2 few times. Wait until CAN frame is sent and
--     check that REC in Node 1 is 511 after first frame (was incremented by 1)
--     and also after each next attempt to transmitt a frame.
--------------------------------------------------------------------------------
-- Revision History:
--    11.11.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package rec_saturation_feature is
    procedure rec_saturation_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body rec_saturation_feature is
    procedure rec_saturation_feature_exec(
        variable    o               : out    feature_outputs_t;
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

        variable ctrs_1             :       SW_traffic_counters;
        variable ctrs_2             :       SW_traffic_counters;
        variable ctrs_3             :       SW_traffic_counters;
        variable ctrs_4             :       SW_traffic_counters;
        variable ctrs_5             :       SW_traffic_counters;
        
        variable status             :       SW_status;
        variable command            :       SW_command := SW_command_rst_val;
        
        variable rx_buf_info        :       SW_RX_Buffer_info;
        variable mode_1             :       SW_mode := SW_mode_rst_val;
        
        variable err_counters       :       SW_error_counters := (0, 0, 0, 0);
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- 1. Set Test mode in Node 1. Disable CAN FD support in Node 1 and Set
        --    REC to 510 in Node 1. Check that REC is 510.
        ------------------------------------------------------------------------
        info("Step 1");
        mode_1.test := true;
        mode_1.flexible_data_rate := false;
        set_core_mode(mode_1, ID_1, mem_bus(1));

        err_counters.rx_counter := 510;
        set_error_counters(err_counters, ID_1, mem_bus(1));
        err_counters.rx_counter := 0;
        read_error_counters(err_counters, ID_1, mem_bus(1));
        check(err_counters.rx_counter = 510, "REC set properly!");

        ------------------------------------------------------------------------
        -- 2. Send CAN FD frame by Node 2 few times. Wait until CAN frame is
        --    sent and check that REC in Node 1 is 511 after first frame (was
        --    incremented by 1) and also after each next attempt to transmitt a
        --    frame.
        ------------------------------------------------------------------------
        info("Step 2");
        
        -- Set to one shot mode
        CAN_enable_retr_limit(true, 0, ID_1, mem_bus(1));

        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_frame.frame_format := FD_CAN;

        for i in 0 to 3 loop
            CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
            CAN_wait_frame_sent(ID_1, mem_bus(1));
            
            read_error_counters(err_counters, ID_1, mem_bus(1));
            
            if (i = 0) then
                check(err_counters.rx_counter = 511, "REC incremented to 511!");
            else
                check(err_counters.rx_counter = 511, "REC remains saturated at 511!");
            end if;
        end loop;

    end procedure;

end package body;