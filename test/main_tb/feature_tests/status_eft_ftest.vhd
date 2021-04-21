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
--  STATUS[EFT] feature test.
--
-- @Verifies:
--  @1. STATUS[EFT] is is set when Error frame is transmitted during Error active
--      and Error passive.
--  @2. STATUS[EFT] is not set when Error frame is not being transmitted.
--
-- @Test sequence:
--  @1. Set Test node to ACF mode. Enable test mode in DUT. Send frame by DUT.
--      Randomize if DUT will be error active or error passive. Monitor
--      STATUS[EFT] and check that it is not set during whole duration of the
--      frame. Wait till ACK field.
--  @2. Wait till DUT is NOT is ACK field anymore. Now since ACK was recessive,
--      DUT should be transmitting error frame! Monitor STATUS[EFT] and check
--      it is set until DUT gets to Intermission. Check it is not set after
--      Intermission has started! Monitor STATUS[EFT] and check it is not set
--      during whole time until unit is Bus Idle!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    31.10.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.mem_bus_agent_pkg.all;

package status_eft_ftest is
    procedure status_eft_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body status_eft_ftest is
    procedure status_eft_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;
        variable frame_sent         :     boolean;

        -- Node status
        variable stat_1             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;        

        variable mode_1             :     SW_mode := SW_mode_rst_val;
        variable mode_2             :     SW_mode := SW_mode_rst_val;

        variable go_err_passive     :     std_logic;
        variable err_counters       :     SW_error_counters := (0,0,0,0);
        variable fault_state        :     SW_fault_state;
    begin

        -----------------------------------------------------------------------
        --  @1. Set Test node to ACF mode. Enable test mode in DUT. Send frame
        --     by DUT. Randomize if DUT will be error active or error
        --     passive. Monitor STATUS[EFT] and check that it is not set 
        --     during whole duration of the frame. Wait till ACK field.
        -----------------------------------------------------------------------
        info_m("Step 1");

        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, TEST_NODE, chn);
        mode_1.test := true;
        set_core_mode(mode_1, DUT_NODE, chn);
        
        -- Randomize error active or passive!
        rand_logic_v(go_err_passive, 0.5);
        if (go_err_passive = '1') then
            info_m("Going Error passive!");
            err_counters.rx_counter := 140; -- Should be in error passive! 
            set_error_counters(err_counters, DUT_NODE, chn);
            get_fault_state(fault_state, DUT_NODE, chn);
            check_m(fault_state = fc_error_passive, "DUT Error Passive!");
        else
            info_m("Going Error active!");
            err_counters.rx_counter := 0; -- Should be in error active! 
            set_error_counters(err_counters, DUT_NODE, chn);
            get_fault_state(fault_state, DUT_NODE, chn);
            check_m(fault_state = fc_error_active, "DUT Error Active!");
        end if;

        CAN_generate_frame(frame_1);
        -- Needed so that there is no prolonged ACK slot!
        frame_1.frame_format := NORMAL_CAN;
        CAN_send_frame(frame_1, 1, DUT_NODE, chn, frame_sent);

        CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
        mem_bus_agent_disable_transaction_reporting(chn);
        while (pc_dbg /= pc_deb_ack) loop
            wait for 200 ns; -- To make checks more sparse!
            CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
            
            get_controller_status(stat_1, DUT_NODE, chn);
            check_false_m(stat_1.error_transmission,
                "STAT[EFT] not set before ACK!");
        end loop;
        mem_bus_agent_enable_transaction_reporting(chn);

        -----------------------------------------------------------------------
        --  @2. Wait till DUT is NOT is ACK field anymore. Now since ACK was
        --     recessive, DUT should be transmitting error frame! Monitor
        --     STATUS[EFT] and check it is set until DUT gets to Intermi-
        --     ssion. Check it is not set after Intermission has started!
        --     Monitor STATUS[EFT] and check it is not set during whole time
        --     until unit is Bus Idle!
        -----------------------------------------------------------------------
        info_m("Step 2");

        CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
        mem_bus_agent_disable_transaction_reporting(chn);
        while (pc_dbg = pc_deb_ack) loop            
            wait for 100 ns; -- To make checks more sparse!
            
            get_controller_status(stat_1, DUT_NODE, chn);
            CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
            
            if (pc_dbg = pc_deb_ack) then
                check_false_m(stat_1.error_transmission, "STAT[EFT] not set in ACK!");
            end if;
        end loop;

        CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
        while (pc_dbg /= pc_deb_intermission) loop            
            wait for 100 ns; -- To make checks more sparse!

            get_controller_status(stat_1, DUT_NODE, chn);
            CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
            if (pc_dbg /= pc_deb_intermission) then
                check_m(stat_1.error_transmission, "STAT[EFT] set during Error frame!");
            end if;
        end loop;

        get_controller_status(stat_1, DUT_NODE, chn);
        while (stat_1.bus_status = false) loop -- Loop until bus is idle
            wait for 100 ns; -- To make checks more sparse!
            
            get_controller_status(stat_1, DUT_NODE, chn);
            check_false_m(stat_1.error_transmission, "STAT[EFT] not set in Intermission!");
        end loop;
        mem_bus_agent_enable_transaction_reporting(chn);

  end procedure;

end package body;
