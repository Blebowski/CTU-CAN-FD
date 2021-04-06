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
--  STATUS[IDLE] feature test.
--
-- @Verifies:
--  @1. STATUS[IDLE] is set when bus is idle (no frame is in progress).
--  @2. STATUS[IDLE] is not set when unit is transmitting or receiving a frame.
--  @3. STATUS[IDLE] is set when unit is bus-off!
--
-- @Test sequence:
--  @1. Check that STATUS[IDLE] is set. Transmitt CAN frame by DUT wait till
--      SOF and check that STATUS[IDLE] is not set. Wait until the end of frame
--      and check that after the end of Intermission STATUS[IDLE] is set again!
--  @2. Transmitt CAN frame by Test node, wait till arbitration field in DUT
--      and check that STATUS[IDLE] is not set. Wait until the end of frame and
--      check that STATUS[IDLE] is set after the end of intermission.
--  @3. Set DUT to Bus-off by the means of modifying TX Error counter and
--      check it is bus-off. Check that STATUS[IDLE] is set.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    07.11.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.mem_bus_agent_pkg.all;

package status_idle_ftest is
    procedure status_idle_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body status_idle_ftest is
    procedure status_idle_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is        
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;
        variable stat_2             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;
        variable frame_sent         :     boolean;

        variable mode_1             :     SW_mode;
        
        variable err_counters       :     SW_error_counters := (0,0,0,0);
        variable fault_state        :     SW_fault_state;

    begin

        -----------------------------------------------------------------------
        -- @1. Check that STATUS[IDLE] is set. Transmitt CAN frame by DUT
        --    wait till SOF and check that STATUS[IDLE] is not set. Wait until
        --    the end of frame and check that after the end of Intermission
        --    STATUS[IDLE] is set again!
        -----------------------------------------------------------------------
        info_m("Step 1");
        
        get_controller_status(stat_1, DUT_NODE, chn);
        check_m(stat_1.bus_status, "DUT idle after reset!");

        CAN_generate_frame(frame_1);
        CAN_send_frame(frame_1, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_pc_state(pc_deb_arbitration, DUT_NODE, chn);
        
        get_controller_status(stat_1, DUT_NODE, chn);
        check_false_m(stat_1.bus_status, "DUT not idle after frame started!");

        CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
        mem_bus_agent_disable_transaction_reporting(chn);
        while (pc_dbg /= pc_deb_intermission) loop
            wait for 200 ns;
            get_controller_status(stat_1, DUT_NODE, chn);

            CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
            if (pc_dbg /= pc_deb_intermission) then
                check_false_m(stat_1.bus_status, "DUT not idle during frame!");
            end if;
        end loop;
        
        while (pc_dbg = pc_deb_intermission) loop
            wait for 200 ns;
            get_controller_status(stat_1, DUT_NODE, chn);

            CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
            if (pc_dbg = pc_deb_intermission) then
                check_false_m(stat_1.bus_status, "DUT not idle during frame!");
            end if;
        end loop;
        mem_bus_agent_enable_transaction_reporting(chn);
        
        wait for 20 ns; -- To cover latency of RX Trigger for sure!
        get_controller_status(stat_1, DUT_NODE, chn);
        check_m(stat_1.bus_status, "DUT idle after frame has ended!");

        -----------------------------------------------------------------------
        -- @2. Transmitt CAN frame by Test node, wait till arbitration field in 
        --    DUT and check that STATUS[IDLE] is not set. Wait until the end
        --    of frame and check that STATUS[IDLE] is set after the end of
        --    intermission.
        -----------------------------------------------------------------------
        info_m("Step 2");
        
        CAN_generate_frame(frame_1);
        CAN_send_frame(frame_1, 1, TEST_NODE, chn, frame_sent);
        CAN_wait_pc_state(pc_deb_arbitration, DUT_NODE, chn);

        get_controller_status(stat_1, DUT_NODE, chn);
        check_false_m(stat_1.bus_status, "DUT not idle after frame started!");
        
        CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
        mem_bus_agent_disable_transaction_reporting(chn);
        while (pc_dbg /= pc_deb_intermission) loop
            wait for 200 ns;
            get_controller_status(stat_1, DUT_NODE, chn);

            CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
            if (pc_dbg /= pc_deb_intermission) then
                check_false_m(stat_1.bus_status, "DUT not idle during frame!");
            end if;
        end loop;

        while (pc_dbg = pc_deb_intermission) loop
            wait for 200 ns;
            get_controller_status(stat_1, DUT_NODE, chn);

            CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
            if (pc_dbg = pc_deb_intermission) then
                check_false_m(stat_1.bus_status, "DUT not idle during frame!");
            end if;
        end loop;
        mem_bus_agent_enable_transaction_reporting(chn);

        wait for 20 ns; -- To cover latency of RX Trigger for sure!
        get_controller_status(stat_1, DUT_NODE, chn);
        check_m(stat_1.bus_status, "DUT idle after frame has ended!");

        -----------------------------------------------------------------------
        -- @3. Set DUT to Bus-off by the means of modifying TX Error counter
        --    and check it is bus-off. Check that STATUS[IDLE] is set.
        -----------------------------------------------------------------------
        info_m("Step 3");

        mode_1.test := true;
        set_core_mode(mode_1, DUT_NODE, chn);

        err_counters.tx_counter := 256;
        set_error_counters(err_counters, DUT_NODE, chn);
        
        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_bus_off, "Unit bus-off!");
        
        get_controller_status(stat_1, DUT_NODE, chn);
        check_m(stat_1.bus_status, "DUT STAT[IDLE] set when node is bus-off!");        

        wait for 100 ns;

  end procedure;

end package body;
