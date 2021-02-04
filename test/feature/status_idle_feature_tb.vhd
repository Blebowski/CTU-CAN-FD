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
--  @1. Check that STATUS[IDLE] is set. Transmitt CAN frame by Node 1 wait till
--      SOF and check that STATUS[IDLE] is not set. Wait until the end of frame
--      and check that after the end of Intermission STATUS[IDLE] is set again!
--  @2. Transmitt CAN frame by Node 2, wait till arbitration field in Node 1
--      and check that STATUS[IDLE] is not set. Wait until the end of frame and
--      check that STATUS[IDLE] is set after the end of intermission.
--  @3. Set Node 1 to Bus-off by the means of modifying TX Error counter and
--      check it is bus-off. Check that STATUS[IDLE] is set.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    07.11.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package status_idle_feature is
    procedure status_idle_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body status_idle_feature is
    procedure status_idle_feature_exec(
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
        variable stat_2             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;
        variable frame_sent         :     boolean;

        variable mode_1             :     SW_mode;
        
        variable err_counters       :     SW_error_counters := (0,0,0,0);
        variable fault_state        :     SW_fault_state;

    begin

        -----------------------------------------------------------------------
        -- @1. Check that STATUS[IDLE] is set. Transmitt CAN frame by Node 1
        --    wait till SOF and check that STATUS[IDLE] is not set. Wait until
        --    the end of frame and check that after the end of Intermission
        --    STATUS[IDLE] is set again!
        -----------------------------------------------------------------------
        info("Step 1");
        
        get_controller_status(stat_1, ID_1, mem_bus(1));
        check(stat_1.bus_status, "Node 1 idle after reset!");

        CAN_generate_frame(rand_ctr, frame_1);
        CAN_send_frame(frame_1, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_pc_state(pc_deb_arbitration, ID_1, mem_bus(1));
        
        get_controller_status(stat_1, ID_1, mem_bus(1));
        check_false(stat_1.bus_status, "Node 1 not idle after frame started!");

        CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
        while (pc_dbg /= pc_deb_intermission) loop
            wait for 200 ns;
            get_controller_status(stat_1, ID_1, mem_bus(1));

            CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
            if (pc_dbg /= pc_deb_intermission) then
                check_false(stat_1.bus_status, "Node 1 not idle during frame!");
            end if;
        end loop;
        
        while (pc_dbg = pc_deb_intermission) loop
            wait for 200 ns;
            get_controller_status(stat_1, ID_1, mem_bus(1));

            CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
            if (pc_dbg = pc_deb_intermission) then
                check_false(stat_1.bus_status, "Node 1 not idle during frame!");
            end if;
        end loop;
        
        wait for 20 ns; -- To cover latency of RX Trigger for sure!
        get_controller_status(stat_1, ID_1, mem_bus(1));
        check(stat_1.bus_status, "Node 1 idle after frame has ended!");

        -----------------------------------------------------------------------
        -- @2. Transmitt CAN frame by Node 2, wait till arbitration field in 
        --    Node 1 and check that STATUS[IDLE] is not set. Wait until the end
        --    of frame and check that STATUS[IDLE] is set after the end of
        --    intermission.
        -----------------------------------------------------------------------
        info("Step 2");
        
        CAN_generate_frame(rand_ctr, frame_1);
        CAN_send_frame(frame_1, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_pc_state(pc_deb_arbitration, ID_1, mem_bus(1));

        get_controller_status(stat_1, ID_1, mem_bus(1));
        check_false(stat_1.bus_status, "Node 1 not idle after frame started!");
        
        CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
        while (pc_dbg /= pc_deb_intermission) loop
            wait for 200 ns;
            get_controller_status(stat_1, ID_1, mem_bus(1));

            CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
            if (pc_dbg /= pc_deb_intermission) then
                check_false(stat_1.bus_status, "Node 1 not idle during frame!");
            end if;
        end loop;

        while (pc_dbg = pc_deb_intermission) loop
            wait for 200 ns;
            get_controller_status(stat_1, ID_1, mem_bus(1));

            CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
            if (pc_dbg = pc_deb_intermission) then
                check_false(stat_1.bus_status, "Node 1 not idle during frame!");
            end if;
        end loop;

        wait for 20 ns; -- To cover latency of RX Trigger for sure!
        get_controller_status(stat_1, ID_1, mem_bus(1));
        check(stat_1.bus_status, "Node 1 idle after frame has ended!");

        -----------------------------------------------------------------------
        -- @3. Set Node 1 to Bus-off by the means of modifying TX Error counter
        --    and check it is bus-off. Check that STATUS[IDLE] is set.
        -----------------------------------------------------------------------
        info("Step 3");

        mode_1.test := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));

        err_counters.tx_counter := 256;
        set_error_counters(err_counters, ID_1, mem_bus(1));
        
        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_bus_off, "Unit bus-off!");
        
        get_controller_status(stat_1, ID_1, mem_bus(1));
        check(stat_1.bus_status, "Node 1 STAT[IDLE] set when node is bus-off!");        

        wait for 100 ns;

  end procedure;

end package body;
