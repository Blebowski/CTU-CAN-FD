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
-- Purpose:
--  STATUS[RXS] feature test.
--
-- Verifies:
--  1. STATUS[RXS] is set when unit is receiver.
--  2. STATUS[RXS] is not set when unit is transmitter.
--
-- Test sequence:
--  1. Send frame by Node 2. Wait until SOF starts and check that STATUS[RXS] is
--     not set till SOF in Node 1. From SOF further monitor STATUS[RXS] and
--     check it set until the end of Intermission. Check that after the end of
--     intermission, STATUS[RXS] is not set anymore.
--  2. Send frame by Node 1. Monitor STATUS[RXS] of Node 1 until Intermission
--     and check STATUS[RXS] is not set. Monitor until the end of intermission
--     and check STATUS[RXS] is not set.
--------------------------------------------------------------------------------
-- Revision History:
--    31.10.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package status_rxs_feature is
    procedure status_rxs_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body status_rxs_feature is
    procedure status_rxs_feature_exec(
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

        variable pc_dbg             :     SW_PC_Debug;
        variable frame_sent         :     boolean;
    begin

        -----------------------------------------------------------------------
        --  1. Send frame by Node 2. Wait until SOF starts and check that
        --     STATUS[RXS] is not set till SOF in Node 1. From SOF further
        --     monitor STATUS[RXS] and check it set until the end of
        --     Intermission. Check that after the end of intermission, 
        --     STATUS[TXS] is not set anymore.
        -----------------------------------------------------------------------
        info("Step 1");
        CAN_generate_frame(rand_ctr, frame_1);
        CAN_send_frame(frame_1, 1, ID_2, mem_bus(2), frame_sent);

        CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
        while (pc_dbg /= pc_deb_arbitration) loop
            CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
        end loop;

        while (pc_dbg /= pc_deb_intermission) loop
            wait for 200 ns;
            get_controller_status(stat_1, ID_1, mem_bus(1));

            CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
            if (pc_dbg /= pc_deb_intermission) then
                check(stat_1.receiver, "Node 1 receiver");
            end if;
        end loop;

        -- There should be no Suspend, Overload frames, so after intermission
        -- we should go to idle
        CAN_wait_not_pc_state(pc_deb_intermission, ID_1, mem_bus(1));
        get_controller_status(stat_1, ID_1, mem_bus(1));
        check_false(stat_1.receiver, "Node 1 not receiver in idle!");

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        -----------------------------------------------------------------------
        -- 2. Send frame by Node 2. Monitor STATUS[RXS] of Node 1 until Inter-
        --    mission and check STATUS[RXS] is not set. Monitor until the end
        --    of intermission and check STATUS[RXS] is not set.
        -----------------------------------------------------------------------
        info("Step 2");
        CAN_generate_frame(rand_ctr, frame_1);
        CAN_send_frame(frame_1, 4, ID_1, mem_bus(1), frame_sent);

        while (pc_dbg /= pc_deb_intermission) loop
            wait for 200 ns;
            get_controller_status(stat_1, ID_1, mem_bus(1));

            CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
            check_false(stat_1.receiver, "Node 1 not transmitter!");
        end loop;

        wait for 100 ns;

  end procedure;

end package body;
