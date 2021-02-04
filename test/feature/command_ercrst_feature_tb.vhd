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
--  Error counters reset (reintegration request) command!
--
-- @Verifies:
--  @1. Reintegration is started by COMMAND[ERCRST] when unit is bus-off.
--  @2. Reintegration is not finished before 128 consecutive occurences of 11 con
--      secutive recessive bits!
--
-- @Test sequence:
--  @1. Set Node 1 TXC to 256 via test mode. This should set node to bus-off.
--  @2. When Node 1 becomes bus off, issue COMMAND[ERCRST] to Node 1. Wait until
--      bus level is recessive (this should be in the start of Error delimiter).
--      Now wait for 127 * 11 + 10 bits. Check that unit is still bus off!
--      Wait for 30 more bits and check that Node 1 is now Error active!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    18.01.2020   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package command_ercrst_feature is
    procedure command_ercrst_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body command_ercrst_feature is
    procedure command_ercrst_feature_exec(
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

        variable command            :     SW_command := SW_command_rst_val;
        
        variable mode_1             :     SW_mode := SW_mode_rst_val;
        variable err_ctrs           :     SW_error_counters := (0,0,0,0);
        
        variable fault_state        :     SW_fault_state;
    begin

        -----------------------------------------------------------------------
        -- @1. Set Node 1 TXC to 259 via test mode. This should set node to
        --     bus-off.
        -----------------------------------------------------------------------
        info("Step 1");

        mode_1.flexible_data_rate := false;
        mode_1.test := true; -- We need it to set error counters!
        set_core_mode(mode_1, ID_1, mem_bus(1));

        err_ctrs.tx_counter := 256;
        set_error_counters(err_ctrs, ID_1, mem_bus(1));

        get_fault_state(fault_state, ID_1, mem_bus(1));
        while (fault_state /= fc_bus_off) loop
            get_fault_state(fault_state, ID_1, mem_bus(1));
            wait for 50 ns;
        end loop;

        -----------------------------------------------------------------------
        -- @2. When Node 1 becomes bus off, issue COMMAND[ERCRST] to Node 1.
        --     Wait until bus level is recessive (this should be in the start
        --     of Error delimiter). Now wait for 127 * 11 + 10 bits. Check that
        --     unit is still bus off! Wait for 10 more bits and check that Node
        --     1 is now Error active!
        -----------------------------------------------------------------------
        info("Step 2");

        -- Now issue Error counter reset!
        command.err_ctrs_rst := true;
        give_controller_command(command, ID_1, mem_bus(1));

        if (bus_level /= RECESSIVE) then
            wait until bus_level = RECESSIVE;
        end if;

        for i in 0 to 1406 loop
            CAN_wait_sample_point(iout(1).stat_bus);
        end loop;

        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_bus_off, "Node still bus off!");

        -- 30 bits is chosen just as upper bound. In fact, we would only need
        -- to account for Error delimiter + Intermission
        for i in 0 to 29 loop
            CAN_wait_sample_point(iout(1).stat_bus);
        end loop;
        
        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_error_active, "Node became error active!");

        wait for 100 ns;

  end procedure;

end package body;