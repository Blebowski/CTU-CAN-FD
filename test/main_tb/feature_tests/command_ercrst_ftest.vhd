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
--  @1. Set DUT TXC to 256 via test mode. This should set node to bus-off.
--  @2. When DUT becomes bus off, issue COMMAND[ERCRST] to DUT. Now wait for
--      127 * 11 + 10 bits. Check that unit is still bus off! Wait for 30 more
--      bits and check that DUT is now Error active!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    18.01.2020   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package command_ercrst_ftest is
    procedure command_ercrst_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body command_ercrst_ftest is
    procedure command_ercrst_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;

        variable command            :     SW_command := SW_command_rst_val;
        
        variable mode_1             :     SW_mode := SW_mode_rst_val;
        variable err_ctrs           :     SW_error_counters := (0,0,0,0);
        
        variable fault_state        :     SW_fault_state;
    begin

        -----------------------------------------------------------------------
        -- @1. Set DUT TXC to 259 via test mode. This should set node to
        --     bus-off.
        -----------------------------------------------------------------------
        info_m("Step 1");

        mode_1.flexible_data_rate := false;
        mode_1.test := true; -- We need it to set error counters!
        set_core_mode(mode_1, DUT_NODE, chn);

        err_ctrs.tx_counter := 256;
        set_error_counters(err_ctrs, DUT_NODE, chn);

        get_fault_state(fault_state, DUT_NODE, chn);
        while (fault_state /= fc_bus_off) loop
            get_fault_state(fault_state, DUT_NODE, chn);
            wait for 50 ns;
        end loop;

        -----------------------------------------------------------------------
        -- @2. When DUT becomes bus off, issue COMMAND[ERCRST] to DUT.
        --     Now wait for 127 * 11 + 10 bits. Check that unit is still bus
        --     off! Wait for 10 more bits and check that Node 1 is now Error 
        --     active!
        -----------------------------------------------------------------------
        info_m("Step 2");

        -- Now issue Error counter reset!
        command.err_ctrs_rst := true;
        give_controller_command(command, DUT_NODE, chn);

        -- Wait till end of error frame!
        for i in 0 to 13 loop
            CAN_wait_sample_point(DUT_NODE, chn);
        end loop;

        for i in 1 to 127 loop
            info_m ("11 recessive bits nr: " & integer'image(i));
            for i in 1 to 11 loop
                CAN_wait_sample_point(DUT_NODE, chn);
            end loop;
        end loop;

        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_bus_off, "Node still bus off!");

        -- 30 bits is chosen just as upper bound. In fact, we would only need
        -- to account for Error delimiter + Intermission
        for i in 0 to 29 loop
            CAN_wait_sample_point(DUT_NODE, chn);
        end loop;
        
        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_error_active, "Node became error active!");

  end procedure;

end package body;