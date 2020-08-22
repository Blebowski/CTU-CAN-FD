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
-- @TestInfoStart
--
-- @Purpose:
--  Fault confinement rules - rule G - feature test.
--
-- @Verifies:
--  @1. After the successful transmission of a frame (getting ACK and no error
--      has been detected until EOF is finished), the transmit error counter
--      shall be decremented by 1 unless it was already 0.
--
-- @Test sequence:
--  @1. Set TEC of Node 1 to random value till 255. Send frame by Node 1, wait
--      till end of EOF and check that TEC is decremented at the end of EOF!
--  @2. Set TEC of Node 1 to 0. Send frame by Node 1. Wait until frame is sent
--      and check that TEC is still 0!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    13.12.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package error_rules_g_feature is
    procedure error_rules_g_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;

package body error_rules_g_feature is
    procedure error_rules_g_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1               :       natural := 1;
        variable ID_2               :       natural := 2;
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;

        variable mode_1             :       SW_mode := SW_mode_rst_val;
        
        variable err_counters_1     :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_2     :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_3     :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_4     :       SW_error_counters := (0, 0, 0, 0);

        variable pc_dbg             :       SW_PC_Debug;
    begin

        -----------------------------------------------------------------------
        -- @1. Set TEC of Node 1 to random value till 255. Send frame by Node 1,
        --    wait till end of EOF and check that TEC is decremented at the
        --    end of EOF!
        -----------------------------------------------------------------------
        info("Step 1");

        mode_1.test := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));

        rand_int_v(rand_ctr, 255, err_counters_1.tx_counter);
        if (err_counters_1.tx_counter = 0) then
            err_counters_1.tx_counter := 1;
        end if;
        set_error_counters(err_counters_1, ID_1, mem_bus(1));
        
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_pc_state(pc_deb_eof, ID_1, mem_bus(1));

        CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
        while (pc_dbg = pc_deb_eof) loop
            CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
            if (pc_dbg = pc_deb_eof) then
                read_error_counters(err_counters_2, ID_1, mem_bus(1));
                check(err_counters_1.tx_counter = err_counters_2.tx_counter,
                        "TEC not decremented before end of EOF!");
            end if;
            wait for 50 ns; -- To make checks more sparse
        end loop;
        
        read_error_counters(err_counters_2, ID_1, mem_bus(1));
        check(err_counters_1.tx_counter - 1 = err_counters_2.tx_counter,
                "TEC decremented by 1 after EOF");

        -----------------------------------------------------------------------
        -- @2. Set TEC of Node 1 to 0. Send frame by Node 1. Wait until frame is
        --    sent and check that TEC is still 0!
        -----------------------------------------------------------------------
        info("Step 2");

        err_counters_1.tx_counter := 0;
        set_error_counters(err_counters_1, ID_1, mem_bus(1));

        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_frame_sent(ID_1, mem_bus(1));

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));
        
        read_error_counters(err_counters_2, ID_1, mem_bus(1));
        check(err_counters_2.tx_counter = 0, "TEC remains zero!");

    end procedure;

end package body;