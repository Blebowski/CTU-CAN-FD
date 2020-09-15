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
--  STATUS[EFT] feature test.
--
-- @Verifies:
--  @1. STATUS[EFT] is is set when Error frame is transmitted during Error active
--      and Error passive.
--  @2. STATUS[EFT] is not set when Error frame is not being transmitted.
--
-- @Test sequence:
--  @1. Set Node 2 to ACF mode. Enable test mode in Node 1. Send frame by Node 1.
--      Randomize if Node 1 will be error active or error passive. Monitor
--      STATUS[EFT] and check that it is not set during whole duration of the
--      frame. Wait till ACK field.
--  @2. Wait till Node 1 is NOT is ACK field anymore. Now since ACK was recessive,
--      Node 1 should be transmitting error frame! Monitor STATUS[EFT] and check
--      it is set until Node 1 gets to Intermission. Check it is not set after
--      Intermission has started! Monitor STATUS[EFT] and check it is not set
--      during whole time until unit is Bus Idle!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    31.10.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package status_eft_feature is
    procedure status_eft_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body status_eft_feature is
    procedure status_eft_feature_exec(
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
        --  @1. Set Node 2 to ACF mode. Enable test mode in Node 1. Send frame
        --     by Node 1. Randomize if Node 1 will be error active or error
        --     passive. Monitor STATUS[EFT] and check that it is not set 
        --     during whole duration of the frame. Wait till ACK field.
        -----------------------------------------------------------------------
        info("Step 1");

        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, ID_2, mem_bus(2));
        mode_1.test := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));
        
        -- Randomize error active or passive!
        rand_logic_v(rand_ctr, go_err_passive, 0.5);
        if (go_err_passive = '1') then
            info("Going Error passive!");
            err_counters.rx_counter := 140; -- Should be in error passive! 
            set_error_counters(err_counters, ID_1, mem_bus(1));
            get_fault_state(fault_state, ID_1, mem_bus(1));
            check(fault_state = fc_error_passive, "Node 1 Error Passive!");
        else
            info("Going Error active!");
            err_counters.rx_counter := 0; -- Should be in error active! 
            set_error_counters(err_counters, ID_1, mem_bus(1));
            get_fault_state(fault_state, ID_1, mem_bus(1));
            check(fault_state = fc_error_active, "Node 1 Error Active!");
        end if;

        CAN_generate_frame(rand_ctr, frame_1);
        -- Needed so that there is no prolonged ACK slot!
        frame_1.frame_format := NORMAL_CAN;
        CAN_send_frame(frame_1, 1, ID_1, mem_bus(1), frame_sent);

        CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
        while (pc_dbg /= pc_deb_ack) loop
            wait for 200 ns; -- To make checks more sparse!
            CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
            
            get_controller_status(stat_1, ID_1, mem_bus(1));
            check_false(stat_1.error_transmission,
                "STAT[EFT] not set before ACK!");
        end loop;

        -----------------------------------------------------------------------
        --  @2. Wait till Node 1 is NOT is ACK field anymore. Now since ACK was
        --     recessive, Node 1 should be transmitting error frame! Monitor
        --     STATUS[EFT] and check it is set until Node 1 gets to Intermi-
        --     ssion. Check it is not set after Intermission has started!
        --     Monitor STATUS[EFT] and check it is not set during whole time
        --     until unit is Bus Idle!
        -----------------------------------------------------------------------
        info("Step 2");

        CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
        while (pc_dbg = pc_deb_ack) loop            
            wait for 100 ns; -- To make checks more sparse!
            
            get_controller_status(stat_1, ID_1, mem_bus(1));
            CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
            
            if (pc_dbg = pc_deb_ack) then
                check_false(stat_1.error_transmission, "STAT[EFT] not set in ACK!");
            end if;
        end loop;

        CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
        while (pc_dbg /= pc_deb_intermission) loop            
            wait for 100 ns; -- To make checks more sparse!

            get_controller_status(stat_1, ID_1, mem_bus(1));
            CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
            if (pc_dbg /= pc_deb_intermission) then
                check(stat_1.error_transmission, "STAT[EFT] set during Error frame!");
            end if;
        end loop;

        get_controller_status(stat_1, ID_1, mem_bus(1));
        while (stat_1.bus_status = false) loop -- Loop until bus is idle
            wait for 100 ns; -- To make checks more sparse!
            
            get_controller_status(stat_1, ID_1, mem_bus(1));
            check_false(stat_1.error_transmission, "STAT[EFT] not set in Intermission!");
        end loop;
        
        wait for 100 ns;

  end procedure;

end package body;
