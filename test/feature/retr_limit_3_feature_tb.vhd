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
--  Retransmitt limit feature test 3 (cornercases).
--
-- Verifies:
--  1. When unit is a receiver without attempt to transmitt a frame
--     (TXT Buffer is ready, unit is Error passive and dominant bit is detected
--      during Suspend field), if an error occurs during such a frame,
--     retransmitt counter is not incremented!
--
-- Test sequence:
--  1. Configure retransmitt limit in Node 1, enable retransmitt limitation.
--     Enable Test Mode in Node 1 to be able manipulate with Error counters.
--     Configure Node 2 to Acknowledge Forbidden Mode to invoke transmission
--     of Error frames during test.
--  2. Set Node 1 TX Error counter to 150. Check that Node 1 is Error Passive.
--     Send frame by Node 1. Wait until Error frame occurs. Check that Retransmit
--     counter in Node 1 is now 1. Insert frame to Node 2. Wait until Suspend
--     transmission in Node 1.
--  3. Wait until Arbitration field in Node 1, check that Node 1 is now receiver.
--     Wait until ACK field, force the bus for the whole duration of ACK field
--     to Recessive. Check that Error frame is transmitted by Node 1. Wait until
--     bus is Idle. Check that Retransmitt counter is stil1 1.
--------------------------------------------------------------------------------
-- Revision History:
--    13.7.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package retr_limit_3_feature is
    procedure retr_limit_3_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body retr_limit_3_feature is
    procedure retr_limit_3_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;
        variable retr_th            :       natural;

        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        variable err_counters       :       SW_error_counters := (0, 0, 0, 0);
        variable status             :       SW_status;
        variable retr_ctr           :       natural;
        variable fault_state        :       SW_fault_state;
    begin
        
        -- Hard coded threshold is enough for this test!
        retr_th := 5;

        ------------------------------------------------------------------------
        -- 1. Configure retransmitt limit in Node 1, enable retransmitt
        --    limitation. Enable Test Mode in Node 1 to be able manipulate with
        --    Error counters. Configure Node 2 to Acknowledge Forbidden Mode to
        --    invoke transmission of Error frames during test.
        ------------------------------------------------------------------------
        info("Step 1: Configuring retransmitt limit to 1 (Node 1), ACF (Node 2)");
        CAN_enable_retr_limit(true, retr_th, ID_1, mem_bus(1));
        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, ID_2, mem_bus(2));
        mode_1.test := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));
        
        ------------------------------------------------------------------------
        -- 2. Set Node 1 TX Error counter to 150. Check that Node 1 is Error
        --    Passive. Send frame by Node 1. Wait until Error frame occurs.
        --    Check that Retransmit counter in Node 1 is now 1. Wait until 
        --    Suspend transmission in Node 1. Insert frame to Node 2.
        ------------------------------------------------------------------------
        info("Step 2: Set Node 1 to Error passive");
        err_counters.tx_counter := 150;
        set_error_counters(err_counters, ID_1, mem_bus(1));
        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_error_passive, "Unit Error Passive!");

        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_error_frame(ID_1, mem_bus(1));
        retr_ctr := CAN_spy_retr_ctr(iout(1).stat_bus);
        check(retr_ctr = 1,
            "Retransmitt counter 1 after Error frame!");
        CAN_wait_pc_state(pc_deb_suspend, ID_1, mem_bus(1)); -- Wait until suspend
        CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
        
        ------------------------------------------------------------------------
        -- 3. Wait until Arbitration field in Node 1, check that Node 1 is now
        --    receiver. Wait until ACK field, force the bus for the whole du-
        --    ration of ACK field to Recessive. Check that Error frame is
        --    transmitted by Node 1. Wait until Intermission. Check that Retran-
        --    smitt counter is stil1 1.
        ------------------------------------------------------------------------
        info("Step 3: Check Retransmitt counter not incremented when receiver only");
        CAN_wait_pc_state(pc_deb_arbitration, ID_1, mem_bus(1));
        wait for 10 ns; -- Operational state updated in the same clock cycle!

        get_controller_status(status, ID_1, mem_bus(1));
        check(status.receiver, "Node 1 turned receiver when in Error Passive!");

        CAN_wait_pc_state(pc_deb_ack, ID_1, mem_bus(1));
        force_bus_level(RECESSIVE, so.bl_force, so.bl_inject);
        CAN_wait_not_pc_state(pc_deb_ack, ID_1, mem_bus(1));
        release_bus_level(so.bl_force);
        get_controller_status(status, ID_1, mem_bus(1));
        check(status.error_transmission, "Error frame being transmitted!");

        CAN_wait_pc_state(pc_deb_intermission, ID_1, mem_bus(1));
        retr_ctr := CAN_spy_retr_ctr(iout(1).stat_bus);
        check(retr_ctr = 1,
            "Retransmitt counter not incremented when unit was only receiver!");

        wait for 1000 ns;
        
  end procedure;

end package body;