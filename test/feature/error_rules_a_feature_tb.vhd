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
--  Fault confinement rules - rule A - feature test.
--
-- Verifies:
--  1. When a receiver detects an error, the receive error counter shall be
--     incremented by 1, except when the detected error was a bit error during
--     the sending of an active error flag or an overload flag.
--
-- Test sequence:
--  1. Send CAN frame by Node 2. Wait until Node 2 is post arbitration and force
--     bus value to opposite level it has transmitted. Check that error frame
--     is transmitted by Node 2. Node 1 should hook up too by error frame. Wait
--     until bus is idle and check that REC was incremented by 1 in Node 1.
--  2. Send CAN frame by Node 2. Wait until Node 2 is post arbitration and force
--     bus value to opposite level it has transmitted. Check that error frame is
--     transmitted by Node 2. Release bus-level. Wait until Error frame in Node 1
--     and check that Node 1 RX counter was incremented by 1. Force bus high
--     and wait for 16 bit times. Release bus-level and check that RX Error
--     counter was incremented by 129 (1 for initial increment, 16 times
--     recessive bit was sampled during Error frame).
--------------------------------------------------------------------------------
-- Revision History:
--    24.11.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package error_rules_a_feature is
    procedure error_rules_a_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;

package body error_rules_a_feature is
    procedure error_rules_a_feature_exec(
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
        
        variable status             :       SW_status;
        variable command            :       SW_command := SW_command_rst_val;
        
        variable rx_buf_info        :       SW_RX_Buffer_info;
        variable mode_1             :       SW_mode := SW_mode_rst_val;
        
        variable err_counters_1     :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_2     :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_3     :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_4     :       SW_error_counters := (0, 0, 0, 0);
    begin
        o.outcome := true;

        -----------------------------------------------------------------------
        -- 1. Send CAN frame by Node 2. Wait until Node 2 is post arbitration
        --    and force bus value to opposite level it has transmitted. Check
        --    that error frame is transmitted by Node 2. Node 1 should hook up
        --    too by error frame. Wait until bus is idle and check that REC was
        --    incremented by 1 in Node 1.
        -----------------------------------------------------------------------
        info("Step 1");

        read_error_counters(err_counters_1, ID_1, mem_bus(1));
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_pc_state(pc_deb_control, ID_2, mem_bus(2));
        CAN_wait_not_pc_state(pc_deb_control, ID_2, mem_bus(2));
        wait for 1000 ns;
        
        get_controller_status(status, ID_1, mem_bus(1));
        check(status.receiver, "Node 1 receiver");

        wait until iout(2).can_tx = DOMINANT;
        force_bus_level(RECESSIVE, so.bl_force, so.bl_inject);
        CAN_wait_sample_point(iout(2).stat_bus);
        wait for 20 ns;
        
        get_controller_status(status, ID_2, mem_bus(2));
        check(status.error_transmission, "Error frame transmitted by Node 2");
        release_bus_level(so.bl_force);
        
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        read_error_counters(err_counters_2, ID_1, mem_bus(1));

        check(err_counters_1.rx_counter + 1 = err_counters_2.rx_counter,
              "RX counter incremented by 1 in Receiver!");

        check(err_counters_1.tx_counter = err_counters_2.tx_counter,
              "TX counter unchanged in Receiver!");

        -----------------------------------------------------------------------
        -- 2. Send CAN frame by Node 2. Wait until Node 2 is post arbitration
        --    and force bus value to opposite level it has transmitted. Check
        --    that error frame is transmitted by Node 2. Release bus-level.
        --    Wait until Error frame in Node 1 and check that Node 1 RX 
        --    counter was incremented by 1.  Force bus high and wait for ten
        --    16 times. Release bus-level and check that RX Error counter was
        --    incremented by 129 (1 for initial increment, 16 times recessive
        --    bit was sampled during Error frame).
        -----------------------------------------------------------------------
        info("Step 2");
        
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_pc_state(pc_deb_control, ID_2, mem_bus(2));
        CAN_wait_not_pc_state(pc_deb_control, ID_2, mem_bus(2));
        wait for 1000 ns;
        
        get_controller_status(status, ID_1, mem_bus(1));
        check(status.receiver, "Node 1 receiver");
        
        wait until iout(2).can_tx = DOMINANT;
        force_bus_level(RECESSIVE, so.bl_force, so.bl_inject);
        CAN_wait_sample_point(iout(2).stat_bus);
        wait for 35 ns; -- To account for all pipeline stages delay!
        
        get_controller_status(status, ID_2, mem_bus(2));
        check(status.error_transmission, "Error frame transmitted by Node 2");
        release_bus_level(so.bl_force);
        
        -- Poll on Node 1 until it transmitts error frame
        get_controller_status(status, ID_1, mem_bus(1));
        while (status.error_transmission = false) loop
            get_controller_status(status, ID_1, mem_bus(1));
        end loop;

        -- Now force the bus low for 10 bit times
        force_bus_level(RECESSIVE, so.bl_force, so.bl_inject);
        for i in 0 to 15 loop
            CAN_wait_sample_point(iout(1).stat_bus);
        end loop;
        wait for 20 ns;
        release_bus_level(so.bl_force);

        wait for 20 ns; -- To update error counters!
        
        -- We need to check right away, not when node becomes idle! The reason
        -- is:
        --  When forcing recessive, Node 2 is already in Error delimiter and
        --  it will therefore finish its error delimiter and go idle during the
        --  time of Active error flag of Node 1 which we force Recessive.
        --  So at time when we release the bus and Node 1 finally transmitts its
        --  Active Error flag, Node 2 will already accept this as SOF, detect
        --  Stuff Error and will transmitt error frame! Due to this Node 1 will
        --  increment its REC by 8 because it detected Dominant bit after sending
        --  Active Error flag. So when we would wait until idle, REC of Node 1
        --  would be higher by 8 due to that detected dominant bit!
        read_error_counters(err_counters_3, ID_1, mem_bus(1));
        check(err_counters_2.rx_counter + 129 = err_counters_3.rx_counter,
              "RX counter incremented by 8 for each recessive during error flag! " &
              " Expected: " & integer'image(err_counters_2.rx_counter + 129) &
              " Real: " & integer'image(err_counters_3.rx_counter));
        check(err_counters_2.tx_counter = err_counters_3.tx_counter,
              "TX counter unchanged in Receiver!");

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));       

        return;

    end procedure;

end package body;