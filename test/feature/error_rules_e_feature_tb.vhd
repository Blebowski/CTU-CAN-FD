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
--  Fault confinement rules - rule E - feature test.
--
-- Verifies:
--  1. If a receiver detects a bit error while sending an active error flag
--     or an overload flag, the receive error counter shall be incremented by
--     8.
--
-- Test sequence:
--  1. Set Node 2 to ACK forbidden mode. Generate CAN frame and send it by Node
--     1. Wait until Error frame is sent by Node 2. Wait for random amount of
--     bits (0-5) and force bus level to recessive. Wait until sample point and
--     check that RX Error counter was incremented by 9 (first form error in EOF,
--     next bit error during Error frame)! Wait until bus is idle!
--  2. Unset ACK Forbidden in Node 2. Generate CAN frame and send it by Node 1.
--     Wait until Intermission in Node 2 and force bus low for duration of one
--     bit time (overload condition). Check that overload frame is being tran-
--     smitted. Wait for random amount of bits (0-5) and force bus level to
--     recessive for one bit-time. Wait until bus is idle and check that RX
--     Error counter was incremented by 7 (incremented by 8, but decremented by
--     1 due to tran frame valid!)
--------------------------------------------------------------------------------
-- Revision History:
--    26.11.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package error_rules_e_feature is
    procedure error_rules_e_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;

package body error_rules_e_feature is
    procedure error_rules_e_feature_exec(
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
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        
        variable err_counters_1     :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_2     :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_3     :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_4     :       SW_error_counters := (0, 0, 0, 0);

        variable id_vect            :       std_logic_vector(28 downto 0);
        variable err_capt           :       SW_error_capture;
        
        variable bit_waits          :       natural := 0;
        variable pc_dbg             :       SW_PC_Debug;
    begin

        -----------------------------------------------------------------------
        -- 1. Set Node 2 to ACK forbidden mode. Generate CAN frame and send it
        --    by Node 1. Wait until Error frame is sent by Node 2. Wait for
        --    random amount of bits (0-5) and force bus level to recessive.
        --    Wait until sample point and check that RX Error counter was
        --    incremented by 9 (first form error in EOF, next bit error during
        --    Error frame)! Wait until bus is idle!
        -----------------------------------------------------------------------
        info("Step 1");
        
        read_error_counters(err_counters_1, ID_2, mem_bus(2));
        
        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, ID_2, mem_bus(2));
        
        CAN_enable_retr_limit(true, 0, ID_1, mem_bus(1));
        
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_error_frame(ID_2, mem_bus(2));
        
        rand_int_v(rand_ctr, 5, bit_waits);
        info ("Waiting for " & integer'image(bit_waits) & " bits!");
        for i in 0 to bit_waits - 1 loop
            CAN_wait_sample_point(iout(2).stat_bus);
        end loop;

        -- Force Recessive during Error Flag!
        force_bus_level(RECESSIVE, so.bl_force, so.bl_inject);
        CAN_wait_sample_point(iout(2).stat_bus, false);
        wait for 20 ns;
        release_bus_level(so.bl_force);

        read_error_counters(err_counters_2, ID_2, mem_bus(2));

        check(err_counters_2.rx_counter = err_counters_1.rx_counter + 9,
            "RX Error counter inctemented by 8 due to bit error in Error flag!");

        check(err_counters_2.tx_counter = err_counters_1.tx_counter,
            "TX Error counter unchanged in receiver!");

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        -----------------------------------------------------------------------
        -- 2. Unset ACK Forbidden in Node 2. Generate CAN frame and send it by
        --    Node 1. Wait until Intermission in Node 2 and force bus low for
        --    duration of one bit time (overload condition). Check that
        --    overload frame is being transmitted. Wait for random amount of
        --    bits (0-5) and force bus level to recessive for one bit-time.
        --    Wait until bus is idle and check that TX Error counter was 
        --    incremented by 7 (incremented by 8, but decremented by 1 due to 
        --    tran frame valid!)
        -----------------------------------------------------------------------
        info("Step 2");

        read_error_counters(err_counters_3, ID_2, mem_bus(2));

        mode_2.acknowledge_forbidden := false;
        set_core_mode(mode_2, ID_2, mem_bus(2));

        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_pc_state(pc_deb_intermission, ID_2, mem_bus(2));

        -- Force Dominant -> Overload condition!
        force_bus_level(DOMINANT, so.bl_force, so.bl_inject);
        CAN_wait_sample_point(iout(2).stat_bus, false);
        wait for 20 ns;
        release_bus_level(so.bl_force);
        
        CAN_read_pc_debug(pc_dbg, ID_2, mem_bus(2));
        check(pc_dbg = pc_deb_overload, "Overload frame transmitted!");

        rand_int_v(rand_ctr, 5, bit_waits);
        for i in 0 to bit_waits - 1 loop
            CAN_wait_sample_point(iout(2).stat_bus);
        end loop;

        -- Force Recessive during Overload Flag!
        force_bus_level(RECESSIVE, so.bl_force, so.bl_inject);
        CAN_wait_sample_point(iout(2).stat_bus, false);
        wait for 20 ns;
        release_bus_level(so.bl_force);

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        read_error_counters(err_counters_4, ID_2, mem_bus(2));

        check(err_counters_3.rx_counter + 7 = err_counters_4.rx_counter,
            "RX Error counter inctemented by 8 due to bit error in Overload flag!");

        check(err_counters_3.tx_counter = err_counters_4.tx_counter,
            "TX Error counter unchanged in transmitter!");

    end procedure;

end package body;