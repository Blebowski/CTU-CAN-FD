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
--  Fault confinement rules - rule F - transmitter - feature test
--
-- Verifies:
--  1. 
--
-- Test sequence:
--  1. Set Node 2 to ACK forbidden mode. Send frame by Node 1. Wait until Error
--     frame is sent by Node 1. Force bus low and wait for 7 bits. Check that
--     Error counter is still the same as after Error frame started! Wait for
--     one more bit and check that TEC counter was incremented by 8! Wait for 7
--     bits and check that TEC remains the same. Wait for one more bit and
--     check that TEC is incremented again by 8. Release the bus and Wait until
--     bus is idle!
--  2. Set Node 1 to Error passive. Send frame by Node 1 and wait until Error
--     frame is sent. Repeat the same procedure as in Step 1. Afterwards
--     release the bus and wait until bus is idle!
--  3. Unset ACK forbidden mode in Node 2. Send frame by Node 1. Wait until
--     Intermission of Node 1 and force bus low. Wait until Overload frame is
--     sent. Repeat the procedure from step 1. Release the bus and wait until
--     bus is idle!
--------------------------------------------------------------------------------
-- Revision History:
--    10.12.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package error_rules_f_tx_feature is
    procedure error_rules_f_tx_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;

package body error_rules_f_tx_feature is

    ---------------------------------------------------------------------------
    -- Executes following sequence:
    --  1. Read TEC and Force bus low and wait for 13 bits.
    --  2. Check that TEC has not changed.
    --  3. Wait for one more bit.
    --  4. Check that TEC was incremented by 8.
    --  5. Wait for 7 bits.
    --  6. Check that TEC has not changed.
    --  7. Wait for one more bit.
    --  8. Check that TEC has changed!
    --  9. Release bus
    ---------------------------------------------------------------------------
    procedure do_14_8_check(
        signal      so              : out    feature_signal_outputs_t;     
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t
    ) is
        variable ID_1               : natural := 1;
        variable err_counters_1     : SW_error_counters := (0, 0, 0, 0);
        variable err_counters_2     : SW_error_counters := (0, 0, 0, 0);
        variable err_counters_3     : SW_error_counters := (0, 0, 0, 0);
        variable err_counters_4     : SW_error_counters := (0, 0, 0, 0);
        variable err_counters_5     : SW_error_counters := (0, 0, 0, 0);
    begin
        read_error_counters(err_counters_1, ID_1, mem_bus(1));
        force_bus_level(DOMINANT, so.bl_force, so.bl_inject);
        for i in 0 to 12 loop
            CAN_wait_sample_point(iout(1).stat_bus, false);
        end loop;
        wait for 20 ns; -- To be sure sample point was processed!
        
        read_error_counters(err_counters_2, ID_1, mem_bus(1));
        check(err_counters_1.tx_counter = err_counters_2.tx_counter,
            "TEC not incremented after 13 dominant bits!");
        
        CAN_wait_sample_point(iout(1).stat_bus, false);
        wait for 20 ns; -- To be sure sample point was processed!
        read_error_counters(err_counters_3, ID_1, mem_bus(1));
        
        check(err_counters_1.tx_counter + 8 = err_counters_3.tx_counter,
            "TEC incremented by 8 after 14 dominant bits!");
        
        for i in 0 to 6 loop
            CAN_wait_sample_point(iout(1).stat_bus, false);
        end loop;
        wait for 20 ns; -- To be sure sample point was processed!
        
        read_error_counters(err_counters_4, ID_1, mem_bus(1));
        check(err_counters_3.tx_counter = err_counters_4.tx_counter,
            "TEC not incremented after first 7 bits");

        CAN_wait_sample_point(iout(1).stat_bus, false);
        wait for 20 ns; -- To be sure sample point was processed!
        read_error_counters(err_counters_5, ID_1, mem_bus(1));
        
        check(err_counters_3.tx_counter + 8 = err_counters_5.tx_counter,
            "TEC incremented after next 8 dominant bits!");
            
        release_bus_level(so.bl_force);
    end procedure;


    procedure error_rules_f_tx_feature_exec(
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
        -- 1. Set Node 2 to ACK forbidden mode. Send frame by Node 1. Wait until
        --    Error frame is sent by Node 1. Force bus low and wait for 7 bits.
        --    Check that Error counter is still the same as after Error frame
        --    started! Wait for one more bit and check that TEC counter was
        --    incremented by 8! Wait for 7 bits and check that TEC remains the
        --    same. Wait for one more bit and check that TEC is incremented
        --    again by 8. Release the bus and Wait until bus is idle!
        -----------------------------------------------------------------------
        info("Step 1");
        
        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, ID_2, mem_bus(2));
        
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_error_frame(ID_1, mem_bus(1));
        
        do_14_8_check(so, iout, mem_bus);
        
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));
        
        wait for 2000 ns;
        
        -----------------------------------------------------------------------
        -- 2. Set Node 1 to Error passive. Send frame by Node 1 and wait until
        --    Error frame is sent. Repeat the same procedure as in Step 1.
        --    Afterwards release the bus and wait until bus is idle!
        -----------------------------------------------------------------------
        info("Step 2");
        
        mode_1.test := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));
        
        err_counters_1.tx_counter := 150;
        err_counters_1.rx_counter := 0;
        
        set_error_counters(err_counters_1, ID_1, mem_bus(1));
        
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_error_frame(ID_1, mem_bus(1));
        
        do_14_8_check(so, iout, mem_bus);
        
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));
        
        wait for 2000 ns;
        
        -----------------------------------------------------------------------
        -- 3. Unset ACK forbidden mode in Node 2. Send frame by Node 1. Wait
        --    until Intermission of Node 1 and force bus low. Wait until
        --    Overload frame is sent. Repeat the procedure from step 1. Release
        --    the bus and wait until bus is idle!
        -----------------------------------------------------------------------
        info("Step 3");
        
        mode_2.acknowledge_forbidden := false;
        set_core_mode(mode_2, ID_2, mem_bus(2));
        
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        
        CAN_wait_pc_state(pc_deb_intermission, ID_1, mem_bus(1));
        force_bus_level(DOMINANT, so.bl_force, so.bl_inject);
        CAN_wait_pc_state(pc_deb_overload, ID_1, mem_bus(1));
        release_bus_level(so.bl_force);
        
        do_14_8_check(so, iout, mem_bus);
        
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

    end procedure;

end package body;