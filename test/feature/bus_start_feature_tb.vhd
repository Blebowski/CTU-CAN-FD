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
--  Bus start feature test
--
-- @Verifies:
--  @1. CTU CAN FD can integrate to bus comunication within 11 consecutive
--      recessive bits!
--
-- @Test sequence:
--  @1. Disable both Nodes. Insert 2 frames to Node 1. Check both Nodes are 
--      Bus off. Enable Node 1.
--  @2. Wait till sample point in Node 1 11 times, check that after 11 recesive
--      bits, Node 1 becomes error active. Wait until Node 1 becomes transmitter.
--  @3. Enable Node 2, wait until ACK field in Node 2. Force the bus low so that
--      Node 1 receives ACK. Wait till Node 1 is not in ACK anymore. Check it
--      is in ACK Delimiter!
--  @4. Wait for 11 sample points in Node 2. Check that Node 2 became Error
--      active (this should have occurred in ACK Delimiter + EOF + Intermission
--      of Node 1).
--  @5. Wait until CAN frame starts in Node 2. Check Node 2 turned receiver!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    30.8.2018   Created file
--   18.10.2019   Re-wrote to be comformant to new test format. Be more strict
--                in Bit time length measurements!
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package bus_start_feature is
    procedure bus_start_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body bus_start_feature is
    procedure bus_start_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;
        variable CAN_frame_1        :       SW_CAN_frame_type;
        variable CAN_frame_2        :       SW_CAN_frame_type;
        
        variable fault_state_1      :       SW_fault_state;
        variable fault_state_2      :       SW_fault_state;
        
        variable read_state         :       SW_PC_Debug;
        variable status             :       SW_status;

    begin

        ------------------------------------------------------------------------
        -- @1. Disable both Nodes. Insert 2 frames to Node 1. Check both Nodes
        --    are Bus off. Enable Node 1.
        ------------------------------------------------------------------------
        info("Step 1: Disable both nodes!");
        get_fault_state(fault_state_1, ID_1, mem_bus(1));
        get_fault_state(fault_state_2, ID_2, mem_bus(2));
        check(fault_state_1 = fc_error_active, "Node 1 Error active!");
        check(fault_state_2 = fc_error_active, "Node 2 Error active!");
        
        CAN_turn_controller(false, ID_1, mem_bus(1));
        CAN_turn_controller(false, ID_2, mem_bus(2));
        CAN_generate_frame(rand_ctr, CAN_frame_1);
        CAN_generate_frame(rand_ctr, CAN_frame_2);
        CAN_insert_TX_frame(CAN_frame_1, 1, ID_1, mem_bus(1));
        CAN_insert_TX_frame(CAN_frame_2, 2, ID_1, mem_bus(1));

        get_fault_state(fault_state_1, ID_1, mem_bus(1));
        get_fault_state(fault_state_2, ID_2, mem_bus(2));
        check(fault_state_1 = fc_bus_off, "Node 1 Bus off!");
        check(fault_state_2 = fc_bus_off, "Node 2 Bus off!");
        
        CAN_turn_controller(true, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- @2. Wait till sample point in Node 1 11 times, check that after 11
        --    recessive bits, Node 1 becomes error active. Wait until Node 1
        --    becomes transmitter.
        ------------------------------------------------------------------------
        info("Step 2: Check integration of Node 1");
        for i in 0 to 10 loop
            CAN_wait_sample_point(iout(1).stat_bus);
            wait for 21 ns; -- For DFF to take place
            get_fault_state(fault_state_1, ID_1, mem_bus(1));
            
            if (i = 10) then
                check(fault_state_1 = fc_error_active,
                    "Node error active after 11 bits of integration!");    
            else
                check(fault_state_1 = fc_bus_off,
                    "Node bus off before 11 bits of integration!");    
            end if;
        end loop;
        send_TXT_buf_cmd(buf_set_ready, 1, ID_1, mem_bus(1));
        send_TXT_buf_cmd(buf_set_ready, 2, ID_1, mem_bus(1));
        CAN_wait_tx_rx_start(true, false, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- @3. Enable Node 2, wait until ACK field in Node 2. Force the bus low
        --    so that Node 1 receives ACK. Wait till Node 1 is not in ACK 
        --    anymore. Check it is in ACK Delimiter!
        ------------------------------------------------------------------------
        info("Step 3: Enable node 2");
        CAN_turn_controller(true, ID_2, mem_bus(2));
        CAN_wait_pc_state(pc_deb_ack, ID_1, mem_bus(1));
        force_bus_level(DOMINANT, so.bl_force, so.bl_inject);
        CAN_wait_not_pc_state(pc_deb_ack, ID_1, mem_bus(1));
        release_bus_level(so.bl_force);

        CAN_read_pc_debug(read_state, ID_1, mem_bus(1));
        check(read_state = pc_deb_ack_delim, "Node 2 is in ACK delimiter!");

        ------------------------------------------------------------------------
        -- @4. Wait for 11 sample points in Node 2. Check that Node 2 became
        --    Error active (this should have occurred in ACK Delimiter + EOF +
        --    Intermission of Node 1).
        ------------------------------------------------------------------------
        info("Step 4: Check integration of Node 2");
        for i in 0 to 10 loop
            CAN_wait_sample_point(iout(2).stat_bus);
            wait for 21 ns; -- For DFF to take place
            get_fault_state(fault_state_1, ID_2, mem_bus(2));
            
            if (i = 10) then
                check(fault_state_1 = fc_error_active,
                    "Node error active after 11 bits of integration!");    
            else
                check(fault_state_1 = fc_bus_off,
                    "Node bus off before 11 bits of integration!");    
            end if;
        end loop;

        ------------------------------------------------------------------------
        -- @5. Wait until CAN frame starts in Node 1. Check Node 1 turned
        --    transmitter!
        ------------------------------------------------------------------------
        info("Step 5: Check Node 2 joined bus communication!");
        CAN_wait_tx_rx_start(true, true, ID_2, mem_bus(2));
        get_controller_status(status, ID_2, mem_bus(2));
        
        check(status.receiver, "Node 2 joined bus communication!");


  end procedure;

end package body;