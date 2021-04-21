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
--  Bus start feature test
--
-- @Verifies:
--  @1. CTU CAN FD can integrate to bus comunication within 11 consecutive
--      recessive bits!
--
-- @Test sequence:
--  @1. Disable both Nodes. Insert 2 frames to DUT. Check both Nodes are 
--      Bus off. Enable DUT.
--  @2. Wait till sample point in DUT 11 times, check that after 11 recesive
--      bits, DUT becomes error active. Wait until DUT becomes transmitter.
--  @3. Enable Test node, wait until ACK field in Test node. Force the bus low so that
--      DUT receives ACK. Wait till DUT is not in ACK anymore. Check it
--      is in ACK Delimiter!
--  @4. Wait for 11 sample points in Test node. Check that Test node became Error
--      active (this should have occurred in ACK Delimiter + EOF + Intermission
--      of DUT).
--  @5. Wait until CAN frame starts in Test node. Check Test node turned receiver!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    30.8.2018   Created file
--   18.10.2019   Re-wrote to be comformant to new test format. Be more strict
--                in Bit time length measurements!
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package bus_start_ftest is
    procedure bus_start_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;

package body bus_start_ftest is
    procedure bus_start_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame_1        :       SW_CAN_frame_type;
        variable CAN_frame_2        :       SW_CAN_frame_type;
        
        variable fault_state_1      :       SW_fault_state;
        variable fault_state_2      :       SW_fault_state;
        
        variable read_state         :       SW_PC_Debug;
        variable status             :       SW_status;
    begin

        ------------------------------------------------------------------------
        -- @1. Disable both Nodes. Insert 2 frames to DUT. Check both Nodes
        --     are Bus off. Enable DUT.
        ------------------------------------------------------------------------
        info_m("Step 1: Disable both nodes!");

        get_fault_state(fault_state_1, DUT_NODE, chn);
        get_fault_state(fault_state_2, TEST_NODE, chn);
        check_m(fault_state_1 = fc_error_active, "DUT Error active!");
        check_m(fault_state_2 = fc_error_active, "Test node Error active!");
        
        CAN_turn_controller(false, DUT_NODE, chn);
        CAN_turn_controller(false, TEST_NODE, chn);
        CAN_generate_frame(CAN_frame_1);
        CAN_generate_frame(CAN_frame_2);
        CAN_insert_TX_frame(CAN_frame_1, 1, DUT_NODE, chn);
        CAN_insert_TX_frame(CAN_frame_2, 2, DUT_NODE, chn);

        get_fault_state(fault_state_1, DUT_NODE, chn);
        get_fault_state(fault_state_2, TEST_NODE, chn);
        check_m(fault_state_1 = fc_bus_off, "DUT Bus off!");
        check_m(fault_state_2 = fc_bus_off, "Test node Bus off!");
        
        CAN_turn_controller(true, DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- @2. Wait till sample point in DUT 11 times, check that after 11
        --     recessive bits, DUT becomes error active. Wait until DUT
        --     becomes transmitter.
        ------------------------------------------------------------------------
        info_m("Step 2: Check integration of DUT");

        for i in 0 to 10 loop
            CAN_wait_sample_point(DUT_NODE, chn);
            wait for 21 ns; -- For DFF to update
            get_fault_state(fault_state_1, DUT_NODE, chn);
            
            if (i = 10) then
                check_m(fault_state_1 = fc_error_active,
                    "Node error active after 11 bits of integration!");    
            else
                check_m(fault_state_1 = fc_bus_off,
                    "Node bus off before 11 bits of integration!");    
            end if;
        end loop;

        send_TXT_buf_cmd(buf_set_ready, 1, DUT_NODE, chn);
        send_TXT_buf_cmd(buf_set_ready, 2, DUT_NODE, chn);
        CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- @3. Enable Test node, wait until ACK field in Test node. Force the
        --     bus low so that DUT receives ACK. Wait till DUT is not in ACK 
        --     anymore. Check it is in ACK Delimiter!
        ------------------------------------------------------------------------
        info_m("Step 3: Enable node 2");

        CAN_turn_controller(true, TEST_NODE, chn);
        CAN_wait_pc_state(pc_deb_ack, DUT_NODE, chn);
        force_bus_level(DOMINANT, chn);
        CAN_wait_not_pc_state(pc_deb_ack, DUT_NODE, chn);
        release_bus_level(chn);

        CAN_read_pc_debug_m(read_state, DUT_NODE, chn);
        check_m(read_state = pc_deb_ack_delim, "Test node is in ACK delimiter!");

        ------------------------------------------------------------------------
        -- @4. Wait for 11 sample points in Test node. Check that Test node became
        --    Error active (this should have occurred in ACK Delimiter + EOF +
        --    Intermission of DUT).
        ------------------------------------------------------------------------
        info_m("Step 4: Check integration of Test node");

        for i in 0 to 10 loop
            CAN_wait_sample_point(TEST_NODE, chn);
            wait for 21 ns; -- For DFF to flip
            get_fault_state(fault_state_1, TEST_NODE, chn);
            
            if (i = 10) then
                check_m(fault_state_1 = fc_error_active,
                    "Node error active after 11 bits of integration!");    
            else
                check_m(fault_state_1 = fc_bus_off,
                    "Node bus off before 11 bits of integration!");    
            end if;
        end loop;

        ------------------------------------------------------------------------
        -- @5. Wait until CAN frame starts in DUT. Check DUT turned
        --    transmitter!
        ------------------------------------------------------------------------
        info_m("Step 5: Check Test node joined bus communication!");

        CAN_wait_tx_rx_start(true, true, TEST_NODE, chn);
        get_controller_status(status, TEST_NODE, chn);
        
        check_m(status.receiver, "Test node joined bus communication!");

  end procedure;

end package body;