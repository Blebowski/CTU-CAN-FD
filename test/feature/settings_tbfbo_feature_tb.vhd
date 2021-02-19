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
--  TX Buffer bus of behavior setting feature test.
--
-- @Verifies:
--  @1. When SETTINGS[TBFBO]=1, then all TXT Buffers will go to TX Failed when
--      node is going to bus-off.
--  @2. When SETTINGS[TBFBO]=0, then transmitting TXT Buffer will go to Ready
--      and after re-integration, node will transmitt again!
--
-- @Test sequence:
--  @1. Configure SETTINGS[TBFBO]=1 in Node 1. Set TEC of Node 1 to 254. Send
--      frame by Node 1.
--  @2. Wait until first bit of EOF and force CAN RX of Node 1 to Dominant. This
--      creates error condition, and Node 1 will transmitt error frame. Wait till
--      error frame is over. Check that Node 1 is bus-off. Check that all its
--      TXT Buffers are in TX Failed state.
--  @3. Turn off Node 1, configure SETTINGS[TBFBO]=1 in Node 1, then turn it on
--      again. Wait till integration is over.
--      Issue COMMAND[ERCRST] (in advance, shall be captured by DUT) to Node 1.
--      Set TEC of Node 1 to 254.
--  @4. Send frame by Node 1. Wait till first bit of EOF, force CAN RX of Node 1
--      to Dominant value. Wait till error frame is over, check that Node 1 is
--      bus-off. Check that TX Buffer that was used for transmission is in
--      TX ready.
--  @5. Poll on fault confinement state of Node 1 until in comes out of bus-off.
--      When it happends, check that Node becomes transmitter on TXT Buffer used
--      for transmission before went to "TX in progress" state.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    22.9.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package settings_tbfbo_feature is
    procedure settings_tbfbo_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body settings_tbfbo_feature is
    procedure settings_tbfbo_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable CAN_TX_frame       :       SW_CAN_frame_type;
        variable CAN_RX_frame       :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;

        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        variable txt_buf_state      :       SW_TXT_Buffer_state_type;
        variable rx_buf_state       :       SW_RX_Buffer_info;
        variable status             :       SW_status;
        variable frames_equal       :       boolean := false;
        variable pc_dbg             :       SW_PC_Debug;
        variable fault_state        :       SW_fault_state;

        variable err_counters       :       SW_error_counters;
        variable buf_index          :       natural;
        
        variable command            :       SW_command := SW_command_rst_val;
    begin

        -----------------------------------------------------------------------
        -- @1. Configure SETTINGS[TBFBO]=1 in Node 1. Set TEC of Node 1 to 254.
        --     Send frame by Node 1.
        -----------------------------------------------------------------------
        info("Step 1");

        -- Choose random TXT Buffer
        rand_int_v(rand_ctr, 4, buf_index);

        mode_1.tx_buf_bus_off_failed := true;
        mode_1.test := true; -- Needed to set TEC!  
        set_core_mode(mode_1, ID_1, mem_bus(1));

        read_error_counters(err_counters, ID_1, mem_bus(1));
        err_counters.tx_counter := 254;
        set_error_counters(err_counters, ID_1, mem_bus(1));

        CAN_generate_frame(rand_ctr, CAN_TX_frame);
        CAN_send_frame(CAN_TX_frame, buf_index, ID_1, mem_bus(1), frame_sent);

        -----------------------------------------------------------------------
        -- @2. Wait until first bit of EOF and force CAN RX of Node 1 to
        --     Dominant. This creates error condition, and Node 1 will transmitt
        --     error frame. Wait till error frame is over. Check that Node 1 is
        --     bus-off. Check that all its TXT Buffers are in TX Failed state.
        -----------------------------------------------------------------------
        info("Step 2");
        
        CAN_wait_pc_state(pc_deb_eof, ID_1, mem_bus(1));
        force_can_rx(DOMINANT, ID_1, so.crx_force, so.crx_inject, so.crx_index);
        CAN_wait_sync_seg(iout(1).stat_bus);
        CAN_wait_sync_seg(iout(1).stat_bus);
        release_can_rx(so.crx_force);
        
        get_controller_status(status, ID_1, mem_bus(1));
        check(status.error_transmission, "Error frame sent");
        while (status.error_transmission) loop
            get_controller_status(status, ID_1, mem_bus(1));
        end loop;

        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_bus_off, "Node bus-off");
        
        get_tx_buf_state(buf_index, txt_buf_state, ID_1, mem_bus(1));
        check(txt_buf_state = buf_failed, "TXT Buffer is failed");
        
        -----------------------------------------------------------------------
        -- @3. Turn off Node 1, configure SETTINGS[TBFBO]=1 in Node 1, then
        --     turn it on again. Wait till integration is over. Issue
        --     COMMAND[ERCRST] (in advance, shall be captured by DUT) to
        --     Node 1. Set TEC of Node 1 to 254.
        -----------------------------------------------------------------------
        info("Step 3");
        
        CAN_turn_controller(false, ID_1, mem_bus(1));
        
        mode_1.tx_buf_bus_off_failed := false;
        mode_1.test := true; -- Needed to set TEC! 
        set_core_mode(mode_1, ID_1, mem_bus(1));
        
        -- Must disable one shot mode which is default for feature tests!
        CAN_enable_retr_limit(false, 0, ID_1, mem_bus(1));
        
        wait for 100 ns;
        CAN_turn_controller(true, ID_1, mem_bus(1));
        
        CAN_wait_bus_on(ID_1, mem_bus(1));

        command.err_ctrs_rst := true;
        give_controller_command(command, ID_1, mem_bus(1));

        read_error_counters(err_counters, ID_1, mem_bus(1));
        err_counters.tx_counter := 254;
        set_error_counters(err_counters, ID_1, mem_bus(1));

        -----------------------------------------------------------------------
        -- @4. Send frame by Node 1. Wait till first bit of EOF, force CAN RX
        --     of Node 1 to Dominant value. Wait till error frame is over,
        --     check that Node 1 is bus-off. Check that TX Buffer that was used
        --     for transmission is in TX ready.
        -----------------------------------------------------------------------
        info("Step 4");
        
        CAN_send_frame(CAN_TX_frame, buf_index, ID_1, mem_bus(1), frame_sent);
        
        CAN_wait_pc_state(pc_deb_eof, ID_1, mem_bus(1));
        force_can_rx(DOMINANT, ID_1, so.crx_force, so.crx_inject, so.crx_index);
        CAN_wait_sync_seg(iout(1).stat_bus);
        CAN_wait_sync_seg(iout(1).stat_bus);
        release_can_rx(so.crx_force);
        
        get_controller_status(status, ID_1, mem_bus(1));
        check(status.error_transmission, "Error frame sent");
        while (status.error_transmission) loop
            get_controller_status(status, ID_1, mem_bus(1));
        end loop;
        
        get_fault_state(fault_state, ID_1, mem_bus(1));
        check(fault_state = fc_bus_off, "Node bus-off");
        
        get_tx_buf_state(buf_index, txt_buf_state, ID_1, mem_bus(1));
        check(txt_buf_state = buf_ready, "TXT Buffer is ready for next transmission");

        -----------------------------------------------------------------------
        -- @5. Poll on fault confinement state of Node 1 until in comes out of 
        --     bus-off. When it happends, check that Node becomes transmitter
        --     on TXT Buffer used for transmission before went to 
        --     "TX in progress" state.
        -----------------------------------------------------------------------
        info("Step 5");

        CAN_wait_bus_on(ID_1, mem_bus(1));
        
        -- Could be one bit before it starts to transmitt
        wait for 5000 ns;
        get_controller_status(status, ID_1, mem_bus(1));
        check(status.transmitter, "Node is transmitting!");

        get_tx_buf_state(buf_index, txt_buf_state, ID_1, mem_bus(1));
        check(txt_buf_state = buf_tx_progress, "TXT Buffer is in progress.");

        wait for 1000 ns;
        
  end procedure;

end package body;