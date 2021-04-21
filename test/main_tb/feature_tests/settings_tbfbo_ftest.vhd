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
--  @1. Configure SETTINGS[TBFBO]=1 in DUT. Set TEC of DUT to 254. Send
--      frame by DUT.
--  @2. Wait until first bit of EOF and force CAN RX of DUT to Dominant. This
--      creates error condition, and DUT will transmitt error frame. Wait till
--      error frame is over. Check that DUT is bus-off. Check that all its
--      TXT Buffers are in TX Failed state.
--  @3. Turn off DUT, configure SETTINGS[TBFBO]=1 in DUT, then turn it on
--      again. Wait till integration is over.
--      Issue COMMAND[ERCRST] (in advance, shall be captured by DUT) to DUT.
--      Set TEC of DUT to 254.
--  @4. Send frame by DUT. Wait till first bit of EOF, force CAN RX of DUT
--      to Dominant value. Wait till error frame is over, check that DUT is
--      bus-off. Check that TX Buffer that was used for transmission is in
--      TX ready.
--  @5. Poll on fault confinement state of DUT until in comes out of bus-off.
--      When it happends, check that Node becomes transmitter on TXT Buffer used
--      for transmission before went to "TX in progress" state.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    22.9.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.mem_bus_agent_pkg.all;

package settings_tbfbo_ftest is
    procedure settings_tbfbo_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body settings_tbfbo_ftest is
    procedure settings_tbfbo_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_TX_frame       :       SW_CAN_frame_type;
        variable CAN_RX_frame       :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;

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
        -- @1. Configure SETTINGS[TBFBO]=1 in DUT. Set TEC of DUT to 254.
        --     Send frame by DUT.
        -----------------------------------------------------------------------
        info_m("Step 1");

        -- Choose random TXT Buffer
        rand_int_v(4, buf_index);

        mode_1.tx_buf_bus_off_failed := true;
        mode_1.test := true; -- Needed to set TEC!  
        set_core_mode(mode_1, DUT_NODE, chn);

        read_error_counters(err_counters, DUT_NODE, chn);
        err_counters.tx_counter := 254;
        set_error_counters(err_counters, DUT_NODE, chn);

        CAN_generate_frame(CAN_TX_frame);
        CAN_send_frame(CAN_TX_frame, buf_index, DUT_NODE, chn, frame_sent);

        -----------------------------------------------------------------------
        -- @2. Wait until first bit of EOF and force CAN RX of DUT to
        --     Dominant. This creates error condition, and DUT will transmitt
        --     error frame. Wait till error frame is over. Check that DUT is
        --     bus-off. Check that all its TXT Buffers are in TX Failed state.
        -----------------------------------------------------------------------
        info_m("Step 2");
        
        CAN_wait_pc_state(pc_deb_eof, DUT_NODE, chn);
        force_can_rx(DOMINANT, DUT_NODE, chn);
        CAN_wait_sync_seg(DUT_NODE, chn);
        CAN_wait_sync_seg(DUT_NODE, chn);
        release_can_rx(chn);
        
        get_controller_status(status, DUT_NODE, chn);
        mem_bus_agent_disable_transaction_reporting(chn);
        check_m(status.error_transmission, "Error frame sent");
        while (status.error_transmission) loop
            get_controller_status(status, DUT_NODE, chn);
        end loop;
        mem_bus_agent_enable_transaction_reporting(chn);

        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_bus_off, "Node bus-off");
        
        get_tx_buf_state(buf_index, txt_buf_state, DUT_NODE, chn);
        check_m(txt_buf_state = buf_failed, "TXT Buffer is failed");
        
        -----------------------------------------------------------------------
        -- @3. Turn off DUT, configure SETTINGS[TBFBO]=1 in DUT, then
        --     turn it on again. Wait till integration is over. Issue
        --     COMMAND[ERCRST] (in advance, shall be captured by DUT) to
        --     DUT. Set TEC of DUT to 254.
        -----------------------------------------------------------------------
        info_m("Step 3");
        
        CAN_turn_controller(false, DUT_NODE, chn);
        
        mode_1.tx_buf_bus_off_failed := false;
        mode_1.test := true; -- Needed to set TEC! 
        set_core_mode(mode_1, DUT_NODE, chn);
        
        -- Must disable one shot mode which is default for feature tests!
        CAN_enable_retr_limit(false, 0, DUT_NODE, chn);
        
        wait for 100 ns;
        CAN_turn_controller(true, DUT_NODE, chn);
        
        CAN_wait_bus_on(DUT_NODE, chn);

        command.err_ctrs_rst := true;
        give_controller_command(command, DUT_NODE, chn);

        read_error_counters(err_counters, DUT_NODE, chn);
        err_counters.tx_counter := 254;
        set_error_counters(err_counters, DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @4. Send frame by DUT. Wait till first bit of EOF, force CAN RX
        --     of DUT to Dominant value. Wait till error frame is over,
        --     check that DUT is bus-off. Check that TX Buffer that was used
        --     for transmission is in TX ready.
        -----------------------------------------------------------------------
        info_m("Step 4");
        
        CAN_send_frame(CAN_TX_frame, buf_index, DUT_NODE, chn, frame_sent);
        
        CAN_wait_pc_state(pc_deb_eof, DUT_NODE, chn);
        wait for 10 ns;
        force_can_rx(DOMINANT, DUT_NODE, chn);
        CAN_wait_sync_seg(DUT_NODE, chn);
        CAN_wait_sync_seg(DUT_NODE, chn);
        release_can_rx(chn);
        
        get_controller_status(status, DUT_NODE, chn);
        check_m(status.error_transmission, "Error frame sent");
        mem_bus_agent_disable_transaction_reporting(chn);
        while (status.error_transmission) loop
            get_controller_status(status, DUT_NODE, chn);
        end loop;
        mem_bus_agent_enable_transaction_reporting(chn);
        
        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_bus_off, "Node bus-off");
        
        get_tx_buf_state(buf_index, txt_buf_state, DUT_NODE, chn);
        check_m(txt_buf_state = buf_ready, "TXT Buffer is ready for next transmission");

        -----------------------------------------------------------------------
        -- @5. Poll on fault confinement state of DUT until in comes out of 
        --     bus-off. When it happends, check that Node becomes transmitter
        --     on TXT Buffer used for transmission before went to 
        --     "TX in progress" state.
        -----------------------------------------------------------------------
        info_m("Step 5");

        CAN_wait_bus_on(DUT_NODE, chn);
        
        -- Could be one bit before it starts to transmitt
        wait for 5000 ns;
        get_controller_status(status, DUT_NODE, chn);
        check_m(status.transmitter, "Node is transmitting!");

        get_tx_buf_state(buf_index, txt_buf_state, DUT_NODE, chn);
        check_m(txt_buf_state = buf_tx_progress, "TXT Buffer is in progress.");
        
  end procedure;

end package body;