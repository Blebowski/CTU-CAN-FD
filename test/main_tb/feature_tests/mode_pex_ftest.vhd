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
--  Protocol exception mode test.
--
-- @Verifies:
--  @1. Functionality of Protocol exception mode (SETTINGS[PEX]) in all
--      four combinations : CAN 2.0, FD Tolerant, FD Enabled, FD Enabled with
--      PEX.
--  @2. Functionality of Protocol exception status (STATUS[PEXS]).
--
-- @Test sequence:
--  @1. First part - CAN 2.0 - no protocol exception.
--      @1.1 Configure SETTINGS[PEX]=0 and MODE[FDE]=0 in DUT. Send CAN FD
--           frame by Test node.
--      @1.2 Wait until sample poing of r0/FDF bit. Check that DUT sends error frame.
--           Wait till end of frame.
--      @1.3 Check that STATUS[PEXS] is not set (no Protocol exception occured).
--  @2. Second part - CAN FD Tolerant
--      @2.1 Configure SETTINGS[PEX]=1 and MODE[FDE]=0 in DUT. Send CAN FD
--           frame by Test node.
--      @2.2 Wait until sample point of r0/FDF bit. Check that DUT is NOT sending
--           error frame. Check that node is idle (in integration state).
--           Wait till end of frame.
--      @2.3 Check that STATUS[PEXS] is set. Clear it via COMMAND[CPEXS] and
--           check that it has been cleared.
--  @3. Third part - CAN FD Enabled - no protocol exception
--      @3.1 Configure SETTINGS[PEX]=0 and MODE[FDE]=1 in DUT. Send CAN FD
--           frame by Test node.
--      @3.2 Wait till sample point of FD in Test node and force CAN RX of DUT
--           to Recessive value.
--      @3.3 Wait till sample point of next bit (r0), release the force, and check
--           that DUT is transmitting Error frame. Wait till end of frame.
--      @3.4 Check that STATUS[PEXS] is not set.
--  @4. Fourth part - CAN FD Enabled - protocol exception
--      @4.1 Configure SETTINGS[PEX]=1 and MODE[FDE]=1 in DUT. Send CAN FD
--           frame by Test node.
--      @4.2 Wait till sample point of FDF in Test node and force CAN RX of
--           DUT to Recessive value.
--      @4.3 Wait till next sample point (R0), release the force, and check that
--           DUT is NOT transmitting Error frame and that it is idle.
--           Wait till end of frame.
--      @4.4 Check that STATUS[PEXS] is set. Clear it via COMMAND[CPEXS] and
--           check that it has been cleared.
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

package mode_pex_ftest is
    procedure mode_pex_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body mode_pex_ftest is


    procedure check_pex_flag(
        constant expected           :       boolean;
        constant node               :       t_feature_node;
        signal   chn                : inout t_com_channel
    ) is
        variable status             :       SW_status;
        variable command            :       SW_command := SW_command_rst_val;
    begin
        get_controller_status(status, node, chn);
        
        if (expected) then
            check_m(status.protocol_exception, "STATUS[PEX] is set");
            command.clear_pexs_flag := true;
            give_controller_command(command, node, chn);
        else
            check_false_m(status.protocol_exception, "STATUS[PEX] is not set");
        end if;
        
    end procedure;

    procedure wait_till_fdf(
        variable can_frame          :       SW_CAN_frame_type;
        constant node               :       t_feature_node;
        signal   chn                : inout t_com_channel
    ) is
        variable num_bits_to_wait : natural;
    begin
        -- Here we assume frame is FD Frame!
        
        if (can_frame.ident_type = BASE) then
            num_bits_to_wait := 14;
        else
            num_bits_to_wait := 33;
        end if;
        
        CAN_wait_tx_rx_start(false, true, node, chn);
        wait for 15 ns;
        for i in 1 to num_bits_to_wait loop
            CAN_wait_sample_point(node, chn);
        end loop;
    end procedure;


    procedure configure_pex_fdf(
        constant pex_ena            :       boolean;
        constant fde_ena            :       boolean;
        constant node               :       t_feature_node;
        signal   chn                : inout t_com_channel
    ) is
        variable mode               : SW_mode := SW_mode_rst_val;
    begin
        get_core_mode(mode, DUT_NODE, chn);
        mode.pex_support := pex_ena;
        mode.flexible_data_rate := fde_ena;
        set_core_mode(mode, DUT_NODE, chn);
    end procedure;


    procedure mode_pex_ftest_exec(
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
    begin

        info_m("Generating random CAN FD frame");
        CAN_generate_frame(CAN_TX_frame);
        CAN_TX_frame.frame_format := FD_CAN;

        ------------------------------------------------------------------------
        -- @1. First part - CAN 2.0 - no protocol exception.
        ------------------------------------------------------------------------
        info_m("Part 1");
        
            --------------------------------------------------------------------
            -- @1.1 Configure SETTINGS[PEX]=0 and MODE[FDE]=0 in DUT. Send 
            --      CAN FD frame by Test node.
            --------------------------------------------------------------------
            info_m("Step 1.1");
            
            configure_pex_fdf(pex_ena => false, fde_ena => false,
                              node => DUT_NODE, chn => chn);
            CAN_send_frame(CAN_TX_frame, 1, TEST_NODE, chn, frame_sent);

            --------------------------------------------------------------------
            -- @1.2 Wait until sample poing of r0/FDF bit. Check that DUT sends
            --      error frame. Wait till end of frame.
            --------------------------------------------------------------------
            info_m("Step 1.2");
            
            wait_till_fdf(CAN_TX_frame, DUT_NODE, chn);
            wait for 20 ns;
            get_controller_status(status, DUT_NODE, chn);
            check_m(status.error_transmission, "Error frame being transmitted");
            
            CAN_wait_bus_idle(DUT_NODE, chn);

            -------------------------------------------------------------------
            -- @1.3 Check that STATUS[PEXS] is not set (no Protocol exception 
            --      occured).
            -------------------------------------------------------------------
            info_m("Step 1.3");
            
            check_pex_flag(false, DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- @2. Second part - CAN FD Tolerant
        ------------------------------------------------------------------------
        info_m("Part 2"); 

            --------------------------------------------------------------------
            -- @2.1 Configure SETTINGS[PEX]=1 and MODE[FDE]=0 in DUT. Send 
            --      CAN FD frame by Test node.
            --------------------------------------------------------------------
            info_m("Step 2.1");
            
            configure_pex_fdf(pex_ena => true, fde_ena => false,
                              node => DUT_NODE, chn => chn);
            CAN_send_frame(CAN_TX_frame, 1, TEST_NODE, chn, frame_sent);

            --------------------------------------------------------------------
            -- @2.2 Wait until start of r0/FDF bit. Wait one more bit and check 
            --      that DUT is NOT sending error frame. Check that node is
            --      idle (in inte gration state). Wait till end of frame.
            --------------------------------------------------------------------
            info_m("Step 2.2");
            
            wait_till_fdf(CAN_TX_frame, DUT_NODE, chn);
            wait for 20 ns;
            get_controller_status(status, DUT_NODE, chn);
            check_false_m(status.error_transmission, "Error frame not transmitted");
            
            -- TODO: If we add "Integrating" STATUS bit, use it to check it!
            --check_m(status.bus_status, "Node is off the bus (integrating)");
            CAN_wait_bus_idle(DUT_NODE, chn);

            --------------------------------------------------------------------
            -- @2.3 Check that STATUS[PEXS] is set. Clear it via COMMAND[CPEXS]
            --     and check that it has been cleared.
            --------------------------------------------------------------------
            info_m("Step 2.3");
            
            check_pex_flag(true, DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- @3. Third part - CAN FD Enabled - no protocol exception.
        ------------------------------------------------------------------------
        info_m("Part 3");
        
            -------------------------------------------------------------------
            -- @3.1 Configure SETTINGS[PEX]=0 and MODE[FDE]=1 in DUT. Send
            --      CAN FD frame by Test node.
            -------------------------------------------------------------------
            info_m("Step 3.1");
            
            configure_pex_fdf(pex_ena => false, fde_ena => true,
                              node => DUT_NODE, chn => chn);
            CAN_send_frame(CAN_TX_frame, 1, TEST_NODE, chn, frame_sent);
            
            -------------------------------------------------------------------
            -- @3.2 Wait till sample point of FDF in Test node and force CAN RX
            --      of DUT to Recessive value.
            -------------------------------------------------------------------
            info_m("Step 3.2");
            
            wait_till_fdf(CAN_TX_frame, DUT_NODE, chn);
            force_can_rx(RECESSIVE, DUT_NODE, chn);

            -------------------------------------------------------------------
            -- @3.3 Wait till sample point of next bit (R0), release the force,
            --      and check that DUT is transmitting Error frame. Wait till
            --      end of frame.
            -------------------------------------------------------------------
            info_m("Step 3.3");
            
            CAN_wait_sample_point(DUT_NODE, chn);
            wait for 20 ns;
            release_can_rx(chn);
            get_controller_status(status, DUT_NODE, chn);
            check_m(status.error_transmission, "Error frame transmitted");
            CAN_wait_bus_idle(DUT_NODE, chn);

            -------------------------------------------------------------------
            -- @3.4 Check that STATUS[PEXS] is not set.
            -------------------------------------------------------------------
            info_m("Step 3.4");
            
            check_pex_flag(false, DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- @4. Fourth part - CAN FD Enabled - Protocol exception.
        ------------------------------------------------------------------------
        info_m("Part 4");
        
            --------------------------------------------------------------------
            -- @4.1 Configure SETTINGS[PEX]=1 and MODE[FDE]=1 in DUT. Send 
            --      CAN FD frame by Test node.
            --------------------------------------------------------------------
            info_m("Part 4.1");
            
            configure_pex_fdf(pex_ena => true, fde_ena => true,
                              node => DUT_NODE, chn => chn);
            CAN_send_frame(CAN_TX_frame, 1, TEST_NODE, chn, frame_sent);
            
            --------------------------------------------------------------------
            -- @4.2 Wait till sample point of FDF in Test node and force CAN RX
            --      of DUT to Recessive value.
            --------------------------------------------------------------------
            info_m("Part 4.2");
            
            wait_till_fdf(CAN_TX_frame, DUT_NODE, chn);
            force_can_rx(RECESSIVE, DUT_NODE, chn);

            --------------------------------------------------------------------
            -- @4.3 Wait till sample point of next bit, release the force, and
            --      check that DUT is NOT transmitting Error frame and that it
            ---     is idle. Wait till end of frame.
            --------------------------------------------------------------------
            info_m("Step 4.3");

            CAN_wait_sample_point(DUT_NODE, chn);
            wait for 20 ns;
            release_can_rx(chn);
            get_controller_status(status, DUT_NODE, chn);
            check_false_m(status.error_transmission, "Error frame NOT transmitted");
            
            -- TODO: Check integration status bit if added!
            
            CAN_wait_bus_idle(DUT_NODE, chn);
            
            --------------------------------------------------------------------
            -- @4.4 Check that STATUS[PEXS] is set. Clear it via COMMAND[CPEXS]
            --      and check that it has been cleared.
            --------------------------------------------------------------------
            info_m("Step 4.4");
            
            check_pex_flag(true, DUT_NODE, chn);
        
  end procedure;

end package body;