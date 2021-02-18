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
--      @1.1 Configure SETTINGS[PEX]=0 and MODE[FDE]=0 in Node 1. Send CAN FD
--           frame by Node 2.
--      @1.2 Wait until start of r0/FDF bit. Wait one more bit and check that
--           Node 1 is sending error frame. Wait till end of frame.
--      @1.3 Check that STATUS[PEXS] is not set (no Protocol exception occured).
--  @2. Second part - CAN FD Tolerant
--      @2.1 Configure SETTINGS[PEX]=1 and MODE[FDE]=0 in Node 1. Send CAN FD
--           frame by Node 2.
--      @2.2 Wait until start of r0/FDF bit. Wait one more bit and check that
--           Node 1 is NOT sending error frame. Check that node is idle (in inte
--           gration state). Wait till end of frame.
--      @2.3 Check that STATUS[PEXS] is set. Clear it via COMMAND[CPEXS] and
--           check that it has been cleared.
--  @3. Third part - CAN FD Enabled - no protocol exception
--      @3.1 Configure SETTINGS[PEX]=0 and MODE[FDE]=1 in Node 1. Send CAN FD
--           frame by Node 2.
--      @3.2 Wait till start of r0 bit (after FDF) in Node 2 and force CAN RX of
--           Node 1 to Recessive value.
--      @3.3 Wait till start of next bit, release the force, and check that
--           Node 1 is transmitting Error frame. Wait till end of frame.
--      @3.4 Check that STATUS[PEXS] is not set.
--  @4. Fourth part - CAN FD Enabled - protocol exception
--      @4.1 Configure SETTINGS[PEX]=1 and MODE[FDE]=1 in Node 1. Send CAN FD
--           frame by Node 2.
--      @4.2 Wait till start of r0 bit (after FDF) in Node 2 and force CAN RX of
--           Node 1 to Recessive value.
--      @4.3 Wait till start of next bit, release the force, and check that
--           Node 1 is NOT transmitting Error frame and that it is idle.
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
context ctu_can_fd_tb.ctu_can_synth_context;
context ctu_can_fd_tb.ctu_can_test_context;

use ctu_can_fd_tb.pkg_feature_exec_dispath.all;

package mode_pex_feature is
    procedure mode_pex_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body mode_pex_feature is


    procedure check_pex_flag(
        constant expected           : boolean;
        constant ID                 : in    natural range 0 to 15;
        signal   mem_bus            : inout Avalon_mem_type
    ) is
        variable status             :       SW_status;
        variable command            :       SW_command := SW_command_rst_val;
    begin
        get_controller_status(status, ID, mem_bus);
        
        if (expected) then
            check(status.protocol_exception, "STATUS[PEX] is set");
            command.clear_pexs_flag := true;
            give_controller_command(command, ID, mem_bus);
        else
            check_false(status.protocol_exception, "STATUS[PEX] is not set");
        end if;
        
    end procedure;

    procedure wait_till_fdf(
        variable can_frame          : SW_CAN_frame_type;
        signal   stat_bus           : in std_logic_vector(511 downto 0);
        signal   mem_bus            : inout mem_bus_arr_t
    ) is
        variable num_bits_to_wait : natural;
        variable ID_1             : natural := 1;
    begin
        -- Here we assume frame is FD Frame!
        
        if (can_frame.ident_type = BASE) then
            num_bits_to_wait := 14;
        else
            num_bits_to_wait := 33;
        end if;
        
        CAN_wait_tx_rx_start(false, true, ID_1, mem_bus(1));
        for i in 1 to num_bits_to_wait loop
            CAN_wait_sync_seg(stat_bus);
        end loop;
    end procedure;


    procedure configure_pex_fdf(
        constant pex_ena            : boolean;
        constant fde_ena            : boolean;
        signal   mem_bus            : inout mem_bus_arr_t
    ) is
        variable mode               : SW_mode := SW_mode_rst_val;
        variable ID_1               :       natural := 1;
    begin
        get_core_mode(mode, ID_1, mem_bus(1));
        mode.pex_support := pex_ena;
        mode.flexible_data_rate := fde_ena;
        set_core_mode(mode, ID_1, mem_bus(1));
    end procedure;


    procedure mode_pex_feature_exec(
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
    begin

        info("Generating random CAN FD frame");
        CAN_generate_frame(rand_ctr, CAN_TX_frame);
        CAN_TX_frame.frame_format := FD_CAN;

        ------------------------------------------------------------------------
        -- @1. First part - CAN 2.0 - no protocol exception.
        ------------------------------------------------------------------------
        info("Part 1");
        
            --------------------------------------------------------------------
            -- @1.1 Configure SETTINGS[PEX]=0 and MODE[FDE]=0 in Node 1. Send 
            --      CAN FD frame by Node 2.
            --------------------------------------------------------------------
            info("Step 1.1");
            configure_pex_fdf(pex_ena => false, fde_ena => false, mem_bus => mem_bus);
            CAN_send_frame(CAN_TX_frame, 1, ID_2, mem_bus(2), frame_sent);

            --------------------------------------------------------------------
            -- @1.2 Wait until start of r0/FDF bit. Wait one more bit and check
            --      that Node 1 is sending error frame. Wait till end of frame.
            --------------------------------------------------------------------
            info("Step 1.2");
            wait_till_fdf(CAN_TX_frame, iout(1).stat_bus, mem_bus);
            CAN_wait_sync_seg(iout(1).stat_bus);
            get_controller_status(status, ID_1, mem_bus(1));
            check(status.error_transmission, "Error frame being transmitted");
            CAN_wait_bus_idle(ID_1, mem_bus(1));

            -------------------------------------------------------------------
            -- @1.3 Check that STATUS[PEXS] is not set (no Protocol exception 
            --      occured).
            -------------------------------------------------------------------
            info("Step 1.3");
            check_pex_flag(false, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- @2. Second part - CAN FD Tolerant
        ------------------------------------------------------------------------
        info("Part 2"); 

            --------------------------------------------------------------------
            -- @2.1 Configure SETTINGS[PEX]=1 and MODE[FDE]=0 in Node 1. Send 
            --      CAN FD frame by Node 2.
            --------------------------------------------------------------------
            info("Step 2.1");
            configure_pex_fdf(pex_ena => true, fde_ena => false, mem_bus => mem_bus);
            CAN_send_frame(CAN_TX_frame, 1, ID_2, mem_bus(2), frame_sent);

            --------------------------------------------------------------------
            -- @2.2 Wait until start of r0/FDF bit. Wait one more bit and check 
            --      that Node 1 is NOT sending error frame. Check that node is
            --      idle (in inte gration state). Wait till end of frame.
            --------------------------------------------------------------------
            info("Step 2.2");
            wait_till_fdf(CAN_TX_frame, iout(1).stat_bus, mem_bus);
            CAN_wait_sync_seg(iout(1).stat_bus);
            get_controller_status(status, ID_1, mem_bus(1));
            check_false(status.error_transmission, "Error frame not transmitted");
            -- TODO: If we add "Integrating" STATUS bit, use it to check it!
            --check(status.bus_status, "Node is off the bus (integrating)");
            CAN_wait_bus_idle(ID_1, mem_bus(1));

            --------------------------------------------------------------------
            -- @2.3 Check that STATUS[PEXS] is set. Clear it via COMMAND[CPEXS]
            --     and check that it has been cleared.
            --------------------------------------------------------------------
            info("Step 2.3");
            check_pex_flag(true, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- @3. Third part - CAN FD Enabled - no protocol exception.
        ------------------------------------------------------------------------
        info("Part 3");
        
            -------------------------------------------------------------------
            -- @3.1 Configure SETTINGS[PEX]=0 and MODE[FDE]=1 in Node 1. Send
            --      CAN FD frame by Node 2.
            -------------------------------------------------------------------
            info("Step 3.1");
            configure_pex_fdf(pex_ena => false, fde_ena => true, mem_bus => mem_bus);
            CAN_send_frame(CAN_TX_frame, 1, ID_2, mem_bus(2), frame_sent);
            
            -------------------------------------------------------------------
            -- @3.2 Wait till start of r0 bit (after FDF) in Node 2 and force
            --      CAN RX of Node 1 to Recessive value.
            -------------------------------------------------------------------
            info("Step 3.2");
            wait_till_fdf(CAN_TX_frame, iout(1).stat_bus, mem_bus);
            CAN_wait_sync_seg(iout(1).stat_bus);
            force_can_rx(RECESSIVE, ID_1, so.crx_force, so.crx_inject, so.crx_index);

            -------------------------------------------------------------------
            -- @3.3 Wait till start of next bit, release the force, and check
            --      that Node 1 is transmitting Error frame. Wait till end of
            --      frame.
            -------------------------------------------------------------------
            info("Step 3.3");
            CAN_wait_sync_seg(iout(1).stat_bus);
            release_can_rx(so.crx_force);
            get_controller_status(status, ID_1, mem_bus(1));
            check(status.error_transmission, "Error frame transmitted");
            CAN_wait_bus_idle(ID_1, mem_bus(1));

            -------------------------------------------------------------------
            -- @3.4 Check that STATUS[PEXS] is not set.
            -------------------------------------------------------------------
            info("Step 3.4");
            check_pex_flag(false, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- @4. Fourth part - CAN FD Enabled - Protocol exception.
        ------------------------------------------------------------------------
        info("Part 4");
        
            --------------------------------------------------------------------
            -- @4.1 Configure SETTINGS[PEX]=1 and MODE[FDE]=1 in Node 1. Send 
            --      CAN FD frame by Node 2.
            --------------------------------------------------------------------
            info("Part 4.1");
            configure_pex_fdf(pex_ena => true, fde_ena => true, mem_bus => mem_bus);
            CAN_send_frame(CAN_TX_frame, 1, ID_2, mem_bus(2), frame_sent);
            
            --------------------------------------------------------------------
            -- @4.2 Wait till start of r0 bit (after FDF) in Node 2 and force
            --      CAN RX of Node 1 to Recessive value.
            --------------------------------------------------------------------
            info("Part 4.2");
            wait_till_fdf(CAN_TX_frame, iout(1).stat_bus, mem_bus);
            CAN_wait_sync_seg(iout(1).stat_bus);
            force_can_rx(RECESSIVE, ID_1, so.crx_force, so.crx_inject, so.crx_index);

            --------------------------------------------------------------------
            -- @4.3 Wait till start of next bit, release the force, and check
            --      that Node 1 is NOT transmitting Error frame and that it is
            ---     idle. Wait till end of frame.
            --------------------------------------------------------------------
            info("Step 3.3");
            CAN_wait_sync_seg(iout(1).stat_bus);
            release_can_rx(so.crx_force);
            get_controller_status(status, ID_1, mem_bus(1));
            check_false(status.error_transmission, "Error frame NOT transmitted");
            -- TODO: Check integration status bit if added!
            CAN_wait_bus_idle(ID_1, mem_bus(1));
            
            --------------------------------------------------------------------
            -- @4.4 Check that STATUS[PEXS] is set. Clear it via COMMAND[CPEXS]
            --      and check that it has been cleared.
            --------------------------------------------------------------------
            info("Step 4.4");
            check_pex_flag(true, ID_1, mem_bus(1));

        wait for 1000 ns;
        
  end procedure;

end package body;