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
--  Self test mode - feature test.
--
-- @Verifies:
--  @1. In Self test mode, transmitted frame is valid even if ACK was recessive!
--
-- @Test sequence:
--  @1. Configures Self Test mode in Node 1. Configure ACK forbidden in Node 2.
--  @2. Send frame by Node 1. Wait till ACK field.
--  @3. Monitor during whole ACK field that frame bus is RECESSIVE.
--  @4. Check that after ACK field, Node 1 is NOT transmitting Error frame!
--      Wait till bus is idle and check that TXT Buffer in Node 1 is in TX OK!
--      Check that frame was received by Node 2.
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

package mode_self_test_feature is
    procedure mode_self_test_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body mode_self_test_feature is
    procedure mode_self_test_feature_exec(
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

        ------------------------------------------------------------------------
        -- @1. Configures Self Test mode in Node 1. Configure ACK forbidden in
        --    Node 2.
        ------------------------------------------------------------------------
        info("Step 1: Configuring STM in Node 1, ACF in Node 2!");
        mode_1.self_test := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));
        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, ID_2, mem_bus(2));

        ------------------------------------------------------------------------
        -- @2. Send frame by Node 1. Wait till ACK field.
        ------------------------------------------------------------------------
        info("Step 2: Send frame by Node 1, Wait till ACK");
        CAN_generate_frame(rand_ctr, CAN_TX_frame);
        CAN_send_frame(CAN_TX_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_pc_state(pc_deb_ack, ID_1, mem_bus(1));
        
        ------------------------------------------------------------------------
        -- @3. Monitor during whole ACK field that frame bus is RECESSIVE.
        ------------------------------------------------------------------------
        info("Step 3: Checking ACK field is recessive"); 
        CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
        while (pc_dbg = pc_deb_ack) loop
            check(bus_level = RECESSIVE, "Dominant ACK transmitted!");
            CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
            
            get_controller_status(status, ID_1, mem_bus(1));
            check(status.transmitter, "Node 1 receiver!");
            
            wait for 100 ns; -- To make checks more sparse
        end loop;
        
        ------------------------------------------------------------------------
        -- @4. Check that after ACK field, Node 1 is NOT transmitting Error 
        --    frame! Wait till bus is idle and check that TXT Buffer in Node 1
        --    is in TX OK! Check that frame was received by Node 2.
        ------------------------------------------------------------------------
        info("Step 4: Check Error frame is not transmitted!"); 
        get_controller_status(status, ID_1, mem_bus(1));
        check_false(status.error_transmission, "Error frame not transmitted!");
        CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
        
        -- For CAN FD frames secondary ACK is still marked as ACK to DEBUG
        -- register! From there if this is recessive (it is now, no ACK is sent),
        -- it is interpreted as ACK Delimiter and we move directly to EOF! This
        -- is OK!
        check(pc_dbg = pc_deb_ack_delim or
              pc_dbg = pc_deb_eof, "ACK delimiter follows recessive ACK!");
        
        CAN_wait_bus_idle(ID_2, mem_bus(2));
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        
        get_tx_buf_state(1, txt_buf_state, ID_1, mem_bus(1));
        check(txt_buf_state = buf_done, "Frame transmitted OK");
        get_rx_buf_state(rx_buf_state, ID_2, mem_bus(2));
        check(rx_buf_state.rx_frame_count = 1, "Frame received in LOM");
        CAN_read_frame(CAN_RX_frame, ID_2, mem_bus(2));
        CAN_compare_frames(CAN_RX_frame, CAN_TX_frame, false, frames_equal);
        
        wait for 1000 ns;
        
  end procedure;

end package body;