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
--  Listen only mode - feature test.
--
-- @Verifies:
--  @1. No frame is transmitted during Listen only mode.
--  @2. No dominant bit is transmitted in Listen only mode. ACK and Error frames
--      are re-reouted internally and bus remains unchanged!
--
-- @Test sequence:
--  @1. Configure Listen only mode in Node 1. Set Self test mode in Node 2.
--  @2. Insert frame for transmission to Node 1. Check for sufficiently long 
--      that it will NOT be transmitted (TXT Buffer remains Ready).
--  @3. Send CAN frame by Node 2. Wait till ACK field.
--  @4. Monitor bus during whole ACK field, check that it is recessive. Wait
--      till bus is idle.
--  @5. Check that frame was received by Node 1.
--  @6. Set Node 1 to NON-ISO FD. Send CAN FD frame by Node 2.
--  @7. Wait till error frame transmitted by Node 1 (should come as CRC is now
--      different).
--  @8. Monitor during whole Error frame transmitted by Node 1, that recessive
--      value is sent on the bus by Node 1! Wait until bus is idle. Check
--      afterwards that frame was sent OK by Node 2.
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

package mode_listen_only_feature is
    procedure mode_listen_only_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body mode_listen_only_feature is
    procedure mode_listen_only_feature_exec(
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
        -- @1. Configure Listen only mode in Node 1. Set Self test mode in Node 2.
        ------------------------------------------------------------------------
        info("Step 1: Configuring LOM in Node 1, STM in Node 2!");
        mode_1.listen_only := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));
        mode_2.self_test := true;
        set_core_mode(mode_2, ID_2, mem_bus(2));
        
        ------------------------------------------------------------------------
        -- @2. Insert frame for transmission to Node 1. Check for sufficiently 
        --    long that it will NOT be transmitted (TXT Buffer remains Ready).
        ------------------------------------------------------------------------
        info("Step 2: Checking frame is not transmitted in Listen only mode!");
        CAN_generate_frame(rand_ctr, CAN_TX_frame);
        CAN_send_frame(CAN_TX_frame, 1, ID_1, mem_bus(1), frame_sent);
        
        for i in 0 to 100 loop
            get_tx_buf_state(1, txt_buf_state, ID_1, mem_bus(1));
            check(txt_buf_state = buf_ready, "TXT buffer remains ready!");
            get_controller_status(status, ID_1, mem_bus(1));
            check_false(status.transmitter, "Node does not transmitt in LOM!");
            check_false(status.receiver, "Node turned receiver in LOM -> WTF?");
            check(status.bus_status, "Node remains idle");
            wait for 50 ns; -- To make checks more sparse
        end loop;
        
        ------------------------------------------------------------------------
        -- @3. Send CAN frame by Node 2. Wait till ACK field of Node 1.
        ------------------------------------------------------------------------
        info("Step 3: Send frame by Node 2, Wait till ACK");
        CAN_generate_frame(rand_ctr, CAN_TX_frame);
        CAN_send_frame(CAN_TX_frame, 1, ID_2, mem_bus(2), frame_sent);
        CAN_wait_pc_state(pc_deb_ack, ID_1, mem_bus(1));
        
        ------------------------------------------------------------------------
        -- @4. Monitor bus during whole ACK field, check that it is recessive.
        --    Wait till bus is idle.
        ------------------------------------------------------------------------       
        info("Step 4: Checking ACK field is recessive"); 
        CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
        while (pc_dbg = pc_deb_ack) loop
            check(bus_level = RECESSIVE, "Dominant ACK transmitted!");
            CAN_read_pc_debug(pc_dbg, ID_1, mem_bus(1));
            
            get_controller_status(status, ID_1, mem_bus(1));
            check(status.receiver, "Node 1 receiver!");
            
            wait for 100 ns; -- To make checks more sparse
        end loop;
        CAN_wait_bus_idle(ID_2, mem_bus(2));
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        
        ------------------------------------------------------------------------
        -- @5. Check that frame was received by Node 1.
        ------------------------------------------------------------------------
        info("Step 5: Checking frame received OK"); 
        get_rx_buf_state(rx_buf_state, ID_1, mem_bus(1));
        check(rx_buf_state.rx_frame_count = 1, "Frame received in LOM");
        CAN_read_frame(CAN_RX_frame, ID_1, mem_bus(1));
        CAN_compare_frames(CAN_RX_frame, CAN_TX_frame, false, frames_equal);
        check(frames_equal, "TX vs. RX frame matching!");
        
        ------------------------------------------------------------------------
        -- @6. Set Node 1 to NON-ISO FD. Send CAN FD frame by Node 2.
        ------------------------------------------------------------------------
        info("Step 6: Set NISOFD for Node 1. Send frame by Node 2"); 
        mode_1.iso_fd_support := false;
        set_core_mode(mode_1, ID_1, mem_bus(1));
        CAN_generate_frame(rand_ctr, CAN_TX_frame);
        CAN_TX_frame.frame_format := FD_CAN;
        CAN_TX_frame.rtr := NO_RTR_FRAME; 
        CAN_send_frame(CAN_TX_frame, 1, ID_2, mem_bus(2), frame_sent);

        ------------------------------------------------------------------------
        -- @7. Wait till error frame transmitted by Node 1 (should come as CRC is
        --    now different).
        ------------------------------------------------------------------------
        info("Step 7: Waiting till error frame");
        CAN_wait_error_frame(ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- @8. Monitor during whole Error frame transmitted by Node 1, that
        --    recessive value is sent on the bus by Node 1! Wait until bus is
        --    idle. Check afterwards that frame was sent OK by Node 2.
        ------------------------------------------------------------------------
        info("Step 8: Monitor that error flag sent by Node 1 is recessive");
        get_controller_status(status, ID_1, mem_bus(1));
        check(status.error_transmission, "Error frame transmitted!");
        while (status.error_transmission) loop
            check(iout(1).can_tx = RECESSIVE, "Dominant Error flag transmitted!");
            get_controller_status(status, ID_1, mem_bus(1));
            wait for 100 ns; -- To make checks more sparse
        end loop;
        CAN_wait_bus_idle(ID_2, mem_bus(2));
        get_tx_buf_state(1, txt_buf_state, ID_2, mem_bus(2));
        check(txt_buf_state = buf_done, "Frame transmitted OK");
        
        wait for 1000 ns;
        
  end procedure;

end package body;