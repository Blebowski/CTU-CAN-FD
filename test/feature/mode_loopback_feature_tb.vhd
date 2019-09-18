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
--  Loopback mode - feature test.
--
-- Verifies:
--  1. Transmitted CAN frame will be also received in Loopback mode when frame
--     is transmitted successfully.
--  2. Transmitted CAN frame will not be received in Loopback mode when error
--     frame occurs.
--  3. Transmitted CAN frame will not be received be node itself when Loopback
--     mode was disabled.
--
-- Test sequence:
--  1. Configure Loopback mode in Node 1.
--  2. Generate random CAN frame and send it by Node 1.
--  3. Wait until frame is received. Check that Node 1 has 1 frame in RX Buffer.
--  4. Read CAN frame from Node 1. Check it is the same as transmitted frame.
--     Check that there are 0 frames in RX Buffer of Node 1. Read also frame
--     from Node 2 not to leave it hanging there!
--  5. Set Node 2 to Acknowledge forbidden mode. Set Node 1 to one shot mode.
--  6. Generate random CAN frame and send it by Node 1.
--  7. Wait until transmission is over. Check that TXT Buffer used for transmi-
--     ssion is in TX failed. Check that RX Buffer in Node 1 has no frame.
--  8. Disable Loopback mode in Node 1. Disable Acknowledge forbidden mode in
--     Node 2.
--  9. Send CAN frame by Node 1. Wait until frame is over.
-- 10. Check that RX Buffer of Node 1 has no CAN frame received. Check that
--     RX Buffer of Node 2 has frame received.
--------------------------------------------------------------------------------
-- Revision History:
--    18.9.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package mode_loopback_feature is
    procedure mode_loopback_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body mode_loopback_feature is
    procedure mode_loopback_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable r_data             :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable CAN_TX_frame       :       SW_CAN_frame_type;
        variable CAN_RX_frame       :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable ctr_1              :       natural;
        variable ctr_2              :       natural;
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;
        variable rand_val           :       real;
        variable retr_th            :       natural;
        variable mode_backup        :       std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');
        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        variable err_counters       :       SW_error_counters := (0, 0, 0, 0);
        variable txt_buf_state      :       SW_TXT_Buffer_state_type;
        variable rx_buf_state       :       SW_RX_Buffer_info;
        variable status             :       SW_status;
        variable frames_equal       :       boolean := false;
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- 1. Configure Loopback mode in Node 1.
        ------------------------------------------------------------------------
        info("Step 1: Configuring Loopback mode in Node 1");
        mode_1.internal_loopback := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- 2. Generate random CAN frame and send it by Node 1.
        ------------------------------------------------------------------------
        info("Step 2: Sending frame by Node 1");
        CAN_generate_frame(rand_ctr, CAN_TX_frame);
        CAN_send_frame(CAN_TX_frame, 1, ID_1, mem_bus(1), frame_sent);
        
        ------------------------------------------------------------------------
        -- 3. Wait until frame is received. Check that Node 1 has 1 frame in 
        --    RX Buffer.
        ------------------------------------------------------------------------
        info("Step 3: Waiting until frame is sent");
        CAN_wait_frame_sent(ID_1, mem_bus(1));
        get_rx_buf_state(rx_buf_state, ID_1, mem_bus(1));
        check(rx_buf_state.rx_frame_count = 1, "Own frame in Loopback received");

        ------------------------------------------------------------------------
        -- 4. Read CAN frame from Node 1. Check it is the same as transmitted 
        --    frame. Check that there are 0 frames in RX Buffer of Node 1.
        ------------------------------------------------------------------------
        info("Step 4: Read own transmitted frame from RX Buffer");
        CAN_read_frame(CAN_RX_frame, ID_1, mem_bus(1));
        CAN_compare_frames(CAN_RX_frame, CAN_TX_frame, false, frames_equal);
        check(frames_equal, "Own frame in Loopback is the same as sent!");
        get_rx_buf_state(rx_buf_state, ID_1, mem_bus(1));
        check(rx_buf_state.rx_frame_count = 0, "Own frame read from RX Buffer");
        CAN_read_frame(CAN_RX_frame, ID_2, mem_bus(2));
        
        ------------------------------------------------------------------------
        -- 5. Set Node 2 to Acknowledge forbidden mode. Set Node 1 to one shot
        --    mode.
        ------------------------------------------------------------------------
        info("Step 5: Configure Node 2 to ACF, Node 1 to One shot mode");
        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, ID_2, mem_bus(2));
        CAN_enable_retr_limit(true, 0, ID_1, mem_bus(1));
        
        ------------------------------------------------------------------------
        -- 6. Generate random CAN frame and send it by Node 1.
        ------------------------------------------------------------------------
        info("Step 6: Send frame by Node 1!");
        CAN_generate_frame(rand_ctr, CAN_TX_frame);
        CAN_send_frame(CAN_TX_frame, 1, ID_1, mem_bus(1), frame_sent);
        
        ------------------------------------------------------------------------
        -- 7. Wait until transmission is over. Check that TXT Buffer used for 
        --    transmission is in TX failed. Check that RX Buffer in Node 1 has 
        --    no frame.
        ------------------------------------------------------------------------
        info("Step 7: Check no own frame received on Error frame!");
        CAN_wait_error_frame(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        get_tx_buf_state(1, txt_buf_state, ID_1, mem_bus(1));
        check(txt_buf_state = buf_failed, "TXT Buffer failed");
        get_rx_buf_state(rx_buf_state, ID_1, mem_bus(1));
        check(rx_buf_state.rx_frame_count = 0,
            "No own frame received when error frame was received!");

        ------------------------------------------------------------------------
        -- 8. Disable Loopback mode in Node 1. Disable Acknowledge forbidden 
        --    mode in Node 2.
        ------------------------------------------------------------------------
        info("Step 8: Disable Loopback in Node 1. Disable ACF in Node 2!");
        mode_1.internal_loopback := false;
        set_core_mode(mode_1, ID_1, mem_bus(1));
        mode_2.acknowledge_forbidden := false;
        set_core_mode(mode_2, ID_2, mem_bus(2));
        
        ------------------------------------------------------------------------
        -- 9. Send CAN frame by Node 1. Wait until frame is over.
        ------------------------------------------------------------------------
        info("Step 9: Send CAN frame by Node 1.");
        CAN_generate_frame(rand_ctr, CAN_TX_frame);
        CAN_send_frame(CAN_TX_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_frame_sent(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        
        ------------------------------------------------------------------------
        -- 10. Check that RX Buffer of Node 1 has no CAN frame received. Check 
        --     that RX Buffer of Node 2 has frame received.
        ------------------------------------------------------------------------
        info("Step 10: Check own frame not received when Loopback is disabled");
        get_rx_buf_state(rx_buf_state, ID_1, mem_bus(1));
        check(rx_buf_state.rx_frame_count = 0,
            "Own frame not received when Loopback mode is disabled!");
        get_rx_buf_state(rx_buf_state, ID_2, mem_bus(2));
        check(rx_buf_state.rx_frame_count = 1,
            "Frame received in Node 2!");
        CAN_read_frame(CAN_RX_frame, ID_2, mem_bus(2));
        CAN_compare_frames(CAN_RX_frame, CAN_TX_frame, false, frames_equal);
        check(frames_equal, "TX vs. RX frame matching!");

        wait for 1000 ns;
        
  end procedure;

end package body;