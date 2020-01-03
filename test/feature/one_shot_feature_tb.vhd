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
--  One shot mode feature test (Retransmitt limit = 0).
--
-- Verifies:
--  1. One shot mode - Retransmitt limit enabled and set to 0. Verifies there is
--     only one atempt to transmitt a frame in one shot mode
--  2. When One shot mode is not set (retransmit limit = 0, but disabled),
--     core does not stop re-transmitting after retransmitt limit number of
--     retransmissions was reached (retransmitts indefinitely).
--  3. When transmission fails as result of Error frame, device in One shot mode
--     does not transmitt anymore!
--  4. When transmission fails as result of Arbitration loss, device in One shot
--     mode does not transmitt anymore!
--
-- Test sequence:
--  1. Set retransmitt limit to 0 in Node 1. Enable retransmitt limitations.
--     Set Acknowledge forbidden mode in Node 2 (to produce ACK errors). Turn
--     on Test mode in Node 1 (to manipulate error counters).
--  2. Generate frame and start sending the frame by Node 1. Wait until
--     error frame occurs and transmission is over.
--  3. Check transmission failed and transmitting TXT Buffer is "TX Error".
--  4. Disable retransmitt limitions in Node 1. Start sending a frame by Node 1.
--     Wait until error frame and check that transmitting TXT Buffer is "Ready"
--     again (hitting current retransmitt limit did not cause stopping
--     retransmissions when retransmitt limit is disabled).
--  5. Abort transmission by Node 1. Wait until transmission was aborted.
--  6. Insert frames for transmission to Node 1 and Node 2 simultaneously
--     to invoke arbitration. ID of frame in Node 1 is higher than the one in
--     Node 2 (to loose arbitration). Wait until node 1 is in Control field of
--     a frame. Check that Node 1 is receiver (arbitration was really lost) and
--     TXT Buffer in Node 1 ended up in "TX Error" state.
--------------------------------------------------------------------------------
-- Revision History:
--    06.7.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package one_shot_feature is
    procedure one_shot_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body one_shot_feature is
    procedure one_shot_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable ID_1           	:       natural := 1;
        variable ID_2           	:       natural := 2;
        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        variable buf_state          :       SW_TXT_Buffer_state_type;
        variable status             :       SW_status;
    begin

        ------------------------------------------------------------------------
        -- 1. Set retransmitt limit to 0 in Node 1. Enable retransmitt 
        --    limitations. Set Acknowledge forbidden mode in Node 2 (to produce
        --    ACK errors). Turn on Test mode in Node 1 (to manipulate error 
        --    counters).
        ------------------------------------------------------------------------
        info("Step 1: Configuring One shot Mode (Node 1), ACF (Node 2)");
        CAN_enable_retr_limit(true, 0, ID_1, mem_bus(1));
        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, ID_2, mem_bus(2));
        mode_1.test := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));
        
        ------------------------------------------------------------------------
        -- 2. Generate frame and start sending the frame by Node 1. Wait until
        --    error frame occurs and transmission is over.
        ------------------------------------------------------------------------
        info("Step 2: Sending frame by Node 1");
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_frame.rtr := RTR_FRAME; -- Use RTR frame to save simulation time
        CAN_frame.frame_format := NORMAL_CAN;
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_error_frame(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- 3. Check transmission failed and transmitting TXT Buffer is
        --    "TX Error".
        ------------------------------------------------------------------------
        info("Step 3: Checking transmission failed.");
        get_tx_buf_state(1, buf_state, ID_1, mem_bus(1));
        check(buf_state = buf_failed, "TXT Buffer failed!");
        
        ------------------------------------------------------------------------
        -- 4. Disable retransmitt limitions in Node 1. Start sending a frame by
        --    Node 1. Wait until error frame and check that transmitting TXT
        --    Buffer is "Ready" again (hitting current retransmitt limit did not
        --    cause stopping retransmissions when retransmitt limit is disabled).
        ------------------------------------------------------------------------
        info("Step 4: Testing disabled One shot mode");
        CAN_enable_retr_limit(false, 0, ID_1, mem_bus(1));
        CAN_send_frame(CAN_frame, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_error_frame(ID_1, mem_bus(1));
        get_tx_buf_state(1, buf_state, ID_1, mem_bus(1));
        check(buf_state = buf_ready, "TXT Buffer ready!");
        
        ------------------------------------------------------------------------
        -- 5. Abort transmission by Node 1. Wait until transmission was aborted.
        ------------------------------------------------------------------------
        info("Step 5: Aborting transmission");
        send_TXT_buf_cmd(buf_set_abort, 1, ID_1, mem_bus(1));
        get_tx_buf_state(1, buf_state, ID_1, mem_bus(1));
        while (buf_state /= buf_aborted) loop
            get_tx_buf_state(1, buf_state, ID_1, mem_bus(1));
        end loop;        
        CAN_wait_bus_idle(ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- 6. Insert frames for transmission to Node 1 and Node 2 simultaneously
        --    to invoke arbitration. ID of frame in Node 1 is higher than the
        --    one in Node 2 (to loose arbitration). Wait until node 1 is in 
        --    Control field of a frame. Check that Node 1 is receiver 
        --    (arbitration was really lost) and TXT Buffer in Node 1 ended up
        --    in "TX Error" state.
        ------------------------------------------------------------------------
        info("Step 6: Testing One shot due to arbitration loss!");
        CAN_enable_retr_limit(true, 0, ID_1, mem_bus(1));
        CAN_frame.ident_type := BASE;
        CAN_frame.identifier := 10;
        CAN_insert_TX_frame(CAN_frame, 1, ID_1, mem_bus(1));
        CAN_frame.identifier := 9;
        CAN_insert_TX_frame(CAN_frame, 1, ID_2, mem_bus(2));
        
        -- TODO: Use atomic procedure after priority test is merged to be sure!
        send_TXT_buf_cmd(buf_set_ready, 1, ID_1, mem_bus(1));
        send_TXT_buf_cmd(buf_set_ready, 1, ID_2, mem_bus(2));
        
        CAN_wait_pc_state(pc_deb_control, ID_1, mem_bus(1));
        get_controller_status(status, ID_1, mem_bus(1));
        check(status.receiver, "Node 1 lost arbitration");
        get_tx_buf_state(1, buf_state, ID_1, mem_bus(1));
        check(buf_state = buf_failed, "TXT Buffer failed");
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        
        wait for 1000 ns;
        
  end procedure;

end package body;