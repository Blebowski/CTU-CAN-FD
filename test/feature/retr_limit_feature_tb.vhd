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
--  Retransmitt limit feature test
--
-- @Verifies:
--  @1. When retransmitt limit is disabled, core does not stop re-transmitting
--      after retransmitt limit number of retransmissions was reached
--      (retransmitts indefinitely).
--  @2. When retransmitt limit is set to 15 (maximum) and retransmitt limitation
--      is enabled, core retransmitts 15 times. After 15 retransmissions, core
--      does not retransmitt anymore.
--  @3. Core retransmitts 'retransmitt limit' times when 'retransmitt limit' is
--      enabled.
--  @4. When transmission fails as result of Error frame, this counts as
--      re-transmission and retransmitt counter is incremented.
--  @5. When transmission fails as result of Arbitration loss, this counts as
--      re-transmission and retransmitt counter is incremented.
--
-- @Test sequence:
--  @1. Set retransmitt limit to 1 in Node 1. Enable retransmitt limitations.
--      Set Acknowledge forbidden mode in Node 2 (to produce ACK errors). Turn
--      on Test mode in Node 1 (to manipulate error counters).
--  @2. Generate frame and start sending the frame by Node 1. Wait until
--      error frame occurs and transmission is over two times.
--  @3. Check transmission failed and transmitting TXT Buffer is "TX Error".
--  @4. Disable retransmitt limitions in Node 1. Start sending a frame by Node 1.
--      Wait until error frame and check that transmitting TXT Buffer is "Ready"
--      again (hitting current retransmitt limit did not cause stopping
--      retransmissions when retransmitt limit is disabled).
--  @5. Abort transmission by Node 1. Wait until transmission was aborted.
--  @6. Generate random retransmitt limit (between 1 and 14). Enable retransmitt
--      limitation in Node 1. Erase TX error counter in Node 1. Erase TX Error
--      counter.
--  @7. Send frame by Node 1. Monitor that after initial transmission and after
--      each next re-transmission sending TXT Buffer in Node 1 is "Ready". After
--      'retransmitt limit' retransmissions check that sending TXT Buffer in
--      Node 1 is in state "TX Error".
--  @8. Check that value of TX Error counter in Node 1 is equal to:
--      (retr_lim + 1) * 8.
--  @9. Set retransmitt limit to 15 and Enable Retransmissions in Node 1.
--      Start Sending frame by Node 1.
-- @10. Monitor that after initial transmission and after each next
--      re-transmission sending TXT Buffer in Node 1 is "Ready". After
--     'retransmitt limit' retransmissions check that sending TXT Buffer in
--      Node 1 is in state "TX Error".
-- @11. Set retransmitt limit to 0 in Node 1. Insert frames for transmission to
--      Node 1 and Node 2 simultaneously to invoke arbitration. ID of frame in
--      Node 1 is higher than the one in Node 2 (to loose arbitration).
--      Wait until node 1 is in Control field of a frame. Check that Node 1
--      is receiver (arbitration was really lost) and TXT Buffer in Node 1
--      ended up in "TX Error" state.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    30.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--    12.06.2018  Modified to use CAN Test lib instead of direct register
--                access functions.
--    06.07.2019  Extended testcase to cover one-shot mode, maximum number of
--                retransmissions in iteration.
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package retr_limit_feature is
    procedure retr_limit_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body retr_limit_feature is
    procedure retr_limit_feature_exec(
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
        variable retr_th            :       natural;
        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        variable err_counters       :       SW_error_counters := (0, 0, 0, 0);
        variable buf_state          :       SW_TXT_Buffer_state_type;
        variable status             :       SW_status;
        variable txt_buf_nr         :       natural range 1 to 4;
    begin

        ------------------------------------------------------------------------
        -- Randomize used TXT Buffer
        ------------------------------------------------------------------------
        rand_int_v(rand_ctr, 3, txt_buf_nr); 
        txt_buf_nr := txt_buf_nr + 1;

        ------------------------------------------------------------------------
        -- @1. Set retransmitt limit to 0 in Node 1. Enable retransmitt 
        --    limitations. Set Acknowledge forbidden mode in Node 2 (to produce
        --    ACK errors). Turn on Test mode in Node 1 (to manipulate error 
        --    counters).
        ------------------------------------------------------------------------
        info("Step 1: Configuring retransmitt limit to 1 (Node 1), ACF (Node 2)");
        CAN_enable_retr_limit(true, 1, ID_1, mem_bus(1));
        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, ID_2, mem_bus(2));
        mode_1.test := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));
        
        ------------------------------------------------------------------------
        -- @2. Generate frame and start sending the frame by Node 1. Wait until
        --    error frame occurs and transmission is over two times.
        ------------------------------------------------------------------------
        info("Step 2: Sending frame by Node 1");
        CAN_generate_frame(rand_ctr, CAN_frame);
        CAN_send_frame(CAN_frame, txt_buf_nr, ID_1, mem_bus(1), frame_sent);
        for i in 0 to 1 loop
            CAN_wait_error_frame(ID_1, mem_bus(1));
            CAN_wait_pc_state(pc_deb_intermission, ID_1, mem_bus(1));
        end loop;

        ------------------------------------------------------------------------
        -- @3. Check transmission failed and transmitting TXT Buffer is
        --    "TX Error".
        ------------------------------------------------------------------------
        info("Step 3: Checking transmission failed.");
        get_tx_buf_state(txt_buf_nr, buf_state, ID_1, mem_bus(1));
        check(buf_state = buf_failed, "TXT Buffer failed!");
        
        ------------------------------------------------------------------------
        -- @4. Disable retransmitt limitions in Node 1. Start sending a frame by
        --    Node 1. Wait until error frame and check that transmitting TXT
        --    Buffer is "Ready" again (hitting current retransmitt limit did not
        --    cause stopping retransmissions when retransmitt limit is disabled).
        ------------------------------------------------------------------------
        info("Step 4: Testing disabled retransmitt limitation");
        CAN_enable_retr_limit(false, 1, ID_1, mem_bus(1));
        CAN_send_frame(CAN_frame, txt_buf_nr, ID_1, mem_bus(1), frame_sent);
        CAN_wait_error_frame(ID_1, mem_bus(1));
        CAN_wait_pc_state(pc_deb_intermission, ID_1, mem_bus(1));
        CAN_wait_error_frame(ID_1, mem_bus(1));
        get_tx_buf_state(txt_buf_nr, buf_state, ID_1, mem_bus(1));
        check(buf_state = buf_ready, "TXT Buffer ready!");
        
        ------------------------------------------------------------------------
        -- @5. Abort transmission by Node 1. Wait until transmission was aborted.
        ------------------------------------------------------------------------
        info("Step 5: Aborting transmission");
        send_TXT_buf_cmd(buf_set_abort, txt_buf_nr, ID_1, mem_bus(1));
        get_tx_buf_state(txt_buf_nr, buf_state, ID_1, mem_bus(1));
        while (buf_state /= buf_aborted) loop
            get_tx_buf_state(txt_buf_nr, buf_state, ID_1, mem_bus(1));
        end loop;        
        CAN_wait_bus_idle(ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- @6. Generate random retransmitt limit (between 1 and 14). Enable 
        --    retransmitt limitation in Node 1. Erase TX Error counter.
        ------------------------------------------------------------------------
        info("Step 6: Setting random retransmitt limit!");
        rand_int_v(rand_ctr, 13, retr_th);
        retr_th := retr_th + 1;
        info("Retransmitt threshold: " & Integer'image(retr_th));
        CAN_enable_retr_limit(true, retr_th, ID_1, mem_bus(1));
        err_counters.tx_counter := 0;
        set_error_counters(err_counters, ID_1, mem_bus(1));
        
        ------------------------------------------------------------------------
        -- @7. Send frame by Node 1. Monitor that after initial transmission and
        --    after each next re-transmission sending TXT Buffer in Node 1 is
        --    "Ready".
        ------------------------------------------------------------------------
        info("Step 7: Checking number of re-transmissions");
        CAN_send_frame(CAN_frame, txt_buf_nr, ID_1, mem_bus(1), frame_sent);
        for i in 0 to retr_th loop
            info("Loop: " & integer'image(i));
            CAN_wait_frame_sent(ID_1, mem_bus(1));
            get_tx_buf_state(txt_buf_nr, buf_state, ID_1, mem_bus(1));
            if (i /= retr_th) then
                check(buf_state = buf_ready, "TXT Buffer ready");
            else
                check(buf_state = buf_failed, "TXT Buffer failed");
            end if;
        end loop;
        CAN_wait_bus_idle(ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- @8. Check that value of TX Error counter in Node 1 is equal to:
        --      (retr_lim + 1) * 8.
        ------------------------------------------------------------------------
        info("Step 8: Checking value of TX Error counter");
        read_error_counters(err_counters, ID_1, mem_bus(1));
        check(err_counters.tx_counter = 8 * (retr_th + 1),
            "Counters exp: " & Integer'Image(err_counters.tx_counter) &
            " counters real: " & Integer'image(8 * (retr_th + 1)));
        
        ------------------------------------------------------------------------
        -- @9. Set retransmitt limit to 15 and Enable Re-transmissions in Node 1.
        --    Start Sending frame by Node 1. Erase error counters so that
        --    we don't go to bus off (just to be sure).
        ------------------------------------------------------------------------
        info("Step 9: Set maximal retransmitt limit (15)");
        CAN_enable_retr_limit(true, 15, ID_1, mem_bus(1));
        CAN_send_frame(CAN_frame, txt_buf_nr, ID_1, mem_bus(1), frame_sent);
        err_counters.tx_counter := 0;
        set_error_counters(err_counters, ID_1, mem_bus(1));
        
        ------------------------------------------------------------------------
        -- @10. Monitor that after initial transmission and after each next
        --     re-transmission sending TXT Buffer in Node 1 is "Ready". After
        --     'retransmitt limit' retransmissions check that sending TXT Buffer
        --     in Node 1 is in state "TX Error".
        ------------------------------------------------------------------------
        info("Step 10: Checking number of re-transmissions");
        for i in 0 to 15 loop
            CAN_wait_frame_sent(ID_1, mem_bus(1));
            get_tx_buf_state(txt_buf_nr, buf_state, ID_1, mem_bus(1));
            if (i /= 15) then
                check(buf_state = buf_ready, "TXT Buffer ready");
            else
                check(buf_state = buf_failed, "TXT Buffer failed");
            end if;
        end loop;
        
        -- Error counters must be erased because unit is already Error Passive!
        err_counters.tx_counter := 0;
        set_error_counters(err_counters, ID_1, mem_bus(1));
        wait for 100 ns;

        ------------------------------------------------------------------------
        -- @11. Set retransmitt limit to 1 in Node 1. Insert frames for 
        --     transmission to Node 1 and Node 2 simultaneously to invoke
        --     arbitration. ID of frame in Node 1 is higher than the one in
        --     Node 2 (to loose arbitration). Wait until node 1 is in Control
        --     field of a frame. Check that Node 1 is receiver (arbitration was
        --     really lost) and TXT Buffer in Node 1 ended up in "TX Error"
        --     state.
        ------------------------------------------------------------------------
        info("Step 11: Testing re-transmitt limit due to arbitration loss!");
        CAN_enable_retr_limit(true, 1, ID_1, mem_bus(1));
        CAN_frame.ident_type := BASE;
        CAN_frame.identifier := 10;
        CAN_insert_TX_frame(CAN_frame, 1, ID_1, mem_bus(1));
        CAN_frame.identifier := 9;
        CAN_insert_TX_frame(CAN_frame, 1, ID_2, mem_bus(2));
        CAN_insert_TX_frame(CAN_frame, 2, ID_2, mem_bus(2));
        
        send_TXT_buf_cmd(buf_set_ready, 1, ID_1, mem_bus(1));
        
        -- Note: There are two frames in Node 2. First one will be transmitted,
        --       because we have one re-transmission, thus Node 2 needs to
        --       send next frame after first one to invoke next arbitration!
        send_TXT_buf_cmd(buf_set_ready, 1, ID_2, mem_bus(2));
        send_TXT_buf_cmd(buf_set_ready, 2, ID_2, mem_bus(2));
        
        CAN_wait_frame_sent(ID_1, mem_bus(1));
        
        CAN_wait_pc_state(pc_deb_control, ID_1, mem_bus(1));
        get_controller_status(status, ID_1, mem_bus(1));
        check(status.receiver, "Node 1 lost arbitration");
        get_tx_buf_state(txt_buf_nr, buf_state, ID_1, mem_bus(1));
        check(buf_state = buf_failed, "TXT Buffer failed");
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        
        wait for 1000 ns;
        
  end procedure;

end package body;