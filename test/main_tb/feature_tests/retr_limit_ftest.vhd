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
--  @1. Set retransmitt limit to 1 in DUT. Enable retransmitt limitations.
--      Set Acknowledge forbidden mode in Test node (to produce ACK errors). Turn
--      on Test mode in DUT (to manipulate error counters).
--  @2. Generate frame and start sending the frame by DUT. Wait until
--      error frame occurs and transmission is over two times.
--  @3. Check transmission failed and transmitting TXT Buffer is "TX Error".
--  @4. Disable retransmitt limitions in DUT. Start sending a frame by DUT.
--      Wait until error frame and check that transmitting TXT Buffer is "Ready"
--      again (hitting current retransmitt limit did not cause stopping
--      retransmissions when retransmitt limit is disabled).
--  @5. Abort transmission by DUT. Wait until transmission was aborted.
--  @6. Generate random retransmitt limit (between 1 and 14). Enable retransmitt
--      limitation in DUT. Erase TX error counter in DUT. Erase TX Error
--      counter.
--  @7. Send frame by DUT. Monitor that after initial transmission and after
--      each next re-transmission sending TXT Buffer in DUT is "Ready". After
--      'retransmitt limit' retransmissions check that sending TXT Buffer in
--      DUT is in state "TX Error".
--  @8. Check that value of TX Error counter in DUT is equal to:
--      (retr_lim + 1) * 8.
--  @9. Set retransmitt limit to 15 and Enable Retransmissions in DUT.
--      Start Sending frame by DUT.
-- @10. Monitor that after initial transmission and after each next
--      re-transmission sending TXT Buffer in DUT is "Ready". After
--     'retransmitt limit' retransmissions check that sending TXT Buffer in
--      DUT is in state "TX Error".
-- @11. Set retransmitt limit to 0 in DUT. Insert frames for transmission to
--      DUT and Test node simultaneously to invoke arbitration. ID of frame in
--      DUT is higher than the one in Test node (to loose arbitration).
--      Wait until node 1 is in Control field of a frame. Check that DUT
--      is receiver (arbitration was really lost) and TXT Buffer in DUT
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

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package retr_limit_ftest is
    procedure retr_limit_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body retr_limit_ftest is
    procedure retr_limit_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable retr_th            :       natural;
        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        variable err_counters       :       SW_error_counters := (0, 0, 0, 0);
        variable buf_state          :       SW_TXT_Buffer_state_type;
        variable status             :       SW_status;
        variable txt_buf_nr         :       natural range 1 to 4;
        variable tmp_int            :       natural;
    begin

        ------------------------------------------------------------------------
        -- Randomize used TXT Buffer
        ------------------------------------------------------------------------
        rand_int_v(3, tmp_int);
        txt_buf_nr := (tmp_int mod 4) + 1;

        ------------------------------------------------------------------------
        -- @1. Set retransmitt limit to 0 in DUT. Enable retransmitt 
        --     limitations. Set Acknowledge forbidden mode in Test node (to
        --     produce ACK errors). Turn on Test mode in DUT (to manipulate 
        --     error counters).
        ------------------------------------------------------------------------
        info_m("Step 1: Configuring retransmitt limit to 1 (DUT), ACF (Test node)");

        CAN_enable_retr_limit(true, 1, DUT_NODE, chn);
        
        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, TEST_NODE, chn);
        
        mode_1.test := true;
        set_core_mode(mode_1, DUT_NODE, chn);
        
        ------------------------------------------------------------------------
        -- @2. Generate frame and start sending the frame by DUT. Wait until
        --     error frame occurs and transmission is over two times.
        ------------------------------------------------------------------------
        info_m("Step 2: Sending frame by DUT");

        CAN_generate_frame(CAN_frame);
        CAN_send_frame(CAN_frame, txt_buf_nr, DUT_NODE, chn, frame_sent);
        for i in 0 to 1 loop
            CAN_wait_error_frame(DUT_NODE, chn);
            CAN_wait_pc_state(pc_deb_intermission, DUT_NODE, chn);
        end loop;

        ------------------------------------------------------------------------
        -- @3. Check transmission failed and transmitting TXT Buffer is
        --     "TX Error".
        ------------------------------------------------------------------------
        info_m("Step 3: Checking transmission failed.");

        get_tx_buf_state(txt_buf_nr, buf_state, DUT_NODE, chn);
        check_m(buf_state = buf_failed, "TXT Buffer failed!");
        
        ------------------------------------------------------------------------
        -- @4. Disable retransmitt limitions in DUT. Start sending a frame by
        --     DUT. Wait until error frame and check that transmitting TXT
        --     Buffer is "Ready" again (hitting current retransmitt limit did not
        --     cause stopping retransmissions when retransmitt limit is disabled).
        ------------------------------------------------------------------------
        info_m("Step 4: Testing disabled retransmitt limitation");

        CAN_enable_retr_limit(false, 1, DUT_NODE, chn);

        CAN_send_frame(CAN_frame, txt_buf_nr, DUT_NODE, chn, frame_sent);
        CAN_wait_error_frame(DUT_NODE, chn);

        CAN_wait_pc_state(pc_deb_intermission, DUT_NODE, chn);
        CAN_wait_error_frame(DUT_NODE, chn);

        get_tx_buf_state(txt_buf_nr, buf_state, DUT_NODE, chn);
        check_m(buf_state = buf_ready, "TXT Buffer ready!");
        
        ------------------------------------------------------------------------
        -- @5. Abort transmission by DUT. Wait until transmission was aborted.
        ------------------------------------------------------------------------
        info_m("Step 5: Aborting transmission");

        send_TXT_buf_cmd(buf_set_abort, txt_buf_nr, DUT_NODE, chn);
        get_tx_buf_state(txt_buf_nr, buf_state, DUT_NODE, chn);
        while (buf_state /= buf_aborted) loop
            get_tx_buf_state(txt_buf_nr, buf_state, DUT_NODE, chn);
        end loop;
        CAN_wait_bus_idle(DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- @6. Generate random retransmitt limit (between 1 and 14). Enable 
        --     retransmitt limitation in DUT. Erase TX Error counter.
        ------------------------------------------------------------------------
        info_m("Step 6: Setting random retransmitt limit!");

        rand_int_v(13, retr_th);
        retr_th := retr_th + 1;
        info_m("Retransmitt threshold: " & Integer'image(retr_th));
        CAN_enable_retr_limit(true, retr_th, DUT_NODE, chn);
        err_counters.tx_counter := 0;
        set_error_counters(err_counters, DUT_NODE, chn);
        
        ------------------------------------------------------------------------
        -- @7. Send frame by DUT. Monitor that after initial transmission and
        --     after each next re-transmission sending TXT Buffer in DUT is
        --     "Ready".
        ------------------------------------------------------------------------
        info_m("Step 7: Checking number of re-transmissions: " &
                integer'image(retr_th));

        CAN_send_frame(CAN_frame, txt_buf_nr, DUT_NODE, chn, frame_sent);
        for i in 0 to retr_th loop
            info_m("Loop: " & integer'image(i));
            CAN_wait_frame_sent(DUT_NODE, chn);
            get_tx_buf_state(txt_buf_nr, buf_state, DUT_NODE, chn);
            if (i /= retr_th) then
                check_m(buf_state = buf_ready, "TXT Buffer ready");
            else
                check_m(buf_state = buf_failed, "TXT Buffer failed");
            end if;
        end loop;
        CAN_wait_bus_idle(DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- @8. Check that value of TX Error counter in DUT is equal to:
        --     (retr_lim + 1) * 8.
        ------------------------------------------------------------------------
        info_m("Step 8: Checking value of TX Error counter");

        read_error_counters(err_counters, DUT_NODE, chn);
        check_m(err_counters.tx_counter = 8 * (retr_th + 1),
            "Counters exp: " & Integer'Image(err_counters.tx_counter) &
            " counters real: " & Integer'image(8 * (retr_th + 1)));
        
        ------------------------------------------------------------------------
        -- @9. Set retransmitt limit to 15 and Enable Re-transmissions in DUT.
        --     Start Sending frame by DUT. Erase error counters so that
        --     we don't go to bus off (just to be sure).
        ------------------------------------------------------------------------
        info_m("Step 9: Set maximal retransmitt limit (15)");

        CAN_enable_retr_limit(true, 15, DUT_NODE, chn);
        CAN_send_frame(CAN_frame, txt_buf_nr, DUT_NODE, chn, frame_sent);

        err_counters.tx_counter := 0;
        set_error_counters(err_counters, DUT_NODE, chn);
        
        ------------------------------------------------------------------------
        -- @10. Monitor that after initial transmission and after each next
        --      re-transmission sending TXT Buffer in DUT is "Ready". After
        --      'retransmitt limit' retransmissions check that sending TXT
        --      Buffer in DUT is in state "TX Error".
        ------------------------------------------------------------------------
        info_m("Step 10: Checking number of re-transmissions");

        for i in 0 to 15 loop
            CAN_wait_frame_sent(DUT_NODE, chn);
            get_tx_buf_state(txt_buf_nr, buf_state, DUT_NODE, chn);
            if (i /= 15) then
                check_m(buf_state = buf_ready, "TXT Buffer ready");
            else
                check_m(buf_state = buf_failed, "TXT Buffer failed");
            end if;
        end loop;
        
        -- Error counters must be erased because unit is already Error Passive!
        err_counters.tx_counter := 0;
        set_error_counters(err_counters, DUT_NODE, chn);
        wait for 100 ns;

        ------------------------------------------------------------------------
        -- @11. Set retransmitt limit to 1 in DUT. Insert frames for 
        --      transmission to DUT and Test node simultaneously to invoke
        --      arbitration. ID of frame in DUT is higher than the one in
        --      Test node (to loose arbitration). Wait until node 1 is in Control
        --      field of a frame. Check that DUT is receiver (arbitration was
        --      really lost) and TXT Buffer in DUT ended up in "TX Error"
        --      state.
        ------------------------------------------------------------------------
        info_m("Step 11: Testing re-transmitt limit due to arbitration loss!");

        CAN_enable_retr_limit(true, 1, DUT_NODE, chn);
        CAN_frame.ident_type := BASE;
        CAN_frame.identifier := 10;
        CAN_insert_TX_frame(CAN_frame, 1, DUT_NODE, chn);
        CAN_frame.identifier := 9;
        CAN_insert_TX_frame(CAN_frame, 1, TEST_NODE, chn);
        CAN_insert_TX_frame(CAN_frame, 2, TEST_NODE, chn);
        
        send_TXT_buf_cmd(buf_set_ready, 1, DUT_NODE, chn);
        
        -- Note: There are two frames in Test node. First one will be transmitted,
        --       because we have one re-transmission, thus Test node needs to
        --       send next frame after first one to invoke next arbitration!
        send_TXT_buf_cmd(buf_set_ready, 1, TEST_NODE, chn);
        send_TXT_buf_cmd(buf_set_ready, 2, TEST_NODE, chn);
        
        CAN_wait_frame_sent(DUT_NODE, chn);
        
        CAN_wait_pc_state(pc_deb_control, DUT_NODE, chn);
        get_controller_status(status, DUT_NODE, chn);
        check_m(status.receiver, "DUT lost arbitration");
        get_tx_buf_state(txt_buf_nr, buf_state, DUT_NODE, chn);
        check_m(buf_state = buf_failed, "TXT Buffer failed");
        CAN_wait_bus_idle(DUT_NODE, chn);

  end procedure;

end package body;