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
--  RX Traffic counter feature test implementation.
--
-- @Verifies:
--  @1. RX Counter is incremented after each succesfully received frame.
--  @2. RX Counter is not incremented when error frame is transmitted.
--  @3. TX Counter is not incremented when frame is succesfully received.
--  @4. RX Counter is cleared by COMMAND[RXFRCRST].
--  @5. RX Counter is NOT cleared by COMMAND[TXFRCRST].
--  @6. RX Counter is incremented when frame is transmitted in Loopback mode.
--
-- @Test sequence:
--  @1. Read TX Counter from DUT. Set One-shot mode (no retransmission) in
--      DUT.
--  @2. Send frame from Test node. Wait until EOF field. Read RX counter from DUT
--      and check it did not change yet.
--  @3. Wait until the end of EOF. Read RX counter and check it was incremented.
--      Read TX counter and check it is not incremented!
--  @4. Send Frame from Test node. Wait till ACK field. Corrupt ACK field to be
--      recessive. Wait till Test node is not in ACK field anymore. Check Test node
--      is transmitting Error frame.
--  @5. Wait until DUT also starts transmitting error frame. Wait until bus 
--      is idle, check that RX Counter was not incremented in DUT.
--  @6. Send random amount of frames by Test node and wait until they are sent.
--  @7. Check that RX counter was incremented by N in DUT!
--  @8. Issue COMMAND[TXFRCRST] and check RX counter was NOT cleared in DUT.
--  @9. Issue COMMAND[RXFRCRST] and check RX counter was cleared in DUT.
-- @10. Read all frames from RX buffer in DUT.
-- @11. Set Loopback mode in DUT. Send frame by DUT and wait until it is
--      sent. Check there is a frame received in RX buffer. Check that RX frame
--      counter was incremented.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    28.9.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package rx_counter_ftest is
    procedure rx_counter_ftest_exec(
        signal      chn             : inout  t_com_channel
    );

end package;


package body rx_counter_ftest is
    procedure rx_counter_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :       SW_CAN_frame_type;
        variable RX_CAN_frame       :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable rand_value         :       natural;

        variable ctrs_1             :       SW_traffic_counters;
        variable ctrs_2             :       SW_traffic_counters;
        variable ctrs_3             :       SW_traffic_counters;
        variable ctrs_4             :       SW_traffic_counters;
        variable ctrs_5             :       SW_traffic_counters;
        
        variable status             :       SW_status;
        variable command            :       SW_command := SW_command_rst_val;
        
        variable rx_buf_info        :       SW_RX_Buffer_info;
        variable mode_1             :       SW_mode := SW_mode_rst_val;
    begin

        ------------------------------------------------------------------------
        -- @1. Read TX Counter from DUT. Set One-shot mode (no retransmission)
        --     in DUT.
        ------------------------------------------------------------------------
        info_m("Step 1: Read initial counter values.");
        
        read_traffic_counters(ctrs_1, DUT_NODE, chn);
        CAN_enable_retr_limit(true, 0, TEST_NODE, chn);
        
        ------------------------------------------------------------------------
        -- @2. Send frame from Test node. Wait until EOF field. Read RX counter
        --     from DUT and check it did not change yet.
        ------------------------------------------------------------------------        
        info_m("Step 2: Send frame by Test node!");
        
        CAN_generate_frame(CAN_frame);
        CAN_send_frame(CAN_frame, 2, TEST_NODE, chn, frame_sent);
        
        CAN_wait_pc_state(pc_deb_eof, DUT_NODE, chn);
        read_traffic_counters(ctrs_2, DUT_NODE, chn);
        
        check_m(ctrs_1.tx_frames = ctrs_2.tx_frames,
            "TX counter unchanged before EOF!");
        check_m(ctrs_1.rx_frames = ctrs_2.rx_frames,
            "RX counter unchanged before EOF!");

        ------------------------------------------------------------------------
        -- @3. Wait until the end of EOF. Read RX counter and check it was
        --     incremented. Read TX counter and check it is not incremented!
        ------------------------------------------------------------------------
        info_m("Step 3: Check TX, RX counters after frame.");
        
        CAN_wait_not_pc_state(pc_deb_eof, DUT_NODE, chn);
        read_traffic_counters(ctrs_3, DUT_NODE, chn);
        
        check_m(ctrs_1.tx_frames = ctrs_3.tx_frames,
            "TX counter unchanged after EOF!");
        check_m(ctrs_1.rx_frames + 1 = ctrs_3.rx_frames,
            "RX counter changed after EOF!");
        
        CAN_wait_bus_idle(DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- @4. Send Frame from Test node. Wait till ACK field. Corrupt ACK field
        --     to be recessive. Wait till Test node is not in ACK field anymore.
        --     Check test node is transmitting Error frame.
        ------------------------------------------------------------------------
        info_m("Step 4: Send frame and force ACK recessive!");
        
        CAN_generate_frame(CAN_frame);
        CAN_frame.frame_format := NORMAL_CAN;
        CAN_send_frame(CAN_frame, 1, TEST_NODE, chn, frame_sent);
        
        CAN_wait_pc_state(pc_deb_ack, TEST_NODE, chn);
        force_bus_level(RECESSIVE, chn);
        
        CAN_wait_not_pc_state(pc_deb_ack, TEST_NODE, chn);
        get_controller_status(status, TEST_NODE, chn);
        
        check_m(status.error_transmission, "Error frame is being transmitted!");
        release_bus_level(chn);

        ------------------------------------------------------------------------
        -- @5. Wait until DUT also starts transmitting error frame. Wait until
        --     bus is idle, check that RX Counter was not incremented in DUT.
        ------------------------------------------------------------------------
        info_m("Step 5: Wait until error frame!");
        
        CAN_wait_error_frame(DUT_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);
        
        read_traffic_counters(ctrs_4, DUT_NODE, chn);
        
        check_m(ctrs_3.tx_frames = ctrs_4.tx_frames,
            "TX counter unchanged after Error frame!");
        check_m(ctrs_3.rx_frames = ctrs_4.rx_frames,
            "RX counter unchanged after Error frame!");

        ------------------------------------------------------------------------
        -- @6. Send random amount of frames by Test node and wait until they are
        --    sent.
        ------------------------------------------------------------------------
        info_m("Step 6: Send N random frames!");
        
        rand_int_v(6, rand_value);
        for i in 0 to rand_value - 1 loop
            CAN_generate_frame(CAN_frame);
            CAN_send_frame(CAN_frame, 3, TEST_NODE, chn, frame_sent);
            CAN_wait_frame_sent(TEST_NODE, chn);
        end loop;

        ------------------------------------------------------------------------
        -- @7. Check that RX counter was incremented by N in DUT!
        ------------------------------------------------------------------------
        info_m("Step 7: Check RX counter was incremented by N!");
        
        read_traffic_counters(ctrs_5, DUT_NODE, chn);
        check_m(ctrs_4.rx_frames + rand_value = ctrs_5.rx_frames,
              "RX Frames counter incremented by: " & integer'image(rand_value));

        ------------------------------------------------------------------------
        -- @8. Issue COMMAND[TXFRCRST] and check RX counter was NOT cleared in 
        --     DUT.
        ------------------------------------------------------------------------
        info_m("Step 8: Issue COMMAND[TXFRCRST]");
        
        command.tx_frame_ctr_rst := true;
        give_controller_command(command, DUT_NODE, chn);
        
        read_traffic_counters(ctrs_1, DUT_NODE, chn);
        check_m(ctrs_1.rx_frames = ctrs_5.rx_frames,
              "RX counter not cleared by COMMAND[TXFRCRST]");

        ------------------------------------------------------------------------
        -- @9. Issue COMMAND[RXFRCRST] and check RX counter was cleared in DUT.
        ------------------------------------------------------------------------
        info_m("Step 9: Issue COMMAND[RXFRCRST]");
        
        command.rx_frame_ctr_rst := true;
        command.tx_frame_ctr_rst := false;
        give_controller_command(command, DUT_NODE, chn);
        
        read_traffic_counters(ctrs_1, DUT_NODE, chn);
        check_m(ctrs_1.rx_frames = 0, "RX counter cleared by COMMAND[RXFRCRST]");

        ------------------------------------------------------------------------
        -- @10. Read all frames from RX buffer in DUT.
        ------------------------------------------------------------------------
        info_m("Step 10: Read all frames from RX Buffer!");
        
        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        if (rx_buf_info.rx_frame_count > 0) then
            for i in 0 to rx_buf_info.rx_frame_count - 1 loop
                CAN_read_frame(RX_CAN_frame, DUT_NODE, chn);
            end loop;
        end if;
        
        ------------------------------------------------------------------------
        -- @11. Set Loopback mode in DUT. Send frame by DUT and wait until
        --      it is sent. Check there is a frame received in RX buffer. Check
        --      that RX frame counter was incremented.
        ------------------------------------------------------------------------
        info_m("Step 11: Check RX counter is incremented in Loopback mode!");
        
        mode_1.internal_loopback := true;
        set_core_mode(mode_1, DUT_NODE, chn);
        
        CAN_generate_frame(CAN_frame);
        CAN_send_frame(CAN_frame, 4, DUT_NODE, chn, frame_sent);
        CAN_wait_frame_sent(DUT_NODE, chn);
        
        read_traffic_counters(ctrs_1, DUT_NODE, chn);
        check_m(ctrs_1.rx_frames = 1,
            "RX counter incremented when frame sent in Loopback mode!");

    end procedure;

end package body;