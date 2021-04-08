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
--  Retransmitt limit feature test 3 (cornercases).
--
-- @Verifies:
--  @1. When unit is a receiver without attempt to transmitt a frame
--      (TXT Buffer is ready, unit is Error passive and dominant bit is detected
--       during Suspend field), if an error occurs during such a frame,
--      retransmitt counter is not incremented!
--
-- @Test sequence:
--  @1. Configure retransmitt limit in DUT, enable retransmitt limitation.
--      Enable Test Mode in DUT to be able manipulate with Error counters.
--      Configure Test node to Acknowledge Forbidden Mode to invoke transmission
--      of Error frames during test.
--  @2. Set DUT TX Error counter to 150. Check that DUT is Error Passive.
--      Send frame by DUT. Wait until Error frame occurs. Check that Retransmit
--      counter in DUT is now 1. Insert frame to Test node. Wait until Suspend
--      transmission in DUT.
--  @3. Wait until Arbitration field in DUT, check that DUT is now receiver.
--      Wait until ACK field, force the bus for the whole duration of ACK field
--      to Recessive. Check that Error frame is transmitted by DUT. Wait until
--      bus is Idle. Check that Retransmitt counter is stil1 1.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    13.7.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package retr_limit_3_ftest is
    procedure retr_limit_3_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body retr_limit_3_ftest is
    procedure retr_limit_3_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable retr_th            :       natural;

        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        variable err_counters       :       SW_error_counters := (0, 0, 0, 0);
        variable status             :       SW_status;
        variable retr_ctr           :       natural;
        variable fault_state        :       SW_fault_state;
    begin
        
        -- Hard coded threshold is enough for this test!
        retr_th := 5;

        ------------------------------------------------------------------------
        -- @1. Configure retransmitt limit in DUT, enable retransmitt
        --     limitation. Enable Test Mode in DUT to be able manipulate with
        --     Error counters. Configure Test node to Acknowledge Forbidden Mode
        --     to invoke transmission of Error frames during test.
        ------------------------------------------------------------------------
        info_m("Step 1: Configuring retransmitt limit to 1 (DUT), ACF (Test node)");

        CAN_enable_retr_limit(true, retr_th, DUT_NODE, chn);

        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, TEST_NODE, chn);

        mode_1.test := true;
        set_core_mode(mode_1, DUT_NODE, chn);
        
        ------------------------------------------------------------------------
        -- @2. Set DUT TX Error counter to 150. Check that DUT is Error
        --     Passive. Send frame by DUT. Wait until Error frame occurs.
        --     Check that Retransmit counter in DUT is now 1. Wait until 
        --     Suspend transmission in DUT. Insert frame to Test node.
        ------------------------------------------------------------------------
        info_m("Step 2: Set DUT to Error passive");
        
        err_counters.tx_counter := 150;
        set_error_counters(err_counters, DUT_NODE, chn);

        get_fault_state(fault_state, DUT_NODE, chn);
        check_m(fault_state = fc_error_passive, "Unit Error Passive!");

        CAN_generate_frame(CAN_frame);
        CAN_send_frame(CAN_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_error_frame(DUT_NODE, chn);

        CAN_read_retr_ctr(retr_ctr, DUT_NODE, chn);
        check_m(retr_ctr = 1,
            "Retransmitt counter 1 after Error frame!");

        CAN_wait_pc_state(pc_deb_suspend, DUT_NODE, chn); -- Wait until suspend
        CAN_send_frame(CAN_frame, 1, TEST_NODE, chn, frame_sent);
        
        ------------------------------------------------------------------------
        -- @3. Wait until Arbitration field in DUT, check that DUT is now
        --     receiver. Wait until ACK field, force the bus for the whole du-
        --     ration of ACK field to Recessive. Check that Error frame is
        --     transmitted by DUT. Wait until Intermission. Check that Retran-
        --     smitt counter is stil1 1.
        ------------------------------------------------------------------------
        info_m("Step 3: Check Retransmitt counter not incremented when receiver only");

        CAN_wait_pc_state(pc_deb_arbitration, DUT_NODE, chn);
        wait for 10 ns; -- Operational state updated in the same clock cycle!

        get_controller_status(status, DUT_NODE, chn);
        check_m(status.receiver, "DUT turned receiver when in Error Passive!");

        CAN_wait_pc_state(pc_deb_ack, DUT_NODE, chn);
        force_bus_level(RECESSIVE, chn);
        
        CAN_wait_not_pc_state(pc_deb_ack, DUT_NODE, chn);
        release_bus_level(chn);
        
        get_controller_status(status, DUT_NODE, chn);
        check_m(status.error_transmission, "Error frame being transmitted!");

        CAN_wait_pc_state(pc_deb_intermission, DUT_NODE, chn);
        CAN_read_retr_ctr(retr_ctr, DUT_NODE, chn);
        check_m(retr_ctr = 1,
            "Retransmitt counter not incremented when unit was only receiver!");
        
  end procedure;

end package body;