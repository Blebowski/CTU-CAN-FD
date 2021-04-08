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
--  TX/RX frame counters clear command.
--
-- @Verifies:
--  @1. TX Frame counter is cleared by COMMAND[TXRFCRST].
--  @2. TX Frame counter is not cleared by COMMAND[TXFRCRST].
--  @3. RX Frame counter is cleared by COMMAND[RXRFCRST].
--  @4. RX Frame counter is not cleared by COMMAND[RXFRCRST].
--
-- @Test sequence:
--  @1. Generate and send frame by Test node. Check that TX frame counter of Test node
--      is not zero. Issue COMMAND[RXFRCRST] and check it is still not 0. Issue
--      COMMAND[TXFRCRST] and check it is 0 now.
--  @2. Check that RX Frame counter of DUT is not zero. Issue COMMAND[TXFRCRST]
--      and check it is still not 0. Issue COMMAND[RXFRCRST] and RX Frame counter
--      in DUT is 0.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    25.10.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package command_frcrst_ftest is
    procedure command_frcrst_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body command_frcrst_ftest is
    procedure command_frcrst_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;

        variable command            :     SW_command := SW_command_rst_val;
        
        variable traff_ctrs_1       :     SW_traffic_counters;
        variable traff_ctrs_2       :     SW_traffic_counters;

    begin

        -----------------------------------------------------------------------
        -- @1. Generate and send frame by Test node. Check that TX frame counter of
        --     Test node is not zero. Issue COMMAND[RXFRCRST] and check it is still
        --     not 0. Issue COMMAND[TXFRCRST] and check it is 0 now.
        -----------------------------------------------------------------------
        info_m("Step 1");

        CAN_generate_frame(frame_1);
        CAN_insert_TX_frame(frame_1, 1, TEST_NODE, chn);
        send_TXT_buf_cmd(buf_set_ready, 1, TEST_NODE, chn);

        CAN_wait_frame_sent(DUT_NODE, chn);

        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);

        read_traffic_counters(traff_ctrs_2, TEST_NODE, chn);
        check_m(traff_ctrs_2.tx_frames /= 0, "TX frame counter not 0!");

        command.rx_frame_ctr_rst := true;
        give_controller_command(command, TEST_NODE, chn);

        read_traffic_counters(traff_ctrs_2, TEST_NODE, chn);
        check_m(traff_ctrs_2.tx_frames /= 0, "TX frame counter not 0 again!");

        command.tx_frame_ctr_rst := true;
        command.rx_frame_ctr_rst := false;
        give_controller_command(command, TEST_NODE, chn);
        
        read_traffic_counters(traff_ctrs_2, TEST_NODE, chn);
        check_m(traff_ctrs_2.tx_frames = 0, "TX frame counter erased!");

        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);

        -----------------------------------------------------------------------
        -- @2. Check that RX Frame counter of DUT is not zero. Issue
        --    COMMAND[TXFRCRST] and check it is still not 0. Issue 
        --    COMMAND[RXFRCRST] and RX Frame counter in DUT is 0. 
        -----------------------------------------------------------------------
        info_m("Step 2");
        
        read_traffic_counters(traff_ctrs_1, DUT_NODE, chn);
        info_m("DUT");
        info_m("RX frames: " & integer'image(traff_ctrs_1.rx_frames));
        info_m("TX frames: " & integer'image(traff_ctrs_1.tx_frames));
        check_m(traff_ctrs_1.rx_frames /= 0, "RX frame counter not 0!");
        
        command.tx_frame_ctr_rst := true;
        command.rx_frame_ctr_rst := false;
        give_controller_command(command, DUT_NODE, chn);
        
        read_traffic_counters(traff_ctrs_1, DUT_NODE, chn);
        check_m(traff_ctrs_1.rx_frames /= 0, "RX frame counter not 0 again!");
        
        command.tx_frame_ctr_rst := false;
        command.rx_frame_ctr_rst := true;
        give_controller_command(command, DUT_NODE, chn);
        
        read_traffic_counters(traff_ctrs_1, DUT_NODE, chn);
        check_m(traff_ctrs_1.rx_frames = 0, "RX frame counter erased!");

  end procedure;

end package body;