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
--  TXT Buffer Set empty - SW command (TX_COMMAND) feature test.
--
-- @Verifies:
--  @1. Set Empty command moves TXT Buffer from TX OK, TX Error, Aborted to Empty.
--
-- @Test sequence:
--  @1. Check TXT Buffer is empty. Issue Set empty and check it is still empty.
--  @2. Generate random CAN frame, send it from TXT Buffer and wait until it is
--      received. Check TXT Buffer is in TX OK. Issue Set Empty command and
--      check TXT Buffer is empty.
--  @3. Generate random CAN frame, send it fom TXT Buffer and issue Set Abort
--      command. Wait until frame is over and check TXT Buffer is Aborted. Issue
--      Set Empty command and check it becomes Empty.
--  @4. Set One shot mode in DUT. Forbid ACK in Test node. Send CAN frame by Node
--      1. Wait until CAN frame is sent and check TXT Buffer from which it was
--      sent ended in TX Error. Issue Set empty command and check TXT Buffer is
--      Empty.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--   16.11.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package tx_cmd_set_empty_ftest is
    procedure tx_cmd_set_empty_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body tx_cmd_set_empty_ftest is
    procedure tx_cmd_set_empty_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable txt_state          :       SW_TXT_Buffer_state_type;
        variable buf_nr             :       natural;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        variable num_buffers        :       natural;
    begin

        get_tx_buf_count(num_buffers, DUT_NODE, chn);

        for buf_nr in 1 to num_buffers loop

            info_m("Testing with TXT Buffer: " & integer'image(buf_nr));

            -----------------------------------------------------------------------
            -- @1. Check TXT Buffer is empty. Issue Set empty and check it is still
            --    empty.
            -----------------------------------------------------------------------
            info_m("Step 1");

            get_tx_buf_state(buf_nr, txt_state, DUT_NODE, chn);
            check_m(txt_state = buf_empty, "TXT Buffer empty upon start");
            send_TXT_buf_cmd(buf_set_empty, buf_nr, DUT_NODE, chn);
            get_tx_buf_state(buf_nr, txt_state, DUT_NODE, chn);
            check_m(txt_state = buf_empty, "TXT Buffer still empty even post restart!");

            ------------------------------------------------------------------------
            -- @2. Generate random CAN frame, send it from TXT Buffer and wait until
            --     it is received. Check TXT Buffer is in TX OK. Issue Set Empty
            --     command and check TXT Buffer is empty.
            ------------------------------------------------------------------------
            info_m("Step 2");

            CAN_generate_frame(CAN_frame);
            CAN_send_frame(CAN_frame, buf_nr, DUT_NODE, chn, frame_sent);
            CAN_wait_frame_sent(DUT_NODE, chn);

            get_tx_buf_state(buf_nr, txt_state, DUT_NODE, chn);
            check_m(txt_state = buf_done, "TXT Buffer OK after frame sent!");
            send_TXT_buf_cmd(buf_set_empty, buf_nr, DUT_NODE, chn);
            wait for 11 ns; -- This command is pipelined, delay must be inserted!
            get_tx_buf_state(buf_nr, txt_state, DUT_NODE, chn);
            check_m(txt_state = buf_empty, "Set Empty: TX OK -> Empty");

            ------------------------------------------------------------------------
            -- @3. Generate random CAN frame, send it fom TXT Buffer and issue Set
            --    Abort command. Wait until frame is over and check TXT Buffer is
            --    Aborted. Issue Set Empty command and check it becomes Empty.
            ------------------------------------------------------------------------
            info_m("Step 3");

            CAN_generate_frame(CAN_frame);
            CAN_send_frame(CAN_frame, buf_nr, DUT_NODE, chn, frame_sent);
            CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);
            send_TXT_buf_cmd(buf_set_abort, buf_nr, DUT_NODE, chn);
            CAN_wait_bus_idle(DUT_NODE, chn);

            get_tx_buf_state(buf_nr, txt_state, DUT_NODE, chn);
            check_m(txt_state = buf_aborted, "TXT Buffer Aborted!");
            send_TXT_buf_cmd(buf_set_empty, buf_nr, DUT_NODE, chn);
            wait for 11 ns; -- This command is pipelined, delay must be inserted!
            get_tx_buf_state(buf_nr, txt_state, DUT_NODE, chn);
            check_m(txt_state = buf_empty, "Set Empty: Aborted -> Empty");

            -----------------------------------------------------------------------
            -- @4. Set One shot mode in DUT. Forbid ACK in Test node. Send CAN frame
            --     by DUT. Wait until CAN frame is sent and check TXT Buffer from
            --     which it was sent ended in TX Error. Issue Set empty command and
            --     check TXT Buffer is Empty.
            -----------------------------------------------------------------------
            info_m("Step 4");

            CAN_enable_retr_limit(true, 0, DUT_NODE, chn);
            mode_2.acknowledge_forbidden := true;
            set_core_mode(mode_2, TEST_NODE, chn);

            CAN_generate_frame(CAN_frame);
            CAN_send_frame(CAN_frame, buf_nr, DUT_NODE, chn, frame_sent);
            CAN_wait_frame_sent(DUT_NODE, chn);

            get_tx_buf_state(buf_nr, txt_state, DUT_NODE, chn);
            check_m(txt_state = buf_failed, "TXT Buffer Failed!");

            send_TXT_buf_cmd(buf_set_empty, buf_nr, DUT_NODE, chn);
            wait for 11 ns; -- This command is pipelined, delay must be inserted!
            get_tx_buf_state(buf_nr, txt_state, DUT_NODE, chn);
            check_m(txt_state = buf_empty, "Set Empty: TX Failed -> Empty");

            -- Re-enable ACK fr next iterations
            CAN_enable_retr_limit(true, 0, DUT_NODE, chn);
            mode_2.acknowledge_forbidden := false;
            set_core_mode(mode_2, TEST_NODE, chn);

        end loop;

  end procedure;

end package body;