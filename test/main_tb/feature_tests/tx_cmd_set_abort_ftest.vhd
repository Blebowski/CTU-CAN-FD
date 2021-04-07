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
--  TXT Buffer Set Abort - SW command (TX_COMMAND) feature test.
--
-- @Verifies:
--  @1. Set Abort command moves TXT Buffer from Ready to Aborted and from
--      TX in progress to Aborted.
--
-- @Test sequence:
--  @1. Generate frame and insert it for transmission to random TXT Buffer.
--      Wait until sample point. Issue Set Ready command and check TXT Buffer
--      becomes Ready. Issue Set Abort command and check TXT Buffer becomes
--      Aborted. Bit time should be sufficiently long so that there is enough
--      time to issue two commands and read buffer state ones before next sample
--      point arrives.
--  @2. Issue Set Ready command and wait until transmission starts. Check that
--      TXT Buffer is in TX in progress. Issue Set Abort and check that TXT
--      Buffer moves to Abort in progress. Wait until transmission is over.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--   17.11.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package tx_cmd_set_abort_ftest is
    procedure tx_cmd_set_abort_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body tx_cmd_set_abort_ftest is
    procedure tx_cmd_set_abort_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :       SW_CAN_frame_type;
        variable txt_state          :       SW_TXT_Buffer_state_type;
        variable buf_nr             :       natural;
    begin

        -- For whole test random TXT Buffer will be used!
        pick_random_txt_buffer(buf_nr, DUT_NODE, chn);
        info_m("Testing with TXT Buffer: " & integer'image(buf_nr));

        -----------------------------------------------------------------------
        -- @1. Generate frame and insert it for transmission to random TXT Buffer.
        --    Wait until sample point. Issue Set Ready command and check TXT 
        --    Buffer becomes Ready. Issue Set Abort command and check TXT Buffer
        --    becomes Aborted. Bit time should be sufficiently long so that there
        --    is enough time to issue two commands and read buffer state ones
        --    before next sample point arrives.
        -----------------------------------------------------------------------
        info_m("Step 1");
        
        CAN_generate_frame(CAN_frame);
        CAN_insert_TX_frame(CAN_frame, buf_nr, DUT_NODE, chn);

        CAN_wait_sample_point(DUT_NODE, chn);
        send_TXT_buf_cmd(buf_set_ready, buf_nr, DUT_NODE, chn);
        wait for 11 ns; -- This command is pipelined, delay must be inserted!
        get_tx_buf_state(buf_nr, txt_state, DUT_NODE, chn);
        check_m(txt_state = buf_ready, "TXT Buffer ready!");

        send_TXT_buf_cmd(buf_set_abort, buf_nr, DUT_NODE, chn);
        wait for 11 ns; -- This command is pipelined, delay must be inserted!
        get_tx_buf_state(buf_nr, txt_state, DUT_NODE, chn);
        check_m(txt_state = buf_aborted, "Set Abort: Ready -> Aborted");

        -----------------------------------------------------------------------
        -- @2. Issue Set Ready command and wait until transmission starts. Check
        --    that TXT Buffer is in TX in progress. Issue Set Abort and check
        --    that TXT Buffer moves to Abort in progress. Wait until 
        --    transmission is over.
        -----------------------------------------------------------------------
        info_m("Step 2");

        send_TXT_buf_cmd(buf_set_ready, buf_nr, DUT_NODE, chn);
        CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);
        get_tx_buf_state(buf_nr, txt_state, DUT_NODE, chn);
        check_m(txt_state = buf_tx_progress, "TXT Buffer TX in Progress");

        send_TXT_buf_cmd(buf_set_abort, buf_nr, DUT_NODE, chn);
        wait for 11 ns; -- This command is pipelined, delay must be inserted!
        get_tx_buf_state(buf_nr, txt_state, DUT_NODE, chn);
        check_m(txt_state = buf_ab_progress,
            "Set Abort: TX in Progress -> Abort in Progress");
            
        CAN_wait_frame_sent(DUT_NODE, chn);

  end procedure;

end package body;