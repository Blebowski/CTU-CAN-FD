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
--   TXT Buffer access test from SW.
--
-- @Verifies:
--  @1. TXT Buffer can not be written by SW when it is Ready, TX In Progress
--      or Abort in Progress.
--
-- @Test sequence:
--  @1. Iterate over all TXT Buffers:
--      @1.1 Prepare two frames (A and B), both with 64 data bytes, and both
--           CAN FD frames. The frames differ in:
--              - Identifier type (IDENT_W), Identifier,
--              - Bit-rate shift (FRAME_FORMAT_W)
--              - Value of data bytes.
--      @1.2 Insert Frame A to TXT Buffer. Wait until sample point of DUT
--           Node. Send Set Ready command to TXT Buffer. Try to insert frame
--           B into the TXT Buffer.
--      @1.3 Wait until frame transmission starts and attempt to insert then
--           frame B again into TXT Buffer.
--      @1.4 Wait until data field starts and issue "Set Abort" Command to
--           TXT Buffer. Check that TXT Buffer is in "Abort in Progress".
--           Again attempt to write the frame B into TXT Buffer.
--      @1.5 Wait until bus is idle. Read the frame from RX Buffer of
--           Test Node. Compare the read frame with Frame A.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--      17.7.2024   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.clk_gen_agent_pkg.all;

package txt_buffer_access_ignore_ftest is
    procedure txt_buffer_access_ignore_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body txt_buffer_access_ignore_ftest is

    procedure txt_buffer_access_ignore_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable num_txt_bufs       :       natural;

        variable CAN_frame_a        :       SW_CAN_frame_type;
        variable CAN_frame_b        :       SW_CAN_frame_type;
        variable CAN_RX_frame       :       SW_CAN_frame_type;

        variable txtb_state         :       SW_TXT_Buffer_state_type;

        variable frames_match       :       boolean;
    begin

        -------------------------------------------------------------------------------------------
        --  @1. Iterate over all TXT Buffers:
        -------------------------------------------------------------------------------------------
        info_m("Step 1");
        get_tx_buf_count(num_txt_bufs, DUT_NODE, chn);

        for txt_buf_index in 1 to num_txt_bufs loop
            info_m("Testing TXT Buffer: " & integer'image(txt_buf_index));

            ---------------------------------------------------------------------------------------
            -- @1.1 Prepare two frames (A and B), both with 64 data bytes, and both
            --           CAN FD frames. The frames differ in:
            --              - Identifier type (IDENT_W), Identifier,
            --              - Bit-rate shift (FRAME_FORMAT_W)
            --              - Value of data bytes.
            ---------------------------------------------------------------------------------------
            info_m("Step 1.1");

            CAN_generate_frame(CAN_frame_a);
            CAN_generate_frame(CAN_frame_b);

            CAN_frame_a.frame_format := FD_CAN;
            CAN_frame_b.frame_format := FD_CAN;

            CAN_frame_a.identifier := 0;
            CAN_frame_b.identifier := 536870911;

            CAN_frame_a.ident_type := BASE;
            CAN_frame_b.ident_type := EXTENDED;

            CAN_frame_a.brs := BR_SHIFT;
            CAN_frame_b.brs := BR_NO_SHIFT;

            CAN_frame_a.data_length := 64;
            CAN_frame_b.data_length := 64;

            for i in 0 to 63 loop
                CAN_frame_a.data(i) := x"AA";
                CAN_frame_b.data(i) := x"55";
            end loop;

            decode_length(CAN_frame_a.data_length, CAN_frame_a.dlc);
            decode_dlc_rx_buff(CAN_frame_a.dlc, CAN_frame_a.rwcnt);

            decode_length(CAN_frame_b.data_length, CAN_frame_b.dlc);
            decode_dlc_rx_buff(CAN_frame_b.dlc, CAN_frame_b.rwcnt);

            ---------------------------------------------------------------------------------------
            -- @1.2 Insert Frame A to TXT Buffer. Wait until sample point of DUT
            --      Node. Send Set Ready command to TXT Buffer. Try to insert frame
            --      B into the TXT Buffer.
            ---------------------------------------------------------------------------------------
            info_m("Step 1.2");

            CAN_insert_TX_frame(CAN_frame_a, txt_buf_index, DUT_NODE, chn);

            CAN_wait_sample_point(DUT_NODE, chn);
            send_TXT_buf_cmd(buf_set_ready, txt_buf_index, DUT_NODE, chn);

            -- TXT Buffer command is pipelined. Without wait, we manage to squeeze in first
            -- write when the buffer is still in "Empty"
            wait for 20 ns;

            CAN_insert_TX_frame(CAN_frame_b, txt_buf_index, DUT_NODE, chn);

            ---------------------------------------------------------------------------------------
            -- @1.3 Wait until frame transmission starts and attempt to insert then
            --      frame B again into TXT Buffer.
            ---------------------------------------------------------------------------------------
            info_m("Step 1.3");

            CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);
            CAN_wait_sample_point(DUT_NODE, chn);

            get_tx_buf_state(txt_buf_index, txtb_state, DUT_NODE, chn);
            check_m(txtb_state = buf_tx_progress, "TXT Buffer in TX in Progress");

            CAN_insert_TX_frame(CAN_frame_b, txt_buf_index, DUT_NODE, chn);

            ---------------------------------------------------------------------------------------
            -- @1.4 Wait until data field starts and issue "Set Abort" Command to TXT Buffer.
            --      Check that TXT Buffer is in "Abort in Progress". Again attempt to write the
            --      frame B into TXT Buffer.
            ---------------------------------------------------------------------------------------
            info_m("Step 1.4");

            CAN_wait_pc_state(pc_deb_data, DUT_NODE, chn);
            send_TXT_buf_cmd(buf_set_abort, txt_buf_index, DUT_NODE, chn);
            wait for 20 ns;

            get_tx_buf_state(txt_buf_index, txtb_state, DUT_NODE, chn);
            check_m(txtb_state = buf_ab_progress, "TXT Buffer in Abort in Progress");

            CAN_insert_TX_frame(CAN_frame_b, txt_buf_index, DUT_NODE, chn);

            ---------------------------------------------------------------------------------------
            -- @1.5 Wait until bus is idle. Read the frame from RX Buffer of
            --      Test Node. Compare the read frame with Frame A.
            ---------------------------------------------------------------------------------------
            info_m("Step 1.5");

            CAN_wait_bus_idle(DUT_NODE, chn);
            CAN_wait_bus_idle(TEST_NODE, chn);

            CAN_read_frame(CAN_RX_frame, TEST_NODE, chn);

            CAN_compare_frames(CAN_RX_frame, CAN_frame_a, false, frames_match);
            check_m(frames_match, "Frame A not corrupted");

            get_tx_buf_state(txt_buf_index, txtb_state, DUT_NODE, chn);
            check_m(txtb_state = buf_done, "TXT Buffer in TX Done");

        end loop;

  end procedure;
end package body;
