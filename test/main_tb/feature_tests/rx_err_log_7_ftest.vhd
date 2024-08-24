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
--  RX Error logging feature test 7
--
-- @Verifies:
--  @1. RX Error frame is logged correctly in RX Buffer when Error frame occurs
--      right after a data word is stored in TX Buffer
--
-- @Test sequence:
--  @1. Configure DUT to MODE[ERFM] = 1, and enable Loopback mode in DUT.
--      Set DUT to One-shot mode.
--  @2. Generate CAN frame and send it byt Test Node. Wait until frame is sent.
--  @3. Generate CAN frame with 8 bytes of data in it. Send the CAN frame
--      by DUT Node. Wait until last bit of 4-th byte of Data field and
--      flip bus value.
--  @4. Wait until error frame in DUT Node, release bus value. Check DUTs RX
--      Buffer contains a two RX frames. Wait until bus is idle.
--  @5. Generate CAN frame and send it byt Test Node. Wait until frame is sent.
--  @6. Read all 3 frames from DUTs RX Buffer, and check that first and third
--      frames are CAN frames matching first and third transmitted frame. Check
--      that secodn frame is an Error frame.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    24.8.2024   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package rx_err_log_7_ftest is
    procedure rx_err_log_7_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body rx_err_log_7_ftest is

    procedure rx_err_log_7_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable mode_1             : SW_mode := SW_mode_rst_val;

        variable tx_frame_1         : SW_CAN_frame_type;
        variable tx_frame_2         : SW_CAN_frame_type;
        variable tx_frame_3         : SW_CAN_frame_type;

        variable rx_frame_1         : SW_CAN_frame_type;
        variable rx_frame_2         : SW_CAN_frame_type;
        variable rx_frame_3         : SW_CAN_frame_type;

        variable frames_match       : boolean;
        variable frame_sent         : boolean;

        variable rx_buf_info        : SW_RX_Buffer_info;

    begin

        -------------------------------------------------------------------------------------------
        -- @1. Configure DUT to MODE[ERFM] = 1, and enable Loopback mode in DUT.
        --     Set DUT to One-shot mode.
        -------------------------------------------------------------------------------------------
        info_m("Step 1");

        mode_1.error_logging := true;
        mode_1.internal_loopback := true;
        set_core_mode(mode_1, DUT_NODE, chn);

        CAN_enable_retr_limit(true, 0, DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @2. Generate CAN frame and send it byt Test Node. Wait until frame is sent.
        -------------------------------------------------------------------------------------------
        info_m("Step 2");

        CAN_generate_frame(tx_frame_1);
        CAN_send_frame(tx_frame_1, 1, TEST_NODE, chn, frame_sent);
        CAN_wait_frame_sent(DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        --  @3. Generate CAN frame with 8 bytes of data in it. Send the CAN frame
        --      by DUT Node. Wait until last bit of 4-th byte of Data field and flip bus value.
        -------------------------------------------------------------------------------------------
        info_m("Step 3");

        CAN_generate_frame(tx_frame_2);
        tx_frame_2.rtr := NO_RTR_FRAME;
        tx_frame_2.data_length := 8;
        decode_length(tx_frame_2.data_length, tx_frame_2.dlc);

        CAN_send_frame(tx_frame_2, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);
        CAN_wait_pc_state(pc_deb_data, DUT_NODE, chn);

        -- First 31 bits of data. Just before last bit of 4-th byte of data.
        for i in 1 to 31 loop
            CAN_wait_sample_point(DUT_NODE, chn);
        end loop;

        CAN_wait_sync_seg(DUT_NODE, chn);
        flip_bus_level(chn);
        CAN_wait_sample_point(DUT_NODE, chn);
        wait for 20 ns;
        release_bus_level(chn);

        wait for 100 ns;

        -------------------------------------------------------------------------------------------
        -- @4. Wait until error frame in DUT Node, release bus value. Check DUTs RX
        --      Buffer contains a two RX frames. Wait until bus is idle.
        -------------------------------------------------------------------------------------------
        info_m("Step 4");

        CAN_wait_error_frame(DUT_NODE, chn);

        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        check_m(rx_buf_info.rx_frame_count = 2, "Two frames in RX Buffer!");

        CAN_wait_bus_idle(DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @5. Generate CAN frame and send it byt Test Node. Wait until frame is sent.
        -------------------------------------------------------------------------------------------
        info_m("Step 5");

        CAN_generate_frame(tx_frame_3);
        CAN_send_frame(tx_frame_3, 1, TEST_NODE, chn, frame_sent);
        CAN_wait_frame_sent(DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @6. Read all 3 frames from DUTs RX Buffer, and check that first and third
        --     frames are CAN frames matching first and third transmitted frame. Check
        --     that second frame is an Error frame.
        -------------------------------------------------------------------------------------------
        info_m("Step 6");

        CAN_read_frame(rx_frame_1, DUT_NODE, chn);
        CAN_read_frame(rx_frame_2, DUT_NODE, chn);
        CAN_read_frame(rx_frame_3, DUT_NODE, chn);

        CAN_compare_frames(rx_frame_1, tx_frame_1, false, frames_match);
        check_m(frames_match, "RX Frame 1 = TX Frame 1");
        check_m(rx_frame_1.erf = '0', "RX Frame 1 is CAN frame");

        check_m(rx_frame_2.erf = '1', "RX Frame 2 is Error frame");

        CAN_compare_frames(rx_frame_3, tx_frame_3, false, frames_match);
        check_m(frames_match, "RX Frame 3 = TX Frame 3");
        check_m(rx_frame_1.erf = '0', "RX Frame 3 is CAN frame");

    end procedure;

end package body;
