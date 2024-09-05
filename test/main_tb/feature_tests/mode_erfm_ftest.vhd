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
--  RX Error logging mode - feature test
--
-- @Verifies:
--  @1. When MODE[ERFM_IND] = 0, Error frames are not logged into RX Buffer.
--  @2. When MODE[ERFM_IND] = 1, Error frames are logged into RX Buffer.
--
-- @Test sequence:
--  @1. Set MODE[ERFM] = 0 in DUT. Set MODE[ACF] = 1 in Test Node.
--  @2. Generate CAN frame, and send it by DUT. Wait until Error frame
--      is transmitted. Wait until Bus is idle.
--  @3. Check that DUT RX Buffer has 0 frames in it. Check that both DUT RX
--      Buffer pointers are 0.
--  @4. Set MODE[ERFM] = 1 in DUT.
--  @5. Generate CAN frame and send it by DUT. Wait until Error frame is
--      transmitted. Wait until bus is idle.
--  @6. Check that DUT RX Buffer has 1 frame in it. Check that RX Buffer Write
--      pointer is 4.
--  @7. Read out the frame from RX Buffer and check that:
--          - FRAME_FORMAT_W[IVLD] = 1
--          - FRAME_FORMAT_W[ERF] = 1
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    5.8.2024   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package mode_erfm_ftest is
    procedure mode_erfm_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body mode_erfm_ftest is

    procedure mode_erfm_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable mode_1             : SW_mode := SW_mode_rst_val;
        variable mode_2             : SW_mode := SW_mode_rst_val;

        variable CAN_frame          : SW_CAN_frame_type;
        variable err_frame          : SW_CAN_frame_type;

        variable rx_buf_info        : SW_RX_Buffer_info;

        variable frame_sent         : boolean;
    begin

        -------------------------------------------------------------------------------------------
        -- @1. Set MODE[ERFM] = 0 in DUT. Set MODE[ACF] = 1 in Test Node.
        -------------------------------------------------------------------------------------------
        info_m("Step 1");

        mode_1.error_logging := false;
        set_core_mode(mode_1, DUT_NODE, chn);

        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, TEST_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @2. Generate CAN frame, and send it by DUT. Wait until Error frame is transmitted.
        --      Wait until Bus is idle.
        -------------------------------------------------------------------------------------------
        info_m("Step 2");

        CAN_generate_frame(CAN_frame);
        CAN_send_frame(CAN_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_error_frame(DUT_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @3. Check that DUT RX Buffer has 0 frames in it. Check that both DUT RX
        --     Buffer pointers are 0.
        -------------------------------------------------------------------------------------------
        info_m("Step 3");

        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        check_m(rx_buf_info.rx_mem_free = rx_buf_info.rx_buff_size,
                    "RX Buffer size = RX Buffer Free Memory");
        check_m(rx_buf_info.rx_frame_count = 0, "RX Frame count = 0");
        check_m(rx_buf_info.rx_write_pointer = 0, "RX Write pointer = 0");
        check_m(rx_buf_info.rx_read_pointer = 0, "RX Write pointer = 0");

        -------------------------------------------------------------------------------------------
        -- @4. Set MODE[ERFM] = 1 in DUT.
        -------------------------------------------------------------------------------------------
        info_m("Step 4");

        mode_1.error_logging := true;
        set_core_mode(mode_1, DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @5. Generate CAN frame and send it by DUT. Wait until Error frame is transmitted.
        --     Wait until bus is idle.
        -------------------------------------------------------------------------------------------
        info_m("Step 5");

        CAN_generate_frame(CAN_frame);
        CAN_send_frame(CAN_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_error_frame(DUT_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @6. Check that DUT RX Buffer has 1 framae in it. Check that RX Buffer Write
        --     pointer is 4.
        -------------------------------------------------------------------------------------------
        info_m("Step 6");

        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        check_m(rx_buf_info.rx_mem_free = rx_buf_info.rx_buff_size - 4,
                    "RX Buffer size - 4 = RX Buffer Free Memory");
        check_m(rx_buf_info.rx_write_pointer = 4, "RX Buffer Write pointer = 4");
        check_m(rx_buf_info.rx_read_pointer = 0, "RX Buffer Read pointer = 0");
        check_m(rx_buf_info.rx_frame_count = 1, "RX Buffer frame count = 1");

        -------------------------------------------------------------------------------------------
        -- @7. Read out the frame from RX Buffer and check that:
        --         - FRAME_FORMAT_W[IVLD] = 1
        --         - FRAME_FORMAT_W[ERF] = 1
        -------------------------------------------------------------------------------------------
        info_m("Step 7");

        CAN_read_frame(err_frame, DUT_NODE, chn);
        check_m(err_frame.erf = '1',                            "FRAME_FORMAT_W[ERF] = 1");
        check_m(err_frame.ivld = '1',                           "FRAME_FORMAT_W[IVLD] = 1");
        check_m(err_frame.identifier = CAN_frame.identifier,    "Identifier match");

  end procedure;

end package body;
