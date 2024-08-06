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
--  RX Error logging feature tests
--
-- @Verifies:
--  @1. RX Error being logged with following ERF_POS values:
--      @1.1 ERF_POS_ARB
--      @1.2 ERF_POS_CTRL
--      @1.3 ERF_POS_DATA
--      @1.4 ERF_POS_CRC
--
-- @Test sequence:
--  @1. Configure DUT to MODE[ERFM] = 1.
--  @2. Iterate through Arbitration, Control, Data and CRC fields of
--      Protocol control:
--      @2.1 Generate CAN frame and send it by DUT.
--      @2.2 Wait until Protocol control field being tested and wait until
--           Dominant bit.
--      @2.3 Flip bus value. Wait until sample point. Release bus value.
--      @2.4 Check Error frame is being transmitted. Check single frame is
--           in RX Buffer.
--      @2.5 Wait until bus is idle. Read the frame from the RX Buffer.
--           Check its ERF_POS value is as expected. Check its ERF bit is set.
--           Check its IVLD bit is set apart from Error frame in Arbitration.
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

package rx_err_log_ftest is
    procedure rx_err_log_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body rx_err_log_ftest is

    procedure rx_err_log_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable mode_1             : SW_mode := SW_mode_rst_val;
        variable pc_dbg             : SW_PC_Debug;
        variable frame_sent         : boolean;
        variable tx_val             : std_logic;
        variable status             : SW_status;
        variable rx_buf_info        : SW_RX_Buffer_info;
        variable CAN_frame          : SW_CAN_frame_type;
        variable err_frame          : SW_CAN_frame_type;
    begin

        -------------------------------------------------------------------------------------------
        -- @1. Configure DUT to MODE[ERFM] = 1.
        -------------------------------------------------------------------------------------------
        info_m("Step 1");

        mode_1.error_logging := true;
        set_core_mode(mode_1, DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @2. Iterate through Arbitration, Control, Data and CRC fields of
        --      Protocol control:
        -------------------------------------------------------------------------------------------
        info_m("Step 2");

        for i in 0 to 3 loop
            case (i) is
            when 0 => pc_dbg := pc_deb_arbitration;
            when 1 => pc_dbg := pc_deb_control;
            when 2 => pc_dbg := pc_deb_data;
            when others => pc_dbg := pc_deb_crc;
            end case;

            info_m("Testing ERF_POS in: " & SW_PC_Debug'image(pc_dbg));

            ---------------------------------------------------------------------------------------
            -- @2.1 Generate CAN frame and send it by DUT.
            ---------------------------------------------------------------------------------------
            info_m("Step 2.1");

            CAN_generate_frame(CAN_frame);
            CAN_frame.rtr := NO_RTR_FRAME;
            CAN_frame.frame_format := NORMAL_CAN;
            CAN_frame.data_length := 8;     -- To make sure data field is there!
            decode_length(CAN_frame.data_length, CAN_frame.dlc);

            CAN_send_frame(CAN_frame, 1, DUT_NODE, chn, frame_sent);

            ---------------------------------------------------------------------------------------
            -- @2.2 Wait until Protocol control field being tested and wait until Dominant bit.
            ---------------------------------------------------------------------------------------
            info_m("Step 2.2");

            CAN_wait_pc_state(pc_dbg, DUT_NODE, chn);
            CAN_wait_sample_point(DUT_NODE, chn);

            dom_bit_wait : while (true) loop
                CAN_wait_sync_seg(DUT_NODE, chn);
                wait for 20 ns;
                get_can_tx(DUT_NODE, tx_val, chn);
                if (tx_val = DOMINANT) then
                    exit dom_bit_wait;
                end if;
            end loop;

            ---------------------------------------------------------------------------------------
            -- @2.3 Flip bus value. Wait until sample point. Release bus value.
            ---------------------------------------------------------------------------------------
            info_m("Step 2.3");

            flip_bus_level(chn);

            CAN_wait_sample_point(DUT_NODE, chn, false);
            wait for 20 ns;

            release_bus_level(chn);

            -- Need to wait for few cycles so that the Error frame is stored there!
            wait for 100 ns;

            ---------------------------------------------------------------------------------------
            -- @2.4 Check Error frame is being transmitted. Check single frame is in RX Buffer.
            ---------------------------------------------------------------------------------------
            info_m("Step 2.4");

            get_controller_status(status, DUT_NODE, chn);
            check_m(status.error_transmission, "Error frame is being transmitted!");

            get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
            check_m(rx_buf_info.rx_frame_count = 1, "Single Error frame in RX Buffer!");

            ---------------------------------------------------------------------------------------
            -- @2.5 Wait until bus is idle. Read the frame from the RX Buffer.
            --      Check its ERF_POS value is as expected. Check its ERF bit is set.
            --      Check its IVLD bit is set apart from Error frame in Arbitration.
            ---------------------------------------------------------------------------------------
            info_m("Step 2.5");

            CAN_wait_bus_idle(DUT_NODE, chn);
            CAN_read_frame(err_frame, DUT_NODE, chn);
            check_m(err_frame.erf = '1',                            "FRAME_FORMAT_W[ERF] = 1");

            -- All fields apart from arbitration shall have IVLD set!
            if (i > 0) then
                check_m(err_frame.ivld = '1',                       "FRAME_FORMAT_W[IVLD] = 1");
                check_m(err_frame.identifier = CAN_frame.identifier,"Identifier match");
            else
                check_m(err_frame.ivld = '0',                       "FRAME_FORMAT_W[IVLD] = 0");
            end if;

            case (i) is
            when 0      => check_m(err_frame.erf_pos = ERC_POS_ARB,    "ERF_POS = Arbitration");
            when 1      => check_m(err_frame.erf_pos = ERC_POS_CTRL,   "ERF_POS = Control");
            when 2      => check_m(err_frame.erf_pos = ERC_POS_DATA,   "ERF_POS = Data");
            when others => check_m(err_frame.erf_pos = ERC_POS_CRC,    "ERF_POS = CRC");
            end case;

        end loop;

  end procedure;

end package body;
