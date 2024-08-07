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
--  RX Error logging feature test - Back to Back Error frames
--
-- @Verifies:
--  @1. Multiple consecutive Error frames logged back-to-back.
--
-- @Test sequence:
--  @1. Configure DUT to MODE[ERFM] = 1. Disable SSP in DUT. Configure
--      fastest possible Nominal and Data Bit-rate for DUT and Test Node.
--  @2. Generate CAN frame. Send it by DUT and wait until CRC field.
--      Flip bus value to force bit error.
--  @3. Wait until sample point and check DUT is transmitting Error frame.
--  @4. Repeat waiting until sample point 16 times, and check that after
--      each sample point DUTs RX Buffer contains one more Error frame.
--      Check DUT becomes bus-off.
--  @5. Read all 16 Error frames logged by DUT, and check that all of them have
--      ERF_TYPE = ERC_BIT_ERR. Check that frames have ERF_ERP = 0, Check that
--      timestamp of error frames are monotonically incrementing.
--  @6. Wait for 16 x 8 bit times. Bus is still flipped so after each next 8
--      frames, next Error frame is going to be started! Check that each
--      time, a new error
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

package rx_err_log_back_to_back_ftest is
    procedure rx_err_log_back_to_back_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body rx_err_log_back_to_back_ftest is

    procedure rx_err_log_back_to_back_ftest_exec(
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
        variable err_frame_2        : SW_CAN_frame_type;
        variable rand_bits          : natural;
        variable bit_timing         : bit_time_config_type;
        variable ts_scratchpad      : std_logic_vector(63 downto 0);
    begin

        -------------------------------------------------------------------------------------------
        -- @1. Configure DUT to MODE[ERFM] = 1. Disable SSP in DUT. Configure
        --     fastest possible Nominal and Data Bit-rate for DUT and Test Node.
        -------------------------------------------------------------------------------------------
        info_m("Step 1");

        mode_1.error_logging := true;
        set_core_mode(mode_1, DUT_NODE, chn);

        bit_timing.tq_nbt     := 1;
        bit_timing.tq_dbt     := 1;

        bit_timing.prop_nbt   := 3;
        bit_timing.ph1_nbt    := 3;
        bit_timing.ph2_nbt    := 3;
        bit_timing.sjw_nbt    := 1;

        bit_timing.prop_dbt   := 0;
        bit_timing.ph1_dbt    := 2;
        bit_timing.ph2_dbt    := 2;
        bit_timing.sjw_dbt    := 1;

        CAN_turn_controller(false, DUT_NODE, chn);
        CAN_turn_controller(false, TEST_NODE, chn);

        CAN_configure_timing(bit_timing, DUT_NODE, chn);
        CAN_configure_timing(bit_timing, TEST_NODE, chn);

        CAN_configure_ssp(ssp_no_ssp, x"00", DUT_NODE, chn);
        CAN_configure_ssp(ssp_no_ssp, x"00", TEST_NODE, chn);

        CAN_turn_controller(true, DUT_NODE, chn);
        CAN_turn_controller(true, TEST_NODE, chn);

        CAN_wait_bus_on(DUT_NODE, chn);
        CAN_wait_bus_on(TEST_NODE, chn);

        ftr_tb_set_tran_delay(1 ns, DUT_NODE, chn);
        ftr_tb_set_tran_delay(1 ns, TEST_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @2. Generate CAN frame. Send it by DUT and wait till CRC field.
        --     Flip bus value to force bit error.
        -------------------------------------------------------------------------------------------
        info_m("Step 2");

        CAN_generate_frame(CAN_frame);
        CAN_frame.frame_format := FD_CAN;
        CAN_frame.brs := BR_SHIFT;
        CAN_send_frame(CAN_frame, 1, DUT_NODE, chn, frame_sent);

        CAN_wait_pc_state(pc_deb_crc, DUT_NODE, chn);

        flip_bus_level(chn);

        -------------------------------------------------------------------------------------------
        -- @3. Wait until sample point and check DUT is transmitting Error frame.
        -------------------------------------------------------------------------------------------
        info_m("Step 3");

        CAN_wait_sample_point(DUT_NODE, chn);

        get_controller_status(status, DUT_NODE, chn);
        check_m(status.error_transmission, "Error frame is being transmitted!");

        -------------------------------------------------------------------------------------------
        -- @4. Repeat waiting until sample point 32 times, and check that after
        --     each sample point DUTs RX Buffer contains one more Error frame.
        --     Check DUT becomes bus-off.
        -------------------------------------------------------------------------------------------
        info_m("Step 4");

        for i in 1 to 16 loop
            CAN_wait_sample_point(DUT_NODE, chn);
            get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
            check_m(rx_buf_info.rx_frame_count = i + 1,
                    "Exptected frames in RX Buffer: " & integer'image(i + 1) &
                    " Real frames in RX Buffer: " & integer'image(rx_buf_info.rx_frame_count));
        end loop;

        release_bus_level(chn);
        CAN_wait_bus_idle(DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @5. Read all 16 Error frames logged by DUT, and check that all of them have
        --     ERF_TYPE = ERC_BIT_ERR. Check that frames have ERF_ERP = 0.
        --     Check that timestamp of error frames are monotonically incrementing.
        -------------------------------------------------------------------------------------------
        ts_scratchpad := (others => '0');

        for i in 1 to 17 loop
            info_m("Reading Error frame: " & integer'image(i));
            CAN_read_frame(err_frame, DUT_NODE, chn);

            check_m(err_frame.erf = '1',                    "FRAME_FORMAT_W[ERF] = 1");
            check_m(err_frame.ivld = '1',                   "FRAME_FORMAT_W[IVLD] = 1");

            if (i = 1) then
                check_m(err_frame.erf_type = ERC_FRM_ERR,   "FRAME_FORMAT_W[ERR_TYPE] = ERC_FRM_ERR");
                check_m(err_frame.erf_pos = ERC_POS_CRC,    "FRAME_FORMAT_W[ERF_POS] = ERC_POS_CRC");
            else
                check_m(err_frame.erf_type = ERC_BIT_ERR,   "FRAME_FORMAT_W[ERR_TYPE] = ERC_BIT_ERR");
                check_m(err_frame.erf_pos = ERC_POS_ERR,    "FRAME_FORMAT_W[ERF_POS] = ERC_POS_ERR");
            end if;

            if (i = 17) then
                check_m(err_frame.erf_erp = '1', "FRAME_FORMAT_W[ERF_ERP] = 1");
            else
                check_m(err_frame.erf_erp = '0', "FRAME_FORMAT_W[ERF_ERP] = 0");
            end if;

            check_m(unsigned(err_frame.timestamp) > unsigned(ts_scratchpad),
                    "Timestamp monotonically incrementing");

            ts_scratchpad := err_frame.timestamp;
        end loop;

    end procedure;

end package body;
