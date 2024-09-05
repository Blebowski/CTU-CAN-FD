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
--  RX Error logging feature test 5
--
-- @Verifies:
--  @1. RX Error being logged with following ERF_TYPE values:
--      @1.1 ERC_PRT_ERR
--
-- @Test sequence:
--  @1. Configure DUT to MODE[ERFM] = 1 and enable Test mode and Parity in DUT
--      Node.
--  @2. Generate CAN frame, and insert it to DUTs TXT Buffer. Use Test Registers
--      to flip a bit in the TXT Buffer word containing first 4 bytes of CAN
--      Data field.
--  @3. Give DUTs TXT Buffer a "Set Ready" Command. Wait until Error frame
--      occurs in DUT Node. Check DUTs RX Buffer has a single frame in its RX
--      Buffer. Wait until bus is idle. Read the frame, and check it is an
--      Error frame and it has ERF_TYPE = ERC_PRT_ERR.
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

package rx_err_log_5_ftest is
    procedure rx_err_log_5_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body rx_err_log_5_ftest is

    procedure rx_err_log_5_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable mode_1             : SW_mode := SW_mode_rst_val;
        variable CAN_frame          : SW_CAN_frame_type;
        variable err_frame          : SW_CAN_frame_type;
        variable corrupt_bit_index  : integer;
        variable rx_buf_info        : SW_RX_Buffer_info;
        variable r_data             : std_logic_vector(31 downto 0);
    begin

        -------------------------------------------------------------------------------------------
        --  @1. Configure DUT to MODE[ERFM] = 1, enable Test mode and Parity in DUT Node.
        -------------------------------------------------------------------------------------------
        info_m("Step 1");

        mode_1.error_logging := true;
        mode_1.test := true;
        mode_1.parity_check := true;
        set_core_mode(mode_1, DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @2. Generate CAN frame, and insert it to DUTs TXT Buffer. Use Test Registers
        --     to flip a bit in the TXT Buffer word containing first 4 bytes of CAN Data field.
        -------------------------------------------------------------------------------------------
        info_m("Step 2");

        CAN_generate_frame(CAN_frame);
        CAN_frame.data_length := 8;
        CAN_frame.rtr := NO_RTR_FRAME;
        decode_length(CAN_frame.data_length, CAN_frame.dlc);

        CAN_insert_TX_frame(CAN_frame, 1, DUT_NODE, chn);

        -- Enable test access
        set_test_mem_access(true, DUT_NODE, chn);

        -- Read, flip, and write back
        rand_int_v(31, corrupt_bit_index);
        test_mem_read(r_data, 5, txt_buf_to_test_mem_tgt(1), DUT_NODE, chn);
        r_data(corrupt_bit_index) := not r_data(corrupt_bit_index);
        test_mem_write(r_data, 5, txt_buf_to_test_mem_tgt(1), DUT_NODE, chn);

        -- Disable test mem access
        set_test_mem_access(false, DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @3. Give DUTs TXT Buffer a "Set Ready" Command. Wait until Error frame occurs in DUT
        --     Node. Check DUTs RX Buffer has a single frame in its RX Buffer. Wait until bus is
        --     idle. Read the frame, and check it is an Error frame and it has
        --     ERF_TYPE = ERC_PRT_ERR.
        -------------------------------------------------------------------------------------------
        info_m("Step 3");

        send_TXT_buf_cmd(buf_set_ready, 1, DUT_NODE, chn);
        CAN_wait_error_frame(DUT_NODE, chn);

        wait for 100 ns;

        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        check_m(rx_buf_info.rx_frame_count = 1, "Single Error frame in RX Buffer!");

        CAN_read_frame(err_frame, DUT_NODE, chn);
        check_m(err_frame.erf = '1', "FRAME_FORMAT_W[ERF] = 1");
        check_m(err_frame.erf_type = ERC_PRT_ERR, "FRAME_FORMAT_W[ERF_TYPE] = ERC_PRT_ERR");
        check_m(err_frame.ivld = '1', "FRAME_FORMAT_W[IVLD] = 1");

        CAN_wait_bus_idle(DUT_NODE, chn);

    end procedure;

end package body;
