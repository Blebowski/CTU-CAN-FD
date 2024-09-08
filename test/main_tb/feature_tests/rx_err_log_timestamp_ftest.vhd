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
--  RX Error logging feature test - Timestamp test.
--
-- @Verifies:
--  @1. Timestamp is recorded in Error frames logged to RX Buffer.
--
-- @Test sequence:
--  @1. Configure DUT to MODE[ERFM] = 1.
--  @2. Generate random CAN frame without bit-rate shift and insert it to DUT
--      for transmission. Wait until DUTs sample point and issue Set ready
--      Command to a TXT buffer containing the CAN frame. Sample current
--      value of timestamp.
--  @3. Wait for random number of DUT bits with dominant bit being sent.
--      Count number of bits waited. Flip bus value, and wait until sample
--      point. Wait until Error frame is transmitted and release bus level.
--      Wait until bus is idle.
--  @4. Pre-compute expected value of Logged RX Error frame timestamp based on
--      number of cycles in the nominal bit-rate and number of elapsed bits
--      before invoking the error. Check the read timestamp of logged Error
--      frame matches the expected timestamp of Error frame.
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

package rx_err_log_timestamp_ftest is
    procedure rx_err_log_timestamp_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body rx_err_log_timestamp_ftest is

    procedure rx_err_log_timestamp_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          : SW_CAN_frame_type;
        variable err_frame          : SW_CAN_frame_type;

        variable mode_1             : SW_mode := SW_mode_rst_val;

        variable rand_vect          : std_logic_vector(31 downto 0);
        variable ts_start           : std_logic_vector(63 downto 0);
        variable n_bits             : natural;
        variable can_tx             : std_logic;
        variable pc_dbg             : SW_PC_Debug;
        variable exit_flag          : std_logic;
        variable exp_ts             : unsigned(63 downto 0);
        variable bust               : bit_time_config_type;
        variable diff               : unsigned(63 downto 0);
        variable cycles_per_bit     : integer;

        variable rx_buf_info        : SW_RX_Buffer_info;
        variable err_counters       : SW_error_counters;
    begin

        -------------------------------------------------------------------------------------------
        -- @1. Configure DUT to MODE[ERFM] = 1. Set random value of timestamp.
        -------------------------------------------------------------------------------------------
        info_m("Step 1");

        mode_1.error_logging := true;
        mode_1.test := true;
        set_core_mode(mode_1, DUT_NODE, chn);

        -- Reset error counters not to go error passive in many iterations!
        err_counters.tx_counter := 0;
        set_error_counters(err_counters, DUT_NODE, chn);

        rand_logic_vect_v(rand_vect, 0.5);
        ftr_tb_set_timestamp(('0' & rand_vect(30 downto 0) & rand_vect), chn);

        CAN_read_timing_v(bust, DUT_NODE, chn);
        cycles_per_bit := bust.tq_nbt * (1 + bust.prop_nbt + bust.ph1_nbt + bust.ph2_nbt);

        -------------------------------------------------------------------------------------------
        -- @2. Generate random CAN frame without bit-rate shift and insert it to DUT
        --     for transmission. Wait until DUTs sample point and issue Set ready
        --     Command to a TXT buffer containing the CAN frame.
        -------------------------------------------------------------------------------------------
        info_m("Step 2");

        CAN_generate_frame(CAN_frame);
        -- This is to easy compute the expected timestamp in Error frame!
        CAN_frame.brs := BR_NO_SHIFT;

        CAN_insert_TX_frame(CAN_frame, 1, DUT_NODE, chn);
        CAN_wait_sample_point(DUT_NODE, chn, false);
        send_TXT_buf_cmd(buf_set_ready, 1, DUT_NODE, chn);

        CAN_read_timestamp(ts_start, DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @3. Wait for random number of DUT bits with dominant bit being sent.
        --     Count number of bits waited. Flip bus value, and wait until sample
        --     point. Wait until Error frame is transmitted and release bus level.
        --     Wait until bus is idle.
        -------------------------------------------------------------------------------------------
        info_m("Step 3");
        n_bits := 0;

        while (true) loop
            n_bits := n_bits + 1;
            CAN_wait_sample_point(DUT_NODE, chn, false);
            CAN_wait_sync_seg(DUT_NODE, chn);
            wait for 20 ns;

            get_can_tx(DUT_NODE, can_tx, chn);
            CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
            rand_logic_v(exit_flag, 0.025);

            -- With a chance of 2.5 % quit on a Dominant bit. At latest in CRC Delim!
            if ((exit_flag = '1' and can_tx = DOMINANT) or (pc_dbg = pc_deb_crc_delim)) then
                exit;
            end if;
        end loop;

        flip_bus_level(chn);
        CAN_wait_sample_point(DUT_NODE, chn, false);
        wait for 20 ns;
        release_bus_level(chn);

        CAN_wait_error_frame(DUT_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @4. Pre-compute expected value of Logged RX Error frame timestamp based on
        --     number of cycles in the nominal bit-rate and number of elapsed bits
        --     before invoking the error. Check the read timestamp of logged Error
        --     frame matches the expected timestamp of Error frame.
        -------------------------------------------------------------------------------------------
        info_m("Step 4");

        exp_ts := unsigned(ts_start) + (n_bits + 1) * cycles_per_bit;

        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        check_m(rx_buf_info.rx_frame_count = 1, "Single Error frame in RX Buffer!");

        CAN_read_frame(err_frame, DUT_NODE, chn);
        check_m(err_frame.erf = '1', "FRAME_FORMAT_W[ERF] = 1");

        -- Check timestamp !
        if (exp_ts > unsigned(err_frame.timestamp)) then
            diff := exp_ts - unsigned(err_frame.timestamp);
        else
            diff := unsigned(err_frame.timestamp) - exp_ts;
        end if;

        info_m("Expected timestamp: " & to_hstring(exp_ts));
        info_m("Observed timestamp: " & to_hstring(err_frame.timestamp));
        info_m("Diff:               " & to_hstring(diff));

        if (diff < 10) then
            info_m("Timestamp OK");
        else
            info_m("Timestamp Error");
        end if;

    end procedure;

end package body;
