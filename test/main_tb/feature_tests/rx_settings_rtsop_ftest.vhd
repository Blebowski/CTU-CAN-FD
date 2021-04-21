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
--  RX Settings Timestamp options feature test.
--
-- @Verifies:
--  @1. RX frame timestamp in sample point of SOF.
--  @2. RX frame timestamp in sample point of EOF.
--  @3. RX frame timestamp when receiving CAN frame without SOF.
--
-- @Test sequence:
--  @1. Configure timestamp to be captured in SOF. Set Loopback mode in DUT
--      (so that we are sure that SOF is transmitted). Generate CAN frame and
--      issue it for transmission by DUT. Wait until DUT turns transmitter
--      and wait till Sample point. Capture timestamp and wait till frame is sent
--      Check that RX frame timestamp is equal to captured timestamp!
--  @2. Generate CAN frame for transmission and send it by Test node. Poll until
--      DUT becomes receiver (this should be right after sample point of
--      Dominant bit in Idle which is interpreted as SOF) and capture timestamp.
--      Wait until CAN frame is sent and check that RX frame timestamp is equal
--      to captured timestamp.
--  @3. Configure timestamp to be captured at EOF. Generate CAN frame and send
--      it by Test node. Wait until EOF and then until EOF ends. At EOF end, capture
--      timestamp and wait till bus is idle! Compare captured timestamp with
--      RX frame timestamp.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    29.6.2018     Created file
--   19.11.2019     Re-wrote to cover more stuff and be more exact! 
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.timestamp_agent_pkg.all;

package rx_settings_rtsop_ftest is
    procedure rx_settings_rtsop_ftest_exec(
        signal      chn             : inout  t_com_channel
	);
end package;


package body rx_settings_rtsop_ftest is
    procedure rx_settings_rtsop_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_TX_frame       :        SW_CAN_frame_type;
        variable CAN_RX_frame       :        SW_CAN_frame_type;
        variable frame_sent         :        boolean := false;
        variable options            :        SW_RX_Buffer_options;
        variable ts_beg             :        std_logic_vector(31 downto 0);
        variable ts_end             :        std_logic_vector(31 downto 0);
        variable diff               :        unsigned(63 downto 0);

        variable rx_options         :        SW_RX_Buffer_options; 
        variable mode_1             :        SW_mode := SW_mode_rst_val;

        variable rand_ts            :        std_logic_vector(63 downto 0);        
        variable capt_ts            :        std_logic_vector(63 downto 0);
    begin

        -----------------------------------------------------------------------
        -- @1. Configure timestamp to be captured in SOF. Set Loopback mode in 
        --     DUT (so that we are sure that SOF is transmitted). Generate
        --     CAN frame and issue it for transmission by DUT. Wait until
        --     DUT turns transmitter and wait till Sample point. Capture 
        --     timestamp, and wait till frame is sent Check that RX frame
        --     timestamp is equal to captured timestamp!
        -----------------------------------------------------------------------
        info_m("Step 1");

        -- Force random timestamp so that we are sure that both words of the
        -- timestamp are clocked properly!
        rand_logic_vect_v(rand_ts, 0.5);
        -- Keep highest bit 0 to avoid complete overflow during the test!
        rand_ts(63) := '0';
        
        -- Timestamp is realized as two 32 bit naturals! Due to this, we can't
        -- have 1 in MSB to avoid overflow.
        rand_ts(31) := '0';

        ftr_tb_set_timestamp(rand_ts, chn);
        info_m("Forcing start timestamp in DUT to: " & to_hstring(rand_ts));

        rx_options.rx_time_stamp_options := true;
        set_rx_buf_options(rx_options, DUT_NODE, chn);

        mode_1.internal_loopback := true;
        set_core_mode(mode_1, DUT_NODE, chn);

        CAN_generate_frame(CAN_TX_frame);
        CAN_send_frame(CAN_TX_frame, 1, DUT_NODE, chn, frame_sent);

        CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);

        -- Now we are in SOF, so we must wait till sample point and capture
        -- the timestamp then. HW should do the same!
        CAN_wait_sample_point(DUT_NODE, chn);
        timestamp_agent_get_timestamp(chn, capt_ts);

        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);

        CAN_read_frame(CAN_RX_frame, DUT_NODE, chn);

        -- Calculate difference. Two separate cases are needed to avoid
        -- underflow
        if (CAN_RX_frame.timestamp > capt_ts) then
            diff := unsigned(CAN_RX_frame.timestamp) - unsigned(capt_ts);
        else
            diff := unsigned(capt_ts) - unsigned(CAN_RX_frame.timestamp);
        end if;

        -- Have some margin on check, as we are polling status which takes
        -- non-zero time!
        check_m(diff <= 3, "Timestamp at SOF. " &
                         " Expected: " & to_hstring(capt_ts) & 
                         " Measured: " & to_hstring(CAN_RX_frame.timestamp) &
                         " Difference: " & to_hstring(diff));

        -----------------------------------------------------------------------
        -- @2. Generate CAN frame for transmission and send it by Test node. Poll
        --     until DUT becomes receiver (this should be right after sample
        --     point of Dominant bit in Idle which is interpreted as SOF) and
        --     capture timestamp. Wait until CAN frame is sent and check that RX
        --     frame timestamp is equal to captured timestamp.
        -----------------------------------------------------------------------
        info_m("Step 2");

        CAN_generate_frame(CAN_TX_frame);
        CAN_send_frame(CAN_TX_frame, 1, TEST_NODE, chn, frame_sent);

        CAN_wait_tx_rx_start(false, true, DUT_NODE, chn);

        -- Now we should go directly to Arbitration because we sample dominant
        -- bit in Idle, therefore this should be the moment of SOF sample point.
        timestamp_agent_get_timestamp(chn, capt_ts);

        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);

        CAN_read_frame(CAN_RX_frame, DUT_NODE, chn);

        -- Calculate difference. Two separate cases are needed to avoid
        -- underflow
        if (CAN_RX_frame.timestamp > capt_ts) then
            diff := unsigned(CAN_RX_frame.timestamp) - unsigned(capt_ts);
        else
            diff := unsigned(capt_ts) - unsigned(CAN_RX_frame.timestamp);
        end if;

        -- Have some margin on check, as we are polling status which takes
        -- non-zero time!
        check_m(diff <= 3, "Timestamp at SOF. " &
                         " Expected: " & to_hstring(capt_ts) & 
                         " Measured: " & to_hstring(CAN_RX_frame.timestamp) &
                         " Difference: " & to_hstring(diff));

        -----------------------------------------------------------------------
        -- @3. Configure timestamp to be captured at EOF. Generate CAN frame
        --    and send it by Test node. Wait until EOF and then until EOF ends. At
        --    EOF end, capture timestamp and wait till bus is idle! Compare
        --    captured timestamp with RX frame timestamp.
        -----------------------------------------------------------------------
        info_m("Step 3");
        
        rx_options.rx_time_stamp_options := false;
        set_rx_buf_options(rx_options, DUT_NODE, chn);

        CAN_generate_frame(CAN_TX_frame);
        CAN_send_frame(CAN_TX_frame, 1, TEST_NODE, chn, frame_sent);
        
        CAN_wait_pc_state(pc_deb_eof, DUT_NODE, chn);

        -- Wait until one bit before the end of EOF. This is when RX frame
        -- is validated according to CAN standard. 
        for i in 0 to 5 loop
            CAN_wait_sample_point(DUT_NODE, chn, false);
        end loop;
        
        -- Now we should be right in sample point of EOF. Get timestamp
        timestamp_agent_get_timestamp(chn, capt_ts);

        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);

        CAN_read_frame(CAN_RX_frame, DUT_NODE, chn);
        
        -- Calculate difference. Two separate cases are needed to avoid
        -- underflow
        if (CAN_RX_frame.timestamp > capt_ts) then
            diff := unsigned(CAN_RX_frame.timestamp) - unsigned(capt_ts);
        else
            diff := unsigned(capt_ts) - unsigned(CAN_RX_frame.timestamp);
        end if;

        -- Have some margin on check, as we are polling status which takes
        -- non-zero time!
        check_m(diff <= 3, "Timestamp at SOF. " &
                         " Expected: " & to_hstring(capt_ts) & 
                         " Measured: " & to_hstring(CAN_RX_frame.timestamp) &
                         " Difference: " & to_hstring(diff));

    end procedure;

end package body;