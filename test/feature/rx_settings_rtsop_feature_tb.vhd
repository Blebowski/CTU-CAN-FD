--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018
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
-- Purpose:
--  RX Settings Timestamp options feature test.
--
-- Verifies:
--  1. RX frame timestamp in sample point of SOF.
--  2. RX frame timestamp in sample point of EOF.
--  3. RX frame timestamp when receiving CAN frame without SOF.
--
-- Test sequence:
--  1. Configure timestamp to be captured in SOF. Set Loopback mode in Node 1
--     (so that we are sure that SOF is transmitted). Generate CAN frame and
--     issue it for transmission by Node 1. Wait until Node 1 turns transmitter
--     and wait till Sample point. Capture timestamp and wait till frame is sent
--     Check that RX frame timestamp is equal to captured timestamp!
--  2. Generate CAN frame for transmission and send it by Node 2. Poll until
--     Node 1 becomes receiver (this should be right after sample point of
--     Dominant bit in Idle which is interpreted as SOF) and capture timestamp.
--     Wait until CAN frame is sent and check that RX frame timestamp is equal
--     to captured timestamp.
--  3. Configure timestamp to be captured at EOF. Generate CAN frame and send
--     it by Node 2. Wait until EOF and then until EOF ends. At EOF end, capture
--     timestamp and wait till bus is idle! Compare captured timestamp with
--     RX frame timestamp.
--
--------------------------------------------------------------------------------
-- Revision History:
--    29.6.2018     Created file
--   19.11.2019     Re-wrote to cover more stuff and be more exact! 
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package rx_settings_rtsop_feature is
    procedure rx_settings_rtsop_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
	);
end package;


package body rx_settings_rtsop_feature is
    procedure rx_settings_rtsop_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable r_data             :        std_logic_vector(31 downto 0) :=
                                                 (OTHERS => '0');
        variable CAN_TX_frame       :        SW_CAN_frame_type;
        variable CAN_RX_frame       :        SW_CAN_frame_type;
        variable frame_sent         :        boolean := false;
        variable options            :        SW_RX_Buffer_options;
        variable ts_beg             :        std_logic_vector(31 downto 0);
        variable ts_end             :        std_logic_vector(31 downto 0);
        variable ID_1               :        natural := 1;
        variable ID_2               :        natural := 2;
        variable diff               :        unsigned(63 downto 0);

        variable rx_options         :        SW_RX_Buffer_options; 
        variable mode_1             :        SW_mode := SW_mode_rst_val;

        variable rand_ts            :        std_logic_vector(63 downto 0);        
        variable capt_ts            :        std_logic_vector(63 downto 0);
    begin

        -----------------------------------------------------------------------
        -- 1. Configure timestamp to be captured in SOF. Set Loopback mode in 
        --    Node 1 (so that we are sure that SOF is transmitted). Generate
        --    CAN frame and issue it for transmission by Node 1. Wait until
        --    Node 1 turns transmitter and wait till Sample point. Capture 
        --    timestamp, and wait till frame is sent Check that RX frame
        --    timestamp is equal to captured timestamp!
        -----------------------------------------------------------------------
        info("Step 1");

        -- Force random timestamp so that we are sure that both words of the
        -- timestamp are clocked properly!
        rand_logic_vect_v(rand_ctr, rand_ts, 0.5);
        -- Keep highest bit 0 to avoid complete overflow during the test!
        rand_ts(63) := '0';
        
        -- Timestamp is realized as two 32 bit naturals! Due to this, we can't
        -- have 1 in MSB to avoid overflow.
        rand_ts(31) := '0';

        ftr_tb_set_timestamp(rand_ts, ID_1, so.ts_preset, so.ts_preset_val);
        info("Forcing start timestamp in Node 1 to: " & to_hstring(rand_ts));

        rx_options.rx_time_stamp_options := true;
        set_rx_buf_options(rx_options, ID_1, mem_bus(1));

        mode_1.internal_loopback := true;
        set_core_mode(mode_1, ID_1, mem_bus(1));

        CAN_generate_frame(rand_ctr, CAN_TX_frame);
        CAN_send_frame(CAN_TX_frame, 1, ID_1, mem_bus(1), frame_sent);

        CAN_wait_tx_rx_start(true, false, ID_1, mem_bus(1));

        -- Now we are in SOF, so we must wait till sample point and capture
        -- the timestamp then. HW should do the same!
        CAN_wait_sample_point(iout(1).stat_bus);
        capt_ts := iout(1).stat_bus(STAT_TS_LOW + 63 downto STAT_TS_LOW);
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        CAN_read_frame(CAN_RX_frame, ID_1, mem_bus(1));

        -- Calculate difference. Two separate cases are needed to avoid
        -- underflow
        if (CAN_RX_frame.timestamp > capt_ts) then
            diff := unsigned(CAN_RX_frame.timestamp) - unsigned(capt_ts);
        else
            diff := unsigned(capt_ts) - unsigned(CAN_RX_frame.timestamp);
        end if;

        -- Have some margin on check, as we are polling status which takes
        -- non-zero time!
        check(diff <= 3, "Timestamp at SOF. " &
                         " Expected: " & to_hstring(capt_ts) & 
                         " Measured: " & to_hstring(CAN_RX_frame.timestamp) &
                         " Difference: " & to_hstring(diff));

        -----------------------------------------------------------------------
        -- 2. Generate CAN frame for transmission and send it by Node 2. Poll
        --    until Node 1 becomes receiver (this should be right after sample
        --    point of Dominant bit in Idle which is interpreted as SOF) and
        --    capture timestamp. Wait until CAN frame is sent and check that RX
        --    frame timestamp is equal to captured timestamp.
        -----------------------------------------------------------------------
        info("Step 2");
        CAN_generate_frame(rand_ctr, CAN_TX_frame);
        CAN_send_frame(CAN_TX_frame, 1, ID_2, mem_bus(2), frame_sent);

        CAN_wait_tx_rx_start(false, true, ID_1, mem_bus(1));

        -- Now we should go directly to Arbitration because we sample dominant
        -- bit in Idle, therefore this should be the moment of SOF sample point.
        capt_ts := iout(1).stat_bus(STAT_TS_LOW + 63 downto STAT_TS_LOW);

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        CAN_read_frame(CAN_RX_frame, ID_1, mem_bus(1));

        -- Calculate difference. Two separate cases are needed to avoid
        -- underflow
        if (CAN_RX_frame.timestamp > capt_ts) then
            diff := unsigned(CAN_RX_frame.timestamp) - unsigned(capt_ts);
        else
            diff := unsigned(capt_ts) - unsigned(CAN_RX_frame.timestamp);
        end if;

        -- Have some margin on check, as we are polling status which takes
        -- non-zero time!
        check(diff <= 3, "Timestamp at SOF. " &
                         " Expected: " & to_hstring(capt_ts) & 
                         " Measured: " & to_hstring(CAN_RX_frame.timestamp) &
                         " Difference: " & to_hstring(diff));

        -----------------------------------------------------------------------
        -- 3. Configure timestamp to be captured at EOF. Generate CAN frame
        --    and send it by Node 2. Wait until EOF and then until EOF ends. At
        --    EOF end, capture timestamp and wait till bus is idle! Compare
        --    captured timestamp with RX frame timestamp.
        -----------------------------------------------------------------------
        info("Step 3");
        
        rx_options.rx_time_stamp_options := false;
        set_rx_buf_options(rx_options, ID_1, mem_bus(1));

        CAN_generate_frame(rand_ctr, CAN_TX_frame);
        CAN_send_frame(CAN_TX_frame, 1, ID_2, mem_bus(2), frame_sent);
        
        CAN_wait_pc_state(pc_deb_eof, ID_1, mem_bus(1));

        -- Wait until one bit before the end of EOF. This is when RX frame
        -- is validated according to CAN standard. 
        for i in 0 to 5 loop
            CAN_wait_sample_point(iout(1).stat_bus, false);
        end loop;
        
        -- Now we should be right in sample point of EOF. Get timestamp
        capt_ts := iout(1).stat_bus(STAT_TS_LOW + 63 downto STAT_TS_LOW);

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        CAN_read_frame(CAN_RX_frame, ID_1, mem_bus(1));
        
        -- Calculate difference. Two separate cases are needed to avoid
        -- underflow
        if (CAN_RX_frame.timestamp > capt_ts) then
            diff := unsigned(CAN_RX_frame.timestamp) - unsigned(capt_ts);
        else
            diff := unsigned(capt_ts) - unsigned(CAN_RX_frame.timestamp);
        end if;

        -- Have some margin on check, as we are polling status which takes
        -- non-zero time!
        check(diff <= 3, "Timestamp at SOF. " &
                         " Expected: " & to_hstring(capt_ts) & 
                         " Measured: " & to_hstring(CAN_RX_frame.timestamp) &
                         " Difference: " & to_hstring(diff));

    end procedure;

end package body;