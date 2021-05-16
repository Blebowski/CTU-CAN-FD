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
--  TX arbitration and time transmittion feature test
--
-- @Verifies:
--  @1. Frame is transmitted from TXT Buffer at time given by Timestamp in
--      TIMESTAMP_U_W and TIMESTAMP_L_W words.
--  @2. When timestamp in TIMESTAMP_U_W and TIMESTAMP_L_W is lower than actual
--      timestamp input of CTU CAN FD, frame is transmitted immediately.
--  @3. When timestamp of a frame in higher priority TXT buffer is not yet elapsed,
--      but timestamp of a frame in lower priority TXT buffer has already elapsed,
--      frame from lower priority buffer is not transmitted before frame from
--      higher priority buffer.
--
-- @Test sequence:
--  @1. Configure random timestamp to testbench. Generate random frame and insert
--      it for transmission from TXT Buffer 1 of DUT. Generate random time of
--      transmission for the frame and Issue "set ready" command for TXT Buffer 1.
--      Wait until node turns transmitter! Read current timestamp from TB and
--      check that difference between expected time of transmission and actual
--      time when transmission started is less than 1 bit time (transmission
--      is not processed immediately, but in each sample point). Wait until
--      frame is sent.
--  @2. Generate CAN frame for transmission with timestamp lower than actual
--      value of timestamp input of CTU CAN FD. Insert it for transmission and
--      issue "set ready" command to this TXT Buffer. Wait until transmission
--      starts and check that difference between start of transmission and time
--      when "set ready" command was sent is less than 1 bit time. Wait until
--      frame is sent.
--  @3. Configure TXT Buffer 1 to higher priority than TXT Buffer 2. Insert CAN
--      frame with earlier time of transmission to TXT Buffer 1. Mark both frames
--      as ready. Wait until both frames are transmitted! Check that first frame
--      from DUT was transmitted!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    23.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--    13.06.2018  1. Used CAN Test lib instead of register access functions.
--                2. Removed transmission from multiple buffers, since buffers
--                   are now compared with priority. This will be covered in
--                   separate test.
--     08.1.2020  Re-wrote to be more exact! 
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.timestamp_agent_pkg.all;

package tx_arb_time_tran_ftest is
    procedure tx_arb_time_tran_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body tx_arb_time_tran_ftest is
    procedure tx_arb_time_tran_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :       SW_CAN_frame_type;
        variable CAN_frame_2        :       SW_CAN_frame_type;
        variable CAN_frame_rx_1     :       SW_CAN_frame_type;
        variable CAN_frame_rx_2     :       SW_CAN_frame_type;
        
        variable frame_sent         :       boolean := false;
        variable act_ts             :       std_logic_vector(63 downto 0);
        variable status             :       SW_Status;
        variable ts_offset          :       natural;
        variable ts_offset_padded   :       std_logic_vector(63 downto 0);
        variable ts_expected        :       std_logic_vector(63 downto 0);
        variable ts_rand            :       std_logic_vector(63 downto 0);
        variable frames_equal       :       boolean;
        
        variable ts_set_ready       :       std_logic_vector(63 downto 0);
        variable ts_tx_start        :       std_logic_vector(63 downto 0);
        variable ts_actual          :       std_logic_vector(63 downto 0);
        variable diff               :       natural;
        variable clk_per_bit        :       natural;
        
        variable bus_timing         :       bit_time_config_type;
        
        variable buf_1_index        :       natural;
        variable buf_2_index        :       natural;
    begin

        -------------------------------------------------------------------------
        -- @1. Configure random timestamp to testbench. Generate random frame and
        --     insert it for transmission from TXT Buffer 1 of DUT. Generate
        --     random time of transmission for the frame and Issue "set ready" 
        --     command for TXT Buffer 1. Wait until node turns transmitter! Read
        --     current timestamp from TB and check that difference between expected
        --     time of transmission and actual time when transmission started is 
        --     less than 1 bit time (transmission is not processed immediately, but
        --     in each sample point). Wait until frame is sent.
        -------------------------------------------------------------------------
        info_m("Step 1");

        -- Force random timestamp so that we are sure that both words of the
        -- timestamp are clocked properly!
        rand_logic_vect_v(ts_rand, 0.5);

        -- Keep highest bit 0 to avoid complete overflow during the test!
        ts_rand(63) := '0';

        -- Additionally, keep bit 31=0. This is because timestamp is internally
        -- in TB implemented from two naturals which are 0 .. 2^31 - 1. If we
        -- would generate bit 31=1, conversion "to_integer" from such unsigned
        -- value is out of scope of natural!
        ts_rand(31) := '0';

        CAN_generate_frame(CAN_frame);
        rand_int_v(10000, ts_offset);
        info_m("Offset from current time of transmission: " &
                integer'image(ts_offset));

        info_m("Random timestamp set: " & to_hstring(ts_rand));
        ftr_tb_set_timestamp(ts_rand, chn);

        ts_offset_padded := (OTHERS => '0');
        ts_offset_padded(31 downto 0) := std_logic_vector(to_unsigned(ts_offset, 32));
        ts_expected := std_logic_vector(unsigned(ts_rand) + to_unsigned(ts_offset, 64));
        CAN_frame.timestamp := ts_expected;
        info_m("Expected time of transmission: " & to_hstring(ts_expected));

        
        CAN_insert_TX_frame(CAN_frame, 1, DUT_NODE, chn);
        send_TXT_buf_cmd(buf_set_ready, 1, DUT_NODE, chn);

        -- Capture immediate timestamp after set ready and after TX start!
        timestamp_agent_get_timestamp(chn, ts_set_ready);
        CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);
        timestamp_agent_get_timestamp(chn, ts_tx_start);

        -- Read bit-time, calculate diff and check
        CAN_read_timing_v(bus_timing, DUT_NODE, chn);
        clk_per_bit := bus_timing.tq_nbt * (1 + bus_timing.prop_nbt +
                        bus_timing.ph1_nbt + bus_timing.ph2_nbt);
        if (unsigned(ts_tx_start) > unsigned(ts_expected)) then
            diff := to_integer(unsigned(ts_tx_start) - unsigned(ts_expected));
        else
            diff := to_integer(unsigned(ts_expected) - unsigned(ts_tx_start));
        end if;

        -- If it happends that RX trigger occurs just after timestamp matches,
        -- but frame validation is still in progress, then diff can be actually
        -- higher than one bit time by the length of validation!
        check_m(diff < clk_per_bit + 6, "Time of transmission correct!" &
            " Diff: " & integer'image(diff) &
            " Time per bit: " & integer'image(clk_per_bit));

        CAN_wait_bus_idle(DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- @2. Generate CAN frame for transmission with timestamp lower than
        --    actual value of timestamp input of CTU CAN FD. Insert it for
        --    transmission and issue "set ready" command to this TXT Buffer.
        --    Wait until transmission starts and check that difference between
        --    start of transmission and time when "set ready" command was sent
        --    is less than 1 bit time. Wait until frame is sent.
        ------------------------------------------------------------------------
        info_m("Step 2");

        timestamp_agent_get_timestamp(chn, ts_actual);
        CAN_generate_frame(CAN_frame);
        CAN_frame.timestamp := std_logic_vector(unsigned(ts_actual) - 1);
        
        CAN_insert_TX_frame(CAN_frame, 1, DUT_NODE, chn);
        send_TXT_buf_cmd(buf_set_ready, 1, DUT_NODE, chn);
        
        -- Capture immediate timestamp after set ready and after TX start!
        timestamp_agent_get_timestamp(chn, ts_set_ready);
        CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);
        timestamp_agent_get_timestamp(chn, ts_tx_start);

        if (unsigned(ts_tx_start) > unsigned(ts_set_ready)) then
            diff := to_integer(unsigned(ts_tx_start) - unsigned(ts_set_ready));
        else
            diff := to_integer(unsigned(ts_set_ready) - unsigned(ts_tx_start));
        end if;
        
        check_m(diff < clk_per_bit, "Time of transmission correct!");
        
        CAN_wait_frame_sent(DUT_NODE, chn);
        
        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);
        
        -- Read two dummy frames to empty RX Buffer for further reads!
        CAN_read_frame(CAN_frame_rx_1, TEST_NODE, chn);
        CAN_read_frame(CAN_frame_rx_1, TEST_NODE, chn);
        
        ------------------------------------------------------------------------
        -- @3. Configure TXT Buffer 1 to higher priority than TXT Buffer 2. Insert
        --    CAN frame with earlier time of transmission to TXT Buffer 2. Mark
        --    both frames as ready. Wait until both frames are transmitted! Check
        --    that first frame from Test node was transmitted!
        ------------------------------------------------------------------------
        info_m("Step 3");
        
        -- Generate buffers random!
        pick_random_txt_buffer(buf_1_index, DUT_NODE, chn);
        buf_2_index := buf_1_index;
        while (buf_2_index = buf_1_index) loop
            pick_random_txt_buffer(buf_2_index, DUT_NODE, chn);         
        end loop;

        CAN_configure_tx_priority(buf_2_index, 1, DUT_NODE, chn);
        CAN_configure_tx_priority(buf_1_index, 2, DUT_NODE, chn);
        info_m("Higher priority buffer: " & integer'image(buf_1_index));
        info_m("Lower priority buffer: " & integer'image(buf_2_index));

        CAN_generate_frame(CAN_frame);
        CAN_generate_frame(CAN_frame_2);
        
        timestamp_agent_get_timestamp(chn, CAN_frame.timestamp);
        CAN_frame_2.timestamp := std_logic_vector(unsigned(CAN_frame.timestamp) + 3000);
        
        CAN_insert_TX_frame(CAN_frame_2, buf_1_index, DUT_NODE, chn);
        CAN_insert_TX_frame(CAN_frame, buf_2_index, DUT_NODE, chn);

        send_TXT_buf_cmd(buf_set_ready, buf_1_index, DUT_NODE, chn);
        send_TXT_buf_cmd(buf_set_ready, buf_2_index, DUT_NODE, chn);

        CAN_wait_frame_sent(TEST_NODE, chn);
        CAN_wait_frame_sent(TEST_NODE, chn);
        
        CAN_read_frame(CAN_frame_rx_1, TEST_NODE, chn);
        CAN_read_frame(CAN_frame_rx_2, TEST_NODE, chn);
        
        CAN_compare_frames(CAN_frame_rx_1, CAN_frame_2, false, frames_equal);
        check_m(frames_equal,
            "Higher priority buffer sent first although it has later timestamp!");

        CAN_compare_frames(CAN_frame_rx_2, CAN_frame, false, frames_equal);
        check_m(frames_equal,
            "Lower priority buffer sent second although it has earlier timestamp!");

  end procedure;

end package body;