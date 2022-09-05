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
--  STATUS[TXPE] feature test.
--
-- @Verifies:
--  @1. STATUS[TXPE] is set when there is a parity error detected in TXT Buffer
--      RAM and SETTINGS[PCHKE] = 1.
--  @2. STATUS[TXPE] is not set when there is a parity error detected in TXT
--      Buffer and SETTINGS[PCHKE] = 0.
--  @2. STATUS[TXPE] is cleared by COMMAND[CTXPE].
--
-- @Test sequence:
--  @1. Set DUT to test mode.
--  @2. Loop 4 times:
--      @2.1 Generate Random CAN frame and set random value of SETTINGS[PCHKE] = 1.
--      @2.2 Insert the CAN frame for transmission into random TXT Buffer.
--      @2.3 Generate random bit-flip in FRAME_FORMAT_W (loop 1), IDENTIFIER_W
--           (loop 2), TIMESTAMP_L_W (loop 3), TIMESTAMP_U_W (loop 4) and
--           corrupt this word in the TXT Buffer memory via test interface.
--      @2.4 Send Set Ready command to this TXT Buffer. Wait for some time, and
--           check that when SETTINGS[PCHKE] = 1, TXT Buffer ended up in
--           "Parity Error" state. When SETTINGS[PCHKE] = 0, check that DUT
--           transmit the frame from TXT Buffer.
--      @2.5 Read STATUS[TXPE] and check it is 1 when SETTINGS[PCHKE]=1. Write
--           COMMAND[CTXPE], and check that STATUS[TXPE] = 0.
--      @2.6 Insert the frame again, send Set Ready command.
--      @2.7 Wait until frame is transmitted. Read frame from Test Node and
--           check it is equal to transmitted frame.
--  @3. Loop 10 times:
--      @3.1 Generate random CAN frame and make sure it has some data words.
--           Insert it into random TXT Buffer. Set random value of
--           SETTINGS[PCHKE] = 1.
--      @3.2 Flip random bit within the data word, and write this flipped bit
--           into TXT Buffer via test access to bypass parity encoding.
--      @3.3 Send Set Ready command to TXT Buffer, wait until frame
--           transmission starts.
--      @3.4 When SETTINGS[PCHKE]=1, wait until error frame, check its
--           transmission from status bit, then check that TXT Buffer ended up
--           in Parity Error state. When SETTINGS[PCHKE]=0, check that frame
--           is transmitted sucesfully.
--      @3.5 Insert CAN frame into the TXT Buffer again.
--      @3.6 Send Set Ready command. Wait until CAN frame is transmitted. Read
--           it from Test Node and check that it matches original CAN frame.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    15.6.2022   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.mem_bus_agent_pkg.all;

package status_txpe_ftest is
    procedure status_txpe_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body status_txpe_ftest is
    procedure status_txpe_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;
        variable frame_2            :     SW_CAN_frame_type;

        variable stat_1             :     SW_status;
        variable command_1          :     SW_command := SW_command_rst_val;
        variable mode_1             :     SW_mode := SW_mode_rst_val;
        variable rx_buf_status      :     SW_RX_Buffer_info;

        variable pc_dbg             :     SW_PC_Debug;
        variable frame_sent         :     boolean;
        variable frames_equal       :     boolean;

        variable rptr_pos           :     integer := 0;
        variable r_data             :     std_logic_vector(31 downto 0);
        type t_raw_can_frame is
            array (0 to 19) of std_logic_vector(31 downto 0);
        variable rx_frame_buffer    :     t_raw_can_frame;
        variable rwcnt              :     integer;
        variable corrupt_wrd_index  :     integer;
        variable corrupt_bit_index  :     integer;
        variable corrupt_insert     :     std_logic;

        variable txt_buf            :     SW_TXT_index_type;
        variable tst_mem            :     t_tgt_test_mem;
        variable txt_buf_state      :     SW_TXT_Buffer_state_type;
        variable prt_en             :     std_logic;
    begin

        -----------------------------------------------------------------------
        -- @1. Set DUT to test mode.
        -----------------------------------------------------------------------
        info_m("Step 1");
        mode_1.test := true;
        set_core_mode(mode_1, DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @2. Loop 4 times
        -----------------------------------------------------------------------
        info_m("Step 2");
        for i in 0 to 3 loop

            -------------------------------------------------------------------
            -- @2.1 Generate Random CAN frame and set random value of
            --      SETTINGS[PCHKE] = 1.
            -------------------------------------------------------------------
            info_m("Step 2.1");
            CAN_generate_frame(frame_1);
            rand_logic_v(prt_en, 0.5);
            if (prt_en = '1') then
                mode_1.parity_check := true;
            else
                mode_1.parity_check := false;
            end if;
            set_core_mode(mode_1, DUT_NODE, chn);

            -------------------------------------------------------------------
            -- @2.2 Insert the CAN frame for transmission into random TXT Buffer.
            -------------------------------------------------------------------
            info_m("Step 2.2");
            pick_random_txt_buffer(txt_buf, DUT_NODE, chn);
            CAN_insert_TX_frame(frame_1, txt_buf, DUT_NODE, chn);
            
            -------------------------------------------------------------------
            -- @2.3 Generate random bit-flip in FRAME_FORMAT_W (loop 1),
            --      IDENTIFIER_W (loop 2), TIMESTAMP_L_W (loop 3), TIMESTAMP_U_W
            --      (loop 4) and corrupt this word in the TXT Buffer memory via
            --      test interface.
            -------------------------------------------------------------------
            info_m("Step 2.3");

            -- Enable test access
            set_test_mem_access(true, DUT_NODE, chn);
            tst_mem := txt_buf_to_test_mem_tgt(txt_buf);

            -- Read, flip, and write back
            test_mem_read(r_data, i, tst_mem, DUT_NODE, chn);
            rand_int_v(31, corrupt_bit_index);
            r_data(corrupt_bit_index) := not r_data(corrupt_bit_index);
            test_mem_write(r_data, i, tst_mem, DUT_NODE, chn);

            -- Disable test mem access
            set_test_mem_access(false, DUT_NODE, chn);

            -------------------------------------------------------------------
            -- @2.4 Send Set Ready command to this TXT Buffer. Wait for some
            -- time, and check that when SETTINGS[PCHKE] = 1, TXT Buffer ended
            -- up in "Parity Error" state. When SETTINGS[PCHKE] = 0, check that
            -- DUT transmit the frame from TXT Buffer.
            -------------------------------------------------------------------
            info_m("Step 2.4");

            send_TXT_buf_cmd(buf_set_ready, txt_buf, DUT_NODE, chn);
            wait for 5 us;
            
            get_tx_buf_state(txt_buf, txt_buf_state, DUT_NODE, chn);

            -- TXT Buffer shall end up in parity error only when parity error
            -- detection is enabled, otherwise DUT will go on and transmit the
            -- frame. Received frame might not be correct though, since it
            -- contained bit flip!
            if (mode_1.parity_check) then
                check_m(txt_buf_state = buf_parity_err,
                        "Bit flip + SETTINGS[PCHKE] = 1 -> TXT Buffer in parity error state!");
            else
                check_m(txt_buf_state = buf_tx_progress,
                         "Bit flip + SETTINGS[PCHKE] = 0 -> TXT Buffer in TX in progress state!");
                CAN_wait_bus_idle(DUT_NODE, chn);
                CAN_wait_bus_idle(TEST_NODE, chn);
                get_tx_buf_state(txt_buf, txt_buf_state, DUT_NODE, chn);
                check_m(txt_buf_state = buf_done, "DUT: TXT Buffer TX OK");
                
                get_rx_buf_state(rx_buf_status, TEST_NODE, chn);
                check_m(rx_buf_status.rx_frame_count = 1,
                        "TEST_NODE: Frame received!");

                -- Need to read-out received frame so that it does not block
                -- RX Buffer FIFO in Test node.
                CAN_read_frame(frame_2, TEST_NODE, chn);
            end if;

            get_controller_status(stat_1, DUT_NODE, chn);
            check_false_m(stat_1.transmitter, "DUT is not transmitter.");

            -------------------------------------------------------------------
            -- @2.5 Read STATUS[TXPE] and check it is 1 when SETTINGS[PCHKE]=1.
            --      Write COMMAND[CTXPE], and check that STATUS[TXPE] = 0.
            -------------------------------------------------------------------
            info_m("Step 2.5");

            get_controller_status(stat_1, DUT_NODE, chn);

            if (mode_1.parity_check) then
                check_m(stat_1.tx_parity_error,
                        "Bit flip + SETTINGS[PCHKE] = 1 -> STATUS[TXPE] = 1");
            else
                check_false_m(stat_1.tx_parity_error,
                        "Bit flip + SETTINGS[PCHKE] = 0 -> STATUS[TXPE] = 0");
            end if;

            command_1.clear_txpe := true;
            give_controller_command(command_1, DUT_NODE, chn);
            
            get_controller_status(stat_1, DUT_NODE, chn);
            check_false_m(stat_1.tx_parity_error, "STATUS[TXPE] = 0");

            -------------------------------------------------------------------
            -- @2.6 Insert the frame again, send Set Ready command.
            -------------------------------------------------------------------
            info_m("Step 2.6");

            CAN_insert_TX_frame(frame_1, txt_buf, DUT_NODE, chn);
            send_TXT_buf_cmd(buf_set_ready, txt_buf, DUT_NODE, chn);

            -------------------------------------------------------------------
            -- @2.7 Wait until frame is transmitted. Read frame from Test Node
            --      and check it is equal to transmitted frame.
            -------------------------------------------------------------------
            info_m("Step 2.7");

            CAN_wait_frame_sent(DUT_NODE, chn);
            CAN_read_frame(frame_2, TEST_NODE, chn);
            
            CAN_compare_frames(frame_1, frame_2, false, frames_equal);
            check_m(frames_equal, "Frames are equal.");

        end loop;

    ---------------------------------------------------------------------------
    --  @3. Loop 10 times:
    ---------------------------------------------------------------------------
    info_m("Step 3");
    for i in 0 to 9 loop

        -----------------------------------------------------------------------
        -- @3.1 Generate random CAN frame and make sure it has some data words.
        --      Insert it into random TXT Buffer. Set random value of
        --      SETTINGS[PCHKE] = 1.
        -----------------------------------------------------------------------
        info_m("Step 3.1");

        pick_random_txt_buffer(txt_buf, DUT_NODE, chn);

        rand_logic_v(prt_en, 0.5);
        if (prt_en = '1') then
            mode_1.parity_check := true;
        else
            mode_1.parity_check := false;
        end if;
        set_core_mode(mode_1, DUT_NODE, chn);

        CAN_generate_frame(frame_1);
        frame_1.rtr := NO_RTR_FRAME;
        if frame_1.data_length = 0 then
            frame_1.data_length := 1;
        end if;
        decode_length(frame_1.data_length, frame_1.dlc);
        decode_dlc_rx_buff(frame_1.dlc, frame_1.rwcnt);

        CAN_insert_TX_frame(frame_1, txt_buf, DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @3.2 Flip random bit within the data word, and write this flipped 
        --      bit into TXT Buffer via test access to bypass parity encoding.
        -----------------------------------------------------------------------
        info_m("Step 3.2");

        -- Generate random word / bit index to flip
        rand_int_v(31, corrupt_bit_index);
        decode_dlc_rx_buff(frame_1.dlc, rwcnt);
        rand_int_v(rwcnt - 4, corrupt_wrd_index);
        corrupt_wrd_index := corrupt_wrd_index + 4;
        info_m("Flipping word index: " & integer'image(corrupt_wrd_index));
        info_m("Flipping bit  index: " & integer'image(corrupt_bit_index));

        -- Enable test access
        set_test_mem_access(true, DUT_NODE, chn);
        tst_mem := txt_buf_to_test_mem_tgt(txt_buf);

        -- Read, flip, and write back
        test_mem_read(r_data, corrupt_wrd_index, tst_mem, DUT_NODE, chn);
        r_data(corrupt_bit_index) := not r_data(corrupt_bit_index);
        test_mem_write(r_data, corrupt_wrd_index, tst_mem, DUT_NODE, chn);

        -- Disable test mem access
        set_test_mem_access(false, DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @3.3 Send Set Ready command to TXT Buffer, wait until frame
        --      transmission starts.
        -----------------------------------------------------------------------
        info_m("Step 3.3");

        send_TXT_buf_cmd(buf_set_ready, txt_buf, DUT_NODE, chn);

        CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @3.4 Wait until error frame, check it is being transmitted. Check
        --      that TXT Buffer ended up in Parity Error state.  When
        --      SETTINGS[PCHKE]=0, check that frame is transmitted sucesfully.
        -----------------------------------------------------------------------
        info_m("Step 3.4");

        -- Note: Despite the fact that we flipped bit of data field, we can't
        --       wait till data field. If we flipped bit in first data word, it
        --       will be read out during control field, and thus protocol control
        --       will never get to data field!
        if (mode_1.parity_check) then
            CAN_wait_error_frame(DUT_NODE, chn);
            get_controller_status(stat_1, DUT_NODE, chn);

            check_m(stat_1.error_transmission, "Error frame is being transmitted");

            get_tx_buf_state(txt_buf, txt_buf_state, DUT_NODE, chn);
            check_m(txt_buf_state = buf_parity_err,
                     "Bit flip + SETTINGS[PCHKE] = 1 -> TXT Buffer in parity error state!");
        else
            CAN_wait_frame_sent(DUT_NODE, chn);
            CAN_wait_bus_idle(DUT_NODE, chn);

            get_tx_buf_state(txt_buf, txt_buf_state, DUT_NODE, chn);
            check_m(txt_buf_state = buf_done,
                    "Bit flip + SETTINGS[PCHKE] = 0 -> TXT Buffer in TX OK state!");
            
            get_rx_buf_state(rx_buf_status, TEST_NODE, chn);
            check_m(rx_buf_status.rx_frame_count = 1,
                    "TEST_NODE: Frame received!");

            -- Need to read-out received frame so that it does not block
            -- RX Buffer FIFO in Test node.
            CAN_read_frame(frame_2, TEST_NODE, chn);
        end if;

        -- Check that parity flag was set, clear it!
        get_controller_status(stat_1, DUT_NODE, chn);
        if (mode_1.parity_check) then
            check_m(stat_1.tx_parity_error,
                    "Bit flip + SETTINGS[PCHKE] = 1 -> STATUS[TXPE] = 1");
        else
            check_false_m(stat_1.tx_parity_error,
                    "Bit flip + SETTINGS[PCHKE] = 0 -> STATUS[TXPE] = 0");
        end if;
        
        command_1.clear_txpe := true;
        give_controller_command(command_1, DUT_NODE, chn);
        
        get_controller_status(stat_1, DUT_NODE, chn);
        check_false_m(stat_1.tx_parity_error, "STATUS[TXPE] = 0");

        ---------------------------------------------------------------------------
        -- @3.5 Insert CAN frame into the TXT Buffer again.
        ---------------------------------------------------------------------------
        info_m("Step 3.5");

        CAN_insert_TX_frame(frame_1, txt_buf, DUT_NODE, chn);

        ---------------------------------------------------------------------------
        -- @3.6 Send Set Ready command. Wait until CAN frame is transmitted. Read
        --      it from Test Node and check that it matches original CAN frame.
        ---------------------------------------------------------------------------
        info_m("Step 3.6");

        send_TXT_buf_cmd(buf_set_ready, txt_buf, DUT_NODE, chn);

        CAN_wait_frame_sent(DUT_NODE, chn);
        CAN_read_frame(frame_2, TEST_NODE, chn);
        
        CAN_compare_frames(frame_1, frame_2, false, frames_equal);
        check_m(frames_equal, "Frames are equal.");
    
    end loop;

  end procedure;

end package body;
