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
--  TXT Buffer FSMs corner-case transitions 4
--
-- @Verifies:
--  @1. When TXT Buffer is in Abortin Progress, and parity error occurs, it
--      moves to Parity Error state
--
-- @Test sequence:
--  @1. Loop for all TXT Buffers:
--      @1.1. Insert a CAN frame into to a TXT Buffer. Insert a bit-flip
--            into a data word inside TXT Buffer.
--      @1.2. Send Set Ready Command. Wait until DUT starts transmitting the
--            frame, and then send Set Abort Command.
--      @1.3. Wait until bus is idle. Check TXT Buffer ended in Parity Error.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--      12.11.2023   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.clk_gen_agent_pkg.all;

package txt_buffer_transitions_4_ftest is
    procedure txt_buffer_transitions_4_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body txt_buffer_transitions_4_ftest is

    procedure txt_buffer_transitions_4_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :       SW_CAN_frame_type;
        variable command            :       SW_command := SW_command_rst_val;
        variable status             :       SW_status;
	    variable txt_buf_state	    :	    SW_TXT_Buffer_state_type;
        variable mode               :       SW_mode := SW_mode_rst_val;
        variable num_txt_bufs       :       natural;
        variable frame_sent         :       boolean;
        variable err_counters       :       SW_error_counters;
        variable fault_state        :       SW_fault_state;
        variable can_tx_val         :       std_logic;
        variable bus_timing         :       bit_time_config_type;
        variable tseg1              :       natural;

        variable tst_mem            :       t_tgt_test_mem;
        variable corrupt_wrd_index  :       natural;
        variable corrupt_bit_index  :       natural;
        variable r_data             :       std_logic_vector(31 downto 0);
    begin

        -------------------------------------------------------------------------------------------
        -- @1. Loop for all TXT Buffers and incrementing wait times within a bit:
        -------------------------------------------------------------------------------------------
        info_m("Step 1");
        get_tx_buf_count(num_txt_bufs, DUT_NODE, chn);

        -- Configure test mode to allow bit-flips via test channel in TXT Buffer Memory.
        mode.test := true;
        mode.parity_check := true;
        set_core_mode(mode, DUT_NODE, chn);

        -- Generate single common frame
        CAN_generate_frame(CAN_frame);
        CAN_frame.frame_format := FD_CAN;
        CAN_frame.data_length := 16;
        CAN_frame.rtr := NO_RTR_FRAME;
        decode_length(CAN_frame.data_length, CAN_frame.dlc);

        for txt_buf_index in 1 to num_txt_bufs loop

            -----------------------------------------------------------------------------------
            -- @1.1. Insert a frame into a TXT Buffer. Insert a bit-flip into a data word
            --       inside TXT Buffer.
            -----------------------------------------------------------------------------------
            info_m("Step 1.1");

            CAN_insert_TX_frame(CAN_frame, txt_buf_index, DUT_NODE, chn);

            set_test_mem_access(true, DUT_NODE, chn);

            tst_mem := txt_buf_to_test_mem_tgt(txt_buf_index);

            -- Read, flip, and write back
            corrupt_wrd_index := 5;             -- Flip bit in data bytes 5-8
            rand_int_v(31, corrupt_bit_index);
            test_mem_read(r_data, corrupt_wrd_index, tst_mem, DUT_NODE, chn);
            r_data(corrupt_bit_index) := not r_data(corrupt_bit_index);
            test_mem_write(r_data, corrupt_wrd_index, tst_mem, DUT_NODE, chn);

            -- Disable test mem access
            set_test_mem_access(false, DUT_NODE, chn);

            ---------------------------------------------------------------------------------------
            -- @1.2. Send Set Ready Command. Wait until DUT starts transmitting the frame, and then
            --       send Set Abort Command.
            ---------------------------------------------------------------------------------------
            info_m("Step 1.2");

            send_TXT_buf_cmd(buf_set_ready, txt_buf_index, DUT_NODE, chn);
            CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);
            wait for 1000 ns;

            send_TXT_buf_cmd(buf_set_abort, txt_buf_index, DUT_NODE, chn);

            ---------------------------------------------------------------------------------------
            -- @1.3. Wait until bus is idle. Check TXT Buffer ended in Parity Error.
            ---------------------------------------------------------------------------------------
            info_m("Step 1.3");

            CAN_wait_bus_idle(DUT_NODE, chn);
            CAN_wait_bus_idle(TEST_NODE, chn);

            get_tx_buf_state(txt_buf_index, txt_buf_state, DUT_NODE, chn);

            check_m(txt_buf_state = buf_parity_err, "TXT Buffer in parity error state");

        end loop;

  end procedure;
end package body;
