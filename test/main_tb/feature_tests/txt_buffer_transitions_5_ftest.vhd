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
--  TXT Buffer FSMs corner-case transitions
--
-- @Verifies:
--  @1. When SETTINGS[TBFBO] = 1, then TXT Buffers will move from Ready
--      TX Failed upon node moving to Bus-off.
--
-- @Test sequence:
--  @1. Enable SETTINGS[TBFBO] = 1 and Test Mode in DUT.
--  @2. Loop for all TXT Buffers. Current TXT buffer is X:
--      @2.1. Set priority of TXT Buffer X to 2 in DUT. Set priorities of other
--            TXT buffers to 1. Set TEC to 254 in DUT.
--      @2.2  Generate frame and insert it to all TXT Buffers in DUT.
--            Send Set Ready to all TXT Buffers in DUT. Wait until frame
--            transmission starts. Check in DUT that TXT Buffer X is in
--            TX in Progress, other TXT Buffers are in Ready.
--      @2.3  Wait until CRC field in DUT. Flip bus value. Wait until Error
--            frame. DUT should increment its TEC due to bit error or
--            stuff error. Check that DUT node is bus-off. Check that all
--            its TXT Buffers are in TX Failed.
--      @2.4  Disable DUT, re-enable it and wait until it integrates on the
--            bus again. Disable test node, not needed in this test.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--      5.11.2023   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.clk_gen_agent_pkg.all;

package txt_buffer_transitions_5_ftest is
    procedure txt_buffer_transitions_5_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body txt_buffer_transitions_5_ftest is

    procedure txt_buffer_transitions_5_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable mode_1               :       SW_mode := SW_mode_rst_val;

        variable CAN_frame            :       SW_CAN_frame_type;

        variable txt_buf_state        :       SW_TXT_Buffer_state_type;
        variable fault_state          :       SW_fault_state;

        variable num_txt_bufs         :       natural;
        variable err_counters         :       SW_error_counters;

        --variable command            :       SW_command := SW_command_rst_val;
        --variable status             :       SW_status;
	    --variable txt_buf_state	    :	    SW_TXT_Buffer_state_type;


        --variable frame_sent         :       boolean;

        --variable fault_state        :       SW_fault_state;
        --variable bus_val            :       std_logic;
    begin

        -------------------------------------------------------------------------------------------
        -- @1. Enable SETTINGS[TBFBO] = 1 and Test Mode in DUT.
        --     Set MODE[ACF] in Test Mode. Disable test node, not needed in this test.
        -------------------------------------------------------------------------------------------
        info_m("Step 1");

        mode_1.tx_buf_bus_off_failed := true;
        mode_1.test := true; -- Test mode must be enabled toallow manipulation of Error counters!

        set_core_mode(mode_1, DUT_NODE, chn);

        CAN_turn_controller(false, TEST_NODE, chn);

        -------------------------------------------------------------------------------------------
        --  @2. Loop for all TXT Buffers:
        -------------------------------------------------------------------------------------------
        info_m("Step 2");
        get_tx_buf_count(num_txt_bufs, DUT_NODE, chn);
        for txt_buf_index in 1 to num_txt_bufs loop

            ---------------------------------------------------------------------------------------
            -- @2.1. Set priority of TXT Buffer X to 1 in DUT. Set priorities of other
            --       TXT buffers to 2. Set TEC to 254 in DUT.
            ---------------------------------------------------------------------------------------
            info_m("Step 2.1");

            for i in 1 to num_txt_bufs loop
                if (i = txt_buf_index) then
                    CAN_configure_tx_priority(i, 2, DUT_NODE, chn);
                else
                    CAN_configure_tx_priority(i, 1, DUT_NODE, chn);
                end if;
            end loop;

            -- Preset the Error counter to just before bus-off
            err_counters.tx_counter := 254;
            err_counters.rx_counter := 0;
            set_error_counters(err_counters, DUT_NODE, chn);
            wait for 20 ns;

            ---------------------------------------------------------------------------------------
            -- @2.2 Generate frame and insert it to all TXT Buffers in DUT. Send Set Ready to all
            --      TXT Buffers in DUT. Wait until frame transmission starts. Check in DUT that
            --      TXT Buffer X is in TX in Progress, other TXT Buffers are in Ready.
            ---------------------------------------------------------------------------------------
            info_m("Step 2.2");

            CAN_generate_frame(CAN_frame);

            for i in 1 to num_txt_bufs loop
                CAN_insert_TX_frame(CAN_frame, i, DUT_NODE, chn);
            end loop;

            -- Send Set Ready to all TXT Buffers simultaneously
            send_TXT_buf_cmd(buf_set_ready, "11111111", DUT_NODE, chn);

            CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);

            for i in 1 to num_txt_bufs loop
                get_tx_buf_state(i, txt_buf_state, DUT_NODE, chn);

                if (i = txt_buf_index) then
                    check_m(txt_buf_state = buf_tx_progress,
                            "TXT Buffer " & integer'image(i) & " in TX in Progress!");
                else
                    check_m(txt_buf_state = buf_ready,
                            "TXT Buffer " & integer'image(i) & " in Ready");
                end if;
            end loop;

            ---------------------------------------------------------------------------------------
            -- @2.3 Wait until CRC field in DUT. Flip bus value. Wait until Error frame.
            --      DUT should increment its TEC due to bit error or stuff error.
            --      Check that DUT node is bus-off. Check that all its TXT Buffers are in TX Failed.
            ---------------------------------------------------------------------------------------
            info_m("Step 2.3");

            CAN_wait_pc_state(pc_deb_crc, DUT_NODE, chn);

            flip_bus_level(chn);

            CAN_wait_error_frame(DUT_NODE, chn);

            get_fault_state(fault_state, DUT_NODE, chn);

            check_m(fault_state = fc_bus_off, "DUT is bus-off");

            for i in 1 to num_txt_bufs loop
                get_tx_buf_state(i, txt_buf_state, DUT_NODE, chn);
                check_m(txt_buf_state = buf_failed,
                        "TXT Buffer " & integer'image(i) & " in TX Failed");
            end loop;

            wait for 2000 ns;
            release_bus_level(chn);
            wait for 10000 ns;

            ---------------------------------------------------------------------------------------
            -- @2.4  Disable DUT, re-enable it and wait until it integrates on the
            --       bus again.
            ---------------------------------------------------------------------------------------

            CAN_turn_controller(false, DUT_NODE, chn);
            CAN_turn_controller(true, DUT_NODE, chn);
            CAN_wait_bus_on(DUT_NODE, chn);

        end loop;

  end procedure;
end package body;
