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
--  @1. When SETTINGS[TBFBO] = 1, then TXT Buffers will move from Abort in
--      Progress to TX Failed.
--
-- @Test sequence:
--  @1. Enable SETTINGS[TBFBO] = 1.
--  @2. Loop for all TXT Buffers:
--      @2.1. Send a frame from a TXT Buffer. Wait until its transmission starts
--            and send Set Abort Command. Check TXT Buffer is in Abort In Progress.
--      @2.2. Invoke DUT Node to become bus-off. Check that the TXT Buffer became
--            TX Failed.
--      @2.3. Reintegrate the node on the bus. Issue "Set empty".
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

package txt_buffer_transitions_ftest is
    procedure txt_buffer_transitions_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body txt_buffer_transitions_ftest is

    procedure txt_buffer_transitions_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :       SW_CAN_frame_type;
        variable command            :       SW_command := SW_command_rst_val;
        variable status             :       SW_status;
	    variable txt_buf_state	    :	    SW_TXT_Buffer_state_type;
        variable mode               :       SW_mode;
        variable num_txt_bufs       :       natural;
        variable frame_sent         :       boolean;
        variable err_counters       :       SW_error_counters;
        variable fault_state        :       SW_fault_state;
        variable bus_val            :       std_logic;
    begin

        -------------------------------------------------------------------------------------------
        -- @1. Enable SETTINGS[TBFBO] = 1.
        -------------------------------------------------------------------------------------------
        info_m("Step 1");

        mode.tx_buf_bus_off_failed := true;
        mode.test := true; -- Test mode must be enabled toallow manipulation of Error counters!
        set_core_mode(mode, DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        --  @2. Loop for all TXT Buffers:
        -------------------------------------------------------------------------------------------
        info_m("Step 2");
        get_tx_buf_count(num_txt_bufs, DUT_NODE, chn);
        for txt_buf_index in 1 to num_txt_bufs loop

            ---------------------------------------------------------------------------------------
            -- @2.1. Send a frame from a TXT Buffer. Wait until its transmission starts
            --       and send Set Abort Command. Check TXT Buffer is in Abort In Progress.
            ---------------------------------------------------------------------------------------
            info_m("Step 2.1");

            -- Preset the Error counter to just before bus-off
            err_counters.tx_counter := 254;
            err_counters.rx_counter := 0;
            set_error_counters(err_counters, DUT_NODE, chn);
            wait for 20 ns;

            -- Send frame
            CAN_generate_frame(CAN_frame);
            CAN_send_frame(CAN_frame, txt_buf_index, DUT_NODE, chn, frame_sent);
            CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);

            -- Send abort
            send_TXT_buf_cmd(buf_set_abort, txt_buf_index, DUT_NODE, chn);
            wait for 30 ns;
            get_tx_buf_state(txt_buf_index, txt_buf_state, DUT_NODE, chn);
            check_m(txt_buf_state = buf_ab_progress,
                    "TXT Buffer " & integer'image(txt_buf_index) & " abort in progress");

            -------------------------------------------------------------------------------------------
            -- @2.2. Invoke DUT Node to become bus-off. Check that the TXT Buffer became TX Failed.
            -------------------------------------------------------------------------------------------
            info_m("Step 2");

            -- Wait until dominant bit sent, and force it Recessive
            while true loop
                get_can_tx(DUT_NODE, bus_val, chn);
                if (bus_val = DOMINANT) then
                    exit;
                end if;
                wait for 10 ns;
            end loop;

            force_bus_level(RECESSIVE, chn);
            wait for 2000 ns;
            release_bus_level(chn);

            -- Check we are surely in Bus off
            get_fault_state(fault_state, DUT_NODE, chn);
            check_m(fault_state = fc_bus_off, "DUT is bus-off");

            -- Check TXT Buffer is in TX Failed.
            get_tx_buf_state(txt_buf_index, txt_buf_state, DUT_NODE, chn);
            check_m(txt_buf_state = buf_failed,
                    "TXT Buffer " & integer'image(txt_buf_index) & " TX failed");

            ---------------------------------------------------------------------------------------
            -- @3. Reintegrate the node on the bus. Issue "Set empty".
            ---------------------------------------------------------------------------------------
            info_m("Step 3");

            command.err_ctrs_rst := true;
            give_controller_command(command, DUT_NODE, chn);

            -- Wait until reintegration is over. Add some margin since DUT just started sending
            -- Error frame!
            for i in 1 to 135 loop
                for j in 1 to 11 loop
                    CAN_wait_sample_point(DUT_NODE, chn, false);
                end loop;
            end loop;
            info_m("Reintegration done...");

            -- Move TXT Buffer back to Empty
            send_TXT_buf_cmd(buf_set_empty, txt_buf_index, DUT_NODE, chn);
            wait for 30 ns;
            get_tx_buf_state(txt_buf_index, txt_buf_state, DUT_NODE, chn);
            check_m(txt_buf_state = buf_empty,
                    "TXT Buffer " & integer'image(txt_buf_index) & " empty");

            -- Check we are back to error active
            get_fault_state(fault_state, DUT_NODE, chn);
            check_m(fault_state = fc_error_active, "DUT is Error active again");

        end loop;

  end procedure;
end package body;
