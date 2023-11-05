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
--  TXT Buffer FSMs corner-case transitions 2
--
-- @Verifies:
--  @1. When Lock is issued by PC FSM on a TXT Buffer simultaneously as Set
--      Abort, TXT Buffer will move immediately to Abort in Progress.
--  @2. When Transmission ends (PC FSM attempts to unlock TXT Buffer) at
--      the same time as User issues Set Empty, command, Set empty will be
--      applied.
--
-- @Test sequence:
--  @1. Loop for all TXT Buffers and incrementing wait times within a bit:
--      @2.1. Generate frame and insert it into TXT Buffer. Wait until Sync
--            segment, and send Set Ready Command.
--      @2.2. Wait until bit phase small amount of time before sample point
--            + per-loop incremental delay. Issue Set Abort Command.
--      @2.3. Wait for some time. Now either TXT Buffer must have been aborted,
--            or the transmission has started. If the transmission was aborted,
--            then finish the loop.
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

package txt_buffer_transitions_2_ftest is
    procedure txt_buffer_transitions_2_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body txt_buffer_transitions_2_ftest is

    procedure txt_buffer_transitions_2_ftest_exec(
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
        variable bus_timing         :       bit_time_config_type;
        variable tseg1              :       natural;
    begin

        -------------------------------------------------------------------------------------------
        -- @1. Loop for all TXT Buffers and incrementing wait times within a bit:
        -------------------------------------------------------------------------------------------
        info_m("Step 2");
        get_tx_buf_count(num_txt_bufs, DUT_NODE, chn);

        -- Query the bus timing
        CAN_read_timing_v(bus_timing, DUT_NODE, chn);
        tseg1 := bus_timing.tq_nbt * (1 + bus_timing.prop_nbt + bus_timing.ph1_nbt);

        for txt_buf_index in 1 to num_txt_bufs loop

            for wait_cycles in 0 to 20 loop

                -----------------------------------------------------------------------------------
                -- @2.1. Generate frame and insert it into TXT Buffer. Wait until Sync
                --       segment, and send Set Ready Command.
                -----------------------------------------------------------------------------------
                info_m("Step 2.1 with wait cycles: " & integer'image(wait_cycles));

                CAN_generate_frame(CAN_frame);
                CAN_insert_TX_frame(CAN_frame, txt_buf_index, DUT_NODE, chn);

                CAN_wait_sync_seg(DUT_NODE, chn);

                send_TXT_buf_cmd(buf_set_ready, txt_buf_index, DUT_NODE, chn);

                -----------------------------------------------------------------------------------
                -- @2.2. Wait until bit phase small amount of time before sample point + per-loop
                --       incremental delay. Issue Set Abort Command.
                -----------------------------------------------------------------------------------
                info_m("Step 2.2 with wait cycles: " & integer'image(wait_cycles));

                -- Wait till somewhere before Sample point
                for i in 1 to tseg1 - 10 loop
                    clk_agent_wait_cycle(chn);
                end loop;

                -- Wait incrementally and try hit the point where Protocol Engine locks the TXT
                -- Buffer.
                for i in 0 to wait_cycles loop
                    clk_agent_wait_cycle(chn);
                end loop;

                send_TXT_buf_cmd(buf_set_abort, txt_buf_index, DUT_NODE, chn);

                -----------------------------------------------------------------------------------
                -- @2.3. Wait for some time. Now either TXT Buffer must have been aborted, or the
                --       transmission has started. If the transmission was aborted, then finish
                --       the loop.
                -----------------------------------------------------------------------------------
                info_m("Step 2.3 with wait cycles: " & integer'image(wait_cycles));

                wait for 1000 ns;

                get_tx_buf_state(txt_buf_index, txt_buf_state, DUT_NODE, chn);
                check_m(txt_buf_state = buf_ab_progress or txt_buf_state = buf_aborted,
                        "TXT Buffer " & integer'image(txt_buf_index) & " aborted or abort in progress");

                if (txt_buf_state = buf_aborted) then
                    info_m("TXT Buffer aborted, not continuing with second part of the test!");
                    next;
                end if;

                -- The first cycle where we are in abort in progress is where we have hit
                -- simultaneous lock and Set Abort
                info_m("TXT Buffer abort in Progress!");
                CAN_wait_bus_idle(DUT_NODE, chn);

                exit;

            end loop;
        end loop;

  end procedure;
end package body;
