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
--  TXT Buffer FSMs corner-case transitions 3
--
-- @Verifies:
--  @1. When Unlock with
--  @2.
--
-- @Test sequence:
--  @1. Loop for all TXT Buffers and incrementing wait times within a bit:
--      @2.1. Generate frame and send it from a TXT Buffer. Wait until it
--            starts being transmitted! Wait until dominant bit is being
--            transmitted. Now we are shortly after SYNC segment of dominant
--            transmitted bit.
--      @2.2. Force the bit to be recessive. Wait until some time before
--            Sample point. Wait small incremental delay. Send Set Abort Command.
--      @2.3. Wait until Sample point and release the bus value.
--      @2.4. Wait until bus is Idle.
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

package txt_buffer_transitions_3_ftest is
    procedure txt_buffer_transitions_3_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body txt_buffer_transitions_3_ftest is

    procedure txt_buffer_transitions_3_ftest_exec(
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
        variable can_tx_val         :       std_logic;
        variable bus_timing         :       bit_time_config_type;
        variable tseg1              :       natural;
    begin

        -------------------------------------------------------------------------------------------
        -- @1. Loop for all TXT Buffers and incrementing wait times within a bit:
        -------------------------------------------------------------------------------------------
        info_m("Step 2");
        get_tx_buf_count(num_txt_bufs, DUT_NODE, chn);

        -- Configure test mode to allow clearing error counters between iterations so that
        -- we dont get to bus off!
        mode.test := true;
        set_core_mode(mode, DUT_NODE, chn);

        -- Query the bus timing
        CAN_read_timing_v(bus_timing, DUT_NODE, chn);
        tseg1 := bus_timing.tq_nbt * (1 + bus_timing.prop_nbt + bus_timing.ph1_nbt);

        -- Generate single common frame
        CAN_generate_frame(CAN_frame);
        CAN_frame.frame_format := NORMAL_CAN;

        for txt_buf_index in 1 to num_txt_bufs loop
            for wait_cycles in 0 to 20 loop

                -----------------------------------------------------------------------------------
                -- @2.1. Generate frame and send it from a TXT Buffer. Wait until it starts being
                --       transmitted! Wait until dominant bit is being transmitted. Now we are
                --       shortly after SYNC segment of dominant transmitted bit.
                -----------------------------------------------------------------------------------
                info_m("Step 2.1 with wait cycles: " & integer'image(wait_cycles));

                CAN_insert_TX_frame(CAN_frame, txt_buf_index, DUT_NODE, chn);
                send_TXT_buf_cmd(buf_set_ready, txt_buf_index, DUT_NODE, chn);

                CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);
                wait for 1000 ns;

                while (true) loop
                    get_can_tx(DUT_NODE, can_tx_val, chn);
                    if (can_tx_val = RECESSIVE) then
                        exit;
                    end if;
                    wait for 10 ns;
                end loop;

                while (true) loop
                    get_can_tx(DUT_NODE, can_tx_val, chn);
                    if (can_tx_val = DOMINANT) then
                        exit;
                    end if;
                    wait for 10 ns;
                end loop;

                -----------------------------------------------------------------------------------
                -- @2.2. Force the bit to be recessive. Wait until some time before Sample point.
                --       Wait small incremental delay.
                -----------------------------------------------------------------------------------
                info_m("Step 2.2 with wait cycles: " & integer'image(wait_cycles));

                force_bus_level(RECESSIVE, chn);

                -- Wait till somewhere before Sample point
                for i in 1 to tseg1 - 10 loop
                    clk_agent_wait_cycle(chn);
                end loop;

                -- Wait incrementally and try hit the point where Protocol Engine unlocks the TXT
                -- Buffer due to an Error.
                for i in 0 to wait_cycles loop
                    clk_agent_wait_cycle(chn);
                end loop;

                send_TXT_buf_cmd(buf_set_abort, txt_buf_index, DUT_NODE, chn);

                -----------------------------------------------------------------------------------
                -- @2.3. Wait until Sample point and release the bus value.
                -----------------------------------------------------------------------------------
                info_m("Step 2.3 with wait cycles: " & integer'image(wait_cycles));

                CAN_wait_sample_point(DUT_NODE, chn);
                wait for 50 ns;

                release_bus_level(chn);

                -----------------------------------------------------------------------------------
                -- @2.4. Wait until bus is Idle and clear TX Error counter.
                -----------------------------------------------------------------------------------
                info_m("Step 2.4 with wait cycles: " & integer'image(wait_cycles));

                CAN_wait_bus_idle(DUT_NODE, chn);
                err_counters.tx_counter := 0;
                set_error_counters(err_counters, DUT_NODE, chn);

                wait for 100 ns;
            end loop;
        end loop;

  end procedure;
end package body;
