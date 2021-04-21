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
--  TX Priority change feature test.
--
-- @Verifies:
--  @1. When TX priority changes even if some TXT buffer is already selected in
--      TX ready state, and with new priority settings another TXT Buffer has
--      higher priority, this new TXT Buffer will be selected.
--
-- @Test sequence:
--  @1. Select random priorities for TXT buffers and write them to TX Priority
--      register. Generate random frames and store them to TXT Buffers.
--  @2. Wait till sample point (+ some time more) and issue TX ready command
--      to all TXT buffers. Read state of the TXT Buffers and check that they
--      are all in ready state.
--  @3. Select new random priorities and write them to TXT Buffers.
--  @4. Wait until frame is sent, and check that received frame was sent from
--      new highest priority TXT buffer. 
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--      27.11.2020  Add revision history
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package tx_priority_change_ftest is
    procedure tx_priority_change_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body tx_priority_change_ftest is

    procedure tx_priority_change_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        
        -- Assume maximal amount of TXT buffers
        type txt_buffer_priorities is array (1 to 8) of integer range 0 to 7;
        type CAN_frame_array_type is array (1 to 8) of SW_CAN_frame_type; 
        
        variable priorities : txt_buffer_priorities;
        variable txtb_state : SW_TXT_Buffer_state_type;

        variable CAN_frame_array_tx :       CAN_frame_array_type;        
        variable CAN_frame_rx       :       SW_CAN_frame_type;

        variable highest_prio_buf   :       integer := 4;
        variable highest_prio       :       integer := 0;
        variable frames_equal       :       boolean;

        variable ID_1               :       natural := 1;   -- Transmiter
        variable ID_2               :       natural := 2;   -- Receiver
        
        variable num_txt_bufs       :       natural;       
    begin

        -----------------------------------------------------------------------
        -- @1. Select random priorities for TXT buffers and write them to TX
        --     Priority register. Generate random frames and store them to
        --     TXT Buffers.
        -----------------------------------------------------------------------
        info_m("Step 1");
        
        get_tx_buf_count(num_txt_bufs, DUT_NODE, chn);

        for i in 1 to num_txt_bufs loop
            rand_int_v(7, priorities(i));
            -- Write generated priority TX_PRIORITY
            CAN_configure_tx_priority(i, priorities(i), DUT_NODE, chn);
        end loop;

        for i in 1 to num_txt_bufs loop
            CAN_generate_frame(CAN_frame_array_tx(i));
            CAN_insert_TX_frame(CAN_frame_array_tx(i), i, DUT_NODE, chn);
        end loop;

        -----------------------------------------------------------------------
        -- @2. Wait till sample point (+ some time more) and issue TX ready
        --     command to all TXT buffers. Read state of the TXT Buffers and
        --     check that they are all in ready state.
        -----------------------------------------------------------------------
        info_m("Step 2");

        CAN_wait_sample_point(DUT_NODE, chn, false);
        -- If there is less buffers than 8, other bits should be reserved and
        -- have no effect!
        send_TXT_buf_cmd(buf_set_ready, "11111111", DUT_NODE, chn);
        wait for 15 ns;
        
        for i in 1 to num_txt_bufs loop
            get_tx_buf_state(i, txtb_state, DUT_NODE, chn);
            check_m(txtb_state = buf_ready, "TXT Buffer ready!");
        end loop;
        
        -----------------------------------------------------------------------
        -- @3. Select new random priorities and write them to TXT Buffers.
        -----------------------------------------------------------------------
        info_m("Step 3");

        for i in 1 to num_txt_bufs loop
            rand_int_v(7, priorities(i));
            -- Write generated priority TX_PRIORITY
            CAN_configure_tx_priority(i, priorities(i), DUT_NODE, chn);
        end loop;
        
        -- Note: From CAN_Wait_sample_point in Step 2, till here, all this
        --       should fit before next sample point (at that point core locks
        --       buffer for transmission). If there are 8 buffers, reading
        --       their state takes 8 cycles (read 1 by 1). Therefore this test
        --       has lower limit of bit-rate it can run on!
        -- Note: There is potential optimisation in reading state of all TXT
        --       buffers at once and also configuring TXT buffer priorities
        --       at once! This is potential TODO!

        -----------------------------------------------------------------------
        -- @4. Wait until frame is sent, and check that received frame was sent
        --     from new highest priority TXT buffer. 
        -----------------------------------------------------------------------
        info_m("Step 4");

        CAN_wait_frame_sent(DUT_NODE, chn);

        -- Pick highest priority buffer
        highest_prio_buf := num_txt_bufs;
        for i in num_txt_bufs downto 1 loop
            if (priorities(i) >= highest_prio) then
                highest_prio := priorities(i);
                highest_prio_buf := i;
            end if;
        end loop;

        CAN_wait_sample_point(TEST_NODE, chn, false);
        wait for 100 ns;

        CAN_read_frame(CAN_frame_rx, TEST_NODE, chn);
        CAN_compare_frames(CAN_frame_rx, CAN_frame_array_tx(highest_prio_buf),
                            false, frames_equal);
        check_m(frames_equal, "TX/RX buffers matching!");

        CAN_wait_bus_idle(DUT_NODE, chn);

        -- Read the remaining received frames
        for i in 2 to num_txt_bufs loop
            CAN_read_frame(CAN_frame_rx, TEST_NODE, chn);
        end loop;

    end procedure;
end package body;
