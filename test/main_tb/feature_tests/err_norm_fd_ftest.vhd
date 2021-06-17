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
--  Jiri Novak <jnovak@fel.cvut.cz>
--  Pavel Pisa <pisa@cmp.felk.cvut.cz>
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
--  ERR_NORM (Nominal bit-rate) error counter feature test.
--
-- @Verifies:
--  @1. ERR_NORM is incremented by 1 when error occurs for trasmitter/receiver
--      during nominal bit-rate.
--  @2. ERR_FD is not incremented by 1 when error occurs for transmitter/receiver
--      during nominal bit-rate.
--  @3. ERR_NORM is not incremented by 1 when error occurs for trasmitter/receiver
--      during data bit-rate.
--  @4. ERR_FD is incremented by 1 when error occurs for trasmitter/receiver
--      during data bit-rate.
--
-- @Test sequence:
--  @1. Generate random frame where bit rate is not switched. Insert the frame
--      to DUT. Wait until DUT starts transmission. Wait for random time
--      until DUT transmits Dominant bit. Force the bus-level Recessive for one
--      bit time! This should invoke bit error in DUT. Wait until bus is idle.
--      Check that ERR_NORM in DUT and 2 incremented by 1. Check that ERR_FD
--      in DUT and Test Node remained the same!
--  @2. Generate random frame where bit rate shall be switched. Wait until data
--      portion of that frame. Wait until Recessive bit is transmitted. Force
--      bus Dominant for 1 bit time! Wait until bus is idle. Check that ERR_FD
--      incremented in DUT and Test node by 1. Check that ERR_NORM remained the
--      same in DUT and Test node.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    16.11.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.clk_gen_agent_pkg.all;

package err_norm_fd_ftest is
    procedure err_norm_fd_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body err_norm_fd_ftest is
    procedure err_norm_fd_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;

        variable err_counters_1_1   :     SW_error_counters;
        variable err_counters_1_2   :     SW_error_counters;

        variable err_counters_2_1   :     SW_error_counters;
        variable err_counters_2_2   :     SW_error_counters;
        
        variable frame_sent         :     boolean;
        variable rand_val           :     integer;
        variable can_tx_val         :     std_logic;
        variable wait_time          :     natural;
    begin

        -----------------------------------------------------------------------
        -- @1. Generate random frame where bit rate is not switched. Insert the
        --     frame to DUT. Wait until DUT starts transmission. Wait for
        --     random time until DUT transmits Dominant bit. Force the bus-
        --     level Recessive for one bit time! This should invoke bit error in
        --     DUT. Wait until bus is idle. Check that ERR_NORM in DUT and
        --     2 incremented by 1. Check that ERR_FD in DUT and Test Node remained
        --     the same!
        -----------------------------------------------------------------------
        info_m("Step 1");

        CAN_enable_retr_limit(true, 0, DUT_NODE, chn);

        read_error_counters(err_counters_1_1, DUT_NODE, chn);
        read_error_counters(err_counters_1_2, TEST_NODE, chn);
        
        CAN_generate_frame(frame_1);
        if (frame_1.frame_format = FD_CAN) then
            frame_1.brs := BR_NO_SHIFT;
        end if;
        CAN_send_frame(frame_1, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);

        -- Should be enough to cover many parts of CAN frame but not go beyond
        -- frame!
        rand_int_v(20, wait_time);
        -- We need at least 1 bit to wait, otherwise we try to corrupt still in SOF!
        wait_time := wait_time + 1;
        info_m("Waiting for:" & integer'image(wait_time) & " bits!");
        
        for i in 1 to wait_time loop
            CAN_wait_sync_seg(DUT_NODE, chn);
            info_m("Wait sync");
            wait for 20 ns;
        end loop;
        info_m("Waiting finished!");
        
        get_can_tx(DUT_NODE, can_tx_val, chn);
        while (can_tx_val = RECESSIVE) loop
            wait for 10 ns;
            get_can_tx(DUT_NODE, can_tx_val, chn);
        end loop;
        
        force_bus_level(RECESSIVE, chn);
        CAN_wait_sample_point(DUT_NODE, chn, false);
        wait for 20 ns; -- To be sure that opposite bit is sampled!
        release_bus_level(chn);

        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);
        read_error_counters(err_counters_2_1, DUT_NODE, chn);
        read_error_counters(err_counters_2_2, TEST_NODE, chn);
        
        check_m(err_counters_1_1.err_norm + 1 = err_counters_2_1.err_norm,
                "ERR_NORM incremented by 1 in transmitter!");
        check_m(err_counters_1_2.err_norm + 1 = err_counters_2_2.err_norm,
                "ERR_NORM incremented by 1 in receiver!");

        check_m(err_counters_1_1.err_fd = err_counters_2_1.err_fd,
                "ERR_FD not incremented by 1 in transmitter!");
        check_m(err_counters_1_2.err_fd = err_counters_2_2.err_fd,
                "ERR_FD not incremented by 1 in receiver!");

        -----------------------------------------------------------------------
        -- @2. Generate random frame where bit rate shall be switched. Wait
        --     until data portion of that frame. Wait until Recessive bit is
        --     transmitted. Force bus Dominant for 1 bit time! Wait until bus is
        --     idle. Check that ERR_FD incremented in DUT and Test node by 1.
        --     Check that ERR_NORM remained the same in DUT and Test node.
        -----------------------------------------------------------------------
        info_m("Step 2");

        read_error_counters(err_counters_1_1, DUT_NODE, chn);
        read_error_counters(err_counters_1_2, TEST_NODE, chn);

        CAN_generate_frame(frame_1);
        frame_1.frame_format := FD_CAN;
        frame_1.brs := BR_SHIFT;
        
        CAN_send_frame(frame_1, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);

        CAN_wait_pc_state(pc_deb_control, DUT_NODE, chn);
        CAN_wait_not_pc_state(pc_deb_control, DUT_NODE, chn);
        
        -- Now we should be in Data bit rate! This is either CRC or data field!
        rand_int_v(15, wait_time);
        -- We need at least 1 bit to wait, otherwise we try to corrupt still in SOF!
        wait_time := wait_time + 1;
        info_m("Waiting for:" & integer'image(wait_time) & " bits!");
        
        for i in 1 to wait_time loop
            CAN_wait_sync_seg(DUT_NODE, chn);
            info_m("Wait sync");
            wait for 20 ns;
        end loop;
        info_m("Waiting finished!");

        get_can_tx(DUT_NODE, can_tx_val, chn);
        while (can_tx_val = DOMINANT) loop
            wait for 10 ns;
            get_can_tx(DUT_NODE, can_tx_val, chn);
        end loop;
        
        force_bus_level(DOMINANT, chn);
        CAN_wait_sample_point(DUT_NODE, chn);
        CAN_wait_sample_point(DUT_NODE, chn);
        wait for 20 ns;
        release_bus_level(chn);

        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);
        read_error_counters(err_counters_2_1, DUT_NODE, chn);
        read_error_counters(err_counters_2_2, TEST_NODE, chn);

        check_m((err_counters_1_1.err_fd + 1 = err_counters_2_1.err_fd),
                "ERR_FD incremented by 1 in transmitter!");
        check_m(err_counters_1_2.err_fd + 1 = err_counters_2_2.err_fd,
                "ERR_FD incremented by 1 in receiver!");

        check_m(err_counters_1_1.err_norm = err_counters_2_1.err_norm,
                "ERR_NORM not incremented by 1 in transmitter!");
        check_m(err_counters_1_2.err_norm = err_counters_2_2.err_norm,
                "ERR_NORM not incremented by 1 in receiver!");

  end procedure;
end package body;
