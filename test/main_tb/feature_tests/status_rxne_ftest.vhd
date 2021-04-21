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
--  STATUS[RXNE] feature test.
--
-- @Verifies:
--  @1. When no frame is stored in RX Buffer, STATUS[RXNE] is not set.
--  @2. When one or more frames is stored in RX Buffer, STATUS[RXNE] is set.
--  @3. STATUS[RXNE] is not set when last word of last frame in RX Buffer is
--      read.
--
-- @Test sequence:
--  @1. Read STATUS[RXNE] of DUT and check it is not set. Send random amount
--      of CAN frames by Test node and wait until they are received. Check that
--      after each one, STATUS[RXNE] is set.
--  @2. Read out frame by frame and check that STATUS[RXNE] is still set. Read
--      all frames but last one.
--  @3. Read out last frame word by word and check that STATUS[RXNE] is still
--      set and STATUS[RXNE] is not set after reading out last word.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    31.10.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package status_rxne_ftest is
    procedure status_rxne_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body status_rxne_ftest is
    procedure status_rxne_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable r_data             :     std_logic_vector(31 downto 0) :=
                                                (OTHERS => '0');

        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;
        variable frame_rx           :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;

        variable num_frames         :     integer;
    begin

        -----------------------------------------------------------------------
        --  @1. Read STATUS[RXNE] of DUT and check it is not set. Send
        --      random amount of CAN frames by Test node and wait until they 
        --      are received. Check that after each one, STATUS[RXNE] is set.
        -----------------------------------------------------------------------
        info_m("Step 1");
        get_controller_status(stat_1, DUT_NODE, chn);
        check_false_m(stat_1.receive_buffer, "RX Buffer empty");
        
        rand_int_v(6, num_frames);
        num_frames := num_frames + 1;
        
        CAN_generate_frame(frame_1);
        frame_1.rtr := RTR_FRAME;
        frame_1.frame_format := NORMAL_CAN;
        CAN_insert_TX_frame(frame_1, 1, TEST_NODE, chn);
        
        for i in 0 to num_frames - 1 loop
            send_TXT_buf_cmd(buf_set_ready, 1, TEST_NODE, chn);
            CAN_wait_frame_sent(TEST_NODE, chn);
            
            CAN_wait_bus_idle(DUT_NODE, chn);
            CAN_wait_bus_idle(TEST_NODE, chn);
            
            get_controller_status(stat_1, DUT_NODE, chn);
            check_m(stat_1.receive_buffer, "RX Buffer not empty");
        end loop;

        -----------------------------------------------------------------------
        --  @2. Read out frame by frame and check that STATUS[RXNE] is still set.
        --     Read all frames but last one.
        -----------------------------------------------------------------------
        info_m("Step 2");
        for i in 0 to num_frames - 2 loop
            CAN_read_frame(frame_rx, DUT_NODE, chn);
            get_controller_status(stat_1, DUT_NODE, chn);
            check_m(stat_1.receive_buffer, "RX Buffer not empty");
        end loop;
        
        -----------------------------------------------------------------------
        --  @3. Read out last frame word by word and check that STATUS[RXNE] is
        --     still set and STATUS[RXNE] is not set after reading out last
        --     word.
        -----------------------------------------------------------------------
        for i in 0 to 3 loop -- RTR frame has 4 words in RX Buffer
            CAN_read(r_data, RX_DATA_ADR, DUT_NODE, chn);
            get_controller_status(stat_1, DUT_NODE, chn);
            
            if (i = 3) then
                check_false_m(stat_1.receive_buffer,
                    "STATUS[RXNE] not set after last word was read out!");
            else
                check_m(stat_1.receive_buffer,
                    "STATUS[RXNE] set before last word was read out!");
            end if;
        end loop;

  end procedure;
end package body;
