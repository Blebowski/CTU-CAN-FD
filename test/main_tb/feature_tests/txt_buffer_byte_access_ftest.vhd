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
--  Feature test verifying 8/16 bit writes to TXT buffer. 
--
-- @Verifies:
--  @1. TXT Buffer RAM can be filled by 8/16 bit accesses.
--
-- @Test sequence:
--  @1. Generate random CAN frame, insert it into TXT buffer 0 of DUT by using
--      byte accesses. Send frame and wait until it is sent.
--  @2. Read received frame from Test Node and check that it is matching sent
--      frame
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    12.06.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package txt_buffer_byte_access_ftest is
    procedure txt_buffer_byte_access_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body txt_buffer_byte_access_ftest is
    procedure txt_buffer_byte_access_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        -- Generated frames
        variable frame_tx           :     SW_CAN_frame_type;
        variable frame_rx           :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;
        variable frame_sent         :     boolean;
        variable frame_equal        :     boolean;
    begin
        
        -----------------------------------------------------------------------
        -- @1. Generate random CAN frame, insert it into TXT buffer 0 of DUT by
        --     using byte accesses. Send frame and wait until it is sent.
        -----------------------------------------------------------------------
        info_m("Step 1");
        
        CAN_generate_frame(frame_tx);
        frame_tx.data_length := 64;
        frame_tx.frame_format := FD_CAN;
        decode_length(frame_tx.data_length, frame_tx.dlc);
        decode_dlc_rx_buff(frame_tx.dlc, frame_tx.rwcnt);
        for i in 0 to 63 loop
            rand_logic_vect_v(frame_tx.data(i), 0.5);
        end loop;

        CAN_insert_TX_frame(frame_tx, 1, DUT_NODE, chn, byte_access => true);
        send_TXT_buf_cmd(buf_set_ready, 1, DUT_NODE, chn);
        CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);
        
        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);
        
        -----------------------------------------------------------------------
        -- @2. Read received frame from Test Node and check that it is matching
        --     sent frame.
        -----------------------------------------------------------------------
        info_m("Step 2");
        
        CAN_read_frame(frame_rx, TEST_NODE, chn);
        CAN_compare_frames(frame_rx, frame_tx, false, frame_equal);

        check_m(frame_equal, "TX/RX frame match");

  end procedure;

end package body;
