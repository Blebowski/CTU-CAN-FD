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
--  Release receive buffer command feature test.
--
-- @Verifies:
--  @1. RX Buffer is reset by issuing COMMAND[RRB].
--  @2. COMMAND[RRB] issued during CAN frame reception will erase RX Buffer
--      properly, and RX buffer will remain consistent.
--
-- @Test sequence:
--  @1. Check RX buffer in DUT is empty. Generate CAN frame and send it by 
--      Test node. Wait until frame is sent. Check RX Buffer is not empty.
--  @2. Issue COMMAND[RRB] and check that RX Buffer is empty again!
--  @3. Send CAN frame by Test node, wait until frame is sent and check that RX
--      Buffer is not empty again.
--  @4. Send frame by Test node. Wait until DUT starts receiving. Wait for
--      random amount of time and issue COMMAND[RRB]. Wait until bus is idle.
--      RX Buffer can be either empty (command was issued after first word
--      was stored), or there are is one frame (command was issued before
--      first word was stored).
--  @5. Send frame by Test node again. Wait until frame is sent and check it is
--      properly received by DUT.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    21.10.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.clk_gen_agent_pkg.all;

package command_rrb_ftest is
    procedure command_rrb_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body command_rrb_ftest is
    procedure command_rrb_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is        
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;
        variable frame_rx           :     SW_CAN_frame_type;
        
        variable rx_buf_info        :     SW_RX_Buffer_info;
        variable frames_equal       :     boolean := false;        

        variable command            :     SW_command := SW_command_rst_val;
        variable rand_val           :     integer;
    begin

        -----------------------------------------------------------------------
        --  @1. Check RX buffer in DUT is empty. Generate CAN frame and send 
        --      it by Test node. Wait until frame is sent. Check RX Buffer is 
        --      not empty.
        -----------------------------------------------------------------------
        info_m("Step 1");

        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        
        check_m(rx_buf_info.rx_write_pointer = 0, "Write pointer 0");
        check_m(rx_buf_info.rx_read_pointer = 0, "Read pointer 0");
        check_false_m(rx_buf_info.rx_full, "Full flag not set");
        check_m(rx_buf_info.rx_empty, "Empty flag set");
        check_m(rx_buf_info.rx_frame_count = 0, "Frame count 0");
        
        CAN_generate_frame(frame_1);
        CAN_insert_TX_frame(frame_1, 1, TEST_NODE, chn);
        send_TXT_buf_cmd(buf_set_ready, 1, TEST_NODE, chn);
        
        CAN_wait_frame_sent(DUT_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);

        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        
        check_m(rx_buf_info.rx_write_pointer /= 0, "Write pointer not 0");
        check_m(rx_buf_info.rx_read_pointer = 0, "Read pointer still 0");
        check_false_m(rx_buf_info.rx_full, "Full flag not set");
        check_false_m(rx_buf_info.rx_empty, "Empty flag not set");
        check_m(rx_buf_info.rx_frame_count = 1, "Frame count 1");

        -----------------------------------------------------------------------
        -- @2. Issue COMMAND[RRB] and check that RX Buffer is empty again!
        -----------------------------------------------------------------------
        info_m("Step 2");
        
        command.release_rec_buffer := true;
        give_controller_command(command, DUT_NODE, chn);

        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        check_m(rx_buf_info.rx_write_pointer = 0, "Write pointer 0");
        check_m(rx_buf_info.rx_read_pointer = 0, "Read pointer 0");
        check_false_m(rx_buf_info.rx_full, "Full flag not set");
        check_m(rx_buf_info.rx_empty, "Empty flag set");
        check_m(rx_buf_info.rx_frame_count = 0, "Frame count 0");
        
        -----------------------------------------------------------------------
        -- @3. Send CAN frame by Test node, wait until frame is sent and check that
        --    RX Buffer is not empty again.
        -----------------------------------------------------------------------
        info_m("Step 3");
        
        CAN_generate_frame(frame_1);
        CAN_insert_TX_frame(frame_1, 1, TEST_NODE, chn);
        send_TXT_buf_cmd(buf_set_ready, 1, TEST_NODE, chn);

        CAN_wait_frame_sent(DUT_NODE, chn);

        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        check_m(rx_buf_info.rx_write_pointer /= 0, "Write pointer not 0");
        check_m(rx_buf_info.rx_read_pointer = 0, "Read pointer still 0");
        check_false_m(rx_buf_info.rx_full, "Full flag not set");
        check_false_m(rx_buf_info.rx_empty, "Empty flag not set");
        check_m(rx_buf_info.rx_frame_count = 1, "Frame count 1");

        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);

        -----------------------------------------------------------------------
        -- @4. Send frame by Test node. Wait until DUT starts receiving. Wait
        --    for random amount of time and issue COMMAND[RRB].  Wait until
        --    bus is idle. RX Buffer can be either empty (command was issued
        --    after first word was stored), or there are is one frame (command
        --    was issued before first word was stored).
        -----------------------------------------------------------------------
        info_m("Step 4");
        
        CAN_generate_frame(frame_1);
        CAN_insert_TX_frame(frame_1, 1, TEST_NODE, chn);
        send_TXT_buf_cmd(buf_set_ready, 1, TEST_NODE, chn);

        CAN_wait_tx_rx_start(false, true, DUT_NODE, chn);

        rand_int_v(20000, rand_val);
        for i in 0 to rand_val loop
            clk_agent_wait_cycle(chn);
        end loop;
       
        command.release_rec_buffer := true;
        give_controller_command(command, DUT_NODE, chn);
        
        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);
        
        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        
        -- Now there could be either one or no frame, if there is one, first
        -- read it out and check it is correct!
        if (rx_buf_info.rx_frame_count > 0) then
            check_m(rx_buf_info.rx_frame_count = 1, "RX frame count 1!");
            CAN_read_frame(frame_rx, DUT_NODE, chn);
            CAN_compare_frames(frame_rx, frame_1, false, frames_equal);
        end if;
        
        -- Now there shouldbe no frame
        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        
        check_m(rx_buf_info.rx_empty, "Empty flag set");
        check_m(rx_buf_info.rx_frame_count = 0, "Frame count 0");
        
        -----------------------------------------------------------------------
        -- @5. Send frame by Test node again. Wait until frame is sent and check it
        --    is properly received by DUT.
        -----------------------------------------------------------------------
        info_m("Step 5");

        CAN_generate_frame(frame_1);
        CAN_insert_TX_frame(frame_1, 1, TEST_NODE, chn);
        send_TXT_buf_cmd(buf_set_ready, 1, TEST_NODE, chn);

        CAN_wait_frame_sent(DUT_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);
        
        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        check_m(rx_buf_info.rx_write_pointer /= 0, "Write pointer not 0");
        check_false_m(rx_buf_info.rx_full, "Full flag not set");
        check_false_m(rx_buf_info.rx_empty, "Empty flag not set");
        check_m(rx_buf_info.rx_frame_count = 1, "Frame count 1");
        
        CAN_read_frame(frame_rx, DUT_NODE, chn);
        CAN_compare_frames(frame_rx, frame_1, false, frames_equal);
        info_m("TX frame:");
        CAN_print_frame(frame_1);
        info_m("RX frame:");
        CAN_print_frame(frame_rx);
        check_m(frames_equal, "TX vs. RX frames match!");

        -- Issue COMMAND[RRB] to clean-up after itself (for next iterations)
        command.release_rec_buffer := true;
        give_controller_command(command, DUT_NODE, chn);

  end procedure;

end package body;
