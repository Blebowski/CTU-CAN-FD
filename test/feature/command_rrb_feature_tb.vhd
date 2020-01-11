--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018
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
--  @1. Check RX buffer in Node 1 is empty. Generate CAN frame and send it by 
--      Node 2. Wait until frame is sent. Check RX Buffer is not empty.
--  @2. Issue COMMAND[RRB] and check that RX Buffer is empty again!
--  @3. Send CAN frame by Node 2, wait until frame is sent and check that RX
--      Buffer is not empty again.
--  @4. Send frame by Node 2. Wait until Node 1 starts receiving. Wait for
--      random amount of time and issue COMMAND[RRB]. Wait until bus is idle.
--      RX Buffer can be either empty (command was issued after first word
--      was stored), or there are is one frame (command was issued before
--      first word was stored).
--  @5. Send frame by Node 2 again. Wait until frame is sent and check it is
--      properly received by Node 1.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    21.10.2019   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package command_rrb_feature is
    procedure command_rrb_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body command_rrb_feature is
    procedure command_rrb_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1               :     natural := 1;
        variable ID_2               :     natural := 2;
        
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;
        variable frame_rx           :     SW_CAN_frame_type;
        
        variable rx_buf_info        :     SW_RX_Buffer_info;
        variable frames_equal       :     boolean := false;        

        variable command            :     SW_command := SW_command_rst_val;
    begin

        -----------------------------------------------------------------------
        --  @1. Check RX buffer in Node 1 is empty. Generate CAN frame and send 
        --     it by Node 2. Wait until frame is sent. Check RX Buffer is not
        --     empty.
        -----------------------------------------------------------------------
        info("Step 1");
        get_rx_buf_state(rx_buf_info, ID_1, mem_bus(1));
        check(rx_buf_info.rx_write_pointer = 0, "Write pointer 0");
        check(rx_buf_info.rx_read_pointer = 0, "Read pointer 0");
        check_false(rx_buf_info.rx_full, "Full flag not set");
        check(rx_buf_info.rx_empty, "Empty flag set");
        check(rx_buf_info.rx_frame_count = 0, "Frame count 0");
        
        CAN_generate_frame(rand_ctr, frame_1);
        CAN_insert_TX_frame(frame_1, 1, ID_2, mem_bus(2));
        send_TXT_buf_cmd(buf_set_ready, 1, ID_2, mem_bus(2));
        
        CAN_wait_frame_sent(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        get_rx_buf_state(rx_buf_info, ID_1, mem_bus(1));
        check(rx_buf_info.rx_write_pointer /= 0, "Write pointer not 0");
        check(rx_buf_info.rx_read_pointer = 0, "Read pointer still 0");
        check_false(rx_buf_info.rx_full, "Full flag not set");
        check_false(rx_buf_info.rx_empty, "Empty flag not set");
        check(rx_buf_info.rx_frame_count = 1, "Frame count 1");

        -----------------------------------------------------------------------
        -- @2. Issue COMMAND[RRB] and check that RX Buffer is empty again!
        -----------------------------------------------------------------------
        info("Step 2");
        command.release_rec_buffer := true;
        give_controller_command(command, ID_1, mem_bus(1));

        get_rx_buf_state(rx_buf_info, ID_1, mem_bus(1));
        check(rx_buf_info.rx_write_pointer = 0, "Write pointer 0");
        check(rx_buf_info.rx_read_pointer = 0, "Read pointer 0");
        check_false(rx_buf_info.rx_full, "Full flag not set");
        check(rx_buf_info.rx_empty, "Empty flag set");
        check(rx_buf_info.rx_frame_count = 0, "Frame count 0");
        
        -----------------------------------------------------------------------
        -- @3. Send CAN frame by Node 2, wait until frame is sent and check that
        --    RX Buffer is not empty again.
        -----------------------------------------------------------------------
        info("Step 3");
        CAN_generate_frame(rand_ctr, frame_1);
        CAN_insert_TX_frame(frame_1, 1, ID_2, mem_bus(2));
        send_TXT_buf_cmd(buf_set_ready, 1, ID_2, mem_bus(2));

        CAN_wait_frame_sent(ID_1, mem_bus(1));

        get_rx_buf_state(rx_buf_info, ID_1, mem_bus(1));
        check(rx_buf_info.rx_write_pointer /= 0, "Write pointer not 0");
        check(rx_buf_info.rx_read_pointer = 0, "Read pointer still 0");
        check_false(rx_buf_info.rx_full, "Full flag not set");
        check_false(rx_buf_info.rx_empty, "Empty flag not set");
        check(rx_buf_info.rx_frame_count = 1, "Frame count 1");

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

        -----------------------------------------------------------------------
        -- @4. Send frame by Node 2. Wait until Node 1 starts receiving. Wait
        --    for random amount of time and issue COMMAND[RRB].  Wait until
        --    bus is idle. RX Buffer can be either empty (command was issued
        --    after first word was stored), or there are is one frame (command
        --    was issued before first word was stored).
        -----------------------------------------------------------------------
        info("Step 4");
        CAN_generate_frame(rand_ctr, frame_1);
        CAN_insert_TX_frame(frame_1, 1, ID_2, mem_bus(2));
        send_TXT_buf_cmd(buf_set_ready, 1, ID_2, mem_bus(2));

        CAN_wait_tx_rx_start(false, true, ID_1, mem_bus(1));

        wait_rand_cycles(rand_ctr, mem_bus(1).clk_sys, 0, 20000);
        command.release_rec_buffer := true;
        give_controller_command(command, ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_1, mem_bus(2));
        
        get_rx_buf_state(rx_buf_info, ID_1, mem_bus(1));
        
        -- Now there could be either one or no frame, if there is one, first
        -- read it out and check it is correct!
        if (rx_buf_info.rx_frame_count > 0) then
            check(rx_buf_info.rx_frame_count = 1, "RX frame count 1!");
            CAN_read_frame(frame_rx, ID_1, mem_bus(1));
            CAN_compare_frames(frame_rx, frame_1, false, frames_equal);
        end if;
        
        -- Now there shouldbe no frame
        get_rx_buf_state(rx_buf_info, ID_1, mem_bus(1));
        
        check(rx_buf_info.rx_empty, "Empty flag set");
        check(rx_buf_info.rx_frame_count = 0, "Frame count 0");
        
        -----------------------------------------------------------------------
        -- @5. Send frame by Node 2 again. Wait until frame is sent and check it
        --    is properly received by Node 1.
        -----------------------------------------------------------------------
        info("Step 5");
        CAN_generate_frame(rand_ctr, frame_1);
        CAN_insert_TX_frame(frame_1, 1, ID_2, mem_bus(2));
        send_TXT_buf_cmd(buf_set_ready, 1, ID_2, mem_bus(2));

        CAN_wait_frame_sent(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));
        
        get_rx_buf_state(rx_buf_info, ID_1, mem_bus(1));
        check(rx_buf_info.rx_write_pointer /= 0, "Write pointer not 0");
        check_false(rx_buf_info.rx_full, "Full flag not set");
        check_false(rx_buf_info.rx_empty, "Empty flag not set");
        check(rx_buf_info.rx_frame_count = 1, "Frame count 1");
        
        CAN_read_frame(frame_rx, ID_1, mem_bus(1));
        CAN_compare_frames(frame_rx, frame_1, false, frames_equal);
        info("TX frame:");
        CAN_print_frame(frame_1);
        info("RX frame:");
        CAN_print_frame(frame_rx);
        check(frames_equal, "TX vs. RX frames match!");

        -- Issue COMMAND[RRB] to clean-up after itself (for next iterations)
        command.release_rec_buffer := true;
        give_controller_command(command, ID_1, mem_bus(1));

        wait for 100 ns;

  end procedure;

end package body;
