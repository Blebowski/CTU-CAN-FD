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
--  RX Buffer status feature test implementation.
--
-- @Test sequence:
--   @1. RX Buffer size is read and buffer is cleared.
--   @2. Free memory, buffer status and message count is checked.
--   @3. Random frames are sent on the bus by Test Node and recieved by DUT.
--   @4. After each frame amount of remaining memory is checked towards expected
--       value.
--   @5. When buffer is filled Data overrun flag is checked and cleared.
--   @6. After clearing Overrun flag, it is checked it was really cleared.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--
--    21.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--     11.6.2018  Modified to use CAN Test lib functions instead of direct
--                register access.
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package rx_status_ftest is
    procedure rx_status_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body rx_status_ftest is
    procedure rx_status_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :       SW_CAN_frame_type;
        variable send_more          :       boolean := true;
        variable in_RX_buf          :       natural;
        variable frame_sent         :       boolean := false;
        variable number_frms_sent   :       natural;

        variable buf_info           :       SW_RX_Buffer_info;
        variable command            :       SW_command := SW_command_rst_val;
        variable status             :       SW_status;
        variable frame_counter      :       natural;
        
        variable big_rx_buffer      :       boolean;
    begin

        ------------------------------------------------------------------------
        -- @1. RX Buffer size is read and buffer is cleared.
        ------------------------------------------------------------------------
        info_m("Step 1");

        command.release_rec_buffer := true;
        give_controller_command(command, DUT_NODE, chn);
        command.release_rec_buffer := false;

        get_rx_buf_state(buf_info, DUT_NODE, chn);

        ------------------------------------------------------------------------
        -- @2. Free memory, buffer status and message count is checked.
        ------------------------------------------------------------------------
        info_m("Step 2");

        check_m(buf_info.rx_empty,
              "RX Buffer is not empty after Release receive Buffer command");

        check_m(buf_info.rx_buff_size = buf_info.rx_mem_free,
             "Number of free words in RX Buffer after Release Receive " &
             "Buffer command is not equal to buffer size");

        check_m(buf_info.rx_frame_count = 0 and
                buf_info.rx_write_pointer = 0 and
                buf_info.rx_read_pointer = 0,
                "RX Buffer pointers are not 0 after Release Receieve Buffer command");

        -- Check RX buffer size. If it is more than 512, mark it as big.
        -- This is used further to reduce lenght of test
        big_rx_buffer := false;
        if (buf_info.rx_buff_size > 511) then
            big_rx_buffer := true;
        end if;

        ------------------------------------------------------------------------
        -- @3. Random frames are sent on the bus by Test Node and received by DUT.
        ------------------------------------------------------------------------
        info_m("Step 3");

        frame_counter := 1;
        while send_more loop
            info_m("Sending frame nr. : " & integer'image(frame_counter));
            frame_counter := frame_counter + 1;

            CAN_generate_frame(CAN_frame);

            -- If RX buffer of DUT is "big" send only frames with 64 bytes
            -- to reduce total number of sent frames and therefore duration of
            -- the test!
            if big_rx_buffer then
                info_m("Using BIG RX Buffer settings (long frame) to reduce test time...");
                CAN_frame.identifier := CAN_frame.identifier mod (2 ** 11);
                CAN_frame.ident_type := BASE;
                CAN_frame.frame_format := FD_CAN;
                CAN_frame.brs := BR_SHIFT;
                CAN_frame.data_length := 64;
                -- We dont care about the data content, they are zeroes!
                decode_length(CAN_frame.data_length, CAN_frame.dlc);
                decode_dlc_rx_buff(CAN_frame.dlc, CAN_frame.rwcnt);
            end if;

            -- Evaluate if next frame should be sent
            if (CAN_frame.rtr = RTR_FRAME and
                CAN_frame.frame_format = NORMAL_CAN)
            then
                if (in_RX_buf + 4 > buf_info.rx_buff_size) then
                    send_more := false;
                end if;
            else
                if (CAN_frame.data_length mod 4 = 0) then
                    if ((in_RX_buf + CAN_frame.data_length / 4 + 4) >
                        buf_info.rx_buff_size)
                    then
                        send_more := false;
                    end if;
                else
                    if ((in_RX_buf + CAN_frame.data_length / 4 + 5) >
                        buf_info.rx_buff_size)
                    then
                        send_more := false;
                    end if;
                end if;
            end if;

            CAN_send_frame(CAN_frame, 1, TEST_NODE, chn, frame_sent);
            CAN_wait_frame_sent(DUT_NODE, chn);
            
            CAN_wait_bus_idle(DUT_NODE, chn);
            CAN_wait_bus_idle(TEST_NODE, chn);

            number_frms_sent := number_frms_sent + 1;
            in_RX_buf := in_RX_buf + CAN_frame.rwcnt + 1;

            --------------------------------------------------------------------
            -- @4. After each frame amount of remaining memory is checked
            --     towards expected value.
            --------------------------------------------------------------------
            info_m("Step 4");
            get_rx_buf_state(buf_info, DUT_NODE, chn);
            check_m((number_frms_sent = buf_info.rx_frame_count) or (not send_more),
                    "Number of frames in RX Buffer not incremented");
                  
            check_m((buf_info.rx_mem_free + in_RX_buf) = buf_info.rx_buff_size or
                    (not send_more),
                    "RX Buffer free memory + Number of stored words does " &
                    "not equal to RX Buffer size!");
        end loop;

        ------------------------------------------------------------------------
        -- @5. When buffer is filled Data overrun flag is checked and cleared.
        ------------------------------------------------------------------------
        info_m("Step 5");

        get_controller_status(status, DUT_NODE, chn);
        check_m(status.data_overrun, "Data overrun not ocurred as expected!");

        ------------------------------------------------------------------------
        -- @6. After clearing Overrun flag, it is checked it was really cleared.
        ------------------------------------------------------------------------
        info_m("Step 6");
        
        command.clear_data_overrun := true;
        give_controller_command(command, DUT_NODE, chn);
        command.clear_data_overrun := false;

        get_controller_status(status, DUT_NODE, chn);
        check_false_m(status.data_overrun, "Data Overrun flag not cleared!");

    end procedure;

end package body;