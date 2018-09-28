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
-- Purpose:
--  RX Buffer status feature test implementation.
--
--  Test sequence:
--    1. RX Buffer size is read and buffer is cleared.
--    2. Free memory, buffer status and message count is checked.
--    3. Random frames are sent on the bus by node 2 and recieved by node 1
--    4. After each frame amount of remaining memory is checked towards expected
--       value.
--    5. When buffer is filled Data overrun flag is checked and cleared.
--    6. After clearing Overrun flag, it is checked it was really cleared.
--
--------------------------------------------------------------------------------
-- Revision History:
--
--    21.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--     11.6.2018  Modified to use CAN Test lib functions instead of direct
--                register access.
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;
use work.pkg_feature_exec_dispath.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

package rx_status_feature is
    procedure rx_status_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body rx_status_feature is
    procedure rx_status_feature_exec(
        variable    o               : out    feature_outputs_t;
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1           	:       natural range 0 to 15 := 1;
        variable ID_2           	:       natural range 0 to 15 := 2;
        variable CAN_frame          :       SW_CAN_frame_type;
        variable send_more          :       boolean := true;
        variable in_RX_buf          :       natural range 0 to 1023;
        variable frame_sent         :       boolean := false;
        variable number_frms_sent   :       natural range 0 to 1023;

        variable buf_info           :       SW_RX_Buffer_info;
        variable command            :       SW_command := (false, false, false);
        variable status             :       SW_status;
    begin
        o.outcome := true;

        ------------------------------------------------------------------------
        -- Restart the content of the buffer...
        ------------------------------------------------------------------------
        command.release_rec_buffer := true;
        give_controller_command(command, ID_1, mem_bus(1));
        command.release_rec_buffer := false;

        ------------------------------------------------------------------------
        -- Read the size of the synthesized buffer
        ------------------------------------------------------------------------
        get_rx_buf_state(buf_info, ID_1, mem_bus(1));

        ------------------------------------------------------------------------
        -- Check that buffer is empty
        ------------------------------------------------------------------------
        if (not buf_info.rx_empty) then
            -- LCOV_EXCL_START
            o.outcome := false;
            report "RX Buffer is not empty after Release receive Buffer command"
                severity error;
            -- LCOV_EXCL_STOP
        end if;

        ------------------------------------------------------------------------
        -- Check that free memory is equal to buffer size
        ------------------------------------------------------------------------
        if (buf_info.rx_buff_size /= buf_info.rx_mem_free) then
            -- LCOV_EXCL_START
            report "Number of free words in RX Buffer after Release Receive " &
                "Buffer command is not equal to buffer size" severity error;
            o.outcome := false;
            -- LCOV_EXCL_STOP
        end if;

        ------------------------------------------------------------------------
        -- Check that both pointers are 0 as well
        -- as message count
        ------------------------------------------------------------------------
        if (buf_info.rx_frame_count /= 0 or buf_info.rx_write_pointer /= 0 or
            buf_info.rx_read_pointer /= 0)
        then
            -- LCOV_EXCL_START
            o.outcome := false;
            report "RX Buffer pointers are not 0 after Release Receieve Buffer" &
                    " command" severity error;
            -- LCOV_EXCL_START
        end if;

        ------------------------------------------------------------------------
        -- Generate the CAN frames and send them by Node 2
        ------------------------------------------------------------------------
        while send_more loop
            CAN_generate_frame(rand_ctr, CAN_frame);

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

            CAN_send_frame(CAN_frame, 1, ID_2, mem_bus(2), frame_sent);
            CAN_wait_frame_sent(ID_1, mem_bus(1));
            number_frms_sent := number_frms_sent + 1;
            in_RX_buf := in_RX_buf + CAN_frame.rwcnt + 1;

            --------------------------------------------------------------------
            -- Check that message count was incremented and memfree is correct!
            --------------------------------------------------------------------
            get_rx_buf_state(buf_info, ID_1, mem_bus(1));
            if (number_frms_sent /= buf_info.rx_frame_count and send_more) then
                -- LCOV_EXCL_START
                o.outcome := false;
                report "Number of frames in RX Buffer not incremented"
                    severity error;
                -- LCOV_EXCL_STOP
            end if;
            if ((buf_info.rx_mem_free + in_RX_buf) /= buf_info.rx_buff_size
                and send_more)
            then
                -- LCOV_EXCL_START
                o.outcome := false;
                report "RX Buffer free memory + Number of stored words does " &
                    "not equal to RX Buffer size!" severity error;
                -- LCOV_EXCL_STOP
            end if;

        end loop;

        ------------------------------------------------------------------------
        -- Check that data overrun status is set (we sent one more frame than
        -- needed... Overrun should be present
        ------------------------------------------------------------------------
        get_controller_status(status, ID_1, mem_bus(1));
        if (not status.data_overrun) then
            -- LCOV_EXCL_START
            o.outcome := false;
            report "Data overrun not ocurred as expected!" severity error;
            -- LCOV_EXCL_STOP
        end if;

        ------------------------------------------------------------------------
        -- Clear the data overrun flag
        ------------------------------------------------------------------------
        command.clear_data_overrun := true;
        give_controller_command(command, ID_1, mem_bus(1));
        command.clear_data_overrun := false;

        ------------------------------------------------------------------------
        -- Check that overrun flag was cleared
        ------------------------------------------------------------------------
        get_controller_status(status, ID_1, mem_bus(1));
        if (status.data_overrun) then
            -- LCOV_EXCL_START
            o.outcome := false;
            report "Data Overrun flag not active!" severity error;
            -- LCOV_EXCL_STOP
        end if;

    end procedure;

end package body;
