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
--  Clear data overrun command feature test.
--
-- @Verifies:
--  @1. Data overrun flag is set when more frames are received than capacity of
--      RX Buffer.
--  @2. RX Buffer full is active when there is no free space available in RX
--      Buffer.
--  @3. COMMAND[CDO] will clear data overrun flag.
--  @4. COMMAND[RRB] will clear data overrun flag.
--
-- @Test sequence:
--  @1. Read size of RX Buffer in DUT. Check that DOR flag is not set.
--      Generate number of RTR CAN frames (4 words in RX Buffer) which exactly
--      fill RX Buffer and send them by Test node. Wait until frames are sent.
--  @2. Read status of RX Buffer in DUT. Check that RX Buffer full is active.
--      Check that DOR flag is not set yet.
--  @3. Send one more frame by Test node and wait until it is sent. Check that Data
--      Overrun flag is set. Issue Clear Data Overrun Command. Check that CDO
--      flag was cleared.
--  @4. Again send frame by Test node. Check DOR is set again. Issue Command[RRB].
--      Check Data Overrun Flag was cleared.
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

package command_cdo_ftest is
    procedure command_cdo_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body command_cdo_ftest is
    procedure command_cdo_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;
        
        variable rx_buf_info        :     SW_RX_Buffer_info;    

        variable command            :     SW_command := SW_command_rst_val;
    begin

        -----------------------------------------------------------------------
        -- @1. Read size of RX Buffer in DUT. Check that DOR flag is not set.
        --     Generate number of RTR CAN frames (4 words in RX Buffer) which
        --     exactly fill RX Buffer and send them by Test node. Wait until
        --     frames are sent. 
        -----------------------------------------------------------------------
        info_m("Step 1");

        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        check_false_m(rx_buf_info.rx_full, "RX full not set!");
        
        get_controller_status(stat_1, DUT_NODE, chn);
        check_false_m(stat_1.data_overrun, "DOR flag not set!");
        
        CAN_generate_frame(frame_1);
        frame_1.rtr := RTR_FRAME;
        frame_1.frame_format := NORMAL_CAN;
        CAN_insert_TX_frame(frame_1, 1, TEST_NODE, chn);
        send_TXT_buf_cmd(buf_set_ready, 1, TEST_NODE, chn);
        
        info_m("DUT RX Buffer size: " & integer'image(rx_buf_info.rx_buff_size));
        info_m("Sending " & integer'image(rx_buf_info.rx_buff_size / 4) &
             " RTR frames");

        for i in 0 to (rx_buf_info.rx_buff_size / 4) - 1 loop
            info_m("Sending frame nr: " & integer'image(i));
            send_TXT_buf_cmd(buf_set_ready, 1, TEST_NODE, chn);
            CAN_wait_frame_sent(TEST_NODE, chn);
        end loop;

        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);

        -----------------------------------------------------------------------
        -- @2. Read status of RX Buffer in DUT. Check that RX Buffer full is
        --     active. Check that DOR flag is not set yet.
        -----------------------------------------------------------------------
        info_m("Step 2");

        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        check_m(rx_buf_info.rx_full, "RX full set");

        get_controller_status(stat_1, DUT_NODE, chn);
        check_false_m(stat_1.data_overrun, "DOR flag not set!");

        -----------------------------------------------------------------------
        -- @3. Send one more frame by Test node and wait until it is sent. Check
        --    that Data Overrun flag is set. Issue Clear Data Overrun Command.
        --    Check that CDO flag was cleared.
        -----------------------------------------------------------------------
        info_m("Step 3");

        send_TXT_buf_cmd(buf_set_ready, 1, TEST_NODE, chn);
        CAN_wait_frame_sent(TEST_NODE, chn);
        
        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);
        
        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        check_m(rx_buf_info.rx_full, "RX full set");
        get_controller_status(stat_1, DUT_NODE, chn);
        check_m(stat_1.data_overrun, "DOR flag set!");       

        command.clear_data_overrun := true;
        give_controller_command(command, DUT_NODE, chn);

        get_controller_status(stat_1, DUT_NODE, chn);
        check_false_m(stat_1.data_overrun, "DOR flag was cleared!");

        -----------------------------------------------------------------------
        -- @4. Again send frame by Test node. Check DOR is set again. Issue
        --     Command[RRB]. Check Data Overrun Flag was cleared.
        -----------------------------------------------------------------------
        info_m("Step 4");

        send_TXT_buf_cmd(buf_set_ready, 1, TEST_NODE, chn);
        CAN_wait_frame_sent(DUT_NODE, chn);
        
        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);
        
        get_controller_status(stat_1, DUT_NODE, chn);
        check_m(stat_1.data_overrun, "DOR flag set!");

        command.clear_data_overrun := false;
        command.release_rec_buffer := true;
        give_controller_command(command, DUT_NODE, chn);

        get_controller_status(stat_1, DUT_NODE, chn);
        check_false_m(stat_1.data_overrun, "DOR flag cleared by COMMAND[RRB]!");

  end procedure;

end package body;