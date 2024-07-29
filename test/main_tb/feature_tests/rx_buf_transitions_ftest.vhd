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
--  RX Buffer FSM transitions - feature test
--
-- @Verifies:
--  @1. Verifies that when Error condition occurs exactly one cycle after
--      "store_metadata" command arrived, the RX frame is aborted.
--
-- @Test sequence:
--  @1. Configure Loopback mode in DUT. This is to be able to easily invoke
--      bit error at the last bit of DLC, while DUT is storing received frame
--      into RX Buffer.
--  @2. Send CAN frame by DUT Node. Wait until last bit of DLC and flip
--      bus level.
--  @3. Wait until error frame is sent by DUT Node. Wait until bus is idle.
--      Check RX Buffer pointers were correctly reset due to frame abort.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    29.7.2024   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package rx_buf_transitions_ftest is
    procedure rx_buf_transitions_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body rx_buf_transitions_ftest is
    procedure rx_buf_transitions_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable mode               :       SW_mode := SW_mode_rst_val;
        variable can_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean;
        variable rx_buf_info        :       SW_RX_Buffer_info;
    begin

        -----------------------------------------------------------------------
        -- @1. Configure Loopback mode in DUT. This is to be able to easily
        --     invoke bit error at the last bit of DLC, while DUT is storing
        --     received frame into RX Buffer.
        -----------------------------------------------------------------------
        info_m("Step 1: Set DUT to loopback");

        mode.internal_loopback := true;
        set_core_mode(mode, DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @2. Send CAN frame by DUT Node. Wait until last bit of DLC and flip
        --     bus level.
        -----------------------------------------------------------------------
        info_m("Step 1: Send CAN frame");

        CAN_generate_frame(can_frame);
        CAN_frame.frame_format := NORMAL_CAN;

        CAN_send_frame(can_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_pc_state(pc_deb_control, DUT_NODE, chn);

        -- Wait until start of last DLC bit
        for i in 1 to 5 loop
            CAN_wait_sample_point(DUT_NODE, chn);
        end loop;
        wait for 20 ns;

        flip_bus_level(chn);
        CAN_wait_sample_point(DUT_NODE, chn);

        wait for 20 ns;
        release_bus_level(chn);

        -----------------------------------------------------------------------
        -- @3. Wait until error frame is sent by DUT Node. Wait until bus is
        --     idle. Check RX Buffer pointers were correctly reset due to
        --     frame abort.
        -----------------------------------------------------------------------
        info_m("Step 3: Wait till Error frame");

        CAN_wait_error_frame(DUT_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);

        get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
        check_m(rx_buf_info.rx_mem_free = rx_buf_info.rx_buff_size,
                    "RX MEM Free = RX Buffer Size");
        check_m(rx_buf_info.rx_write_pointer = 0,
                    "RX Buffer write pointer 0");
        check_m(rx_buf_info.rx_read_pointer = 0,
                    "RX Buffer read pointer 0");

  end procedure;

end package body;
