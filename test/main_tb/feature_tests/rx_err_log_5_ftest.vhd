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
--  RX Error logging feature test 5
--
-- @Verifies:
--  @1. When an Error occurs after Metadata word of a received frame was
--      written into the RX Buffer, the RX Buffer pointers are properly
--      reverted and the Error frame is corerctly logged into Rx Buffer.
--  @2. When an Error occurs after a Data Word of a received frame is logged
--      into RX Buffer, the Error frame is correctly logged and pointers are
--      properly reverted.
--
-- @Test sequence:
--  @1. Configure DUT to MODE[ERFM] = 1.
--  @2. Iterate over following scenarios:
--          - Error frame invoked on DUT side by causing stuff error during
--            identifier.
--          - Error frame invoked on DUT side by causing stuff error after
--            the last bit of DLC (Metadata has been stored).
--          - Error frame invoked on DUT side by causing stuff error in 5-th
--            byte of Data word (after first data word has been stored).
--
--      @2.1 Generate CAN frame, send it by Test Node and wait until it is
--           received by DUT Node. Check there is one frame in DUTs RX Buffer.
--      @2.2 Generate another CAN frame, send it by Test Node and wait until
--           the stuff bit that shall be flipped. Invoke stuff error on DUTs
--           side, and check there are two CAN frames in DUTs RX Buffer.
--           Wait until bus is idle.
--      @2.3 Generate CAN frame, send it by Test Node and wait until it is
--           received by DUT Node. Check there are 3 frames in DUTs RX Buffer.
--      @2.4 Read all 3 frames from previous 3 steps. Check that first frame is
--           regular CAN frame, and matches frame sent by Test Node in Step 1.
--           Check second frame is an Error frame with IVLD=1 and Identifier
--           matching the Identifier of frame sent by Test Node in Step 2.
--           Check third frame is a CAN frame mathcing frame sent in Step 1.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    15.8.2024   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package rx_err_log_5_ftest is
    procedure rx_err_log_5_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body rx_err_log_5_ftest is

    procedure rx_err_log_5_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        --variable mode_1             : SW_mode := SW_mode_rst_val;
        --variable mode_2             : SW_mode := SW_mode_rst_val;
        --variable status             : SW_status;
        --variable frame_sent         : boolean;
        --variable CAN_frame          : SW_CAN_frame_type;
        --variable err_frame          : SW_CAN_frame_type;
        --variable rx_buf_info        : SW_RX_Buffer_info;
        --variable can_rx             : std_logic;
    begin



    end procedure;

end package body;
