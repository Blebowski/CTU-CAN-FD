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
--  ERR_CAPT[ERR_POS] = ERC_POS_SOF feature test. 
--
-- @Verifies:
--  @1. Detection of form error in SOF bit.
--
-- @Test sequence:
--  @1. Generate CAN frame and send it by DUT. Wait until transmission starts
--      and force bus Recessive. Wait until sample point and check that Error
--      frame is transmitted. Check that ERR_CAPT says that Form Error during
--      SOF was detected!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    02.02.2020   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package err_capt_sof_ftest is
    procedure err_capt_sof_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body err_capt_sof_ftest is
    procedure err_capt_sof_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is        
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;
        
        -- Node status
        variable stat_1             :     SW_status;
        
        variable frame_sent         :     boolean;
        variable err_capt           :     SW_error_capture;
    begin

        -----------------------------------------------------------------------
        -- @1. Generate CAN frame and send it by DUT. Wait until transmission
        --    starts and force bus Recessive. Wait until sample point and check
        --    that Error frame is transmitted. Check that ERR_CAPT says that
        --    Form Error during SOF was detected!
        -----------------------------------------------------------------------
        info_m("Step 1");

        CAN_generate_frame(frame_1);
        CAN_send_frame(frame_1, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);

        force_bus_level(RECESSIVE, chn);
        CAN_wait_sample_point(DUT_NODE, chn);
        wait for 20 ns; -- To be sure that opposite bit is sampled!
        release_bus_level(chn);
        
        get_controller_status(stat_1, DUT_NODE, chn);
        check_m(stat_1.error_transmission, "Error frame is being transmitted!");
        
        CAN_read_error_code_capture(err_capt, DUT_NODE, chn);
        check_m(err_capt.err_type = can_err_form, "Form error detected!");
        check_m(err_capt.err_pos = err_pos_sof, "Error detected in SOF!");
        
        CAN_wait_bus_idle(DUT_NODE, chn);

  end procedure;

end package body;
