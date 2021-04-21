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
--  ERR_CAPT[ERR_POS] = ERC_POS_ACK, ack error feature test. 
--
-- @Verifies:
--  @1. Detection of ACK error in ACK slot (when dominant ACK was not send).
--      Value of ERR_CAPT[ERR_POS] when ACK error should have been detected in
--      ACK field.
--
-- @Test sequence:
--  @1. Check that ERR_CAPT contains no error (post reset).
--  @2. Configure Test node to ACK forbidden mode. Generate frame (use CAN 2.0
--      not to extend ACK field) and send it by DUT. Wait until ACK field and
--      monitor that it is recessive during whole duration of ACK field. Wait
--      until Sample point and check that after it, error frame is transmitted
--      by DUT. Check that ERR_CAPT register signals ACK Error in ACK field.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    12.01.2020   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package err_capt_ack_ack_ftest is
    procedure err_capt_ack_ack_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body err_capt_ack_ack_ftest is
    procedure err_capt_ack_ack_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;    

        variable frame_sent         :     boolean;
        
        variable err_capt           :     SW_error_capture;
        variable mode_2             :     SW_mode := SW_mode_rst_val;

    begin

        -----------------------------------------------------------------------
        -- @1. Check that ERR_CAPT contains no error (post reset).
        -----------------------------------------------------------------------
        info_m("Step 1");
        
        CAN_read_error_code_capture(err_capt, DUT_NODE, chn);
        check_m(err_capt.err_pos = err_pos_other, "Reset of ERR_CAPT!");
        
        -----------------------------------------------------------------------        
        -- @2. Configure Test node to ACK forbidden mode. Generate frame (use CAN
        --     2.0 not to extend ACK field) and send it by DUT. Wait until
        --     ACK field and monitor that it is recessive during whole duration
        --     of ACK field. Wait until Sample point and check that after it,
        --     error frame is transmitted by DUT. Check that ERR_CAPT
        --     register signals ACK Error in ACK field.
        -----------------------------------------------------------------------
        info_m("Step 2");
        
        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, TEST_NODE, chn);

        CAN_generate_frame(frame_1);
        frame_1.frame_format := NORMAL_CAN;
        CAN_send_frame(frame_1, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_pc_state(pc_deb_ack, DUT_NODE, chn);

        CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
        while (pc_dbg = pc_deb_ack) loop
            check_bus_level(RECESSIVE, "Recessive ACK received in DUT!", chn);
            CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
        end loop;

        -- Now state has changed, we should be in Error frame because ACK was
        -- recessive
        get_controller_status(stat_1, DUT_NODE, chn);
        check_m(stat_1.error_transmission, "Error frame transmitted by DUT!");

        CAN_read_error_code_capture(err_capt, DUT_NODE, chn);
        check_m(err_capt.err_type = can_err_ack, "ACK error detected!");
        check_m(err_capt.err_pos = err_pos_ack, "Error detected in ACK field!");
                
        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);

  end procedure;

end package body;
