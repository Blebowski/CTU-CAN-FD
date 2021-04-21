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
--  REC (RX Error counter) saturation feature test.
--
-- @Verifies:
--  @1. REC is saturated at its width (9 bits) and further errors in receiver
--      will not cause its overflow.
--
-- @Test sequence:
--  @1. Set Test mode in DUT. Disable CAN FD support in DUT and Set REC to
--      510 in DUT. Check that REC is 510.
--  @2. Send CAN FD frame by Test node few times. Wait until CAN frame is sent 
--      andcheck that REC in DUT is 511 after first frame (was incremented by 1)
--      and also after each next attempt to transmitt a frame.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    11.11.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package rec_saturation_ftest is
    procedure rec_saturation_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body rec_saturation_ftest is
    procedure rec_saturation_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame          :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable err_counters       :       SW_error_counters := (0, 0, 0, 0);
    begin

        ------------------------------------------------------------------------
        -- @1. Set Test mode in DUT. Disable CAN FD support in DUT and Set
        --     REC to 510 in DUT. Check that REC is 510.
        ------------------------------------------------------------------------
        info_m("Step 1");
        
        mode_1.test := true;
        mode_1.flexible_data_rate := false;
        set_core_mode(mode_1, DUT_NODE, chn);

        err_counters.rx_counter := 510;
        set_error_counters(err_counters, DUT_NODE, chn);
        
        err_counters.rx_counter := 0;
        read_error_counters(err_counters, DUT_NODE, chn);
        
        check_m(err_counters.rx_counter = 510, "REC set properly!");

        ------------------------------------------------------------------------
        -- @2. Send CAN FD frame by Test node few times. Wait until CAN frame is
        --     sent and check that REC in DUT is 511 after first frame (was
        --     incremented by 1) and also after each next attempt to transmitt a
        --     frame.
        ------------------------------------------------------------------------
        info_m("Step 2");
        
        -- Set to one shot mode
        CAN_enable_retr_limit(true, 0, DUT_NODE, chn);

        CAN_generate_frame(CAN_frame);
        CAN_frame.frame_format := FD_CAN;

        for i in 0 to 3 loop
            CAN_send_frame(CAN_frame, 1, TEST_NODE, chn, frame_sent);
            CAN_wait_frame_sent(TEST_NODE, chn);
            
            CAN_wait_bus_idle(DUT_NODE, chn);
            CAN_wait_bus_idle(TEST_NODE, chn);

            read_error_counters(err_counters, DUT_NODE, chn);
            
            if (i = 0) then
                check_m(err_counters.rx_counter = 511, "REC incremented to 511!");
            else
                check_m(err_counters.rx_counter = 511, "REC remains saturated at 511!");
            end if;
        end loop;

    end procedure;

end package body;