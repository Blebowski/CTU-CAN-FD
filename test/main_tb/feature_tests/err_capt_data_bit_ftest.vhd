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
--  ERR_CAPT[ERR_POS] = ERC_POS_DATA feature test - bit error.
--
-- @Verifies:
--  @1. Detection of bit error in Data field.
--  @2. Value of ERR_CAPT when bit error is detected in Data field.
--
-- @Test sequence:
--  @1. Check that ERR_CAPT contains no error (post reset).
--  @2. Generate CAN frame, send it by DUT. Wait until data field. Wait for
--      random duration of data field. Force bus to opposite value as transmitted
--      bit wait until sample point. Check that error frame is being transmitted.
--      Check that ERR_CAPT signals bit error in data field!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    03.02.2020   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package err_capt_data_bit_ftest is
    procedure err_capt_data_bit_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body err_capt_data_bit_ftest is
    procedure err_capt_data_bit_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;
        
        variable frame_sent         :     boolean;
        
        variable err_capt           :     SW_error_capture;
        variable tmp                :     natural;
        
        variable tx_bus_level       :     std_logic;
    begin

        -----------------------------------------------------------------------
        -- @1. Check that ERR_CAPT contains no error (post reset).
        -----------------------------------------------------------------------
        info_m("Step 1");

        CAN_read_error_code_capture(err_capt, DUT_NODE, chn);
        check_m(err_capt.err_pos = err_pos_other, "Reset of ERR_CAPT!");

        -----------------------------------------------------------------------
        -- @2. Generate CAN frame, send it by DUT. Wait until data field.
        --    Wait for random duration of data field. Force bus to opposite
        --    value as transmitted bit wait until sample point. Check that
        --    error frame is being transmitted. Check that ERR_CAPT signals bit
        --    error in data field!
        -----------------------------------------------------------------------
        info_m("Step 2");

        CAN_generate_frame(frame_1);
        frame_1.rtr := NO_RTR_FRAME;
        
        -- Don't sample by SSP!
        frame_1.brs := BR_NO_SHIFT;
        
        if (frame_1.data_length = 0) then
            frame_1.data_length := 1;
            decode_length(frame_1.data_length, frame_1.dlc);
        end if;
        
        CAN_send_frame(frame_1, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);
        CAN_wait_pc_state(pc_deb_data, DUT_NODE, chn);

        -- Wait for random number of bits within data field
        rand_int_v((frame_1.data_length * 8) - 1, tmp);
        info_m("Waiting for: " & integer'image(tmp) & " bits!");
        for i in 0 to tmp - 1 loop
            CAN_wait_sample_point(DUT_NODE, chn);
        end loop;

        CAN_wait_sync_seg(DUT_NODE, chn);
        wait for 20 ns;

        get_can_tx(DUT_NODE, tx_bus_level, chn);
        force_bus_level(not tx_bus_level, chn);
        CAN_wait_sample_point(DUT_NODE, chn);
        wait for 20 ns; -- To be sure that opposite bit is sampled!
        release_bus_level(chn);
        
        get_controller_status(stat_1, DUT_NODE, chn);
        check_m(stat_1.error_transmission, "Error frame is being transmitted!");
        
        CAN_read_error_code_capture(err_capt, DUT_NODE, chn);
        check_m(err_capt.err_type = can_err_bit, "Bit error detected!");
        check_m(err_capt.err_pos = err_pos_data, "Error detected in Data field!");
        
        CAN_wait_bus_idle(DUT_NODE, chn);

  end procedure;

end package body;
