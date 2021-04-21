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
--  ERR_CAPT[ERR_POS] = ERC_POS_CRC feature test - bit error.
--
-- @Verifies:
--  @1. Detection of bit error in CRC field.
--  @2. Value of ERR_CAPT when bit error is detected in CRC field.
--
-- @Test sequence:
--  @1. Check that ERR_CAPT contains no error (post reset).
--  @2. Generate CAN frame, send it by DUT. Wait until CRC field. Wait for
--      random duration of CRC field. Force bus to opposite value as transmitted
--      bit wait until sample point. Check that error frame is being transmitted.
--      Check that ERR_CAPT signals bit error in CRC field!
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

package err_capt_crc_bit_ftest is
    procedure err_capt_crc_bit_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body err_capt_crc_bit_ftest is
    procedure err_capt_crc_bit_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;

        variable frame_sent         :     boolean;
        
        variable err_capt           :     SW_error_capture;
        variable tmp                :     natural;
        variable crc_len            :     natural;
        
        variable can_tx_val         :     std_logic;
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
        
        CAN_send_frame(frame_1, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);
        
        if (frame_1.frame_format = FD_CAN) then
            CAN_wait_pc_state(pc_deb_stuff_count, DUT_NODE, chn);
        else
            CAN_wait_pc_state(pc_deb_crc, DUT_NODE, chn);
        end if;

        -- Wait for random number of bits
        if (frame_1.frame_format = FD_CAN) then
            if (frame_1.data_length > 16) then
                crc_len := 24; -- CRC21 + Stuff count + Parity - 1
            else
                crc_len := 20; -- CRC17 + Stuff count + Parity - 1
            end if;
        else
            crc_len := 15; -- CRC 15 (CAN 2.0 frames have no Stuff count)!
        end if;

        -- Wait for Random amount of bits, but not longer than CRC field!
        rand_int_v(crc_len - 1, tmp);
        info_m("Waiting for: " & integer'image(tmp) & " bits!");
        for i in 1 to tmp loop
            CAN_wait_sample_point(DUT_NODE, chn);
        end loop;

        CAN_wait_sync_seg(DUT_NODE, chn);
        wait for 20 ns;

        get_can_tx(DUT_NODE, can_tx_val, chn);
        force_bus_level(not can_tx_val, chn);
        CAN_wait_sample_point(DUT_NODE, chn);
        wait for 20 ns; -- To be sure that opposite bit is sampled!
        release_bus_level(chn);
        
        get_controller_status(stat_1, DUT_NODE, chn);
        check_m(stat_1.error_transmission, "Error frame is being transmitted!");
        
        CAN_read_error_code_capture(err_capt, DUT_NODE, chn);
        
        -----------------------------------------------------------------------
        -- It might happend that we corrupt a bit which is just stuff bit (due
        -- to randomization). If this is a CAN FD frame, this will be considered
        -- as form error (stuff error during fixed bit stuffing shall be reported
        -- as form error). In such case, form error has higher priority than
        -- stuff error, so form error is reported. Tolerate this as this is not
        -- a bug!
        -----------------------------------------------------------------------
        if (frame_1.frame_format = FD_CAN) then
            check_m(err_capt.err_type = can_err_bit or
                  err_capt.err_type = can_err_form, "Bit or Form error detected!");
        else
            check_m(err_capt.err_type = can_err_bit, "Bit error detected!");
        end if;
            
        check_m(err_capt.err_pos = err_pos_crc, "Error detected in CRC field!");
        
        CAN_wait_bus_idle(DUT_NODE, chn);

  end procedure;

end package body;
