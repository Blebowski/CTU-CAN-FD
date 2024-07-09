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
--  ERR_CAPT CRC error feature test.
--
-- @Verifies:
--  @1. Detection of CRC error when calculated CRC is not equal to received
--      CRC. Value of ERR_CAPT when CRC error should have been detected in
--      ACK field.
--
-- @Test sequence:
--  @1. Check that ERR_CAPT contains no error (post reset).
--  @2. Generate CAN frame and send it by Test node. Wait until CRC field in DUT
--      and wait for random number of bits. Force CAN_RX of DUT for duration
--      of 1 bit to opposite value (to mess up received CRC of Test node). Wait
--      until ACK bit in Test node. Force bus low (as if other node is ACKing the
--      frame) and wait until sample point. Wait until ACK delimiter of DUT.
--      Wait until DUT is NOT in ACK delimiter and check that it transmitts
--      Error frame. Check that ERR_CAPT contains CRC Error.
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

package err_capt_crc_err_ftest is
    procedure err_capt_crc_err_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body err_capt_crc_err_ftest is
    procedure err_capt_crc_err_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;
        variable stat_2             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;

        variable frame_sent         :     boolean;

        variable err_capt           :     SW_error_capture;
        variable mode_2             :     SW_mode := SW_mode_rst_val;
        variable wait_time          :     natural;
        variable can_rx_val         :     std_logic;

        variable pc_fsm_state       :     SW_PC_Debug;
    begin

        -----------------------------------------------------------------------
        -- @1. Check that ERR_CAPT contains no error (post reset).
        -----------------------------------------------------------------------
        info_m("Step 1");

        CAN_read_error_code_capture(err_capt, TEST_NODE, chn);
        check_m(err_capt.err_pos = err_pos_other, "Reset of ERR_CAPT!");

        -----------------------------------------------------------------------
        -- @2. Generate CAN frame and send it by Test Node.
        --     Wait until CRC field in DUT and wait for random number of bits.
        --     Force CAN_RX of DUT for duration of 1 bit to opposite value (to
        --     mess up received CRC of Test node).
        --     Wait until ACK bit in Test Node.
        --     Force bus low (as if other node is ACKing the frame) and wait
        --     until sample point.
        --     Wait until ACK delimiter of DUT. Wait until DUT is
        --     NOT in ACK delimiter and check that it transmitts Error frame.
        --     Check that ERR_CAPT contains CRC Error.
        -----------------------------------------------------------------------
        info_m("Step 2");

        CAN_generate_frame(frame_1);
        frame_1.frame_format := NORMAL_CAN; --Use CAN 2.0 to have single bit ACK
        frame_1.identifier := 67;
        frame_1.dlc := "0001";
        frame_1.data_length := 1;
        frame_1.data(0) :=  x"55";
        frame_1.rtr := NO_RTR_FRAME;
        CAN_send_frame(frame_1, 1, TEST_NODE, chn, frame_sent);

        CAN_wait_pc_state(pc_deb_crc, DUT_NODE, chn);
        rand_int_v(12, wait_time);

        info_m("waiting for:" & integer'image(wait_time) & " bits!");
        wait_time := wait_time + 1;
        for i in 1 to wait_time loop
            CAN_wait_sample_point(DUT_NODE, chn);
        end loop;
        CAN_wait_sync_seg(DUT_NODE, chn);
        wait for 100 ns; -- To be sure we are ssuficiently far in the bit!

        -- Force can_rx of Test node to oposite value!
        get_can_rx(DUT_NODE, can_rx_val, chn);
        force_can_rx(not can_rx_val, DUT_NODE, chn);
        CAN_wait_sample_point(DUT_NODE, chn);
        wait for 20 ns;
        release_can_rx(chn);

        -------------------------------------------------------------------------------------------
        -- Following scenarios can occur:
        --   1. We flipped Stuff bit in CRC -> Error frame will occur immediately!
        --   2. We flipped a bit that will cause stuff rule violation later -> Error frame will
        --      come later within the CRC field.
        --   3. We flipped non-stuff bit in CRC for DUT -> CRC Error.
        --      DUT will transmit recessive ACK, and Error frame after ACK delimiter!
        -------------------------------------------------------------------------------------------

        CAN_wait_not_pc_state(pc_deb_crc, DUT_NODE, chn);
        wait for 10 ns;
        CAN_read_pc_debug_m(pc_fsm_state, DUT_NODE, chn);

        -- No error frame occured within CRC field -> Expect CRC Error in DUT
        if (pc_fsm_state = pc_deb_crc_delim) then


            -- Generate ACK for Test node.
            -- We have to wait till both Nodes are in ACK so
            -- that we don't accidentaly generate it for Test node while it is still in
            -- in CRC Delimiter, that would be form error. Thus we would not ever test form Error!
            CAN_wait_pc_state(pc_deb_ack, TEST_NODE, chn);
            CAN_wait_pc_state(pc_deb_ack, DUT_NODE, chn);

            force_bus_level(DOMINANT, chn);
            check_can_tx(RECESSIVE, DUT_NODE, "Send Recessive ACK upon CRC error [1]!", chn);
            CAN_wait_sample_point(TEST_NODE, chn);
            check_can_tx(RECESSIVE, DUT_NODE, "Send Recessive ACK upon CRC error [2]!", chn);
            wait for 20 ns;
            release_bus_level(chn);

            -- Wait till End of ACK delimiter of Test node. Now DUT should have
            -- received ACK from TB, therefore it should not be transmitting
            -- error frame from ACK delimiter! But Test node should not have sent ACK
            -- and should have mismatching CRC, therefore after ACK delimiter,
            -- it should send Error frame!
            CAN_wait_pc_state(pc_deb_ack_delim, DUT_NODE, chn);
            CAN_wait_not_pc_state(pc_deb_ack_delim, DUT_NODE, chn);
            wait for 20 ns;

            -- Now state has changed, we should be in Error frame because ACK was
            -- recessive
            get_controller_status(stat_2, DUT_NODE, chn);
            check_m(stat_2.error_transmission, "Error frame transmitted by Test node!");

            CAN_read_error_code_capture(err_capt, DUT_NODE, chn);
            check_m(err_capt.err_type = can_err_crc, "CRC error detected!");
            check_m(err_capt.err_pos = err_pos_ack,
                "Error detected in CRC Delim/ACK/ACK Delim field!");

        -- Error frame occcured within CRC, it should be stuff error !
        else
            get_controller_status(stat_2, DUT_NODE, chn);
            check_m(stat_2.error_transmission, "Error frame transmitted by Test node!");

            CAN_read_error_code_capture(err_capt, DUT_NODE, chn);
            check_m(err_capt.err_type = can_err_stuff, "Stuff error detected!");
            check_m(err_capt.err_pos = err_pos_crc, "Error detected in CRC field!");
        end if;

        CAN_wait_bus_idle(TEST_NODE, chn);
        CAN_wait_bus_idle(DUT_NODE, chn);

  end procedure;

end package body;
