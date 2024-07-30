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
--  Arbitration lost capture - IDE bit feature test.
--
-- @Verifies:
--  @1. CAN frame with base identifier only wins arbitration over CAN frame with
--      extended identifier when base identifier of both frames is equal.
--  @2. Arbitration lost capture position on IDE bit after Base identifier.
--
-- @Test sequence:
--  @1. Configure both Nodes to retransmit limit 1.
--  @2. Iterate through all TXT Buffers:
--      @2.1 Generate two CAN frames: Frame 1 with Extended identifier, Frame 2
--           with Base Identifier only, RTR frame. Base identifier of both CAN
--           frames is matching!
--      @2.2 Wait till sample point in DUT. Send Frame 1 by DUT and Frame 2 by
--           Test node.
--      @2.3 Wait till arbitration field in DUT. After first bit, send Set
--           Abort command. Then wait till sample point 12 times
--           (11 Base ID + RTR/SRR + IDE). Check DUT is transmitting recessive,
--           Check Test node is transmitting dominant. Check DUT lost arbitration.
--           Check Test node is still transmitter. Read ALC from DUT and check it.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    05.10.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package alc_ide_ftest is
    procedure alc_ide_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body alc_ide_ftest is
    procedure alc_ide_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable alc                :       natural;

        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;
        variable frame_2            :     SW_CAN_frame_type;
        variable frame_rx           :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;
        variable stat_2             :     SW_status;

        variable pc_dbg             :     SW_PC_Debug;

        variable txt_buf_state      :     SW_TXT_Buffer_state_type;
        variable rx_buf_info        :     SW_RX_Buffer_info;
        variable frames_equal       :     boolean := false;

        variable num_txt_bufs       :     natural;
        variable id_vect            :     std_logic_vector(28 downto 0);
    begin

        -----------------------------------------------------------------------
        -- @1. Configure both Nodes to retransmit limit 1.
        -----------------------------------------------------------------------
        info_m("Step 1: Configure retransmit limit to 1");
        CAN_enable_retr_limit(true, 1, DUT_NODE, chn);
        CAN_enable_retr_limit(true, 1, TEST_NODE, chn);

        -----------------------------------------------------------------------
        -- @2. Iterate through all TXT Buffers:
        -----------------------------------------------------------------------
        info_m("Step 2: Iterate through all TXT Buffers");

        get_tx_buf_count(num_txt_bufs, DUT_NODE, chn);

        for txt_buf_index in 1 to num_txt_bufs loop
            info_m("TXT Buffer: " & integer'image(txt_buf_index));

            -----------------------------------------------------------------------
            -- @2.1 Generate two CAN frames: Frame 1 with Extended identifier,
            --      Frame 2 with Base Identifier only, RTR frame. Base identifier of
            --      both CAN frames is matching!
            -----------------------------------------------------------------------
            info_m("Step 2: Generate CAN frames with matching IDs!");
            CAN_generate_frame(frame_1);
            CAN_generate_frame(frame_2);

            frame_1.ident_type := EXTENDED;
            frame_2.ident_type := BASE;
            frame_2.rtr := RTR_FRAME;
            frame_2.frame_format := NORMAL_CAN;
            frame_2.identifier := (frame_2.identifier mod 2**11);
            id_vect := std_logic_vector(to_unsigned(frame_2.identifier, 29));

            -- Shift base ID up for extended id to match Base ID of Test node!
            id_vect := id_vect(10 downto 0) & "000000000000000000";
            frame_1.identifier := to_integer(unsigned(id_vect));

            ------------------------------------------------------------------------
            -- @2.2 Wait till sample point in DUT. Send Frame 1 by DUT and
            --      Frame 2 by Test node.
            ------------------------------------------------------------------------
            info_m("Step 3: Send frames");
            CAN_insert_TX_frame(frame_1, txt_buf_index, DUT_NODE, chn);
            CAN_insert_TX_frame(frame_2, 1, TEST_NODE, chn);
            CAN_wait_sample_point(DUT_NODE, chn);

            send_TXT_buf_cmd(buf_set_ready, txt_buf_index, DUT_NODE, chn);
            send_TXT_buf_cmd(buf_set_ready, 1, TEST_NODE, chn);

            -----------------------------------------------------------------------
            -- @2.3 Wait till arbitration field in DUT. After first bit, send Set
            --      Abort command. Then wait till sample point 12 times Wait till
            --      sample point 12 times (11 Base ID + RTR/SRR + IDE). Check DUT is
            --      transmitting recessive, Check Test node is transmitting dominant.
            --      Check DUT lost arbitration. Check Test node is still transmitter.
            --      Read ALC from DUT and check it.
            -----------------------------------------------------------------------
            info_m("Step 4: Check arbitration lost on SRR/RTR");
            CAN_wait_pc_state(pc_deb_arbitration, DUT_NODE, chn);

            -- This is to get TXT Buffer into abort in progress to get full
            -- expression coverage on all sub-expressions in TXT Buffer FSM.
            -- Functionally, it does not matter if DUT loses arbitration when
            -- TXT Buffer is TX in Progress or Abort in Progress.
            CAN_wait_sample_point(DUT_NODE, chn);
            send_TXT_buf_cmd(buf_set_abort, txt_buf_index, DUT_NODE, chn);

            for i in 0 to 11 loop
                CAN_wait_sample_point(DUT_NODE, chn);
            end loop;
            check_can_tx(RECESSIVE, DUT_NODE, "Recessive IDE transmitted!", chn);
            check_can_tx(DOMINANT, TEST_NODE, "Dominant IDE transmitted!", chn);
            wait for 20 ns; -- To account for trigger processing

            get_controller_status(stat_2, TEST_NODE, chn);
            check_m(stat_2.transmitter, "Test node transmitting!");
            get_controller_status(stat_1, DUT_NODE, chn);
            check_m(stat_1.receiver, "DUT lost arbitration!");

            read_alc(alc, DUT_NODE, chn);
            check_m(alc = 13, "Arbitration lost at correct bit by DUT!");

            CAN_wait_bus_idle(DUT_NODE, chn);
            CAN_wait_bus_idle(TEST_NODE, chn);

        end loop;

  end procedure;

end package body;
