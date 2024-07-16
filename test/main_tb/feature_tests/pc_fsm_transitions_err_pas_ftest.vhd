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
--  Corner-case PC FSM transitions
--
-- @Verifies:
--  @1. Corner-case transitions of Protocol Control FSM to Passive Error Flag.
--
-- @Test sequence:
--  @1. Set DUT to test mode (to be able to modify REC and TEC)
--  @2. Loop through all combinations of frames:
--          CAN 2.0 Base, CAN 2.0 Extended, CAN FD Based, CAN FD Extended.
--      @2.1. Loop through all bits of a frame:
--          @2.1.1 Set DUT node to Error Passive.
--          @2.2.1 Send a Frame by DUT node. Wait for incrementing number of bits
--          @2.3.1 Flip a bit on DUT CAN RX.
--          @2.4.1 Check that DUT is either transmitting an error frame, or it has
--                 lost arbitration.
--          @2.5.1 Wait until bus is idle.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    14.12.2023   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package pc_fsm_transitions_err_pas_ftest is
    procedure pc_fsm_transitions_err_pas_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body pc_fsm_transitions_err_pas_ftest is

    procedure pc_fsm_transitions_err_pas_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable r_data             :       std_logic_vector(31 downto 0) := (OTHERS => '0');
        variable CAN_TX_frame       :       SW_CAN_frame_type;
        variable tx_val             :       std_logic;
        variable err_counters       :       SW_error_counters;
        variable status             :       SW_status;
        variable mode               :       SW_mode := SW_mode_rst_val;
        variable frame_bits         :       integer;
        variable bit_index          :       integer;
        variable pc_dbg             :       SW_PC_Debug;
    begin

        -------------------------------------------------------------------------------------------
        -- @1. Set DUT to test mode (to be able to modify REC and TEC)
        -------------------------------------------------------------------------------------------
        info_m("Step 1: Set DUT to test mode");
        mode.test := true;
        set_core_mode(mode, DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @2. Loop through all combinations of frames:
        --     CAN 2.0 Base, CAN 2.0 Extended, CAN FD Based, CAN FD Extended.
        -------------------------------------------------------------------------------------------
        info_m("Step 1: Loop through all combinations of frames:");
        info_m("        CAN 2.0 Base, CAN 2.0 Extended, CAN FD Based, CAN FD Extended.");

        for frame_format in NORMAL_CAN to FD_CAN loop
        for ident_type in BASE to EXTENDED loop

            CAN_generate_frame(CAN_TX_frame);
            CAN_TX_frame.identifier := 0;
            CAN_TX_frame.frame_format := frame_format;
            CAN_TX_frame.ident_type := ident_type;
            CAN_TX_frame.data_length := 1;
            CAN_TX_frame.rtr := NO_RTR_FRAME;

            bit_index := 0;
            bit_iter_loop: loop

                -----------------------------------------------------------------------
                -- @2.1.1  Set DUT node to Error Passive.
                -----------------------------------------------------------------------
                info_m("Step 2.1.1: Set DUT node to Error Passive.");

                err_counters.rx_counter := 180;
                set_error_counters(err_counters, DUT_NODE, chn);

                -----------------------------------------------------------------------
                -- @2.1.2 Send a Frame by DUT node. Wait for incrementing number of bits
                -----------------------------------------------------------------------
                info_m("Step 2.1.2: Send a Frame by DUT node. Wait for incrementing number of bits");

                info_m("Identifier type: " & std_logic'image(frame_format));
                info_m("Frame format: " & std_logic'image(ident_type));

                CAN_insert_TX_frame(CAN_TX_frame, 1, DUT_NODE, chn);
                send_TXT_buf_cmd(buf_set_ready, 1, DUT_NODE, chn);

                CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);

                info_m("Waiting for " & integer'image(bit_index) & " bits!");
                for j in 0 to bit_index loop
                    CAN_wait_sync_seg(DUT_NODE, chn);
                end loop;

                wait for 20 ns;

                -- If we get up to ACK, we finish, flipping ACK will not result in
                -- immediate Error frame!
                CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
                if (pc_dbg = pc_deb_ack) then
                    CAN_wait_bus_idle(DUT_NODE, chn);
                    CAN_wait_bus_idle(TEST_NODE, chn);
                    exit bit_iter_loop;
                end if;

                -----------------------------------------------------------------------
                -- @2.1.3 Flip a bit on DUT CAN RX.
                -----------------------------------------------------------------------
                info_m("Step 2.1.3 Flip a bit on DUT CAN RX.");

                flip_bus_level(chn);
                CAN_wait_sync_seg(DUT_NODE, chn);
                release_bus_level(chn);

                CAN_wait_sync_seg(DUT_NODE, chn);

                -----------------------------------------------------------------------
                -- @2.1.4 Check that DUT is either transmitting an error frame, or it
                --        has lost arbitration.
                -----------------------------------------------------------------------
                info_m("Step 2.1.4 Check error frame or arbitration lost");

                get_controller_status(status, DUT_NODE, chn);

                check_m(status.receiver or status.error_transmission,
                        "DUT either lost arbitration or is transmitting error frame");

                -----------------------------------------------------------------------
                -- @2.1.5 Wait until bus is idle.
                -----------------------------------------------------------------------
                info_m("Step 2.1.5 Wait until bus is idle.");

                CAN_wait_bus_idle(DUT_NODE, chn);

                bit_index := bit_index + 1;

            end loop;

        end loop;
        end loop;

  end procedure;

end package body;
