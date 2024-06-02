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
--  @1. Loop through all bits of a frame:
--      @1.1  Set DUT node to Error Passive.
--      @1.2 Send a Frame by DUT node. Wait for incrementing number of bits
--      @1.3 Flip a bit on DUT CAN RX.
--      @1.4 Check that DUT is either transmitting an error frame, or it has
--           lost arbitration.
--      @1.5 Wait until bus is idle.
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

package pc_fsm_transitions_ftest is
    procedure pc_fsm_transitions_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body pc_fsm_transitions_ftest is

    procedure pc_fsm_transitions_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable r_data             :       std_logic_vector(31 downto 0) := (OTHERS => '0');
        variable CAN_TX_frame       :       SW_CAN_frame_type;
        variable tx_val             :       std_logic;
        variable err_counters       :       SW_error_counters;
        variable status             :       SW_status;
        variable mode               :       SW_mode := SW_mode_rst_val;
    begin

        -----------------------------------------------------------------------
        -- @1. Loop through all bits of a frame:
        -----------------------------------------------------------------------
        info_m("Step 1: Loop through all bits of a frame:");

        -- Enable test mode to be able to modify REC and TEC counters
        mode.test := true;
        set_core_mode(mode, DUT_NODE, chn);

        CAN_generate_frame(CAN_TX_frame);
        CAN_TX_frame.identifier := 0;
        CAN_TX_frame.frame_format := FD_CAN;
        CAN_TX_frame.ident_type := BASE;
        CAN_TX_frame.data_length := 1;
        decode_length(CAN_TX_frame.data_length, CAN_TX_frame.dlc);

        -- Precomputed length of frame
        for i in 0 to 57 loop

            -----------------------------------------------------------------------
            -- @1.1  Set DUT node to Error Passive.
            -----------------------------------------------------------------------
            info_m("Step 1.1: Set DUT node to Error Passive.");

            err_counters.rx_counter := 130;
            set_error_counters(err_counters, DUT_NODE, chn);

            -----------------------------------------------------------------------
            -- @1.2 Send a Frame by DUT node. Wait for incrementing number of bits
            -----------------------------------------------------------------------
            info_m("Step 1.2: Set DUT node to Error Passive.");

            CAN_insert_TX_frame(CAN_TX_frame, 1, DUT_NODE, chn);
            send_TXT_buf_cmd(buf_set_ready, 1, DUT_NODE, chn);

            CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);

            for j in 0 to i loop
                CAN_wait_sync_seg(DUT_NODE, chn);
            end loop;

            wait for 20 ns;

            -----------------------------------------------------------------------
            -- @1.3 Flip a bit on DUT CAN RX.
            -----------------------------------------------------------------------
            info_m("Step 1.3 Flip a bit on DUT CAN RX.");

            get_can_tx(DUT_NODE, tx_val, chn);
            force_can_rx(not tx_val, DUT_NODE, chn);

            CAN_wait_sync_seg(DUT_NODE, chn);

            release_can_rx(chn);

            CAN_wait_sync_seg(DUT_NODE, chn);

            -----------------------------------------------------------------------
            -- @1.4 Check that DUT is either transmitting an error frame, or it has
            --      lost arbitration.
            -----------------------------------------------------------------------
            info_m("Step 1.4 Check error frame or arbitration lost");

            get_controller_status(status, DUT_NODE, chn);

            check_m(status.receiver or status.error_transmission,
                    "DUT either lost arbitration or is transmitting error frame");

            -----------------------------------------------------------------------
            -- @1.5 Wait until bus is idle.
            -----------------------------------------------------------------------
            info_m("Step 1.5 Wait until bus is idle.");

            CAN_wait_bus_idle(DUT_NODE, chn);

        end loop;

  end procedure;

end package body;
