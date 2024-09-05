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
--  @1. When bit error occurs in the last bit of CRC in Data phase and CTU CAN
--      FD uses Secondary sampling point, CTU CAN FD is able to detect error
--      when SSP for the errornous last CRC bit occurs before regular sample
--      point of CRC Delimiter (before switch to nominal bit rate and cut-off
--      of Secondary Sample point sequence).
--
-- @Test sequence:
--  @1. Set DUT to Test mode (to be able to modify REC and TEC). Set DUT TX
--      to RX delay to 1 ns. Configure DUT Node to have SSP enabled with
--      "Measured + offset". Configure the offset in such way that it is
--      equal to regular sample point. Thus DUT is configured to sample at
--      the sample place, only with "secondary" sample point.
--  @2. Generate CAN FD frame with bit-rate shift. Loop through all bits of a frame:
--      @2.1 Set DUT node to Error Active.
--      @2.2 Send a Frame by DUT node. Wait for incrementing number of bits
--      @2.3 Flip a bit on DUT CAN RX.
--      @2.4 Check that DUT is either transmitting an error frame, or it has
--             lost arbitration.
--      @2.5 Wait until bus is idle.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    5.9.2024   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package ssp_last_crc_bit_error_ftest is
    procedure ssp_last_crc_bit_error_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body ssp_last_crc_bit_error_ftest is

    procedure ssp_last_crc_bit_error_ftest_exec(
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
        variable bit_timing         :       bit_time_config_type;
    begin

        -------------------------------------------------------------------------------------------
        -- @1. Set DUT to Test mode (to be able to modify REC and TEC). Set DUT TX
        --     to RX delay to 1 ns. Configure DUT Node to have SSP enabled with
        --     "Measured + offset". Configure the offset in such way that it is
        --     equal to regular sample point. Thus DUT is configured to sample at
        --     the sample place, only with "secondary" sample point.
        -------------------------------------------------------------------------------------------
        info_m("Step 1: Configure DUT");
        mode.test := true;
        set_core_mode(mode, DUT_NODE, chn);

        ftr_tb_set_tran_delay(1 ns, DUT_NODE, chn);

        -- Reconfigure bit-rate and SSP so that we are sure that SSP position is configured on the
        -- sample place as regular sample point position!
        bit_timing.tq_nbt     := 10;
        bit_timing.tq_dbt     := 4;

        bit_timing.prop_nbt   := 4;
        bit_timing.ph1_nbt    := 3;
        bit_timing.ph2_nbt    := 4;
        bit_timing.sjw_nbt    := 1;

        -- Equal to reset values of BTR_FD
        bit_timing.prop_dbt   := 3;
        bit_timing.ph1_dbt    := 3;
        bit_timing.ph2_dbt    := 3;
        bit_timing.sjw_dbt    := 1;

        CAN_turn_controller(false, DUT_NODE, chn);
        CAN_turn_controller(false, TEST_NODE, chn);

        CAN_configure_timing(bit_timing, DUT_NODE, chn);
        CAN_configure_timing(bit_timing, TEST_NODE, chn);

        -- Configure SSP with measured and offset.
        -- The SSP delay of 10 corresponds to 7 regular SP delay + synchronization
        -- of CAN RX input
        CAN_configure_ssp(ssp_meas_n_offset, x"0A", DUT_NODE, chn);
        CAN_configure_ssp(ssp_meas_n_offset, x"0A", TEST_NODE, chn);

        CAN_turn_controller(true, DUT_NODE, chn);
        CAN_turn_controller(true, TEST_NODE, chn);

        CAN_wait_bus_on(DUT_NODE, chn);
        CAN_wait_bus_on(TEST_NODE, chn);


        -------------------------------------------------------------------------------------------
        -- @2. Generate CAN FD frame with bit-rate shift. Loop through all bits of a frame:
        -------------------------------------------------------------------------------------------
        info_m("Step 2");

        CAN_generate_frame(CAN_TX_frame);
        CAN_TX_frame.identifier := 0;
        CAN_TX_frame.frame_format := FD_CAN;
        CAN_TX_frame.data_length := 1;
        CAN_TX_frame.brs := BR_SHIFT;

        bit_index := 0;
        bit_iter_loop: loop

            ---------------------------------------------------------------------------------------
            -- @2.1 Set DUT node to Error Active.
            ---------------------------------------------------------------------------------------
            info_m("Step 2.1: Set DUT node to Error Active.");

            err_counters.rx_counter := 0;
            set_error_counters(err_counters, DUT_NODE, chn);

            ---------------------------------------------------------------------------------------
            -- @2.2 Send a Frame by DUT node. Wait for incrementing number of bits
            ---------------------------------------------------------------------------------------
            info_m("Step 2.2: Send a Frame by DUT node. Wait for incrementing number of bits");

            CAN_insert_TX_frame(CAN_TX_frame, 1, DUT_NODE, chn);
            send_TXT_buf_cmd(buf_set_ready, 1, DUT_NODE, chn);

            CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);

            info_m("Waiting for " & integer'image(bit_index) & " bits!");
            for j in 0 to bit_index loop
                CAN_wait_sync_seg(DUT_NODE, chn);
            end loop;

            wait for 20 ns;

            -- If we get up to CRC Delim, we finish, flipping CRC Delimt will not
            -- result in Error frame.
            CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
            if (pc_dbg = pc_deb_crc_delim) then
                CAN_wait_bus_idle(DUT_NODE, chn);
                CAN_wait_bus_idle(TEST_NODE, chn);
                exit bit_iter_loop;
            end if;

            -----------------------------------------------------------------------
            -- @2.3 Flip a bit on DUT CAN RX.
            -----------------------------------------------------------------------
            info_m("Step 2.3 Flip a bit on DUT CAN RX.");

            flip_bus_level(chn);
            CAN_wait_sync_seg(DUT_NODE, chn);
            release_bus_level(chn);

            CAN_wait_sync_seg(DUT_NODE, chn);

            -----------------------------------------------------------------------
            -- @2.4 Check that DUT is either transmitting an error frame, or it
            --      has lost arbitration.
            -----------------------------------------------------------------------
            info_m("Step 2.4 Check error frame or arbitration lost");

            get_controller_status(status, DUT_NODE, chn);

            check_m(status.receiver or status.error_transmission,
                    "DUT either lost arbitration or is transmitting error frame");

            -----------------------------------------------------------------------
            -- @2.5 Wait until bus is idle.
            -----------------------------------------------------------------------
            info_m("Step 2.5 Wait until bus is idle.");

            CAN_wait_bus_idle(DUT_NODE, chn);

            bit_index := bit_index + 1;

        end loop;

  end procedure;

end package body;
