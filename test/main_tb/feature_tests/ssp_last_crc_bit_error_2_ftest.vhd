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
--      FD uses Secondary sampling point, CTU CAN FD ignores such error when
--      secondary sample point for the last bit of CRC is at the regular
--      sample point of CRC delimiter. See Figure 25 in ISO11898-1 2015.
--
-- @Test sequence:
--  @1. Set DUT to Test mode (to be able to modify REC and TEC). Set DUT TX
--      to RX delay to one data bit time. Configure DUT Node to have SSP enabled
--      with "Measured + offset". Configure the offset in such way that it is
--      equal to regular sample point. Thus DUT is configured to sample at
--      the sample place, only with "secondary" sample point.
--  @2. Generate CAN FD frame with bit-rate shift. Send a Frame by DUT node.
--  @3. Wait until CRC delimiter and flip bus level. At this time, the DUT Node
--      shall be received last bit of CRC due to 1 bit transmitter delay.
--      Wait until start of next bit and release bus level!
--  @4. Check that DUT Node is not transmitting an error frame
--      (Local disturbance to be sampled in the moment of bit-rate switch was
--       ignored). Wait until bus is idle.
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

package ssp_last_crc_bit_error_2_ftest is
    procedure ssp_last_crc_bit_error_2_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body ssp_last_crc_bit_error_2_ftest is

    procedure ssp_last_crc_bit_error_2_ftest_exec(
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
        -- @1. Set DUT to Test mode (to be able to modify REC and TEC).
        --     Set DUT TX
        --     to RX delay to one data bit time. Configure DUT Node to have SSP enabled with
        --     "Measured + offset". Configure the offset in such way that it is
        --     equal to regular sample point. Thus DUT is configured to sample at
        --     the sample place, only with "secondary" sample point.
        -------------------------------------------------------------------------------------------
        info_m("Step 1: Configure DUT");
        mode.test := true;
        set_core_mode(mode, DUT_NODE, chn);

        -- (1 + 3 + 3 + 3) * 4 = 40 * 10 ns = 400 ns
        ftr_tb_set_tran_delay(400 ns, DUT_NODE, chn);

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
        -- 0x1B = 27 = (1 + 3 + 3) * 4 - 1
        -- -1 is to align with "regular" sample point based on simulation!
        CAN_configure_ssp(ssp_meas_n_offset, x"1B", DUT_NODE, chn);
        CAN_configure_ssp(ssp_meas_n_offset, x"1B", TEST_NODE, chn);

        CAN_turn_controller(true, DUT_NODE, chn);
        CAN_turn_controller(true, TEST_NODE, chn);

        CAN_wait_bus_on(DUT_NODE, chn);
        CAN_wait_bus_on(TEST_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @2. Generate CAN FD frame with bit-rate shift. Send a Frame by DUT node.
        -------------------------------------------------------------------------------------------
        info_m("Step 2");

        CAN_generate_frame(CAN_TX_frame);
        CAN_TX_frame.identifier := 0;
        CAN_TX_frame.frame_format := FD_CAN;
        CAN_TX_frame.data_length := 1;
        CAN_TX_frame.brs := BR_SHIFT;

        CAN_insert_TX_frame(CAN_TX_frame, 1, DUT_NODE, chn);
        send_TXT_buf_cmd(buf_set_ready, 1, DUT_NODE, chn);

        -------------------------------------------------------------------------------------------
        -- @3. Wait until CRC delimiter and flip bus level. At this time, the DUT Node
        --     shall be received last bit of CRC due to 1 bit transmitter delay.
        --     Wait until start of next bit and release bus level!
        -------------------------------------------------------------------------------------------
        info_m("Step 3");

        CAN_wait_pc_state(pc_deb_crc_delim, DUT_NODE, chn);
        CAN_wait_sync_seg(DUT_NODE, chn);

        flip_bus_level(chn);
        CAN_wait_sync_seg(DUT_NODE, chn);
        release_bus_level(chn);

        -------------------------------------------------------------------------------------------
        -- @4. Check that DUT Node is not transmitting an error frame
        --     (Local disturbance to be sampled in the moment of bit-rate switch was ignored).
        --      Wait until bus is idle.
        -------------------------------------------------------------------------------------------
        info_m("Step 4");

        get_controller_status(status, DUT_NODE, chn);

        check_false_m(status.error_transmission, "DUT is NOT transmitting error frame");
        CAN_wait_bus_idle(DUT_NODE, chn);

  end procedure;

end package body;
