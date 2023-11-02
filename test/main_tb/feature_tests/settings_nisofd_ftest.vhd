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
--
--
-- @Verifies:
--  @1. When SETTINGS[NISOFD]=1, then Stuff count field is not transmitted in
--      the CAN FD Frame.
--
-- @Test sequence:
--  @1. Configure SETTINGS[NISOFD]=1 in DUT. Send CAN FD frame by DUT.
--  @2. Wait until the data field in DUT. Wait until not in the data field.
--      Check that DUT is NOT in CRC field.
--  @3. Set SETTINGS[NISOFD]=0 in DUT. Send CAN FD frame by DUT.
--  @4. Wait until the data field in DUT. Wait until not in the data field.
--      Check that DUT is NOT in Stuff count field.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    2.11.2023   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.mem_bus_agent_pkg.all;

package settings_nisofd_ftest is
    procedure settings_nisofd_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body settings_nisofd_ftest is
    procedure settings_nisofd_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_TX_frame       :       SW_CAN_frame_type;
        variable CAN_RX_frame       :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;

        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        variable txt_buf_state      :       SW_TXT_Buffer_state_type;
        variable rx_buf_state       :       SW_RX_Buffer_info;
        variable status             :       SW_status;
        variable frames_equal       :       boolean := false;
        variable pc_dbg             :       SW_PC_Debug;
        variable fault_state        :       SW_fault_state;

        variable err_counters       :       SW_error_counters;
        variable buf_index          :       natural;

        variable command            :       SW_command := SW_command_rst_val;
    begin

        -----------------------------------------------------------------------
        -- @1. Configure SETTINGS[NISOFD]=1 in DUT. Send CAN FD frame by DUT.
        -----------------------------------------------------------------------
        info_m("Step 1");

        mode_1.iso_fd_support := false;
        set_core_mode(mode_1, DUT_NODE, chn);

        mode_2.iso_fd_support := false;
        set_core_mode(mode_2, TEST_NODE, chn);

        CAN_generate_frame(CAN_TX_frame);
        CAN_TX_Frame.frame_format := FD_CAN;
        CAN_TX_Frame.data_length := 4;
        decode_length(CAN_TX_Frame.data_length, CAN_TX_Frame.dlc);

        CAN_send_frame(CAN_TX_frame, 1, DUT_NODE, chn, frame_sent);

        -----------------------------------------------------------------------
        -- @2. Wait until the data field in DUT. Wait until not in the data
        --     field. Check that DUT is NOT in CRC field.
        -----------------------------------------------------------------------
        info_m("Step 2");

        CAN_wait_pc_state(pc_deb_data, DUT_NODE, chn);
        CAN_wait_not_pc_state(pc_deb_data, DUT_NODE, chn);

        CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
        check_m(pc_dbg = pc_deb_crc, "Protocol control in CRC");

        CAN_wait_bus_idle(DUT_NODE, chn);

        -----------------------------------------------------------------------
        --  @3. Set SETTINGS[NISOFD]=0 in DUT. Send CAN FD frame by DUT.
        -----------------------------------------------------------------------
        info_m("Step 3");

        mode_1.iso_fd_support := true;
        set_core_mode(mode_1, DUT_NODE, chn);

        mode_2.iso_fd_support := true;
        set_core_mode(mode_2, TEST_NODE, chn);

        CAN_send_frame(CAN_TX_frame, 1, DUT_NODE, chn, frame_sent);

        -----------------------------------------------------------------------
        --  @4. Wait until the data field in DUT. Wait until not in the data
        --      field. Check that DUT is NOT in Stuff count field.
        -----------------------------------------------------------------------
        CAN_wait_pc_state(pc_deb_data, DUT_NODE, chn);
        CAN_wait_not_pc_state(pc_deb_data, DUT_NODE, chn);

        CAN_read_pc_debug_m(pc_dbg, DUT_NODE, chn);
        check_m(pc_dbg = pc_deb_stuff_count, "Protocol Control in Stuff Count");

        CAN_wait_bus_idle(DUT_NODE, chn);

  end procedure;

end package body;