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
--  TXT Buffer backup mode 2 feature test.
--
-- @Verifies:
--  @1. When MODE[TBBM] = 1, and transmission from "original" TXT Buffer fails,
--      "backup" TXT Buffer ends up in "aborted".
--
-- @Test sequence:
--  @1. Set TXT Buffer backup mode in DUT. Set Test node to ACK forbidden mode.
--      Set DUT retransmit limit threshold to 0 (One shot mode).
--  @2. Loop 10 times:
--      @2.1 Generate random priorities of TXT Buffers and apply them in DUT.
--           Generate random index of TXT Buffer which is for sure "original"
--           TXT Buffer and insert random frame to it.
--      @2.2 Send set ready command to selected original TXT Buffer. Wait until
--           DUT starts transmission, and check that "original" TXT Buffer is in
--           TX in Progress and "backup" TXT Buffer is in "ready" state.
--      @2.3 Wait until transmission ends and bus is idle. Check that "original"
--           TXT Buffer is in "TX Failed" state and "backup" TXT buffer ended in
--           "Aborted" state.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    29.06.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package mode_txbbm_2_ftest is
    procedure mode_txbbm_2_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body mode_txbbm_2_ftest is
    procedure mode_txbbm_2_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_TX_frame       :       SW_CAN_frame_type;
        variable CAN_RX_frame       :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable frames_equal       :       boolean := false;
        variable mode_1             :       SW_mode := SW_mode_rst_val;
        variable mode_2             :       SW_mode := SW_mode_rst_val;
        
        variable err_counters       :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_2     :       SW_error_counters := (0, 0, 0, 0);

        variable fault_th           :       SW_fault_thresholds;
        variable fault_th_2         :       SW_fault_thresholds;

        variable txt_buf_count      :       natural;
        variable tmp_int            :       natural;
        variable txt_buf_index      :       natural;

        variable txt_buf_vector     :       std_logic_vector(7 downto 0) := x"00";
        variable txt_buf_state      :       SW_TXT_Buffer_state_type;

        variable status_1           :       SW_status;
    begin

        -----------------------------------------------------------------------
        -- @1. Set TXT Buffer backup mode in DUT. Set Test node to ACK
        --     forbidden mode. Set DUT retransmit limit threshold to 0
        --     (One shot mode).
        -----------------------------------------------------------------------
        info_m("Step 1");

        mode_1.tx_buf_backup := true;
        set_core_mode(mode_1, DUT_NODE, chn);

        mode_2.acknowledge_forbidden := true;
        set_core_mode(mode_2, TEST_NODE, chn);

        CAN_enable_retr_limit(true, 0, DUT_NODE, chn);

        get_tx_buf_count(txt_buf_count, DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @2. Loop 10 times
        -----------------------------------------------------------------------
        for iteration in 1 to 10 loop
            info_m("Step 2, iteration" & integer'image(iteration));

            -------------------------------------------------------------------
            -- @2.1 Generate random priorities of TXT Buffers and apply them in
            --      DUT. Generate random index of TXT Buffer which is for sure
            --      "original" TXT Buffer and insert random frame to it.
            -------------------------------------------------------------------
            info_m("Step 2.1");

            for i in 1 to txt_buf_count loop
                rand_int_v(7, tmp_int);
                CAN_configure_tx_priority(i, tmp_int, DUT_NODE, chn);
            end loop;

            -- Pick random TXT Buffer which is "original" buffer.
            pick_random_txt_buffer(txt_buf_index, DUT_NODE, chn);
            if (txt_buf_index mod 2 = 0) then
                txt_buf_index := txt_buf_index - 1;
            end if;

            CAN_generate_frame(CAN_TX_frame);
            CAN_insert_TX_frame(CAN_TX_frame, txt_buf_index, DUT_NODE, chn);
            CAN_insert_TX_frame(CAN_TX_frame, txt_buf_index + 1, DUT_NODE, chn);
            
            -----------------------------------------------------------------------
            -- @2.2 Send set ready command to selected original TXT Buffer. Wait
            --      until DUT starts transmission, and check that "original" TXT
            --      Buffer is in TX in Progress and "backup" TXT Buffer is in
            --      "ready" state.
            -----------------------------------------------------------------------
            info_m("Step 2.2");

            txt_buf_vector := x"00";
            txt_buf_vector(txt_buf_index - 1) := '1';
            txt_buf_vector(txt_buf_index) := '1';

            send_TXT_buf_cmd(buf_set_ready, txt_buf_vector, DUT_NODE, chn);

            CAN_wait_tx_rx_start(true, false, DUT_NODE, chn);
            
            get_tx_buf_state(txt_buf_index, txt_buf_state, DUT_NODE, chn);
            check_m(txt_buf_state = buf_tx_progress, "'Original' TXT Buffer is in 'TX in Progress'");
            get_tx_buf_state(txt_buf_index + 1, txt_buf_state, DUT_NODE, chn);
            check_m(txt_buf_state = buf_ready, "'Backup' TXT Buffer is in 'TX Ready'");

            -----------------------------------------------------------------------
            -- @2.3 Wait until transmission ends and bus is idle. Check that
            --      "original" TXT Buffer is in "TX Failed" state and "backup" TXT
            --      buffer ended in "Aborted" state. 
            -----------------------------------------------------------------------
            info_m("Step 2.3");
            
            CAN_wait_frame_sent(DUT_NODE, chn);
            CAN_wait_bus_idle(TEST_NODE, chn);
            CAN_wait_bus_idle(DUT_NODE, chn);

            get_tx_buf_state(txt_buf_index, txt_buf_state, DUT_NODE, chn);
            check_m(txt_buf_state = buf_failed, "'Original' TXT Buffer is in 'TX Failed'");
            get_tx_buf_state(txt_buf_index + 1, txt_buf_state, DUT_NODE, chn);
            check_m(txt_buf_state = buf_aborted, "'Backup' TXT Buffer is in 'Aborted'");

            get_controller_status(status_1, DUT_NODE, chn);
            check_false_m(status_1.tx_parity_error, "Parity error not set!");
            check_false_m(status_1.tx_double_parity_error, "Double parity error not set!");
        end loop;

  end procedure;

end package body;