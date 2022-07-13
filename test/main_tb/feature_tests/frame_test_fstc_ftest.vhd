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
--  FRAME_TEST_W[FSTC] feature test
--
-- @Verifies:
--  @1. When MODE[TSTM] = 1, FRAME_TEST_W[FSTC] CTU CAN FD flips stuff count bit
--      at position of FRAME_TEST_W[TPRM].
--
-- @Test sequence:
--  @1. Set Test mode in DUT. 
--  @2. Generate random CAN FD frame. Transmit it by DUT, record transmitted value
--      of stuff count ignoring stuff bits.
--  @3. Send again the same frame as in previous point, only flip random bit
--      of Stuff count, again record the stuff count. Check that transmitted
--      stuff count has correct bit flipped.
--  @4. Wait until error frame is transmitted (frame is corrupted, TEST_NODE 
--      should transmit error frame). Check that TEST_NODE detects CRC error.
--      Wait until frame is transmitted. 
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    12.07.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package frame_test_fstc_ftest is
    procedure frame_test_fstc_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body frame_test_fstc_ftest is
    procedure frame_test_fstc_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_TX_frame       :       SW_CAN_frame_type;
        variable CAN_RX_frame       :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable frames_equal       :       boolean := false;
        variable mode_1             :       SW_mode := SW_mode_rst_val;
        
        variable err_counters       :       SW_error_counters := (0, 0, 0, 0);
        variable err_counters_2     :       SW_error_counters := (0, 0, 0, 0);

        variable fault_th           :       SW_fault_thresholds;
        variable fault_th_2         :       SW_fault_thresholds;

        variable txt_buf_count      :       natural;
        variable tmp_int            :       natural;
        variable txt_buf_index      :       natural;

        variable status_1           :       SW_status;

        variable txt_buf_vector     :       std_logic_vector(7 downto 0) := x"00";
        variable txt_buf_state      :       SW_TXT_Buffer_state_type;

        variable bit_to_flip        :       natural;
        variable golden_stc         :       std_logic_vector(3 downto 0) := "0000";
        variable expected_stc       :       std_logic_vector(3 downto 0) := "0000";
        variable real_stc           :       std_logic_vector(3 downto 0) := "0000";

        variable err_capt           :       SW_error_capture;
    begin

        -----------------------------------------------------------------------
        -- @1. Set Test mode in DUT. 
        -----------------------------------------------------------------------
        info_m("Step 1");

        mode_1.test := true;
        -- Self test mode is needed so that DUT will not send Error frame after
        -- not getting ACK due to flipped bit. This-way we can check that test-node
        -- has detected CRC error!
        mode_1.self_test := true;
        set_core_mode(mode_1, DUT_NODE, chn);
        
        get_tx_buf_count(txt_buf_count, DUT_NODE, chn);
        
        -----------------------------------------------------------------------
        -- @2. Generate random CAN FD frame. Transmit it by DUT, record
        --     transmitted value of stuff count ignoring stuff bits.
        -----------------------------------------------------------------------
        info_m("Step 2");

        CAN_generate_frame(CAN_TX_frame);
        CAN_TX_frame.frame_format := FD_CAN;

        pick_random_txt_buffer(txt_buf_index, DUT_NODE, chn);
        CAN_insert_TX_frame(CAN_TX_frame, txt_buf_index, DUT_NODE, chn);

        send_TXT_buf_cmd(buf_set_ready, txt_buf_index, DUT_NODE, chn);

        CAN_wait_pc_state(pc_deb_stuff_count, DUT_NODE, chn);
        
        for i in 0 to 3 loop
            CAN_wait_sample_point(DUT_NODE, chn);
            get_can_tx(DUT_NODE, golden_stc(i), chn);
        end loop;

        info_m("Original stuff count value is: 0x" & to_string(golden_stc));

        CAN_wait_bus_idle(DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @3. Send again the same frame as in previous point, only flip 
        --     random bit of Stuff count, again record the stuff count.
        --     Check that transmitted stuff count has correct bit flipped.
        -----------------------------------------------------------------------
        info_m("Step 3");

        CAN_insert_TX_frame(CAN_TX_frame, txt_buf_index, DUT_NODE, chn);

        rand_int_v(3, bit_to_flip);
        CAN_set_frame_test(txt_buf_index, bit_to_flip, true, false, false,
                           DUT_NODE, chn);

        send_TXT_buf_cmd(buf_set_ready, txt_buf_index, DUT_NODE, chn);

        CAN_wait_pc_state(pc_deb_stuff_count, DUT_NODE, chn);
        
        for i in 0 to 3 loop
            CAN_wait_sample_point(DUT_NODE, chn);
            get_can_tx(DUT_NODE, real_stc(i), chn);
        end loop;

        -- Calculate expected stuff count
        expected_stc := golden_stc;
        expected_stc(bit_to_flip) := not expected_stc(bit_to_flip);

        info_m("Golden stuff count: " & to_string(golden_stc));
        info_m("Expected stuff count: " & to_string(expected_stc));
        info_m("Real stuff count: " & to_string(real_stc));
        check_m(expected_stc = real_stc, "Expected Stuff count = Real stuff count");

        -----------------------------------------------------------------------
        -- @4. Wait until error frame is transmitted (frame is corrupted,
        --     TEST_NODE should transmit error frame). Check that TEST_NODE
        --     detects Form error. Wait until frame is transmitted.
        -----------------------------------------------------------------------
        info_m("Step 4");

        CAN_wait_error_frame(TEST_NODE, chn);
        wait for 20 ns;

        CAN_read_error_code_capture(err_capt, TEST_NODE, chn);

        check_m(err_capt.err_pos = err_pos_ack, "Error in ACK field");
        check_m(err_capt.err_type = can_err_crc, "CRC error detected");

        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);

  end procedure;

end package body;