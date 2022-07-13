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
--  FRAME_TEST_W feature test when MODE[TSTM] = 0.
--
-- @Verifies:
--  @1. When MODE[TSTM] = 0, FRAME_TEST_W does not affect value of transmitted
--      DLC, CRC or Stuff count.
--
-- @Test sequence:
--  @1. Disable test mode in DUT.
--  @2. Generate random CAN FD frame. Set FRAME_TEST_W so that no CRC, Stuff
--      count bit is flipped, no DLC is swapped. Send the can frame by DUT.
--      Record transmitted value of DLC, Stuff count and CRC.
--  @3. Set FRAME_TEST_W so that Stuff count, CRC shall be flipped, and DLC
--      shall be swapped. Keep MODE[TSTM] = 0. Send the same CAN frame as in
--      previous step, and again record transmitted value of DLC, stuff count
--      and CRC. Check that transmitted values match (nothing was flipped).
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    13.07.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package frame_test_ignore_ftest is
    procedure frame_test_ignore_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body frame_test_ignore_ftest is


    procedure record_dlc_stc_crc(
        constant    crc_length      : in     natural;
        variable    output_dlc      : out    std_logic_vector(3 downto 0);
        variable    output_stc      : out    std_logic_vector(3 downto 0);
        variable    output_crc      : out    std_logic_vector(20 downto 0);
        signal      chn             : inout  t_com_channel
    ) is
    begin
        -- Record DLC
        output_dlc := (others => '0');
        CAN_wait_pc_state(pc_deb_control, DUT_NODE, chn);
        
        -- Fixed frame format assumed since Frame is with Base identifier and CAN FD.
        -- IDE, EDL, r0, BRS, ESI
        for i in 0 to 4 loop
            CAN_wait_sample_point(DUT_NODE, chn);
        end loop;

        for i in 0 to 3 loop
            CAN_wait_sample_point(DUT_NODE, chn);
            get_can_tx(DUT_NODE, output_dlc(3 - i), chn);
        end loop;

        -- Record stuff count
        output_stc := (others => '0');
        CAN_wait_pc_state(pc_deb_stuff_count, DUT_NODE, chn);
        
        for i in 0 to 3 loop
            CAN_wait_sample_point(DUT_NODE, chn);
            get_can_tx(DUT_NODE, output_stc(i), chn);
        end loop;

        -- Record CRC
        output_crc := (others => '0');
        CAN_wait_pc_state(pc_deb_crc, DUT_NODE, chn);
        
        for i in 0 to crc_length - 1 loop
            CAN_wait_sample_point(DUT_NODE, chn);
            get_can_tx(DUT_NODE, output_crc(i), chn);
        end loop;
    end procedure;


    procedure frame_test_ignore_ftest_exec(
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

        variable golden_dlc         :       std_logic_vector(3 downto 0);
        variable golden_crc         :       std_logic_vector(20 downto 0);
        variable golden_stc         :       std_logic_vector(3 downto 0);

        variable real_dlc           :       std_logic_vector(3 downto 0);
        variable real_crc           :       std_logic_vector(20 downto 0);
        variable real_stc           :       std_logic_vector(3 downto 0);

        variable tprm               :       natural;
        variable crc_length         :       natural;
    begin

        -----------------------------------------------------------------------
        -- @1. Disable test mode in DUT.
        -----------------------------------------------------------------------
        info_m("Step 1");

        mode_1.test := false;
        set_core_mode(mode_1, DUT_NODE, chn);
        
        get_tx_buf_count(txt_buf_count, DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @2. Generate random CAN FD frame. Set FRAME_TEST_W so that no CRC,
        --     Stuff count bit is flipped, no DLC is swapped. Send the can
        --     frame by DUT. Record transmitted value of DLC, Stuff count and
        --     CRC.
        -----------------------------------------------------------------------
        info_m("Step 2");

        CAN_generate_frame(CAN_TX_frame);
        CAN_TX_frame.frame_format := FD_CAN;
        CAN_TX_frame.ident_type := BASE;
        CAN_TX_frame.identifier := CAN_TX_frame.identifier mod 2**11;
        info_m("Transmitted frame:");
        CAN_print_frame(CAN_TX_frame);

        if (CAN_TX_frame.data_length > 16) then
            crc_length := 21;
        else
            crc_length := 17;
        end if;

        pick_random_txt_buffer(txt_buf_index, DUT_NODE, chn);
        CAN_insert_TX_frame(CAN_TX_frame, txt_buf_index, DUT_NODE, chn);

        CAN_set_frame_test(txt_buf_index, 0, false, false, false,
                           DUT_NODE, chn);

        send_TXT_buf_cmd(buf_set_ready, txt_buf_index, DUT_NODE, chn);

        record_dlc_stc_crc(crc_length, golden_dlc, golden_stc, golden_crc, chn);

        CAN_wait_bus_idle(DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @3. Set FRAME_TEST_W so that Stuff count, CRC shall be flipped, and
        --     DLC shall be swapped. Keep MODE[TSTM] = 0. Send the same CAN
        --     frame as in previous step, and again record transmitted value
        --     of DLC, stuff count and CRC. Check that transmitted values
        --     match (nothing was flipped).
        -----------------------------------------------------------------------
        info_m("Step 3");

        CAN_insert_TX_frame(CAN_TX_frame, txt_buf_index, DUT_NODE, chn);

        -- Attempt to swap DLC, bit-flip CRC and Stuff count fields.
        rand_int_v(15, tprm);
        CAN_set_frame_test(txt_buf_index, tprm, true, true, true,
                           DUT_NODE, chn);

        send_TXT_buf_cmd(buf_set_ready, txt_buf_index, DUT_NODE, chn);

        record_dlc_stc_crc(crc_length, real_dlc, real_stc, real_crc, chn);

        wait for 50 ns;

        info_m("*******************************************************");
        info_m("Golden DLC:         " & to_string(golden_dlc));
        info_m("Golden Stuff count: " & to_string(golden_stc));
        info_m("Golden CRC:         " & to_string(golden_crc));
        info_m("*******************************************************");
        info_m("Real DLC:           " & to_string(real_dlc));
        info_m("Real Stuff count:   " & to_string(real_stc));
        info_m("Real CRC:           " & to_string(real_crc));
        info_m("*******************************************************");
        
        check_m(golden_dlc = real_dlc, "DLC match");
        check_m(golden_stc = real_stc, "Stuff count match");
        check_m(golden_crc = real_crc, "CRC match");

        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);

  end procedure;

end package body;