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
--  RX Buffer consistency 2 feature test implementation.
--
-- @Verifies:
--  @1. RX Buffer Memory capacity indication (RX Mem Free) consistency at the
--      time when "abort" of currently stored frame occurs (due to error frame)
--      and a frame is read at the same time.
--
-- @Test sequence:
--   @1. Disable DUT, configure fast Nominal and Data bit-rate (to reduce test
--       length). Enable Test Mode in both DUT and Test Node. Test Mode is
--       needed to flip CRC bits in the transmitted frame!
--       Generate two random CAN frames. Second frame will contain a
--       bit-flip in CRC field.
--   @2. Iterate with incrementing wait time X.
--       @2.1 Send both frames by Test Node.
--       @2.2 Wait until first frame is sent by DUT. Check that RX Mem Free
--            is equal to RX Buffer size minus size of the first frame.
--       @2.3 Wait until start of CRC Delimiter in DUT. Wait for X clock cycles.
--       @2.4 Read out frame from DUT Node. Due to incrementing time X, the
--            test will hit the scenario where frame is simultaneously being
--            read, and second frame is aborted due to Error frame. Error frame
--            is caused since a CRC bit was flipped, causing CRC Error.
--       @2.5 Wait until Error frame in DUT. Wait until bus is idle!
--       @2.6 Check that RX Mem Free of DUT node is eqal to size of RX Buffer.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--
--    17.7.2024  Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.clk_gen_agent_pkg.all;

package rx_buf_consistency_2_ftest is
    procedure rx_buf_consistency_2_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body rx_buf_consistency_2_ftest is
    procedure rx_buf_consistency_2_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable bus_timing         :       bit_time_config_type;

        variable CAN_TX_frame_1     :       SW_CAN_frame_type;
        variable CAN_TX_frame_2     :       SW_CAN_frame_type;
        variable CAN_RX_frame_1     :       SW_CAN_frame_type;
        variable CAN_RX_frame_2     :       SW_CAN_frame_type;

        variable rx_buf_info        :       SW_RX_Buffer_info;

        variable frames_match       :       boolean;
        variable frame_sent         :       boolean;

        variable err_counters       :       SW_error_counters;

        variable mode               :       SW_mode := SW_mode_rst_val;
    begin

        ------------------------------------------------------------------------
        -- @1. Disable DUT, configure fast Nominal and Data bit-rate (to reduce
        --     test length). Enable Test Mode in both DUT and Test Node. Test
        --     Mode is needed to flip CRC bits in the transmitted frame!
        --     Generate two random CAN frames. Second frame will contain a
        --     bit-flip in CRC field.
        ------------------------------------------------------------------------
        info_m("Step 1");

        bus_timing.tq_nbt     := 1;
        bus_timing.tq_dbt     := 1;
        bus_timing.prop_nbt   := 5;
        bus_timing.ph1_nbt    := 2;
        bus_timing.ph2_nbt    := 4;
        bus_timing.sjw_nbt    := 1;
        bus_timing.prop_dbt   := 3;
        bus_timing.ph1_dbt    := 1;
        bus_timing.ph2_dbt    := 3;
        bus_timing.sjw_dbt    := 1;

        CAN_turn_controller(false, DUT_NODE, chn);
        CAN_turn_controller(false, TEST_NODE, chn);

        CAN_configure_timing(bus_timing, DUT_NODE, chn);
        CAN_configure_timing(bus_timing, TEST_NODE, chn);

        CAN_configure_ssp(ssp_no_ssp, x"00", DUT_NODE, chn);
        CAN_configure_ssp(ssp_no_ssp, x"00", TEST_NODE, chn);

        mode.test := true;
        set_core_mode(mode, DUT_NODE, chn);
        set_core_mode(mode, TEST_NODE, chn);

        CAN_turn_controller(true, DUT_NODE, chn);
        CAN_turn_controller(true, TEST_NODE, chn);

        CAN_wait_bus_on(DUT_NODE, chn);
        CAN_wait_bus_on(TEST_NODE, chn);

        -- First frame - Will always take 4 words in RX Buffer
        CAN_generate_frame(CAN_TX_frame_1);
        CAN_TX_frame_1.data_length := 0;
        decode_length(CAN_TX_frame_1.data_length, CAN_TX_frame_1.dlc);
        decode_dlc_rx_buff(CAN_TX_frame_1.dlc, CAN_TX_frame_1.rwcnt);

        -- Second frame
        -- Make it fixed so that we don't see spurious fails due to immediate
        -- frame on flipped stuff-bit!
        CAN_generate_frame(CAN_TX_frame_2);
        CAN_TX_frame_2.identifier := 0;
        CAN_TX_frame_2.ident_type := BASE;
        CAN_TX_frame_2.frame_format := NORMAL_CAN;
        CAN_TX_frame_2.rtr := NO_RTR_FRAME;
        CAN_TX_frame_2.data_length := 0;

        decode_length(CAN_TX_frame_2.data_length, CAN_TX_frame_2.dlc);
        decode_dlc_rx_buff(CAN_TX_frame_2.dlc, CAN_TX_frame_2.rwcnt);

        ------------------------------------------------------------------------
        -- @2. Iterate with incrementing wait time X.
        ------------------------------------------------------------------------
        info_m("Step 2");

        for wait_multiple in 1 to 150 loop

            --------------------------------------------------------------------
            -- @2.1 Send both frames by Test Node. Set Error counters to 0
            --      in both nodes.
            --------------------------------------------------------------------
            info_m("Step 2.1");

            err_counters.rx_counter := 0;
            err_counters.tx_counter := 0;
            err_counters.err_norm   := 0;
            err_counters.err_fd     := 0;

            set_error_counters(err_counters, DUT_NODE, chn);
            set_error_counters(err_counters, TEST_NODE, chn);

            CAN_insert_TX_frame(CAN_TX_frame_1, 1, TEST_NODE, chn);
            CAN_insert_TX_frame(CAN_TX_frame_2, 2, TEST_NODE, chn);

            -- Configure bit-flip in CRC bit 12 from frame 2
            CAN_set_frame_test(2, 12, false, true, false, TEST_NODE, chn);

            send_TXT_buf_cmd(buf_set_ready, 1, TEST_NODE, chn);
            wait for 100 ns;
            send_TXT_buf_cmd(buf_set_ready, 2, TEST_NODE, chn);

            --------------------------------------------------------------------
            -- @2.2 Wait until first frame is sent by DUT. Check that RX Mem Free
            --      is equal to RX Buffer size minus size of the first frame.
            --------------------------------------------------------------------
            info_m("Step 2.2");

            CAN_wait_frame_sent(DUT_NODE, chn);

            get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
            check_m(rx_buf_info.rx_mem_free = rx_buf_info.rx_buff_size - 4,
                     "RX MEM Free = RX Buffer Size - 4");

            --------------------------------------------------------------------
            -- @2.3 Wait until start of CRC Delimiter in DUT.
            --      Wait for X clock cycles.
            --------------------------------------------------------------------
            info_m("Step 2.3");

            CAN_wait_pc_state(pc_deb_crc, DUT_NODE, chn);
            for i in 1 to 10 loop
                CAN_wait_sample_point(DUT_NODE, chn);
            end loop;

            for i in 1 to wait_multiple loop
                info_m("Waiting for " & integer'image(i) & " clock cycles");
                clk_agent_wait_cycle(chn);
            end loop;

            --------------------------------------------------------------------
            -- @2.4 Read out frame from DUT Node. Due to incrementing time X,
            --      the test will hit the scenario where frame is simultaneously
            --      being read, and second frame is aborted due to Error frame.
            --      Error frame is caused since a CRC bit was flipped,
            --      causing CRC Error.
            --------------------------------------------------------------------
            info_m("Step 2.4");

            CAN_read_frame(CAN_RX_frame_1, DUT_NODE, chn);

            --------------------------------------------------------------------
            -- @2.5 Wait until Error frame in DUT. Wait until bus is idle!
            --------------------------------------------------------------------
            info_m("Step 2.5");

            -- This will stuck and time-out if error frame is not transmitted!
            CAN_wait_error_frame(DUT_NODE, chn);

            CAN_wait_bus_idle(DUT_NODE, chn);
            CAN_wait_bus_idle(TEST_NODE, chn);

            --------------------------------------------------------------------
            -- @2.6 Check that RX Mem Free of DUT node is equal to
            --      size of RX Buffer.
            --------------------------------------------------------------------
            info_m("Step 2.6");

            get_rx_buf_state(rx_buf_info, DUT_NODE, chn);
            check_m(rx_buf_info.rx_mem_free = rx_buf_info.rx_buff_size,
                     "RX MEM Free = RX Buffer Size");

        end loop;

    end procedure;

end package body;