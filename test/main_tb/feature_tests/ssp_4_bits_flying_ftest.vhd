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
--  SSP feature test with four bits on the fly (maximum guaranteed by datasheet).
--
-- @Verifies:
--  @1. DUT operating as transmitter is able to compensate up to 4 bits on the
--      fly (SSP position is four bits later)
--
-- @Test sequence:
--  @1. Configure hard-coded Nominal and Data bit-rat in which Data bit-rate
--      is 10 times faster than Nominal bit-rate. This is to have delay in
--      Nominal bit-rate fairly negligible in both DUT and TEST Nodes.
--  @2. Configure TRV_DELAY to be 4 data bit times.
--  @3. Configure DUTs SSP to be in Measured + offset with offset being
--      half of data bit time!
--  @4. Enable DUT and wait till it becomes error active.
--  @5. Send FD frame by DUT and verify it will be succesfully received by
--      Test node.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    26.5.2021   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.clk_gen_agent_pkg.all;

package ssp_4_bits_flying_ftest is
    procedure ssp_4_bits_flying_ftest_exec(
        signal      chn             : inout  t_com_channel
    );

end package;


package body ssp_4_bits_flying_ftest is
    procedure ssp_4_bits_flying_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_TX_frame       :       SW_CAN_frame_type;
        variable CAN_RX_frame       :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        variable measured_delay     :       natural;
        variable frames_equal       :       boolean;
        variable rand_time          :       natural;
        variable rand_time_ceiled   :       natural;

        variable bus_timing         :       bit_time_config_type;
        variable trv_delay          :       time;
        variable clk_period         :       time;
    begin

        -----------------------------------------------------------------------
        -- @1. Configure hard-coded Nominal and Data bit-rat in which Data 
        --     bit-rate is 10 times faster than Nominal bit-rate. This is to
        --     have delay in Nominal bit-rate fairly negligible in both DUT
        --     and TEST Nodes.
        -----------------------------------------------------------------------
        info_m("Step 1");

        CAN_turn_controller(false, DUT_NODE, chn);
        CAN_turn_controller(false, TEST_NODE, chn);        

        -- Should be 250 Kbit/s
        bus_timing.prop_nbt := 10;
        bus_timing.ph1_nbt := 10;
        bus_timing.ph2_nbt := 10;
        bus_timing.tq_nbt := 4;
        bus_timing.sjw_nbt := 5;

        -- Should be 2 Mbit/s
        bus_timing.prop_dbt := 2;
        bus_timing.ph1_dbt := 5;
        bus_timing.ph2_dbt := 5;
        bus_timing.tq_dbt := 1;
        bus_timing.sjw_dbt := 4;

        CAN_configure_timing(bus_timing, DUT_NODE, chn);
        CAN_configure_timing(bus_timing, TEST_NODE, chn);

        -----------------------------------------------------------------------
        -- @2. Configure TRV_DELAY to be 4 data bit times.
        -----------------------------------------------------------------------
        info_m("Step 2");

        -- Data bit timing is: 5+5+2+1=13 cycles per bit. 4 bit times delay
        -- will be 13 * 4 = 42 cycles.
        clk_agent_get_period(chn, clk_period);
        trv_delay := 42 * clk_period; 
        ftr_tb_set_tran_delay(trv_delay, DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @3. Configure DUTs SSP to be in Measured + offset with offset being
        --     half of data bit time!
        -----------------------------------------------------------------------
        info_m("Step 3");

        -- Half of next bit: 6
        CAN_configure_ssp(ssp_meas_n_offset, "00000110", DUT_NODE, chn);
        CAN_configure_ssp(ssp_meas_n_offset, "00000110", TEST_NODE, chn);

        -----------------------------------------------------------------------
        -- @4. Enable DUT and wait till it becomes error active.
        -----------------------------------------------------------------------
        info_m("Step 4");

        -- Turn the controllers on!
        CAN_turn_controller(true, DUT_NODE, chn);
        CAN_turn_controller(true, TEST_NODE, chn);

        -- Wait till integration is over!
        CAN_wait_bus_on(DUT_NODE, chn);
        CAN_wait_bus_on(TEST_NODE, chn);

        -----------------------------------------------------------------------
        -- @5. Send FD frame by DUT and verify it will be succesfully received
        --     by Test node.
        -----------------------------------------------------------------------
        info_m("Step 5");

        CAN_generate_frame(CAN_TX_frame);
        CAN_TX_frame.rtr := NO_RTR_FRAME;
        CAN_TX_frame.frame_format := FD_CAN;
        CAN_TX_frame.brs := BR_SHIFT;

        CAN_send_frame(CAN_TX_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_frame_sent(DUT_NODE, chn);

        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);
        
        -- Check measured delay too! Include input delay!
        read_trv_delay(measured_delay, DUT_NODE, chn);
        check_m(measured_delay = (trv_delay / clk_period) + 2, "Measured delay is OK!" &
              " Expected: " & integer'image(trv_delay / clk_period) &
              " Measured: " & integer'image(measured_delay));

        -- Read from from TEST Node
        CAN_read_frame(CAN_RX_frame, TEST_NODE, chn);
        CAN_compare_frames(CAN_RX_frame, CAN_TX_frame, false, frames_equal);
        
        check_m(frames_equal, "TX RX frames match");

  end procedure;

end package body;