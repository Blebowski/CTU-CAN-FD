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
--  Transceiver delay measurement feature test.
--
-- @Verifies:
--  @1. Transceiver delay measurement in its range (2 - 127).
--  @2. Shadowing of TRV_DELAY register (register updated only at the end of
--      measurement).
--  @3. TRV_DELAY measuremenr does not overflow when measuring delay longer than
--      127 clock cycles.
--
-- @Test sequence:
--  @1. Configure SSP Offset to 7 + TRV_DELAY and SSP source to use Measured
--      value + offset in DUT. Configure bit-rate to 250 Kbit/s in Nominal
--      bit-rate. This-way bit-error detection will not get confused on high
--      TRV Delays!
--  @2. Configure delay to 1 ns in TB. Run CAN FD frame and verify that measured
--      delay is correct! 
--  @3. Configure delay to 1255 ns in TB. Run CAN FD frame and verify that
--      measured delay is 127.
--  @4. Configure Transmitter delay to 130. Run CAN FD frame and verify that
--      measured value is 127 (value has not overflown!).
--  @5. Configure transmitter delay to random value between 0 and 126. Run CAN
--      FD frame and check it is measured correctly!
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--
--    28.6.2016   Created file
--    12.6.2018   Changed to use CAN test lib instead of direct register access.
--   18.11.2019   Re-wrote the TC to cover cornercases.
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package trv_delay_ftest is
    procedure trv_delay_ftest_exec(
        signal      chn             : inout  t_com_channel
    );

end package;


package body trv_delay_ftest is
    procedure trv_delay_ftest_exec(
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
    begin

        -----------------------------------------------------------------------
        -- @1. Configure SSP Offset to 7 + TRV_DELAY and SSP source to use
        --     Measured value + offset in DUT.Configure bit-rate to 250
        --     Kbit/s in Nominal bit-rate. This-way bit-error detection will
        --     not get confused on high TRV Delays!
        -----------------------------------------------------------------------
        info_m("Step 1");

        CAN_turn_controller(false, DUT_NODE, chn);
        CAN_turn_controller(false, TEST_NODE, chn);

        -- Should be 250 Kbit/s
        bus_timing.prop_nbt := 37;
        bus_timing.ph1_nbt := 37;
        bus_timing.ph2_nbt := 25;
        bus_timing.tq_nbt := 4;
        bus_timing.sjw_nbt := 5;

        -- Should be 2 Mbit/s
        bus_timing.prop_dbt := 10;
        bus_timing.ph1_dbt := 20;
        bus_timing.ph2_dbt := 19;
        bus_timing.tq_dbt := 1;
        bus_timing.sjw_dbt := 5;

        CAN_configure_timing(bus_timing, DUT_NODE, chn);
        CAN_configure_timing(bus_timing, TEST_NODE, chn);

        CAN_configure_ssp(ssp_meas_n_offset, "00000111", DUT_NODE, chn);
        CAN_configure_ssp(ssp_meas_n_offset, "00000111", TEST_NODE, chn);

        -- Turn the controllers on!
        CAN_turn_controller(true, DUT_NODE, chn);
        CAN_turn_controller(true, TEST_NODE, chn);

        -- Wait till integration is over!
        CAN_wait_bus_on(DUT_NODE, chn);
        CAN_wait_bus_on(TEST_NODE, chn);
        
        -----------------------------------------------------------------------
        -- @2. Configure delay to 1 ns in TB. Run CAN FD frame and verify that
        --     measured delay is correct! 
        -----------------------------------------------------------------------
        info_m("Step 2");
        ftr_tb_set_tran_delay(1 ns, DUT_NODE, chn);
        CAN_generate_frame(CAN_TX_frame);
        CAN_TX_frame.rtr := NO_RTR_FRAME;
        CAN_TX_frame.frame_format := FD_CAN;
        CAN_TX_frame.brs := BR_SHIFT;

        CAN_send_frame(CAN_TX_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_frame_sent(DUT_NODE, chn);

        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);

        read_trv_delay(measured_delay, DUT_NODE, chn);
        
        -- Measured delay is always rounded up to nearest multiple of 10 ns
        -- (Delay of 1 ns -> 10 ns -> 1)!
        check_m(measured_delay = 2, "Minimal transmitter delay!" &
              " Expected: " & integer'image(2) &
              " Measured: " & integer'image(measured_delay));

        CAN_read_frame(CAN_RX_frame, TEST_NODE, chn);
        CAN_compare_frames(CAN_RX_frame, CAN_TX_frame, false, frames_equal);
        
        check_m(frames_equal, "TX RX frames match");

        -----------------------------------------------------------------------
        -- @3. Configure delay to 1255 ns in TB. Run CAN FD frame and verify
        --     that measured delay is 127.
        -----------------------------------------------------------------------
        info_m("Step 3");
        ftr_tb_set_tran_delay(1255 ns, DUT_NODE, chn);

        CAN_send_frame(CAN_TX_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_frame_sent(DUT_NODE, chn);

        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);

        read_trv_delay(measured_delay, DUT_NODE, chn);
        
        -- Measured delay is always rounded up to nearest multiple of 10 ns
        -- (Delay of 1255 ns -> 1250 -> 125 + 2 synchronisation cycles = 127)!
        check_m(measured_delay = 127, "Maximal transmitter delay!" &
              " Expected: " & integer'image(127) &
              " Measured: " & integer'image(measured_delay));

        CAN_read_frame(CAN_RX_frame, TEST_NODE, chn);
        CAN_compare_frames(CAN_RX_frame, CAN_TX_frame, false, frames_equal);

        check_m(frames_equal, "TX RX frames match");

        -----------------------------------------------------------------------
        -- @4. Configure Transmitter delay to 130. Run CAN FD frame and verify
        --     that measured value is 127 (value has not overflown!).
        -----------------------------------------------------------------------
        info_m("Step 4");
        ftr_tb_set_tran_delay(1305 ns, DUT_NODE, chn);

        CAN_send_frame(CAN_TX_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_frame_sent(TEST_NODE, chn);

        read_trv_delay(measured_delay, DUT_NODE, chn);
        
        -- Measured delay should have saturated at 127!
        check_m(measured_delay = 127, "Saturated transmitter delay!" &
              " Expected: " & integer'image(127) &
              " Measured: " & integer'image(measured_delay));

        -- Now CAN frame should pass because SSP Offset is high enough that
        -- it will compensate for missing delay caused by saturation!

        CAN_read_frame(CAN_RX_frame, TEST_NODE, chn);
        CAN_compare_frames(CAN_RX_frame, CAN_TX_frame, false, frames_equal);

        check_m(frames_equal, "TX RX frames match");

        -----------------------------------------------------------------------
        --  @5. Configure transmitter delay to random value between 0 and 126.
        --      Run CAN FD frame and check it is measured correctly!
        -----------------------------------------------------------------------
        info_m("Step 5");

        rand_int_v(1259, rand_time);
        if (rand_time = 0) then
            rand_time := 1;
        end if;

        -----------------------------------------------------------------------
        -- Here we avoid explicit multiples of 10 ns! The reason is following:
        --  When delay is e.g. 120 ns, then value will arrive at CAN RX when
        --  rising_edge is active. Therefore sampled value might, or might not
        --  be processed by clock based on which delta cycle was processed
        --  first (Since signal delayer does not work with system clocks, it
        --  might not be processed the same way as e.g. shift register!)
        --  This would cause occasional test failures based on which process
        --  was executed first (either rising_edge sampling the data, or data
        --  delayed by signal delayer).
        -----------------------------------------------------------------------
        if (rand_time mod 10 = 0) then
            rand_time := rand_time + 1;
        end if;
        
        info_m("Random time is: " & integer'image(rand_time) & " ns");
        ftr_tb_set_tran_delay((rand_time * 1 ns), DUT_NODE, chn);

        CAN_send_frame(CAN_TX_frame, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_frame_sent(TEST_NODE, chn);

        read_trv_delay(measured_delay, DUT_NODE, chn);

        -- Ceil will give us one more clock cycle. We need one more to
        -- compensate full input delay.
        rand_time_ceiled := integer(ceil(real(rand_time) / 10.0)) + 1;

        -- Measured delay is always rounded up to nearest multiple of 10 ns
        check_m(measured_delay = rand_time_ceiled, "Random transmitter delay!" &
              " Expected: " & integer'image(rand_time_ceiled) &
              " Measured: " & integer'image(measured_delay));

        -- Now CAN frame should pass because SSP Offset is high enough that
        -- it will compensate for missing delay caused by saturation!

        CAN_read_frame(CAN_RX_frame, TEST_NODE, chn);
        CAN_compare_frames(CAN_RX_frame, CAN_TX_frame, false, frames_equal);

        check_m(frames_equal, "TX RX frames match");

  end procedure;

end package body;