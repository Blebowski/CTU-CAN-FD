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
--  BTR (Bit timing register) feature test.
--
-- @Verifies:
--  @1. BTR register properly configures PROP, PH1, PH2 registers.
--  @2. Transmission/reception at random bit-rate.
--
-- @Test sequence:
--  @1. Disable both Nodes. Generate random bit-rate and configure it sa Nominal
--      bit-rate! Enable both nodes and wait till both nodes are on.
--  @2. Wait until sample point in DUT and measure number of clock cycles
--      till next sample point. Check that it corresponds to pre-computed value!
--  @3. Send frame by DUT and wait until it is sent. Read frame from Test node
--      and check they are matching.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--   11.11.2019   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;
use ctu_can_fd_tb.clk_gen_agent_pkg.all;

package btr_ftest is
    procedure btr_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body btr_ftest is
    procedure btr_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame_1        :       SW_CAN_frame_type;
        variable CAN_frame_2        :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        
        variable bus_timing         :       bit_time_config_type;

        variable clock_per_bit      :       natural := 0;

        variable clock_meas         :       natural := 0;
        variable frames_equal       :       boolean;
        
        variable tx_delay           :       time;
        
        variable t_meas_start       :       time;
        variable t_meas_stop        :       time;
        variable clk_sys_period     :       time;
    begin

        -----------------------------------------------------------------------
        -- @1. Disable both Nodes. Generate random bit-rate and configure it sa 
        --     Nominal bit-rate! Enable both nodes and wait till both nodes are 
        --     on.
        -----------------------------------------------------------------------
        info_m("Step 1");

        CAN_turn_controller(false, DUT_NODE, chn);
        CAN_turn_controller(false, TEST_NODE, chn);

        -- Generate random Nominal bit rate!
        rand_int_v(127, bus_timing.prop_nbt);
        rand_int_v(63, bus_timing.ph1_nbt);
        rand_int_v(63, bus_timing.ph2_nbt);
        
        -- Longer TQ is possible but test run-time is killing us!
        rand_int_v(32, bus_timing.tq_nbt);
        rand_int_v(33, bus_timing.sjw_nbt);
        
        -- Configure delay of TX -> RX so that for any generated bit-rate, it
        -- is not too high! Otherwise, roundtrip will be too high and Node will
        -- not manage to receive ACK in time!
        -- Before sample point, whole roundtrip must be made (and 2 more clock
        -- cycles due to input delay!). Lets take the delay as one third of
        -- TSEG1. Roundtrip will take two thirds and we should be safe!
        
        tx_delay := (((1 + bus_timing.prop_nbt + bus_timing.ph1_nbt) *
                       bus_timing.tq_nbt) / 3) * 10 ns;
        info_m("TX delay is: " & time'image(tx_delay));
        ftr_tb_set_tran_delay(tx_delay, DUT_NODE, chn);
        ftr_tb_set_tran_delay(tx_delay, TEST_NODE, chn);

        -----------------------------------------------------------------------
        -- Leave Data bit rate unconfigured! This will result in XXXs in register
        -- map, and XXXs in prescaler (we will see about warnings), but in this
        -- test we will NOT use data bit-rate, therefore all logic operating
        -- on XXXs should be ignored! So it will be a nice experiment that
        -- even crazy things in DBT does not ruin NBT operation...
        -- This test will never have chance to pass at any "X pesimistic"
        -- simulation. But who has money for X pessimism simulator? :(
        -----------------------------------------------------------------------

        -- SJW should be at least one because clocks differ by some value so
        -- there should be chance to compensate
        if (bus_timing.sjw_nbt = 0) then
            bus_timing.sjw_nbt := 1;
        end if;

        -- Constrain minimal BRP (0 is not allowed)!
        if (bus_timing.tq_nbt = 0) then
            bus_timing.tq_nbt := 1;
        end if;

        -- Pre-calculate expected number of clock cycles
        clock_per_bit := (1 + bus_timing.prop_nbt + bus_timing.ph1_nbt +
                          bus_timing.ph2_nbt) * bus_timing.tq_nbt;

        -- It has no sense to test frequencies above 5 MHz for Nominal Bit-rate
        -- lets constrain it to something reasonable.
        while (clock_per_bit < 20) loop

            if (bus_timing.prop_nbt < 127) then
                bus_timing.prop_nbt := bus_timing.prop_nbt + 1;
            end if;

            if (bus_timing.ph1_nbt < 63) then
                bus_timing.ph1_nbt := bus_timing.ph1_nbt + 1;
            end if;

            if (bus_timing.ph2_nbt < 63) then
                bus_timing.ph2_nbt := bus_timing.ph2_nbt + 1;
            end if;

            clock_per_bit := (1 + bus_timing.prop_nbt + bus_timing.ph1_nbt +
                              bus_timing.ph2_nbt) * bus_timing.tq_nbt;
        end loop;

        -- Constrain minimal duration of PH2 to be 2 clock cycles!
        if (bus_timing.ph2_nbt * bus_timing.prop_nbt < 2) then
            bus_timing.ph2_nbt := 2;
        end if;

        -- Constrain minimal duration of TSEG1 to be 2 clock cycles!
        if ((bus_timing.prop_nbt + bus_timing.ph1_nbt + 1) * bus_timing.tq_nbt < 2) then
            bus_timing.prop_nbt := 1;
        end if;

        -- Pre-calculate expected number of clock cycles after all corrections!
        clock_per_bit := (1 + bus_timing.prop_nbt + bus_timing.ph1_nbt +
                          bus_timing.ph2_nbt) * bus_timing.tq_nbt;

        CAN_configure_timing(bus_timing, DUT_NODE, chn);
        CAN_configure_timing(bus_timing, TEST_NODE, chn);

        CAN_turn_controller(true, DUT_NODE, chn);
        CAN_turn_controller(true, TEST_NODE, chn);

        CAN_wait_bus_on(DUT_NODE, chn);
        CAN_wait_bus_on(TEST_NODE, chn);

        info_m("CAN bus nominal bit-rate:");
        info_m("PROP: " & integer'image(bus_timing.prop_nbt));
        info_m("PH1: " & integer'image(bus_timing.ph1_nbt));
        info_m("PH2: " & integer'image(bus_timing.ph2_nbt));
        info_m("SJW: " & integer'image(bus_timing.sjw_nbt));

        -----------------------------------------------------------------------
        -- @2. Wait until sample point in DUT and measure number of clock
        --     cycles till next sample point. Check that it corresponds to
        --     pre-computed value!
        -----------------------------------------------------------------------
        info_m("Step 2");

        CAN_wait_sample_point(DUT_NODE, chn, false);
        t_meas_start := now;
        CAN_wait_sample_point(DUT_NODE, chn, false);
        t_meas_stop := now;

        clk_agent_get_period(chn, clk_sys_period);

        clock_meas := ((t_meas_stop - t_meas_start) / clk_sys_period);

        check_m(clock_per_bit = clock_meas,
            " Expected clock per bit: " & integer'image(clock_per_bit) &
            " Measured clock per bit: " & integer'image(clock_meas));

        -----------------------------------------------------------------------
        -- @3. Send frame by DUT and wait until it is sent. Read frame from
        --     Test node and check they are matching.
        -----------------------------------------------------------------------
        info_m("Step 3");
        
        -- Shorten length of generated frame to max 4 data bytes! The thing is
        -- that if generated bit rate is too low, and data field length too
        -- high, test run time explodes! It has no sense to test long data fields
        -- on any bit-rate since its functionality should not depend on it!
        CAN_generate_frame(CAN_frame_1);
        info_m("Generated frame");
        CAN_frame_1.frame_format := NORMAL_CAN;

        if (CAN_frame_1.data_length > 4) then
            CAN_frame_1.data_length := 4;
            decode_length(CAN_frame_1.data_length, CAN_frame_1.dlc);
            decode_dlc_rx_buff(CAN_frame_1.dlc, CAN_frame_1.rwcnt);
        end if;

        -- Force frame type to CAN 2.0 since we are measuring nominal bit rate!
        CAN_send_frame(CAN_frame_1, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_frame_sent(TEST_NODE, chn);
        CAN_read_frame(CAN_frame_2, TEST_NODE, chn);

        CAN_compare_frames(CAN_frame_1, CAN_frame_2, false, frames_equal);
        check_m(frames_equal, "TX/RX frame equal!");

  end procedure;

end package body;