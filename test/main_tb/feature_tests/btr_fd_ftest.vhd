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
--  BTR FD (Bit timing register - flexible data rate) feature test.
--
-- @Verifies:
--  @1. BTR FD register properly configures PROP, PH1, PH2 registers.
--  @2. Transmission/reception at random bit-rate.
--
-- @Test sequence:
--  @1. Disable both Nodes. Generate random bit-rate and configure it sa Data
--      bit-rate! Nominal bit-rate remains the default one which was set by
--      testbench.
--  @2. Enable both Nodes and send CAN FD frame where bit-rate is shifted by
--      DUT. Wait until data field in DUT and measure duration till next
--      sample point! Transmitter during Data phase shall no re-synchronize
--      therefore, bit time will always have nominal length!
--  @3. Wait until frame is sent and check it is correctly received by Test node.
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

package btr_fd_ftest is
    procedure btr_fd_ftest_exec(
        signal      chn             : inout  t_com_channel
    );
end package;


package body btr_fd_ftest is
    procedure btr_fd_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is
        variable CAN_frame_1        :       SW_CAN_frame_type;
        variable CAN_frame_2        :       SW_CAN_frame_type;
        variable frame_sent         :       boolean := false;
        
        variable bus_timing         :       bit_time_config_type;

        variable clock_per_bit      :       natural := 0;
        
        variable tx_delay           :       time;
        
        variable clock_meas         :       natural := 0;
        variable frames_equal       :       boolean;
        variable ssp_pos            :       std_logic_vector(7 downto 0);
        
        variable t_meas_start       :       time;
        variable t_meas_stop        :       time;
        variable clk_sys_period     :       time;
    begin

        -----------------------------------------------------------------------
        -- @1. Disable both Nodes. Generate random bit-rate and configure it as 
        --    Data bit-rate! Nominal bit-rate remains the default one which was
        --    set by testbench.
        -----------------------------------------------------------------------
        info_m("Step 1");

        CAN_turn_controller(false, DUT_NODE, chn);
        CAN_turn_controller(false, TEST_NODE, chn);

        -- Read timing so that NBT values are kept!
        CAN_read_timing_v(bus_timing, DUT_NODE, chn);

        CAN_generate_random_bit_timing(bus_timing, chn);

        -- Constrain nominal bit-timing to reduce test time!
        -- This is test which verifies data bit-rate so we can afford some
        -- relaxations in nominal bit rate.
        if (bus_timing.tq_nbt > 10) then
            bus_timing.tq_nbt := 10;
        end if;
        if (bus_timing.prop_nbt > 20) then
            bus_timing.prop_nbt := 20;
        end if;
        if (bus_timing.ph1_nbt > 15) then
            bus_timing.ph1_nbt := 15;
        end if;
        if (bus_timing.ph2_nbt > 15) then
            bus_timing.ph2_nbt := 15;
        end if;
        CAN_print_timing(bus_timing);

        -----------------------------------------------------------------------
        -- Configure delay of TX -> RX so that for any generated bit-rate, it
        -- is not too high! Otherwise, roundtrip will be too high and Node will
        -- not manage to receive ACK in time!
        -- Before sample point, whole roundtrip must be made (and 2 more clock
        -- cycles due to input delay!). Lets take the delay as one third of
        -- TSEG1. Roundtrip will take two thirds and we should be safe!
        -----------------------------------------------------------------------
        tx_delay := (((1 + bus_timing.prop_nbt + bus_timing.ph1_nbt) *
                       bus_timing.tq_nbt) / 3) * 10 ns;
        info_m("TX delay is: " & time'image(tx_delay));
        ftr_tb_set_tran_delay(tx_delay, DUT_NODE, chn);
        ftr_tb_set_tran_delay(tx_delay, TEST_NODE, chn);


        -- Pre-calculate expected number of clock cycles after all corrections!
        clock_per_bit := (1 + bus_timing.prop_dbt + bus_timing.ph1_dbt +
                          bus_timing.ph2_dbt) * bus_timing.tq_dbt;

        CAN_configure_timing(bus_timing, DUT_NODE, chn);
        CAN_configure_timing(bus_timing, TEST_NODE, chn);

        -- Configure SSP so that it samples in Data-bit rate and in 50 % of
        -- expected received bit! We need it only for DUT!
        ssp_pos := std_logic_vector(to_unsigned(clock_per_bit/2, 8));
        CAN_configure_ssp(ssp_meas_n_offset, ssp_pos, DUT_NODE, chn);

        -----------------------------------------------------------------------
        -- @2. Enable both Nodes and send CAN FD frame where bit-rate is shifted
        --    by DUT. Wait until data field in DUT and measure duration
        --    till next sample point! Transmitter during Data phase shall no
        --    re-synchronize therefore, bit time will always have nominal length!
        -----------------------------------------------------------------------
        info_m("Step 2");

        CAN_turn_controller(true, DUT_NODE, chn);
        CAN_turn_controller(true, TEST_NODE, chn);

        CAN_wait_bus_on(DUT_NODE, chn);
        CAN_wait_bus_on(TEST_NODE, chn);

        info_m("CAN bus nominal bit-rate:");
        info_m("BRP: " & integer'image(bus_timing.tq_nbt));
        info_m("PROP: " & integer'image(bus_timing.prop_nbt));
        info_m("PH1: " & integer'image(bus_timing.ph1_nbt));
        info_m("PH2: " & integer'image(bus_timing.ph2_nbt));
        info_m("SJW: " & integer'image(bus_timing.sjw_nbt));

        info_m("CAN bus Data bit-rate:");
        info_m("BRP: " & integer'image(bus_timing.tq_dbt));
        info_m("PROP: " & integer'image(bus_timing.prop_dbt));
        info_m("PH1: " & integer'image(bus_timing.ph1_dbt));
        info_m("PH2: " & integer'image(bus_timing.ph2_dbt));
        info_m("SJW: " & integer'image(bus_timing.sjw_dbt));

        CAN_generate_frame(CAN_frame_1);
        CAN_frame_1.brs := BR_SHIFT;
        CAN_frame_1.frame_format := FD_CAN;

        -- Force DLC length to 1 byte only not to have long test run time!
        CAN_frame_1.dlc := "0001";
        decode_dlc(CAN_frame_1.dlc, CAN_frame_1.data_length);
        CAN_frame_1.data(0) := x"AA";

        -- We need to make sure that frame is not RTR frame, because CAN FD
        -- frames have no RTR frames! This would lead to fail in check between
        -- TX and RX frame! Also, we have to re-calculate RWCNT for the check
        -- accordingly!
        CAN_frame_1.rtr := NO_RTR_FRAME;
        decode_dlc_rx_buff(CAN_frame_1.dlc, CAN_frame_1.rwcnt);

        -- These data bytes are preloaded to have all elements of memory word
        -- defined!
        CAN_frame_1.data(1) := x"BB";
        CAN_frame_1.data(2) := x"CC";
        CAN_frame_1.data(3) := x"DD";
    
        CAN_send_frame(CAN_frame_1, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_pc_state(pc_deb_data, DUT_NODE, chn);

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
        -- @3. Wait until frame is sent and check it is correctly received by
        --    Test node.
        -----------------------------------------------------------------------
        info_m("Step 3");

        CAN_wait_bus_idle(TEST_NODE, chn);
        CAN_read_frame(CAN_frame_2, TEST_NODE, chn);

        CAN_compare_frames(CAN_frame_1, CAN_frame_2, false, frames_equal);
        check_m(frames_equal, "TX/RX frame equal!");

  end procedure;

end package body;