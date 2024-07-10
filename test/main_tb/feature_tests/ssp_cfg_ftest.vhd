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
--  SSP_CFG register feature test.
--
-- @Verifies:
--  @1. When SSP_CFG[SSP_SRC] = SSP_OFFSET, position of secondary sampling point
--      will be given only by SSP_OFFSET.
--  @2. When SSP_CFG[SSP_SRC] = SSP_SRC_NO_SSP, there will be no SSP and regular
--      sample point will be used to detect bit error by bit-error detector!
--  @3. When SSP_CFG[SSP_SRC] = SSP_SRC_MEAS_N_OFFSET, position of secondary
--      sampling point will be given as SSP_OFFSET + TRV_DELAY.
--  @4. Position of Secondary sampling point is saturated to 255.
--  @5. Transmitter detecting bit error in SSP will transmitt error frame at
--      nearest regular sample point, not earlier!
--
-- @Test sequence:
--  @1. Generate random TRV_DELAY between 0 and 125. Configure it in TB as delay
--      between CAN TX and CAN RX.
--  @2. Generate random SSP_CFG[SSP_SRC]. If it is offset only, generate
--      SSP_OFFSET which is higher than TRV_DELAY. If it is SSP_SRC_MEAS_N_OFFSET,
--      set SSP_OFFSET to random value between 0 and 255. Saturate calculated
--      value of SSP_SRC at 255. If it is SSP_SRC_NO_SSP, calculate SSP position
--      from regular data-bit rate.
--  @3. Generate random CAN FD frame with bit-rate shift. Wait until bit-rate is
--      shifted and wait for random number of bits (but do not exceed length of
--      data phase). Wait until edge on CAN TX or CAN RX. Store transmitted value
--      on CAN TX after the edge. Wait for expected position of Secondary sample
--      point - 3 clock cycles.
--  @4. Now we are 3 clock cycles before Secondary sampling point. Flip bus
--      to opposite value than is currently received.
--      This will cause bit error to be detected at nearest SSP.
--  @5. Wait for one clock cycle and if SSP_CFG[SSP_SRC] = SSP_SRC_NO_SSP, error
--      frame is being transmitted (regular sample point should be used to detect
--      bit errors). If SSP_CFG[SSP_SRC] /= SSP_SRC_NO_SSP check that Error frame
--      is not transmitted and wait until nearest Sample point. Check that after
--      this sample point, error frame is transmitted. Wait until bus is idle in
--      both nodes.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    02.1.2020   Created file
--------------------------------------------------------------------------------

Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.rtl_context;
context ctu_can_fd_tb.tb_common_context;

use ctu_can_fd_tb.feature_test_agent_pkg.all;

package ssp_cfg_ftest is
    procedure ssp_cfg_ftest_exec(
        signal      chn             : inout  t_com_channel
    );

    procedure generate_ssp_offset(
       bus_timing                   : in    bit_time_config_type;
       ssp_offset                   : inout std_logic_vector(7 downto 0)
    );

end package;


package body ssp_cfg_ftest is

    ---------------------------------------------------------------------------
    -- Generated TRV_DELAY must meet some criteria:
    --
    -- 1. TRV_DELAY < TSEG_1 (MBT)
    --    This is to make sure that in Nominal bit-rate, the regular sampling
    --    will work OK!
    --
    -- 2. TRV_DELAY > 0
    --    To avoid races!
    --
    -- 3. Bit Time DBT * 8 < TRV_DELAY
    --    This is to meet maximal size of TX Data cache!
    --
    -- 4. TRV_DELAY MOD 10 != 0
    --    When delay is e.g. 120 ns, then value will arrive at CAN RX when
    --    rising_edge is active. Therefore sampled value might, or might not
    --    be processed by clock based on which delta cycle was processed
    --    first (Since signal delayer does not work with system clocks, it
    --    might not be processed the same way as e.g. shift register!)
    --    This would cause occasional test failures based on which process
    --    was executed first (either rising_edge sampling the data, or data
    --    delayed by signal delayer).
    -----------------------------------------------------------------------
    procedure generate_trv_delay(
        variable trv_delay              : inout    natural;
        variable bit_timing             : inout    bit_time_config_type
    ) is
        -- Length of TSEG1 in ns, assumes 10 ns clock period!
        variable tseg_1_nbt             : natural :=
                    bit_timing.tq_nbt * (1 + bit_timing.prop_nbt + bit_timing.ph1_nbt) * 10;

        -- Length of Bit timing DBT, assumes 10 ns clock period
        variable bit_len_dbt            : natural :=
                    bit_timing.tq_dbt * (1 + bit_timing.prop_dbt + bit_timing.ph1_dbt + bit_timing.ph2_dbt);
    begin
        rand_int_v(1240, trv_delay);
        while (
            (trv_delay > tseg_1_nbt - 50)           or -- Account for 50ns margin!
            (trv_delay = 0)                         or
            (trv_delay > (bit_len_dbt * 8) - 50)    or -- Account for 50 ns margin!
            ((trv_delay mod 10) = 0)
        ) loop
            -- Currently maximal measurable value of TRV_DELAY is 127.
            -- Account +3 for CTU CAN FD input delay (delay is 2, one cycle reserve for alignment issues.)
            rand_int_v(1240, trv_delay);
        end loop;
    end procedure;


    ---------------------------------------------------------------------------
    -- There are following criterias generated SSP offset needs to meet:
    --  1. SSP_OFFSET < Bit Time (DBT)
    --
    --     If generated SSP_OFFSET is higher than duration of bit, then
    --     we will never sample correct value, because we just sample next bit
    --     already! So we need to constrain configured SSP_OFFSET to less than
    --     data bit time!
    --
    --  2. SSP_OFFSET > 0
    --
    --     To avoid races.
    --
    --  3. SSP_OFFSET < 240
    --
    --     Maximal sample point position is 255 cycles. Subtracting 2 for CTU CAN FD
    --     input delay gives 253. We keep 13 cycles reserve, since the actual
    --     sample point can include TRV_DELAY! This-way during TRV_DELAY adjustement,
    --     we always can adjust it to fit within DUT specification.
    ---------------------------------------------------------------------------
    procedure generate_ssp_offset(
       bus_timing                   : in    bit_time_config_type;
       ssp_offset                   : inout std_logic_vector(7 downto 0)
    ) is
        variable bit_time_length    :       natural :=
            bus_timing.tq_dbt *
                (1 + bus_timing.prop_dbt + bus_timing.ph1_dbt + bus_timing.ph2_dbt);
    begin
        rand_logic_vect_v (ssp_offset, 0.3);
        while (
            (to_integer(unsigned(ssp_offset)) >= bit_time_length - 3)  or -- Account for 3 cycles margin
            (ssp_offset = "00000000")                                  or
            (to_integer(unsigned(ssp_offset)) > 240)
        ) loop
            rand_logic_vect_v (ssp_offset, 0.3);
        end loop;
    end procedure;


    procedure ssp_cfg_ftest_exec(
        signal      chn                     : inout  t_com_channel
    ) is

        -- Generated frames
        variable frame_1                    :     SW_CAN_frame_type;

        -- Node status
        variable stat_1                     :     SW_status;

        variable frame_sent                 :     boolean;

        variable rand_trv_delay             :     natural;
        variable tmp                        :     natural;

        variable ssp_source                 :     SSP_set_command_type;
        variable ssp_offset_var             :     std_logic_vector(7 downto 0);
        variable ssp_pos                    :     natural;

        variable nominal_bus_timing         :     bit_time_config_type;
        variable bus_timing                 :     bit_time_config_type;
        variable num_bit_waits              :     natural;
        variable num_bit_waits_max          :     natural;
        variable bit_rate                   :     real;
        variable nominal_cycles_per_bit     :     integer;
        variable data_cycles_per_bit        :     integer;
    begin

        -----------------------------------------------------------------------
        -- @1. Generate random TRV_DELAY between 0 and 125. Configure it in TB
        --    as delay between CAN TX and CAN RX.
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

        -- Generate random data bit timing!
        rand_int_v(63, bus_timing.prop_dbt);
        rand_int_v(31, bus_timing.ph1_dbt);
        rand_int_v(31, bus_timing.ph2_dbt);

        -- Constrain time quanta to small value for data phase so that we don't have long run times!
        rand_int_v(4, bus_timing.tq_dbt);
        rand_int_v(33, bus_timing.sjw_dbt);

        -- Minimal time quanta
        if (bus_timing.tq_dbt = 0) then
            bus_timing.tq_dbt := 1;
        end if;

        data_cycles_per_bit := bus_timing.tq_dbt *
                                (1 + bus_timing.prop_dbt + bus_timing.ph1_dbt + bus_timing.ph2_dbt);

        -- Constrain minimal bit times
        if (data_cycles_per_bit < 7) then
            bus_timing.prop_dbt := 7;
        end if;

        if (bus_timing.ph2_dbt = 0) then
            bus_timing.ph2_dbt := 1;
        end if;

        if (bus_timing.tq_dbt = 1 and (bus_timing.ph1_dbt + bus_timing.prop_dbt < 2)) then
            bus_timing.ph1_dbt := 1;
            bus_timing.prop_dbt := 1;
        end if;

        if (bus_timing.tq_dbt = 1 and bus_timing.ph2_dbt = 1) then
            bus_timing.ph2_dbt := 2;
        end if;

        if (bus_timing.tq_dbt = 1 and (bus_timing.ph1_dbt + bus_timing.prop_dbt) < 4) then
            bus_timing.ph1_dbt := 1;
            bus_timing.prop_dbt := 2;
        end if;

        data_cycles_per_bit := bus_timing.tq_dbt *
                                (1 + bus_timing.prop_dbt + bus_timing.ph1_dbt + bus_timing.ph2_dbt);

        info_m("Data Cycles per bit:" & integer'image(data_cycles_per_bit));

        info_m("Generated data bit time bit-rate:");
        info_m("TQ: "   & integer'image(bus_timing.tq_dbt));
        info_m("PROP: " & integer'image(bus_timing.prop_dbt));
        info_m("PH1: "  & integer'image(bus_timing.ph1_dbt));
        info_m("PH2: "  & integer'image(bus_timing.ph2_dbt));
        bit_rate := 100000000.0 / (real(data_cycles_per_bit));
        info_m("Data bit rate: " & real'image(bit_rate/1000000.0) & " Mbit/s");

        -- We configure Nominal bit-rate to 500 Kbit/s so that generated
        -- TRV_DELAY will not cause error frames in arbitration bit-rate!
        CAN_configure_timing(bus_timing, DUT_NODE, chn);
        CAN_configure_timing(bus_timing, TEST_NODE, chn);

        -- Generate random transceiver delay
        generate_trv_delay(rand_trv_delay, bus_timing);

        -----------------------------------------------------------------------
        -- @2. Generate random SSP_CFG[SSP_SRC]. If it is offset only, generate
        --    SSP_OFFSET which is higher than TRV_DELAY. If it is
        --    SSP_SRC_MEAS_N_OFFSET, set SSP_OFFSET to random value between 0
        --    and 255. Saturate calculated value of SSP_SRC at 255. If it is
        --    SSP_SRC_NO_SSP, calculate SSP position from regular data-bit rate.
        -----------------------------------------------------------------------
        info_m("Step 2");

        -- Init values
        ssp_offset_var := (OTHERS => '0');
        ssp_source := ssp_meas_n_offset;

        info_m("SSP source:");
        rand_int_v(2, tmp);

        generate_ssp_offset(bus_timing, ssp_offset_var);

        if (tmp = 0) then
            info_m("TRV_DELAY + Offset");
            ssp_source := ssp_meas_n_offset;

            -- Total SSP position needs to be smaller than maximal SSP positon!
            -- Subtract TRV_DELAY until we get there. Generated SSP leaves us
            -- margin, so this is always feasible.
            ssp_pos := to_integer(unsigned(ssp_offset_var)) + rand_trv_delay / 10;
            while (ssp_pos > 252) loop
                rand_trv_delay := rand_trv_delay - 10;
                ssp_pos := to_integer(unsigned(ssp_offset_var)) + rand_trv_delay / 10;
            end loop;

            -- This is to compensate input delay of CTU CAN FD. Actual delay will be
            -- +2 higher! See Datasheet section 2.5.3.
            ssp_pos := ssp_pos + 2;

            -- SSP position is offset + delay
            info_m("Post correction SSP offset: " &
                    integer'image(to_integer(unsigned(ssp_offset_var))));

        elsif (tmp = 1) then
            info_m("NO SSP");
            ssp_source := ssp_no_ssp;

            CAN_read_timing_v(bus_timing, DUT_NODE, chn);
            ssp_pos := bus_timing.tq_dbt *
                        (bus_timing.prop_dbt + bus_timing.ph1_dbt + 1);

            -- In case of no SSP, we sample by regular sample point. Due to this,
            -- we need to shorten trv_delay less than delay of regular sample
            -- point! SP in data sample here is in 20 + 10 + 1 = 31 System clocks.
            -- Consider 2 clock cycle input delay and 1 cycle reserve!
            rand_int_v(280, rand_trv_delay);
            if ((rand_trv_delay / 10) > (ssp_pos - 3)) then
                rand_trv_delay := (ssp_pos - 3) * 10;
            end if;
            if (rand_trv_delay mod 10 = 0) then
                rand_trv_delay := rand_trv_delay + 1;
            end if;

        else
            info_m("Offset only");
            ssp_source := ssp_offset;

            -- Total SSP position needs to be smaller than maximal SSP positon!
            ssp_pos := to_integer(unsigned(ssp_offset_var));
            while (ssp_pos > 252) loop
                ssp_offset_var := std_logic_vector(to_unsigned( to_integer(unsigned(ssp_offset_var)) - 1, 8));
                ssp_pos := to_integer(unsigned(ssp_offset_var));
            end loop;

            -- If we are offset only, we need  to compensate for CTU CAN FD input delay!
            if (ssp_pos < 4) then
                ssp_offset_var := "00000100";
                ssp_pos := to_integer(unsigned(ssp_offset_var));
            end if;

            -- Here trunacte the TRV_DELAY so that the Offset only is larger than the delay!
            -- Otherwise with offset only, we are not able to sample the point!
            while (to_integer(unsigned(ssp_offset_var)) - 3 <= rand_trv_delay / 10) loop
                rand_trv_delay := rand_trv_delay - 10;
            end loop;

            info_m("Post correction SSP offset: " &
                    integer'image(to_integer(unsigned(ssp_offset_var))));
        end if;

        info_m("Random TRV_DELAY is: " & integer'image(rand_trv_delay) & " ns");
        ftr_tb_set_tran_delay((rand_trv_delay * 1 ns), DUT_NODE, chn);

        info_m("SSP position: " & integer'image(ssp_pos));
        CAN_configure_ssp(ssp_source, ssp_offset_var, DUT_NODE, chn);
        CAN_configure_ssp(ssp_source, ssp_offset_var, TEST_NODE, chn);

        CAN_turn_controller(true, DUT_NODE, chn);
        CAN_turn_controller(true, TEST_NODE, chn);

        -- Wait till integration is over!
        CAN_wait_bus_on(DUT_NODE, chn);
        CAN_wait_bus_on(TEST_NODE, chn);

        -----------------------------------------------------------------------
        -- @3. Generate random CAN FD frame with bit-rate shift. Wait until
        --    bit-rate is shifted and wait for random number of bits (but do
        --    not exceed length of data phase). Wait until edge on CAN TX or
        --    CAN RX. Store transmitted value on CAN TX after the edge. Wait
        --    for expected position of Secondary sample point - 3 clock cycle.
        -----------------------------------------------------------------------
        info_m("Step 3");

        CAN_generate_frame(frame_1);
        frame_1.frame_format := FD_CAN;
        frame_1.brs := BR_SHIFT;
        if (frame_1.data_length = 0) then
            frame_1.data_length := 1;
            decode_length(frame_1.data_length, frame_1.dlc);
        end if;

        CAN_send_frame(frame_1, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_pc_state(pc_deb_control, DUT_NODE, chn);

        CAN_wait_not_pc_state(pc_deb_control, DUT_NODE, chn);

        num_bit_waits_max := frame_1.data_length * 8;
        rand_int_v(num_bit_waits_max, num_bit_waits);

        info_m("Frame data length: " & integer'image(frame_1.data_length * 8) &
              " bits");
        info_m("Waiting for: " & integer'image(num_bit_waits) & " bits");
        for i in 0 to num_bit_waits - 1 loop
            CAN_wait_sample_point(DUT_NODE, chn, false);
        end loop;

        -- Wait until SYNC segment. This is 1 clock cycle after start of bit.
        CAN_wait_sync_seg(DUT_NODE, chn);
        wait for (ssp_pos - 2) * 10 ns;

        -----------------------------------------------------------------------
        -- @4. Now we are 3 cycles before Secondary sampling point. Flip bus
        --     to opposite value than is currently received.
        --     This will cause bit error to be detected at nearest SSP.
        -----------------------------------------------------------------------
        info_m("Step 4");

        flip_bus_level(chn);

        -- Now we should be in the cycle where SSP is active!!
        wait for 21 ns;

        -----------------------------------------------------------------------
        -- @5. Wait for one clock cycle and if SSP_CFG[SSP_SRC] = SSP_SRC_NO_SSP,
        --     error frame is being transmitted (regular sample point should be
        --     used to detect bit errors). If SSP_CFG[SSP_SRC] /= SSP_SRC_NO_SSP
        --     check that Error frame is not transmitted and wait until nearest
        --     Sample point. Check that after this Sample point, error frame is
        --     transmitted. Wait until bus is idle in both nodes.
        -----------------------------------------------------------------------
        info_m("Step 5");

        wait for 11 ns;
        release_bus_level(chn);

        if (ssp_source = ssp_no_ssp) then
            wait for 20 ns;
            get_controller_status(stat_1, DUT_NODE, chn);
            check_m(stat_1.error_transmission,
                    "Error frame transmitted with NO_SSP");
        else
            get_controller_status(stat_1, DUT_NODE, chn);
            check_false_m(stat_1.error_transmission,
                          "Error frame NOT transmitted yet!");
            CAN_wait_sample_point(DUT_NODE, chn, false);
            wait for 21 ns;
            get_controller_status(stat_1, DUT_NODE, chn);
            check_m(stat_1.error_transmission,
                    "Error frame transmitted after nearest sample point!");
        end if;

        CAN_wait_bus_idle(DUT_NODE, chn);
        CAN_wait_bus_idle(TEST_NODE, chn);

    end procedure;

end package body;
