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
--  @4. Now we are 3 clock cycles before Secondary sampling point. Force bus to
--      opposite value than was sent.
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
    
    procedure correct_ssp_offset(
       ssp_offset_generated         : in    std_logic_vector(7 downto 0);
       bus_timing                   : in    bit_time_config_type;
       ssp_offset_corrected         : out   std_logic_vector(7 downto 0)
    );

end package;


package body ssp_cfg_ftest is

    ---------------------------------------------------------------------------
    -- Data bit time is generated random. Also SSP_OFFSET is generated random.
    -- We set real delay from CAN_TX to CAN_RX in TB also to random generated
    -- value. If generated SSP_OFFSET is higher than duration of bit, then
    -- we will never sample correct value, because we just sample next bit
    -- already! So we need to constrain configured SSP_OFFSET to less than
    -- data bit time!
    --------------------------------------------------------------------------- 
    procedure correct_ssp_offset(
       ssp_offset_generated         : in    std_logic_vector(7 downto 0);
       bus_timing                   : in    bit_time_config_type;
       ssp_offset_corrected         : out   std_logic_vector(7 downto 0)
    ) is
        variable bit_time_length    :       natural;
    begin
        bit_time_length := bus_timing.tq_dbt * (1 + bus_timing.prop_dbt +
            bus_timing.ph1_dbt + bus_timing.ph2_dbt);

        if (to_integer(unsigned(ssp_offset_generated)) >= bit_time_length) then
            ssp_offset_corrected := std_logic_vector(to_unsigned(bit_time_length - 1, 8));
            info_m("Correcting SSP offset. Bit time length: " &
                  integer'image(bit_time_length) & " cycles. New SSP offset value:" &
                  integer'image(to_integer(unsigned(ssp_offset_corrected))));
        else
            ssp_offset_corrected := ssp_offset_generated;
        end if;

    end procedure;

    procedure ssp_cfg_ftest_exec(
        signal      chn             : inout  t_com_channel
    ) is

        -- Generated frames
        variable frame_1            :     SW_CAN_frame_type;

        -- Node status
        variable stat_1             :     SW_status;
    
        variable frame_sent         :     boolean;        
        
        variable rand_trv_delay     :     natural;
        variable tmp                :     natural;
        
        variable ssp_source         :     SSP_set_command_type;
        variable ssp_offset_var     :     std_logic_vector(7 downto 0);
        variable ssp_pos            :     natural;

        variable bus_timing         :     bit_time_config_type;
        variable num_bit_waits      :     natural;
        variable num_bit_waits_max  :     natural;
        variable tx_val             :     std_logic;
        variable bit_rate           :     real;
        variable cycles_per_bit     :     integer;
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
        
        -- Constrain time quanta to something realistinc for data phase so
        -- that we don't have too long run times!
        rand_int_v(4, bus_timing.tq_dbt);
        rand_int_v(33, bus_timing.sjw_dbt);
        
        -- Minimal time quanta
        if (bus_timing.tq_dbt = 0) then
            bus_timing.tq_dbt := 1;
        end if;
        
        cycles_per_bit := bus_timing.tq_dbt * (1 + bus_timing.prop_dbt +
                            bus_timing.ph1_dbt + bus_timing.ph2_dbt);
        
        -- Constrain minimal bit times
        if (cycles_per_bit < 7) then
            bus_timing.prop_dbt := 7;
        end if;

        if (bus_timing.tq_dbt = 1 and bus_timing.ph2_dbt = 1) then
            bus_timing.ph2_dbt := 2;
        end if;
        
        cycles_per_bit := bus_timing.tq_dbt * (1 + bus_timing.prop_dbt +
                            bus_timing.ph1_dbt + bus_timing.ph2_dbt);
        info_m("Cycles per bit:" & integer'image(cycles_per_bit));

        info_m("Generated data bit time bit-rate:");
        info_m("TQ: " & integer'image(bus_timing.tq_dbt));
        info_m("PROP: " & integer'image(bus_timing.prop_dbt));
        info_m("PH1: " & integer'image(bus_timing.ph1_dbt));
        info_m("PH2: " & integer'image(bus_timing.ph2_dbt));
        bit_rate := 100000000.0 / (real(cycles_per_bit));
        info_m("Data bit rate: " & real'image(bit_rate/1000000.0) & " Mbit/s");

        -- We configure Nominal bit-rate to 500 Kbit/s so that generated
        -- TRV_DELAY will not cause error frames in arbitration bit-rate!
        CAN_configure_timing(bus_timing, DUT_NODE, chn);
        CAN_configure_timing(bus_timing, TEST_NODE, chn);

        rand_int_v(1259, rand_trv_delay);
        if (rand_trv_delay = 0) then
            rand_trv_delay := 1;
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
        if (rand_trv_delay mod 10 = 0) then
            rand_trv_delay := rand_trv_delay + 1;
        end if;

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
        if (tmp = 0) then
            info_m("TRV_DELAY + Offset");
            ssp_source := ssp_meas_n_offset;
            rand_logic_vect_v (ssp_offset_var, 0.3);
            info_m("Generated SSP offset: " & integer'image(to_integer(unsigned(ssp_offset_var))));
            
            correct_ssp_offset(ssp_offset_var, bus_timing, ssp_offset_var);
            
            -- SSP position is offset + delay
            info_m("Post correction SSP offset: " & integer'image(to_integer(unsigned(ssp_offset_var))));
            info_m("Trv delay div: " & integer'image(rand_trv_delay / 10));
            
            ssp_pos := to_integer(unsigned(ssp_offset_var)) + rand_trv_delay / 10;
            if (ssp_pos > 255) then
                ssp_pos := 255;
            end if;
            
            -- This is to compensate input delay of CTU CAN FD! See Datasheet.
            -- section 2.5.3. This applies only for case without Datasheet
            ssp_pos := ssp_pos + 2;

        elsif (tmp = 1) then
            info_m("NO SSP");
            ssp_source := ssp_no_ssp;

            CAN_read_timing_v(bus_timing, DUT_NODE, chn);
            ssp_pos := bus_timing.tq_dbt *
                        (bus_timing.prop_dbt + bus_timing.ph1_dbt + 1);
                        
                        
            -- In case of no SSP, we sample by regular sample point. Due to this,
            -- we need to shorten trvdelay to less than delay of regular sample
            -- point! SP in data sample here is in 20 + 10 + 1 = 31 System clocks.
            -- Consider 2 clock cycle input delay and 1 cycle reserve!
            rand_int_v(280, rand_trv_delay);
            if (rand_trv_delay mod 10 = 0) then
                rand_trv_delay := rand_trv_delay + 1;
            end if;
        else
            info_m("Offset only");
            ssp_source := ssp_offset;
            rand_logic_vect_v (ssp_offset_var, 0.3);
            
            correct_ssp_offset(ssp_offset_var, bus_timing, ssp_offset_var);
            
            info_m("Post correction SSP offset: " & integer'image(to_integer(unsigned(ssp_offset_var))));

            -- Here lengthen the SSP offset so that we are sufficiently over TRV_DELAY!
            -- It should be enough to lengthen it by two clock cycles (input delay of
            -- CTU CAN FD) + one cycle reserve for truncation of non-multiple of 10
            -- divided by 10!
            if (to_integer(unsigned(ssp_offset_var)) <= rand_trv_delay/10) then
                ssp_offset_var := std_logic_vector(to_unsigned(rand_trv_delay/10, 8) + 3);
            end if;
            
            -- SSP position is offset only!
            ssp_pos := to_integer(unsigned(ssp_offset_var));
            if (ssp_pos > 255) then
                ssp_pos := 255;
            end if;
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

        CAN_send_frame(frame_1, 1, DUT_NODE, chn, frame_sent);
        CAN_wait_pc_state(pc_deb_control, DUT_NODE, chn);
        
        CAN_wait_not_pc_state(pc_deb_control, DUT_NODE, chn);

        -- +10 is to cover some part of CRC
        num_bit_waits_max := frame_1.data_length * 8 + 10 ;
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
        -- @4. Now we are 3 cycles before Secondary sampling point. Force bus
        --     to opposite value than was sent.
        -----------------------------------------------------------------------
        info_m("Step 4");
        force_bus_level(not tx_val, chn);        

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
