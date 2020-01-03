--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018
-- 
-- Authors:
--     Ondrej Ille <ondrej.ille@gmail.com>
--     Martin Jerabek <martin.jerabek01@gmail.com>
-- 
-- Project advisors: 
--  Jiri Novak <jnovak@fel.cvut.cz>
--  Pavel Pisa <pisa@cmp.felk.cvut.cz>
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
-- Purpose:
--  SSP_CFG register feature test.
--
-- Verifies:
--  1. When SSP_CFG[SSP_SRC] = SSP_OFFSET, position of secondary sampling point
--     will be given only by SSP_OFFSET.
--  2. When SSP_CFG[SSP_SRC] = SSP_SRC_NO_SSP, there will be no SSP and regular
--     sample point will be used to detect bit error by bit-error detector!
--  3. When SSP_CFG[SSP_SRC] = SSP_SRC_MEAS_N_OFFSET, position of secondary
--     sampling point will be given as SSP_OFFSET + TRV_DELAY.
--  4. Position of Secondary sampling point is saturated to 255.
--  5. Transmitter detecting bit error in SSP will transmitt error frame at
--     nearest regular sample point, not earlier!
--
-- Test sequence:
--  1. Generate random TRV_DELAY between 0 and 125. Configure it in TB as delay
--     between CAN TX and CAN RX.
--  2. Generate random SSP_CFG[SSP_SRC]. If it is offset only, generate
--     SSP_OFFSET which is higher than TRV_DELAY. If it is SSP_SRC_MEAS_N_OFFSET,
--     set SSP_OFFSET to random value between 0 and 255. Saturate calculated
--     value of SSP_SRC at 255. If it is SSP_SRC_NO_SSP, calculate SSP position
--     from regular data-bit rate.
--  3. Generate random CAN FD frame with bit-rate shift. Wait until bit-rate is
--     shifted and wait for random number of bits (but do not exceed length of
--     data phase). Wait until edge on CAN TX or CAN RX. Store transmitted value
--     on CAN TX after the edge. Wait for expected position of Secondary sample
--     point - 3 clock cycles.
--  4. Now we are 3 clock cycles before Secondary sampling point. Force bus to
--     opposite value than was sent. Check that Secondary sample point is active
--     (via Status Bus), if SSP_CFG[SSP_SRC] /= SSP_SRC_NO_SSP. Check that it is
--     not active when SSP_CFG[SSP_SRC] = SSP_SRC_NO_SSP.
--  5. Wait for one clock cycle and if SSP_CFG[SSP_SRC] = SSP_SRC_NO_SSP, error
--     frame is being transmitted (regular sample point should be used to detect
--     bit errors). If SSP_CFG[SSP_SRC] /= SSP_SRC_NO_SSP check that Error frame
--     is not transmitted and wait until nearest Sample point. Check that after
--     this sample point, error frame is transmitted. Wait until bus is idle in
--     both nodes.
--------------------------------------------------------------------------------
-- Revision History:
--    02.1.2020   Created file
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

use lib.pkg_feature_exec_dispath.all;

package ssp_cfg_feature is
    procedure ssp_cfg_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    );
end package;


package body ssp_cfg_feature is
    procedure ssp_cfg_feature_exec(
        signal      so              : out    feature_signal_outputs_t;
        signal      rand_ctr        : inout  natural range 0 to RAND_POOL_SIZE;
        signal      iout            : in     instance_outputs_arr_t;
        signal      mem_bus         : inout  mem_bus_arr_t;
        signal      bus_level       : in     std_logic
    ) is
        variable ID_1               :     natural := 1;
        variable ID_2               :     natural := 2;

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

    begin

        -----------------------------------------------------------------------
        -- 1. Generate random TRV_DELAY between 0 and 125. Configure it in TB
        --    as delay between CAN TX and CAN RX.
        -----------------------------------------------------------------------
        info("Step 1");

        CAN_turn_controller(false, ID_1, mem_bus(1));
        CAN_turn_controller(false, ID_2, mem_bus(2));

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

        -- We configure Nominal bit-rate to 500 Kbit/s so that generated
        -- TRV_DELAY will not cause error frames in arbitration bit-rate!
        CAN_configure_timing(bus_timing, ID_1, mem_bus(1));
        CAN_configure_timing(bus_timing, ID_2, mem_bus(2));

        rand_int_v(rand_ctr, 1259, rand_trv_delay);
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
        -- 2. Generate random SSP_CFG[SSP_SRC]. If it is offset only, generate
        --    SSP_OFFSET which is higher than TRV_DELAY. If it is
        --    SSP_SRC_MEAS_N_OFFSET, set SSP_OFFSET to random value between 0
        --    and 255. Saturate calculated value of SSP_SRC at 255. If it is
        --    SSP_SRC_NO_SSP, calculate SSP position from regular data-bit rate.
        -----------------------------------------------------------------------
        info("Step 2");

        -- Init values
        ssp_offset_var := (OTHERS => '0');
        ssp_source := ssp_meas_n_offset;

        info("SSP source:");
        rand_int_v(rand_ctr, 2, tmp);
        if (tmp = 0) then
            info("TRV_DELAY + Offset");
            ssp_source := ssp_meas_n_offset;
            rand_logic_vect_v (rand_ctr, ssp_offset_var, 0.3);
            
            -- SSP position is offset + delay
            info("SSP offset: " & integer'image(to_integer(unsigned(ssp_offset_var))));
            info("Trv delay div: " & integer'image(rand_trv_delay / 10));
            
            ssp_pos := to_integer(unsigned(ssp_offset_var)) + rand_trv_delay / 10;
            if (ssp_pos > 255) then
                ssp_pos := 255;
            end if;
            
            -- This is to compensate input delay of CTU CAN FD! See Datasheet.
            -- section 2.5.3. This applies only for case without Datasheet
            ssp_pos := ssp_pos + 2;

        elsif (tmp = 1) then
            info("NO SSP");
            ssp_source := ssp_no_ssp;

            CAN_read_timing_v(bus_timing, ID_1, mem_bus(1));
            ssp_pos := bus_timing.tq_dbt *
                        (bus_timing.prop_dbt + bus_timing.ph1_dbt + 1);
                        
                        
            -- In case of no SSP, we sample by regular sample point. Due to this,
            -- we need to shorten trvdelay to less than delay of regular sample
            -- point! SP in data sample here is in 20 + 10 + 1 = 31 System clocks.
            -- Consider 2 clock cycle input delay and 1 cycle reserve!
            rand_int_v(rand_ctr, 280, rand_trv_delay);
            if (rand_trv_delay mod 10 = 0) then
                rand_trv_delay := rand_trv_delay + 1;
            end if;
        else
            info("Offset only");
            ssp_source := ssp_offset;
            rand_logic_vect_v (rand_ctr, ssp_offset_var, 0.3);
            
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

        info("Random TRV_DELAY is: " & integer'image(rand_trv_delay) & " ns");
        ftr_tb_set_tran_delay((rand_trv_delay * 1 ns), ID_1, so.ftr_tb_trv_delay);

        info("SSP position: " & integer'image(ssp_pos));
        CAN_configure_ssp(ssp_source, ssp_offset_var, ID_1, mem_bus(1));
        CAN_configure_ssp(ssp_source, ssp_offset_var, ID_2, mem_bus(2));
        
        CAN_turn_controller(true, ID_1, mem_bus(1));
        CAN_turn_controller(true, ID_2, mem_bus(2));
        
        -- Wait till integration is over!
        CAN_wait_bus_on(ID_1, mem_bus(1));
        CAN_wait_bus_on(ID_2, mem_bus(2));

        -----------------------------------------------------------------------
        -- 3. Generate random CAN FD frame with bit-rate shift. Wait until
        --    bit-rate is shifted and wait for random number of bits (but do
        --    not exceed length of data phase). Wait until edge on CAN TX or
        --    CAN RX. Store transmitted value on CAN TX after the edge. Wait
        --    for expected position of Secondary sample point - 3 clock cycle.
        -----------------------------------------------------------------------
        info("Step 3");

        CAN_generate_frame(rand_ctr, frame_1);
        frame_1.frame_format := FD_CAN;
        frame_1.brs := BR_SHIFT;

        CAN_send_frame(frame_1, 1, ID_1, mem_bus(1), frame_sent);
        CAN_wait_pc_state(pc_deb_control, ID_1, mem_bus(1));
        CAN_wait_not_pc_state(pc_deb_control, ID_1, mem_bus(1));

        -- +10 is to cover some part of CRC
        num_bit_waits_max := frame_1.data_length * 8 + 10 ;
        rand_int_v(rand_ctr, num_bit_waits_max, num_bit_waits);

        info ("Frame data length: " & integer'image(frame_1.data_length * 8) &
              " bits");
        info ("Waiting for: " & integer'image(num_bit_waits) & " bits");
        for i in 0 to num_bit_waits - 1 loop
            CAN_wait_sample_point(iout(1).stat_bus, false);
        end loop;
        
        -- Wait until edge is transmitted (for sure start of bit) and then
        -- until expected sample point
        wait until rising_edge(iout(1).can_tx) or falling_edge(iout(1).can_tx);
        wait for 1 ps;
        tx_val := iout(1).can_tx;
        wait for (ssp_pos - 3) * 10 ns;

        -----------------------------------------------------------------------
        -- 4. Now we are 3 cycles before Secondary sampling point. Force bus
        --    to opposite value than was sent. Check that Secondary sample point
        --    is active (via Status Bus), if SSP_CFG[SSP_SRC] /= SSP_SRC_NO_SSP.
        --    Check that it is not active when SSP_CFG[SSP_SRC] = SSP_SRC_NO_SSP.
        -----------------------------------------------------------------------
        info("Step 4");
        force_bus_level(not tx_val, so.bl_force, so.bl_inject);        

        -- Now we should be in the cycle where SSP is active!!
        wait for 21 ns;

        if (ssp_source = ssp_no_ssp) then
            check(iout(1).stat_bus(STAT_SAMPLE_SEC) = '0',
                "SSP Sample point NOT active!");
        else
            check(iout(1).stat_bus(STAT_SAMPLE_SEC) = '1',
                "SSP Sample point active!");
        end if;
        
        -----------------------------------------------------------------------
        -- 5. Wait for one clock cycle and if SSP_CFG[SSP_SRC] = SSP_SRC_NO_SSP,
        --    error frame is being transmitted (regular sample point should be
        --    used to detect bit errors). If SSP_CFG[SSP_SRC] /= SSP_SRC_NO_SSP
        --    check that Error frame is not transmitted and wait until nearest
        --    Sample point. Check that after this Sample point, error frame is
        --    transmitted. Wait until bus is idle in both nodes.
        -----------------------------------------------------------------------
        info("Step 5");

        wait for 11 ns;
        
        if (ssp_source = ssp_no_ssp) then
            wait for 20 ns;
            get_controller_status(stat_1, ID_1, mem_bus(1));
            check (stat_1.error_transmission,
                    "Error frame transmitted with NO_SSP");
        else
            get_controller_status(stat_1, ID_1, mem_bus(1));
            check_false (stat_1.error_transmission,
                    "Error frame NOT transmitted yet!");
            CAN_wait_sample_point(iout(1).stat_bus, false);
            wait for 21 ns;
            get_controller_status(stat_1, ID_1, mem_bus(1));
            check (stat_1.error_transmission,
                    "Error frame transmitted after nearest sample point!");
        end if;

        release_bus_level(so.bl_force);

        CAN_wait_bus_idle(ID_1, mem_bus(1));
        CAN_wait_bus_idle(ID_2, mem_bus(2));

    end procedure;

end package body;
