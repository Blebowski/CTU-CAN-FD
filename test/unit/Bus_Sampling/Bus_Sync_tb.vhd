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
-- Purpose:
--  Unit test for Bus synchronizer
--------------------------------------------------------------------------------
-- Revision History:
--
--    6.6.2016   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
use work.CANcomponents.ALL;
USE work.CANtestLib.All;
USE work.randomLib.All;
use work.ID_transfer.all;

architecture bus_sync_unit_test of CAN_test is

    -- System clock
    signal clk_sys                  :   std_logic := '0';

    -- Async Reset
    signal res_n                    :   std_logic := '0';

    -- CAN data input from transciever
    signal CAN_rx                   :   std_logic := '0';

    -- CAN data output to transciever
    signal CAN_tx                   :   std_logic := '0';
    signal drv_bus                  :   std_logic_vector(1023 downto 0) :=
                                            (OTHERS => '0');

    -- Sample command for nominal bit time
    signal sample_nbt               :   std_logic := '0';

    -- Sample command for data bit timing
    signal sample_dbt               :   std_logic:= '0';

    -- Synchronisation edge appeared
    signal sync_edge                :   std_logic:= '0';

    -- Transcieve data value
    signal data_tx                  :   std_logic:= '0';

    -- Recieved data value
    signal data_rx                  :   std_logic:= '0';
    signal sp_control               :   std_logic_vector(1 downto 0) :=
                                            (OTHERS => '0');
    signal ssp_reset                :   std_logic:= '0';
    signal trv_delay_calib          :   std_logic:= '0';
    signal bit_err_enable           :   std_logic:= '1';

    -- Secondary sample signal
    signal sample_sec_out           :   std_logic:= '0';

    -- Bit destuffing trigger for secondary sample point
    signal sample_sec_del_1_out     :   std_logic:= '0';

    -- Rec trig for secondary sample point
    signal sample_sec_del_2_out     :   std_logic:= '0';

    signal trv_delay_out            :   std_logic_vector(15 downto 0);
    signal bit_Error                :   std_logic:= '0';

    -- Internal testbench signals
    signal seg1                     :   natural     := 20;
    signal seg2                     :   natural     := 10;
    signal tran_del                 :   natural     := 5;
    signal tran_del_sr              :   std_logic_vector(160 downto 0) :=
                                             (OTHERS => RECESSIVE);
    signal tran_data_sr             :   std_logic_vector(160 downto 0) :=
                                             (OTHERS => RECESSIVE);
    signal tx_trig                  :   std_logic   := '0';
    signal rx_trig                  :   std_logic   := '0';

    -- Additional random counters
    signal rand_ctr_data_gen        :   natural range 0 to RAND_POOL_SIZE := 0;
    signal rand_ctr_noise_gen       :   natural range 0 to RAND_POOL_SIZE := 0;

    signal drv_ena                  :   std_logic;

    signal generate_ones            :   boolean := false;


    ------------------------------------------------------------------------------
    -- Generates bit timing settings with respect to sampling type and
    -- minimal IPT
    ------------------------------------------------------------------------------
    procedure generate_settings(
        signal    rand_ctr          : inout  natural range 0 to RAND_POOL_SIZE;
        signal    tran_del          : inout  natural;
        signal    seg1              : inout  natural;
        signal    seg2              : inout  natural;
        constant  is_secondary      : in     boolean
    ) is
        variable rand_val : real := 0.0;
    begin
        rand_real_v(rand_ctr, rand_val);
        seg1      <= integer(rand_val * 90.0) + 3;
        rand_real_v(rand_ctr, rand_val);
        seg2      <= integer(rand_val * 50.0) + 4;
        rand_real_v(rand_ctr, rand_val);
        tran_del  <= integer(rand_val * 123.0);
        wait for 0 ns;

        if (is_secondary = false and seg1 - 3 < tran_del) then
            tran_del  <= seg1 - 3;
        end if;

        wait for 0 ns;
    end procedure;

begin

    bus_sampling_comp : bus_sampling
    GENERIC map(
        use_Sync              =>  true
    )
    PORT map(
        clk_sys                =>  clk_sys,
        res_n                  =>  res_n ,
        CAN_rx                 =>  CAN_rx,
        CAN_tx                 =>  CAN_tx,
        drv_bus                =>  drv_bus,
        sample_nbt             =>  sample_nbt,
        sample_dbt             =>  sample_dbt,
        sync_edge              =>  sync_edge,
        data_tx                =>  data_tx,
        data_rx                =>  data_rx,
        sp_control             =>  sp_control,
        ssp_reset              =>  ssp_reset,
        trv_delay_calib        =>  trv_delay_calib,
        bit_err_enable         =>  bit_err_enable,
        sample_sec_out         =>  sample_sec_out ,
        sample_sec_del_1_out   =>  sample_sec_del_1_out,
        sample_sec_del_2_out   =>  sample_sec_del_2_out,
        trv_delay_out          =>  trv_delay_out,
        bit_Error              =>  bit_Error
    );

    drv_ena                    <= '1';
    drv_bus(DRV_ENA_INDEX)     <= drv_ena;


    ----------------------------------------------------------------------------
    -- Clock generation
    ----------------------------------------------------------------------------
    clock_gen_proc(period => f100_Mhz, duty => 50, epsilon_ppm => 0,
                   out_clk => clk_sys);


    ----------------------------------------------------------------------------
    -- Sampling signals generation
    ----------------------------------------------------------------------------
    sample_gen : process
        variable min_diff : natural := 3;
    begin
        generate_trig(tx_trig, rx_trig, clk_sys, seg1, seg2);
    end process;

    sample_nbt <= rx_trig;
    sample_dbt <= rx_trig;


    ----------------------------------------------------------------------------
    -- Data generation process
    ----------------------------------------------------------------------------
    data_gen : process
    begin
        wait until falling_edge(clk_sys) and tx_trig = '1' and res_n = '1';

        if (res_n = ACT_RESET) then
            apply_rand_seed(seed, 2, rand_ctr_data_gen);
        end if;

        if (generate_ones = true) then
            data_tx <= RECESSIVE;
        else
            rand_logic_s(rand_ctr_data_gen, data_tx, 0.5);
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Realizing transciever delay
    ----------------------------------------------------------------------------
    tran_del_proc : process
        variable rand_val   : real := 0.0;
        variable rand_min   : real := 0.0;
        variable rand_max   : real := 0.0;
    begin
        wait until rising_edge(clk_sys);

        if (res_n = ACT_RESET) then
            apply_rand_seed(seed, 1, rand_ctr_noise_gen);
        end if;

        tran_del_sr <= tran_del_sr(159 downto 0) & CAN_tx;

        -- Here we generate random glitch on the bus
        rand_real_v(rand_ctr_noise_gen, rand_val);

        ------------------------------------------------------------------------
        -- Add Errors, but rarely...
        -- Note that we have to forbid generating bit flips
        -- during TLD measurment!! If glitch would be generated
        -- here then we could measure wrong TLD! De facto
        -- we would think that glitch is the edge we were
        -- looking for and stop the measurment.
        --
        -- This is weak spot of FD protocol since not only
        -- sampling wrong value in sample point, but
        -- ANY glitch during duration of EDL and r0 bits
        -- will be sampled as edge!
        -- It should be considered to use tripple sampling
        -- and selection from 3 values as in tripple sampling
        -- mode!!!
        ------------------------------------------------------------------------
        if (rand_val > 0.995 and generate_ones = false) then
            rand_real_v(rand_ctr_noise_gen, rand_min);
            rand_real_v(rand_ctr_noise_gen, rand_max);

            if (integer(rand_max * 127.0) < integer(rand_min * 127.0)) then
                rand_max := rand_min;
                end if;

            --Invert random range of bits in shift register...
            tran_del_sr(integer(rand_max * 127.0) downto
                        integer(rand_min * 127.0)) <=
            not tran_del_sr(integer(rand_max * 127.0) downto
                            integer(rand_min * 127.0));
        end if;

        ------------------------------------------------------------------------
        -- Storing the transmitted data for testbench
        ------------------------------------------------------------------------
        tran_data_sr <= tran_data_sr(159 downto 0) & data_tx;
    end process;

    CAN_rx <= tran_del_sr(tran_del);


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Main Test process
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    test_proc : process
        variable outcome   : boolean := false;
        variable rand_val  : real    := 0.0;
    begin
        log("Restarting Bus sampling unit test!", info_l, log_level);
        wait for 5 ns;
        reset_test(res_n, status, run, error_ctr);
        apply_rand_seed(seed, 0, rand_ctr);
        log("Restarted Bus sampling unit test", info_l, log_level);
        print_test_info(iterations, log_level, error_beh, error_tol);

        -------------------------------
        -- Main loop of the test
        -------------------------------
        log("Starting Bus sampling main loop", info_l, log_level);

        while (loop_ctr < iterations  or  exit_imm)
        loop
            log("Starting loop nr " & integer'image(loop_ctr), info_l,
                    log_level);

            --------------------------------------------------------------------
            -- NOMINAL sampling
            --------------------------------------------------------------------

            -- Generates bit time setting
            generate_settings(rand_ctr, tran_del, seg1, seg2, false);
            sp_control <= NOMINAL_SAMPLE;

            -- Wait until there is for sure first bit
            wait for (seg1 + seg2) * 1 ns;

            -- Check bits
            for i in 0 to 50 loop
                wait until rising_edge(rx_trig);
                wait for 20 ns;

                if ((data_tx /= data_rx) and bit_Error = '0') then
                    -- LCOV_EXCL_START
                    process_error(error_ctr, error_beh, exit_imm);
                    log("TX and RX Data mismatch and no bit error fired!",
                      error_l, log_level);
                    -- LCOV_EXCL_STOP
                end if;
            end loop;

            --------------------------------------------------------------------
            -- Data sampling
            --------------------------------------------------------------------

            -- Generates bit time setting
            generate_settings(rand_ctr, tran_del, seg1, seg2, false);
            sp_control <= DATA_SAMPLE;

            -- Wait until there is for sure first bit
            wait for (seg1 + seg2) * 1 ns;

            -- Check the bits
            for i in 0 to 50 loop
                wait until rising_edge(rx_trig);
                wait for 20 ns;

                if ((data_tx /= data_rx) and bit_Error = '0') then
                    -- LCOV_EXCL_START
                    process_error(error_ctr, error_beh, exit_imm);
                    log("TX and RX Data are mismatching and no bit error fired!",
                      error_l, log_level);
                    -- LCOV_EXCL_STOP
                end if;
            end loop;


            --------------------------------------------------------------------
            -- Secondary sampling
            --------------------------------------------------------------------

            -- Generates bit time setting
            generate_settings(rand_ctr, tran_del, seg1, seg2, true);

            --------------------------------------------------------------------
            -- Here we wait until the bus truly works with this delay!!
            -- We want longest possible delay to be really propagated!
            -- Additionally we say that now we generate only Recessive
            -- bits to the BUS. Since now we have arbitrarily fast bit
            -- time in testbench, it can happend that due to too large
            -- delay previous bits were not received yet! Then earlier
            -- edge on RX can be captured and wrong transciever delay
            -- measured!
            -- But in real frame this never happends! Why? because
            -- Delay measurment is executed during NOMINAL bit time.
            -- It is not allowed that NOMINAL bit time is faster than
            -- transciever delay since in NOMINAL bit time we have
            -- to sample in normal sampling point!!! This error happends
            -- only in testbench then! It would be best to generate
            -- data with bit time which is longer than TLD, measure
            -- and then generate data with shorter bit time.
            -- Instead we use this simple workaround that for settling
            -- time we generate only RECESSIVE to avoid any unwanted
            -- errors!
            --------------------------------------------------------------------
            generate_ones <= true;
            wait for 2000 ns;
            generate_ones <= false;

            --We set the circuit to measure transciever delay
            trv_delay_calib <= '1';
            wait until falling_edge(can_tx);

            --Wait until bit is for sure propagated trough the transciever
            -- and delay is measured
            wait for (tran_del + 10) * 10 ns;
            trv_delay_calib <= '0';

            sp_control <= SECONDARY_SAMPLE;

            -- Wait for next bit
            wait until tx_trig'event;

            --Check the bits
            for i in 0 to 50 loop
                wait until rising_edge(sample_sec_out);
                wait for 20 ns;

                -- Here we compare the TX data delayed by Transciever delay
                -- measured amount!
                if ((tran_data_sr(to_integer(unsigned(trv_delay_out))) /=
                     data_rx) and
                     bit_Error='0')
                then
                    -- LCOV_EXCL_START
                    process_error(error_ctr, error_beh, exit_imm);
                    log("TX and RX Data mismatch, and no bit error fired!",
                       error_l, log_level);
                    -- LCOV_EXCL_STOP
                end if;
            end loop;

          loop_ctr <= loop_ctr + 1;
        end loop;

        evaluate_test(error_tol, error_ctr, status);
  end process;

end architecture;
