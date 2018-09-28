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
--  Unit test for the Interrupt manager.
--  Random interrupt source signals are generated in the testbench. Periodically
--  random setting of interrupt generator is used. Then test waits and evaluates
--  whether interrupt prediction (int_test_ctr) matches the actual number of
--  interrupts measured on the int_out rising and falling edges. Also interrupt
--  vector is read and compared with modeled interrupt vector.
--------------------------------------------------------------------------------
-- Revision History:
--    6.6.2016   Created file
--   19.4.2018   Modified testbench to be compliant with separation of interrupt
--               set, Interrupt clear and interrupt mask separation.
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
use work.CANcomponents.ALL;
USE work.CANtestLib.All;
USE work.randomLib.All;
use work.reduce_lib.all;

use work.CAN_FD_register_map.all;
use work.ID_transfer.all;

architecture int_man_unit_test of CAN_test is

    -- System Clock and reset
    signal clk_sys                :   std_logic := '0';
    signal res_n                  :   std_logic := '0';

    ----------------------------------------------
    -- Interrupt inputs
    ----------------------------------------------

    -- Valid Error appeared for interrupt
    signal error_valid            :   std_logic := '0';

    -- Error pasive /Error acitve functionality changed
    signal error_passive_changed  :   std_logic := '0';

    -- Error warning limit reached
    signal error_warning_limit    :   std_logic := '0';

    -- Arbitration was lost input
    signal arbitration_lost       :   std_logic := '0';

    -- Message stored in CAN Core was sucessfully transmitted
    signal tx_finished            :   std_logic := '0';

    -- Bit Rate Was Shifted
    signal br_shifted             :   std_logic := '0';

    -- Income message was discarded
    signal rx_message_disc        :   std_logic := '0';

    -- Message recieved!
    signal rec_message_valid      :   std_logic := '0';

    -- RX Buffer full
    signal rx_full                :   std_logic := '0';

    -- Event logging finsihed
    signal loger_finished         :   std_logic := '0';

    -- RX Buffer not empty
    signal rx_empty               :   std_logic := '1';

    -- HW command on TX Buffer
    signal txt_hw_cmd_int         :   std_logic_vector(TXT_BUFFER_COUNT - 1
                                                        downto 0);

    ----------------------------------------------
    -- Status signals
    ----------------------------------------------
    signal int_ena                :   std_logic_vector(int_count - 1 downto 0);
    signal int_vector             :   std_logic_vector(int_count - 1 downto 0);
    signal int_mask               :   std_logic_vector(int_count - 1 downto 0);

    signal int_out                :   std_logic;

    ----------------------------------------------
    -- Internal testbench signals
    ----------------------------------------------
    signal drv_bus                :   std_logic_vector(1023 downto 0) :=
                                            (OTHERS => '0');

    signal rand_ctr_1             :   natural range 0 to RAND_POOL_SIZE;

    signal error_ctr_2            :   natural;

    constant int_count            :   natural := 12;

    -- Driving signals from memory registers!
    signal drv_int_clear          :   std_logic_vector(int_count - 1 downto 0)
                                          := (OTHERS =>'0');
    signal drv_int_ena_set        :   std_logic_vector(int_count - 1 downto 0)
                                          := (OTHERS =>'0');
    signal drv_int_ena_clear      :   std_logic_vector(int_count - 1 downto 0)
                                          := (OTHERS =>'0');
    signal drv_int_mask_set       :   std_logic_vector(int_count - 1 downto 0)
                                          := (OTHERS =>'0');
    signal drv_int_mask_clear     :   std_logic_vector(int_count - 1 downto 0)
                                          := (OTHERS =>'0');

    -- Expected status, mask and enable (similar as in DUT)
    signal int_ena_exp            :   std_logic_vector(int_count - 1 downto 0)
                                          := (OTHERS =>'0');
    signal int_status_exp         :   std_logic_vector(int_count - 1 downto 0)
                                          := (OTHERS =>'0');
    signal int_mask_exp           :   std_logic_vector(int_count - 1 downto 0)
                                          := (OTHERS =>'0');

    -- Joined input vector
    signal int_input              :   std_logic_vector(int_count - 1 downto 0)
                                          := (OTHERS =>'0');


    ----------------------------------------------------------------------------
    -- Generates random interrupt sources
    ----------------------------------------------------------------------------
    procedure generate_sources(
        signal rand_ctr               :inout   natural range 0 to RAND_POOL_SIZE;

        -- Valid Error appeared for interrupt
        signal error_valid            :inout   std_logic;

        -- Error pasive /Error acitve functionality changed
        signal error_passive_changed  :inout   std_logic;

        -- Error warning limit reached
        signal error_warning_limit    :inout   std_logic;

        -- Arbitration was lost input
        signal arbitration_lost       :inout   std_logic;

        -- Message stored in CAN Core was sucessfully transmitted
        signal tx_finished            :inout   std_logic;

        -- Bit Rate Was Shifted
        signal br_shifted             :inout   std_logic;

        -- Income message was discarded
        signal rx_message_disc        :inout   std_logic;

        -- Message recieved!
        signal rec_message_valid      :inout   std_logic;

        -- RX Buffer full
        signal rx_full                :inout   std_logic;

        -- Event loggging finished
        signal loger_finished         :inout   std_logic;

        -- RX Buffer empty
        signal rx_empty               :inout   std_logic;

        -- TXT HW Command
        signal txt_hw_cmd_int         :inout   std_logic_vector(TXT_BUFFER_COUNT - 1
                                                                downto 0)
    )is
        variable tmp                  :        std_logic;
    begin
        if (error_valid = '1') then
            rand_logic_s(rand_ctr, error_valid, 0.85);
        else
            rand_logic_s(rand_ctr, error_valid, 0.1);
        end if;

        if (error_passive_changed = '1') then
            rand_logic_s(rand_ctr, error_passive_changed, 0.85);
        else
            rand_logic_s(rand_ctr, error_passive_changed, 0.05);
        end if;

        if (error_warning_limit = '1') then
            rand_logic_s(rand_ctr, error_warning_limit, 0.85);
        else
            rand_logic_s(rand_ctr, error_warning_limit, 0.05);
        end if;

        if (arbitration_lost = '1') then
            rand_logic_s(rand_ctr, arbitration_lost, 0.95);
        else
            rand_logic_s(rand_ctr, arbitration_lost, 0.05);
        end if;

        if (tx_finished = '1') then
            rand_logic_s(rand_ctr, tx_finished, 0.95);
        else
            rand_logic_s(rand_ctr, tx_finished, 0.05);
        end if;

        if (br_shifted = '1') then
            rand_logic_s(rand_ctr, br_shifted, 0.95);
        else
            rand_logic_s(rand_ctr, br_shifted, 0.05);
        end if;

        if (rx_message_disc = '1') then
            rand_logic_s(rand_ctr, rx_message_disc, 0.95);
        else
            rand_logic_s(rand_ctr, rx_message_disc, 0.05);
        end if;

        if (rec_message_valid = '1') then
            rand_logic_s(rand_ctr, rec_message_valid, 0.95);
        else
            rand_logic_s(rand_ctr, rec_message_valid, 0.05);
        end if;

        if (rx_full = '1') then
            rand_logic_s(rand_ctr, rx_full, 0.95);
        else
            rand_logic_s(rand_ctr, rx_full, 0.05);
        end if;

        if (loger_finished = '1') then
            rand_logic_s(rand_ctr, loger_finished, 0.95);
        else
            rand_logic_s(rand_ctr, loger_finished, 0.05);
        end if;

        if (rx_empty = '0') then
            rand_logic_s(rand_ctr, rx_empty, 0.95);
        else
            rand_logic_s(rand_ctr, rx_empty, 0.05);
        end if;

        for i in 0 to TXT_BUFFER_COUNT - 1 loop
            if (txt_hw_cmd_int(i) = '1') then
                rand_logic_v(rand_ctr, tmp, 0.95);
            else
                rand_logic_v(rand_ctr, tmp, 0.05);
            end if;
            txt_hw_cmd_int(i) <= tmp;
        end loop;

    end procedure;


    ----------------------------------------------------------------------------
    -- Generates interrupt commands as if comming on driving bus from memory
    -- registers!
    ----------------------------------------------------------------------------
    procedure generate_commands(
        signal drv_int_clear       :out std_logic_vector(int_count - 1 downto 0);
        signal drv_int_ena_set     :out std_logic_vector(int_count - 1 downto 0);
        signal drv_int_ena_clear   :out std_logic_vector(int_count - 1 downto 0);
        signal drv_int_mask_set    :out std_logic_vector(int_count - 1 downto 0);
        signal drv_int_mask_clear  :out std_logic_vector(int_count - 1 downto 0);
        signal rand_ctr            :inout natural range 0 to RAND_POOL_SIZE
    ) is
        variable tmp             : real;
    begin
        rand_real_v(rand_ctr, tmp);

        -- Erase the commands by default!
        drv_int_clear            <= (OTHERS => '0');
        drv_int_ena_set          <= (OTHERS => '0');
        drv_int_ena_clear        <= (OTHERS => '0');
        drv_int_mask_set         <= (OTHERS => '0');
        drv_int_mask_clear       <= (OTHERS => '0');

        -- Only one command is generated at any time, since commands are
        -- coming from different registers!
        if (tmp < 0.2) then
            rand_logic_vect_s(rand_ctr, drv_int_clear, 0.4);

        elsif (tmp < 0.4) then
            rand_logic_vect_s(rand_ctr, drv_int_ena_set, 0.2);

        elsif (tmp < 0.6) then
            rand_logic_vect_s(rand_ctr, drv_int_ena_clear, 0.4);

        elsif (tmp < 0.8) then
            rand_logic_vect_s(rand_ctr, drv_int_mask_set, 0.2);

        else
            rand_logic_vect_s(rand_ctr, drv_int_mask_clear, 0.4);
        end if;

        wait for 0 ns;

    end procedure;

begin

    ----------------------------------------------------------------------------
    -- DUT
    ----------------------------------------------------------------------------
    int_man_comp : intManager
    GENERIC map(
        int_count             => int_count
    )
    PORT map(
        clk_sys               =>   clk_sys,
        res_n                 =>   res_n,
        error_valid           =>   error_valid,
        error_passive_changed =>   error_passive_changed,
        error_warning_limit   =>   error_warning_limit,
        arbitration_lost      =>   arbitration_lost,
        tx_finished           =>   tx_finished ,
        br_shifted            =>   br_shifted,
        rx_empty              =>   rx_empty,
        txt_hw_cmd_int        =>   txt_hw_cmd_int,
        rx_message_disc       =>   rx_message_disc ,
        rec_message_valid     =>   rec_message_valid ,
        rx_full               =>   rx_full,
        loger_finished        =>   loger_finished,
        drv_bus               =>   drv_bus ,
        int_out               =>   int_out,
        int_vector            =>   int_vector,
        int_mask              =>   int_mask,
        int_ena               =>   int_ena
    );

    -- Joining interrupt inputs to interrupt status
    int_input(BEI_IND)            <=  error_valid;
    int_input(ALI_IND)            <=  arbitration_lost;
    int_input(EPI_IND)            <=  error_passive_changed;
    int_input(DOI_IND)            <=  rx_message_disc;
    int_input(EI_IND)             <=  error_warning_limit;
    int_input(TI_IND)             <=  tx_finished;
    int_input(RI_IND)             <=  rec_message_valid;
    int_input(LFI_IND)            <=  loger_finished;
    int_input(RFI_IND)            <=  rx_full;
    int_input(BSI_IND)            <=  br_shifted;
    int_input(RBNEI_IND)          <=  not rx_empty;
    int_input(TXBHCI_IND)         <=  or_reduce(txt_hw_cmd_int);


    ----------------------------------------------------------------------------
    -- Clock generation
    ----------------------------------------------------------------------------
    clock_gen_proc(period => f100_Mhz, duty => 50, epsilon_ppm => 0,
                   out_clk => clk_sys);


    ----------------------------------------------------------------------------
    -- Generating random sources
    ----------------------------------------------------------------------------
    src_gen : process
    begin
        wait for 195 ns;
        if (res_n = ACT_RESET) then
            apply_rand_seed(seed, 1, rand_ctr_1);
        end if;

        while true loop
            wait until falling_edge(clk_sys);
            generate_sources(rand_ctr_1, error_valid, error_passive_changed ,
                           error_warning_limit , arbitration_lost, tx_finished,
                           br_shifted, rx_message_disc , rec_message_valid ,
                           rx_full , loger_finished, rx_empty, txt_hw_cmd_int);
        end loop;
    end process;


    ----------------------------------------------------------------------------
    -- Connection to Driving bus
    ----------------------------------------------------------------------------
    drv_bus(DRV_INT_CLR_HIGH downto DRV_INT_CLR_LOW)
            <= drv_int_clear;

    drv_bus(DRV_INT_ENA_SET_HIGH downto DRV_INT_ENA_SET_LOW)
            <= drv_int_ena_set;

    drv_bus(DRV_INT_ENA_CLR_HIGH downto DRV_INT_ENA_CLR_LOW)
            <= drv_int_ena_clear;

    drv_bus(DRV_INT_MASK_SET_HIGH downto DRV_INT_MASK_SET_LOW)
            <= drv_int_mask_set;

    drv_bus(DRV_INT_MASK_CLR_HIGH downto DRV_INT_MASK_CLR_LOW)
            <= drv_int_mask_clear;


    ----------------------------------------------------------------------------
    -- Calculate expected outputs
    ----------------------------------------------------------------------------
    int_emu_proc : process
        variable int_output     : boolean;
        variable outcome        : boolean;
        variable exp_output     : boolean;
        constant zeroes         : std_logic_vector(int_count - 1 downto 0) :=
                                      (OTHERS => '0');
    begin

        while (run = false) loop
            wait until rising_edge(clk_sys);
        end loop;

        wait until rising_edge(clk_sys);

        outcome := true;

        for i in 0 to int_count - 1 loop

            -- Interrupt enable
            if (drv_int_ena_set(i) = '1') then
                int_ena_exp(i) <= '1';
            elsif (drv_int_ena_clear(i) = '1') then
                int_ena_exp(i) <= '0';
            end if;

            -- Interrupt mask
            if (drv_int_mask_set(i) = '1') then
                int_mask_exp(i) <= '1';
            elsif (drv_int_mask_clear(i) = '1') then
                int_mask_exp(i) <= '0';
            end if;

            -- Interrupt clear and capturing!
            if (int_input(i) = '1' and int_mask_exp(i) = '0') then
                int_status_exp(i) <= '1';
            elsif (drv_int_clear(i) = '1') then
                int_status_exp(i) <= '0';
            end if;
        end loop;

        -- Calculating expected interrupt output
        if ((int_vector AND int_ena_exp) = zeroes) then
            exp_output      := false;
        else
            exp_output      := true;
        end if;

        -- Checking the expected and real outputs
        if (int_ena         /= int_ena_exp) then
            -- LCOV_EXCL_START
            outcome         := false;
            log("Interrupt enable mismatch", error_l, log_level);
            -- LCOV_EXCL_STOP
        end if;

        if (int_mask        /= int_mask_exp) then
            -- LCOV_EXCL_START
            outcome         := false;
            log("Interrupt mask mismatch", error_l, log_level);
            -- LCOV_EXCL_STOP
        end if;

        if (int_vector      /= int_status_exp) then
            -- LCOV_EXCL_START
            outcome         := false;
            log("Interrupt vector mismatch", error_l, log_level);
            -- LCOV_EXCL_STOP
        end if;

        if ((exp_output = true  and int_out = '0') or
            (exp_output = false and int_out = '1'))
        then
            -- LCOV_EXCL_START
            outcome         := false;
            log("Interrupt output mismatch", error_l, log_level);
            -- LCOV_EXCL_STOP
        end if;

        -- Checking the outputs
        if (outcome = false) then
            -- LCOV_EXCL_START
            process_error(error_ctr_2, error_beh, exit_imm);
            -- LCOV_EXCL_STOP
        end if;

    end process;

    -- Error propagation to the output
    errors <= error_ctr;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Main Test process
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    test_proc : process
        variable outcome : boolean := false;
    begin
        log("Restarting Interrupt test!", info_l, log_level);
        wait for 5 ns;
        reset_test(res_n, status, run, error_ctr);
        apply_rand_seed(seed, 1, rand_ctr);
        log("Restarted Interrupttest", info_l, log_level);
        print_test_info(iterations, log_level, error_beh, error_tol);

        -------------------------------
        -- Main loop of the test
        -------------------------------
        log("Starting Interrupt main loop", info_l, log_level);

        while (loop_ctr < iterations  or  exit_imm)
        loop
              log("Starting loop nr "&integer'image(loop_ctr), info_l,
                    log_level);

              wait until falling_edge(clk_sys);

              -- Generate commands as coming from user registers
              generate_commands(drv_int_clear, drv_int_ena_set,
                                drv_int_ena_clear, drv_int_mask_set,
                                drv_int_mask_clear, rand_ctr);
              wait for 50 ns;

              -- Errors are evaluated in separate process.
              error_ctr <= error_ctr_2;

              loop_ctr <= loop_ctr + 1;
        end loop;

        evaluate_test(error_tol, error_ctr, status);
    end process;

end architecture;
