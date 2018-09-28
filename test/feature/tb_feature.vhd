--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018
-- 
-- Authors:
--     Ondrej Ille <ondrej.ille@gmail.com>
--     Martin Jerabek <jerabma7@fel.cvut.cz>
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
--  Main environment for feature tests
--
--------------------------------------------------------------------------------
-- Revision History:
--    20.6.2016   Created file
--    June 2018   Major rewrite for support of new unified testing framework.
--------------------------------------------------------------------------------


Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
use work.CANcomponents.ALL;
USE work.CANtestLib.All;
USE work.randomLib.All;
USE work.pkg_feature_exec_dispath.All;

use work.ID_transfer.all;
--------------------------------------------------------------------------------
-- Test enity for feature tests. Additional signals representing two memory
-- buses are present to connect two DUTs of feature tests!
--------------------------------------------------------------------------------
entity CAN_feature_test is
    port (
        -- Input trigger, test starts running when true
        signal run              : in    boolean;

        -- Number of iterations that test should do
        signal iterations       : in    natural;

        -- Logging level, severity which should be shown
        signal log_level        : in    log_lvl_type;

        -- Test behaviour when error occurs: Quit, or Go on
        signal error_beh        : in    err_beh_type;

        -- Error tolerance, error counter should not
        -- exceed this value in order for the test to pass
        signal error_tol        : in    natural;

        -- Status of the test
        signal status           : out   test_status_type;

        -- Memory access buses
        signal mem_bus          : inout mem_bus_arr_t;

        -- Bus level injected value and whether it should be forced on bus
        signal bl_inject        : in    std_logic;
        signal bl_force         : in    boolean;

        -- Internal signals; TODO: direction
        signal iteration_done   : in boolean := false;
        signal hw_reset_on_new_test         : in boolean := true;

        signal iout             : out instance_outputs_arr_t;

        --CAN bus signals
        signal bus_level        : out std_logic := RECESSIVE;

        --Test name to be loaded by the TCL script from TCL test FIFO
        --Note that string always have to have fixed length
        signal test_name        : in string (1 to 20) := strtolen(20, "overload")
    );

    -- Internal test signals
    signal error_ctr            :       natural :=  0;
    signal loop_ctr             :       natural :=  0;
    signal exit_imm             :       boolean :=  false;
end entity;

architecture feature_env_test of CAN_feature_test is
    type instance_signals_t is record
        tr_del          : time;
        tr_delayed      : std_logic;
        clk_sys         : std_logic;
        res_n           : std_logic;
        int             : std_logic;
        CAN_tx          : std_logic;
        CAN_rx          : std_logic;
        time_quanta_clk : std_logic;--Time Quantum clocks possible to be used for synchronisation
        timestamp       : std_logic_vector(63 downto 0);

        data_in         : std_logic_vector(31 downto 0);
        data_out        : std_logic_vector(31 downto 0);
        adress          : std_logic_vector(23 downto 0);
        scs             : std_logic; --Chip select
        srd             : std_logic; --Serial read
        swr             : std_logic; --Serial write
        sbe             : std_logic_vector(3 downto 0); --Byte enable
    end record;
    type instance_signals_arr_t is array(1 to NINST) of instance_signals_t;

    signal p : instance_signals_arr_t := (others => (
        tr_del          => 20 * f100_Mhz * 1 ps,
        tr_delayed      => RECESSIVE,
        clk_sys         => '0',
        res_n           => '0',
        int             => '0',
        CAN_tx          => RECESSIVE,
        CAN_rx          => RECESSIVE,
        time_quanta_clk => '0',
        timestamp       => (OTHERS=>'0'),

        data_in         => (OTHERS=>'0'),
        data_out        => (OTHERS=>'0'),
        adress          => (OTHERS=>'0'),
        scs             => '0',
        srd             => '0',
        swr             => '0',
        sbe             => (OTHERS => '1')
    ));

    signal s_bus_level : std_logic := RECESSIVE;
begin

    g_inst: for i in 1 to 2 generate
        CAN_inst: CAN_top_level
        generic map(
            use_logger        => true,
            rx_buffer_size    => 64,
            use_sync          => true,
            ID                => i,
            logger_size       => 16
        )
        port map(
            clk_sys           => p(i).clk_sys,
            res_n             => p(i).res_n,
            data_in           => p(i).data_in,
            data_out          => p(i).data_out,
            adress            => p(i).adress,
            scs               => p(i).scs,
            srd               => p(i).srd,
            swr               => p(i).swr,
            sbe               => p(i).sbe,
            int               => p(i).int,
            CAN_tx            => p(i).CAN_tx,
            CAN_rx            => p(i).CAN_rx,
            time_quanta_clk   => p(i).time_quanta_clk,
            timestamp         => p(i).timestamp,
            drv_bus_o         => iout(i).drv_bus,
            stat_bus_o        => iout(i).stat_bus
        );

        i_txdelay : entity work.signal_delayer
            generic map (
                NSAMPLES    => 16
            )
            port map (
                input       => p(i).CAN_tx,
                delay       => p(i).tr_del,
                delayed     => p(i).tr_delayed
            );

        -------------------------------------------------
        -- Connect individual bus signals of memory buses
        -------------------------------------------------
        x1: mem_bus(i).clk_sys    <= p(i).clk_sys;
        x2: p(i).data_in          <= mem_bus(i).data_in;
        x3: p(i).adress           <= mem_bus(i).address;
        x4: p(i).scs              <= mem_bus(i).scs;
        x5: p(i).swr              <= mem_bus(i).swr;
        x6: p(i).srd              <= mem_bus(i).srd;
        x7: p(i).sbe              <= mem_bus(i).sbe;
        x8: mem_bus(i).data_out   <= p(i).data_out;
        x9: iout(i).irq           <= p(i).int;
        xc: iout(i).hw_reset      <= p(i).res_n;
        -- stat_bus and drv_bus passwd directly, as indirection costs
        -- 10% of simulation time

        ---------------------------------
        --Transceiver and bus realization
        ---------------------------------
        xd: p(i).CAN_rx           <= s_bus_level;
        xe: bus_level             <= s_bus_level;

        ---------------------------------
        -- Clock & timestamp generation
        ---------------------------------
        clk_gen_proc: clock_gen_proc(period => f100_Mhz, duty => 50,
                       epsilon_ppm => (i - 1) * 100, out_clk => p(i).clk_sys);
        tsgen_proc: timestamp_gen_proc(p(i).clk_sys, p(i).timestamp);
    end generate;

    tr_proc:process(all)
        variable busl : std_logic;
    begin
        if bl_force then
            s_bus_level <= bl_inject;
        else
            busl := RECESSIVE;
            for i in 1 to 2
            loop
                busl := busl and p(i).tr_delayed;
            end loop;
            s_bus_level <= busl;
        end if;
    end process;

    ---------------------------------
    --Test process listening to the
    --  higher hierarchy wrapper!
    ---------------------------------
    test_proc:process
    begin
        status <= waiting;
        loop_ctr <= 0;
        wait until run = true;
        if hw_reset_on_new_test then
            log("HW Restart of feature test environment started!",info_l,log_level);
            wait for 5 ns;
            error_ctr <= 0;
            p(1).res_n <= '0';
            p(2).res_n <= '0';
            wait for 100 ns;
            p(1).res_n <= '1';
            p(2).res_n <= '1';
            log("HW Restart of feature test environment finished",info_l,log_level);
            wait for 250 ns; -- wait until the core is really out of reset
        end if;

        --Status is restarted no matter the HW reset
        status <= running;
        print_test_info(iterations, log_level, error_beh, error_tol);

        -------------------------------
        --Main loop of the test
        -------------------------------
        while (loop_ctr<iterations and not exit_imm)
        loop
            log("Starting loop nr " & integer'image(loop_ctr), info_l, log_level);
            --Wait on signal from higher level wrapper to move to the next iteration
            wait until iteration_done = true;

            loop_ctr <= loop_ctr + 1;
            wait for 0 ns;
        end loop;

        status <= passed;
        wait until run = false;
    end process;
end architecture;


Library ieee;
library vunit_lib;
context vunit_lib.vunit_context;

USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;
use work.pkg_feature_exec_dispath.all;

entity tb_feature is
    generic (
        runner_cfg : string := runner_cfg_default;
        iterations : natural := 1;
        log_level  : log_lvl_type := info_l;

        -- Test behaviour when error occurs: Quit, or Go on
        error_beh  : err_beh_type := quit;

        -- Error tolerance, error counter should not exceed this value
        -- in order for the test to pass
        error_tol  : natural := 0;

        -- Timeout in simulation time. 0 means no limit
        timeout    : string := "0 ms";

        seed       : natural := 0;

        hw_reset_on_new_test : boolean := true;

        test_name : string

    );
end entity;
-----------------------------------------------------------------------------------------------------------------
-- Test wrapper and control signals generator
-----------------------------------------------------------------------------------------------------------------
architecture tb of tb_feature is
    signal run         : boolean;          -- Input trigger, test starts running when true                                                        -- exceed this value in order for the test to pass
    signal status_int  : test_status_type; -- Status of the test

    constant mem_bus_init : Avalon_mem_type := (
        scs         => '0',
        swr         => '0',
        srd         => '0',
        address     => (OTHERS =>'0'),
        data_in     => (OTHERS =>'0'),
        clk_sys     => '0',
        data_out    => (OTHERS =>'0'),
        sbe         => x"0"
    );

    procedure restart_mem_bus(
        signal mem_bus : out  Avalon_mem_type
    ) is begin
        mem_bus.scs         <= '0';
        mem_bus.swr         <= '0';
        mem_bus.srd         <= '0';
        mem_bus.address     <= (OTHERS =>'0');
        mem_bus.data_in     <= (OTHERS =>'0');
        mem_bus.clk_sys     <= 'Z';
        mem_bus.data_out    <= (OTHERS =>'Z');
        mem_bus.sbe         <= x"F";
    end procedure;

    --Additional signals definitions
    signal error_ctr      : natural := 0;
    signal exit_imm       : boolean := false;

    signal bl_inject      : std_logic := RECESSIVE;
    signal bl_force       : boolean := false;

    -- test internal signals
    signal iteration_done : boolean := false;

    signal mem_bus        : mem_bus_arr_t := (OTHERS => mem_bus_init);

    signal iout           : instance_outputs_arr_t;

    signal bus_level      : std_logic;

    signal rand_ctr       : natural range 0 to RAND_POOL_SIZE;
    constant padded_test_name : string(1 to 20) := strtolen(20, test_name);

    signal so : feature_signal_outputs_t;
begin
    bl_inject <= so.bl_inject;
    bl_force  <= so.bl_force;

    --In this test wrapper generics are directly connected to the signals
    -- of test entity
    test_comp: entity work.CAN_feature_test
    port map(
        run              =>  run,
        iterations       =>  iterations,
        log_level        =>  log_level,
        error_beh        =>  error_beh,
        error_tol        =>  error_tol,
        status           =>  status_int,
        mem_bus          =>  mem_bus,
        bl_inject        =>  bl_inject,
        bl_force         =>  bl_force,

        iteration_done   => iteration_done,
        hw_reset_on_new_test => hw_reset_on_new_test,
        test_name        => padded_test_name,
        iout             => iout,
        --Internal signals of CAN controllers
        bus_level        => bus_level
    );

    ---------------------------------------
    ---------------------------------------
    --Starts the test and lets it run
    ---------------------------------------
    ---------------------------------------
    test:process
        constant ID_1    : natural range 0 to 15 := 1;
        constant ID_2    : natural range 0 to 15 := 2;
        variable o       : feature_outputs_t;
    begin
        test_runner_setup(runner, runner_cfg);

        --Set the process to run and wait until it comes out of reset
        iteration_done    <= false;
        run               <= true;
        error_ctr         <= 0;

        apply_rand_seed(seed, 0, rand_ctr);

        report "Restarting mem_bus(1)";
        restart_mem_bus(mem_bus(1));
        report "Restarting mem_bus(1)";
        restart_mem_bus(mem_bus(2));
        report "Waiting for out of reset";

        --wait for 10 ns;
        wait until status_int = running;
        --wait until iout(1).hw_reset = '1' and iout(2).hw_reset = '1';
        --wait for 10 ns;
        report "... ready .. let's begin";

        --Execute the controllers configuration
        CAN_turn_controller(true, ID_1, mem_bus(1));
        CAN_turn_controller(true, ID_2, mem_bus(2));
        report "Controllers are ON";

        --Set default retransmitt limit to 0
        -- Failed frames are not retransmited
        -- by default!!!
        CAN_enable_retr_limit(true, 0, ID_1, mem_bus(1));
        CAN_enable_retr_limit(true, 0, ID_2, mem_bus(2));

        report "RETR limit set";
        -------------------------------------------------
        -- Main test loop
        -------------------------------------------------
        while status_int /= passed loop
            report "Iteration ...";
            iteration_done <= false;
            exec_feature_test(test_name => test_name,
                              o         => o,
                              rand_ctr  => rand_ctr,
                              mem_bus   => mem_bus,
                              iout      => iout,
                              so        => so,
                              bus_level => bus_level
                              );
            report "... out of exec function";

            if (o.outcome = false) then
                -- LCOV_EXCL_START
                process_error(error_ctr, error_beh, exit_imm);
                report "Feature test failed!" severity error;
                -- LCOV_EXCL_STOP
            end if;

            wait for 200 ns;
            iteration_done <= true;
            wait for 10 ns;
        end loop;

        run               <= false;
        test_runner_cleanup(runner, error_ctr > error_tol);
    end process;
    watchdog: if time'value(timeout) > 0 ns generate
        test_runner_watchdog(runner, time'value(timeout));
    end generate;
end architecture;
