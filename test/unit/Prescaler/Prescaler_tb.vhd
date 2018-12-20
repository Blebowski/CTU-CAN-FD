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
--  Unit test for the prescaler circuit. At the time of implementation
--  "prescaler_v3.vhd" was used!
--------------------------------------------------------------------------------
-- Revision History:
--    7.6.2016   Created file
--   28.3.2018   Changed to be compatible with extended bit time fields in the
--               register map for Socket CAN.
--   29.3.2018   Added check for the bit-rate switching to verify the compen-
--               sation mechanism, and provide stable reference in case of
--               reimplementation.
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

architecture presc_unit_test of CAN_test is

    -- System clock
    signal clk_sys              : std_logic := '0';

    -- Async reset
    signal res_n                : std_logic := '0';

    -- Edge for synchronisation
    signal sync_edge            : std_logic := '0';

    -- Protocol control state
    signal OP_State             : oper_mode_type := reciever;
    signal drv_bus              : std_logic_vector(1023 downto 0) :=
                                    (OTHERS => '0');

    -- Time quantum clock - Nominal bit time
    signal clk_tq_nbt           : std_logic := '0';

    -- Bit time - Nominal bit time
    signal clk_tq_dbt           : std_logic := '0';

    -- Sample signal for nominal bit time
    signal sample_nbt           : std_logic := '0';

    -- Sample signal of data bit time
    signal sample_dbt           : std_logic := '0';

    -- Delayed signals
    signal sample_nbt_del_1     : std_logic := '0';
    signal sample_dbt_del_1     : std_logic := '0';
    signal sample_nbt_del_2     : std_logic := '0';
    signal sample_dbt_del_2     : std_logic := '0';

    signal sync_nbt             : std_logic := '0';
    signal sync_dbt             : std_logic := '0';
    signal sync_nbt_del_1       : std_logic := '0';
    signal sync_dbt_del_1       : std_logic := '0';
    signal bt_FSM_out           : bit_time_type;
    signal hard_sync_edge_valid : std_logic := '0';
    signal sp_control           : std_logic_vector(1 downto 0) :=
                                      (OTHERS => '0');
    signal sync_control         : std_logic_vector(1 downto 0) :=
                                      (OTHERS => '0');
    signal data_tx              : std_logic := RECESSIVE;

    -- Driving bus aliases
    signal drv_tq_nbt           :   std_logic_vector (7 downto 0) := "00000000";
    signal drv_tq_dbt           :   std_logic_vector (7 downto 0) := "00000000";

    signal drv_prs_nbt          :   std_logic_vector (6 downto 0) := "0000000";
    signal drv_ph1_nbt          :   std_logic_vector (5 downto 0) := "000000";
    signal drv_ph2_nbt          :   std_logic_vector (5 downto 0) := "000000";

    signal drv_prs_dbt          :   std_logic_vector (5 downto 0) := "000000";
    signal drv_ph1_dbt          :   std_logic_vector (4 downto 0) := "00000";
    signal drv_ph2_dbt          :   std_logic_vector (4 downto 0) := "00000";

    signal drv_sjw_nbt          :   std_logic_vector (4 downto 0) := "00000";
    signal drv_sjw_dbt          :   std_logic_vector (4 downto 0) := "00000";

    ---------------------------------------
    --Internal test signals and constants
    ---------------------------------------
    signal setting              :   presc_drv_type :=
                                     ((OTHERS => '0'), (OTHERS => '0'),
                                      (OTHERS => '0'), (OTHERS => '0'),
                                      (OTHERS => '0'), (OTHERS => '0'),
                                      (OTHERS => '0'), (OTHERS => '0'),
                                      (OTHERS => '0'), (OTHERS => '0'));

    signal trig_signals         :   presc_triggers_type;
    constant  inf_proc_time     :   natural := 3; --Information processing time
    signal clock_counter        :   natural := 0;

    --Additional error counters
    signal ipt_err_ctr            :   natural := 0;
    signal coh_err_ctr            :   natural := 0;
    signal sync_seq_err_ctr       :   natural := 0;
    signal sample_seq_err_ctr     :   natural := 0;
    signal main_err_ctr           :   natural := 0;

    --Additional exit_imm
    signal exit_imm_2             :   boolean := false;
    signal exit_imm_3             :   boolean := false;
    signal exit_imm_4             :   boolean := false;
    signal exit_imm_5             :   boolean := false;

    -- Expected duration of bit time during re-synchronisation (in clock cycles)
    signal resync_bit_time_length :   integer := 1;

    -- Random counter for synchronisation edge generation
    signal rand_ctr_sync_edge     :   natural range 0 to RAND_POOL_SIZE := 0;

    -- Generates random bit timing settings
    procedure gen_bit_time_setting(
        signal rand_ctr             : inout natural range 0 to RAND_POOL_SIZE;
        signal setting              : inout presc_drv_type
    ) is
        variable tseg1_dur          :       natural;
    begin
        rand_logic_vect_bt_s(rand_ctr, setting.drv_tq_nbt, 0, 0.2);
        rand_logic_vect_bt_s(rand_ctr, setting.drv_tq_dbt, 0, 0.1);

        rand_logic_vect_bt_s(rand_ctr, setting.drv_prs_nbt, 0, 0.4);
        rand_logic_vect_bt_s(rand_ctr, setting.drv_ph1_nbt, 0, 0.2);

        rand_logic_vect_bt_s(rand_ctr, setting.drv_ph2_nbt, 0, 0.2);
        rand_logic_vect_bt_s(rand_ctr, setting.drv_prs_dbt, 0, 0.3);
        rand_logic_vect_bt_s(rand_ctr, setting.drv_ph1_dbt, 0, 0.15);
        rand_logic_vect_bt_s(rand_ctr, setting.drv_ph2_dbt, 0, 0.15);

        rand_logic_vect_s(rand_ctr, setting.drv_sjw_nbt, 0.2);
        rand_logic_vect_s(rand_ctr, setting.drv_sjw_dbt, 0.2);

        wait for 0 ns;

        ------------------------------------------------------------------------
        -- Here we check that settings are matching IPT!!
        -- This is stated in documentation and is up to responsible
        -- user to set. If minimal IPT is corrupted in Bit timing settings,
        -- PH2 will last IPT (4 clock cycles), NOT shorter!
        ------------------------------------------------------------------------

        ------------------------------------------------------------------------
        -- NBT
        ------------------------------------------------------------------------
        -- PH2 can NOT be 0!
        if (setting.drv_ph2_nbt = "000000") then
            setting.drv_ph2_nbt <= "000001";
        end if;

        -- Time quanta cannot be 0!
        if (setting.drv_tq_nbt = "00000000") then
            setting.drv_tq_nbt <= "00000001";
        end if;
        wait for 0 ns;

        -- Time quanta 1 -> PH2 must be minimum 4!
        if (setting.drv_tq_nbt = "00000001" and
            unsigned(setting.drv_ph2_nbt) < 4)
        then
            setting.drv_ph2_nbt <= "000100";
        end if;

        -- Time quanta 2 or 3 -> PH2 must be minimum 2!
        if ((setting.drv_tq_nbt = "00000010" or
             setting.drv_tq_nbt = "00000011")
             and unsigned(setting.drv_ph2_nbt) < 2)
        then
            setting.drv_ph2_nbt <= "000010";
        end if;

        -- Making sure that PH1 + Prop lasts at least 2 cycles
        tseg1_dur := to_integer(unsigned(setting.drv_tq_nbt)) *
                     (to_integer(unsigned(setting.drv_prs_nbt)) +
                      to_integer(unsigned(setting.drv_ph1_nbt)));
        if (tseg1_dur < 2) then
            setting.drv_prs_nbt <= "0000011";
        end if;

        ------------------------------------------------------------------------
        -- DBT
        ------------------------------------------------------------------------
        if (setting.drv_ph2_dbt = "00000") then
            setting.drv_ph2_dbt <= "00001";
        end if;

        -- Time quanta cannot be 0!
        if (setting.drv_tq_dbt = "00000000") then
            setting.drv_tq_dbt <= "00000001";
        end if;
        wait for 0 ns;

        if (setting.drv_tq_dbt = "00000001" and
            unsigned(setting.drv_ph2_dbt) < 4)
        then
            setting.drv_ph2_dbt <= "00100";
        end if;

        if ((setting.drv_tq_dbt = "00000010" or
             setting.drv_tq_dbt = "00000011")
            and unsigned(setting.drv_ph2_dbt) < 2)
        then
            setting.drv_ph2_dbt <= "00010";
        end if;

        -- Making sure that PH1 + Prop lasts at least 2 cycles
        tseg1_dur := to_integer(unsigned(setting.drv_tq_dbt)) *
                     (to_integer(unsigned(setting.drv_prs_dbt)) +
                      to_integer(unsigned(setting.drv_ph1_dbt)));
        if (tseg1_dur < 2) then
            setting.drv_prs_dbt <= "000011";
        end if;

    end procedure;


    procedure count_cycles_until(
        signal    clk_sys          : in      std_logic;
        variable  counter          : inout   natural;
        signal    condition1       : in      std_logic;
        signal    condition2       : in      std_logic
    )is
    begin
        counter := 0;
        while condition1 = '0' and condition2 = '0' loop
            wait until falling_edge(clk_sys);
            counter := counter + 1;
        end loop;
    end procedure;


    procedure gen_sync_edge_settings(
        signal   rand_ctr       : inout     natural range 0 to RAND_POOL_SIZE;
        constant setting        : in        presc_drv_type;
        constant sample_control : in        std_logic_vector(1 downto 0);
        variable sync_time      : inout     integer;
        variable is_positive    : inout     boolean
    ) is
        variable brp            :           natural;
        variable prs            :           natural;
        variable ph1            :           natural;
        variable ph2            :           natural;
        variable tmp            :           std_logic;
        variable sync_max       :           natural;
    begin

        if (sample_control = NOMINAL_SAMPLE) then
            brp   := to_integer(unsigned(setting.drv_tq_nbt));
            prs   := to_integer(unsigned(setting.drv_prs_nbt));
            ph1   := to_integer(unsigned(setting.drv_ph1_nbt));
            ph2   := to_integer(unsigned(setting.drv_ph2_nbt));
        else
            brp   := to_integer(unsigned(setting.drv_tq_dbt));
            prs   := to_integer(unsigned(setting.drv_prs_dbt));
            ph1   := to_integer(unsigned(setting.drv_ph1_dbt));
            ph2   := to_integer(unsigned(setting.drv_ph2_dbt));
        end if;

        -- Generate random POSITIVE or NEGAIVE resync
        rand_logic_v(rand_ctr, tmp, 0.5);
        if (tmp = '1') then
            is_positive := true;
        else
            is_positive := false;
        end if;

        -- Calculate maximum possible time to wait before sync. edge
        if (is_positive) then
            sync_max := (brp * (prs + ph1 + 1)) - 1;
        else
            sync_max := (brp * ph2);
        end if;

        -- Generate waiting time
        if (sync_max = 0) then
            sync_max := 1;
        end if;
        rand_int_v(rand_ctr, sync_max, sync_time);

    end procedure;


    procedure calc_resync_bit_time(
        signal   setting          : in      presc_drv_type;
        constant positive_resync  : in      boolean;
        constant sync_time        : in      integer;
        constant sample_control   : in      std_logic_vector(1 downto 0);
        signal   exp_bit_time     : out     integer
    ) is
        variable brp            :           natural;
        variable prs            :           natural;
        variable ph1            :           natural;
        variable ph2            :           natural;
        variable sjw            :           natural;
        variable tmp            :           integer;
        variable nom_bit_time   :           natural;
    begin

        --report "Sync time: " & Integer'Image(sync_time);

        if (sample_control = NOMINAL_SAMPLE) then
            brp   := to_integer(unsigned(setting.drv_tq_nbt));
            prs   := to_integer(unsigned(setting.drv_prs_nbt));
            ph1   := to_integer(unsigned(setting.drv_ph1_nbt));
            ph2   := to_integer(unsigned(setting.drv_ph2_nbt));
            sjw   := to_integer(unsigned(setting.drv_sjw_nbt));
        else
            brp   := to_integer(unsigned(setting.drv_tq_dbt));
            prs   := to_integer(unsigned(setting.drv_prs_dbt));
            ph1   := to_integer(unsigned(setting.drv_ph1_dbt));
            ph2   := to_integer(unsigned(setting.drv_ph2_dbt));
            sjw   := to_integer(unsigned(setting.drv_sjw_dbt));
        end if;

        nom_bit_time := brp * (1 + prs + ph1 + ph2);

        -- For positive resynchronisation, amount to lengthen is equal to
        -- synchronisation time.
        if (positive_resync) then
            tmp := sync_time;

        -- For negative resynchronisation, amount to shorten is equal to
        -- PH2 - synchronisation time.
        else
            tmp := (ph2 * brp) - sync_time + 1;
        end if;

        -- Amount to lengthen / shorten should not be more than SJW.
        if (tmp > (sjw * brp)) then
            tmp := sjw * brp;
        end if;

        -- Re-synchronize
        if (positive_resync) then
           exp_bit_time     <= nom_bit_time + tmp;

        else
            -- Check of information processing time (4 clock cycles).
            -- If shorten should be to less than 4 clock cycles, shorten
            -- to MIN 4 clock cycles
            if ( (ph2 * brp) - tmp < 4) then
                tmp := (ph2 * brp) - 4;
            end if;

            exp_bit_time    <= nom_bit_time - tmp;
        end if;


    end procedure;


    ----------------------------------------------------------------------------
    -- Counts number of clock cycles elapse until condition becomes '1'.
    ----------------------------------------------------------------------------
    procedure count_cycles_until(
        signal clk_sys            : in       std_logic;
        variable counter          : inout    natural;
        signal condition          : in       std_logic
    ) is
    begin
        counter := 0;
        while condition = '0' loop
            wait until falling_edge(clk_sys);
            counter := counter + 1;
        end loop;
    end procedure;

begin

    ----------------------------------------------------------------------------
    -- Instance of Prescaler
    ----------------------------------------------------------------------------
    prescaler_comp : prescaler
    PORT map(
        clk_sys              =>  clk_sys,
        res_n                =>  res_n,
        sync_edge            =>  sync_edge,
        OP_State             =>  OP_State,
        drv_bus              =>  drv_bus  ,
        clk_tq_nbt           =>  clk_tq_nbt,
        clk_tq_dbt           =>  clk_tq_dbt,
        sample_nbt           =>  sample_nbt,
        sample_dbt           =>  sample_dbt,
        sample_nbt_del_1     =>  sample_nbt_del_1,
        sample_dbt_del_1     =>  sample_dbt_del_1,
        sample_nbt_del_2     =>  sample_nbt_del_2,
        sample_dbt_del_2     =>  sample_dbt_del_2,
        sync_nbt             =>  sync_nbt,
        sync_dbt             =>  sync_dbt,
        data_tx              =>  data_tx,
        sync_nbt_del_1       =>  sync_nbt_del_1,
        sync_dbt_del_1       =>  sync_dbt_del_1 ,
        bt_FSM_out           =>  bt_FSM_out,
        hard_sync_edge_valid =>  hard_sync_edge_valid,
        sp_control           =>  sp_control,
        sync_control         =>  sync_control
    );

    drv_bus(DRV_TQ_NBT_HIGH downto DRV_TQ_NBT_LOW)    <= drv_tq_nbt;
    drv_bus(DRV_TQ_DBT_HIGH downto DRV_TQ_DBT_LOW)    <= drv_tq_dbt;
    drv_bus(DRV_PRS_NBT_HIGH downto DRV_PRS_NBT_LOW)  <= drv_prs_nbt;
    drv_bus(DRV_PH1_NBT_HIGH downto DRV_PH1_NBT_LOW)  <= drv_ph1_nbt;
    drv_bus(DRV_PH2_NBT_HIGH downto DRV_PH2_NBT_LOW)  <= drv_ph2_nbt;
    drv_bus(DRV_PRS_DBT_HIGH downto DRV_PRS_DBT_LOW)  <= drv_prs_dbt;
    drv_bus(DRV_PH1_DBT_HIGH downto DRV_PH1_DBT_LOW)  <= drv_ph1_dbt;
    drv_bus(DRV_PH2_DBT_HIGH downto DRV_PH2_DBT_LOW)  <= drv_ph2_dbt;
    drv_bus(DRV_SJW_HIGH downto DRV_SJW_LOW)          <= drv_sjw_nbt;
    drv_bus(DRV_SJW_DBT_HIGH downto DRV_SJW_DBT_LOW)  <= drv_sjw_dbt;


    ----------------------------------------------------------------------------
    -- Joining of signal groups into records
    ----------------------------------------------------------------------------
    drv_tq_nbt    <=  setting.drv_tq_nbt;
    drv_tq_dbt    <=  setting.drv_tq_dbt;
    drv_prs_nbt   <=  setting.drv_prs_nbt;
    drv_ph1_nbt   <=  setting.drv_ph1_nbt;
    drv_ph2_nbt   <=  setting.drv_ph2_nbt;
    drv_prs_dbt   <=  setting.drv_prs_dbt;
    drv_ph1_dbt   <=  setting.drv_ph1_dbt;
    drv_ph2_dbt   <=  setting.drv_ph2_dbt;
    drv_sjw_nbt   <=  setting.drv_sjw_nbt;
    drv_sjw_dbt   <=  setting.drv_sjw_dbt;

    trig_signals.sample_nbt        <=  sample_nbt;
    trig_signals.sample_dbt        <=  sample_dbt;
    trig_signals.sample_nbt_del_1  <=  sample_nbt_del_1;
    trig_signals.sample_dbt_del_1  <=  sample_dbt_del_1;
    trig_signals.sample_nbt_del_2  <=  sample_nbt_del_2;
    trig_signals.sample_dbt_del_2  <=  sample_dbt_del_2;
    trig_signals.sync_nbt          <=  sync_nbt;
    trig_signals.sync_dbt          <=  sync_dbt;
    trig_signals.sync_nbt_del_1    <=  sync_nbt_del_1;
    trig_signals.sync_dbt_del_1    <=  sync_dbt_del_1;


    ----------------------------------------------------------------------------
    -- Clock generation
    ----------------------------------------------------------------------------
    clock_gen : process
        constant period   : natural := f100_Mhz;
        constant duty     : natural := 50;
        constant epsilon  : natural := 0;
        constant par      : generate_clock_precomputed_t
                             := precompute_clock(period, duty, epsilon);
    begin
        loop
            generate_clock(par, clk_sys);
            clock_counter <= clock_counter + 1;
        end loop;
    end process;


    ----------------------------------------------------------------------------
    -- Checking of Information processing time
    ----------------------------------------------------------------------------
    ipt_proc_check : process
        variable store : natural := 0;
    begin
        if (sp_control = NOMINAL_SAMPLE) then

            wait until rising_edge(sample_nbt);
            store := clock_counter;
            wait until rising_edge(sync_nbt);
            if ((clock_counter - store) < inf_proc_time) then
                log("Information processing time corrupted", error_l, log_level);
                process_error(ipt_err_ctr, error_beh, exit_imm_2);
            end if;

        elsif (sp_control = DATA_SAMPLE) then

            wait until rising_edge(sample_dbt);
            store := clock_counter;
            wait until rising_edge(sync_dbt);
            if ((clock_counter - store) < inf_proc_time) then
                log("Information processing time corrupted", error_l, log_level);
                process_error(ipt_err_ctr, error_beh, exit_imm_2);
            end if;

        else
            report "Only NOMINAL and DATA sampling is supported" severity error;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Checking that two consecutive sync or sample signals are not present!!
    ----------------------------------------------------------------------------
    trig_coherency_proc : process
        variable was_sync : boolean := false;
    begin
        wait until falling_edge(clk_sys) and
                    (sync_nbt = '1' or sample_nbt = '1' or
                     sync_dbt = '1' or sample_dbt = '1');

        if (sync_nbt = '1' or sync_dbt = '1') then

            -- Here error occures due to two consecutive sync signals
            if (was_sync = true) then
                log("Two consecutive sync signals!", error_l, log_level);
                process_error(coh_err_ctr, error_beh, exit_imm_3);
            end if;
            was_sync := true;

        elsif (sample_nbt = '1' or sample_dbt = '1') then

            -- Here error occures due to two consecutive sample signals
            if (was_sync = false) then
                log("Two consecutive sample signals!", error_l, log_level);
                process_error(coh_err_ctr, error_beh, exit_imm_3);
            end if;
            was_sync := false;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Checking that all sync signals in the sequence are generated every time!
    ----------------------------------------------------------------------------
    sync_seq_check_proc : process
    begin
        wait until rising_edge(sync_nbt) or rising_edge(sync_dbt);

        wait for 15 ns; -- One and half clock cycle
        if (sync_nbt_del_1 = '0' and sync_dbt_del_1 = '0') then
           log("Sync sequnce not complete, delay 1 CLK signal missing!",
                error_l, log_level);
           process_error(sync_seq_err_ctr, error_beh, exit_imm_4);
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Checking that all sample signals in the sequence are generated every
    -- time!
    ----------------------------------------------------------------------------
    sample_seq_check_proc : process
    begin
        wait until falling_edge(clk_sys) and
                    (sample_nbt = '1' or sample_dbt = '1');

        wait for 10 ns; -- One and half clock cycle
        if (sample_nbt_del_1 = '0' and sample_dbt_del_1 = '0') then
           log("Sample sequnce not complete, delay 1 CLK signal missing!",
                error_l, log_level);
           process_error(sample_seq_err_ctr, error_beh, exit_imm_5);
        end if;

        wait for 10 ns;

        if (sample_nbt_del_2 = '0' and sample_dbt_del_2 = '0') then
           log("Sample sequnce not complete, delay 2 CLK signal missing!",
                error_l, log_level);
           process_error(sample_seq_err_ctr, error_beh, exit_imm_5);
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Generation of synchronisation edges for resynchronisation
    ----------------------------------------------------------------------------
    sync_gen_proc : process
        variable positive_resync    : boolean;
        variable sync_time          : integer;
        variable skip_sync          : boolean;
    begin
        if (res_n = ACT_RESET) then
            apply_rand_seed(seed, 1, rand_ctr_sync_edge);
        end if;

        -- Generate parameters for new resynchronisation
        wait until falling_edge(clk_sys) and bt_FSM_out = sync;
        gen_sync_edge_settings(rand_ctr_sync_edge, setting, sp_control,
                                    sync_time, positive_resync);

        calc_resync_bit_time(setting, positive_resync, sync_time, sp_control,
                               resync_bit_time_length);

        wait for 0 ns;
        skip_sync := false;

        -- If resynchronisation length is generated as 0, "sync_edge" should
        -- come in "sync" state
        if (sync_time = 0) then
            sync_edge <= '1';
            wait until falling_edge(clk_sys);
            sync_edge <= '0';
            wait until (bt_FSM_out /= sync);
        else

            -- For positive resync -> start from PROP(or PH1) phase, for
            -- Negative resync -> start by PH2 phase
            if (positive_resync) then
                wait until (bt_FSM_out /= sync);
            else
                wait until (bt_FSM_out = ph2);
            end if;

            for i in 1 to sync_time - 1 loop
                wait until rising_edge(clk_sys);

                -- Break if Bit time in any case finished earlier, e.g. BRS
                -- testing started
                if (bt_FSM_out = sync) then
                    skip_sync := true;
                    exit;
                end if;
            end loop;

            if (skip_sync = false) then
                sync_edge <= '1';
                wait until rising_edge(clk_sys);
                sync_edge <= '0';
            end if;

        end if;

    end process;


    -- Sum of error counters
    error_ctr <= sample_seq_err_ctr + sync_seq_err_ctr +
                   coh_err_ctr + ipt_err_ctr + main_err_ctr;
    errors    <= error_ctr;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Main Test process
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    test_proc : process
        variable rand_real_value  : real;
        variable check_ctr        : natural := 0;
        variable nom_ctr          : natural := 0;
        variable data_ctr         : natural := 0;
        variable exp_dur          : integer;

        -- Expected duration of Bit Rate shift bit
        variable exp_dur_BRS      : integer;

        -- Expected duration of CRC delimiter
        variable exp_dur_CRC_del  : integer;

        variable tmp_text         : string (1 to 10) := "Nominal   ";

        variable brp              : integer;

    begin
        log("Restarting Prescaler unit test!", info_l, log_level);
        wait for 5 ns;

        -- Generates random initial bit time settings to avoid having zero
        -- values on the input of DUT after reset!
        apply_rand_seed(seed, 0, rand_ctr);
        gen_bit_time_setting(rand_ctr, setting);

        reset_test(res_n, status, run, main_err_ctr);
        log("Restarted Prescaler unit test", info_l, log_level);
        print_test_info(iterations, log_level, error_beh,  error_tol);


        ------------------------------------------------------------------------
        -- Main test loop
        ------------------------------------------------------------------------
        log("Starting Prescaler unit main loop", info_l, log_level);

        while (loop_ctr < iterations or exit_imm) loop
            log("Starting loop nr " & integer'image(loop_ctr), info_l, log_level);

            -- Generates random bit time settings for new bits.
            wait until bt_FSM_out = ph2;
            wait until bt_FSM_out = sync;
            gen_bit_time_setting(rand_ctr, setting);

            -- Sets random sampling
            rand_real_v(rand_ctr, rand_real_value);
            if (rand_real_value > 0.5) then
                sp_control <= DATA_SAMPLE;
            else
                sp_control <= NOMINAL_SAMPLE;
            end if;

            -- After applying the Bit time settings the first bit can be fucked
            -- up due to register updates. Wait for a bit which starts with
            -- clean new timing set properly (tq_edge update takes one clock
            -- cycle, thus it can happend that SYNC will last only one clock
            -- cycle instead of one time quanta). Note that Bit Timing does not
            -- have to be changed during the bit duration, but is set only once
            -- at controller configuration!
            wait until bt_FSM_out = ph2;
            wait until bt_FSM_out = sync;

            --------------------------------------------------------------------
            -- Check duration of default bit lenght without synchronisation
            --------------------------------------------------------------------
            sync_control  <= NO_SYNC;
            log("Starting Check without synchronisation", info_l, log_level);
            for i in 1 to 4 loop

                -- Check distance between "SYNC" and "SAMPLE" trigger
                log("Checking distance between SYNC and SAMPLE", info_l,
                    log_level);
                wait until rising_edge(sync_nbt) or rising_edge(sync_dbt);
                count_cycles_until(clk_sys, check_ctr, sample_nbt, sample_dbt);

                if (sp_control = NOMINAL_SAMPLE) then
                    exp_dur := ( (to_integer(unsigned(drv_ph1_nbt)) +
                                to_integer(unsigned(drv_prs_nbt)) + 1)
                               *
                               to_integer(unsigned(drv_tq_nbt)));
                    tmp_text := "Nominal   ";
                else
                    exp_dur := ( (to_integer(unsigned(drv_ph1_dbt)) +
                                to_integer(unsigned(drv_prs_dbt)) + 1)
                               *
                               to_integer(unsigned(drv_tq_dbt))
                              );
                    tmp_text := "Data      ";
                end if;

                if (check_ctr /= exp_dur) then
                    -- LCOV_EXCL_START
                    log("SYNC+PROP+PH1 " & tmp_text &
                        " did not last expected time!", error_l, log_level);
                    process_error(main_err_ctr, error_beh, exit_imm);
                    -- LCOV_EXCL_STOP
                end if;

                -- Check distance between two consecutive "SYNC" triggers
                -- (whole bit time)
                log("Checking distance two consecutive SYNC", info_l, log_level);
                wait until rising_edge(sync_nbt) or rising_edge(sync_dbt);
                wait for 15 ns;
                count_cycles_until(clk_sys, check_ctr, sync_nbt, sync_dbt);

                if (sp_control = NOMINAL_SAMPLE) then
                    exp_dur := ((to_integer(unsigned(drv_prs_nbt)) +
                                 to_integer(unsigned(drv_ph1_nbt)) +
                                 to_integer(unsigned(drv_ph2_nbt)) + 1)
                                *
                                to_integer(unsigned(drv_tq_nbt)));
                    tmp_text := "Nominal   ";
                else
                    exp_dur := ((to_integer(unsigned(drv_prs_dbt)) +
                                 to_integer(unsigned(drv_ph1_dbt)) +
                                 to_integer(unsigned(drv_ph2_dbt)) + 1)
                                *
                                to_integer(unsigned(drv_tq_dbt)));
                    tmp_text := "Data      ";
                end if;

                if (check_ctr /= exp_dur) then
                    -- LCOV_EXCL_START
                    log("SYNC+PROP+PH1+PH2 " & tmp_text &
                        " did not last expected time!", error_l, log_level);
                    process_error(main_err_ctr, error_beh, exit_imm);
                    -- LCOV_EXCL_STOP
                end if;

            end loop;


            --------------------------------------------------------------------
            -- Check duration with Re-synchronisation turned ON
            --------------------------------------------------------------------
            wait until bt_FSM_out = sync;
            wait until rising_edge(clk_sys);
            sync_control    <= RE_SYNC;

            log("Starting Check with Resynchronisation", info_l, log_level);
            for i in 0 to 4 loop

                wait until rising_edge(clk_sys) and
                        ((sync_nbt = '1') or (sync_dbt = '1'));
                wait for 1 ns;
                count_cycles_until(clk_sys, check_ctr, sync_nbt, sync_dbt);

                ----------------------------------------------------------------
                -- Evaluate outcome of resynchronisation. Resynchronisation can
                -- come any time, but bit time is processed only with time
                -- quanta! Calculated expected time with one clock cycle
                -- precision, not time quanta precision! Thus real difference
                -- between calculated and measured difference should be less
                -- than Time quanta. Note that this is totally exact for Time
                -- quanta = 1.
                ----------------------------------------------------------------
                if (sp_control = NOMINAL_SAMPLE) then
                    brp := to_integer(unsigned(drv_tq_nbt));
                else
                    brp := to_integer(unsigned(drv_tq_dbt));
                end if;

                if (abs(integer(check_ctr) - resync_bit_time_length) > brp) then
                    -- LCOV_EXCL_START
                    log("Resync bit length wrong! Expected length: " &
                            integer'image(resync_bit_time_length) &
                        " Real length: " & integer'image(check_ctr),
                        error_l, log_level);
                    process_error(main_err_ctr, error_beh, exit_imm);
                    -- LCOV_EXCL_STOP
                end if;

            end loop;
            wait until rising_edge(clk_sys);
            sync_control    <= NO_SYNC;

            wait until rising_edge(clk_sys);

            -- Wait till next bit so that resynchronisation and bit rate switch
            -- does not affect each other!
            wait until bt_fsm_out /= sync;
            wait until bt_fsm_out = sync;

            --------------------------------------------------------------------
            -- Test the duration of bits during bit-rate switching, to verify
            -- BRS realisation via two counters in Prescaler.
            --------------------------------------------------------------------
            exp_dur_BRS := (to_integer(unsigned(drv_tq_nbt)) *
                         (to_integer(unsigned(drv_prs_nbt)) +
                          to_integer(unsigned(drv_ph1_nbt)) + 1))
                         +
                         (to_integer(unsigned(drv_tq_dbt)) *
                          to_integer(unsigned(drv_ph2_dbt)));

            exp_dur_CRC_del := to_integer(unsigned(drv_tq_dbt)) *
                             (to_integer(unsigned(drv_prs_dbt)) +
                              to_integer(unsigned(drv_ph1_dbt)) + 1)
                             +
                             (to_integer(unsigned(drv_tq_nbt)) *
                              to_integer(unsigned(drv_ph2_nbt)));

            --------------------------------------------------------------------
            -- Emulate a BRS bit
            --------------------------------------------------------------------
            log("Checking duration of BRS bit", info_l, log_level);

            -- Nominal Bit-rate part, count length between sync trigger and
            -- sample trigger!
            sp_control <= NOMINAL_SAMPLE;
            wait until bt_FSM_out = ph2;
            wait until falling_edge(clk_sys) and sync_nbt = '1';
            count_cycles_until(clk_sys, nom_ctr, sample_nbt);

            -- Delay before Sampling type switching as if caused by Protocol
            -- Control! (three clock cycles)
            wait until rising_edge(clk_sys);
            wait until rising_edge(clk_sys);
            wait until rising_edge(clk_sys);
            sp_control <= DATA_SAMPLE;

            -- Wait till the end of the bit time
            count_cycles_until(clk_sys, data_ctr, sync_dbt);

            -- Check duration, count with two cycle delay.
            if (exp_dur_BRS /= nom_ctr + data_ctr + 2) then
                -- LCOV_EXCL_START
                log("BRS bit length not as expected, " &
                    "Expected: " & integer'image(exp_dur_BRS) &
                    "Real: " & integer'image(nom_ctr + data_ctr + 2),
                     error_l, log_level);
                -- LCOV_EXCL_STOP
                process_error(main_err_ctr, error_beh, exit_imm);
            end if;

            --------------------------------------------------------------------
            -- Emulate CRC delimiter bit (as if switching back to Nominal
            -- data-rate)
            --------------------------------------------------------------------
            log("Checking duration of CRC delimiter bit", info_l, log_level);
            wait until bt_FSM_out = ph2;
            wait until falling_edge(clk_sys) and sync_dbt = '1';
            count_cycles_until(clk_sys, data_ctr, sample_dbt);

            -- Delay three clock cycles again, as if caused by Protocol Control !
            wait until rising_edge(clk_sys);
            wait until rising_edge(clk_sys);
            wait until rising_edge(clk_sys);
            sp_control <= NOMINAL_SAMPLE;

            -- Wait until the end of bit time
            count_cycles_until(clk_sys, nom_ctr, sync_nbt);

            -- Check the duration
            if (exp_dur_CRC_del /= nom_ctr + data_ctr + 2) then
                -- LCOV_EXCL_START
                log("CRC delimiter bit length not as expected, " &
                    "Expected: " & integer'image(exp_dur_CRC_del) &
                    "Real: " & integer'image(nom_ctr + data_ctr + 2),
                     error_l, log_level);
                process_error(main_err_ctr, error_beh, exit_imm);
                -- LCOV_EXCL_STOP
            end if;

            wait until rising_edge(clk_sys);

            loop_ctr <= loop_ctr + 1;
        end loop;

        evaluate_test(error_tol, error_ctr, status);
    end process;

end architecture;
