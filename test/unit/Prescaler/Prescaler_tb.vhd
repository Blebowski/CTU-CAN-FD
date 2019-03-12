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
--  Unit test for the prescaler circuit.
--------------------------------------------------------------------------------
-- Revision History:
--    7.6.2016   Created file
--   28.3.2018   Changed to be compatible with extended bit time fields in the
--               register map for Socket CAN.
--   29.3.2018   Added check for the bit-rate switching to verify the compen-
--               sation mechanism, and provide stable reference in case of
--               reimplementation.
--   12.3.2019   Replaced direct checks by a model of prescaler.
--------------------------------------------------------------------------------

context work.ctu_can_synth_context;
context work.ctu_can_test_context;

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

    -- Connections for sample and sync signals to DUT
    signal sample_nbt_i         : std_logic_vector(2 downto 0);
    signal sample_dbt_i         : std_logic_vector(2 downto 0);
    signal sync_nbt_i           : std_logic_vector(1 downto 0);
    signal sync_dbt_i           : std_logic_vector(1 downto 0);

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
   
    signal drv_ena              :   std_logic := '0';

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
    
    -- No positive re-synchronisation
    signal no_pos_resync          :   std_logic;
    
    ---------------------------------------------------------------------------
    -- Model outputs
    ---------------------------------------------------------------------------
    signal clk_tq_nbt_mod     : std_logic;
    signal clk_tq_dbt_mod     : std_logic;
    signal sample_nbt_mod     : std_logic_vector(2 downto 0); 
    signal sample_dbt_mod     : std_logic_vector(2 downto 0);
    signal sync_nbt_mod       : std_logic_vector(1 downto 0);
    signal sync_dbt_mod       : std_logic_vector(1 downto 0);
    signal bt_FSM_mod         : bit_time_type;
    signal hard_sync_edge_valid_mod : std_logic; 

    -- Check towards model outputs enabled
    signal mod_check_enabled  : boolean;

    signal rand_val : integer;
    
    signal rand_ctr_2 : natural range 0 to 3800 := 0;


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
   
begin

    ----------------------------------------------------------------------------
    -- Instance of Prescaler
    ----------------------------------------------------------------------------
    prescaler_comp : prescaler
    PORT map(
        clk_sys              =>  clk_sys,
        res_n                =>  res_n,
        sync_edge            =>  sync_edge,
        drv_bus              =>  drv_bus  ,
        sample_nbt           =>  sample_nbt_i,
        sample_dbt           =>  sample_dbt_i,
        sync_nbt             =>  sync_nbt_i,
        sync_dbt             =>  sync_dbt_i,
        bt_FSM_out           =>  bt_FSM_out,
        hard_sync_edge_valid =>  hard_sync_edge_valid,
        sp_control           =>  sp_control,
        sync_control         =>  sync_control,
        no_pos_resync        =>  no_pos_resync
    );

    -- Connect new vector signals to old signals for backwards compatibility
    sample_nbt       <= sample_nbt_i(2);
    sample_nbt_del_1 <= sample_nbt_i(1);
    sample_nbt_del_2 <= sample_nbt_i(0);
    
    sample_dbt       <= sample_dbt_i(2);
    sample_dbt_del_1 <= sample_dbt_i(1);
    sample_dbt_del_2 <= sample_dbt_i(0);

    sync_nbt        <= sync_nbt_i(1);
    sync_nbt_del_1  <= sync_nbt_i(0);

    sync_dbt        <= sync_dbt_i(1);
    sync_dbt_del_1  <= sync_dbt_i(0);

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
    
    drv_bus(DRV_ENA_INDEX)  <= drv_ena;


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

    no_pos_resync <= '1' when (OP_State = transciever and data_tx = DOMINANT)
                         else
                     '0';

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
            
            check((clock_counter - store) >= inf_proc_time,
                   "Information processing time corrupted");

        elsif (sp_control = DATA_SAMPLE) then

            wait until rising_edge(sample_dbt);
            store := clock_counter;
            wait until rising_edge(sync_dbt);
            check((clock_counter - store) >= inf_proc_time,
                  "Information processing time corrupted");

        else
            error("Only NOMINAL and DATA sampling is supported");
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Checking that two consecutive sync or sample signals are not present!!
    ----------------------------------------------------------------------------
    trig_coherency_proc : process
        variable was_sync : boolean := true;
        variable mod_check_prev : boolean := false;
    begin
        wait until rising_edge(clk_sys) and
                    ((sync_nbt = '1' or sample_nbt = '1' or
                      sync_dbt = '1' or sample_dbt = '1') or (drv_ena = '0'));

        if (drv_ena = '1') then
            if (sync_nbt = '1' or sync_dbt = '1') then
    
                -- Here error occures due to two consecutive sync signals
                if (was_sync and (mod_check_prev and mod_check_enabled)) then
                    error("Two consecutive sync signals!");
                end if;
                was_sync := true;
    
            elsif (sample_nbt = '1' or sample_dbt = '1') then
    
                -- Here error occures due to two consecutive sample signals
                if ((not was_sync) and (mod_check_prev and mod_check_enabled)) then
                    error("Two consecutive sample signals!");
                end if;
                was_sync := false;
            end if;

        
            if (mod_check_prev = false) then
                mod_check_prev := true;
            end if;
        else
            mod_check_prev := false;
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
            error("Sync sequnce not complete, delay 1 CLK signal missing!");
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Checking that all sample signals in the sequence are generated every
    -- time!
    ----------------------------------------------------------------------------
    sample_seq_check_proc : process
    begin
        wait until rising_edge(clk_sys) and
                    (sample_nbt = '1' or sample_dbt = '1');

        wait until rising_edge(clk_sys);
            if (sample_nbt_del_1 = '0' and sample_dbt_del_1 = '0') then
               error("Sample sequnce not complete, delay 1 CLK signal missing!");
            end if;
           
        wait until rising_edge(clk_sys);
            if (sample_nbt_del_2 = '0' and sample_dbt_del_2 = '0') then
              error("Sample sequnce not complete, delay 2 CLK signal missing!");
            end if;
    end process;


    ----------------------------------------------------------------------------
    -- Generation of synchronisation edges
    ----------------------------------------------------------------------------
    sync_gen_proc : process
        variable positive_resync    : boolean;
        variable sync_time          : integer;
        variable skip_sync          : boolean;
        variable rand_wait_time     : integer;
    begin
        if (res_n = ACT_RESET) then
            apply_rand_seed(seed, 1, rand_ctr_sync_edge);
        end if;

        -- Wait for random amount of time!
        rand_int_v(rand_ctr_2, 10000, rand_wait_time);
        rand_wait_time := rand_wait_time + 100;
        for i in 0 to rand_wait_time loop
            wait until falling_edge(clk_sys);
        end loop;

        -- Generate the edge
        wait until falling_edge(clk_sys);
        sync_edge <= '1';
        wait until falling_edge(clk_sys);
        sync_edge <= '0';
    
        info("-------------------------------------------------");
        info("Synchronisation edge generated!");
        info("-------------------------------------------------");
   
        if (sync_control = RE_SYNC) then
            info("This is RE-SYNCHRONISATION edge!");
            if (bt_fsm_out = tseg1) then
                info("Positive resynchronisation");
            else
                info("Negative Resynchronisation");
            end if;
        elsif (sync_control = HARD_SYNC) then
            info("This is HARD-SYNCHRONISATION edge");
        else
            info("Synchronisation is disabled");
        end if;
        
        info("-------------------------------------------------");
    end process;


    ----------------------------------------------------------------------------
    -- Checking DUT vs model outputs!
    -- Model is cycle-accurate, there should be no mismatch !!
    ----------------------------------------------------------------------------
    mod_dut_check_proc : process
    begin
        wait until falling_edge(clk_sys);
        
        if (mod_check_enabled) then
            
            if (bt_fsm_out /= bt_fsm_mod) then
                wait for 50 ns;
                error("Bit time FSM state mismatch");
            end if;

        end if;
    end process;


    ---------------------------------------------------------------------------
    -- Behavioral model of prescaler
    ---------------------------------------------------------------------------
    prescaler_model_comp : prescaler_model
    generic map(
        reset_polarity          => '0',
        clock_period            => 10 ns
    )
    port map(
        clk_sys                 => clk_sys,
        res_n                   => res_n,
        sync_edge               => sync_edge,
        OP_State                => OP_State,
        drv_bus                 => drv_bus,
        bt_FSM_out              => bt_FSM_mod,
        data_tx                 => data_tx,
        sp_control              => sp_control,
        sync_control            => sync_control
    );


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
        info("Restarting Prescaler unit test!");
        wait for 5 ns;

        -- Generates random initial bit time settings to avoid having zero
        -- values on the input of DUT after reset!
        apply_rand_seed(seed, 0, rand_ctr);
        gen_bit_time_setting(rand_ctr, setting);

        reset_test(res_n, status, run, main_err_ctr);
        info("Restarted Prescaler unit test");
        print_test_info(iterations, log_level, error_beh, error_tol);

        ------------------------------------------------------------------------
        -- Main test loop
        ------------------------------------------------------------------------
        info("Starting Prescaler unit main loop");

        while (loop_ctr < iterations or exit_imm) loop
            
            info("*************************************************");
            info("Starting loop nr " & integer'image(loop_ctr));
            info("*************************************************");

            -- Generates random bit time settings for new bits.
            gen_bit_time_setting(rand_ctr, setting);

            -- Sets random sampling
            rand_real_v(rand_ctr, rand_real_value);
            if (rand_real_value > 0.5) then
                sp_control <= DATA_SAMPLE;
            else
                sp_control <= NOMINAL_SAMPLE;
            end if;


            -- Disable the model checks for config change
            mod_check_enabled <= false;
            sync_control      <= NO_SYNC;
            info("model check - DISABLE");
            drv_ena <= '0';
            
            wait until rising_edge(clk_sys) and (bt_FSM_out = reset and bt_fsm_mod = reset);
            res_n <= '0';
            
            wait for 100 ns;
            
            res_n <= '1';
            drv_ena <= '1';
            wait for 1000 ns;
            
            mod_check_enabled <= true;
            info("model check - ENABLE");
            
            info("*************************************************");
            info("*************** BIT TIME SETTINGS ***************");
            info("*************************************************");
            
            info("BRP Data   : " & integer'image(to_integer(unsigned(drv_tq_dbt))));
            info("TSEG1 Data : " & integer'image(to_integer(
                             unsigned(drv_ph1_dbt) +
                             unsigned(drv_prs_dbt) + 1)));
            info("TSEG2 Data : " & integer'image(to_integer(unsigned(drv_ph2_dbt))));
            info("SJW Data   : " & integer'image(to_integer(unsigned(drv_sjw_dbt))));
            
            info("BRP Nominal   : " & integer'image(to_integer(unsigned(drv_tq_nbt))));
            info("TSEG1 Nominal : " & integer'image(to_integer(
                               unsigned(drv_ph1_nbt) +
                               unsigned(drv_prs_nbt) + 1)));
            info("TSEG2 Nominal : " & integer'image(to_integer(unsigned(drv_ph2_nbt))));
            info("SJW Nominal   : " & integer'image(to_integer(unsigned(drv_sjw_nbt))));
            
            if (sp_control = DATA_SAMPLE) then
                info("Basic bit rate: DATA");
            else
                info("Basic bit rate: NOMINAL");
            end if;
        
            wait until bt_FSM_out = tseg2;
            wait until bt_FSM_out = tseg1;
            wait until bt_FSM_out = tseg2;
            wait until bt_FSM_out = tseg1;
                    
            info("************************************************");
            
            info("************************************************");
            info("Starting Check without synchronisation");
            info("************************************************");
            
            sync_control  <= NO_SYNC;
            
            for i in 1 to 4 loop

                -- Check distance between "SYNC" and "SAMPLE" trigger
                info("Checking distance between SYNC and SAMPLE");
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

                -- We must extend the duration of TSEG1 by one clock cycle since
                -- Sync trigger now occurs in the last cycle of TSEG2 instead of
                -- first cycle in TSEG1! This was done as part of Prescaler re-work
                -- in 02-2019.
                exp_dur := exp_dur + 1;

                check(check_ctr = exp_dur, "SYNC+PROP+PH1 " & tmp_text &
                          " did not last expected time!," &
                          "Expected cycles: " & integer'image(exp_dur) &
                          " Real cycles: " & integer'image(check_ctr));
                
                -- Check distance between two consecutive "SYNC" triggers
                -- (whole bit time)
                info("Checking distance of two consecutive SYNC signals");
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

                check(check_ctr = exp_dur, "SYNC+PROP+PH1+PH2 " & tmp_text &
                      " did not last expected time!");
            end loop;


            --------------------------------------------------------------------
            -- Check duration with Re-synchronisation turned ON
            --------------------------------------------------------------------
            wait until bt_FSM_out = tseg1;
            
            -- Switch to Re-synchronisation in sample point as Protocol Control
            -- would do!
            wait until bt_FSM_out = tseg2;
            wait until falling_edge(clk_sys);
            wait until falling_edge(clk_sys);
            wait until falling_edge(clk_sys);
            sync_control <= RE_SYNC;

            info("************************************************");
            info("Starting Check with Resynchronisation");
            info("************************************************");
            
            -- Here just iterate 5 bits and rely on model to check the re-sync.
            for i in 0 to 4 loop
                wait until bt_fsm_out = tseg1;
                wait until bt_fsm_out = tseg2;
            end loop;

            info("************************************************");
            info("Checking duration of BRS bit");
            info("************************************************");

            -- Force Nominal sampling, keep model disabled since this
            -- switch is after TSEG2.
            mod_check_enabled <= false;
            sp_control        <= NOMINAL_SAMPLE;
            
            wait until bt_FSM_out = tseg2;
            wait until bt_FSM_out = tseg1;
            
            mod_check_enabled <= true;
            
            -- Move to sample point! 
            wait until bt_FSM_out = tseg2;
            
            -------------------------------------------------------------------
            -- Delay for 0 to 3 clock cycles. This accounts for possible
            -- future implementations of protocol control where number of
            -- sampling triggers might be decreased!
            -------------------------------------------------------------------
            rand_int_s(rand_ctr, 2, rand_val);
            wait for 0 ns;
            
            for i in 0 to rand_val loop 
                wait until falling_edge(clk_sys);
            end loop;
            
            sp_control <= DATA_SAMPLE;

            wait until bt_FSM_out = tseg1;
            
            -- Wait for two more bits between (in case models would get lost)
            wait until bt_FSM_out = tseg1;
            wait until bt_FSM_out = tseg1;

            --------------------------------------------------------------------
            -- Emulate CRC delimiter bit (as if switching back to Nominal
            -- data-rate)
            --------------------------------------------------------------------
            
            info("************************************************");
            info("Checking duration of CRC delimiter bit");
            info("************************************************");

            wait until bt_FSM_out = tseg2;

            -------------------------------------------------------------------
            -- Delay for 0 to 3 clock cycles. This accounts for possible
            -- future implementations of protocol control where number of
            -- sampling triggers might be decreased!
            -------------------------------------------------------------------
            rand_int_s(rand_ctr, 2, rand_val);
            wait for 0 ns;
            
            for i in 0 to rand_val loop 
                wait until falling_edge(clk_sys);
            end loop;
            
            sp_control <= NOMINAL_SAMPLE;
            
            wait until bt_FSM_out = tseg1;

            --------------------------------------------------------------------
            -- Test Hard synchronisation!
            --------------------------------------------------------------------

            info("************************************************");
            info("Checking HARD Synchronisation!");
            info("************************************************");
            
            mod_check_enabled <= false;
            sp_control        <= NOMINAL_SAMPLE;
            
            wait until bt_FSM_out = tseg2;
            wait until bt_FSM_out = tseg1;
            
            mod_check_enabled <= true;
            
            -- Move to sample point! 
            wait until bt_FSM_out = tseg2;
            
            sync_control <= HARD_SYNC;
            
            for i in 0 to 10 loop
                wait until bt_FSM_out = tseg1;
                wait until bt_FSM_out = tseg2;
            end loop;
            
            wait until bt_fsm_out = tseg2;
            sync_control <= NO_SYNC;
            
            wait until bt_fsm_out = tseg1;
            wait until bt_fsm_out = tseg2;
            wait until bt_fsm_out = tseg1;
            
            
            loop_ctr <= loop_ctr + 1;
        end loop;

        evaluate_test(error_tol, error_ctr, status);
    end process;

end architecture;