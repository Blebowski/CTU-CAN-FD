Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;

use work.ID_transfer.all;

-----------------------------------------------------------------------------------------------------------------
-- Author:      Ondrej Ille , Czech Technical University, FEL
-- Device:      Altera FPGA - Cyclone IV
-- Begin Date:  July 2015
-- Project:     CAN FD IP Core Project
--
-- Revision History Date Author Comments:
--
--    7.6.2016   Created file
-----------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Unit test for the prescaler circuit. At the time of implementation prescaler_v3.vhd was used!                                                 
-----------------------------------------------------------------------------------------------------------------


-----------------------------------------------------------------------------------------------------------------
-- Test implementation                                            
-----------------------------------------------------------------------------------------------------------------

architecture presc_unit_test of CAN_test is
  component prescaler_v3 is
  PORT(
    signal clk_sys              :in std_logic;  --System clock
    signal res_n                :in std_logic;   --Async reset
    signal sync_edge            :in std_logic;        --Edge for synchronisation
    signal OP_State             :in oper_mode_type;   --Protocol control state
    signal drv_bus              :in std_logic_vector(1023 downto 0);    
    signal clk_tq_nbt           :out std_logic; --Time quantum clock - Nominal bit time
    signal clk_tq_dbt           :out std_logic; --bit time - Nominal bit time
    signal sample_nbt           :out std_logic; --Sample signal for nominal bit time
    signal sample_dbt           :out std_logic; --Sample signal of data bit time
    signal sample_nbt_del_1     :out std_logic;
    signal sample_dbt_del_1     :out std_logic;
    signal sample_nbt_del_2     :out std_logic;
    signal sample_dbt_del_2     :out std_logic;
    signal sync_nbt             :out std_logic;
    signal sync_dbt             :out std_logic;
    signal sync_nbt_del_1       :out std_logic;
    signal sync_dbt_del_1       :out std_logic;    
    signal data_tx              :in  std_logic;
    signal bt_FSM_out           :out bit_time_type;
    signal hard_sync_edge_valid :out std_logic; 
    signal sp_control           :in std_logic_vector(1 downto 0);
    signal sync_control         :in std_logic_vector(1 downto 0)
  );
end component;

    signal clk_sys              : std_logic:='0';  --System clock
    signal res_n                : std_logic:='0';   --Async reset
    signal sync_edge            : std_logic:='0';        --Edge for synchronisation
    signal OP_State             : oper_mode_type:=reciever;   --Protocol control state
    signal drv_bus              : std_logic_vector(1023 downto 0):=(OTHERS =>'0');    
    signal clk_tq_nbt           : std_logic:='0'; --Time quantum clock - Nominal bit time
    signal clk_tq_dbt           : std_logic:='0'; --bit time - Nominal bit time
    signal sample_nbt           : std_logic:='0'; --Sample signal for nominal bit time
    signal sample_dbt           : std_logic:='0'; --Sample signal of data bit time
    signal sample_nbt_del_1     : std_logic:='0';
    signal sample_dbt_del_1     : std_logic:='0';
    signal sample_nbt_del_2     : std_logic:='0';
    signal sample_dbt_del_2     : std_logic:='0';
    signal sync_nbt             : std_logic:='0';
    signal sync_dbt             : std_logic:='0';
    signal sync_nbt_del_1       : std_logic:='0';
    signal sync_dbt_del_1       : std_logic:='0';    
    signal bt_FSM_out           : bit_time_type;
    signal hard_sync_edge_valid : std_logic:='0'; 
    signal sp_control           : std_logic_vector(1 downto 0):=(OTHERS =>'0');
    signal sync_control         : std_logic_vector(1 downto 0):=(OTHERS =>'0');
    signal data_tx              : std_logic;
    
    --Driving bus aliases
    signal drv_tq_nbt           :   std_logic_vector (5 downto 0):=(OTHERS =>'0'); 
    signal drv_tq_dbt           :   std_logic_vector (5 downto 0):=(OTHERS =>'0');
    signal drv_prs_nbt          :   std_logic_vector (5 downto 0):=(OTHERS =>'0'); 
    signal drv_ph1_nbt          :   std_logic_vector (5 downto 0):=(OTHERS =>'0');  
    signal drv_ph2_nbt          :   std_logic_vector (5 downto 0):=(OTHERS =>'0'); 
    signal drv_prs_dbt          :   std_logic_vector (3 downto 0):=(OTHERS =>'0'); 
    signal drv_ph1_dbt          :   std_logic_vector (3 downto 0):=(OTHERS =>'0');  
    signal drv_ph2_dbt          :   std_logic_vector (3 downto 0):=(OTHERS =>'0'); 
    signal drv_sjw_nbt          :   std_logic_vector(3 downto 0):=(OTHERS =>'0'); 
    signal drv_sjw_dbt          :   std_logic_vector(3 downto 0):=(OTHERS =>'0'); 
    
    ---------------------------------------
    --Internal test signals and constants
    ---------------------------------------
    signal setting              :   presc_drv_type:= ((OTHERS=>'0'),(OTHERS=>'0'),(OTHERS=>'0'),(OTHERS=>'0'),(OTHERS=>'0'),
                                                      (OTHERS=>'0'),(OTHERS=>'0'),(OTHERS=>'0'),(OTHERS=>'0'),(OTHERS=>'0'));
    signal trig_signals         :   presc_triggers_type;
    constant  inf_proc_time     :   natural := 4; --Information processing time
    signal clock_counter        :   natural :=0;
    
    --Additional error counters
    signal ipt_err_ctr            :   natural:=0;
    signal coh_err_ctr            :   natural:=0;
    signal sync_seq_err_ctr       :   natural:=0;
    signal sample_seq_err_ctr     :   natural:=0;
    signal main_err_ctr           :   natural:=0;
    
    --Additional exit_imm
    signal exit_imm_2             :   boolean:=false; 
    signal exit_imm_3             :   boolean:=false;
    signal exit_imm_4             :   boolean:=false;
    signal exit_imm_5             :   boolean:=false;
     
    --Generates random bit timing settings
    procedure gen_bit_time_setting(
      signal rand_ctr           : inout natural range 0 to RAND_POOL_SIZE;
      signal setting            : inout presc_drv_type
    )is
    begin
      rand_logic_vect_bt(rand_ctr,setting.drv_tq_nbt,0,0.2);
      rand_logic_vect_bt(rand_ctr,setting.drv_tq_dbt,0,0.1);
      
      rand_logic_vect_bt(rand_ctr,setting.drv_prs_nbt,0,0.4);
      rand_logic_vect_bt(rand_ctr,setting.drv_ph1_nbt,0,0.2);
      
      rand_logic_vect_bt(rand_ctr,setting.drv_ph2_nbt,0,0.2);
      rand_logic_vect_bt(rand_ctr,setting.drv_prs_dbt,0,0.3);
      rand_logic_vect_bt(rand_ctr,setting.drv_ph1_dbt,0,0.15);
      rand_logic_vect_bt(rand_ctr,setting.drv_ph2_dbt,0,0.15);
      
      rand_logic_vect(rand_ctr,setting.drv_sjw_nbt,0.2);
      rand_logic_vect(rand_ctr,setting.drv_sjw_dbt,0.2);
      
      --Here we check that settings are matching IPT!!
      --This is stated in documentation and is up to responsible 
      -- user to set. Otherwise controller is not working!
      
      --NBT
      if(setting.drv_tq_nbt="000001" and unsigned(setting.drv_ph2_nbt)<4)then
        setting.drv_ph2_nbt <= "000100";
      end if;
      if((setting.drv_tq_nbt="000010" or setting.drv_tq_nbt="000011") and unsigned(setting.drv_ph2_nbt)<2)then
        setting.drv_ph2_nbt <= "000010";
      end if;
      
      --DBT
      if(setting.drv_tq_dbt="0001" and unsigned(setting.drv_ph2_dbt)<4)then
        setting.drv_ph2_dbt <= "0100";
      end if;
      if((setting.drv_tq_dbt="0010" or setting.drv_tq_dbt="0011") and unsigned(setting.drv_ph2_dbt)<2)then
        setting.drv_ph2_dbt <= "0010";
      end if;
      
    end procedure;
    
    ------------------------------------------------------
    --Generates random synchronisation edges coming from
    -- bus synchronizer/sampler circuit
    ------------------------------------------------------
    procedure generate_sync_edges(
      signal rand_ctr         :  inout  natural range 0 to RAND_POOL_SIZE;
      signal sync_edge        :  out    std_logic
    )is
    begin
      
    end procedure;
    
    
    procedure count_cycles_until(
      signal    clk_sys          : in      std_logic;
      variable  counter          : inout   natural;
      signal    condition1       : in      std_logic;
      signal    condition2       : in      std_logic
    )is
    begin
      counter:=0;
      while condition1='0' and condition2='0' loop
        wait until falling_edge(clk_sys);
        counter:= counter+1;
      end loop;      
    end procedure;
    
    
    
begin
  
  ---------------------------------
  --Instance of Prescaler
  ---------------------------------
  prescaler_comp:prescaler_v3
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
  drv_bus(DRV_SJW_DBT_HIGH downto DRV_SJW_DBT_LOW)   <= drv_sjw_dbt;
  
  
  ----------------------------------------
  --Joining of signal groups into records
  ----------------------------------------
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
  
   trig_signals.sample_nbt        <=  sample_nbt ;      
   trig_signals.sample_dbt        <=  sample_dbt ;
   trig_signals.sample_nbt_del_1  <=  sample_nbt_del_1;
   trig_signals.sample_dbt_del_1  <=  sample_dbt_del_1 ;
   trig_signals.sample_nbt_del_2  <=  sample_nbt_del_2 ;
   trig_signals.sample_dbt_del_2  <=  sample_dbt_del_2 ;
   trig_signals.sync_nbt          <=  sync_nbt;
   trig_signals.sync_dbt          <=  sync_dbt ;
   trig_signals.sync_nbt_del_1    <=  sync_nbt_del_1;
   trig_signals.sync_dbt_del_1    <=  sync_dbt_del_1 ;
  
  
  
  ---------------------------------
  --Clock generation
  ---------------------------------
  clock_gen:process
  variable period   :natural:=f100_Mhz;
  variable duty     :natural:=50;
  variable epsilon  :natural:=0;
  begin
    generate_clock(period,duty,epsilon,clk_sys);
    clock_counter<=clock_counter+1;
  end process;  
  
--------------------------------------------
-- Checking of Information processing time
-------------------------------------------- 
ipt_proc_check:process 
variable store:natural:=0;
begin
  if( sp_control = NOMINAL_SAMPLE)then
     
     wait until rising_edge(sample_nbt);
     store:= clock_counter; 
     wait until rising_edge(sync_nbt); 
     if(clock_counter-store<inf_proc_time)then 
      log("Information processing time corrupted",error_l,log_level);
       process_error(ipt_err_ctr,error_beh,exit_imm_2); 
     end if; 
  
  elsif( sp_control = DATA_SAMPLE) then
 
    wait until rising_edge(sample_dbt); 
     store:= clock_counter;
     wait until rising_edge(sync_dbt);
     if(clock_counter-store<inf_proc_time)then 
        log("Information processing time corrupted",error_l,log_level);
        process_error(ipt_err_ctr,error_beh,exit_imm_2);
     end if; 
 
  else
    --TODO...
  end if;
 
end process;
 
 
--------------------------------------------------
-- Checking that two consecutive sync or sample
-- signals are not present!!
--------------------------------------------------
 trig_coherency_proc:process
 variable was_sync : boolean:=false;
 begin
  if( sp_control = NOMINAL_SAMPLE)then
    wait until rising_edge(sync_nbt) or rising_edge(sample_nbt);
    
    if (sync_nbt='1')then
      
      --Here error occures due to two consecutive sync signals
      if (was_sync=true)then
        log("Two consecutive sync signals!",error_l,log_level);
        process_error(coh_err_ctr,error_beh,exit_imm_3);
      end if; 
      was_sync:=true;
      
    elsif(sample_nbt='1')then
      
       --Here error occures due to two consecutive sample signals
      if (was_sync=false)then
        log("Two consecutive sample signals!",error_l,log_level);
        process_error(coh_err_ctr,error_beh,exit_imm_3);
      end if;  
      was_sync:=false;
      
    end if;
  
  elsif( sp_control = DATA_SAMPLE or sp_control = SECONDARY_SAMPLE) then
    wait until rising_edge(sync_dbt) or rising_edge(sample_dbt);
    
    if (sync_dbt='1')then
      
      --Here error occures due to two consecutive sync signals
      if (was_sync=true)then
        log("Two consecutive sync signals!",error_l,log_level);
        process_error(coh_err_ctr,error_beh,exit_imm_3);
      end if;
      was_sync:=true;
      
    elsif(sample_dbt='1')then
      
       --Here error occures due to two consecutive sample signals
      if (was_sync=false)then
        log("Two consecutive sample signals!",error_l,log_level);
        process_error(coh_err_ctr,error_beh,exit_imm_3);
      end if;
      was_sync:=false;
      
    end if;
  end if;
 end process; 


  --------------------------------------------------
  -- Checking that all sync signals in the sequence
  -- are generated every time!!
  -------------------------------------------------
  sync_seq_check_proc:process
  begin
    wait until rising_edge(sync_nbt) or rising_edge(sync_dbt);
    
    wait for 15 ns; --One ad half clock cycle
    if(sync_nbt_del_1='0' and sync_dbt_del_1='0')then
       log("Sync sequnce not complete, delay 1 CLK signal missing!",error_l,log_level);
       process_error(sync_seq_err_ctr,error_beh,exit_imm_4);  
    end if;
    
  end process;
  
  
  --------------------------------------------------
  -- Checking that all sample signals in the sequence
  -- are generated every time!!
  -------------------------------------------------
  sample_seq_check_proc:process
  begin
    wait until rising_edge(sample_nbt) or rising_edge(sample_dbt);
    
    wait for 15 ns; --One ad half clock cycle
    if(sample_nbt_del_1='0' and sample_dbt_del_1='0')then
       log("Sample sequnce not complete, delay 1 CLK signal missing!",error_l,log_level);
       process_error(sample_seq_err_ctr,error_beh,exit_imm_5);  
    end if;
    
    wait for 10 ns;
    
    if(sample_nbt_del_2='0' and sample_dbt_del_2='0')then
       log("Sample sequnce not complete, delay 2 CLK signal missing!",error_l,log_level);
       process_error(sample_seq_err_ctr,error_beh,exit_imm_5);  
    end if;
    
  end process;
  
  --Sum of the error counters
  error_ctr <= sample_seq_err_ctr + sync_seq_err_ctr + coh_err_ctr + ipt_err_ctr + main_err_ctr;
  
  ---------------------------------
  ---------------------------------
  --Main Test process
  ---------------------------------
  ---------------------------------
  test_proc:process
  variable rand_real_value  :real;
  variable check_ctr        :natural:=0;
  variable exp_dur          :integer;
  begin
    log("Restarting Prescaler unit test!",info_l,log_level);
    wait for 5 ns;
    reset_test(res_n,status,run,main_err_ctr);
    log("Restarted Prescaler unit test",info_l,log_level);
    print_test_info(iterations,log_level,error_beh,error_tol);
    
    -------------------------------
    --Main loop of the test
    -------------------------------
    log("Starting Prescaler unit main loop",info_l,log_level);
    
    while (loop_ctr<iterations  or  exit_imm)
    loop
      log("Starting loop nr "&integer'image(loop_ctr),info_l,log_level);
      
      --Generates random bit time settings
      gen_bit_time_setting(rand_ctr,setting); 
      
      --Sets random sampling
      rand_real_v(rand_ctr,rand_real_value);
      if(rand_real_value>0.5)then
        sp_control  <= DATA_SAMPLE;
      else
        sp_control <= NOMINAL_SAMPLE;
      end if;
      wait for 0 ns;
      
      -------------------------
      --Set no synchronization
      -------------------------
      sync_control  <= NO_SYNC;
      for i in 1 to 20 loop
        wait until rising_edge(sync_nbt) or rising_edge(sync_dbt);
        count_cycles_until(clk_sys,check_ctr,sample_nbt,sample_dbt);
        
        if(sp_control=NOMINAL_SAMPLE)then
          exp_dur:= (( ( to_integer(unsigned(drv_ph1_nbt)) + to_integer(unsigned(drv_prs_nbt)) +1) *
                       to_integer(unsigned(drv_tq_nbt)) 
                     ) +1
                    ); 
          if( check_ctr /= exp_dur )then
            log("SYNC+PROP+PH1(Data) did not last expected time!",error_l,log_level);
            process_error(main_err_ctr,error_beh,exit_imm);
          end if;
        else
          exp_dur:= (( ( to_integer(unsigned(drv_ph1_dbt)) + to_integer(unsigned(drv_prs_dbt)) +1) *
                       to_integer(unsigned(drv_tq_dbt)) 
                     ) +1
                    );
          if( check_ctr /= exp_dur )then
            log("SYNC+PROP+PH1(Nominal) did not last expected time!",error_l,log_level);
            process_error(main_err_ctr,error_beh,exit_imm);
          end if;
        end if;
        
      end loop;
      
      loop_ctr<=loop_ctr+1;
    end loop;
    
    evaluate_test(error_tol,error_ctr,status);
  end process;
  
end architecture;



-----------------------------------------------------------------------------------------------------------------
-- Test wrapper and control signals generator                                           
-----------------------------------------------------------------------------------------------------------------
architecture presc_unit_test_wrapper of CAN_test_wrapper is
  
  --Test component itself
  component CAN_test is
  port (
    signal run            :in   boolean;                -- Input trigger, test starts running when true
    signal iterations     :in   natural;                -- Number of iterations that test should do
    signal log_level      :in   log_lvl_type;           -- Logging level, severity which should be shown
    signal error_beh      :in   err_beh_type;           -- Test behaviour when error occurs: Quit, or Go on
    signal error_tol      :in   natural;                -- Error tolerance, error counter should not
                                                         -- exceed this value in order for the test to pass
    signal status         :out  test_status_type;      -- Status of the test
    signal errors         :out  natural                -- Amount of errors which appeared in the test
    --TODO: Error log results 
  );
  end component;
  
  --Select architecture of the test
  for test_comp : CAN_test use entity work.CAN_test(presc_unit_test);
  
    signal run              :   boolean;                -- Input trigger, test starts running when true                                                        -- exceed this value in order for the test to pass
    signal status_int       :   test_status_type;      -- Status of the test
    signal errors           :   natural;                -- Amount of errors which appeared in the test

begin
  
  --In this test wrapper generics are directly connected to the signals
  -- of test entity
  test_comp:CAN_test
  port map(
     run              =>  run,
     iterations       =>  iterations , 
     log_level        =>  log_level,
     error_beh        =>  error_beh,
     error_tol        =>  error_tol,                                                     
     status           =>  status_int,
     errors           =>  errors
  );
  
  status              <= status_int;
  
  ------------------------------------
  --Starts the test and lets it run
  ------------------------------------
  test:process
  begin
    run               <= true;
    wait for 1 ns;
    
    --Wait until the only test finishes and then propagate the results
    wait until (status_int=passed or status_int=failed);  
    
    wait for 100 ns;
    run               <= false;
        
  end process;
  
  
end;


