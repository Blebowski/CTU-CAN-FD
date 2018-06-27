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
--    20.6.2016   Created file
-----------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Main environment for feature tests  
--                                       
-----------------------------------------------------------------------------------------------------------------


-----------------------------------------------------------------------------------------------------------------
-- Test implementation                                            
-----------------------------------------------------------------------------------------------------------------

architecture feature_env_test of CAN_feature_test is

  ----------------------------------------------
  -- CAN Node 
  ----------------------------------------------
  component CAN_top_level is
  generic(
      constant use_logger     :boolean  :=true; --Whenever event logger should be synthetised (not yet implemented)
      constant rx_buffer_size :natural  :=128; --Transcieve Buffer size
      constant useFDSize      :boolean  :=true; --Transcieve buffer size should be synthetised as FD Size (640 bits) or normal CAN (128 bits)
      constant use_sync       :boolean  :=true; --Whenever internal synchroniser chain should be used for incoming bus signals
                                                --Dont turn off unless external synchronisation chain is put on input of FPGA by
                                                --synthetiser
      constant ID             :natural  range 0 to 15:=1; --ID (bits  19-16 of adress) 
      constant logger_size    :natural --range 0 to 512:=8
  );
  port(
      signal clk_sys:in std_logic;
      signal res_n:in std_logic;
      signal data_in:in std_logic_vector(31 downto 0);
      signal data_out:out std_logic_vector(31 downto 0);
      signal adress:in std_logic_vector(23 downto 0);
    	 signal scs:in std_logic; --Chip select
      signal srd:in std_logic; --Serial read
      signal swr:in std_logic; --Serial write
      signal int:out std_logic;
      signal CAN_tx:out std_logic;
      signal CAN_rx:in std_logic;
      signal time_quanta_clk:out std_logic; --Time Quantum clocks possible to be used for synchronisation
      signal timestamp:in std_logic_vector(63 downto 0)
  );
  end component;
  
    --Controller 1 signals 
    signal clk_sys_1          : std_logic:= '0';
    signal res_n_1            : std_logic:= '0';
    signal int_1              : std_logic:= '0';
    signal CAN_tx_1           : std_logic:= RECESSIVE;
    signal CAN_rx_1           : std_logic:= RECESSIVE;
    signal time_quanta_clk_1  : std_logic:= '0'; --Time Quantum clocks possible to be used for synchronisation
    signal timestamp_1        : std_logic_vector(63 downto 0) := (OTHERS=>'0');
    
    signal data_in_1: std_logic_vector(31 downto 0):= (OTHERS=>'0');
    signal data_out_1: std_logic_vector(31 downto 0):= (OTHERS=>'0');
    signal adress_1: std_logic_vector(23 downto 0):= (OTHERS=>'0');
  	 signal scs_1: std_logic:= '0'; --Chip select
    signal srd_1: std_logic:= '0'; --Serial read
    signal swr_1: std_logic:= '0'; --Serial write
      
  
    --Controller 2 signals
    signal clk_sys_2          : std_logic:= '0';
    signal res_n_2            : std_logic:= '0';
    signal int_2              : std_logic:= '0';
    signal CAN_tx_2           : std_logic:= RECESSIVE;
    signal CAN_rx_2           : std_logic:= RECESSIVE;
    signal time_quanta_clk_2  : std_logic:= '0'; --Time Quantum clocks possible to be used for synchronisation
    signal timestamp_2        : std_logic_vector(63 downto 0):= (OTHERS=>'0');
    
    signal data_in_2: std_logic_vector(31 downto 0):= (OTHERS=>'0');
    signal data_out_2: std_logic_vector(31 downto 0):= (OTHERS=>'0');
    signal adress_2: std_logic_vector(23 downto 0):= (OTHERS=>'0');
  	 signal scs_2: std_logic:= '0'; --Chip select
    signal srd_2: std_logic:= '0'; --Serial read
    signal swr_2: std_logic:= '0'; --Serial write
    
    
    --------------------------------------------------
    --------------------------------------------------
    --Internal testbench signals
    --------------------------------------------------
    --------------------------------------------------
    signal hw_reset_on_new_test   : boolean := true;
    signal iteration_done         : boolean := false;
    
    --Test name to be loaded by the TCL script from TCL test FIFO
    --Note that string always have to have fixed length
    signal test_name              : string (1 to 20) :="            overload";
        
    --CAN bus signals
    signal bus_level              : std_logic       := RECESSIVE;
    signal tr_del_1_sr            : std_logic_vector(255 downto 0):= (OTHERS => RECESSIVE);
    signal tr_del_2_sr            : std_logic_vector(255 downto 0):= (OTHERS => RECESSIVE);
    signal tr_del_1               : natural         :=20;
    signal tr_del_2               : natural         :=20;
    
begin
  
  CAN_inst_1:CAN_top_level 
  generic map(
       use_logger       => true,
       rx_buffer_size   => 64,
       useFDSize        => true,
       use_sync         => true,
       ID               => 1,
       logger_size      => 16
  )
  port map(
       clk_sys            =>  clk_sys_1,
       res_n              =>  res_n_1,
       data_in            =>  data_in_1,
       data_out           =>  data_out_1,
       adress             =>  adress_1,
    	  scs                =>  scs_1,
       srd                =>  srd_1,
       swr                =>  swr_1,
       int                =>  int_1,
       CAN_tx             =>  CAN_tx_1,
       CAN_rx             =>  CAN_rx_1,
       time_quanta_clk    =>  time_quanta_clk_1,
       timestamp          =>  timestamp_1
  );
  
  CAN_inst_2:CAN_top_level
  generic map(
       use_logger       => true,
       rx_buffer_size   => 64,
       useFDSize        => true,
       use_sync         => true,
       ID               => 2,
       logger_size      => 16
  )
  port map(
       clk_sys            =>  clk_sys_2,
       res_n              =>  res_n_2,
       data_in            =>  data_in_2,
       data_out           =>  data_out_2,
       adress             =>  adress_2,
    	  scs                =>  scs_2,
       srd                =>  srd_2,
       swr                =>  swr_2,
       int                =>  int_2,
       CAN_tx             =>  CAN_tx_2,
       CAN_rx             =>  CAN_rx_2,
       time_quanta_clk    =>  time_quanta_clk_2,
       timestamp          =>  timestamp_2
  );
  
  -------------------------------------------------
  --Connect individual bus signals of memory buses
  -------------------------------------------------
  mem_bus_1.clk_sys       <=  clk_sys_1;
  data_in_1               <=  mem_bus_1.data_in;
  adress_1                <=  mem_bus_1.address;
  scs_1                   <=  mem_bus_1.scs;
  swr_1                 	 <=  mem_bus_1.swr;
  srd_1                   <=  mem_bus_1.srd;
  mem_bus_1.data_out      <=  data_out_1;
  
  mem_bus_2.clk_sys       <=  clk_sys_2;
  data_in_2               <=  mem_bus_2.data_in;
  adress_2                <=  mem_bus_2.address;
  scs_2                   <=  mem_bus_2.scs;
  swr_2                 	 <=  mem_bus_2.swr;
  srd_2                   <=  mem_bus_2.srd;
  mem_bus_2.data_out      <=  data_out_2;
  
  
  ---------------------------------
  --Transciever and bus realization
  ---------------------------------
  tr_1_proc:process
  begin
   wait until falling_edge(clk_sys_1);
   tr_del_1_sr <= tr_del_1_sr(254 downto 0)&CAN_tx_1;    
  end process;
  
  tr_2_proc:process
  begin
   wait until falling_edge(clk_sys_2);
   tr_del_2_sr <= tr_del_2_sr(254 downto 0)&CAN_tx_2;    
  end process;
  
  bus_level    <= tr_del_1_sr(tr_del_1) AND tr_del_2_sr(tr_del_2) when bl_force=false else
                  bl_inject;
                
  
  CAN_rx_1     <= bus_level;
  CAN_rx_2     <= bus_level;
  
  
  ---------------------------------
  --Clock generation (1)
  ---------------------------------
  clock_gen_1:process
  variable period   :natural:=f100_Mhz;
  variable duty     :natural:=50;
  variable epsilon  :natural:=0;
  begin
    generate_clock(period,duty,epsilon,clk_sys_1);
    timestamp_1 <= std_logic_vector(unsigned(timestamp_1)+1);
  end process; 
  
  ---------------------------------
  --Clock generation (2)
  ---------------------------------
  clock_gen_2:process
  variable period   :natural:=f100_Mhz;
  variable duty     :natural:=50;
  variable epsilon  :natural:=100;
  begin
    generate_clock(period,duty,epsilon,clk_sys_2);
    timestamp_2 <= std_logic_vector(unsigned(timestamp_2)+1);
  end process; 
  
  
  ---------------------------------
  ---------------------------------
  --Test process listening to the
  --  higher hierarchy wrapper!
  ---------------------------------
  ---------------------------------
  test_proc:process
  begin
    status<=waiting;
    loop_ctr<=0;
    wait for 500 ns;
    if(hw_reset_on_new_test=true)then
      log("HW Restart of feature test environment started!",info_l,log_level);
      wait for 5 ns;
      reset_test(res_n_1,status,run,error_ctr);
      reset_test(res_n_2,status,run,error_ctr);
      log("HW Restart of feature test environment finished",info_l,log_level);
    end if;
    
    --Status is restarted no matter the HW reset
    status<=running;
    
    print_test_info(iterations,log_level,error_beh,error_tol);
    
    
    -------------------------------
    --Main loop of the test
    -------------------------------
    while (loop_ctr<iterations  or  exit_imm)
    loop
      log("Starting loop nr "&integer'image(loop_ctr),info_l,log_level);
      --Wait on signal from higher level wrapper to move to the next iteration
      wait until iteration_done=true;

      loop_ctr<=loop_ctr+1;
    end loop;
    
    status<=passed;
    wait until run=false;
    wait until run=true;
    
  end process;
  
end architecture;


Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;


--Testbench packages
USE work.Arbitration_feature.All;
USE work.rx_status_feature.All;
USE work.forbid_fd_feature.All;
USE work.abort_transmittion_feature.All;
use work.rtr_pref_feature.All;
use work.tx_arb_time_tran_feature.All;
use work.traf_meas_feature.All;
use work.spec_mode_feature.All;
use work.interrupt_feature.All;
use work.soft_reset_feature.All;
use work.tran_delay_feature.All;
use work.invalid_config_feature.All;
use work.fault_conf_feature.All;
use work.retr_limit_feature.All;
use work.overload_feature.All;


-----------------------------------------------------------------------------------------------------------------
-- Test wrapper and control signals generator                                           
-----------------------------------------------------------------------------------------------------------------
architecture feature_env_test_wrapper of CAN_test_wrapper is  
  
  --Test component itself
  component CAN_feature_test is
  port (
    signal run            :in   boolean;               -- Input trigger, test starts running when true
    signal iterations     :in   natural;                -- Number of iterations that test should do
    signal log_level      :in   log_lvl_type;           -- Logging level, severity which should be shown
    signal error_beh      :in   err_beh_type;           -- Test behaviour when error occurs: Quit, or Go on
    signal error_tol      :in   natural;                -- Error tolerance, error counter should not
                                                         -- exceed this value in order for the test to pass
    signal status         :out  test_status_type;      -- Status of the test
    signal errors         :out  natural;                -- Amount of errors which appeared in the test
    
    signal mem_bus_1      :inout Avalon_mem_type;
    signal mem_bus_2      :inout Avalon_mem_type;
    
    signal bl_inject      :in   std_logic;
    signal bl_force       :in   boolean

  );
  end component;
  
  --Select architecture of the test
  for test_comp : CAN_feature_test use entity work.CAN_feature_test(feature_env_test);
  
    signal run             :    boolean;                -- Input trigger, test starts running when true                                                        -- exceed this value in order for the test to pass
    signal status_int      :    test_status_type;       -- Status of the test
    signal errors          :    natural;                -- Amount of errors which appeared in the test
    
    signal mem_bus_1      : Avalon_mem_type;
    signal mem_bus_2      : Avalon_mem_type;
     
    --Procedure for processing the feature tests!
    procedure exec_feature_test(
        --Common test parameters
        signal      test_name       :in     String;
        variable    outcome         :inout  boolean;
        signal      rand_ctr        :inout  natural range 0 to RAND_POOL_SIZE;
        --Additional signals for tests
        --Pretty much everything can be read out of stat bus...
        signal mem_bus_1            :inout  Avalon_mem_type;
        signal mem_bus_2            :inout  Avalon_mem_type;
        signal      int_1           :in     std_logic;
        signal      int_2           :in     std_logic;
        signal      bus_level       :in     std_logic;
        signal      drv_bus_1       :in     std_logic_vector(1023 downto 0);
        signal      drv_bus_2       :in     std_logic_vector(1023 downto 0);
        signal      stat_bus_1      :in     std_logic_vector(511 downto 0);
        signal      stat_bus_2      :in     std_logic_vector(511 downto 0);
        signal      bl_inject       :inout  std_logic;
        signal      bl_force        :inout  boolean
    )is
    begin
      outcome:=false;
      
      if(test_name="         arbitration")then
        arbitration_feature_exec(outcome,rand_ctr,mem_bus_1,mem_bus_2,bus_level,
                                  drv_bus_1,drv_bus_2,stat_bus_1,stat_bus_2);
                                  
      elsif(test_name="           rx_status")then
        rx_status_feature_exec(outcome,rand_ctr,mem_bus_1,mem_bus_2,bus_level,
                                drv_bus_1,drv_bus_2,stat_bus_1,stat_bus_2);
                                
      elsif(test_name="           forbid_fd")then
        forbid_fd_feature_exec(outcome,rand_ctr,mem_bus_1,mem_bus_2,bus_level,
                                drv_bus_1,drv_bus_2,stat_bus_1,stat_bus_2);
      
      elsif(test_name="  abort_transmittion")then
        abort_transmittion_feature_exec(outcome,rand_ctr,mem_bus_1,mem_bus_2,bus_level,
                                drv_bus_1,drv_bus_2,stat_bus_1,stat_bus_2);
      
      elsif(test_name="            rtr_pref")then
        rtr_pref_feature_exec(outcome,rand_ctr,mem_bus_1,mem_bus_2,bus_level,
                                drv_bus_1,drv_bus_2,stat_bus_1,stat_bus_2);
      
      elsif(test_name="    tx_arb_time_tran")then
        tx_arb_time_tran_feature_exec(outcome,rand_ctr,mem_bus_1,mem_bus_2,bus_level,
                                drv_bus_1,drv_bus_2,stat_bus_1,stat_bus_2);
      
      elsif(test_name="        traf_measure")then
        traf_meas_feature_exec(outcome,rand_ctr,mem_bus_1,mem_bus_2,bus_level,
                                drv_bus_1,drv_bus_2,stat_bus_1,stat_bus_2);
      
      elsif(test_name="           spec_mode")then
        spec_mode_feature_exec(outcome,rand_ctr,mem_bus_1,mem_bus_2,bus_level,
                                drv_bus_1,drv_bus_2,stat_bus_1,stat_bus_2);
      
      elsif(test_name="           interrupt")then
        interrupt_feature_exec(outcome,rand_ctr,mem_bus_1,mem_bus_2,int_1,int_2,
                                bus_level,drv_bus_1,drv_bus_2,stat_bus_1,stat_bus_2);                                                                                                                                 
      
      elsif(test_name="          soft_reset")then
        soft_reset_feature_exec(outcome,rand_ctr,mem_bus_1,mem_bus_2,bus_level,
                                drv_bus_1,drv_bus_2,stat_bus_1,stat_bus_2);
      
      elsif(test_name="           trv_delay")then
        tran_delay_feature_exec(outcome,rand_ctr,mem_bus_1,mem_bus_2,bus_level,
                                drv_bus_1,drv_bus_2,stat_bus_1,stat_bus_2);
      
      elsif(test_name="     invalid_configs")then
        invalid_config_feature_exec(outcome,rand_ctr,mem_bus_1,mem_bus_2,bus_level,
                                drv_bus_1,drv_bus_2,stat_bus_1,stat_bus_2);                         
      
      elsif(test_name="   fault_confinement")then
        fault_conf_feature_exec(outcome,rand_ctr,mem_bus_1,mem_bus_2,bus_level,
                                drv_bus_1,drv_bus_2,stat_bus_1,stat_bus_2);
      
      elsif(test_name="          retr_limit")then
        retr_limit_feature_exec(outcome,rand_ctr,mem_bus_1,mem_bus_2,bus_level,
                                drv_bus_1,drv_bus_2,stat_bus_1,stat_bus_2);
                                                          
      elsif(test_name="            overload")then
        overload_feature_exec(outcome,rand_ctr,mem_bus_1,mem_bus_2,bus_level,
                                drv_bus_1,drv_bus_2,stat_bus_1,stat_bus_2,bl_inject,bl_force);                                                    
                                                          
      end if;
      
    end procedure;
    
    
    procedure restart_mem_bus(
      signal mem_bus_1          :out  Avalon_mem_type;
      signal mem_bus_2          :out  Avalon_mem_type
    )is begin
      mem_bus_1.scs         <= '0';
      mem_bus_1.swr         <= '0';
      mem_bus_1.srd         <= '0';
      mem_bus_1.address     <= (OTHERS =>'0');
      mem_bus_1.data_in     <= (OTHERS =>'0');
      mem_bus_1.clk_sys     <= 'Z';
      mem_bus_1.data_out    <= (OTHERS =>'Z');
      
      mem_bus_2.scs         <= '0';
      mem_bus_2.swr         <= '0';
      mem_bus_2.srd         <= '0';
      mem_bus_2.address     <= (OTHERS =>'0');
      mem_bus_2.data_in     <= (OTHERS =>'0');
      mem_bus_2.clk_sys     <= 'Z';
      mem_bus_2.data_out    <= (OTHERS =>'Z');
    end procedure;
  
  
  --Additional signals definitions
  signal error_ctr      : natural:=0;
  signal exit_imm       : boolean:=false;
  signal error_tol_int  : natural;
  signal error_beh_int  : err_beh_type;
  
  signal bl_inject      :   std_logic:=RECESSIVE;
  signal bl_force       :   boolean:=false;
  
begin
  
  error_tol_int           <=  error_tol;
  error_beh_int           <=  error_beh;
  
  --In this test wrapper generics are directly connected to the signals
  -- of test entity
  test_comp:CAN_feature_test
  port map(
     run              =>  run,
     iterations       =>  iterations ,
     log_level        =>  log_level,
     error_beh        =>  error_beh,
     error_tol        =>  error_tol,                                                     
     status           =>  status_int,
     errors           =>  errors,
     mem_bus_1        =>  mem_bus_1,
     mem_bus_2        =>  mem_bus_2,
     bl_inject        =>  bl_inject,
     bl_force         =>  bl_force
  );
  
  ---------------------------------------
  ---------------------------------------
  --Starts the test and lets it run
  ---------------------------------------
  ---------------------------------------
  test:process
    variable outcome :boolean := false;
    
    alias iteration_done is <<signal test_comp.iteration_done : boolean>>;
    alias hw_reset       is <<signal test_comp.hw_reset_on_new_test : boolean>>;
    alias test_name      is <<signal test_comp.test_name : string(1 to 20)>>;
    alias hw_reset_1     is <<signal test_comp.res_n_1  : std_logic>>;
    alias hw_reset_2     is <<signal test_comp.res_n_2  : std_logic>>;
    
    --Internal signals of CAN controllers
    alias bus_level      is <<signal test_comp.bus_level : std_logic>>;
    alias drv_bus_1      is <<signal test_comp.CAN_inst_1.drv_bus :std_logic_vector(1023 downto 0)>>;
    alias drv_bus_2      is <<signal test_comp.CAN_inst_2.drv_bus :std_logic_vector(1023 downto 0)>>;
    alias stat_bus_1     is <<signal test_comp.CAN_inst_1.stat_bus :std_logic_vector(511 downto 0)>>;
    alias stat_bus_2     is <<signal test_comp.CAN_inst_2.stat_bus :std_logic_vector(511 downto 0)>>;
    alias rand_ctr       is <<signal test_comp.rand_ctr : natural range 0 to RAND_POOL_SIZE>>;
    alias int_1          is <<signal test_comp.int_1    : std_logic>>;
    alias int_2          is <<signal test_comp.int_2    : std_logic>>;
    variable    ID_1    : natural range 0 to 15 :=1;
    variable    ID_2    : natural range 0 to 15 :=2;
  begin
    
    --Set the process to run and wait until it comes out of reset
    iteration_done    <= false;
    hw_reset          <= true;
    run               <= true;
    error_ctr         <= 0;
    restart_mem_bus(mem_bus_1,mem_bus_2);
     
    wait for 10 ns;
    wait until hw_reset_1='1' and hw_reset_2='1';
    wait for 10 ns;
    
    status            <= running;
    
    --Execute the controllers configuration
    CAN_turn_controller(true,ID_1,mem_bus_1);
    CAN_turn_controller(true,ID_2,mem_bus_2);
    
    --Set default retransmitt limit to 0
    -- Failed frames are not retransmitted
    -- by default!!!
    CAN_enable_retr_limit(true,0,ID_1,mem_bus_1);
    CAN_enable_retr_limit(true,0,ID_2,mem_bus_2);
    
    
    -------------------------------------------------
    -- Main test loop
    -------------------------------------------------
    while (status_int=running) loop
      iteration_done <=false;
      
      exec_feature_test(test_name,outcome,rand_ctr,mem_bus_1,mem_bus_2,int_1,int_2,bus_level,
                       drv_bus_1,drv_bus_2,stat_bus_1,stat_bus_2,bl_inject,bl_force);
                       
      if(outcome=false)then
        process_error(error_ctr,error_beh_int,exit_imm);
      end if;
      
      wait for 200 ns;
      iteration_done <= true;
      wait for 10 ns;
    end loop;
      
    run               <= false; 
    evaluate_test(error_tol_int,error_ctr,status);   
    wait for 100 ns;
    
         
  end process;
  
  
end;
