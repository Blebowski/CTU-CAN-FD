Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;
use work.CANcomponents.ALL;
USE work.CANtestLib.All;
USE work.randomLib.All;

use work.ID_transfer.all;

-------------------------------------------------------------------------------------------------------------
--
-- CAN with Flexible Data-Rate IP Core 
--
-- Copyright (C) 2015 Ondrej Ille <ondrej.ille@gmail.com>
--
-- This program is free software; you can redistribute it and/or
-- modify it under the terms of the GNU General Public License
-- as published by the Free Software Foundation; either version 2
-- of the License, or (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.
--
-- The CAN protocol is developed by Robert Bosch GmbH and     
-- protected by patents. Anybody who wants to implement this    
-- IP core on silicon has to obtain a CAN protocol license
-- from Bosch.
--
--
-- Revision History:
--
--    6.6.2016   Created file
-----------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Unit test for the Interrupt manager.
--  Random interrupt source signals are generated in the testbench. Periodically random setting of interrupt
--  generator is used. After setting is set, test waits and evaluates whether interrupt prediction (int_test_ctr)
--  matches the actual number of interrupts measured on the int_out rising and falling edges. Also interrupt
--  vector is read and compared with modeled interrupt vector.                                              
-----------------------------------------------------------------------------------------------------------------

architecture int_man_unit_test of CAN_test is
    
    signal clk_sys                :   std_logic:='0'; --System Clock
    signal res_n                  :   std_logic:='0'; --Async Reset
    signal error_valid            :   std_logic:='0'; --Valid Error appeared for interrupt
    signal error_passive_changed  :   std_logic:='0'; --Error pasive /Error acitve functionality changed
    signal error_warning_limit    :   std_logic:='0'; --Error warning limit reached
    signal arbitration_lost       :   std_logic:='0'; --Arbitration was lost input
    signal wake_up_valid          :   std_logic:='0'; --Wake up appeared
    signal tx_finished            :   std_logic:='0'; --Message stored in CAN Core was sucessfully transmitted
    signal br_shifted             :   std_logic:='0'; --Bit Rate Was Shifted
    signal rx_message_disc        :   std_logic:='0'; --Income message was discarded
    signal rec_message_valid      :   std_logic:='0'; --Message recieved!
    signal rx_full                :   std_logic:='0';
    signal loger_finished         :   std_logic:='0';  --Event logging finsihed
    signal drv_bus                :   std_logic_vector(1023 downto 0):= (OTHERS =>'0');
    signal int_out                :   std_logic:='0'; --Interrupt output
    signal int_vector             :   std_logic_vector(10 downto 0) := (OTHERS =>'0');
    
    signal drv_bus_err_int_ena      :     std_logic:='0'; --Bus Error interrupt enable
    signal drv_arb_lst_int_ena      :     std_logic:='0'; --Arbitrarion lost interrupt enable
    signal drv_err_pas_int_ena      :     std_logic:='0'; --Error state changed interrupt enable
    signal drv_wake_int_ena         :     std_logic:='0'; --Wake up interrupt enable
    signal drv_dov_int_ena          :     std_logic:='0'; --Data OverRun interrupt enable
    signal drv_err_war_int_ena      :     std_logic:='0'; --Error warning limit reached
    signal drv_tx_int_ena           :     std_logic:='0'; --Frame sucessfully transcieved
    signal drv_rx_int_ena           :     std_logic:='0'; --Frame sucessfully recieved
    signal drv_log_fin_int_ena      :     std_logic:='0'; --Event logging finished interrupt enable
    signal drv_rx_full_int_ena      :     std_logic:='0'; --Recieve buffer full interrupt enable
    signal drv_brs_int_ena          :     std_logic:='0'; --Bit Rate Shift interrupt enable  

    signal drv_int_vect_erase       :     std_logic:='0'; --Logic 1 erases interrupt vector
    signal drv_int_vect_erase_prev  :     std_logic:='0';
    
    signal error_valid_r              :   std_logic:='0'; --Valid Error appeared for interrupt
    signal error_passive_changed_r    :   std_logic:='0'; --Error pasive /Error acitve functionality changed
    signal error_warning_limit_r      :   std_logic:='0'; --Error warning limit reached
    signal arbitration_lost_r         :   std_logic:='0'; --Arbitration was lost input
    signal wake_up_valid_r            :   std_logic:='0'; --Wake up appeared
    signal tx_finished_r              :   std_logic:='0'; --Message stored in CAN Core was sucessfully transmitted
    signal br_shifted_r               :   std_logic:='0'; --Bit Rate Was Shifted
    signal rx_message_disc_r          :   std_logic:='0'; --Income message was discarded
    signal rec_message_valid_r        :   std_logic:='0'; --Message recieved!
    signal rx_full_r                  :   std_logic:='0';
    signal loger_finished_r           :   std_logic:='0';  --Event logging finsihed
    
    
    --Test signals
    signal int_ctr                  :     natural:=0;
    signal int_test_ctr             :     natural:=0;
    signal int_test_mask            :     std_logic_vector(10 downto 0):=(OTHERS => '0');
    signal int_test_vector          :     std_logic_vector(10 downto 0):=(OTHERS => '0');
    signal rand_ctr_2               :     natural range 0 to RAND_POOL_SIZE;
    
    constant int_length             :     natural:=5;
    
    --Generates random setting of the interrupt enables
    procedure generate_setting(
      signal rand_ctr                 :inout   natural range 0 to RAND_POOL_SIZE;
      signal drv_bus_err_int_ena      :out     std_logic;
      signal drv_arb_lst_int_ena      :out     std_logic;
      signal drv_err_pas_int_ena      :out     std_logic;
      signal drv_wake_int_ena         :out     std_logic;
      signal drv_dov_int_ena          :out     std_logic;
      signal drv_err_war_int_ena      :out     std_logic;
      signal drv_tx_int_ena           :out     std_logic;
      signal drv_rx_int_ena           :out     std_logic;
      signal drv_log_fin_int_ena      :out     std_logic;
      signal drv_rx_full_int_ena      :out     std_logic;
      signal drv_brs_int_ena          :out     std_logic
    )is
    begin
      rand_logic(rand_ctr,drv_bus_err_int_ena,0.1);
      rand_logic(rand_ctr,drv_arb_lst_int_ena,0.1);
      rand_logic(rand_ctr,drv_err_pas_int_ena,0.1);
      rand_logic(rand_ctr,drv_wake_int_ena,0.1);
      rand_logic(rand_ctr,drv_dov_int_ena,0.1);
      rand_logic(rand_ctr,drv_err_war_int_ena,0.1);
      rand_logic(rand_ctr,drv_tx_int_ena,0.1);
      rand_logic(rand_ctr,drv_rx_int_ena,0.1);
      rand_logic(rand_ctr,drv_log_fin_int_ena,0.1);
      rand_logic(rand_ctr,drv_rx_full_int_ena,0.1);
      rand_logic(rand_ctr,drv_brs_int_ena,0.1);
    end procedure;  
    
    --Generates random interrupt sources for arbitrary amount of time
    procedure generate_sources(
      signal rand_ctr               :inout   natural range 0 to RAND_POOL_SIZE;
      signal error_valid            :inout   std_logic; --Valid Error appeared for interrupt
      signal error_passive_changed  :inout   std_logic; --Error pasive /Error acitve functionality changed
      signal error_warning_limit    :inout   std_logic; --Error warning limit reached
      signal arbitration_lost       :inout   std_logic; --Arbitration was lost input
      signal wake_up_valid          :inout   std_logic; --Wake up appeared
      signal tx_finished            :inout   std_logic; --Message stored in CAN Core was sucessfully transmitted
      signal br_shifted             :inout   std_logic; --Bit Rate Was Shifted
      signal rx_message_disc        :inout   std_logic; --Income message was discarded
      signal rec_message_valid      :inout   std_logic; --Message recieved!
      signal rx_full                :inout   std_logic;
      signal loger_finished         :inout   std_logic
    )is
    begin
      if(error_valid='1')then
        rand_logic(rand_ctr,error_valid,0.85);
      else
        rand_logic(rand_ctr,error_valid,0.1);
      end if;
      
      if(error_passive_changed='1')then
        rand_logic(rand_ctr,error_passive_changed,0.85);
      else
        rand_logic(rand_ctr,error_passive_changed,0.05);
      end if;
      
      if(error_warning_limit='1')then
        rand_logic(rand_ctr,error_warning_limit,0.85);
      else
        rand_logic(rand_ctr,error_warning_limit,0.05);
      end if;
      
      if(arbitration_lost='1')then
        rand_logic(rand_ctr,arbitration_lost,0.95);
      else
        rand_logic(rand_ctr,arbitration_lost,0.05);
      end if;
      
      if(wake_up_valid='1')then
        rand_logic(rand_ctr,wake_up_valid,0.95);
      else
        rand_logic(rand_ctr,wake_up_valid,0.05);
      end if;
      
      if(tx_finished='1')then
        rand_logic(rand_ctr,tx_finished,0.95);
      else
        rand_logic(rand_ctr,tx_finished,0.05);
      end if;
      
      if(br_shifted='1')then
        rand_logic(rand_ctr,br_shifted,0.95);
      else
        rand_logic(rand_ctr,br_shifted,0.05);
      end if;
      
      if(rx_message_disc='1')then
        rand_logic(rand_ctr,rx_message_disc,0.95);
      else
        rand_logic(rand_ctr,rx_message_disc,0.05);
      end if;
      
      if(rec_message_valid='1')then
        rand_logic(rand_ctr,rec_message_valid,0.95);
      else
        rand_logic(rand_ctr,rec_message_valid,0.05);
      end if;
      
      if(rx_full='1')then
        rand_logic(rand_ctr,rx_full,0.95);
      else
        rand_logic(rand_ctr,rx_full,0.05);
      end if;
      
      if(loger_finished='1')then
        rand_logic(rand_ctr,loger_finished,0.95);
      else
        rand_logic(rand_ctr,loger_finished,0.05);
      end if;
      
    end procedure;    
    
    
    --Clears the interrupt vector and compares the interrupt mask
    -- with reference value
    procedure process_interrupts(
      signal int_mask           :in     std_logic_vector(10 downto 0);
      signal int_test_mask      :in     std_logic_vector(10 downto 0);
      signal drv_int_vect_erase :inout  std_logic;
      signal clk_sys            :in     std_logic;
      signal test_ctr           :in     natural;
      signal int_ctr            :in     natural;
      variable outcome          :out    boolean
    )is
    begin
      wait until falling_edge(clk_sys);
      if(int_mask=int_test_mask and test_ctr=int_ctr)then
        outcome:=true;
      else
        outcome:=false;
      end if;
      
      drv_int_vect_erase<= '1';
      wait until rising_edge(clk_sys);
      wait until falling_edge(clk_sys);
      drv_int_vect_erase<= '0';
      
    end procedure;       
      
begin
  int_man_comp:intManager
  GENERIC map(
     int_length => int_length
    )
  PORT map(
     clk_sys               =>   clk_sys,
     res_n                 =>   res_n,
     error_valid           =>   error_valid,
     error_passive_changed =>   error_passive_changed,
     error_warning_limit   =>   error_warning_limit,
     arbitration_lost      =>   arbitration_lost,
     wake_up_valid         =>   wake_up_valid,
     tx_finished           =>   tx_finished ,
     br_shifted            =>   br_shifted,
     rx_message_disc       =>   rx_message_disc ,
     rec_message_valid     =>   rec_message_valid ,
     rx_full               =>   rx_full,
     loger_finished        =>   loger_finished,
     drv_bus               =>   drv_bus ,
     int_out               =>   int_out,
     int_vector            =>   int_vector
    
  );
  
   --Interrupt register masking and enabling
  int_test_mask(BUS_ERR_INT)       <=  drv_bus_err_int_ena   and error_valid             and (not error_valid_r);
  int_test_mask(ARB_LST_INT)       <=  drv_arb_lst_int_ena   and arbitration_lost        and (not arbitration_lost_r);
  int_test_mask(ERR_PAS_INT)       <=  drv_err_pas_int_ena   and error_passive_changed   and (not error_passive_changed_r);
  int_test_mask(WAKE_INT)          <=  drv_wake_int_ena      and wake_up_valid           and (not wake_up_valid_r);
  int_test_mask(DOV_INT)           <=  drv_dov_int_ena       and rx_message_disc         and (not rx_message_disc_r);
  int_test_mask(ERR_WAR_INT)       <=  drv_err_war_int_ena   and error_warning_limit     and (not error_warning_limit_r);
  int_test_mask(TX_INT)            <=  drv_tx_int_ena        and tx_finished             and (not tx_finished_r);
  int_test_mask(RX_INT)            <=  drv_rx_int_ena        and rec_message_valid       and (not rec_message_valid_r);
  int_test_mask(LOG_FIN_INT)       <=  drv_log_fin_int_ena   and loger_finished          and (not loger_finished_r);
  int_test_mask(RX_FULL_INT)       <=  drv_rx_full_int_ena   and rx_full and rec_message_valid and (not rec_message_valid_r);
  --Note: also rec_message_valid has to be compared otherwise interrupt would start always when the buffer is full 
  int_test_mask(BRS_INT)           <=  drv_brs_int_ena       and br_shifted; 
  
  
   drv_bus(DRV_BUS_ERR_INT_ENA_INDEX)  <= drv_bus_err_int_ena;
   drv_bus(DRV_ARB_LST_INT_ENA_INDEX)  <= drv_arb_lst_int_ena;
   drv_bus(DRV_ERR_PAS_INT_ENA_INDEX)  <= drv_err_pas_int_ena;
   drv_bus(DRV_WAKE_INT_ENA_INDEX)     <= drv_wake_int_ena;
   drv_bus(DRV_DOV_INT_ENA_INDEX)      <= drv_dov_int_ena;
   drv_bus(DRV_ERR_WAR_INT_ENA_INDEX)  <= drv_err_war_int_ena;
   drv_bus(DRV_TX_INT_ENA_INDEX)       <= drv_tx_int_ena;
   drv_bus(DRV_RX_INT_ENA_INDEX)       <= drv_rx_int_ena;
   drv_bus(DRV_LOG_FIN_INT_ENA_INDEX)  <= drv_log_fin_int_ena;
   drv_bus(DRV_RX_FULL_INT_ENA_INDEX)  <= drv_rx_full_int_ena;
   drv_bus(DRV_BRS_INT_ENA_INDEX)      <= drv_brs_int_ena;
   drv_bus(DRV_INT_VECT_ERASE_INDEX)   <= drv_int_vect_erase;
  
  ---------------------------------
  --Clock generation
  ---------------------------------
  clock_gen:process
  variable period   :natural:=f100_Mhz;
  variable duty     :natural:=50;
  variable epsilon  :natural:=0;
  begin
    generate_clock(period,duty,epsilon,clk_sys);
  end process;
  
  ---------------------------------
  --Counting the interrupts
  ---------------------------------
  int_counter:process
  begin
    wait until rising_edge(int_out);
    int_ctr<=int_ctr+1;
    wait until falling_edge(int_out);
  end process;  
  
  --------------------------------------
  -- Emulating the interrupt generation
  --------------------------------------
  int_emul:process
  begin
    if(int_test_mask="00000000000") then
      wait until int_test_mask /= "00000000000";
      wait for 1 ns;
      if(int_test_mask="00000000000") then
        wait until int_test_mask /= "00000000000";
      end if;
      wait for 4 ns;
    end if;
    
    int_test_ctr <= int_test_ctr+1;
    
    wait for (int_length+2)*10 ns;
    
  end process;
  
  ---------------------------------
  -- Generating random sources
  ---------------------------------
  src_gen:process
  begin
    wait for 195 ns;
    while true loop
      wait until falling_edge(clk_sys);
      generate_sources(rand_ctr, error_valid, error_passive_changed ,error_warning_limit ,
                        arbitration_lost, wake_up_valid, tx_finished, br_shifted,         
                        rx_message_disc , rec_message_valid , rx_full , loger_finished );
    end loop;
  end process;
  
  ---------------------------------
  -- Storing the interrupt mask--
  ---------------------------------
  int_msk_proc:process(clk_sys)
  begin
    if rising_edge(clk_sys)then
      drv_int_vect_erase_prev <= drv_int_vect_erase;
      
      error_valid_r               <= error_valid; 
      error_passive_changed_r     <= error_passive_changed; 
      error_warning_limit_r       <= error_warning_limit; 
      arbitration_lost_r          <= arbitration_lost; 
      wake_up_valid_r             <= wake_up_valid; 
      tx_finished_r               <= tx_finished; 
      br_shifted_r                <= br_shifted; 
      rx_message_disc_r           <= rx_message_disc; 
      rec_message_valid_r         <= rec_message_valid; 
      rx_full_r                   <= rx_full; 
      loger_finished_r            <= loger_finished; 
      
      if(drv_int_vect_erase='0' and drv_int_vect_erase_prev='1')then
        int_test_vector<=(OTHERS => '0');
      else
        int_test_vector<= int_test_vector OR int_test_mask;
      end if;
    end if;
    
  end process;
  
  ---------------------------------
  ---------------------------------
  --Main Test process
  ---------------------------------
  ---------------------------------
  test_proc:process
  variable outcome:boolean:=false;
  begin
    log("Restarting Interrupt test!",info_l,log_level);
    wait for 5 ns;
    reset_test(res_n,status,run,error_ctr);
    log("Restarted Interrupttest",info_l,log_level);
    print_test_info(iterations,log_level,error_beh,error_tol);
    
    -------------------------------
    --Main loop of the test
    -------------------------------
    log("Starting Interrupt main loop",info_l,log_level);
    
    while (loop_ctr<iterations  or  exit_imm)
    loop
      log("Starting loop nr "&integer'image(loop_ctr),info_l,log_level);
      
      --Generate the random setting of the interrupt manager
      generate_setting( rand_ctr_2,drv_bus_err_int_ena, drv_arb_lst_int_ena, drv_err_pas_int_ena, drv_wake_int_ena,         
                        drv_dov_int_ena , drv_err_war_int_ena , drv_tx_int_ena ,drv_rx_int_ena,           
                        drv_log_fin_int_ena , drv_rx_full_int_ena ,drv_brs_int_ena );             
      wait for 300 ns;
      
      process_interrupts( int_vector , int_test_vector , drv_int_vect_erase ,
                         clk_sys   , int_test_ctr      , int_ctr            ,outcome);
      if(outcome=false)then
        process_error(error_ctr,error_beh,exit_imm); 
        log("Error while evaluating interrupts!",error_l,log_level);
      end if;
      
      loop_ctr<=loop_ctr+1;
    end loop;
    
    evaluate_test(error_tol,error_ctr,status);
  end process;
  
end architecture;



-----------------------------------------------------------------------------------------------------------------
-- Test wrapper and control signals generator                                           
-----------------------------------------------------------------------------------------------------------------
architecture int_man_test_wrapper of CAN_test_wrapper is
  
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
  for test_comp : CAN_test use entity work.CAN_test(int_man_unit_test);
  
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


