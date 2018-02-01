--------------------------------------------------------------------------------
-- 
-- CAN with Flexible Data-Rate IP Core 
-- 
-- Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
-- 
-- Project advisor: Jiri Novak <jnovak@fel.cvut.cz>
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
--  Unit test for the Event Logger circuit.
--   For simplicity always only one type of event is logged at a time!                     
--------------------------------------------------------------------------------
-- Revision History:
--    4.6.2016   Created file
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Test implementation                                            
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

architecture Event_logger_unit_test of CAN_test is
  
    signal clk_sys              :   std_logic:='0';
    signal res_n                :   std_logic:='0';
    signal drv_bus              :   std_logic_vector(1023 downto 0):=(OTHERS => '0');
    signal stat_bus             :   std_logic_vector(511 downto 0):=(OTHERS => '0');
    signal sync_edge            :   std_logic:='0';
    signal data_overrun         :   std_logic:='0';
    signal timestamp            :   std_logic_vector(63 downto 0):=(OTHERS => '0');
    signal bt_FSM               :   bit_time_type;
    signal loger_finished       :   std_logic; --Logger finished interrrupt output
    signal loger_act_data       :   std_logic_vector(63 downto 0):=(OTHERS => '0');
    signal log_write_pointer    :   std_logic_vector(7 downto 0):=(OTHERS => '0');
    signal log_read_pointer     :   std_logic_vector(7 downto 0):=(OTHERS => '0');
    signal log_size             :   std_logic_vector(7 downto 0):=(OTHERS => '0');
    signal log_state_out        :   logger_state_type;
    
    constant event_amount       :   integer:=21;
    constant trig_amount        :   integer:=16;
    
    signal PC_State             :   protocol_type:=sof;
    signal OP_State             :   oper_mode_type:=transciever;
    signal stat_bus_short       :   std_logic_vector(505 downto 0):=(OTHERS => '0');
    
    
    --Additional random counters
    signal rand_ctr_2           :   natural range 0 to RAND_POOL_SIZE:=0;
    
    
    ------------------------------------------------------------
    -- Generates random setting of event logger to capture 
    -- mostly one event at a time!
    -- Sets the trigger to the same event type as the
    --  recorded event
    ------------------------------------------------------------
    procedure generate_setting(
      signal    rand_ctr           : inout  natural range 0 to RAND_POOL_SIZE;
      signal    drv_bus            : inout  std_logic_vector(1023 downto 0);
       variable  ev_type            : inout  integer
    )is
    variable rand_val           : real:=0.0;
    variable rand_index         : integer:=0;
    begin
      rand_real_v(rand_ctr,rand_val);
      rand_index  := integer(rand_val*(real(trig_amount)-1.0));
      drv_bus     <= (OTHERS => '0');
      
      --Set random trigger
      drv_bus(552+trig_amount)  <=  '1';
      wait for 0 ns;
      
      --Set random event to record
      rand_real_v(rand_ctr,rand_val);
      ev_type  := integer(rand_val*real(event_amount-1));
      drv_bus(580+ev_type)   <=  '1';
      
      wait for 0 ns;
      
      --Start the event logger
      drv_bus(DRV_LOG_CMD_STR_INDEX)<=  '1';
      wait for 10 ns;
      drv_bus(DRV_LOG_CMD_STR_INDEX)<=  '0';
       
    end procedure;
    
    ---------------------------------------------------------------------
    --
    ---------------------------------------------------------------------
    procedure read_events(
      signal    drv_bus            :inout std_logic_vector(1023 downto 0);
      signal    clk_sys            :in    std_logic;
      variable  exp_event_type     :in    integer;    
      signal    logger_act_dat     :in    std_logic_vector(63 downto 0);
      signal    log_write_pointer  :in    std_logic_vector(7 downto 0);
      signal    log_read_pointer   :in    std_logic_vector(7 downto 0);
      variable  outcome            :out   boolean
    )is
    begin
      outcome:=true;
      wait until falling_edge(clk_sys);
      
      while log_write_pointer /= log_read_pointer loop
        drv_bus(DRV_LOG_CMD_UP_INDEX) <= '1';
        
        if (to_integer(unsigned(logger_act_dat(7 downto 0))) /= exp_event_type) then
          outcome:=false;
        end if;
        
        wait until falling_edge(clk_sys);
      end loop;
      
      drv_bus(DRV_LOG_CMD_UP_INDEX) <= '1';
      wait for 10 ns;
      drv_bus(DRV_LOG_CMD_UP_INDEX) <= '0';
    end procedure;
          
          
begin
    
  CAN_logger_comp:CAN_logger 
  generic map(
     memory_size                =>  16 --Only 2^k possible!
  )
  port map(
     clk_sys                    =>  clk_sys,
     res_n                      =>  res_n,
     drv_bus                    =>  drv_bus,
     stat_bus                   =>  stat_bus,
     sync_edge                  =>  sync_edge,
     data_overrun               =>  data_overrun,
     timestamp                  =>  timestamp,
     bt_FSM                     =>  bt_FSM,
     loger_finished             =>  loger_finished,
     loger_act_data             =>  loger_act_data,
     log_write_pointer          =>  log_write_pointer,
     log_read_pointer           =>  log_read_pointer,
     log_size                   =>  log_size,
     log_state_out              =>  log_state_out
  );
  
  stat_bus    <=   stat_bus_short&
                   std_logic_vector(to_unsigned(protocol_type'pos(PC_State),4)) &
                   std_logic_vector(to_unsigned(oper_mode_type'pos(OP_State),2));
                     
  
  ---------------------------------
  -- Clock generation
  ---------------------------------
  clock_gen:process
  variable period   :natural:=f100_Mhz;
  variable duty     :natural:=50;
  variable epsilon  :natural:=0;
  begin
    generate_clock(period,duty,epsilon,clk_sys);
    timestamp <= std_logic_vector(unsigned(timestamp)+1);
  end process;  
  
  ---------------------------------
  -- Event source generation
  ---------------------------------
  ev_gen:process
  variable wt:time:=0 ns;
  variable rand_val:real:=0.0;
  begin
    wait until falling_edge(clk_sys);
    rand_logic_vect(rand_ctr_2,stat_bus_short,0.1);
        
        
    --Here we emulate frame
    if(PC_State=sof)then
      PC_State  <=  arbitration;
    elsif(PC_State=arbitration)then
      PC_State  <=  control;
    elsif(PC_State=control)then
      PC_State  <=  data;
    elsif(PC_State=data)then
      PC_State  <=  crc;
    elsif(PC_State=crc)then
      PC_State  <=  delim_ack;
    elsif(PC_State=delim_ack)then
      PC_State  <=  eof;
    elsif(PC_State=eof)then
      PC_State  <=  overload;
    elsif(PC_State=overload)then
      PC_State  <=  error;
    elsif(PC_State=error)then
      PC_State  <=  sof;
    end if;
    
    rand_logic(rand_ctr_2,data_overrun,0.2);
    rand_logic(rand_ctr_2,sync_edge,0.2);  
      
    rand_real_v(rand_ctr_2,rand_val);
    wt:= integer(rand_val*100.0) * 1 ns;
    wait for wt;
  end process;
  
  -----------------------------------
  -- Counting the events in software
  -----------------------------------
  --TODO
  
  
  ---------------------------------
  -- Main test process 
  ---------------------------------
  test_proc:process
  variable ev_type    :  integer:=0;
  variable outcome    :  boolean:=false;
  begin
    log("Restarting Event logget unit test!",info_l,log_level);
    wait for 5 ns;
    reset_test(res_n,status,run,error_ctr);
    log("Restarted Event logget unit test",info_l,log_level);
    print_test_info(iterations,log_level,error_beh,error_tol);
    
    while (loop_ctr<iterations  or  exit_imm)
    loop
      log("Starting loop nr "&integer'image(loop_ctr),info_l,log_level);
      
      generate_setting(rand_ctr,drv_bus,ev_type);
      
      wait for 10 ns;
      wait until log_state_out=config;
      wait for 10 ns;
      
      --Recorded type is one higher than index of active source
      ev_type:=ev_type+1;
      
      read_events(drv_bus,clk_sys,ev_type,loger_act_data,log_write_pointer,
                  log_read_pointer,outcome);
                  
      if(outcome=false)then
          log("Recorded event not matching expected value",error_l,log_level);
          process_error(error_ctr,error_beh,exit_imm);
      end if;
      
      loop_ctr<=loop_ctr+1;
    end loop;
    
    evaluate_test(error_tol,error_ctr,status);
  end process;
  
  
  
end architecture;




-----------------------------------------------------------------------------------------------------------------
-- Test wrapper and control signals generator                                           
-----------------------------------------------------------------------------------------------------------------

architecture Event_logger_unit_test_wrapper of CAN_test_wrapper is
  
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
  for test_comp : CAN_test use entity work.CAN_test(Event_logger_unit_test);
  
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