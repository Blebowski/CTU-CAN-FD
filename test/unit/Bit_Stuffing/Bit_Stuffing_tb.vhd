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
--  Unit test for bit stuffing and bit destuffing circuit
--
--  TODO: Detailed describtion        
--                                       
--------------------------------------------------------------------------------
-- Revision History:
--
--    13.6.2016   Created file
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

architecture bit_stuffing_unit_test of CAN_test is

  signal clk_sys              :   std_logic:='0'; --System clock
  signal res_n                :   std_logic:='0'; --Async Reset
  
  --Triggering signals
  signal tx_trig              :   std_logic:='0';
  signal bs_trig              :   std_logic:='0';
  signal bd_trig              :   std_logic:='0';
  signal rx_trig              :   std_logic:='0';
  
  --Datapath
  signal tx_data              :   std_logic:='0';
  signal stuffed_data         :   std_logic:='0';
  signal joined_data          :   std_logic:='0';
  signal err_data             :   std_logic:='0';
  signal rx_data              :   std_logic:='0';
  
  --Control signals
  signal bs_enable            :   std_logic:='0';
  signal bd_enable            :   std_logic:='0';
  signal fixed_stuff          :   std_logic:='0';  --Common for both
  signal bs_length            :   std_logic_vector(2 downto 0):= "100";
  signal bd_length            :   std_logic_vector(2 downto 0):= "100";
  
  --TODO: Testbench not yet debugged with stuff_error enabled!!
  --      Bit destuffing is detecting the stuff errors OK, just TB is not finished with this feature!
  signal stuff_error_enable   :   std_logic:='0';
  
  --Status signals
  signal data_halt            :   std_logic:='0';
  signal destuffed            :   std_logic:='0';
  signal stuff_error          :   std_logic:='0';
  signal bs_ctr               :   natural range 0 to 7:=0;
  signal bd_ctr               :   natural range 0 to 7:=0;
  
  --Other signals
  signal rand_set_ctr         :   natural range 0 to RAND_POOL_SIZE:=0;
  signal rand_tx_ctr          :   natural range 0 to RAND_POOL_SIZE:=0;
  signal rand_st_err_ctr      :   natural range 0 to RAND_POOL_SIZE:=0;
  signal exit_imm_1           :   boolean:=false;
  signal exit_imm_2           :   boolean:=false;
  
  --Error counters
  signal  rx_err_ctr          :   natural:=0;
  signal  stat_err_ctr        :   natural:=0;
  signal  st_er_err_ctr       :   natural:=0;
  
begin
  
   bitStufComp:bitStuffing_v2 
  port map(
     clk_sys            =>  clk_sys, 
     res_n              =>  res_n,
     tran_trig_1        =>  bs_trig,
     enable             =>  bs_enable,
     data_in            =>  tx_data,
     fixed_stuff        =>  fixed_stuff,
     data_halt          =>  data_halt,
     length             =>  bs_length,
     bst_ctr            =>  bs_ctr,
     data_out           =>  stuffed_data
  );
  
  
  bitDestComp:bitDestuffing
  PORT map(
     clk_sys            =>  clk_sys,      
     res_n              =>  res_n,  
     data_in            =>  joined_data,
     trig_spl_1         =>  bd_trig,
     stuff_Error        =>  stuff_error,  
     data_out           =>  rx_data,  
     destuffed          =>  destuffed,
     enable             =>  bd_enable,
     stuff_Error_enable =>  stuff_error_enable, 
     fixed_stuff        =>  fixed_stuff,
     length             =>  bd_length,
     dst_ctr            =>  bd_ctr
  );
  
  
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
  --Trigger generation
  ---------------------------------
  stuff_gen:process
  begin
    wait until falling_edge(clk_sys);
     tx_trig              <= '1';
    wait until falling_edge(clk_sys);
     tx_trig              <= '0'; 
     bs_trig              <= '1';
    wait until falling_edge(clk_sys);  
     bs_trig              <= '0';
     bd_trig              <= '1';
    wait until falling_edge(clk_sys);  
     bd_trig              <= '0';
     rx_trig              <= '1';  
    wait until falling_edge(clk_sys);  
     rx_trig              <= '0';
    wait until falling_edge(clk_sys);
    wait until falling_edge(clk_sys);
  end process; 
  
  
  ---------------------------------
  --Random settings generation
  ---------------------------------
  set_gen:process
  variable rand_val: real;
  begin
    wait until falling_edge(clk_sys);
    
    rand_real_v(rand_set_ctr,rand_val);
    if(tx_trig='0' and bs_trig='0' and bd_trig='0' and rx_trig='0' and err_data='0')then
      if(rand_val>0.95)then
        
        rand_logic(rand_set_ctr,bs_enable,0.9);
        wait for 0 ns;
        bd_enable <= bs_enable;
        
        rand_logic(rand_set_ctr,fixed_stuff,0.25);
        
        rand_logic_vect(rand_set_ctr,bs_length,0.2);
        wait for 0 ns;
        if(bs_length="000" or bs_length="001")then
          bs_length<="010";
        end if;
        wait for 0 ns;
        bd_length <= bs_length; 
             
      end if;
    end if;
  end process;
  
  
  --------------------------------
  -- TX Data generation
  --------------------------------
  tx_data_gen:process
  begin
    wait until rising_edge(tx_trig) and data_halt='0' and stuff_error='0' and err_data='0';
    rand_logic(rand_tx_ctr,tx_data,0.5);
  end process;
  
  --------------------------------
  -- RX Data sampling
  --------------------------------
  rx_data_sam:process
  begin
    wait for 150 ns;
    
    while true loop
      wait until falling_edge(rx_trig) and destuffed='0' and data_halt='0' and err_data='0';
      wait for 10 ns;
      if(tx_data /= rx_data)then
        log("TX Data not matching RX Data",error_l,log_level);
        process_error(rx_err_ctr,error_beh,exit_imm_1);
      end if;
    end loop;
    
  end process;
  
  ----------------------------------
  -- Testing Stuff error detection
  ----------------------------------
  st_err_det_proc:process
  variable rand_val: real;
  variable wt      : time:= 0 ns;
  variable int_tm  : integer:=0;
  begin
    wait for 150 ns;
    
    while true loop
      wait until rising_edge(clk_sys) and (tx_trig='0' and bs_trig='0' and bd_trig='0' and rx_trig='0') and bs_enable='1' and stuff_error_enable='1';
      
      rand_real_v(rand_st_err_ctr,rand_val);
      if(rand_val>0.98)then
        err_data  <= '1';
        --int_tm    := 70*(to_integer(unsigned(bs_length))+1);
        --wt        := int_tm * 10 ns;
        wait until stuff_error='1';
        wait for 10 ns;
        
        if(stuff_error='0')then
          log("Stuff error not fired as expected!",error_l,log_level);
          process_error(st_er_err_ctr,error_beh,exit_imm_2);
        end if;
        
        err_data<='0';
      end if;
    end loop;
    
  end process;
    
  error_ctr     <=  rx_err_ctr + stat_err_ctr + st_er_err_ctr;
  joined_data   <=  stuffed_data or err_data;
  
  ---------------------------------
  ---------------------------------
  --Main Test process
  ---------------------------------
  ---------------------------------
  test_proc:process
  begin
    log("Restarting Bit stuffing-destuffing test!",info_l,log_level);
    wait for 5 ns;
    reset_test(res_n,status,run,stat_err_ctr);
    log("Restarted Bit stuffing-destuffing test",info_l,log_level);
    print_test_info(iterations,log_level,error_beh,error_tol);
    
    -------------------------------
    --Main loop of the test
    -------------------------------
    log("Starting Bit stuffing-destuffing main loop",info_l,log_level);
    
    while (loop_ctr<iterations  or  exit_imm)
    loop
      log("Starting loop nr "&integer'image(loop_ctr),info_l,log_level);
      
      wait until (tx_trig='0' and bs_trig='0' and bd_trig='0' and rx_trig='0' and err_data='0');
      
      if (data_halt /= destuffed or
          bd_ctr    /= bs_ctr  )then
        log("Status signals are mismatching",error_l,log_level);
        process_error(stat_err_ctr,error_beh,exit_imm);
      end if;  
      
      loop_ctr<=loop_ctr+1;
    end loop;
    
    evaluate_test(error_tol,error_ctr,status);
  end process;
    
end architecture;


-----------------------------------------------------------------------------------------------------------------
-- Test wrapper and control signals generator                                           
-----------------------------------------------------------------------------------------------------------------
architecture bit_stuffing_unit_test_wrapper of CAN_test_wrapper is
  
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
  for test_comp : CAN_test use entity work.CAN_test(bit_stuffing_unit_test);
  
    signal run             :    boolean;                -- Input trigger, test starts running when true                                                        -- exceed this value in order for the test to pass
    signal status_int      :    test_status_type;       -- Status of the test
    signal errors          :    natural;                -- Amount of errors which appeared in the test

begin
  
  --In this test wrapper generics are directly connected to the signals
  -- of test entity
  test_comp:CAN_test
  port map(
     run              =>  run,
     iterations       =>  iterations , 
   --iterations       =>  100000 ,
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