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

--------------------------------------------------------------------------------
--
-- CAN with Flexible Data-Rate IP Core 
--
-- Copyright (C) 2015 Ondrej Ille <ondrej.ille@gmail.com>
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy 
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is 
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in 
-- all copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
-- AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
-- FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
-- IN THE SOFTWARE.
--
-- The CAN protocol is developed by Robert Bosch GmbH and protected by patents. 
-- Anybody who wants to implement this IP core on silicon has to obtain a CAN 
-- protocol license from Bosch.
--
-- Revision History:
--
--    14.6.2016   Created file
-----------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Unit test for the TX Buffer circuit                                                 
-----------------------------------------------------------------------------------------------------------------


-----------------------------------------------------------------------------------------------------------------
-- Test implementation                                            
-----------------------------------------------------------------------------------------------------------------

architecture tx_buf_unit_test of CAN_test is
  
      --Common signals
      signal clk_sys            :   std_logic:='0';
      signal res_n              :   std_logic:='0';                        --Async reset
      signal drv_bus            :   std_logic_vector(1023 downto 0):=(OTHERS => '0');  --Driving bus
      signal tran_data_in       :   std_logic_vector(639 downto 0):=(OTHERS => '0');   --Input data frame
      
      --Buffer 1 
      signal txt_empty_1        :   std_logic:='0';                       --Logic 1 signals empty TxTime buffer
      signal txt_disc_1         :   std_logic:='0';                       --Info that  message was stored into buffer
      signal txt_buffer_out_1   :   std_logic_vector(639 downto 0):=(OTHERS => '0');  --Output value of message in the buffer  
      signal txt_data_ack_1     :   std_logic:='0';  
      
      --Buffer 2 
      signal txt_empty_2        :   std_logic:='0';                       --Logic 1 signals empty TxTime buffer
      signal txt_disc_2         :   std_logic:='0';                       --Info that  message was stored into buffer
      signal txt_buffer_out_2   :   std_logic_vector(639 downto 0):=(OTHERS => '0');  --Output value of message in the buffer  
      signal txt_data_ack_2     :   std_logic:='0';            
      
      --Driving bus aliases
      signal drv_erase_txt_1    :   std_logic:='0';                           --Command for erasing time transcieve buffer
      signal drv_store_txt_1    :   std_logic:='0';                           --Command for storing data from tran_data_in into txt_buffer
      
      signal drv_erase_txt_2    :   std_logic:='0';                           --Command for erasing time transcieve buffer
      signal drv_store_txt_2    :   std_logic:='0';                           --Command for storing data from tran_data_in into txt_buffer
      
      --Internal test signals
      signal small_mem          :   std_logic_vector(191 downto 0):=(OTHERS => '0');
      signal big_mem            :   std_logic_vector(639 downto 0):=(OTHERS => '0');
      signal rand_ctr_2         :   natural range 0 to RAND_POOL_SIZE:=0;
begin
   
   --Driving bus aliases
   drv_bus(DRV_ERASE_TXT1_INDEX) <=  drv_erase_txt_1;
   drv_bus(DRV_ERASE_TXT2_INDEX) <=  drv_erase_txt_1;
   
   drv_bus(DRV_STORE_TXT1_INDEX) <=  drv_store_txt_1;
   drv_bus(DRV_STORE_TXT2_INDEX) <=  drv_store_txt_2;
  
  --------------------------------------------
  -- Buffer components
  --------------------------------------------
  
  txt_Buf_comp_1:txtBuffer 
    generic map(
       ID             => 1,
       useFDsize      => true
    )
    PORT map(
       clk_sys        =>  clk_sys,
       res_n          =>  res_n,
       drv_bus        =>  drv_bus,  
       tran_data_in   =>  tran_data_in,
       txt_empty      =>  txt_empty_1,
       txt_disc       =>  txt_disc_1,
       txt_buffer_out =>  txt_buffer_out_1,
       txt_data_ack   =>  txt_data_ack_1
      );
  
  txt_Buf_comp_2:txtBuffer 
    generic map(
       ID             => 2,
       useFDsize      => false
    )
    PORT map(
       clk_sys        =>  clk_sys,
       res_n          =>  res_n,
       drv_bus        =>  drv_bus,  
       tran_data_in   =>  tran_data_in,
       txt_empty      =>  txt_empty_2,
       txt_disc       =>  txt_disc_2,
       txt_buffer_out =>  txt_buffer_out_2,
       txt_data_ack   =>  txt_data_ack_2
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
    
    --------------------------------------------
    -- Data generation
    -------------------------------------------- 
    data_gen_proc:process
    variable rand_nr :real;
    variable rand_time: time;
    begin
      wait until falling_edge(clk_sys) and res_n='1';
      
      if(txt_empty_2='1' or txt_empty_1='1')then
        rand_logic_vect(rand_ctr,tran_data_in,0.5);
        wait for 0 ns;
        
        if(txt_empty_1='1')then
          drv_store_txt_1 <=  '1';
          big_mem         <=  tran_data_in;
        elsif(txt_empty_2='1')then
          drv_store_txt_2 <=  '1';
          small_mem       <=  tran_data_in(639 downto 448);
        end if;
        
        wait for 10 ns;
        drv_store_txt_1   <=  '0';
        drv_store_txt_2   <=  '0';
        
        rand_real_v(rand_ctr,rand_nr);
        rand_nr:= rand_nr*2.0;
        rand_time := (integer(rand_nr) + 10) * 1 ns;
        wait for rand_time;
      end if;
      
    end process;
  
      
    ---------------------------------
    ---------------------------------
    --Main Test process
    ---------------------------------
    ---------------------------------
    test_proc:process
    variable rand_nr :real;
    variable rand_time: time;
    begin
      log("Restarting TXT Buffer test!",info_l,log_level);
      wait for 5 ns;
      reset_test(res_n,status,run,error_ctr);
      log("Restarted TXT Buffer test",info_l,log_level);
      print_test_info(iterations,log_level,error_beh,error_tol);
      
      -------------------------------
      --Main loop of the test
      -------------------------------
      log("Starting TXT Buffer main loop",info_l,log_level);
      
      while (loop_ctr<iterations  or  exit_imm)
      loop
        log("Starting loop nr "&integer'image(loop_ctr),info_l,log_level);
        
        wait until falling_edge(clk_sys);
      
        if(txt_empty_2='0' or txt_empty_1='0')then
          
          --Reading from Buffer 1
          if(txt_empty_1='0')then
            txt_data_ack_1  <=  '1';
            if(txt_buffer_out_1 /= big_mem)then
              log("TXT Buffer with FD support data mismatch",error_l,log_level);
              process_error(error_ctr,error_beh,exit_imm);  
            end if;
          end if;
          
          --Reading from Buffer 2
          if(txt_empty_2='0')then
            txt_data_ack_2  <=  '1';
            if(txt_buffer_out_2(639 downto 448) /= small_mem)then
              log("TXT Buffer without FD support data mismatch",error_l,log_level);
              process_error(error_ctr,error_beh,exit_imm);  
            end if;
          end if;
          
          wait for 10 ns;
          txt_data_ack_2<='0';
          txt_data_ack_1<='0';
          rand_real_v(rand_ctr_2,rand_nr);
          rand_nr:= rand_nr*3.0;
          rand_time := (integer(rand_nr) + 10) * 1 ns;
          wait for rand_time;
          
        end if;
        
        loop_ctr<=loop_ctr+1;
      end loop;
      
      evaluate_test(error_tol,error_ctr,status);
    end process;
      
end architecture;




-----------------------------------------------------------------------------------------------------------------
-- Test wrapper and control signals generator                                           
-----------------------------------------------------------------------------------------------------------------
architecture tx_buf_unit_test_wrapper of CAN_test_wrapper is
  
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
  for test_comp : CAN_test use entity work.CAN_test(tx_buf_unit_test);
  
    signal run              :   boolean;                -- Input trigger, test starts running when true                                                        -- exceed this value in order for the test to pass
    signal status_int       :   test_status_type;      -- Status of the test
    signal errors           :   natural;                -- Amount of errors which appeared in the test

begin
  
  --In this test wrapper generics are directly connected to the signals
  -- of test entity
  test_comp:CAN_test
  port map(
     run              =>  run,
     --iterations       =>  10000,
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