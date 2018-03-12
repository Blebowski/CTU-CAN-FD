--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018 Ondrej Ille <ondrej.ille@gmail.com>
-- 
-- Project advisors and co-authors: 
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 	Martin Jerabek <jerabma7@fel.cvut.cz>
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
--  Unit test for the CRC circuit.
--  Unit test generates bit sequence 10 to 620 bits long as well as random drv_fd_type setting. Then it precal-
--  culates the CRC values by calc_crc procedure. Afterwads it emulates reception of this sequence on input of
--  CRC circuit. Sync signal is  used for enable transition. Simple rx trigger signal is used as trigger signal
--  for processing input data. After the calculation, software precalculated CRC and CRC from DUT are compared!
--  If mismatch occurs error is detected and test fails. 
--  Whole procedure is repeated "iterations" times.                          
--------------------------------------------------------------------------------
-- Revision History:
--
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
use work.CAN_FD_register_map.all;

use work.ID_transfer.all;

architecture CRC_unit_test of CAN_test is
  
    signal data_in     :   std_logic:='0'; --Serial data input
    signal clk_sys     :   std_logic:='0'; --System clock input
    signal trig        :   std_logic:='0'; --Trigger to sample the input value
    signal res_n       :   std_logic:='0'; --Asynchronous reset
    signal enable      :   std_logic:='0'; 
    signal drv_bus     :   std_logic_vector(1023 downto 0):=(OTHERS => '0');
    
    signal crc15       :   std_logic_vector(14 downto 0):=(OTHERS => '0');
    signal crc17       :   std_logic_vector(16 downto 0):=(OTHERS => '0');
    signal crc21       :   std_logic_vector(20 downto 0):=(OTHERS => '0');
  
    signal tx_trig     :   std_logic:='0'; --TX trigger for crc, corresponds to trigger
                                          -- signal delayed by one from Protocol control trigger
    signal rx_trig     :   std_logic:='0'; 
    signal rnd_ctr_tr  :   natural range 0 to RAND_POOL_SIZE:=0;                           
    signal drv_fd_type :   std_logic:='0';
  
  ---------------------------------------------
  --Generate bit sequence for CRC calculation
  ---------------------------------------------
  procedure gen_bit_sequence(
    signal   rand_ctr    :inout   natural range 0 to RAND_POOL_SIZE;
    constant min_length  :in      natural range 2  to 10;
    constant max_length  :in      natural range 11 to 640;
    variable bit_seq     :out     std_logic_vector(660 downto 0);
    variable act_length  :inout   natural
  )is
  variable rand_nr       :real;
  begin
    rand_real_v(rand_ctr,rand_nr);
    act_length:= min_length+integer( real(max_length-min_length)*rand_nr);
    rand_logic_vect_v(rand_ctr,bit_seq,0.5);
    
    --From length until the end of vector we should have zeroes
    --Here we left enough space at the end of the vector for 
    --right alignment by zeroes from right!
    for i in 0 to 660-act_length loop
      bit_seq(i):='0';
    end loop;
    
  end procedure;
  
  ---------------------------------------------
  --Calculate the CRC in behavioral way
  ---------------------------------------------
  procedure calc_crc(
    variable bit_seq    :in     std_logic_vector(660 downto 0);
    variable act_length :in     natural;
    constant crc_15_pol :in     std_logic_vector(15 downto 0);
    constant crc_17_pol :in     std_logic_vector(19 downto 0);
    constant crc_21_pol :in     std_logic_vector(23 downto 0);
    signal   drv_fd_type:in     std_logic;
    variable crc_15_res :inout  std_logic_vector(14 downto 0);
    variable crc_17_res :inout  std_logic_vector(16 downto 0);
    variable crc_21_res :inout  std_logic_vector(20 downto 0)
  )is
  variable crc_15_next  :       std_logic;
  variable crc_17_next  :       std_logic;
  variable crc_21_next  :       std_logic;
  variable iterator     :       natural;
  begin
    
    --Set up the initial polynomials
    crc_15_res := (OTHERS => '0');
    crc_17_res := (OTHERS => '0');
    crc_21_res := (OTHERS => '0');
    if(drv_fd_type = ISO_FD)then
       crc_17_res(16) := '1';
       crc_21_res(20) := '1';
    end if;
    
       
    --Calculate CRC in behavioral way
    --use the same algorithm as in the actual circuit!
    --I was too lazy to debug the algoryhtm with polynomial
    --division.
    --FOllowing algorythm is the same as in CAN FD spec.
    
    iterator:=0;    
    while iterator<act_length loop
      
      --Check whether we go XORing
      crc_15_next:= crc_15_res(14) xor bit_seq(660-iterator);
      crc_17_next:= crc_17_res(16) xor bit_seq(660-iterator);
      crc_21_next:= crc_21_res(20) xor bit_seq(660-iterator);
      
      --Shift left by one
      crc_15_res:= crc_15_res(13 downto 0)&'0';
      crc_17_res:= crc_17_res(15 downto 0)&'0';
      crc_21_res:= crc_21_res(19 downto 0)&'0';
      
      --Now do the XOR if we have to
      if(crc_15_next='1')then
        crc_15_res := crc_15_res xor crc_15_pol(14 downto 0);
      end if;
      
      if(crc_17_next='1')then
        crc_17_res := crc_17_res xor crc_17_pol(16 downto 0);
      end if;
      
      if(crc_21_next='1')then
        crc_21_res := crc_21_res xor crc_21_pol(20 downto 0);
      end if;
      
      iterator:=iterator+1;
    end loop;
    
  end procedure;
  
  ---------------------------------------------
  --Puts the CRC on the input of the  
  ---------------------------------------------
  procedure put_to_dut(
    signal    sync         :in     std_logic;
    signal    sample       :in     std_logic;
    signal    enable       :out    std_logic;
    signal    data_in      :out    std_logic;
    signal    clk_sys      :in     std_logic;
    variable  bit_seq      :in     std_logic_vector(660 downto 0);
    variable  act_length   :in     natural
  )is
  variable iterator: natural :=0;
  begin
    wait until rising_edge(sync);
    enable <= '1';
    wait until falling_edge(clk_sys);
    
    
    iterator:=0;
    while iterator<act_length loop  
      log("Putting bit nr "&integer'image(iterator),info_l,log_level);
      data_in <= bit_seq(660-iterator);
      wait until rising_edge(sample);
      iterator:=iterator+1;
      wait until rising_edge(sync);
    end loop;
    
    wait until falling_edge(clk_sys);
    enable <= '0';
    
  end procedure;
    
  ---------------------------------------------
  --Compare results
  ---------------------------------------------
  procedure compare_results(
    signal    crc_15_dut  :in     std_logic_vector(14 downto 0);
    signal    crc_17_dut  :in     std_logic_vector(16 downto 0);
    signal    crc_21_dut  :in     std_logic_vector(20 downto 0);
    variable  crc_15_mod  :in     std_logic_vector(14 downto 0);
    variable  crc_17_mod  :in     std_logic_vector(16 downto 0);
    variable  crc_21_mod  :in     std_logic_vector(20 downto 0); 
    variable  c15_mism    :out    boolean;
    variable  c17_mism    :out    boolean;
    variable  c21_mism    :out    boolean
  )is
  begin
    
    if(crc_15_dut=crc_15_mod)then
      c15_mism:=false;
    else
      c15_mism:=true;
    end if;
    
    if(crc_17_dut=crc_17_mod)then
      c17_mism:=false;
    else
      c17_mism:=true;
    end if;
    
    if(crc_21_dut=crc_21_mod)then
      c21_mism:=false;
    else
      c21_mism:=true;
    end if;
    
  end procedure;
    
begin
  
  CRC_comp:canCRC
  generic map(
     crc15_pol              =>  CRC15_POL,
     crc17_pol              =>  CRC17_POL,
     crc21_pol              =>  CRC21_POL
  )
  port map(
     data_in    =>  data_in,
     clk_sys    =>  clk_sys,
     trig       =>  trig,
     res_n      =>  res_n,
     enable     =>  enable,
     drv_bus    =>  drv_bus,
    
     crc15      =>  crc15, 
     crc17      =>  crc17,
     crc21      =>  crc21
  );
  
  
  drv_bus(DRV_FD_TYPE_INDEX) <= drv_fd_type;
  
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
  -- Sampling signals generation
  ---------------------------------
  sample_gen:process
  variable min_diff: natural :=0;
  begin
    generate_simple_trig(rnd_ctr_tr,tx_trig,rx_trig,clk_sys,min_diff);    
  end process;
  
  --Propagate the trigger to the CRC circuit...
  trig <= rx_trig;
  
  ---------------------------------
  -- Main test process 
  ---------------------------------
  test_proc:process
  variable bit_seq    :std_logic_vector(660 downto 0);
  variable gen_length :natural;
  variable crc_15_mod :std_logic_vector(14 downto 0):=(OTHERS => '0');
  variable crc_17_mod :std_logic_vector(16 downto 0):=(OTHERS => '0');
  variable crc_21_mod :std_logic_vector(20 downto 0):=(OTHERS => '0');
  variable c15_mism   :boolean:=false;
  variable c17_mism   :boolean:=false;
  variable c21_mism   :boolean:=false;
  begin
    log("Restarting CRC test!",info_l,log_level);
    wait for 5 ns;
    reset_test(res_n,status,run,error_ctr);
    log("Restarted CRC test",info_l,log_level);
    print_test_info(iterations,log_level,error_beh,error_tol);
    
    while (loop_ctr<iterations  or  exit_imm)
    loop
      log("Starting loop nr "&integer'image(loop_ctr),info_l,log_level);
      
      --Generate random ISO, non ISO
      rand_logic(rand_ctr,drv_fd_type,0.5);
      
      --Generate bit sequence
      log("Generating random bit sequence",info_l,log_level);
      gen_bit_sequence(rand_ctr,10,620,bit_seq,gen_length);
            
      log("Calculating software CRC",info_l,log_level);
      calc_crc(bit_seq,gen_length,CRC15_POL,CRC17_POL,CRC21_POL,
               drv_fd_type,crc_15_mod,crc_17_mod,crc_21_mod);
      
      log("Putting bit sequence to DUT",info_l,log_level);
      put_to_dut(tx_trig,rx_trig,enable,data_in,clk_sys,bit_seq,gen_length);
      
      log("Comparing SW CRC and DUT output",info_l,log_level);
      compare_results(crc15,crc17,crc21,crc_15_mod,crc_17_mod,crc_21_mod,
                      c15_mism,c17_mism,c21_mism);
        
      if(c15_mism=true)then
        process_error(error_ctr,error_beh,exit_imm);
        log("Mismatch in CRC15",error_l,log_level);
      end if;
      
      if(c17_mism=true)then
        process_error(error_ctr,error_beh,exit_imm);
        log("Mismatch in CRC17",error_l,log_level);
      end if;
     
      if(c21_mism=true)then
        process_error(error_ctr,error_beh,exit_imm);
        log("Mismatch in CRC21",error_l,log_level);
      end if;
          
      loop_ctr<=loop_ctr+1;
    end loop;
    
    evaluate_test(error_tol,error_ctr,status);
  end process;
  
  
end architecture;



-----------------------------------------------------------------------------------------------------------------
-- Test wrapper and control signals generator                                           
-----------------------------------------------------------------------------------------------------------------

architecture CRC_unit_test_wrapper of CAN_test_wrapper is
  
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
  for test_comp : CAN_test use entity work.CAN_test(CRC_unit_test);
  
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