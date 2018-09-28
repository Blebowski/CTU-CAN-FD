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

-----------------------------------------------------------------------------------------
-- Purpose:
--  TestBench simulating sycnhronisation and sampling while transcieving in Data Bit time--
--  Data bits (EDL,r0,BRS) with nominal bit time are send and recieed with delay. Edges ---
--  are detected for synchronisation. Transciever delay compensation is measured and ------
--  when bit time is switched, secondary sample point is generated from sample dbt signal.-
--  Then secondary sample point is used for sampling the data and comparing with delayed --
--  TX Data. Bit error is signalised for both : first 3 bits (sampling with sample_nbt) ---
--  and for rest of the bits in data phase (sampling with secondary sample point)       ---
-----------------------------------------------------------------------------------------
-- Revision History:
--
--  July 2015  Original version
-------------------------------------------------------------------------------------------------------------


Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE WORK.CANconstants.ALL;
use work.CANcomponents.ALL;


entity busSync_tb_sample is
end entity;

architecture behav of busSync_tb_sample is

  -----------------
  --Signal aliases-
  -----------------    
    -------------------------
    --Clock and Async reset--
    -------------------------
    signal clk_sys: std_logic; --System clock
    signal res_n: std_logic; --Async Reset
    
    ---------------------------
    --Physical layer interface-
    ---------------------------
    signal CAN_rx: std_logic; --CAN data input from transciever
    signal CAN_tx: std_logic; --CAN data output to transciever  
    
    -------------------------------
    --Driving registers interface--
    -------------------------------
    signal drv_bus: std_logic_vector(1023 downto 0); --Driving bus
      
    -----------------------
    --Prescaler interface--
    ----------------------- 
    signal sample_nbt: std_logic; --Sample command for nominal bit time
    signal sample_dbt: std_logic; --Sample command for data bit time    
    signal sync_edge: std_logic; --Synchronisation edge appeared
    
    --Secondary sample signal outputs
    signal sample_sec_out: std_logic; --Secondary sample signal 
    signal sample_sec_del_1_out: std_logic; --Bit destuffing trigger for secondary sample point
    signal sample_sec_del_2_out: std_logic; --Rec trig for secondary sample point
        
    ----------------------
    --CAN Core Interface--
    ----------------------
    signal data_tx: std_logic; --Transcieve data value
    signal data_rx: std_logic; --Recieved data value
    
    --Sample point control
    signal sp_control: std_logic_vector(1 downto 0); --Control sequence for sampling:
                    --00:sample_nbt used for sampling (Nominal bit time sampling, Transciever and Reciever)
                    --01:sample_dbt used for sampling (Data bit time sampling, only Reciever)
                    --10:Sampling with transciever delay compensation (Data bit time, transciever)
    signal ssp_reset: std_logic; --Clear the Shift register at the  beginning of Data Phase!!!
    signal trv_delay_calib: std_logic; --Calibration command for transciever delay compenstation (counter)
    
    --Bit Error detection enable (Ex. disabled when recieving data)
    signal bit_err_enable: std_logic; 
    
    signal trv_delay_out: std_logic_vector(15 downto 0);
    ---------------------------
    --Error Handler Interface--
    ---------------------------
    signal bit_Error: std_logic; --Bit Error appeared (monitored value different than transcieved value)   
    
  ------------------------
  --TestBench parameters--
  ------------------------
  constant clk_per:time:=10 ns;
  constant bit_per_dbt:time:=200 ns; --Bit time period (5 MBit/s) for data phase
  constant bit_per_nbt:time:=1000 ns; --Bit time period nominal bit time arbitration phase
  
  constant ph1_nbt:time:=840 ns; --Phase 1 of the bit time
  constant ph2_nbt:time:=150 ns; --Phase 2 of bit time
  constant ph1_dbt:time:=120 ns; --Phase 1 of the bit time
  constant ph2_dbt:time:=70 ns; --Phase 2 of bit time
  
  constant rec_data_length:integer:=20; --20 Bits reccieved
  constant rec_data:std_logic_vector:= "01001110011100101100"; --Random recieved data for data phase
  constant tran_delay:time:=250 ns; --Delay of TJA1041
  
  
begin
  busSynctb:busSync
  generic map(
    use_Sync => true
  )
  port map(
    clk_sys=>clk_sys,
    res_n=>res_n,
    CAN_rx=>CAN_rx,
    CAN_tx=>CAN_tx,
    drv_bus=>drv_bus,
    sample_nbt=>sample_nbt,
    sample_dbt=>sample_dbt,
    sync_edge=>sync_edge,
    data_tx=>data_tx,
    data_rx=>data_rx,
    sp_control=>sp_control,
    ssp_reset=>ssp_reset,
    trv_delay_calib=>trv_delay_calib,
    bit_err_enable=>bit_err_enable,
    bit_Error=>bit_Error,
    sample_sec_out=>sample_sec_out,
    trv_delay_out=>trv_delay_out,
    sample_sec_del_1_out=>sample_sec_del_1_out,
    sample_sec_del_2_out=>sample_sec_del_2_out  
  );

  --Clock generation process
  clock_gen:process
  begin
    clk_sys<='1';
    wait for clk_per/2;
    clk_sys<='0';
    wait for clk_per/2;   
  end process;
  
  --Generating sample signal (for NBT)
  sample_nbt_gen:process
  begin
    sample_nbt<='0';
    wait for ph1_nbt;
    sample_nbt<='1';
    wait for clk_per;
    sample_nbt<='0';
    wait for ph2_nbt;
  end process sample_nbt_gen;
  
  --Generating sample signal (for DBT)
  sample_dbt_gen:process
  begin
    sample_dbt<='0';
    wait for ph1_dbt;
    sample_dbt<='1';
    wait for clk_per;
    sample_dbt<='0';
    wait for ph2_dbt;
  end process sample_dbt_gen;
  
  --Generating transcieved data
  tx_data_gen:process
  begin
    --Idle or Intermission
    data_tx<='1';
    wait for 27 ns;
    
    --Generating data in NBT Phase
    data_tx<='1'; --EDL bit
    wait for bit_per_nbt;
    data_tx<='0'; --r0 bit , Tr_delay measurment runs here
    wait for bit_per_nbt;
    data_tx<='1'; --BRS Bit
    wait for bit_per_nbt; --Only PH1 should be waiting! , in BRS bit bit rate is shifted directly after Sampling point
    
    --Transcieving data in DBT Phase
    for i in 0 to rec_data_length-1 loop
      data_tx<=rec_data(i);
      wait for bit_per_dbt;
    end loop;
    
    data_tx<='1';
    wait for 10000 ns;
  end process;
  
  
  --Generating input data sequence
  data_gen:process
  begin
  --First no data are recieved (ex. intermission field)
  CAN_rx<='1';
  wait for 27 ns+tran_delay;
  
  --Recieving NBT bits
  CAN_rx<='1'; --EDL bit
  wait for bit_per_nbt;
  CAN_rx<='0'; --r0 bit , Tr_delay measurment runs here
  wait for bit_per_nbt;
  CAN_rx<='1';  --BRS Bit
  wait for bit_per_nbt; --Only PH1 should be waiting, in BRS bit bit rate is shifted directly after Sampling point
  
  --Recieving the same data as transcived with delay (data bit time)
    for i in 0 to rec_data_length-1 loop
      CAN_rx<=rec_data(i);
      wait for bit_per_dbt;
    end loop;    
    
  CAN_rx<='1';
  wait for 10000 ns ;

  end process;
  
  --Test of the functionality
  test_proc:process
  begin
  res_n<=ACT_RESET;
  ssp_reset<='1';  --Reseting secondary sample point shift registers
  drv_bus<=(OTHERS=>'0'); --No driving signals
  sp_control<="00";
  trv_delay_calib<='0';
  bit_err_enable<='1';
  wait for 27 ns;
  res_n<=not ACT_RESET;
  
  --Start of the transciever delay calibration
  trv_delay_calib<='1';
  wait for 2*bit_per_nbt;
  trv_delay_calib<='0';
  
  wait for bit_per_nbt; --Waiting trough BRS bit

  --Releasing SSP reset. Has to be released here!!! Otherwise values from NBT will be stored into shift register!!!
  ssp_reset<='0';  
  sp_control<="10";  
  
  wait for 10000 ns;
end process;
  
  
end architecture;