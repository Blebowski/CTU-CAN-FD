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

---------------------------------------------------------------------------------------
-- Purpose:
--  Testbench is testing busSync.vhd circuit. This testbench verifies generic synchroni--
--  sation chain from CAN Physical layer Input and edge detection on incoming data.    --
---------------------------------------------------------------------------------------
-- Revision History:
-- 
--  July 2015  Original version
-------------------------------------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANcomponents.ALL;
USE WORK.CANconstants.ALL;

entity busSync_tb_edge is
end entity;

architecture behav of busSync_tb_edge is

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
    
    signal trv_delay_out: std_logic_vector(15 downto 0);
    
    --Sample point control
    signal sp_control: std_logic_vector(1 downto 0); --Control sequence for sampling:
                    --00:sample_nbt used for sampling (Nominal bit time sampling, Transciever and Reciever)
                    --01:sample_dbt used for sampling (Data bit time sampling, only Reciever)
                    --10:Sampling with transciever delay compensation (Data bit time, transciever)
    signal ssp_reset: std_logic; --Clear the Shift register at the  beginning of Data Phase!!!
    signal trv_delay_calib: std_logic; --Calibration command for transciever delay compenstation (counter)
    
    --Bit Error detection enable (Ex. disabled when recieving data)
    signal bit_err_enable: std_logic; 
    
    ---------------------------
    --Error Handler Interface--
    ---------------------------
    signal bit_Error: std_logic; --Bit Error appeared (monitored value different than transcieved value)   
    
  ------------------------
  --TestBench parameters--
  ------------------------
  constant clk_per:time:=10 ns;
  constant bit_per:time:=160 ns; --Bit time period (6.25 MBit/s)
  constant rec_data_length:integer:=20; --20 Bits reccieved
  constant rec_data:std_logic_vector:= "10101110011100101100"; --Random recieved data
  
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
    sample_sec_del_1_out=>sample_sec_del_1_out,
    trv_delay_out=>trv_delay_out,
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
  
  test_proc:process
  begin
  res_n<=ACT_RESET;
  drv_bus<=(OTHERS=>'0'); --No driving signals
  data_tx<=RECESSIVE; --No data to send
  CAN_rx<=RECESSIVE; --No Incoming data
  sample_nbt<='0';  --Sample signals not needed in this testbench
  sample_dbt<='0';
  sp_control<="00";
  ssp_reset<='0';
  trv_delay_calib<='0';
  bit_err_enable<='0';
  wait for 27 ns;
  res_n<=not ACT_RESET;
  
  --Recieving some random data
  for i in 0 to rec_data_length-1 loop
    CAN_rx<=rec_data(i);
    wait for bit_per;
  end loop;
  
  wait for 300 ns;
  end process;
  
end architecture;