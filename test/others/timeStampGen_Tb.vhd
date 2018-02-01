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
-- Revision History:
--
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;

entity timeStampGen_Tb is
end entity;

architecture behavioral of timeStampGen_Tb is
    ----------
    --INPUTS--
    ----------
    --Source clock signals
    signal clk_sys: std_logic; --System clock
    signal clk_tq_nbt : std_logic; --NBT time quantum clock
    signal clk_tq_dbt : std_logic; --DBT time qunatum clock
    signal clk_nbt : std_logic; --NBT Bit time clock
    signal clk_dbt : std_logic; --DBT Bit time clock
    
    signal res_n: std_logic; --Async reset
    
    --Driving bus from user registers
    signal drv_bus: std_logic_vector(1023 downto 0);
    
    ----------
    --OUTPUTS-
    ----------
    --TimeStamp counter values
    signal ts_1: std_logic_vector(49 downto 0);
    signal ts_2: std_logic_vector(49 downto 0);
    --TimeStamp registers output
    signal ts_reg: std_logic_vector(199 downto 0);   
  
component timeStampGen is
  port(
    ----------
    --INPUTS--
    ----------
    --Source clock signals
    signal clk_sys: in std_logic; --System clock
    signal clk_tq_nbt :in std_logic; --NBT time quantum clock
    signal clk_tq_dbt :in std_logic; --DBT time qunatum clock
    signal clk_nbt :in std_logic; --NBT Bit time clock
    signal clk_dbt :in std_logic; --DBT Bit time clock
    
    signal res_n:in std_logic; --Async reset
    
    --Driving bus from user registers
    signal drv_bus:in std_logic_vector(1023 downto 0);
    
    ----------
    --OUTPUTS-
    ----------
    --TimeStamp counter values
    signal ts_1:out std_logic_vector(49 downto 0);
    signal ts_2:out std_logic_vector(49 downto 0);
    --TimeStamp registers output
    signal ts_reg:out std_logic_vector(199 downto 0)  
  );
end component;  
begin
  timeStampGen_tb:timeStampGen
  port map(
    clk_sys=>clk_sys,
    clk_tq_nbt=>clk_tq_nbt,
    clk_tq_dbt=>clk_tq_dbt,
    clk_nbt=>clk_nbt,
    clk_dbt=>clk_dbt,
    res_n=>res_n,
    drv_bus=>drv_bus,
    ts_1=>ts_1,
    ts_2=>ts_2,
    ts_reg=>ts_reg
  );
 
clock_gen:process
constant clk_per:time:=10 ns;
begin
  clk_sys<='1';
  wait for clk_per/2;
  clk_sys<='0';
  wait for clk_per/2;
end process clock_gen;

clock_gen2:process
constant clk_per:time:=20 ns;
begin
  clk_tq_nbt<='1';
  clk_tq_dbt<='1';
  wait for clk_per/2;
  clk_tq_nbt<='0';
  clk_tq_dbt<='1';
  wait for clk_per/2;
end process clock_gen2;   

clock_gen3:process
constant clk_per:time:=100 ns;
begin
  clk_nbt<='1';
  clk_dbt<='1';
  wait for clk_per/2;
  clk_nbt<='0';
  clk_dbt<='1';
  wait for clk_per/2;
end process clock_gen3;   

counter1:process
begin
  drv_bus<=(OTHERS=>'0');
  res_n<='0';
  wait for 5 ns;
  res_n<='1';
  
  drv_bus(63 downto 61)<="011"; --clk_tq_nbt as source signal for first counter
  drv_bus(64)<='1'; --Reset the counter after setting the source signal
  wait for 10 ns; 
  drv_bus(64)<='0'; --Clear the reset
  
  wait for 200 ns;
  drv_bus(68 downto 65)<="0011"; --Save the value into A and B registers
  wait for 10 ns;
  drv_bus(68 downto 65)<="0000"; --Clear the saving flag
  
  drv_bus(64)<='1'; --Reset the counter after setting the source signal
  wait for 10 ns; 
  drv_bus(64)<='0'; --Clear the reset
  
  wait for 200 ns;
  
end process counter1;
  
    
  
end architecture;
