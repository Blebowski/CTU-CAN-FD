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
-- Revision History:
--
--------------------------------------------------------------------------------


Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;

entity bitStuffing_tb is
end entity;

architecture behav of bitStuffing_tb is
  constant length:natural:=5;
  constant maxDataSize:natural:=512;
  
  --Inputs
  signal clk_sys : std_logic; --System clock
  signal clk_nbt : std_logic; --Nominal bit time clock
  signal clk_dbt : std_logic; --Data bit time clock
  signal start:std_logic;
  signal data_size:std_logic_vector(11 Downto 0);
  signal data_in:std_logic;
  signal res_n:std_logic;
  signal fixed_stuff:std_logic;
  signal stop:std_logic;
  signal bt_type : std_logic; --Bit time type, if either clk_nbt or clk_dbt is considered as bit time for transcieving
  signal bs_trig_nbt : std_logic; --Trigger signal from prescaler to sample inpu value and propagate or bit stuff
  signal bs_trig_dbt : std_logic; --Trigger signal from prescaler to sample inpu value and propagate or bit stuff 
   
  --Outputs
  signal data_out:std_logic;
  signal finished:std_logic;
  signal data_rem:std_logic_vector (11 downto 0);
  signal valid:std_logic;
  signal error:std_logic;
  
  --Test outputs
  signal data_mismatch:std_logic;
  signal test_data:std_logic;
  
  component bitStuffer is
  --number of equal bits before sutff bit is inserted
  generic (
    length:natural:=5;
    maxDataSize:natural:=512
  );
  port(
    --INPUTS
    signal data_in : in std_logic;  --serial data input 
    signal clk_sys : in std_logic; --System clock
    signal clk_nbt : in std_logic; --Nominal bit time clock
    signal clk_dbt : in std_logic; --Data bit time clock
    signal start : in std_logic; --Logic '1' starts bit stuffing
    signal data_size : in std_logic_vector (11 downto 0); --must be valid data size to be transmitted when start='1'
    signal fixed_stuff : in std_logic; --fixed stuff insert stuffed bit after 'length' bits even if they are not the same value
    signal stop : in std_logic; --stops the bit stuffing when 1 is present
    signal res_n : in std_logic; --reset (active in '0')
    signal bt_type : in std_logic; --Bit time type, if either clk_nbt or clk_dbt is considered as bit time for transcieving
    signal bs_trig_nbt : in std_logic; --Trigger signal from prescaler to sample inpu value and propagate or bit stuff
    signal bs_trig_dbt : in std_logic; --Trigger signal from prescaler to sample inpu value and propagate or bit stuff
   
    --OUTPUTS
    signal data_out : out std_logic; --serial data output
    signal finished : out std_logic; --signalizing all data are stuffed
    signal valid : out std_logic; --data_out is valid (not valid when stuffing finished,stopped or error)
    signal error : out std_logic; --error occurs when input dataSize is bigger than data_buffer size
    signal data_rem : out std_logic_vector (11 downto 0) --amount of bits remaining to be transmitted
  ); 
  end component;
 begin
  bitStuffer_tb:bitStuffer 
  GENERIC MAP(
      length=>5,
      maxDataSize=>512
  )
  PORT MAP(
      clk_sys=>clk_sys,
      clk_nbt=>clk_nbt,
      clk_dbt=>clk_dbt,
      data_in=>data_in,
      start=>start,
      data_size=>data_size,
      fixed_stuff=>fixed_stuff,
      stop=>stop,
      res_n=>res_n,
      data_out=>data_out,
      finished=>finished,
      valid=>valid,
      error=>error,
      data_rem=>data_rem,
      bt_type=>bt_type,
      bs_trig_nbt=>bs_trig_nbt,
      bs_trig_dbt=>bs_trig_dbt
  );
  
 --clock generation process
 clock_gen:process
  begin
  clk_sys<='1';
  wait for 10 ns;
  clk_sys<='0';
  wait for 10 ns;
 end process;
 
 --clock generation process with trigger for sample of input 40 ns period
 clock_gen2:process
  begin
   clk_nbt<='1';
  wait for 20 ns;
   clk_nbt<='0';
   bs_trig_nbt<='1';
   wait for 10 ns;
   bs_trig_nbt<='0';
   wait for 10 ns; 
  end process;
 
 clock_gen3:process
  begin
   clk_dbt<='1';
   wait for 30 ns;
   clk_dbt<='0';
   wait for 30 ns;  
  end process;
  
  
  
 --simple test for custom CAN frame from WIKIPEDIA 
 --Note: bs_trig_nbt has to be sent with the same period as data!! (40 ns)
 --In reality prescaler generates trig signal and also bit time clock for data transcieving!!!
 test1:process
 variable data:std_logic_vector (42 downto 0):= "0000000101000000001000000010100001100000001";
  begin
    bt_type<='0'; --Nominal bit time is taken
    res_n<='0';
    --NOT real in syntehsys only for simulation
    wait for 30 ns;
    res_n<='1';
    start<='1';
    data_size<=std_logic_vector(to_unsigned(43,12));
    fixed_stuff<='0';
    stop<='0';
    for I in 0 to 42 loop
      data_in<=data(42-I);
      wait for 40 ns;
       start<='0';
    end loop;
    data_in<='0';
    wait for 1 us;
  end process test1;
  
  --THIS TEST IS more reliable than test1  
  --test with randomly generated data
  --test_random:process
  --variable data_length:integer:=200;
  --variable input_data:std_logic_vector(data_length-1 downto 0);
 -- variable output_data:std_logic_vector(data_length+data_length/length downto 0);
 -- variable seed1,seed2:positive;
 -- variable rand:real;
 -- variable sameCount:integer;
 -- variable stuffed:integer;
 -- begin
 --   
 --   --Generating random input data
 --   for i in 0 to data_length-1 loop
 --    UNIFORM(seed1, seed2, rand);     
 --    if(rand>0.5)then
 --     input_data(i):='1';
 --     else
 --     input_data(i):='0';
 --     end if; 
 --   end loop;
    
 --   --Generating correct bitstuffed output
 --   sameCount:=0;
 --   stuffed:=0;
 --   for i in 0 to data_length-1 loop
 --     if(sameCount=5)then
 --       sameCount:=0;
 --       stuffed:=stuffed+1;
 --       output_data(i+stuffed):=not output_data(i+stuffed-1);
 --     else
 --       sameCount:=sameCount+1;
 --       output_data(i+stuffed):=input_data(i);
 --     end if;
 --   end loop;
 --   
 --   res_n<='0';
 --   wait for 99 ns;
 --   res_n<='1';
 --   start<='1';
 --   data_size<=std_logic_vector(to_unsigned(43,12));
 --   fixed_stuff<='0';
 --   stop<='0';
 --   
 --   for I in 0 to data_length-1 loop
 --     data_in<=input_data(i);
 --     wait for 100 ns;
 --     test_data<=output_data(i);
 --     if(NOT data_out=input_data(i))then
 --       data_mismatch<='1';  
 --     else
 --       data_mismatch<='0';
 --     end if; 
 --     
 --     start<='0';
 --   end loop;
 --   data_in<='0';
 --   wait for 1 ms;
    
--end process test_random;
  
    
  
  

end architecture;
  
  
  
