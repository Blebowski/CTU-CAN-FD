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

---------------------------------------------------------------------------------------------------------------
--Simple testbench for CAN top level entity. Refer to top_level_2_tb.vhd. There are all supported tests with --
--whole interfacing connection!                                                                              --
---------------------------------------------------------------------------------------------------------------
-- Revision History:
--
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
USE WORK.CANconstants.ALL;


entity CAN_top_level_tb is
end entity;

architecture behav of CAN_top_level_tb is
  component CAN_top_level is
  generic(
      constant use_logger:boolean:=false; --Whenever event logger should be synthetised (not yet implemented)
      constant rx_buffer_size:natural:=128; --Transcieve Buffer size
      constant use_sync:boolean:=false; --Whenever internal synchroniser chain should be used for incoming bus signals
      constant ID:natural range 0 to 15:=1; --ID (bits  19-16 of adress) 
      constant useFDSize:boolean
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
      --Note: This bus is Avalon compatible!
      
      signal int:out std_logic;
      
      signal CAN_tx:out std_logic;
      signal CAN_rx:in std_logic;
      
      signal time_quanta_clk:out std_logic; --Bit time clocks possible to be used for synchronisation
      
      signal timestamp:in std_logic_vector(63 downto 0)
  );
  end component;
  
      constant use_logger:boolean:=false; --Whenever event logger should be synthetised (not yet implemented)
      constant rx_buffer_size:natural:=128; --Transcieve Buffer size
      constant use_sync:boolean:=false; --Whenever internal synchroniser chain should be used for incoming bus signals
      constant ID:natural range 0 to 15:=1 ;--ID (bits  19-16 of adress) 
  
      signal clk_sys: std_logic;
      signal res_n: std_logic;
      
      signal data_in: std_logic_vector(31 downto 0);
      signal data_out: std_logic_vector(31 downto 0);
      signal adress: std_logic_vector(23 downto 0);
    	 signal scs: std_logic; --Chip select
      signal srd: std_logic; --Serial read
      signal swr: std_logic; --Serial write
      --Note: This bus is Avalon compatible!
      
      signal int: std_logic;     
      signal CAN_tx: std_logic;
      signal CAN_rx: std_logic;     
      signal time_quanta_clk: std_logic; --Bit time clocks possible to be used for synchronisation
      signal timestamp: std_logic_vector(63 downto 0);
  
      -----------------------
      --Testbench constants--
      -----------------------
      constant clk_per:time:=10 ns;
      
      procedure data_write
      (constant data_value : in std_logic_vector(31 downto 0);
       constant adress_value:in  std_logic_vector(15 downto 0);
       signal data:out std_logic_vector(31 downto 0);
       signal adress_in:out std_logic_vector(23 downto 0);
       signal scs_in: out std_logic;
       signal swr_in: out std_logic
       ) is
      begin   
        data<=data_value;
        adress_in<=TO_STDLOGICVECTOR(X"41")&adress_value;
        swr_in<='1';
        scs_in<='1';
        wait for 3*clk_per;
        swr_in<='0';
        scs_in<='0';
        data<=(OTHERS =>'0');
        adress_in<=(OTHERS=>'0');
        wait for 3*clk_per;
        
      end procedure;
      
      procedure data_read
      (constant adress_value:in  std_logic_vector(15 downto 0);
       signal adress_in:out std_logic_vector(23 downto 0);
       signal scs_in: out std_logic;
       signal srd_in: out std_logic
       ) is
      begin   
        adress_in<=TO_STDLOGICVECTOR(X"41")&adress_value;
        srd_in<='1';
        scs_in<='1';
        wait for 4*clk_per;
        srd_in<='0';
        scs_in<='0';
        wait for 3*clk_per;
      end procedure;
      
begin
  top_lvl_comp:CAN_top_level
  generic map(
    use_logger=>use_logger,
    rx_buffer_size=>rx_buffer_size,
    use_sync=>use_sync,
    useFDSize=>true,
    ID=>ID
    )
  port map(
    	  clk_sys=>clk_sys,
       res_n=>res_n,
       data_in=>data_in,
       data_out=>data_out,
       adress=>adress,
    	  scs=>scs,
       srd=>srd,
       swr=>swr,
       int=>int,
       CAN_tx=>CAN_tx,
       CAN_rx=>CAN_rx,
       time_quanta_clk=>time_quanta_clk,
       timestamp=>timestamp
  );
  
  clock_gen:process
  begin
    clk_sys<='1';
    wait for clk_per/2;
    clk_sys<='0';
    wait for clk_per/2;
  end process;
  
  CAN_rx<=CAN_tx;
  
  test1:process
  begin
    --Resetting the circuit
    res_n<=ACT_RESET;
    data_in<=(OTHERS=>'0');
    adress<=(OTHERS=>'0');
    scs<='0';
    swr<='0';
    srd<='0';
    timestamp<=(OTHERS=>'1');
    
    wait for 2*clk_per-3 ns;
    res_n<=not ACT_RESET;
    wait for 100 ns;
    
    data_read(TO_STDLOGICVECTOR(X"0000"),adress,scs,srd); --Reading out identifier
    data_read(TO_STDLOGICVECTOR(X"0004"),adress,scs,srd); --Reading out MODE_REG(MODE,COMAND,STAT,RETR)
    
    --Configure : retransmit enable=1, retr_limit=3
    data_write(TO_STDLOGICVECTOR(X"07000C10"),TO_STDLOGICVECTOR(X"0004"),data_in,adress,scs,swr);
    
    --Note: Timing and synchronisation default settings: PResc:10,5
    data_read(TO_STDLOGICVECTOR(X"0044"),adress,scs,srd);
    
    --DLC=4, FF=Normal, Ft=Basic, Switch BRS=No
    data_write(TO_STDLOGICVECTOR(X"00000004"),TO_STDLOGICVECTOR(X"005C"),data_in,adress,scs,swr);
    data_write(TO_STDLOGICVECTOR(X"00000000"),TO_STDLOGICVECTOR(X"0060"),data_in,adress,scs,swr); --TimeStamp 1
    data_write(TO_STDLOGICVECTOR(X"00000000"),TO_STDLOGICVECTOR(X"0064"),data_in,adress,scs,swr); --TimeStamp 2
    data_write(TO_STDLOGICVECTOR(X"000000AA"),TO_STDLOGICVECTOR(X"0068"),data_in,adress,scs,swr); --Identifier
    data_write(TO_STDLOGICVECTOR(X"AABBCCDD"),TO_STDLOGICVECTOR(X"006C"),data_in,adress,scs,swr); --Identifier
    
    --Commit the data
    data_write(TO_STDLOGICVECTOR(X"0000000B"),TO_STDLOGICVECTOR(X"0058"),data_in,adress,scs,swr); --Store the messages
    
   
    
    wait for 100 ms;
  end process;
  
end architecture;
