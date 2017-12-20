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
--  Wrapper for testing the synthesis size with different options
--------------------------------------------------------------------------------
-- Revision History:
--    19.12.2017   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.std_logic_unsigned.All;
USE WORK.CANconstants.ALL;
use work.CANcomponents.ALL;

entity CAN_Wrapper is
  
    generic (
	  constant use_logger     : boolean := false;              
	  constant rx_buffer_size : natural := 32;
     constant  useFDSize      : boolean := true;          
     constant  use_sync       : boolean:= true;             
     constant  ID             : natural:= 1; 
     constant  sup_filtA      : boolean:= true;              
     constant  sup_filtB      : boolean:= true;                
     constant  sup_filtC      : boolean:= true;                
     constant  sup_range      : boolean:= true;               
     constant  tx_time_sup    : boolean:= true;                
     constant  logger_size    : natural:= 8  
    );
	 port (
     signal clk_sys : in std_logic;
     signal res_n   : in std_logic;

     signal data_in  : in  std_logic_vector(31 downto 0);
     signal data_out : out std_logic_vector(31 downto 0);
     signal adress   : in  std_logic_vector(23 downto 0);
     signal scs      : in  std_logic;    --Chip select
     signal srd      : in  std_logic;    --Serial read
     signal swr      : in  std_logic;    --Serial write
	  signal sbe      : in  std_logic_vector(3 downto 0); --BE
     
	  signal int : out std_logic;

     signal CAN_tx : out std_logic;
     signal CAN_rx : in  std_logic;

     signal time_quanta_clk : out std_logic;  
     signal timestamp : in std_logic_vector(63 downto 0)
    );  
 
end entity CAN_Wrapper;

architecture rtl of CAN_Wrapper is

     
--		--Minimal configuration
--		(
--		 use_logger     => false,
--		 rx_buffer_size => 16,
--       useFDSize      => false,
--       use_sync       => true,
--       ID             => 1,
--       sup_filtA      => false,
--       sup_filtB      => false,
--       sup_filtC      => false,
--       sup_range      => false,
--       tx_time_sup    => false,
--       logger_size    => 8
--		),
--	  
--	   --Minimal FD configuration
--		(
--		 use_logger     => false,
--		 rx_buffer_size => 32,
--       useFDSize      => true,
--       use_sync       => true,
--       ID             => 1,
--       sup_filtA      => false,
--       sup_filtB      => false,
--       sup_filtC      => false,
--       sup_range      => false,
--       tx_time_sup    => false,
--       logger_size    => 8
--		),
--	  
--	   --Small FD configuration
--		(
--		 use_logger     => false,
--		 rx_buffer_size => 32,
--       useFDSize      => true,
--       use_sync       => true,
--       ID             => 1,
--       sup_filtA      => true,
--       sup_filtB      => false,
--       sup_filtC      => false,
--       sup_range      => false,
--       tx_time_sup    => true,
--       logger_size    => 8
--		),
--	  
--	   --Medium FD configuration
--		(
--		 use_logger     => false,
--		 rx_buffer_size => 64,
--       useFDSize      => true,
--       use_sync       => true,
--       ID             => 1,
--       sup_filtA      => true,
--       sup_filtB      => false,
--       sup_filtC      => false,
--       sup_range      => true,
--       tx_time_sup    => true,
--       logger_size    => 8
--		),
--		
--		--Full FD configuration
--		(
--		 use_logger     => false,
--		 rx_buffer_size => 128,
--       useFDSize      => true,
--       use_sync       => true,
--       ID             => 1,
--       sup_filtA      => true,
--       sup_filtB      => true,
--       sup_filtC      => true,
--       sup_range      => true,
--       tx_time_sup    => true,
--       logger_size    => 8
--		),
--		
--		--Full FD configuration + Small Logger
--		(
--		 use_logger     => true,
--		 rx_buffer_size => 128,
--       useFDSize      => true,
--       use_sync       => true,
--       ID             => 1,
--       sup_filtA      => true,
--       sup_filtB      => true,
--       sup_filtC      => true,
--       sup_range      => true,
--       tx_time_sup    => true,
--       logger_size    => 8
--		),
--		
--		--Full FD configuration + Big Logger
--		(
--		 use_logger     => true,
--		 rx_buffer_size => 128,
--       useFDSize      => true,
--       use_sync       => true,
--       ID             => 1,
--       sup_filtA      => true,
--       sup_filtB      => true,
--       sup_filtC      => true,
--       sup_range      => true,
--       tx_time_sup    => true,
--       logger_size    => 64
--		)
--	 );
	  
		
	  

begin

  CAN_comp:CAN_top_level
  generic map(
	  use_logger     => use_logger,
     rx_buffer_size => rx_buffer_size,
     useFDSize      => useFDSize,
     use_sync       => use_sync,
     ID             => ID,
     sup_filtA      => sup_filtA,
     sup_filtB      => sup_filtB,
     sup_filtC      => sup_filtC,
     sup_range      => sup_range,
     tx_time_sup    => tx_time_sup,
     logger_size    => logger_size
    )
  port map(
     clk_sys 		  => clk_sys,
     res_n          => res_n,
     data_in        => data_in,
     data_out       => data_out,
     adress         => adress,
     scs            => scs,
     srd            => srd,
     swr            => swr,
	  sbe			     => sbe,
     int            => int,
     CAN_tx         => CAN_tx,
     CAN_rx         => CAN_rx,
     time_quanta_clk => time_quanta_clk,
     timestamp       => timestamp
    );
  
end architecture;
