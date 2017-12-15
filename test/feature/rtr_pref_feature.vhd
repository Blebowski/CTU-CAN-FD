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

-----------------------------------------------------------------------------------------------------------------
-- Purpose:
--  
-- 
--  Test sequence is like so:
--    1. 
--    2. 
--    3. 
--    4.
--    5. 
--    6. 
--
-----------------------------------------------------------------------------------------------------------------
-- Revision History:
--
--    23.6.2016   Created file
-----------------------------------------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;

package rtr_pref_feature is
  
  procedure rtr_pref_feature_exec(
    variable   outcome      : inout boolean;
    signal      rand_ctr        :inout  natural range 0 to RAND_POOL_SIZE;
    signal      mem_bus_1       :inout  Avalon_mem_type;
    signal      mem_bus_2       :inout  Avalon_mem_type;
    --Additional signals for tests
    --Pretty much everything can be read out of stat bus...
    signal      bus_level       :in     std_logic;
    signal      drv_bus_1       :in     std_logic_vector(1023 downto 0);
    signal      drv_bus_2       :in     std_logic_vector(1023 downto 0);
    signal      stat_bus_1      :in     std_logic_vector(511 downto 0);
    signal      stat_bus_2      :in     std_logic_vector(511 downto 0) 
  );
  
end package;


package body rtr_pref_feature is
  
  procedure rtr_pref_feature_exec(
    variable   outcome      : inout boolean;
    signal      rand_ctr        :inout  natural range 0 to RAND_POOL_SIZE;
    signal      mem_bus_1       :inout  Avalon_mem_type;
    signal      mem_bus_2       :inout  Avalon_mem_type;
    --Additional signals for tests
    --Pretty much everything can be read out of stat bus...
    signal      bus_level       :in     std_logic;
    signal      drv_bus_1       :in     std_logic_vector(1023 downto 0);
    signal      drv_bus_2       :in     std_logic_vector(1023 downto 0);
    signal      stat_bus_1      :in     std_logic_vector(511 downto 0);
    signal      stat_bus_2      :in     std_logic_vector(511 downto 0) 
  )is
  variable r_data               :       std_logic_vector(31 downto 0):=(OTHERS => '0');
  variable w_data               :       std_logic_vector(31 downto 0):=(OTHERS => '0');
  variable ID_1           	     :     natural:=1;
  variable ID_2           	     :     natural:=2;
  variable CAN_frame            :     SW_CAN_frame_type;
  variable frame_sent           :     boolean:=false;
  begin
    outcome:=true;
    
    -----------
    --Part 1
    -----------
    ----------------------------------------------
    --Generate CAN frame
    ----------------------------------------------
    CAN_generate_frame(rand_ctr,CAN_frame);
    CAN_frame.rtr:='1';
    CAN_frame.frame_format:='0';
    
    ----------------------------------------------
    -- Set the RTR preferred behaviour to 
    -- send the DLC all zeroes..
    ---------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    r_data(5) := '1';  --RTR preffered bit
    CAN_write(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    
    ---------------------------------------------
    --Restart the content of the Node 2 RX Buffer
    ---------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    r_data(10) := '1';  --Release recieve buffer bit
    CAN_write(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    
    ----------------------------------------------
    --Insert the frame for transmittion
    -- and wait until recieved
    ----------------------------------------------
    CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent);
    if(frame_sent=false)then
      outcome:=false;
    end if;
    CAN_wait_frame_sent(ID_2,mem_bus_2);
    
    ---------------------------------------------
    --Check that recieved DLC is zero
    ---------------------------------------------
    CAN_read(r_data,RX_DATA_ADR,ID_2,mem_bus_2);
    if(r_data(3 downto 0) /= "0000")then
      outcome:=false;
    end if;
    
  
    -----------
    --Part 2
    -----------
    ----------------------------------------------
    -- Set the RTR preferred behaviour to 
    -- send the original DLC
    ---------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    r_data(5) := '0';  --RTR preffered bit
    CAN_write(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    
    ---------------------------------------------
    --Restart the content of the Node 2 RX Buffer
    ---------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    r_data(10) := '1';  --Release recieve buffer bit
    CAN_write(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    
    ----------------------------------------------
    --Insert the frame for transmittion
    -- and wait until recieved
    ----------------------------------------------
    CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent);
    if(frame_sent=false)then
      outcome:=false;
    end if;
    CAN_wait_frame_sent(ID_2,mem_bus_2);
    
    ---------------------------------------------
    --Check that recieved DLC is matching transc.
    ---------------------------------------------
    CAN_read(r_data,RX_DATA_ADR,ID_2,mem_bus_2);
    if(r_data(3 downto 0) /= CAN_frame.dlc)then
      outcome:=false;
    end if;
    
  end procedure;
  
end package body;


