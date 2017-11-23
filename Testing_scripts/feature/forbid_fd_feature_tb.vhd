Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;

-------------------------------------------------------------------------------------------------------------
--
-- CAN with Flexible Data-Rate IP Core 
--
-- Copyright (C) 2015 Ondrej Ille <ondrej.ille@gmail.com>
--
-- This program is free software; you can redistribute it and/or
-- modify it under the terms of the GNU General Public License
-- as published by the Free Software Foundation; either version 2
-- of the License, or (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.
--
-- The CAN protocol is developed by Robert Bosch GmbH and     
-- protected by patents. Anybody who wants to implement this    
-- IP core on silicon has to obtain a CAN protocol license
-- from Bosch.
--
--
-- Revision History:
--
--    21.6.2016   Created file
-----------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Forbidding FD frames feature test implementation
--                                      
-----------------------------------------------------------------------------------------------------------------


package forbid_fd_feature is
  
  procedure forbid_fd_feature_exec(
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


package body forbid_fd_feature is
  
   procedure forbid_fd_feature_exec(
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
  variable r_data               :     std_logic_vector(31 downto 0):=(OTHERS => '0');
  variable CAN_frame            :     SW_CAN_frame_type;
  variable frame_sent           :     boolean:=false;
  variable ctr_1                :     natural;
  variable ctr_2                :     natural;
  variable ID_1           	     :     natural:=1;
  variable ID_2           	     :     natural:=2;
  begin
    outcome:= true;
    
    -----------------------------------------------
    --First disable the FD support of Node 1
    -----------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    r_data(4) := '0';
    CAN_write(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    
    -----------------------------------------------
    -- Read RX Error counter node 1
    -----------------------------------------------
    CAN_read(r_data,ERROR_COUNTERS_ADR,ID_1,mem_bus_1);
    ctr_1 := to_integer(unsigned(r_data(15 downto 0)));
    
    -----------------------------------------------
    --Send FD frame by node 2 and wait for error
    -- frame...
    -----------------------------------------------
    CAN_generate_frame(rand_ctr,CAN_frame);
    CAN_frame.frame_format:= '1';
    CAN_send_frame(CAN_frame,1,ID_2,mem_bus_2,frame_sent);
    CAN_wait_error_transmitted(ID_2,mem_bus_2);
    CAN_wait_bus_idle(ID_2,mem_bus_2);
    
    -----------------------------------------------
    -- Read RX Error counter node 1 again
    -----------------------------------------------
    CAN_read(r_data,ERROR_COUNTERS_ADR,ID_1,mem_bus_1);
    ctr_2 := to_integer(unsigned(r_data(15 downto 0)));
    
    --Counter should be increased by one
    if( ctr_1+1+8 /= ctr_2)then
      outcome := false;
    end if;
    
    -----------------------------------------------
    -- Now send the same frame, but not the FD
    -- type. Wait until bus is idle
    -----------------------------------------------
    CAN_frame.frame_format:= '0';
    CAN_send_frame(CAN_frame,1,ID_2,mem_bus_2,frame_sent);
    CAN_wait_frame_sent(ID_2,mem_bus_2);
    
    -----------------------------------------------
    -- Read RX Error counter node 1 again
    -----------------------------------------------
    CAN_read(r_data,ERROR_COUNTERS_ADR,ID_1,mem_bus_1);
    ctr_2 := to_integer(unsigned(r_data(15 downto 0)));
    
    --Counter should be decreased by one now due
    -- to sucesfull reception.
    -- But it should be increased by 8 since it is the
    -- first node that detected the error!
    if( ctr_1+8 /= ctr_2)then
      outcome := false;
    end if;
    
    
    -----------------------------------------------
    --Now enable the FD support of Node 1
    -----------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    r_data(4) := '1';
    CAN_write(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    
    -----------------------------------------------
    -- Now again send the same frame but FD type
    -- now unit should accept the frame OK!
    -----------------------------------------------
    CAN_frame.frame_format:= '1';
    CAN_send_frame(CAN_frame,1,ID_2,mem_bus_2,frame_sent);
    CAN_wait_frame_sent(ID_2,mem_bus_2);
    
    -----------------------------------------------
    -- Read RX Error counter node 1 again
    -----------------------------------------------
    CAN_read(r_data,ERROR_COUNTERS_ADR,ID_1,mem_bus_1);
    ctr_2 := to_integer(unsigned(r_data(15 downto 0)));
    
    --Counter should be less than the value read now
    -- or both should be zeroes when counter
    --  cannnot already be lowered...
    if( ctr_1+7 /= ctr_2)then
      outcome := false;
    end if; 
    
    
    -----------------------------------------------
    -- Since counter is incremented more than 
    -- decremented, after many iterations UNIT
    -- will go to error_passive and then bus_off
    -- To forbid this we clear error counters
    -----------------------------------------------
    if(ctr_2>70)then
      report "Resetting error counters";
      r_data :=(OTHERS => '0');
      r_data(10 downto 9) := "11";
      CAN_write(r_data,ERROR_COUNTERS_ADR,ID_1,mem_bus_1);
      CAN_write(r_data,ERROR_COUNTERS_ADR,ID_2,mem_bus_2);
    end if;
    
    
  end procedure;
  
end package body;


