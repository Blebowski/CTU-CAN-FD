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
--  Feature test for setting error counters from user and its appropriate fault
--  confinement state manipulation!
--                                      
--------------------------------------------------------------------------------
-- Revision History:
--
--    30.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;

use work.CAN_FD_register_map.all;

package overload_feature is
  
  procedure overload_feature_exec(
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
    signal      stat_bus_2      :in     std_logic_vector(511 downto 0);
    signal      bl_inject       :inout  std_logic;
    signal      bl_force        :inout  boolean
  );
  
end package;


package body overload_feature is
  
   procedure overload_feature_exec(
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
    signal      stat_bus_2      :in     std_logic_vector(511 downto 0);
    signal      bl_inject       :inout  std_logic;
    signal      bl_force        :inout  boolean
  )is
  variable r_data               :     std_logic_vector(31 downto 0):=(OTHERS => '0');
  variable CAN_frame            :     SW_CAN_frame_type;
  variable frame_sent           :     boolean:=false;
  variable ctr_1                :     natural;
  variable ctr_2                :     natural;
  variable ID_1           	     :     natural:=1;
  variable ID_2           	     :     natural:=2;
  variable rand_val             :     real;
  variable retr_th              :     natural;
  variable mode_backup          :     std_logic_vector(31 downto 0):=(OTHERS => '0');
  begin
    outcome:=true;
    
    CAN_generate_frame(rand_ctr,CAN_frame);     
    CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent);
    
    --Wait until intermission field starts
    wait until protocol_type'VAL(to_integer(unsigned(stat_bus_1(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))))=
                interframe;
    
    -----------------------------------------------
    --Inject dominant bit during the intermission
    ----------------------------------------------
    bl_inject<= DOMINANT;
    bl_force <= true;
    
    --Wait for change on protocol state    
    wait until protocol_type'VAL(to_integer(unsigned(stat_bus_1(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW)))) /=
                interframe;    
    
    --Now overload should have started
    if (protocol_type'VAL(to_integer(unsigned(stat_bus_1(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW)))) /= overload)then
      outcome:=false;
    end if;
    
    --Read overload from debug register
    CAN_read(r_data,DEBUG_REGISTER_ADR,ID_1,mem_bus_1);
    if(r_data(PC_OVR_IND)='0')then
      outcome:=false;
    end if;
    
    bl_inject<= RECESSIVE;
    bl_force<=false;
    
    
    
    
    --Wait until intermission field starts
    wait until protocol_type'VAL(to_integer(unsigned(stat_bus_1(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))))=
                interframe;
    
    -----------------------------------------------
    --Inject dominant bit during the intermission
    ----------------------------------------------
    bl_inject<= DOMINANT;
    bl_force <= true;            
        
        
    --Wait for change on protocol state    
    wait until protocol_type'VAL(to_integer(unsigned(stat_bus_1(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW)))) /=
                interframe;    
    
    --Now overload should have started
    if (protocol_type'VAL(to_integer(unsigned(stat_bus_1(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW)))) /= overload)then
      outcome:=false;
    end if;
    
    --Read overload from debug register
    CAN_read(r_data,DEBUG_REGISTER_ADR,ID_1,mem_bus_1);
    if(r_data(11)='0')then
      outcome:=false;
    end if;
    
    bl_inject <=  RECESSIVE;
    bl_force  <=  false;        
    
    CAN_wait_frame_sent(ID_1,mem_bus_1);
    
  end procedure;
  
end package body;
