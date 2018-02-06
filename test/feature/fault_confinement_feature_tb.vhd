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

package fault_conf_feature is
  
  procedure fault_conf_feature_exec(
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


package body fault_conf_feature is
  
   procedure fault_conf_feature_exec(
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
  variable rand_val             :     real;
  variable th_1                 :     natural:=0;
  variable rxc                  :     natural:=0;
  variable txc                  :     natural:=0;
  begin
    outcome:=true;
    
    -----------------------------------------------
    -- Generate random setting of ERP treshold
    -- and RX counters to preset
    -----------------------------------------------
    rand_real_v(rand_ctr,rand_val);
    th_1:=integer(rand_val*254.0);
    
    rand_real_v(rand_ctr,rand_val);
    rxc:=integer(rand_val*257.0);
    
    rand_real_v(rand_ctr,rand_val);
    txc:=integer(rand_val*257.0);
    
    
    -----------------------------------------------
    -- Set the counter and tresholds
    -----------------------------------------------
    r_data := (OTHERS => '0');
    r_data(ERP_LIMIT_H downto ERP_LIMIT_L):= 
         std_logic_vector(to_unsigned(th_1,8));
    CAN_write(r_data,EWL_ADR,ID_1,mem_bus_1);
    
    r_data := (OTHERS => '0');
    r_data(CTR_PRES_VAL_H downto CTR_PRES_VAL_L):= 
         std_logic_vector(to_unsigned(txc,9));
    r_data(PTX_IND):='1';
    CAN_write(r_data,CTR_PRES_ADR,ID_1,mem_bus_1);
    
    r_data := (OTHERS => '0');
    r_data(CTR_PRES_VAL_H downto CTR_PRES_VAL_L):= 
         std_logic_vector(to_unsigned(rxc,9));
    r_data(PRX_IND):='1';
    CAN_write(r_data,CTR_PRES_ADR,ID_1,mem_bus_1);
    
    
    -----------------------------------------------
    -- Read counters back
    -----------------------------------------------
    CAN_read(r_data,RXC_ADR,ID_1,mem_bus_1);
    
   if( to_integer(unsigned(r_data(RXC_VAL_H downto RXC_VAL_L))) /= rxc )then
      outcome:=false;
    end if;
    
    if( to_integer(unsigned(r_data(TXC_VAL_H downto TXC_VAL_L))) /= txc )then
      outcome:=false;
    end if;
    
    -----------------------------------------------
    -- Read fault confinement state
    -----------------------------------------------
    CAN_read(r_data,EWL_ADR,ID_1,mem_bus_1);
    
    if(txc>255 or rxc>255)then
      if(r_data(ERA_IND)='1' or 
         r_data(ERP_IND)='1' or 
         r_data(BOF_IND)='0')
      then
        outcome:=false;
      end if;
    elsif(txc<th_1 and rxc<th_1) then
      if(r_data(ERA_IND)='0' or
         r_data(ERP_IND)='1' or
         r_data(BOF_IND)='1')
      then
        outcome:=false;
      end if;
    else
      if(r_data(ERA_IND)='1' or
         r_data(ERP_IND)='0' or
         r_data(BOF_IND)='1')
      then
        outcome:=false;
      end if;
      
    end if;
    
    
  end procedure;
  
end package body;
