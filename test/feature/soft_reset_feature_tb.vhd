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
--------------------------------------------------------------------------------
-- Revision History:
--
--    28.6.2016   Created file
--    1.9.2016    Changed test to be compliant with latest change in register memory map!
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;

use work.CAN_FD_register_map.all;

package soft_reset_feature is
  
  procedure soft_reset_feature_exec(
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


package body soft_reset_feature is
  
  procedure soft_reset_feature_exec(
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
    
    --------------------------------------------------
    --Write into the restart bit of MODE_REG
    --------------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    r_data(RST_IND) := '1';
    CAN_write(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    
    --------------------------------------------------
    --Continously check on expected default values!
    --------------------------------------------------
    CAN_read(r_data,DEVICE_ID_ADR,ID_1,mem_bus_1);
    if(r_data /= DEVICE_ID_RSTVAL)then
      outcome:=false;
    end if;
    
    --Mode register
    CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    if(r_data /= "00000000100000000000000000110000")then
      outcome:=false;
    end if;
    
    --Interrupt registers
    CAN_read(r_data,INTERRUPT_REG_ADR,ID_1,mem_bus_1);
    if(r_data /= "00000000001011000000000000000000")then
      outcome:=false;
    end if;
    
     --Timing register
    CAN_read(r_data,TIMING_REG_ADR,ID_1,mem_bus_1);
    if(r_data /= "00011000110000110010100011000101")then
      outcome:=false;
    end if;
    
     --ALC presc register
    CAN_read(r_data,ARB_ERROR_PRESC_ADR,ID_1,mem_bus_1);
    if(r_data(7 downto 0) /= "00000000")then
      outcome:=false;
    end if;
    if(r_data(15 downto 8) /= "00100010")then
      outcome:=false;
    end if;
    if(r_data(31 downto 16) /= "0000010000001010")then
      outcome:=false;
    end if;
    
    --EWL register
    CAN_read(r_data,ERROR_TH_ADR,ID_1,mem_bus_1);
    if(r_data(7 downto 0) /= "01100000")then --96
      outcome:=false;
    end if;
    if(r_data(15 downto 8) /= "10000000")then --128
      outcome:=false;
    end if;
    
    if(r_data(31 downto 16) /= "0000000000000001")then --Only Error active
      outcome:=false;
    end if;
    
    --Error registers
    CAN_read(r_data,ERROR_COUNTERS_ADR,ID_1,mem_bus_1);
     if(r_data(31 downto 0) /= "00000000000000000000000000000000")then
      outcome:=false;
    end if;
    
    --Special Error registers
    CAN_read(r_data,ERROR_COUNTERS_SPEC_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "00000000000000000000000000000000")then
      outcome:=false;
    end if;
    
    ----------
    --Filters
    ----------
    CAN_read(r_data,FILTER_A_VAL_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "00000000000000000000000000000000")then
      outcome:=false;
    end if;
    
    CAN_read(r_data,FILTER_B_VAL_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "00000000000000000000000000000000")then
      outcome:=false;
    end if;
    
    CAN_read(r_data,FILTER_C_VAL_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "00000000000000000000000000000000")then
      outcome:=false;
    end if;
    
    CAN_read(r_data,FILTER_A_MASK_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "00000000000000000000000000000000")then
      outcome:=false;
    end if;
    
    CAN_read(r_data,FILTER_B_MASK_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "00000000000000000000000000000000")then
      outcome:=false;
    end if;
    
    CAN_read(r_data,FILTER_C_MASK_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "00000000000000000000000000000000")then
      outcome:=false;
    end if;
    
    CAN_read(r_data,FILTER_RAN_LOW_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "00000000000000000000000000000000")then
      outcome:=false;
    end if;
    
    CAN_read(r_data,FILTER_RAN_HIGH_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "00000000000000000000000000000000")then
      outcome:=false;
    end if;
    
    CAN_read(r_data,FILTER_CONTROL_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "00000000000000000000000000001111")then
      outcome:=false;
    end if;
    
    --RX Info 1
    CAN_read(r_data,RX_INFO_1_ADR,ID_1,mem_bus_1);
    if(r_data(15 downto 0) /= "0000000000000001")then
      outcome:=false;
    end if;
    if(r_data(23 downto 16) /= "01000000")then
      outcome:=false;
    end if;
    
    
    --RX Info 2
    CAN_read(r_data,RX_INFO_2_ADR,ID_1,mem_bus_1);
    if(r_data(7 downto 0) /= "01000000")then
      outcome:=false;
    end if;
    
    if(r_data(31 downto 8) /= "000000000000000000000000")then
      outcome:=false;
    end if;
    
    --Rx address
    CAN_read(r_data,RX_DATA_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "000000000000000000000000000000000")then
      outcome:=false;
    end if;
    
    --TX Status
    CAN_read(r_data,TX_STATUS_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "000000000000000000000000000000011")then
      outcome:=false;
    end if;
    
    --TX Settings
    CAN_read(r_data,TX_SETTINGS_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "000000000000000000000000000000011")then
      outcome:=false;
    end if;
    
    --RX Counter
    CAN_read(r_data,RX_COUNTER_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "000000000000000000000000000000000")then
      outcome:=false;
    end if;
    
    --TX Counter
    CAN_read(r_data,TX_COUNTER_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "000000000000000000000000000000000")then
      outcome:=false;
    end if;
    
    --Log trig config
    CAN_read(r_data,LOG_TRIG_CONFIG_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "000000000000000000000000000000000")then
      outcome:=false;
    end if;
    
    --Log capt config
    CAN_read(r_data,LOG_CAPT_CONFIG_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "000000000000000000000000000000000")then
      outcome:=false;
    end if;
    
    --Log status
    CAN_read(r_data,LOG_STATUS_ADR,ID_1,mem_bus_1);
    if(r_data(7 downto 0) /= "00000001")then
      outcome:=false;
    end if;
    if(r_data(15 downto 8) /= "00010000")then
      outcome:=false;
    end if;
    if(r_data(31 downto 16) /= "0000000000000000")then
      outcome:=false;
    end if;
    
    --DEBUG
    CAN_read(r_data,DEBUG_REG_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= "00000000000000000000000000000000")then
      outcome:=false;
    end if;
    
    --YOLO
    CAN_read(r_data,YOLO_REG_ADR,ID_1,mem_bus_1);
    if(r_data(31 downto 0) /= x"DEADBEEF")then
      outcome:=false;
    end if;
    
  end procedure;
  
end package body;

