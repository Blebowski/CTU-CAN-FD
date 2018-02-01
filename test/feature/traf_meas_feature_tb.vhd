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
--  Traffic measurment feature test implementation
-- 
--  Test sequence is like so:
--    1. Generate random number N from 0 to 5
--    2. Measure TX counter of node 1 and RX counter of node 2
--    3. Send N random frames
--    4. Measure TX counter of node 1 and RX counter of node 2 again
--    5. Compare if both counters were increased by N 
--  
--
-----------------------------------------------------------------------------------------------------------------
-- Revision History:
--
--    24.6.2016   Created file
-----------------------------------------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;

use work.CAN_FD_register_map.all;

package traf_meas_feature is
  
  procedure traf_meas_feature_exec(
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


package body traf_meas_feature is
  
  procedure traf_meas_feature_exec(
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
  variable size_of_buf          :       natural;
  variable ID_1           	     :     natural:=1;
  variable ID_2           	     :     natural:=2;
  variable CAN_frame            :     SW_CAN_frame_type;
  variable frame_sent           :     boolean:=false;
  variable aux                 :     natural range 0 to 1023;
  variable rand_value           :     real;
  variable tx_ctr               :    natural range 0 to 536870912;
  variable rx_ctr               :    natural range 0 to 536870912;
  variable tx_ctr_2               :    natural range 0 to 536870912;
  variable rx_ctr_2               :    natural range 0 to 536870912;
  begin
    outcome:= true;
    --------------------------------------------
    -- Check the TX RX counters
    --------------------------------------------
    CAN_read(r_data,TX_COUNTER_ADR,ID_1,mem_bus_1);
    tx_ctr:= to_integer(unsigned(r_data(TX_COUNTER_VAL_H downto TX_COUNTER_VAL_L)));
    CAN_read(r_data,RX_COUNTER_ADR,ID_2,mem_bus_2);
    rx_ctr:= to_integer(unsigned(r_data(RX_COUNTER_VAL_H downto RX_COUNTER_VAL_L)));
    
    --------------------------------------------
    -- Generate the CAN frames to send
    --------------------------------------------
    rand_real_v(rand_ctr,rand_value);
    aux:=integer(5.0*rand_value);
    
    for i in 0 to aux loop
      
      CAN_generate_frame(rand_ctr,CAN_frame);
      CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent);
      CAN_wait_frame_sent(ID_1,mem_bus_1);
      
    end loop;
    
    --------------------------------------------
    -- Check the TX RX counters
    --------------------------------------------
    CAN_read(r_data,TX_COUNTER_ADR,ID_1,mem_bus_1);
    tx_ctr_2:= to_integer(unsigned(r_data(TX_COUNTER_VAL_H downto TX_COUNTER_VAL_L)));
    CAN_read(r_data,RX_COUNTER_ADR,ID_2,mem_bus_2);
    rx_ctr_2:= to_integer(unsigned(r_data(RX_COUNTER_VAL_H downto RX_COUNTER_VAL_L)));
    
    if(tx_ctr+aux+1 /= tx_ctr_2)then
      outcome:=false;
    end if;
    
    if(rx_ctr+aux+1 /= rx_ctr_2)then
      outcome:=false;
    end if;
    
    
  end procedure;
  
end package body;