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
--    22.6.2016   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;


package abort_transmittion_feature is
  
  procedure abort_transmittion_feature_exec(
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


package body abort_transmittion_feature is
  
  procedure abort_transmittion_feature_exec(
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
  variable frame_length         :     natural;
  variable rand_value           :     real;
  variable wt                   :     time;
  variable bus_config           :     bit_time_config_type;
  variable still_tx             :     boolean:=false;
  variable PC_State             :     protocol_type;
  begin
    outcome:=true;
    
    ----------------------------------------------
    --Generate CAN frame
    ----------------------------------------------
    CAN_generate_frame(rand_ctr,CAN_frame);
    
    ----------------------------------------------
    --Insert the frame for transmittion
    ----------------------------------------------
    CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent);
    
    ----------------------------------------------
    --Wait until unit turns transciever
    ----------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    while (r_data(16+TS_IND)='0') loop
      CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    end loop;
    
    ----------------------------------------------
    --Now wait random time up to 250 000 clock
    -- cycles!
    -- But at least 2000 clock cycles! We want
    -- to avoid situation that unit aborts
    -- immediately after the frame was commited
    -- and SOF was not yet sent!
    ----------------------------------------------
    rand_real_v(rand_ctr,rand_value);
    rand_value:= (rand_value*248000.0)+2000.0; 
    
    wt :=  integer(rand_value) * 1 ns;
    wait for wt;
    
    ----------------------------------------------
      --Check that unit is not transciever anymore, unit
      -- should be now as if bus was idle...
      -- But unit is still transciever even in interframe
      -- space. So we check the PC_State from status
      -- bus explicitly. This is not available for
      -- user but could be possibly added into the
      -- registers...
      -- Update: it was added...
      ----------------------------------------------
      PC_State:=  protocol_type'VAL
                                  (to_integer(unsigned
                                   (
                                    stat_bus_1(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW))
                                   )
                                  );
                                
      if (PC_State=sof or PC_State=arbitration or PC_State=control or
          PC_State=data) then
        still_tx:=true;
      else
        still_tx:=false;
      end if; 
    
    if(still_tx=true)then
      ------------------------------------------------
      --Now send the command to abort the transmittion
      ------------------------------------------------
      CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
      r_data(9) := '1';  --Abort transmittion bit
      CAN_write(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
   
      ----------------------------------------------
      --Now wait for few clock cycles until Node aborts the
      -- transmittion
      ----------------------------------------------
      for i in 0 to 5 loop
        wait until rising_edge(mem_bus_1.clk_sys);
      end loop;
    
      CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
      if (r_data(16+TS_IND)='1') then
        outcome:=false;
      end if;
    
    
      ----------------------------------------------
      --Now wait until unit 2 starts transmitting error frame!
      -- Note that we cant wait until unit 1 starts to
      -- transmitt error frame! THis works only when both
      --  units are error active! When unit 2 turns
      --  error passive unit 1 never starts transmitting active
      --  error frame. It stays idle since it recieves
      --  recessive error passive errro frame from 2! 
      ----------------------------------------------
      CAN_wait_error_transmitted(ID_2,mem_bus_2);
      --CAN_wait_error_transmitted(ID_1,mem_bus_1);
      
      ----------------------------------------------
      --Now wait until bus is idle in both units
      ----------------------------------------------
      CAN_wait_bus_idle(ID_2,mem_bus_2);
      CAN_wait_bus_idle(ID_1,mem_bus_1);
      
    else
      
      ----------------------------------------------
      --Now wait until bus is idle in both units
      ----------------------------------------------
      CAN_wait_bus_idle(ID_2,mem_bus_2);
      CAN_wait_bus_idle(ID_1,mem_bus_1);
    
      ----------------------------------------------
      --Check that unit is now idle since it is
      -- after transmittion already
      ----------------------------------------------
      CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
      if (r_data(16+BS_IND)='0') then
        outcome:=false;
      end if;
    
    end if;
    
    ------------------------------------------------
    -- Here we check for error state of Node 1
    -- and set it to error active. Erase the
    -- error counter. We need node 1 to be error
    -- active since when transmittion is aborted
    -- in last bit of crc field then Unit 2 sends
    -- ack and unit 1 ddetects it as SOF and then
    -- starts sending error frame. THis needs to be 
    -- active error frame otherwise unit 2 will never
    -- hook up by error frame and test will stuck
    -- in infinite loop!
    ------------------------------------------------
    r_data :=(OTHERS => '0');
    r_data(10 downto 9) := "11";
    CAN_write(r_data,ERROR_COUNTERS_ADR,ID_1,mem_bus_1);
  
  end procedure;
  
end package body;
