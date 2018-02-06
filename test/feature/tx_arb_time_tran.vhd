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
--  TX arbitration and time transmittion feature test
-- 
--  Test sequence is like so:
--    Part 1:
--      1. Measure timestamp from status bus
--      2. Insert frame to be transmitted from actual time further by random interval
--      3. Wait until frame is started to be transmitted
--      4. Check whether difference between actual timestamp and time when frame should have been transmitted
--         is less than 150. Note that timestamp is in feature environment increased every clock cycle.
--         One bit time in default configuration has 130 clock cycles. Thus if we insert the frame in begining
--          of bit time in takes nearly whole bit time until its transmittion is started.
--      5. Repeat steps 1-4 but use Buffer 2 for transmittion
--    Part 2:
--      1. Insert two frames into Buffer 1 and 2 with different transmittion times, but higher than actual time
--      2. Wait until frame transmittion starts.
--      3. Check whether the buffer which contained the frame with lower transmitt time is empty, that indicates
--         that this frame is now correctly transmitted
--      4. Wait until actual and also second frame are sucesfully transmitted.
--    Part 3:
--      1. Insert two frames into Buffer 1 and 2 with equal transmittion times, but different identifiers
--      2. Wait until frame transmittion starts.
--      3. Check whether the buffer which contained the frame with lower identifier time is empty, that indicates
--         that this frame is now correctly transmitted!
--      4. Wait until actual and also second frame are sucesfully transmitted.
--
-----------------------------------------------------------------------------------------------------------------
-- Revision History:
--    23.6.2016   Created file
--    06.02.2018  Modified to work with the IP-XACT generated memory map
-----------------------------------------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;

use work.CAN_FD_register_map.all;

package tx_arb_time_tran_feature is
  
  procedure tx_arb_time_tran_feature_exec(
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


package body tx_arb_time_tran_feature is
  
  procedure tx_arb_time_tran_feature_exec(
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
  variable CAN_frame_2          :     SW_CAN_frame_type;
  variable frame_sent           :     boolean:=false;
  variable act_ts               :     std_logic_vector(63 downto 0);
  variable rand_value           :     real:=0.0;
  variable rand_value_2         :     real:=0.0;
  variable aux1                 :     natural;
  variable aux2                 :     natural;
  begin
    outcome:= true;
    -------------------------------------------
    -- TODO: Allow both buffers
    -------------------------------------------
    
    
    ----------
    -- Part 1
    ----------
    -------------------------------------------
    --Measure timestamp and generate frame
    -------------------------------------------
    CAN_generate_frame(rand_ctr,CAN_frame); 
    CAN_generate_frame(rand_ctr,CAN_frame_2);   
    act_ts := stat_bus_1(STAT_TS_HIGH downto STAT_TS_LOW);
    
    -------------------------------------------
    -- Add random value 
    -------------------------------------------
    rand_real_v(rand_ctr,rand_value);
    --Here we assume this test will  use only lowest 32 bits!
    CAN_frame.timestamp(63 downto 32) := act_ts(63 downto 32);
    CAN_frame.timestamp(31 downto 0)  :=  std_logic_vector(unsigned(act_ts(31 downto 0))+30+integer(rand_value*5000.0));
    
    -------------------------------------------
    -- Send frame and check when TX started
    -------------------------------------------
    CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent);
    
    CAN_read(r_data,MODE_ADR,ID_1,mem_bus_1);
     while (r_data(TS_IND)='0') loop
       CAN_read(r_data,MODE_ADR,ID_1,mem_bus_1);
     end loop;
    
    aux1:=to_integer(unsigned(stat_bus_1(STAT_TS_HIGH-32 downto STAT_TS_LOW)));
    aux2:=to_integer(unsigned(CAN_frame.timestamp(31 downto 0)));
    --We tolerate up to 150 clock cycles between actual timestamp and transmitt time
    -- This fits to the default setting of up to 130 clock cycles per bit time!
    if(aux1-aux2>150)then
      outcome:=false;
    end if;
    
    CAN_wait_bus_idle(ID_1,mem_bus_1);
    
    ------------------------------
    --Do  the same with buffer 2
    ------------------------------
    act_ts := stat_bus_1(STAT_TS_HIGH downto STAT_TS_LOW);
    
    -------------------------------------------
    -- Add random value 
    -------------------------------------------
    rand_real_v(rand_ctr,rand_value);
    --Here we assume this test will  use only lowest 32 bits!
    CAN_frame.timestamp(63 downto 32) := act_ts(63 downto 32);
    CAN_frame.timestamp(31 downto 0)  :=  std_logic_vector(unsigned(act_ts(31 downto 0))+30+integer(rand_value*5000.0));
    
    -------------------------------------------
    -- Send frame and check when TX started
    -------------------------------------------
    CAN_send_frame(CAN_frame,2,ID_1,mem_bus_1,frame_sent);
    
    CAN_read(r_data,MODE_ADR,ID_1,mem_bus_1);
     while (r_data(TS_IND)='0') loop
       CAN_read(r_data,MODE_ADR,ID_1,mem_bus_1);
     end loop;
    
    aux1:=to_integer(unsigned(stat_bus_1(STAT_TS_HIGH-32 downto STAT_TS_LOW)));
    aux2:=to_integer(unsigned(CAN_frame.timestamp(31 downto 0)));
    --We tolerate up to 150 clock cycles between actual timestamp and transmitt time
    -- This fits to the default setting of up to 130 clock cycles per bit time!
    if(aux1-aux2>150)then
      outcome:=false;
    end if;
    
    CAN_wait_bus_idle(ID_1,mem_bus_1);
    
     ----------
     -- Part 2
     ----------
     CAN_frame.timestamp:=(OTHERS=>'0');
     
    
     -------------------------------------------
     -- Check the actual timestamp
     -- and generate the frames to be transmitted
     -- at slightly different times
     -- Insert them into different buffers
     -------------------------------------------
     act_ts := stat_bus_1(STAT_TS_HIGH downto STAT_TS_LOW);
     rand_real_v(rand_ctr,rand_value);
     rand_real_v(rand_ctr,rand_value_2);
     if(rand_value=rand_value_2)then
       rand_value:=rand_value+0.1;
     end if;
     
     --Buffer1 
     --Here we assume this test will  use only lowest 32 bits!
     --report "Inserting frame to buffer 1";
     CAN_frame.timestamp(63 downto 32) := act_ts(63 downto 32);
     CAN_frame.timestamp(31 downto 0)  :=  std_logic_vector(unsigned(act_ts(31 downto 0))+500+integer(rand_value*5000.0));    
     CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent);
     
     wait for 100 ns;
     
     --Buffer 2
     --report "Inserting frame to buffer 2";
     CAN_frame.timestamp(63 downto 32) := act_ts(63 downto 32);
     CAN_frame.timestamp(31 downto 0)  :=  std_logic_vector(unsigned(act_ts(31 downto 0))+500+integer(rand_value_2*5000.0));    
     CAN_send_frame(CAN_frame,2,ID_1,mem_bus_1,frame_sent);
     
    -------------------------------------------
    --  Check when TX started
    ------------------------------------------
    --report "Waiting until TX Starts";
    CAN_read(r_data,MODE_ADR,ID_1,mem_bus_1);
     while (r_data(TS_IND)='0') loop
       CAN_read(r_data,MODE_ADR,ID_1,mem_bus_1);
     end loop;
     
    -------------------------------------------
    --  Check that frame from correct buffer
    -- was taken as first
    ------------------------------------------
    --report "Checking the data";
     CAN_read(r_data,TX_STATUS_ADR,ID_1,mem_bus_1);
     if(rand_value<rand_value_2)then
       if(r_data(TXT1E_IND)='0' or r_data(TXT2E_IND)='1')then
         outcome:=false;
       end if;
     else
       if(r_data(TXT2E_IND)='0' or r_data(TXT1E_IND)='1')then
         outcome:=false;
       end if; 
     end if;
     
     -------------------------------------------------
     -- Now we wait until both frames are transmitted
     -------------------------------------------------
     CAN_wait_frame_sent(ID_1,mem_bus_1);
     CAN_wait_frame_sent(ID_1,mem_bus_1);
     
     
     ----------
     -- Part 3
     ----------
          
     -------------------------------------------
     -- Check the actual timestamp
     -- and generate the frames to be transmitted
     -- at slightly equal times but not immediately
     -------------------------------------------
     act_ts := stat_bus_1(STAT_TS_HIGH downto STAT_TS_LOW);
     rand_real_v(rand_ctr,rand_value);
     
     --Buffer1 
     --Here we assume this test will  use only lowest 32 bits!
     CAN_frame.timestamp(63 downto 32) := act_ts(63 downto 32);
     CAN_frame.timestamp(31 downto 0)  :=  std_logic_vector(unsigned(act_ts(31 downto 0))+500+integer(rand_value*5000.0));    
     CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent);
     
     wait for 100 ns;
     
     --Buffer 2
     CAN_frame_2.timestamp(63 downto 32) := act_ts(63 downto 32);
     CAN_frame_2.timestamp(31 downto 0)  :=  std_logic_vector(unsigned(act_ts(31 downto 0))+500+integer(rand_value*5000.0));    
     CAN_send_frame(CAN_frame_2,2,ID_1,mem_bus_1,frame_sent);
     
     -------------------------------------------
     --  Check when TX started
     ------------------------------------------
     --report "Waiting until TX Starts";
     CAN_read(r_data,MODE_ADR,ID_1,mem_bus_1);
     while (r_data(TS_IND)='0') loop
       CAN_read(r_data,MODE_ADR,ID_1,mem_bus_1);
     end loop;
     
     
     -------------------------------------------
     --  Check that frame from correct buffer
     -- was taken as first, the one with lower
     -- identifier!
     ------------------------------------------
     CAN_read(r_data,TX_STATUS_ADR,ID_1,mem_bus_1);
     if(CAN_frame.identifier<=CAN_frame_2.identifier)then
       if(r_data(TXT1E_IND)='0' or r_data(TXT2E_IND)='1')then
         outcome:=false;
       end if;
     else
       if(r_data(TXT2E_IND)='0' or r_data(TXT1E_IND)='1')then
         outcome:=false;
       end if; 
     end if;
     
     -------------------------------------------------
     -- Now we wait until both frames are transmitted
     -------------------------------------------------
     CAN_wait_frame_sent(ID_1,mem_bus_1);
     CAN_wait_frame_sent(ID_1,mem_bus_1);
     
     
  end procedure;
  
end package body;
