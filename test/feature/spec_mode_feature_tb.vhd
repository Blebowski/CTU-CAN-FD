Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;
USE work.CANtestLib.All;
USE work.randomLib.All;

--------------------------------------------------------------------------------
--
-- CAN with Flexible Data-Rate IP Core 
--
-- Copyright (C) 2015 Ondrej Ille <ondrej.ille@gmail.com>
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy 
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is 
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in 
-- all copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
-- AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
-- FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
-- IN THE SOFTWARE.
--
-- The CAN protocol is developed by Robert Bosch GmbH and protected by patents. 
-- Anybody who wants to implement this IP core on silicon has to obtain a CAN 
-- protocol license from Bosch.
--
-- Revision History:
--
--    24.6.2016   Created file
-----------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Special modes feature testbench
-- 
--  Test sequence is like so:
--   Part 1:
--    1. Set Self test in mode 1 and 2, Set acknowledge forbidden in node 2
--    2. Transmitt frame and wait until ack_field starts
--    3. 
--    4. 
--    5. 
--    6. 
--
-----------------------------------------------------------------------------------------------------------------


package spec_mode_feature is
  
  procedure spec_mode_feature_exec(
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


package body spec_mode_feature is
  
  procedure spec_mode_feature_exec(
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
    
    -----------------
    -- Part 1
    -----------------
    ---------------------------------------
    -- Set STM in node 1 and STM,ACF in node 2
    ---------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    r_data(2) := '1';  --Self test mode bit
    CAN_write(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    
    CAN_read(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    r_data(7) := '1';  --Acknowledge forbidden
    r_data(2) := '1';  --Self test mode bit
    CAN_write(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    
    --------------------------------------------
    -- Check the TX RX counters
    --------------------------------------------
    CAN_read(r_data,TX_COUNTER_ADR,ID_1,mem_bus_1);
    tx_ctr:= to_integer(unsigned(r_data));
    CAN_read(r_data,RX_COUNTER_ADR,ID_2,mem_bus_2);
    rx_ctr:= to_integer(unsigned(r_data));
    
    --------------------------------------------
    -- Send frame by node 1
    --------------------------------------------
    CAN_generate_frame(rand_ctr,CAN_frame);
    CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent);
    
    -----------------------------------------------
    -- Wait until one of the nodes is in ack field
    -- plus feq more clock cycles since after
    -- CRC we are in ack_delim immediately, thus
    -- bus level can still be last bit of CRC which
    -- can be dominant!
    -----------------------------------------------
    while(protocol_type'VAL(to_integer(unsigned(stat_bus_2(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW)))) /= delim_ack ) loop
      wait until rising_edge(mem_bus_1.clk_sys);
    end loop;
    if(bus_level=DOMINANT)then
      wait until rising_edge(bus_level);
    end if;
    
    -----------------------------------------------
    -- Now monitor the bus level to see if it is
    --  recessive during whole acknowledge field
    -- Monitor always on reciever! IN FD transciever
    -- workaround is used for state switching in
    -- TX trigger just slightly delayed!!!
    -----------------------------------------------
    while(protocol_type'VAL(to_integer(unsigned(stat_bus_2(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW)))) = delim_ack ) loop
      wait until rising_edge(mem_bus_1.clk_sys);
      if(bus_level=DOMINANT)then
        outcome:=false;
      end if;
    end loop;
    
    CAN_wait_bus_idle(ID_1,mem_bus_1);
    
    --------------------------------------------
    -- Check the TX RX counters
    --------------------------------------------
    CAN_read(r_data,TX_COUNTER_ADR,ID_1,mem_bus_1);
    tx_ctr_2:= to_integer(unsigned(r_data));
    CAN_read(r_data,RX_COUNTER_ADR,ID_2,mem_bus_2);
    rx_ctr_2:= to_integer(unsigned(r_data));
    
    if(tx_ctr+1 /= tx_ctr_2)then
      outcome:=false;
    end if;
    
    if(rx_ctr+1 /= rx_ctr_2)then
      outcome:=false;
    end if;
    
    -----------------
    -- Part 2
    -----------------
    -----------------------------------------------
    -- Set STM in node 1 and LOM mode in Node 2
    -- Thisway node 1 does not expect acknowledge
    --  and node 2 reroutes the acknowledge to
    --  itself internally so it gets the acknowledge
    --  from itself but it is not on the bus!
    -----------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    r_data(2) := '1';  --Self test mode bit
    CAN_write(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    
    CAN_read(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    r_data(7) := '0';  --Acknowledge forbidden
    r_data(1) := '1';  -- Listen only mode
    r_data(2) := '0';  --Self test mode bit
    CAN_write(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    
    --------------------------------------------
    -- Send frame by node 1
    --------------------------------------------
    CAN_generate_frame(rand_ctr,CAN_frame);
    CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent);
    
    -----------------------------------------------
    -- Wait until node 2 is in ack field
    -- Since bus is delayed we have to wait
    -- until the first rising edge on income data!
    -----------------------------------------------
    while(protocol_type'VAL(to_integer(unsigned(stat_bus_2(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW)))) /= delim_ack ) loop
      wait until rising_edge(mem_bus_1.clk_sys);
    end loop;
    if(bus_level=DOMINANT)then
      wait until rising_edge(bus_level);
    end if;
    
    
     -----------------------------------------------
    -- Now monitor the bus level to see if it is
    --  recessive during whole acknowledge field
    -----------------------------------------------
    while(protocol_type'VAL(to_integer(unsigned(stat_bus_2(STAT_PC_STATE_HIGH downto STAT_PC_STATE_LOW)))) = delim_ack ) loop
      wait until rising_edge(mem_bus_1.clk_sys);
      if(bus_level=DOMINANT)then
        report "FUCK";
        outcome:=false;
      end if;
    end loop;
    
    
    CAN_wait_bus_idle(ID_1,mem_bus_1);
    
    --------------------------------------------
    -- Check the TX RX counters
    --------------------------------------------
    CAN_read(r_data,TX_COUNTER_ADR,ID_1,mem_bus_1);
    tx_ctr_2:= to_integer(unsigned(r_data));
    CAN_read(r_data,RX_COUNTER_ADR,ID_2,mem_bus_2);
    rx_ctr_2:= to_integer(unsigned(r_data));
    
    if(tx_ctr+2 /= tx_ctr_2)then
      outcome:=false;
    end if;
    
    if(rx_ctr+2 /= rx_ctr_2)then
      outcome:=false;
    end if;
    
    
    -----------------
    -- Part 3
    -----------------
    
    --------------------------------------------
    -- Turn on the AFM
    --------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    r_data(7) := '0';  --Acknowledge forbidden
    r_data(1) := '0';  -- Listen only mode
    r_data(2) := '0';  --Self test mode bit
    r_data(3) := '1';  -- AFM
    r_data(10) := '1'; --Release recieve buffer!
    CAN_write(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    
    --------------------------------------------
    -- Configure AFM not to pass anything...
    --------------------------------------------
    CAN_read(r_data,FILTER_CONTROL_ADR,ID_2,mem_bus_2);
    r_data(15 downto 0) := (OTHERS => '0'); 
    CAN_write(r_data,FILTER_CONTROL_ADR,ID_2,mem_bus_2);
    
    --------------------------------------------
    -- Send frame by node 1
    --------------------------------------------
    CAN_generate_frame(rand_ctr,CAN_frame);
    CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent);
    CAN_wait_frame_sent(ID_1,mem_bus_1);
    
    -------------------------------------------
    -- Now check that we still dont have anything
    -- in the buffer
    ---------------------------------------------
    CAN_read(r_data,RX_INFO_1_ADR,ID_2,mem_bus_2);
    if(r_data(0) /= '1')then
      outcome:= false;
    end if;
    
    --------------------------------------------
    -- Turn off the AFM
    --------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    r_data(3) := '0';  -- AFM
    CAN_write(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    
     --------------------------------------------
    -- Send frame by node 1
    --------------------------------------------
    CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent);
    CAN_wait_frame_sent(ID_1,mem_bus_1);
    
    CAN_read(r_data,RX_INFO_1_ADR,ID_2,mem_bus_2);
    if(r_data(0) = '1')then
      outcome:= false;
    end if;
    
    wait for 5000 ns;
    
    CAN_wait_bus_idle(ID_1,mem_bus_1);
    
  end procedure;
  
end package body;
