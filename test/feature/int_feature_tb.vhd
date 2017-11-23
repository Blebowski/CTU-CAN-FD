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
--    27.6.2016   Created file
--
-----------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Interrupt generation feature test
--
--  Test sequence is like so:
--    1. 
--    2. 
--    3. 
--    4. 
--    5. 
--                                      
-----------------------------------------------------------------------------------------------------------------


package interrupt_feature is
  
  procedure interrupt_feature_exec(
    variable   outcome      : inout boolean;
    signal      rand_ctr        :inout  natural range 0 to RAND_POOL_SIZE;
    signal      mem_bus_1       :inout  Avalon_mem_type;
    signal      mem_bus_2       :inout  Avalon_mem_type;
    signal      int_1           :in     std_logic;
    signal      int_2           :in     std_logic;
    --Additional signals for tests
    --Pretty much everything can be read out of stat bus...
    signal      bus_level       :in     std_logic;
    signal      drv_bus_1       :in     std_logic_vector(1023 downto 0);
    signal      drv_bus_2       :in     std_logic_vector(1023 downto 0);
    signal      stat_bus_1      :in     std_logic_vector(511 downto 0);
    signal      stat_bus_2      :in     std_logic_vector(511 downto 0) 
  );
  
end package;


package body interrupt_feature is
  
   procedure interrupt_feature_exec(
    variable   outcome      : inout boolean;
    signal      rand_ctr        :inout  natural range 0 to RAND_POOL_SIZE;
    signal      mem_bus_1       :inout  Avalon_mem_type;
    signal      mem_bus_2       :inout  Avalon_mem_type;
    signal      int_1           :in     std_logic;
    signal      int_2           :in     std_logic;
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
  variable size_of_buf          :     natural;
  variable ctr_1                :     natural;
  variable ctr_2                :     natural;
  variable ID_1           	     :     natural:=1;
  variable ID_2           	     :     natural:=2;
  variable vect_1               :     std_logic_vector(31 downto 0);
  variable vect_2               :     std_logic_vector(31 downto 0);
  variable mode_prev            :     std_logic_vector(31 downto 0);
  variable mode_prev_2          :     std_logic_vector(31 downto 0);
  begin
    outcome:=true;
    
    ------------
    --Part 1
    ------------
    -----------------------------------------------
    -- Recieve INT node 1, TX int node 2
    -----------------------------------------------
    report "Starting TX RX interrupt";
    r_data :=(OTHERS => '0');
    r_data(16):='1';
    CAN_write(r_data,INTERRUPT_REG_ADR,ID_1,mem_bus_1);
    r_data(16):='0';
    r_data(17):='1';
    CAN_write(r_data,INTERRUPT_REG_ADR,ID_2,mem_bus_2);
    
    -----------------------------------------------
    -- Send by node 2
    -----------------------------------------------
    CAN_generate_frame(rand_ctr,CAN_frame);
    CAN_send_frame(CAN_frame,1,ID_2,mem_bus_2,frame_sent);
    
    wait until rising_edge(int_1) or rising_edge(int_2);
    wait until rising_edge(int_1) or rising_edge(int_2);
    
    CAN_wait_frame_sent(ID_2,mem_bus_2);
    
    CAN_read(vect_1,INTERRUPT_REG_ADR,ID_1,mem_bus_1);
    CAN_read(vect_2,INTERRUPT_REG_ADR,ID_2,mem_bus_2);
    
    if(vect_1(0)='0')then
      outcome:=false;
    end if;
    
    if(vect_2(1)='0')then
      outcome:=false;
    end if;
  
  
    ------------
    --Part 2
    ------------
    -----------------------------------------------
    -- Error Interrupt both nodes
    -----------------------------------------------
    report "Starting Error interrupt";
    r_data :=(OTHERS => '0');
    r_data(23):='1';
    CAN_write(r_data,INTERRUPT_REG_ADR,ID_1,mem_bus_1);
    CAN_write(r_data,INTERRUPT_REG_ADR,ID_2,mem_bus_2);
    
    -----------------------------------------------
    --Send conflicting frames
    -----------------------------------------------
    CAN_frame.data(511 downto 480) := x"ABCDABCD";
    CAN_frame.rtr:='0';
    CAN_send_frame(CAN_frame,1,ID_2,mem_bus_2,frame_sent);
    CAN_frame.data(511 downto 480) := x"AAAABBBB";
    CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent);
    
    wait until rising_edge(int_1) or rising_edge(int_2);
    wait until rising_edge(int_1) or rising_edge(int_2);
    
    -----------------------------------------------
    -- Detect interrupt error flag
    -----------------------------------------------
    CAN_read(vect_1,INTERRUPT_REG_ADR,ID_1,mem_bus_1);
    CAN_read(vect_2,INTERRUPT_REG_ADR,ID_2,mem_bus_2);
    
    if(vect_1(7)='0')then
      outcome:=false;
    end if;
    
    if(vect_2(7)='0')then
      outcome:=false;
    end if;
    CAN_wait_frame_sent(ID_1,mem_bus_1);
    wait for 15000 ns;
    
    ------------
    --Part 3
    ------------
    -----------------------------------------------
    -- Data overrun interrupt and recieve buffer
    -- full interrupt node 2
    -----------------------------------------------
    report "Starting Data overrun recieve buffer interrupt";
    r_data :=(OTHERS => '0');
    r_data(19):='1';
    r_data(25):='1';
    CAN_write(r_data,INTERRUPT_REG_ADR,ID_1,mem_bus_1);
    CAN_write(r_data,INTERRUPT_REG_ADR,ID_2,mem_bus_2);
    
    --Release the recieve buffer
    CAN_read(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    r_data(10):='1';
    CAN_write(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    
    --Size of buffer 2
    -- Note that size of RTR is 4 thus each synthesizable
    -- size of buffer is multiple of 4!
    CAN_read(r_data,RX_INFO_2_ADR,ID_2,mem_bus_2);
    size_of_buf:= to_integer(unsigned(r_data(7 downto 0)));
    
    --Send RTR frames since it has fixed length...
    -- We can fill buffer in short time
    CAN_frame.rtr:='1';
    CAN_frame.frame_format:='0';
    for i in 0 to (size_of_buf/4)+1 loop
      CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent);
      
      if(i<size_of_buf/4)then
        CAN_wait_frame_sent(ID_1,mem_bus_1);
      else
         wait until rising_edge(int_2);
      end if;
    end loop;
    CAN_wait_frame_sent(ID_1,mem_bus_1);
    
    -----------------------------------------------
    -- Detect the data overrun interrupt flag
    -- and recieve buffer full flag
    -----------------------------------------------
    CAN_read(vect_2,INTERRUPT_REG_ADR,ID_2,mem_bus_2);
    
    if(vect_2(3)='0' or vect_2(9)='0')then
      outcome:=false;
    end if;
    wait for 30000 ns;
        

    ------------
    --Part 4
    ------------
    -----------------------------------------------
    -- Bit rate shift interrupt on both nodes
    -----------------------------------------------
    report "Starting Bit rate shift interrupt";
    r_data :=(OTHERS => '0');
    r_data(26):='1';
    CAN_write(r_data,INTERRUPT_REG_ADR,ID_1,mem_bus_1);
    CAN_write(r_data,INTERRUPT_REG_ADR,ID_2,mem_bus_2);
    
    CAN_frame.frame_format:='1';
    CAN_frame.rtr:='0';
    CAN_frame.brs:='1';
    CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent);
    
    -----------------------------------------------
    -- Wait on bit rate shift twice
    -----------------------------------------------
    wait until rising_edge(int_1);
    wait until rising_edge(int_1);
    
    -----------------------------------------------
    -- Detect the Bit rate shift interrupt flag
    -----------------------------------------------
    CAN_read(vect_2,INTERRUPT_REG_ADR,ID_2,mem_bus_2);
    
    if(vect_2(10)='0')then
      outcome:=false;
    end if;
    
    CAN_wait_frame_sent(ID_2,mem_bus_2);
    
    ------------
    --Part 5
    ------------
    -----------------------------------------------
    -- Arbitration lost interrupt in node 1
    -----------------------------------------------
    report "Starting ALC interrupt";
    r_data :=(OTHERS => '0');
    r_data(22):='1';
    CAN_write(r_data,INTERRUPT_REG_ADR,ID_1,mem_bus_1);
    
    
    -----------------------------------------------
    -- Send frames by both nodes, assuming node 1
    -- loses!
    -----------------------------------------------
    CAN_frame.frame_format:='0';
    CAN_frame.rtr:='0';
    CAN_frame.brs:='0';
    CAN_frame.ident_type:='1';
    CAN_frame.identifier:=5;
    CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent);
    CAN_frame.identifier:=4;
    CAN_send_frame(CAN_frame,2,ID_2,mem_bus_2,frame_sent);
    
    wait until rising_edge(int_1);
    
    CAN_read(vect_2,INTERRUPT_REG_ADR,ID_1,mem_bus_1);
    
    if(vect_2(6)='0')then
      outcome:=false;
    end if;
    
    --Note that here it is important to wait on Node 1!
    -- Node 2 wins the arbitration and transmitts the frame!
    -- If is error passive then it goes to suspend transmittion
    -- field, and during suspend transmittion it samples the first
    -- bit of another frame by Node 1 (the one which lost first arbit.)
    -- Thus node 2 can never go to Bus idle, and thus procedure 
    -- CAN_wait_frame_send never ends and test ends in infinite loop!
    CAN_wait_frame_sent(ID_1,mem_bus_1);
    CAN_wait_frame_sent(ID_1,mem_bus_1);
    
    ------------
    --Part 6
    ------------
    -------------------------------------------------
    -- Error warning limit and error passive enabled
    -- in node 1
    -------------------------------------------------
    report "Starting Error warning limit interrupt";
    r_data :=(OTHERS => '0');
    r_data(21):='1';
    r_data(18):='1';
    CAN_write(r_data,INTERRUPT_REG_ADR,ID_1,mem_bus_1);
    
    ----------------------------------
    -- Set the EWL, ERP to 5  and 10
    -- and erase the error counters
    ----------------------------------
    r_data :=(OTHERS => '0');
    r_data(10):='1';
    r_data(9):='1';
    CAN_write(r_data,ERROR_COUNTERS_ADR,ID_1,mem_bus_1);
    r_data :=(OTHERS => '0');
    r_data(7 downto 0)  := "00000101";
    r_data(15 downto 8) := "00001010";
    CAN_write(r_data,ERROR_TH_ADR,ID_1,mem_bus_1);
    
    ----------------------------------
    -- Store previous value of MODE
    -- register
    ----------------------------------
    CAN_read(mode_prev,MODE_REG_ADR,ID_1,mem_bus_1);
    r_data:=mode_prev;
    
    --Disable the retransmitt limit in node 1
    r_data(24):='0';
    CAN_write(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    
    --Forbid the acknowledge in node 2
    CAN_read(mode_prev_2,MODE_REG_ADR,ID_2,mem_bus_2);
    r_data:=mode_prev_2;
    r_data(7):='1';
    CAN_write(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    
    --Insert frame to be transmitted 
    CAN_send_frame(CAN_frame,1,ID_1,mem_bus_1,frame_sent);
      
    wait until rising_edge(int_1);
    
    -----------------------------------------------
    -- Detect the EWL interrupt flag
    -----------------------------------------------
    CAN_read(vect_2,INTERRUPT_REG_ADR,ID_1,mem_bus_1);
    if(vect_2(2)='0')then
      outcome:=false;
    end if;
    CAN_wait_frame_sent(ID_1,mem_bus_1);
      
    --Wait except the last frame
    wait until rising_edge(int_1);
   
    -----------------------------------------------
    -- Detect the ERP interrupt flag
    -----------------------------------------------
    CAN_read(vect_2,INTERRUPT_REG_ADR,ID_1,mem_bus_1);
    if(vect_2(5)='0')then
      outcome:=false;
    end if;
    
    -----------------------------------------------
    -- Now abort the frame transmittion
    -----------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    r_data(9):='1';
    CAN_write(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    
    ------------------------------------------------
    -- Write previous MODE values
    -------------------------------------------------
    CAN_write(mode_prev,MODE_REG_ADR,ID_1,mem_bus_1);
    CAN_write(mode_prev_2,MODE_REG_ADR,ID_2,mem_bus_2);
    
    report "Finished interrupt test";
    wait for 750000 ns;
    
  end procedure;
  
end package body;
