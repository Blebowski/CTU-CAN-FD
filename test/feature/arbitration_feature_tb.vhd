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
--  Arbitration feature test implementation
--
--  Arbitration test generates random identifier, identifier type, frame_type, RTR frame
--   into both CAN Nodes of feature_test environment. Then it inserts both frames to be
--   transmitted immediately. Then it waits whether one of the unit turns to be reciever
--   or an error appears!
--   It checks out which unit lost the arbitration and compares with expected loser.
--   Situations like Identifier collision, Victory of Base vs extended, victory of non-rtr
--   vs RTR frame are covered in this testbench!
--
--   TODO: Check the content of ALC register
--                                       
--------------------------------------------------------------------------------
-- Revision History:
--
--    20.6.2016   Created file
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
use work.CAN_FD_frame_format.all;

package Arbitration_feature is
  
  procedure arbitration_feature_exec(
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



package body Arbitration_feature is
  
  procedure arbitration_feature_exec(
    variable    outcome         : inout boolean;
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
  variable ident1               :     natural range 0 to 536870911:=0;
  variable ident2               :     natural range 0 to 536870911:=0;
  variable rand_value           :     real;
  variable ident_type_1         :     std_logic;
  variable ident_type_2         :     std_logic;
  variable frame_type           :     std_logic;
  variable is_rtr_1             :     std_logic;
  variable is_rtr_2             :     std_logic;
  variable exp_winner           :     natural:=0; -- 0-Node 1; 1-Node 2; 2-Collision
  
  --BRS always zero in this test
  variable brs_1                :     std_logic:='0';
  variable brs_2                :     std_logic:='0';
  variable w_data               :     std_logic_vector(31 downto 0):=(OTHERS => '0');
  variable ID_1           	     :     natural:=1;
  variable ID_2           	     :     natural:=2;
  variable id_1_vect            :     std_logic_vector(28 downto 0):=(OTHERS => '0');
  variable id_2_vect            :     std_logic_vector(28 downto 0):=(OTHERS => '0');
  variable r_data               :     std_logic_vector(31 downto 0):=(OTHERS => '0');
  
  --Some unit lost the arbitration...
  variable unit_rec             :     natural:=0; -- 0 - initial , 1-Node 1 turned rec
    	                                             -- 2 - Node 2 turned rec
  begin
    outcome:= true;
    
    -----------------------------------------------
    --Generate Two random Identifiers!
    --Consider the type of identifier during its
    -- generation. Also generate RTR frame
    -----------------------------------------------
    rand_logic_v(rand_ctr,ident_type_1,0.4);
    rand_logic_v(rand_ctr,ident_type_2,0.4);
    rand_logic_v(rand_ctr,is_rtr_1,0.2);
    rand_logic_v(rand_ctr,is_rtr_2,0.2);
    rand_logic_v(rand_ctr,frame_type,0.5); --It is enough to have one frame type...
    
    rand_real_v(rand_ctr,rand_value);
    ident1 := integer(536870911.0*rand_value);
    
    rand_real_v(rand_ctr,rand_value);
    ident2 := integer(536870911.0*rand_value);
    
    ------------------------------------------------
    --Special case for collision
    -----------------------------------------------
    rand_real_v(rand_ctr,rand_value);
    if(rand_value<0.1)then
      ident1:=ident2;
      is_rtr_1:=is_rtr_2;
      
      --We have to have also Identifier types matching!
      --Otherwise BASE identifier wins over the Extended!
      ident_type_1:=ident_type_2;
    elsif(rand_value<0.2)then
      ident1:=ident2;
      is_rtr_1:=is_rtr_2;
      ident_type_1:= not ident_type_2;
    elsif(rand_value<0.3)then
      ident1:=ident2;
      is_rtr_1:= not is_rtr_2;
    end if;
    
    
    -----------------------------------------------
    -- Evaluate who should win the arbitration or
    -- if collision should appear...
    -----------------------------------------------
    if(ident1=ident2)then
      if(is_rtr_1=is_rtr_2 and ident_type_1=ident_type_2)then
        exp_winner:=2;
        report "Expecting collision";
      
       --Same RTR but different ident type
      --IDENT type sets the winner!
      elsif(ident_type_1=BASE and ident_type_2=EXTENDED)then
        report "Testing victory of BASE against EXTENDED";
        exp_winner:=0;
      elsif(ident_type_1=EXTENDED and ident_type_2=BASE)then
        report "Testing victory of BASE against EXTENDED";
        exp_winner:=1;
        
      --Same identifiers, different RTRs
      --RTR always selects the winner
      elsif(is_rtr_1=NO_RTR_FRAME and is_rtr_2=RTR_FRAME)then
        report "Testing victory of non RTR against RTR";
        exp_winner:=0;
      elsif(is_rtr_1=RTR_FRAME and is_rtr_2=NO_RTR_FRAME)then
        report "Testing victory of non RTR against RTR";
        exp_winner:=1;
      end if;    
      
    elsif (ident1>ident2)then
      exp_winner:=1;
    elsif (ident2>ident1)then
      exp_winner:=0;
    end if;
    
    if(is_rtr_1=RTR_FRAME or is_rtr_2=RTR_FRAME)then
      frame_type:=NORMAL_CAN;    --FD Frames have no RTR frames...
    end if;
    
    -----------------------------------------------
    -- Insert the frames to transmitt
    --  have the data different so that we can
    --  observe collision!
    --
    --  We put DLC fixed to 4 and 8 in these cases!
    -----------------------------------------------
    --Frame format word
    w_data := (OTHERS => '0');
    w_data(DLC_H downto DLC_L) := "1000";
    w_data(RTR_IND) := is_rtr_1;
    w_data(ID_TYPE_IND) := ident_type_1;
    w_data(FR_TYPE_IND) := frame_type;
    w_data(TBF_IND) := '1';
    w_data(BRS_IND) := brs_1;
    CAN_write(w_data,TX_DATA_1_ADR,ID_1,mem_bus_1);
    
    
    w_data := (OTHERS => '0');
    w_data(DLC_H downto DLC_L) := "0100";
    w_data(RTR_IND) := is_rtr_2;
    w_data(ID_TYPE_IND) := ident_type_2;
    w_data(FR_TYPE_IND) := frame_type;
    w_data(TBF_IND) := '1';
    w_data(BRS_IND) := brs_2;
    CAN_write(w_data,TX_DATA_1_ADR,ID_2,mem_bus_2);
    
    --Identifier word
    id_1_vect := std_logic_vector(to_unsigned(ident1,29));
    id_2_vect := std_logic_vector(to_unsigned(ident2,29));
    
    w_data := (OTHERS => '0');
    w_data(IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L) := 
           id_1_vect(28 downto 18);
           
    if (ident_type_1 = EXTENDED) then
      w_data(IDENTIFIER_EXT_H downto IDENTIFIER_EXT_L) := 
           id_1_vect(17 downto 0);
    end if;
    CAN_write(w_data,TX_DATA_4_ADR,ID_1,mem_bus_1);
    

    w_data := (OTHERS => '0');
    w_data(IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L) := 
           id_2_vect(28 downto 18);
  
    if (ident_type_2=EXTENDED)then --Basic
      w_data(IDENTIFIER_EXT_H downto IDENTIFIER_EXT_L) := 
           id_2_vect(17 downto 0);
    end if;    
    CAN_write(w_data,TX_DATA_4_ADR,ID_2,mem_bus_2);
    
    --Timestamp words
    w_data      := (OTHERS => '0');
    CAN_write(w_data,TX_DATA_2_ADR,ID_1,mem_bus_1);
    CAN_write(w_data,TX_DATA_3_ADR,ID_1,mem_bus_1);
    CAN_write(w_data,TX_DATA_2_ADR,ID_2,mem_bus_2);
    CAN_write(w_data,TX_DATA_3_ADR,ID_2,mem_bus_2);
    
    --Data words
    w_data      := x"ABCDABCD";
    CAN_write(w_data,TX_DATA_5_ADR,ID_1,mem_bus_1);
    CAN_write(w_data,TX_DATA_6_ADR,ID_1,mem_bus_1);
    w_data      := x"55555555";
    CAN_write(w_data,TX_DATA_5_ADR,ID_1,mem_bus_1);

    -----------------------------------------------
    -- Commit the FRAMES for transmittion into
    --  TXT Buffer 1 both!
    -----------------------------------------------
    -- TODO: rethink for new buffer implementation
    w_data      := "00000000000000000000000000001010";
    CAN_write(w_data,TX_SETTINGS_ADR,ID_1,mem_bus_1);
    CAN_write(w_data,TX_SETTINGS_ADR,ID_2,mem_bus_2);
    
    -------------------------------------------------
    -- Now we have to wait until both units starts to
    -- transmitt!!!
    -------------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    while r_data(TS_IND)='0' loop
      CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
      wait for 10 ns;
    end loop;
    
    CAN_read(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
    while r_data(TS_IND)='0' loop
      CAN_read(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
      wait for 10 ns;
    end loop;
    
    -----------------------------------------------
    -- Iterate as long as one of the units turns
    -- to be reciever, or bus is idle.
    -----------------------------------------------
    while unit_rec=0 loop
      
      --Unit 1 turned reciever
      CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
      if(r_data(RS_IND)='1')then
        unit_rec:=1;
      end if;
      
      --Unit 2 turned receiver
      CAN_read(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
      if(r_data(RS_IND)='1')then
        unit_rec:=2;
      end if;
      
      --Bus is idle in unit 2
      CAN_read(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
      if(r_data(BS_IND)='1')then
        unit_rec:=3;
      end if;
      
      --Error frame transmitted by unit 2
      CAN_read(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
      if(r_data(ET_IND)='1')then
        unit_rec:=3;
      end if;
      
      --Error frame transmitted by unit 1
      CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
      if(r_data(ET_IND)='1')then
        unit_rec:=3;
      end if;
      
    end loop;
    
    -----------------------------------------------
    -- Check whether expected winner is the unit
    -- which lost the arbitration...
    -----------------------------------------------    
    if(unit_rec=1 and exp_winner=0) or 
      (unit_rec=2 and exp_winner=1) 
    then
      outcome:=false;
    end if;
    
    -----------------------------------------------
    -- Check what is the value in the ALC register
    ----------------------------------------------- 
    if(unit_rec=1)then
      CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
      
    elsif(unit_rec=2)then
      CAN_read(r_data,MODE_REG_ADR,ID_2,mem_bus_2);
      
    end if;
    --TODO:Check the ALC value according to SJA1000!
    
    
    -----------------------------------------------
    -- If error frame is transmitted and collision
    -- should not appear...
    -----------------------------------------------  
    if(unit_rec=3 and  exp_winner /= 2)then
      outcome:=false;
    end if;
    
    -----------------------------------------------
    -- Wait definitely until the bus is idle
    -----------------------------------------------
    CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
    while r_data(BS_IND)='0' loop
      CAN_read(r_data,MODE_REG_ADR,ID_1,mem_bus_1);
      wait for 10 ns;
    end loop;
    
    wait for 300000 ns;
  end procedure;
  
end package body;

