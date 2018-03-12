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
--  Processing of the CAN RX data in sample point involves mutli-stage pipeline
--  after sampling itself.
--  First stage - bit destuffing. Second stage - Processing by Protocol control
--  and update of state signals. This involves "sp_control" which drives the
--  time quanta counter.
--  Due to this architecture it takes 3 clock cycles after sample point (of a
--  bit where bit-rate was switched) till time quantum starts to be "counted"
--  with proper value. During these clock cycles ph2 phase is counted with old
--  value and thus it reaches incorrect value.
--  To correct this, compensation of "ph2_real" is performed. It considers 
--  relations of Nominal/Data time quanta durations, switching from slower to 
--  faster bit-rate or vice versa, and duration of 3 cycles of "incorrect"
--  counting.
--
--  Note that other parts of the architecture are not affected since update of
--  all state variables happends during Information Processing Time. Time quanta
--  duration is the only state information which needs to be updated directly
--  after sample point!
--
--  Another approach is to provide extra signal from protocol control which
--  would be set at the beginning of the "switching bit" and prescaler would
--  use hard-coded value of time quanta during ph1 and ph2 bit times.
--------------------------------------------------------------------------------
-- Revision History:
--    July 2015   Created file
--
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE WORK.CANconstants.ALL;

package brs_comp_package is
  procedure brs_comp(
        signal tq_nbt : in natural range 0 to 63;
        signal tq_dbt : in natural range 0 to 63;
        signal sp_control : in std_logic_vector(1 downto 0);
        signal ph2_nbt : in natural range 0 to 63;
        signal ph2_dbt : in natural range 0 to 15;
        signal ph2_real : out integer range -63 to 63
  );
end brs_comp_package;

package body brs_comp_package is
  
  ---------------------------------
  -- Bit rate switch compensation
  ---------------------------------
  --Sample type has changed we have to modify "ph2_real" accordingly...
  -- This corresponds to moment of bit rate switching. Since the processing
  -- of switching takes 3 clock cycles (bit destuffing, protocol control,
  -- sp_control update, actual update of the ph2_real), compensation
  -- of the ph2_real is needed based on Time quanta(TQ) duration
  procedure brs_comp(
        signal tq_nbt : in natural range 0 to 63;
        signal tq_dbt : in natural range 0 to 63;
        signal sp_control : in std_logic_vector(1 downto 0);
        signal ph2_nbt : in natural range 0 to 63;
        signal ph2_dbt : in natural range 0 to 15;
        signal ph2_real : out integer range -63 to 63
  )is
  variable ntd : boolean; --Nominal to Data switch
  begin
    
    if ((sp_control = SECONDARY_SAMPLE) or (sp_control=DATA_SAMPLE)) then
       ntd := true;
    elsif (sp_control = NOMINAL_SAMPLE) then
       ntd := false;
    else
       ntd := false;
       report "Unknown sampling type" severity error;
    end if;
    
    if(ntd=true)then
      if (tq_nbt=1 and tq_dbt>3) then
        ph2_real <= ph2_dbt+3; 
      elsif ((tq_nbt=2 or tq_nbt=3) and (tq_dbt>3)) then
        ph2_real <= ph2_dbt+1;
      elsif (tq_nbt=1 and (tq_dbt=2 or tq_dbt=3)) then
        ph2_real <= ph2_dbt+2;
      elsif ((tq_nbt=2 or tq_nbt=3) and (tq_dbt=1)) then
        ph2_real <= ph2_dbt-2;
      elsif ((tq_dbt=2 or tq_dbt=3) and (tq_nbt>3)) then
        ph2_real <= ph2_dbt-1;
      elsif (tq_nbt>3 and tq_dbt=1) then
        ph2_real <= ph2_dbt-3; 
      else 
        ph2_real <= ph2_dbt;
      end if;
    else
      if (tq_nbt>3 and tq_dbt=1)then
        ph2_real <= ph2_nbt+3;
      elsif (tq_nbt>3 and (tq_dbt=2 or tq_dbt=3)) then
        ph2_real <= ph2_nbt+1;
      elsif (tq_dbt=1 and (tq_nbt=2 or tq_nbt=3)) then
        ph2_real <= ph2_nbt+1;
      elsif (tq_dbt>3 and (tq_nbt=2 or tq_nbt=3)) then
        ph2_real <= ph2_nbt-1;
      elsif (tq_nbt=1 and (tq_dbt=2 or tq_dbt=3)) then
        ph2_real <= ph2_nbt-2;
      elsif (tq_nbt=1 and tq_dbt>3) then
        ph2_real <= ph2_nbt-3;
      else
        ph2_real <= ph2_nbt;
      end if;
    end if;
  end procedure;

end package body;