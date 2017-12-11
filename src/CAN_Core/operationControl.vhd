Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;

--------------------------------------------------------------------------------
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
--    June 2015  Created file
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Purpose:
--  Operation control state machine, handling whenever unit is Transciever, 
--  Reciever, Bus is idle or Integrating. Simple logic implemented for integra-
--  ting and possible to set machine into transciever or reciever state by 
--  set_transciever, set_reciever signals. (in start of frame, lost of 
--  arbitration)
--------------------------------------------------------------------------------

entity operationControl is
  PORT(
      -------------------
      --Clock and reset--
      -------------------
      signal clk_sys              :in   std_logic; 
      signal res_n                :in   std_logic;
    
      --Driving bus
      signal drv_bus              :in   std_logic_vector(1023 downto 0);
      
      --Driving signals
      signal arbitration_lost     :in   std_logic;
      signal PC_State             :in   protocol_type;
      signal tran_data_valid_in   :in   std_logic;
      
      --Set OP_State FSM into transciever state (Used at SOF)
      signal set_transciever      :in   std_logic; 
      
       --Set OP_State FSM into reciever state
      signal set_reciever         :in   std_logic;
      
      signal is_idle              :in   std_logic; --Unit is idle
    
      --Bit time triggering signals
      signal tran_trig            :in   std_logic;
      signal rec_trig             :in   std_logic;
      
      signal data_rx              :in   std_logic;
      
      --Status outputs
      signal OP_State             :out  oper_mode_type
    );
    ----------------------
    --Internal registers--
    ----------------------
    signal OP_State_r:oper_mode_type; --Operational mode
    
    --Counter to INTEGRATING_DURATION, to switch from integrating to bus idle
    signal integ_counter:natural range 0 to 11; 
    signal drv_ena:std_logic;
    
end entity;


architecture rtl of operationControl is
begin
  
  --Registers to output propagation
  OP_State                          <=  OP_State_r;
  drv_ena                           <=  drv_bus(DRV_ENA_INDEX);
  
  OP_proc:process(clk_sys,res_n)
  begin
    if(res_n=ACT_RESET)then
      OP_State_r                    <=  integrating;
      integ_counter                 <=  1;
    elsif rising_edge(clk_sys)then
      --Presetting the registers to avoid latches
      OP_State_r                    <=  OP_State_r;
      integ_counter                 <=  integ_counter;
      
      if(set_transciever='1')then
        OP_State_r                  <=  transciever;
      elsif(set_reciever='1')then
        OP_State_r                  <=  reciever;
      else
        case OP_State_r is
          when integrating =>
              if(drv_ena=ENABLED)then
                if(rec_trig='1')then
                  
                  --Counting up the integration period
                  if(integ_counter=INTEGRATING_DURATION)then
                    OP_State_r      <=  idle;
                    integ_counter   <=  1;
                  else
                    
                    if(data_rx=RECESSIVE)then 
                      integ_counter <=  integ_counter+1;
                    else
                      integ_counter <=  1;
                    end if;
                    
                  end if;
                  
                 end if;
               else
                 integ_counter      <=  1;
                 OP_State_r         <=  OP_State_r;
               end if;
          
          when idle =>
               if(is_idle='0')then
                if(tran_trig='1' and tran_data_valid_in='1')then
                  OP_State_r        <=  transciever;    
                elsif(rec_trig='1' and data_rx=DOMINANT)then
                  OP_State_r        <=  reciever;
                end if;
               end if;
               
          when transciever =>
                if(arbitration_lost='1')then
                  OP_State_r        <=  reciever;
                elsif(is_idle='1')then
                  OP_State_r        <=  idle;
                end if;
                
          when reciever =>
                if(is_idle='1')then
                  OP_State_r        <=  idle;
                end if;              
          when others =>--TODO:Error
        end case;
     end if;
    end if;
  end process;
  
end architecture;
