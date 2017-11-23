Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.std_logic_unsigned.All;
USE WORK.CANconstants.ALL;

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
--    June 2015  Created file
--    July 2015  Created second version of the bitstuffing circuit
--    19.5.2016  same_bits counter erased when edge detection on fixed_stuff detected. Avoids inserting
--               stuff bit in CRC field after less than stuff_count bits when last bits of data field
--               were equal!
--    6.6.2016   Added fixed stuff bit at the transition from non fixed stuff to fixed stuff! Thisway
--               bit stuffing also covers the one fixed stuff bit in the beginning of CRC phase!!
--               Added bit stuffing counter to count the dynamic stuff bits in ISO FD.
--    13.6.2016  Added mod 8 into same_bits counter increase
--    12.1.2017  Changed priority of fixed bit-stuffing processing. Fixed bit stuffing should always have
--               higher priority than non-fixed bit-stuffing and thus be before in the If-elsif condition!
--               This is due to possible conflic of normal and fixed bit stuffing in the start of FD CRC.
--               Fixed bit-stuff should win!
-------------------------------------------------------------------------------------------------------------

---------------------------------------------------------------------------------------------------------------
-- Purpose:
--  Simple bit stuffing circuit with HandShake protocol. When bit is stuffed transciever --
--  (CAN Core) has to stop transcieving for one bit time. data_halt output is set to logic--
--  1 when bit is stuffed.
-----------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------
--Second version of bit Stuffing circuit. Enables configurable stuff length. Operation starts when     ---
--enable='1'. Valid data already has to be on data_in then. Operates with triggering signal tran_trig_1 --
--Fixed Stuffing method can be used by setting logic on fixed_stuff input. In fixed stuff inverse bit   --
--is inserted after every stuff_length bits, even if their polarity is not equal!                       --
---------------------------------------------------------------------------------------------------------

entity bitStuffing_v2 is 
  port(
    -------------------------
    --Clock and Async reset--
    -------------------------
    signal clk_sys        :in   std_logic;
    signal res_n          :in   std_logic;
    
    ---------------------------------
    --Prescaler interface - sampling-
    ---------------------------------
    --Trigger signal for propagating the data (one clk_sys delayed behind beginning of bit time) 
    signal tran_trig_1    :in   std_logic; 
    
    -----------------------
    --CAN Core interface --
    -----------------------
    signal enable         :in   std_logic;    --Enabling the operation of the circuit
    signal data_in        :in   std_logic;    --Data Input sampled 
    signal fixed_stuff    :in   std_logic;    --Whenever fixed bit stuffing should be used (CAN FD Option)
    signal data_halt      :out  std_logic;    --Logic 1 signals stuffed bit for CAN Core. CAN Core has 
                                              --to halt the data sending for one bit-time  
    
    --Length of Bit Stuffing
    signal length         :in   std_logic_vector(2 downto 0); 
    --Note: "HandShake like" protocol between CAN Core is 
    --      easier implementation of bitStuffing than Bit-Stuffing with Buffer
    
    signal bst_ctr        :out  natural range 0 to 7; --Bit stuffing counter
    
    -------------------------
    --Bus Synchro interface--
    -------------------------
    signal data_out       :out  std_logic --Data output
    --Note: Data are sent into bus synchroniser but can be also
    --      fed back to CAN Core for CRC calculation in CAN FD Phase!
        
  );
  
  --Note: Bit Stufffing has no driving bus aliases, bit stuffing function cant be user controlled
  
  ----------------------
  --Internal Registers--
  ----------------------
  signal same_bits        :     natural range 0 to 7;   --Number of equal consequent bits
  signal prev_bit         :     std_logic;              --Value of previously transcieved bit
  signal halt_reg         :     std_logic;              --Halt for CAN Core
  signal fixed_prev       :     std_logic;              --Registered value of fixed stuffing
  signal stuff_ctr        :     natural range 0 to 7;
  signal enable_prev      :     std_logic;

end entity;


architecture rtl of bitStuffing_v2 is
begin
  
  bst_ctr <= stuff_ctr;
  
  stuff_proc:process(res_n,clk_sys)
  begin
  if(res_n=ACT_RESET)then
    same_bits           <=  1;
    prev_bit            <=  RECESSIVE;
    halt_reg            <=  '0';
    enable_prev         <=  '0';
    stuff_ctr           <=  0;
    fixed_prev          <=  '0';
  elsif rising_edge(clk_sys)then
  
   --Registering fixed stuff value for edge detection
   enable_prev  <= enable;
   stuff_ctr   <= stuff_ctr;
    
   if(enable='1')then
      
      --Start of the bit stuffing
      if(enable_prev='0')then
        prev_bit    <=  RECESSIVE;
        same_bits   <= 1;
        stuff_ctr   <= 0;
        fixed_prev  <= '0';
           
      --Trigger signal
      elsif(tran_trig_1='1')then 
        
        --When stuffing method is changed in the beginning of the
        --CRC field the stuffing counter needs to be erased!
        if(fixed_stuff='1' and fixed_prev='0')then
          prev_bit      <=  not prev_bit;
          halt_reg      <= '1';
          same_bits     <=  1;  --TODO: think if here shouldnt be zero
                                --      due to extra inserted stuff bit!!!          
          fixed_prev    <= fixed_stuff;
      
        --If number of bits was reached
        elsif(same_bits=unsigned(length) and fixed_stuff='0')  or
          (same_bits=unsigned(length)+1 and fixed_stuff='1')
          --Fixed stuff is must be plus one since also the stuffed bit is counted!
          --In normal bit stuffing when bit is stuffed same_bits is erased and counted
          --from first bit after stuffed bit!
        then 
        
          same_bits     <=  1; --Since the inverted bit also counts as bit starting value is not zero  but one
          prev_bit      <=  not prev_bit;
          halt_reg      <=  '1';
          
          --Stuff bit occured increase the stuffing counter!
          --but only in the case of the original stuffing method
          if(fixed_stuff='0')then
            stuff_ctr   <= (stuff_ctr+1) mod 8;
          else
            stuff_ctr   <=  stuff_ctr;
          end if;
            
        else --Not reached nuber of bits for stuff bit
          
          --If fixed stuffing selected or incoming bit is equal with last bit
          if(data_in=prev_bit or fixed_stuff='1')then 
            same_bits   <=  (same_bits+1) mod 8;
          else
            same_bits   <=  1;
          end if;
          
          halt_reg      <=  '0';  
          prev_bit      <=  data_in;
        end if;
      
      else
        prev_bit        <=  prev_bit;
        same_bits       <=  same_bits;
      end if; 
      
   else
     same_bits          <=  1;
     halt_reg           <=  '0';
     
     --When circuit is disabled it passes the data on trigger signal from input to output without Stuffing!
     if(tran_trig_1='1')then 
       prev_bit         <=  data_in;
     else
       prev_bit         <=  prev_bit;
     end if;
     
   end if;  
  end if;
  end process;
  
  data_out              <=  prev_bit; --Proapagating bit on output
  data_halt             <=  halt_reg; --Propagating halt value BACK to CAN Core
  
end architecture;