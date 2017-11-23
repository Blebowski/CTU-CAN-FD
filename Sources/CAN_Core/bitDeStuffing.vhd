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
--    July 2015   Created file
--    19.5.2016   1. Added Stuff bit counter to cover ISO FD extra field!
--                2. Edge detection 0->1 added at fixed_stuff input. Once edge is detected
--                    same_bits counter is erased! This prevents the error of inserting stuff
--                    bit sooner than fixed length when last bit of data field have equal value!
--    6.6.2016    Added fixed stuff bit at the transition from non fixed stuff to fixed stuff! Thisway
--                bit stuffing also covers the one fixed stuff bit in the beginning of CRC phase!!
--                Added bit stuffing counter to count the dynamic stuff bits in ISO FD.
--   13.6.2016    1.Added mod 8 into same_bits counter increase
--                2.Added keeping previous value of dst_counter when circuit is disabled instead of erasing!
--                  This way ciruit is compatible with bit stuffing!
--                3.dst_bit_ctr value kept when destuffing error occured for correct behaviour in testbench.
--                  This is only a small thing since when stuff error occurs bit stuffing is turned off 
--                  immediately due to error frame transmittion and dst_ctr is not needed anymore!!! 
--                4.Added warning when bit stuffing rule is set to 0 or 1 which is invalid setting!
--    12.1.2017  Changed priority of fixed bit-destuffing processing. Fixed bit destuffing should always have
--               higher priority than non-fixed bit-destuffing and thus be before in the If-elsif condition!
--               This is due to possible conflic of normal and fixed bit destuffing in the start of FD CRC.
--               Fixed bit-destuff should win!
-------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------------------
--Purpose:
--  Bit destuffing circuit. Data sampled always with valid trig_spl_1 signal. Length of bitStuffing controlled via   
--  stuff_length input. Stuff error signalises Error when the stuff rule is not valid (stuff_lenght+1) consecutive   
--  bits of the same polarity. Signal destuffed  indicates that current output bit is not valid data bit, but is     
--  destuffed bit taken out from input data stream!                                                                  
-----------------------------------------------------------------------------------------------------------------------


entity bitDestuffing is
  PORT(
    -------------------
    --Clock And Reset--
    -------------------
    signal clk_sys              :in   std_logic; --System clock
    signal res_n                :in   std_logic; --Async Reset
     
    -------------------------
    --Bus Sampling Interface-
    -------------------------
    signal data_in              :in   std_logic; --Sampled data from busSync.vhd
    
    -----------------------
    --Prescaler interface -
    -----------------------
    signal trig_spl_1           :in   std_logic; --Triggering signal with one clk_sys delay behind the used sampling signal
    --Note: signals nbt_trig_spl_1 or dbt_trig_spl_1 can be used 
    --      depending whenever data bit time or nominal bit tim is used
    
    --------------------
    --Error Signalling--
    --------------------
    signal stuff_Error          :out  std_logic; --Stuff Error
    
    -----------------------
    --CAN Core interface --
    -----------------------
    signal data_out             :out  std_logic; --Data output for CAN Core
    signal destuffed            :out  std_logic; --Signal that data on output are not valid but it is a stuff bit
    signal enable               :in   std_logic; --Enable of the circuit
    signal stuff_Error_enable   :in   std_logic; --Enable stuff Error logging
    signal fixed_stuff          :in   std_logic; --Whenever fixed bit Destuffing method is used    
    signal length               :in   std_logic_vector(2 downto 0); --Length of bit stuffing rule
    signal dst_ctr              :out  natural range 0 to 7  --Number of destuffed bits with
                                                             --regular bit stuffing method
  );
  --Note:Bit Destuffing has no driving bus aliases
  
  ---------------------------------
  --Internal signals and registers
  ---------------------------------
  signal same_bits              :     natural range 0 to 15; --Number of equal consecutive bits
  signal prev_val               :     std_logic; --Previous value of the bit
  signal destuffed_reg          :     std_logic; --Registered value of destuffed
  signal error_reg              :     std_logic; --Registred value of destuffed
  signal enable_prev            :     std_logic; --Previous value of enable
  signal fixed_prev             :     std_logic; --Previous value of fixed stuff method
  
  --ISO CAN FD destuff bit counter
  signal dst_bit_ctr            :     natural range 0 to 7;
  --Note: Number of stuffed, destuffed bits is transmitted
  --      modulo 8. Thus only 3 bits counter is enough!!
  
end entity;

architecture rtl of bitDestuffing is
begin

  -----------------------
  --Destuffing process
  -----------------------
  destuf_proc:process(res_n,clk_sys)
  begin
  if(res_n=ACT_RESET)then
    same_bits             <=  1;
    prev_val              <=  RECESSIVE;
    fixed_prev            <=  '0';
    destuffed_reg         <=  '0';
    error_reg             <=  '0';
    
    --Bit stuff counter for ISO FD
    dst_bit_ctr           <=  0;
    enable_prev           <= '0';
    
  elsif (rising_edge(clk_sys))then
    
    --Edge detection on enable and fixed stuff
    enable_prev           <=  enable;
    dst_bit_ctr           <=  dst_bit_ctr;
    
    if((length="000" or length="001")and (enable='1'))then
      report "0 and 1 bit stuffing length is invalid" severity warning;
    end if;
    
    if(enable='1')then
        
        --When transition starts prev_val needs to be deleted! Otherwise 
        --stuff error might occur when first bits of identifier are zero
        if(enable_prev='0')then 
          prev_val        <=  RECESSIVE;
          dst_bit_ctr     <=  0;        
          fixed_prev      <=  '0';
          same_bits       <=  1;
          
        --Destuffing is processed with triggering signal
        elsif(trig_spl_1='1')then
          prev_val        <=  data_in;  --Data is always propagated
          
          --When stuffing method is changed in the beginning of the
          --CRC field the stuffing counter needs to be erased!
          if(fixed_stuff='1' and fixed_prev='0')then
            prev_val        <=  RECESSIVE;
            same_bits       <=  1;  --TODO: think if here shouldnt be zero
                                    --      due to extra inserted stuff bit!!!
            destuffed_reg   <=  '1';
            fixed_prev      <=  fixed_stuff;
            
          --If number of bits was reached then
          elsif(same_bits=unsigned(length) and fixed_stuff='0')or
            (same_bits=unsigned(length)+1 and fixed_stuff='1')then 
            --Fixed stuff is must be plus one since also the stuffed bit is counted!
            --In normal bit stuffing when bit is stuffed same_bits is erased and counted
            --from first bit after stuffed bit!
            
            destuffed_reg <=  '1';
            same_bits     <=  1;
            
            --Stuff bit occured increase the stuffing counter!
            --but only in the case of the original stuffing method
            if(fixed_stuff='0')then
              dst_bit_ctr   <= (dst_bit_ctr+1) mod 8;
            else
              dst_bit_ctr   <=  dst_bit_ctr;
            end if;
          
            --Stuff Rule violation 
            if(prev_val=data_in and stuff_Error_enable='1')then 
              error_reg   <=  '1';
              dst_bit_ctr <=  dst_bit_ctr;
            else
              error_reg   <=  '0';
            end if;
          
          --If number of bits not reached then
          --only increase number of same bits
          else
            
            if(data_in=prev_val) or (fixed_stuff='1')then
              same_bits   <=  (same_bits+1) mod 8;
            else 
              same_bits   <=  1;
            end if;
            
            --Trigger not occured keep the value of the
            --stuff counter
            dst_bit_ctr   <=  dst_bit_ctr;
            
            destuffed_reg <=  '0';
            error_reg     <=  '0';
            
          end if;
        
        --Trigger not occured keep the previous values  
        else
          same_bits       <=  same_bits;
          prev_val        <=  prev_val;
          destuffed_reg   <=  destuffed_reg;
          error_reg       <=  '0';
          dst_bit_ctr     <=  dst_bit_ctr;
        end if;
        
    else
      same_bits           <=  1;
      destuffed_reg       <=  '0';
      error_reg           <=  '0';
      dst_bit_ctr         <=  dst_bit_ctr;
      
      --Sample the data with trigger even if destuffing
      -- is disabled
      if(trig_spl_1='1')then
        prev_val          <=  data_in;
      else
        prev_val          <=  prev_val;
      end if;
      
    end if;
  end if;    
  end process;
  
  --Register propagation on output
  data_out                <=  data_in;  
  destuffed               <=  destuffed_reg;
  stuff_Error             <=  error_reg;
  dst_ctr                 <=  dst_bit_ctr;
  
end architecture;

