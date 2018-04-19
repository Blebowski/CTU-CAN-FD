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
--  CRC Checking for CAN flexible data Rate. Three CRC are calculated simulta-
--  neously. Serial Data input. Operation starts with enable transition from 0 
--  to 1. Valid input data has to be present then. Circuit processes the data on
--  trig signal in logic 1. Circuit operation finishes when 1 to 0 transition on
--  enable signal appears. The output CRC is valid then. CRC stays valid until
--  following 0 to 1 enable transition. This also erases CRC registers.
--
--  Refer to CAN 2.0 or CAN FD Specification for CRC calculation algorythm                                                   -- 
--------------------------------------------------------------------------------
-- Revision History:
--    June 2015   Created file
--    28.5.2016   Starting polynomial changed for crc 17 and crc 21. Highest bit
--                is now fixed in logic one to be compliant with CAN ISO FD. It
--                will be needed to implement both ways still since ISO and 
--                non-ISO FD will be changable via configuration bit! 
--    4.6.2016    Added drv_is_fd to cover differencce in highest bit of crc17
--                and crc21 polynomial             
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use work.CANconstants.all;
use work.CAN_FD_register_map.all;


entity canCRC is
  generic(
    constant crc15_pol :     std_logic_vector(15 downto 0):=x"C599";
    constant crc17_pol :     std_logic_vector(19 downto 0):=x"3685B";
    constant crc21_pol :     std_logic_vector(23 downto 0):=x"302899"  
  );
  port(
    ----------
    --Inputs--
    ----------
    signal data_in    :in   std_logic; --Serial data input
    signal clk_sys    :in   std_logic; --System clock input
    signal trig       :in   std_logic; --Trigger to sample the input value
        --Note: trigger for CAN FD should be 1 clk_sys behind normal CAN 
        --since for CAN FD bit stuffing is made before CRC calculation
        
    signal res_n      :in   std_logic; --Asynchronous reset
    
    signal enable     :in   std_logic; 
    --By transition from 0 to 1 on enable sampled on clk_sys rising edge 
    --(and with trig='1') operation is started. First bit of data already has 
    --to be on data_in input.
    --Circuit works as long as enable=1.
    
    signal drv_bus    :in   std_logic_vector(1023 downto 0);
    
    -----------
    --Outputs--
    -----------
    signal crc15      :out  std_logic_vector(14 downto 0);
    signal crc17      :out  std_logic_vector(16 downto 0);
    signal crc21      :out  std_logic_vector(20 downto 0)
  );    
  
  ---------------------
  --Internal registers-
  ---------------------
  signal crc15_reg    :     std_logic_vector(14 downto 0);
  signal crc17_reg    :     std_logic_vector(16 downto 0);
  signal crc21_reg    :     std_logic_vector(20 downto 0);
  
  --Holds previous value of enable input. Detects 0 to 1 transition
  signal start_reg    :     std_logic;
  
  --ISO CAN FD or NON ISO CAN FD Value
  signal drv_fd_type  :     std_logic; 
  
end entity;


architecture rtl of canCRC is
begin
  crc15               <= crc15_reg;
  crc17               <= crc17_reg;
  crc21               <= crc21_reg;
  drv_fd_type         <= drv_bus(DRV_FD_TYPE_INDEX); 
  
  ---------------------------------------------
  --Registering previous value of 
  -- enable input to detec 0 to 1 transition
  ---------------------------------------------
  start_reg_proc:process(res_n,clk_sys)
  begin
    if(res_n='0')then
      start_reg       <= '0';
    elsif rising_edge(clk_sys) then
      start_reg       <= enable;
    end if;
  end process start_reg_proc;
  
  --------------------------------------------
  -- Calculation of CRC15 value 
  --------------------------------------------
  crc15_cycle:process(res_n,clk_sys)
  variable crc15_nxt:std_logic;
  begin 
  if(res_n=ACT_RESET)then
    crc15_reg         <= (OTHERS=>'0');
    crc15_nxt         := '0';
  elsif rising_edge(clk_sys) then 
    
    --Erase the CRC value at the begining of
    --calculation
    if(start_reg='0' and enable='1')then
      crc15_reg       <= (OTHERS=>'0');
    else
      
      --Calculate the next value only when
      --triggered at some part of bit time
      if(enable='1' and trig='1') then
          crc15_nxt   := data_in xor crc15_reg(14);
          
          --The main CRC calculation
          if crc15_nxt='1'then
            --Shift and xor with polynomial
            crc15_reg <= (crc15_reg(13 downto 0)&'0') xor 
                          crc15_pol(14 downto 0); 
          else 
            crc15_reg <= crc15_reg(13 downto 0)&'0'; --Only shift bits
          end if; 
      else
        crc15_reg     <= crc15_reg;
      end if;
      
    end if;
  end if;
  end process crc15_cycle;
  
  -----------------------------------
  --Calculation of CRC17 value 
  -----------------------------------
  crc17_cycle:process(res_n,clk_sys)
  variable crc17_nxt:std_logic;
  begin 
  if(res_n='0')then
    crc17_reg         <= (OTHERS=>'0');
    crc17_reg(16)     <= '1';
    crc17_nxt         := '0';
  elsif rising_edge(clk_sys)then 
    
    --Erase the CRC value at the begining of
    --calculation
    if(start_reg='0' and enable='1')then
      crc17_reg       <= (OTHERS=>'0');
      
      if(drv_fd_type=ISO_FD)then
        crc17_reg(16)   <= '1';
      end if;

    else
      
      --Calculate the next value only when
      --triggered at some part of bit time
      if(enable='1'and trig='1')then
        crc17_nxt     := data_in xor crc17_reg(16);
          
          --The main CRC calculation
          if crc17_nxt='1'then
            --Shift and xor
            crc17_reg <= (crc17_reg(15 downto 0)&'0') xor
                          crc17_pol(16 downto 0); 
          else  
            crc17_reg <= crc17_reg(15 downto 0)&'0'; --Only shift bits
          end if;
      else
        crc17_reg     <= crc17_reg;
      end if;
      
    end if;
  end if;
  end process crc17_cycle; 
  
  ----------------------------------
  --Calculation of CRC21 value 
  ----------------------------------
  crc21_cycle:process(res_n,clk_sys)
  variable crc21_nxt:std_logic;
  begin 
  if(res_n='0')then
    crc21_reg         <= (OTHERS=>'0');
    crc21_reg(20)     <= '1';
    crc21_nxt         := '0';
  elsif rising_edge(clk_sys)then
   
   --Erase the CRC value at the begining of
   --calculation
   if(start_reg='0' and enable='1')then 
    crc21_reg         <= (OTHERS=>'0');
         
    if(drv_fd_type=ISO_FD)then
        crc21_reg(20)     <= '1';
    end if;
    
   else
     
     --Calculate the next value only when
     --triggered at some part of bit time
     if(enable='1'and trig='1')then
         crc21_nxt    := data_in xor crc21_reg(20);
         --Following bits
         if crc21_nxt='1'then
          --Shift and xor
          crc21_reg   <= (crc21_reg(19 downto 0)&'0') xor 
                          crc21_pol(20 downto 0); 
         else 
          crc21_reg   <= crc21_reg(19 downto 0)&'0'; --Only shift bits
         end if;
      else
         crc21_reg    <= crc21_reg;
      end if;    
   end if; 
   
  end if;
  end process crc21_cycle;   
      
end architecture;