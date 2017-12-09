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
--    July 2015   Created file
--    30.11.2017  Changed the buffer implementation from parallel into 32*20 
--                buffer of data. Reading so far left parallel. User is directly
--                accessing the buffer and storing the data to it.
--    04.12.2017  Buffer split to "Frame metadata" (txt_buffer_info) and "Data" 
--                (txt_buffer_data). Frame metadata consists of first 4 words 
--                (Frame format, Timestamps and Identifier). Frame metadata are
--                available combinationally at all times. Frame data are accessed
--                directly from CAN Core by new pointer "txt_data_addr". 
--                txt_buffer_data is synthesized as RAM memory and significant
--                reource reduction was achieved.
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Purpose:
--  Transmit message buffer. Access to TX_DATA_REG of user registers is combi-
--  nationally mapped to the TXT Buffers. User is storing the data directly into
--  the TX buffer. Once the user allows to transmitt from the buffer, content of
--  the buffer is validated and "empty" is cleared.
--------------------------------------------------------------------------------

entity txtBuffer is
    generic(
      constant ID           :natural :=1;
      constant useFDsize    :boolean :=false
    );
    PORT(
      ------------------
      --Clock and reset-
      ------------------
      signal clk_sys        :in   std_logic;
      signal res_n          :in   std_logic; --Async reset
      
      -------------------------------
      --Driving Registers Interface--
      -------------------------------
      
      --Driving bus
      signal drv_bus        :in   std_logic_vector(1023 downto 0);  
      
       --Data into the RAM of TXT Buffer
      signal tran_data      :in   std_logic_vector(31 downto 0);
      
      --Address in the RAM of TXT buffer  
      signal tran_addr      :in   std_logic_vector(4 downto 0);
      
      ------------------     
      --Status signals--
      ------------------
      
      --Logic 1 signals empty TxTime buffer
      signal txt_empty      :out  std_logic;
          
      ------------------------------------
      --CAN Core and TX Arbiter Interface-
      ------------------------------------
      
      --Signal from TX Arbiter that data were transmitted and buffer 
      --can be erased
      signal txt_data_ack   :in   std_logic;                        
      
      -- Data of the frame to be transmitted and pointer to the RAM memory
      -- of TXT buffer
      signal txt_data_word      :out  std_logic_vector(31 downto 0);
      signal txt_data_addr      :in   natural range 0 to 15;
      
      --First 4 words (frame format, timestamps, identifier) are available 
      --combinationally, to be able instantly decide on higher priority frame
      signal txt_frame_info_out :out  std_logic_vector(127 downto 0)
      
      );
             
end entity;


architecture rtl of txtBuffer is

  ----------------------
  --Internal registers--
  ----------------------
  type frame_data_memory is array(0 to 15) of std_logic_vector(31 downto 0);
  type frame_info_memory is array (0 to 3) of std_logic_vector(31 downto 0);

  ------------------
  --Signal aliases--
  ------------------
  
  -- Time transcieve buffer - Data memory
  signal txt_buffer_data  : frame_data_memory;
  
  -- Frame format, Timestamps and Identifier
  signal txt_buffer_info  : frame_info_memory;
   
  -- Store into TXT buffer 1 or 2 
  signal tran_wr          : std_logic_vector(1 downto 0);
  
  -- Status of the register
  signal txt_empty_reg    : std_logic;
  
  -- Allow/forbid transmission from the buffer
  signal drv_allow        : std_logic;
  
  -- Registered value for the detection 0-1 transition and signalling 
  -- that the buffer is full
  signal drv_allow_reg    : std_logic;
  
begin
    
    --Write signals for buffer
    tran_wr           <= drv_bus(DRV_TXT2_WR)&drv_bus(DRV_TXT1_WR);
    txt_empty         <= txt_empty_reg;
    
    --Driving bus aliases
    drv_allow         <= drv_bus(DRV_ALLOW_TXT1_INDEX) when ID=1 else
                         drv_bus(DRV_ALLOW_TXT2_INDEX) when ID=2 else
                        '0';
    
    --Output data are given by the address from the Core
    txt_data_word <= txt_buffer_data(txt_data_addr);
    
    --First 4 words of the Frame are available constantly...
    txt_frame_info_out <= txt_buffer_info(0)&
													txt_buffer_info(1)&
													txt_buffer_info(2)&
													txt_buffer_info(3);
    
    ----------------------------------------------------------------------------
    -- Main buffer comment
    ----------------------------------------------------------------------------
    tx_buf_proc:process(res_n,clk_sys)
    begin
      if (res_n = ACT_RESET) then
        
          -- In order to use RAM for the buffer, async reset cannot be done!
          -- synthesis translate_off
          txt_buffer_data <= (OTHERS => (OTHERS => '0'));
          -- synthesis translate_on
        
          -- Frame info is stored in registers
          txt_buffer_info <= (OTHERS => (OTHERS => '0'));
          txt_empty_reg <= '1';
      elsif (rising_edge(clk_sys))then
        
        --Registering the previous allow value
        drv_allow_reg <= drv_allow;
        
        --Updating the value of empty either from Registers or TX Arbitrator
        if (txt_data_ack='1') then
          txt_empty_reg <= '1';  
        elsif (drv_allow_reg='0' and drv_allow='1') then 
					-- 0-1 on drv_allow signals validation of the buffer content!
          txt_empty_reg <= '0';
        else 
          txt_empty_reg <= txt_empty_reg;
        end if;
        
        --Store the data into the Buffer during the access
        if (tran_wr(ID-1)='1') then
          if (tran_addr<4) then
            txt_buffer_info(to_integer(unsigned(tran_addr))) <= tran_data;
          else  
            txt_buffer_data(to_integer(unsigned(tran_addr-4))) <= tran_data;
          end if;
        end if;
        
      end if;
    end process;
  
end architecture;