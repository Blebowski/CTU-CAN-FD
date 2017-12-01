Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;

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
--    30.11.2017  Changed the buffer implementation from parallel into 32*20 buffer of data. Reading so far
--                left parallel. User is directly accessing the buffer and storing the data to it.
-------------------------------------------------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------
-- Purpose:
--  Transmit message buffer. Access to TX_DATA_REG of user registers is combinationally mapped
--  to the TXT Buffers. User is storing the data directly into the TX buffer. Once the user allows to
--  transmitt from the buffer, content of the buffer is validated and "empty" is cleared.
---------------------------------------------------------------------------------------------------------

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
      signal res_n          :in   std_logic;                        --Async reset
      
      -------------------------------
      --Driving Registers Interface--
      -------------------------------
      signal drv_bus        :in   std_logic_vector(1023 downto 0);  --Driving bus
      
      signal tran_data      :in   std_logic_vector(31 downto 0);  --Data into the RAM of TXT Buffer
      signal tran_addr      :in   std_logic_vector(4 downto 0);  --Address in the RAM of TXT buffer  
      
      ------------------     
      --Status signals--
      ------------------
      signal txt_empty      :out  std_logic;                       --Logic 1 signals empty TxTime buffer
          
      ------------------------------------
      --CAN Core and TX Arbiter Interface-
      ------------------------------------
      signal txt_buffer_out :out  std_logic_vector(639 downto 0);  --Output value of message in the buffer  
      signal txt_data_ack   :in   std_logic                        --Signal from TX Arbiter that data were sent and buffer can be erased     
      );
             
end entity;


architecture rtl of txtBuffer is

  ----------------------
  --Internal registers--
  ----------------------
  type memory is array(0 to 19) of 
      std_logic_vector(31 downto 0);

  ------------------
  --Signal aliases--
  ------------------
  signal txt_buffer       : memory;                              -- Time transcieve buffer  
  signal tran_wr          : std_logic_vector(1 downto 0);        -- Store into TXT buffer 1 or 2 
  signal txt_empty_reg    : std_logic;                           -- Status of the register
  
  signal drv_allow        : std_logic;                           
  signal drv_allow_reg    : std_logic;                           -- Registered value for the detection 0-1 transition and signalling that the buffer is full
  
begin
    
    --Write signals for buffer
    tran_wr           <= drv_bus(DRV_TXT2_WR)&drv_bus(DRV_TXT1_WR);
    txt_empty         <= txt_empty_reg;
    
    --Driving bus aliases
    drv_allow         <= drv_bus(DRV_ALLOW_TXT1_INDEX) when ID=1 else
                         drv_bus(DRV_ALLOW_TXT2_INDEX) when ID=2 else
                        '0';
    
    --Output assignment and aliases
    -- So far reading of the data from buffer is parelell. It will be modified to reading by word...
    txt_buffer_out  <= txt_buffer(0)&txt_buffer(1)&txt_buffer(2)&txt_buffer(3)&
                       txt_buffer(4)&txt_buffer(5)&txt_buffer(6)&txt_buffer(7)&txt_buffer(8)&
                       txt_buffer(9)&txt_buffer(10)&txt_buffer(11)&txt_buffer(12)&txt_buffer(13)&
                       txt_buffer(14)&txt_buffer(15)&txt_buffer(16)&txt_buffer(17)&txt_buffer(18)&txt_buffer(19);
  
    --------------------------------------------------------------------------------
    -- Main buffer comment
    --------------------------------------------------------------------------------
    tx_buf_proc:process(res_n,clk_sys)
    begin
      if (res_n = ACT_RESET) then
        
        -- synthesis translate_off
          txt_buffer <= (OTHERS => (OTHERS => '0'));
        -- synthesis translate_on
        
          txt_empty_reg <= '1';
      elsif (rising_edge(clk_sys))then
        
        --Registering the previous allow value
        drv_allow_reg <= drv_allow;
        
        --Updating the value of empty either from Registers or TX Arbitrator
        if (txt_data_ack='1') then
          txt_empty_reg <= '1';  
        elsif (drv_allow_reg='0' and drv_allow='1') then -- 0-1 on drv_allow signals validation of the buffer content!
          txt_empty_reg <= '0';
        else 
          txt_empty_reg <= txt_empty_reg;
        end if;
        
        --Store the data into the Buffer during the access
        if (tran_wr(ID-1)='1') then
          txt_buffer(to_integer(unsigned(tran_addr))) <= tran_data;
        end if;
        
      end if;
    end process;
  
end architecture;