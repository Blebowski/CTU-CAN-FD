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
--
-------------------------------------------------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------
-- Purpose:
--  Transmit message buffer. RAM type memory. Storing by command on drv bus. All the data stored at once --
--  Erasing by activating signal txt_data_ack. Possible to store and erase at the same cycle without     --
--  losing the data. If buffer is full and new data are commited then data are not written but lost.     --
--  txt_disc output is active in this case !                                                             --
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
      signal tran_data_in   :in   std_logic_vector(639 downto 0);   --Input data frame (Format B value of transcieve register of driving registers) 
      
      ------------------     
      --Status signals--
      ------------------
      signal txt_empty      :out  std_logic;                       --Logic 1 signals empty TxTime buffer
      signal txt_disc       :out  std_logic;                       --Info that message store into buffer from driving registers failed because buffer is full
            
      ------------------------------------
      --CAN Core and TX Arbiter Interface-
      ------------------------------------
      signal txt_buffer_out :out  std_logic_vector(639 downto 0);  --Output value of message in the buffer  
      signal txt_data_ack   :in   std_logic                         --Signal from TX Arbiter that data were sent and buffer can be erased     
      );
             
end entity;


architecture rtl of txtBuffer is

  ----------------------
  --Internal registers--
  ----------------------
  type memory is array(0 to 0) of 
      std_logic_vector(639 downto 0);
  
  --Modification for SRAM inferrence, only attempt
  --type RAM_memory is array(0 to 0) of std_logic_vector(127 downto 0);
  --signal data_memory:RAM_memory;
  
  type short_memory is array(0 to 0) of 
      std_logic_vector(639 downto 448);
   
  signal txt_empty_reg    :std_logic;                           --Register of empty buffer
  signal prev_store       :std_logic;                           --Registred value of store command for edge detection

  ------------------
  --Signal aliases--
  ------------------
  signal drv_erase_txt    :std_logic;                           --Command for erasing time transcieve buffer
  signal drv_store_txt    :std_logic;                           --Command for storing data from tran_data_in into txt_buffer
  
  constant empty          :std_logic_vector(447 downto 0):=(OTHERS=>'0');
  
  signal txt_buffer       :memory;                              --Time transcieve buffer
  signal txt_buffer_s     :short_memory;                        --Time transcieve buffer
    
begin
    
    drv_erase_txt     <= drv_bus(DRV_ERASE_TXT1_INDEX) when ID=1 else
                         drv_bus(DRV_ERASE_TXT2_INDEX) when ID=2 else 
                         '0';
    drv_store_txt     <= drv_bus(DRV_STORE_TXT1_INDEX) when ID=1 else
                         drv_bus(DRV_STORE_TXT2_INDEX) when ID=2 else 
                         '0';
    txt_empty         <= txt_empty_reg;
    
 GEN:if(useFDsize=true)generate
    
    --Output assignment and aliases
    --txt_buffer_out  <= txt_buffer(0);
    txt_buffer_out    <= txt_buffer(0);
    txt_buffer_s(0)   <= (OTHERS=>'0');
    
  mem_acess:process(clk_sys,res_n)
  variable aux:std_logic_vector(1 downto 0); --Auxiliarly variable
  begin
  if (res_n=ACT_RESET) then
      txt_buffer(0)   <= (OTHERS=>'0');
      txt_disc        <= '0';
      txt_empty_reg   <= '1';
      prev_store      <= '0';
  elsif rising_edge(clk_sys) then
    
      --Registering the store value for edge detection
      prev_store      <= drv_erase_txt; 
      
      ------------------------------------------------------------
      --Decoding the control signals for storing and erasing the
      --buffer into auxiliarly variable
      ------------------------------------------------------------
      if((drv_erase_txt='1') or (txt_data_ack='1'))then
        if(drv_store_txt='1' and prev_store='0')then
          aux         := "11";
        else
          aux         := "01";
        end if; 
      else
        if(drv_store_txt='1' and prev_store='0')then
          aux:="10";
        else
          aux:="00";
        end if; 
      end if;  
      
      -----------------------------------------
      --Storing and erasing the memory buffer--
      -----------------------------------------
      case aux is
        
        --No write, No discard
        when "00" =>  
            txt_buffer      <= txt_buffer;
            txt_disc        <= '0';
            txt_empty_reg   <= txt_empty_reg;
        
        --No write, discard
        when "01" =>  
            txt_buffer(0)   <= (OTHERS=>'0');
            txt_empty_reg   <= '1';
            txt_disc        <= '0';
        
        --Write, No discard
        when "10" =>  
            if(txt_empty_reg='0')then
              txt_buffer    <= txt_buffer;
              txt_disc      <= '1';
            else
              txt_buffer(0) <= tran_data_in;
              txt_disc<='0';
            end if;
              txt_empty_reg <= '0';
        
        --Write, and discard    
        when "11" =>  
            if(txt_empty_reg='0')then --If write and discard is at same then old data discarded new data stored
              txt_empty_reg <= '0';
              txt_disc      <= '0';
              txt_buffer(0) <= tran_data_in;         
            else
              txt_empty_reg <= '0';
              txt_buffer(0) <= tran_data_in;
              txt_disc      <= '0';
            end if;
          
        when others =>
             txt_buffer     <= txt_buffer;
             txt_empty_reg  <= txt_empty_reg;
             txt_disc       <= '0';
             
      end case;          
    end if;  
  end process mem_acess;
     
  end generate GEN; 
   
  GEN2:if(useFDsize=false)generate
  
    --Output assignment and aliases
    txt_buffer_out          <= txt_buffer_s(0)&empty; 
    txt_buffer(0)           <= (OTHERS=>'0');
    
    mem_acess:process(clk_sys,res_n)
    variable aux:std_logic_vector(1 downto 0); --Auxiliarly variable
    begin
    if res_n=ACT_RESET then
        txt_buffer_s(0)     <= (OTHERS=>'0');
        txt_disc            <= '0';
        txt_empty_reg       <= '1';
        prev_store          <= '0';
    elsif rising_edge(clk_sys) then
      
        --Registering the store value for edge detection
        prev_store<=drv_erase_txt; 
        
        ------------------------------------------------------------
        --Decoding the control signals for storing and erasing the
        --buffer into auxiliarly variable
        ------------------------------------------------------------
        if((drv_erase_txt='1') or (txt_data_ack='1'))then
          if(drv_store_txt='1' and prev_store='0')then
            aux             := "11";
          else
            aux             := "01";
          end if; 
        else
          if(drv_store_txt='1' and prev_store='0')then
            aux             := "10";
          else
            aux             := "00";
          end if; 
        end if;  
        
        -----------------------------------------
        --Storing and erasing the memory buffer--
        -----------------------------------------
        case aux is
          when "00" =>  --No write, No discard
              txt_buffer_s      <= txt_buffer_s;
              txt_disc          <= '0';
              txt_empty_reg     <= txt_empty_reg;
          when "01" =>  --No write, discard
              txt_buffer_s(0)   <= (OTHERS=>'0');
              txt_empty_reg     <= '1';
              txt_disc          <= '0';
          when "10" =>  --Write, no discard
              if(txt_empty_reg='0')then
                txt_buffer_s    <= txt_buffer_s;
                txt_disc        <= '1';
              else
                txt_buffer_s(0) <= tran_data_in(639 downto 448);
                txt_disc        <= '0';
              end if;
                txt_empty_reg   <= '0';
          when "11" =>  --Write, and discard
              if(txt_empty_reg='0')then --If write and discard is at same then old data discarded new data stored
                txt_empty_reg   <= '0';
                txt_disc        <= '0';
                txt_buffer_s(0) <= tran_data_in(639 downto 448);
              else
                txt_empty_reg   <= '0';
                txt_buffer_s(0) <= tran_data_in(639 downto 448);
                txt_disc        <= '0';
              end if;  
          when others =>
               txt_buffer_s     <= txt_buffer_s;
               txt_empty_reg    <= txt_empty_reg;
               txt_disc         <= '0';
        end case;
                  
      end if;  
    end process mem_acess;
   end generate GEN2;      
  
end architecture;