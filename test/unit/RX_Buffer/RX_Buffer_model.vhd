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
--  Behavioral model of RX_Buffer. Implements architecture of RX_Buffer entity but in
--  behavioral way, not RTL. This model is intended to be used in RX_buffer automated
--  testbench.             
--  Not yet tested and debugged. Only first implementation                               
--------------------------------------------------------------------------------
-- Revision History:
--
--    1.6.2016   Created file - First implementation of the model!!
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
USE ieee.std_logic_unsigned.All;
use work.CANconstants.all;
USE work.CANtestLib.All;
use work.CANcomponents.ALL;
USE work.randomLib.All;

use work.ID_transfer.all;

architecture behav of rxBuffer is
  
  procedure calc_free_memory(
    signal    write_pointer :in natural range 0 to buff_size-1;
    signal    read_pointer  :in natural range 0 to buff_size-1;
    variable  free_space  :out natural
  )is
  begin
    if(write_pointer>read_pointer)then
      free_space := buff_size - (write_pointer-read_pointer);
    elsif(read_pointer>write_pointer)then
      free_space := write_pointer - read_pointer;
    else
      free_space := buff_size;
    end if;
     
  end procedure;

  procedure store_frame(
    signal rec_ident_in         :in std_logic_vector(28 downto 0);    --Message Identifier
    signal rec_data_in          :in std_logic_vector(511 downto 0);   --Message Data (up to 64 bytes);
    signal rec_dlc_in           :in std_logic_vector(3 downto 0);     --Data length code
    signal rec_ident_type_in    :in std_logic;                        --Recieved identifier type (0-BASE Format, 1-Extended Format);
    signal rec_frame_type_in    :in std_logic;                        --Recieved frame type (0-Normal CAN, 1- CAN FD)
    signal rec_is_rtr           :in std_logic;                        --Recieved frame is RTR Frame(0-No, 1-Yes)
    signal rec_brs              :in std_logic;                        --Whenever frame was recieved with BIT Rate shift     
    signal memory               :out rx_memory;
    signal write_pointer        :inout natural range 0 to buff_size-1;
    variable dlc_length         :in  natural
    )is
  begin
    
    --Frame format word
    memory(write_pointer)<=   "0000000000000000000000"&rec_brs&'1'&rec_frame_type_in&rec_ident_type_in&rec_is_rtr&'0'&rec_dlc_in;
    write_pointer  <= write_pointer+1;
    wait until rising_edge(clk_sys);
      
    --Timestamp
    memory(write_pointer) <= timestamp(63 downto 32);
    write_pointer  <= write_pointer+1;
    wait until rising_edge(clk_sys);  
    
    memory(write_pointer) <= timestamp(31 downto 0);
    write_pointer  <= write_pointer+1;
    wait until rising_edge(clk_sys);   
      
    --Identifier
    memory(write_pointer)<=   "000"&rec_ident_in;
    write_pointer  <= write_pointer+1;
    wait until rising_edge(clk_sys);
    
    --Data bytes
    if(dlc_length>0)then
      for i in 0 to (dlc_length-1)/4 loop
         memory(write_pointer)  <= rec_data_in(511-i*32 downto 480-i*32);
         write_pointer          <= write_pointer+1;
         wait until rising_edge(clk_sys);
      end loop;
    end if;
  
  end procedure;
  
  procedure count_frames(
    signal message_mark       :in   std_logic_vector(buff_size-1 downto 0);
    variable frame_count      :out  natural
    )is
  variable frame_count_int    :     natural;
  begin
   frame_count_int    :=  0;
   for i in 0 to message_mark'length-1 loop
     frame_count_int  :=  frame_count_int+1;
   end loop;
    frame_count       := frame_count_int;
  end procedure;
  
  
begin
  
  --Internal signal aliases
  drv_erase_rx          <= drv_bus(DRV_ERASE_RX_INDEX);
  drv_ovr_rx            <= drv_bus(DRV_OVR_RX_INDEX);
  drv_read_start        <= drv_bus(DRV_READ_START_INDEX);
  drv_clr_ovr           <= drv_bus(DRV_CLR_OVR_INDEX);
  rx_buf_size           <= std_logic_vector(to_unsigned(buff_size,8));
  
  
  storing:process
  variable dlc_length :natural;
  variable free_mem   :natural;
  begin
    if res_n = '0' then
      rx_mem_free   <= std_logic_vector(to_unsigned(buff_size,8));
      memory        <= (OTHERS => (OTHERS => '0'));
      message_mark  <= (OTHERS => '0');
    else   
      while rec_message_valid='0' loop    
         
        --Message discarding and data overrun
        rx_message_disc<='0';       
        if(drv_clr_ovr='1')then
          rx_data_overrun<='0';
        end if;       
        
        --When reading message mark is reduced by one
        if(drv_read_start='1' and message_mark(read_pointer)='1')then
           message_mark(read_pointer)<='0';
        end if;
      
        wait until rising_edge(clk_sys);
      end loop;
      
      calc_free_memory(write_pointer,read_pointer,free_mem);      
      decode_dlc(rec_dlc_in,dlc_length);
      
      -----------------------------------------------------
      --Store the input frame to the buffer
      --when there is enough space
      -----------------------------------------------------
      if(dlc_length<free_mem or dlc_length=free_mem) then
        message_mark(write_pointer)<='1';
        store_frame(rec_ident_in,rec_data_in,rec_dlc_in,rec_ident_type_in,rec_frame_type_in,rec_is_rtr,
                    rec_brs,memory,write_pointer,dlc_length);         
      else
        rx_data_overrun<='1';
        rx_message_disc<='1';
      end if;     
    
    end if;
  end process;
  
  --Async propagation of the read data
  rx_read_buff          <= memory(read_pointer);

  reading:process
  begin
    wait until rising_edge(clk_sys);
    
    if(drv_read_start='1')then
      read_pointer<=read_pointer+1;
    end if;
    
  end process;
  
  
  status:process
  variable free_mem :natural range 0 to buff_size-1;
  variable mes_cnt  :natural;
  begin
    wait until rising_edge(clk_sys);
    
    calc_free_memory(write_pointer,read_pointer,free_mem);
    
    if(free_mem = 0)then
      rx_full <= '1';
    else
      rx_full <= '0';
    end if;
    
    if(free_mem = buff_size)then
      rx_empty <= '1';
    else
      rx_empty <= '0';
    end if;
    
    rx_mem_free           <= std_logic_vector(to_unsigned(free_mem,8));
    rx_read_pointer_pos   <= std_logic_vector(to_unsigned(read_pointer,8));
    rx_write_pointer_pos  <= std_logic_vector(to_unsigned(write_pointer,8));
    
    count_frames(message_mark,mes_cnt);
    wait for 0 ns;
    rx_message_count <=  std_logic_vector(to_unsigned(mes_cnt,8));
    
  end process;
   
   
end architecture;