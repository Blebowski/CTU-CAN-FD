Library ieee;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.ALL;
use ieee.std_logic_unsigned.All;
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
--    18.12.2015  RX Buffer inference from Flip-flops changed to native SRAM memory on FPGA. Dual port memory used for
--                this purpose (sync write and async read). Memory is automatically recongized by sythetizer. Erase 
--                of buffer needed to be removed, because SRAM cant be erased all at once, due to this SRAM wasnt
--                inferred before. To achieve "erase like" behaviour simple workaround was implemented. Additional
--                vector "memory_valid" is kept. This vector is erased at once (FF based) and when write into
--                memory is performed this bit is set to logic '1' for appropriate field. Async read returns data from
--                memory when valid bit is set, and all zeroes if it is not. Due to this, from user point of view
--                memory acts as erased after initialization.
--                Due to this  "RAM initialiser" IP function or State machine does not have to be used. Memory is thus
--                available directly after async reset.
--                Disadvantage is that one memory vector of the same size as memory need to be kept!
--    2.6.2016    Added data_size set to 0 at start time to avoid possible storing of invalid frame
--    3.6.2016    1.Bug fix of mem_free variable. Variable was decreased by frame size at arrival of new frame! This is
--                  not expected behaviour since it takes up to 20 clock cycles to store the frame and mem_free is now
--                  reduced by one with each word stored!
--                2.Detected and fixed incorrect behaviour during data overrun! Wrong setting of copy_counter caused
--                  part of the frame to be stored during data_overrun
--    21.6.2016   Added limit of 512 to the RX Buffer size! Thisway it is compliant with memory map width of readable size
--    22.6.2016   1.Added RTR frame detection. Any RTR frame is recieved no data words are stored!!!
--                2.Added rec_esi bit stored into the buffer!
--    15.7.2016    Changed handling of moving to  next word in RX buffer RX_DATA. Falling edge detection removed
--                Now memory registers set drv_read_start only for ONE clock cycle per each access. So it is enough to
--                check whether signal is active! Thisway it is not necessary to add empty clock cycles between consecutive
--                reads from RX_DATA register!
--    29.11.2017  Changed hadnling of received data. "rec_data_in" replaced by "rec_dram_word" and "rec_dram_addr" as part of
--                resource optimizations. Data are not available in parallel at input of the RX buffer but addressed in
--                internal RAM of Protocol controller.
--
-----------------------------------------------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------------
-- Purpose:
--  Recieve buffer for messages. RAM memory type of N*32 bit words. Reading of data from registers done  --
--  word by word. In registers reading implemented in the way that one read moves to the next word.      --
--  Storing the message into the buffer is sequential operation started with valid rec_message_valid for --
--  one clock cycle. In following up to 20 clock cycles recieved data has to be valid to be fully stored --
--
--  Note:This is guaranteed from CAN Core. rec_message_valid is active in the end of EOF field. Intermi  --
--  ssion field follows with 11 bit times (minimum 55 clock cycles) where recieved data are not changed, --
--  only overload condition may be signallised!                                                          --
-----------------------------------------------------------------------------------------------------------

entity rxBuffer is
  GENERIC(
      buff_size                 :natural range 4 to 512 :=32 
      
       --Maximal number of 32 bit words to store (Minimal value=16, one 64 bytes message length)
       --Only 2^k are allowed as buff_size. Memory adressing is in modular arithmetic,
       --synthesis of modulo by number other than 2^k is not supported!!!
  );
  PORT(
    -------------------
    --Clocks and reset-
    -------------------
    signal clk_sys              :in std_logic; --System clock
    signal res_n                :in std_logic; --Async. reset
    
    -------------------------------------------------------
    --CAN Core interface (rec. data,validity, acknowledge)-
    -------------------------------------------------------
    signal rec_ident_in         :in std_logic_vector(28 downto 0);    --Message Identifier
    signal rec_dlc_in           :in std_logic_vector(3 downto 0);     --Data length code
    signal rec_ident_type_in    :in std_logic;                        --Recieved identifier type (0-BASE Format, 1-Extended Format);
    signal rec_frame_type_in    :in std_logic;                        --Recieved frame type (0-Normal CAN, 1- CAN FD)
    signal rec_is_rtr           :in std_logic;                        --Recieved frame is RTR Frame(0-No, 1-Yes)
    signal rec_brs              :in std_logic;                        --Whenever frame was recieved with BIT Rate shift 
    signal rec_esi              :in std_logic;                        --Recieved error state indicator
    
    signal rec_message_ack      :out std_logic;                       --Acknowledge for CAN Core about accepted data
    signal rec_message_valid    :in std_logic;                        --Output from acceptance filters (out_ident_valid) if message fits the filters
    
    --Added interface for aux SRAM
    signal rec_dram_word        :in  std_logic_vector(31 downto 0);
    signal rec_dram_addr        :out natural range 0 to 15;
    
    ------------------------------------
    --Status signals of recieve buffer--
    ------------------------------------
    signal rx_buf_size          :out std_logic_vector(7 downto 0);    --Actual size of synthetised message buffer (in 32 bit words)
    signal rx_full              :out std_logic;                       --Signal whenever buffer is full
    signal rx_empty             :out std_logic;                       --Signal whenever buffer is empty
    signal rx_message_count     :out std_logic_vector(7 downto 0);    --Number of messaged stored in recieve buffer
    signal rx_mem_free          :out std_logic_vector(7 downto 0);    --Number of free 32 bit wide ''windows''
    signal rx_read_pointer_pos  :out std_logic_vector(7 downto 0);    --Position of read pointer
    signal rx_write_pointer_pos :out std_logic_vector(7 downto 0);    --Position of write pointer
    signal rx_message_disc      :out std_logic;                       --Message was discarded since Memory is full
    signal rx_data_overrun      :out std_logic;                       --Some data were discarded, register
    
    signal timestamp            :in std_logic_vector(63 downto 0);
    
    ------------------------------------
    --User registers interface
    ------------------------------------  
    signal rx_read_buff         :out std_logic_vector(31 downto 0);   --Actually loaded data for reading
    signal drv_bus              :in std_logic_vector(1023 downto 0)   --Driving bus from registers
  );
    
  -----------------------------
  --Driving bus signal aliases-
  ----------------------------- 
  signal drv_erase_rx           :std_logic;                           --Erase command from driving registers
  signal drv_ovr_rx             :std_logic;                           --OverRun behaviour (0 - Discard new message if full , 1 - Rewrite the oldest message)
  signal drv_read_start         :std_logic;                           --Command to load increase the reading pointer
  signal drv_clr_ovr            :std_logic;                           --Clear data OverRun Flag
  
  ------------------
  --FIFO  Memory   -
  ------------------
  constant data_width           :natural := 32;                       --Word data width
  
  type rx_memory is array(0 to buff_size-1) of std_logic_vector(data_width-1 downto 0); --Memory type
  
  signal memory                 :rx_memory;  --Memory declaration inferred in SRAM
  signal memory_valid           :std_logic_vector(0 to buff_size-1);  --Vector if data word in memory is valid.
                                                                      --SRAM cant be erased at reset, this vector can! (because is DQ-FF based)
                                                                      --Workaround strategy for erasing memory!
  
  signal message_mark           :std_logic_vector(buff_size -1 downto 0); --Mark of new message('1') positions-
  signal read_pointer           :natural range 0 to buff_size-1;          --Read Pointer for data
  signal write_pointer          :natural range 0 to buff_size-1;          --Write pointer
  signal prev_read              :std_logic;                               --Registered value of read command for eedge detection. 
  --Note :1->0 edge detection is used here, to move to the next data when the previous read is finished!
  
  signal data_overrun_r         :std_logic;                               --Recieved message was lost because RX Buffer was full
  signal copy_counter           :natural range 0 to 20;                   --Counter used for copying Recieved data to recieve buffer
  signal data_size              :natural range 0 to 32; 
  
end entity;


architecture rtl of rxBuffer is
begin
  --Driving bus aliases
  drv_erase_rx          <= drv_bus(DRV_ERASE_RX_INDEX);
  drv_ovr_rx            <= drv_bus(DRV_OVR_RX_INDEX);
  drv_read_start        <= drv_bus(DRV_READ_START_INDEX);
  drv_clr_ovr           <= drv_bus(DRV_CLR_OVR_INDEX);
  rx_buf_size           <= std_logic_vector(to_unsigned(buff_size,8));
  
  --Propagating status registers on output
  rx_read_pointer_pos   <= std_logic_vector(to_unsigned(read_pointer,8)); --Output information about pointer position
  rx_write_pointer_pos  <= std_logic_vector(to_unsigned(write_pointer,8));
  rx_data_overrun       <= data_overrun_r;
  
  --TODO: check if timing analysis will stay still over 50 Mhz with following logic. 
  --This integrates into one combinational path memory read + MUX, possibly dangerous for Timing performance
  
  --Buffer output is propagated only if memory entry is marked as valid, 
  rx_read_buff          <= memory(read_pointer) when (memory_valid(read_pointer)='1') 
                           else (OTHERS => '0');  --Read data from SRAM memory (1 port, async read)
  
  -- Address for the Receive data RAM in the CAN Core!
  -- Comparator is temporary before the data order will be reversed!
  rec_dram_addr         <= 18-copy_counter when (copy_counter>2 and copy_counter<19) else 0;
  
  ------------------------------------------------------------------
  --Storing data from CANCore and loading data into reading buffer--
  ------------------------------------------------------------------
  memory_acess:process(clk_sys,res_n)
    variable data_length    : natural range 0 to 16;        --Length variable for frame stored into reading buffer (in 32 bit words)
    variable mem_free       : natural range 0 to buff_size:= buff_size; --Amount of free words
    variable message_count  : natural range 0 to 255;       --Message Count already stored  
 begin     
    if (res_n=ACT_RESET) or (drv_erase_rx='1') then
      write_pointer     <= 0;
      read_pointer      <= 0;
      rx_full           <= '0';
      rx_empty          <= '1';
      mem_free          := buff_size;
      rx_mem_free       <= std_logic_vector(to_unsigned(buff_size,8));
          
      --After reset SRAM contents stays but it is not valid!
      memory_valid      <= (OTHERS=>'0');
      
      message_count     := 0;
      rx_message_count  <= (OTHERS=>'0');
      message_mark      <= (OTHERS=>'0');
      
      --Nulling output signals
      rec_message_ack   <= '0';
      rx_message_disc   <= '0';
      data_overrun_r    <= '0';
      prev_read         <= '0';
      copy_counter      <= 16; 
      data_size         <= 0;
      
    elsif rising_edge(clk_sys) then
      
      prev_read         <= drv_read_start;
      memory_valid      <= memory_valid;
      read_pointer      <= read_pointer;
      
      --Clearing the overRun flag
      if(drv_clr_ovr='1')then
       data_overrun_r   <= '0';
      else  
       data_overrun_r   <= data_overrun_r;
      end if;
      
      ------------------------------------------------------------
      --Moving to next word by reading (if there is sth to read)
      ------------------------------------------------------------ 
      if (    (drv_read_start='1')  and 
         (not (read_pointer=write_pointer and mem_free=buff_size)) )then 
             
        read_pointer                <= (read_pointer+1) mod buff_size; --Increase the reading pointer
        
        --Test erase when moved further!!!
        memory_valid(read_pointer)  <= '0'; --Mark the actual field as invalid, since we want the behaviour of FIFO from user
                                            --perspective to be that data are automatically erased after reading...
        
        message_mark(read_pointer)  <= '0'; --If begin of new message then nulling
        mem_free                    := mem_free+1;
         
        --If new message was moved then decrease number of messages
        if(message_mark(read_pointer)='1')then
          message_count             := message_count-1; 
        end if;
           
      end if;
       
      
      ----------------------------
      --Storing recieved message--
      ----------------------------
      if(rec_message_valid='1')then
          rec_message_ack <= '1'; --Acknowledge message reception for CAN Core
          
          --If frame is RTR we dont CARE about recieved DLC, that can be arbitrary
          -- due to the rtr preffered behaviour! RTR frame, automatically no data
          -- are stored!!!
          if(rec_is_rtr='1' and rec_frame_type_in='0')then
            data_length := 0;
          else
           case rec_dlc_in is
            when "0000" => data_length:=0; --Zero bits
            when "0001" => data_length:=1; --1 byte
            when "0010" => data_length:=1; --2 bytes
            when "0011" => data_length:=1; --3 bytes
            when "0100" => data_length:=1; --4 bytes
            when "0101" => data_length:=2; --5 bytes
            when "0110" => data_length:=2; --6 bytes
            when "0111" => data_length:=2; --7 bytes
            when "1000" => data_length:=2; --8 bytes
            when "1001" => data_length:=3; --12 bytes
            when "1010" => data_length:=4; --16 bytes
            when "1011" => data_length:=5; --20 bytes
            when "1100" => data_length:=6; --24 bytes
            when "1101" => data_length:=8; --32 bytes
            when "1110" => data_length:=12; --48 bytes
            when "1111" => data_length:=16; --64 bytes
            when others => data_length:=0;
          end case;
        end if; 
        
          if( (mem_free>data_length+4) or 
              (mem_free=data_length+4) )then --Checking if message can be stored
            --Marking new message
            message_mark(write_pointer) <= '1';
            message_count               := message_count+1;
            
            --Writing Frame format Word
            rx_message_disc             <= '0';
            memory(write_pointer)       <= "000000000000000000000"&rec_esi&rec_brs&'1'&rec_frame_type_in&rec_ident_type_in&rec_is_rtr&'0'&rec_dlc_in;
            memory_valid(write_pointer) <= '1';
            
           --Increasing write pointer -- TODO: think if modulo can be discarded for synthesis
           write_pointer                <= (write_pointer+1) mod buff_size;
           --mem_free                   := mem_free-4-data_length;
           mem_free                     := mem_free-1;
           
           if(rec_is_rtr='1' and rec_frame_type_in='0')then
             data_size    <= 3;
           else
             case rec_dlc_in is
              when "0000" => data_size    <= 3; --Zero bits
              when "0001" => data_size    <= 4; --1 byte
              when "0010" => data_size    <= 4; --2 bytes
              when "0011" => data_size    <= 4; --3 bytes
              when "0100" => data_size    <= 4; --4 bytes
              when "0101" => data_size    <= 5; --5 bytes
              when "0110" => data_size    <= 5; --6 bytes
              when "0111" => data_size    <= 5; --7 bytes
              when "1000" => data_size    <= 5; --8 bytes
              when "1001" => data_size    <= 6; --12 bytes
              when "1010" => data_size    <= 7; --16 bytes
              when "1011" => data_size    <= 8; --20 bytes
              when "1100" => data_size    <= 9; --24 bytes
              when "1101" => data_size    <= 11; --32 bytes
              when "1110" => data_size    <= 15; --48 bytes
              when "1111" => data_size    <= 19; --64 bytes
              when others => data_size    <= 0;
              end case;
          end if;
          
          --Set the copy counter to properly copy the data in next cycles
          copy_counter                  <= 0;
          
          else 
            rx_message_disc             <= '1';
            data_overrun_r              <= '1';
            write_pointer               <= write_pointer;
            copy_counter                <= 16;
            data_size                   <= 0;
          end if;
          
      elsif(copy_counter<3)then  --Here only Identifier and Timestamp is stored
        
        --------------------------------------------------------------
        --copy_counter decodes which part of received frame to store--
        --------------------------------------------------------------
        if(copy_counter=0)then
            memory(write_pointer)       <= timestamp(63 downto 32);
            memory_valid(write_pointer) <= '1';
        elsif(copy_counter=1)then
            memory(write_pointer)       <= timestamp(31 downto 0);
            memory_valid(write_pointer) <= '1';
        elsif(copy_counter=2)then   
            memory(write_pointer)       <= "000"&rec_ident_in;
            memory_valid(write_pointer) <= '1';
        else
           memory(write_pointer)        <= memory(write_pointer);
           memory_valid(write_pointer)  <= '1';
        end if;
        
        write_pointer                   <= (write_pointer+1) mod buff_size;
        copy_counter                    <= copy_counter+1;
        data_size                       <= data_size;
        rec_message_ack                 <=  '0';
        
        mem_free                        := mem_free-1;
      
      elsif(copy_counter<data_size)then -- Here the data words are stored
        
        --Optimized implementation of the storing with auxiliarly receive data RAM
        memory(write_pointer)           <= rec_dram_word;
        
        memory_valid(write_pointer)     <= '1';
        write_pointer                   <= (write_pointer+1) mod buff_size;
        copy_counter                    <= copy_counter+1;
        data_size                       <= data_size;
        rec_message_ack                 <= '0';
        mem_free                        := mem_free-1;
        
      else
        rx_message_disc                 <= '0';
        rec_message_ack                 <= '0';
        write_pointer                   <= write_pointer;
        copy_counter                    <= 16;
        data_size                       <= 0;
      end if;
  
  rx_mem_free                           <= std_logic_vector(to_unsigned(mem_free,8));
  
  --Assigning output whenever memory is full
  if (mem_free=0) then 
    rx_full                             <= '1';
  else
    rx_full                             <= '0';
  end if;
  
  --Memory empty output
  if (mem_free=buff_size) then 
    rx_empty                            <= '1';
  else
    rx_empty                            <= '0';
  end if;
  
  --Propagating message count to output
  rx_message_count                      <= std_logic_vector(to_unsigned(message_count,8));

  end if;
end process memory_acess;

end architecture;