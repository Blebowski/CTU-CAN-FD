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
--  Recieve buffer for messages. RAM memory type of N*32 bit words. Reading of 
--  data from registers done word by word. In registers reading implemented in 
--  the way that one read moves to the next word. Storing the message into the
--  buffer is sequential operation started with valid rec_message_valid for one
--  clock cycle. In following up to 20 clock cycles recieved data has to be va-
--  lid to be fully stored
--
--  Note:This is guaranteed from CAN Core. rec_message_valid is active in the 
--  end of EOF field. Intermission field follows with 11 bit times (minimum 55 
--  clock cycles) where recieved data are not changed, only overload condition 
--  may be signallised!
--------------------------------------------------------------------------------
-- Revision History:
--    July 2015   Created file
--    18.12.2015  RX Buffer inference from Flip-flops changed to native SRAM me-
--                mory on FPGA. Dual port memory used for this purpose (sync 
--                write and async read). Memory is automatically recongized by
--                synthetizer. Erase of buffer needed to be removed, because 
--                SRAM cant be erased all at once, due to this SRAM wasnt in-
--                ferred before. To achieve "erase like" behaviour simple work-
--                around was implemented. Additional vector "memory_valid" is 
--                kept. This vector is erased at once (FF based) and when write
--                into memory is performed this bit is set to logic '1' for app-
--                ropriate field. Async read returns data from memory when valid
--                bit is set, and all zeroes if it is not. Due to this, from user
--                point of view memory acts as erased after initialization.
--                Due to this  "RAM initialiser" IP function or State machine
--                does not have to be used. Memory is thus available directly
--                after async reset.
--                Disadvantage is that one memory vector of the same size as 
--                memory need to be kept!
--    2.6.2016    Added data_size set to 0 at start time to avoid possible 
--                storing of invalid frame
--    3.6.2016    1.Bug fix of mem_free variable. Variable was decreased by frame
--                  size at arrival of new frame! This is not expected behaviour
--                  since it takes up to 20 clock cycles to store the frame and 
--                  mem_free is now reduced by one with each word stored!
--                2.Detected and fixed incorrect behaviour during data overrun! 
--                  Wrong setting of copy_counter caused part of the frame to be
--                  stored during data_overrun
--    21.6.2016   Added limit of 512 to the RX Buffer size! Thisway it is comp-
--                liant with memory map width of readable size
--    22.6.2016   1.Added RTR frame detection. Any RTR frame is recieved no data
--                  words are stored!!!
--                2.Added rec_esi bit stored into the buffer!
--    15.7.2016   Changed handling of moving to next word in RX buffer RX_DATA.
--                Falling edge detection removed Now memory registers set 
--                "drv_read_start" only for ONE clock cycle per each access. So
--                it is enough to check whether signal is active! Thisway it is
--                not necessary to add empty clock cycles between consecutive
--                reads from RX_DATA register!
--    29.11.2017  Changed hadnling of received data. "rec_data_in" replaced by 
--                "rec_dram_word" and "rec_dram_addr" as part of resource opti-
--                mizations. Data are not available in parallel at input of the
--                RX buffer but addressed in internal RAM of Protocol control.
--    09.02.2018  1. Fixed data_size upper range threshold from 32 to 31.
--                2. Added "frame_form_w" to assign it with generated indices
--                3. Added combinational decoder on received DLC to frame
--                   length in words (without frame_format word) into new signal
--                   "data_size_comb".
--    19.02.2018  Removed memory valid vector. Output word is 0 only if
--                memory is completely empty.
--    20.02.2018  1. Implemented process for counting frames "read_frame_proc".
--                   It stores size of the frame at the first read and decrements
--                   it until 1. At transition from 1 to 0, message counter is
--                   decremented.
--                2. Added commit_rx_frame which will be active for one clock
--                   cycle when frame storing finished. This is preparation for
--                   continous storing of the frame during reception, instead
--                   of storing it at once at the end.
--                3. Changed read handling. Read is allowed to proceed (increment
--                   read pointer) only if new message counter is non-zero. This
--                   guarantees that frame is fully stored (again preparation
--                   for later) at the time of first read. IT also keeps the 
--                   read pointer in sync with frame counting process from p.1.
--                4. Since "rx_empty" will be used for detection of frame in the
--                   buffer, its now driven by non-zero message counter instead
--                   of non-zero amount of stored words. Non-zero amount of
--                   stored words would indicate that buffer is not empty even
--                   if the frame was not committed yet! We dont want to signal
--                   it since storing of the rest of the words (in case of con-
--                   tinous storing during reception) might take longer than
--                   reading the words out! Thus we would end up in a situation
--                   where buffer is marked as non-empty but, SW cant read whole
--                   frame from it! This is undesirable.
--                5. Removed "message_mark" signal and original "message_counter"
--                   variable in memory access process due to beiing obsolete
--                   with new implementation of message counter.
--                6. Increased maximal buffer depth to 4096, resized output
--                   vectors accordingly.
--    22.02.2018  1. Removed obsolete "drv_ovr_rx".
--                2. Added configurable capturing of timestamp on beginning or
--                   end of the frame.
--------------------------------------------------------------------------------

Library ieee;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.ALL;
use work.CANconstants.all;
use work.CAN_FD_frame_format.all;
use work.CAN_FD_register_map.all;

entity rxBuffer is
  GENERIC(
  
      -- Only 2^k are allowed as buff_size. Memory adressing is in modular 
      -- arithmetic, synthesis of modulo by number other than 2^k might not play
      -- nicely (will consume lot of LUTs)!!
      buff_size                 :natural range 32 to 4096 := 32
  );
  PORT(
    -------------------
    --Clocks and reset-
    -------------------
    signal clk_sys              :in std_logic; --System clock
    signal res_n                :in std_logic; --Async. reset
    
    -------------------------------------------------------
    --CAN Core interface (rec. data,validity, acknowledge)
    -------------------------------------------------------
    
    --Message Identifier
    signal rec_ident_in         :in std_logic_vector(28 downto 0);
    
    --Data length code
    signal rec_dlc_in           :in std_logic_vector(3 downto 0);
    
    --Recieved identifier type (0-BASE Format, 1-Extended Format);
    signal rec_ident_type_in    :in std_logic;
    
    --Recieved frame type (0-Normal CAN, 1- CAN FD)
    signal rec_frame_type_in    :in std_logic;
    
    --Recieved frame is RTR Frame(0-No, 1-Yes)
    signal rec_is_rtr           :in std_logic;
    
    --Whenever frame was recieved with BIT Rate shift 
    signal rec_brs              :in std_logic;
    
    --Recieved error state indicator
    signal rec_esi              :in std_logic;
    
    --Acknowledge for CAN Core about accepted data
    signal rec_message_ack      :out std_logic;
    
    --Output from acceptance filters (out_ident_valid) if message fits filters
    signal rec_message_valid    :in std_logic;
    
    --Added interface for aux SRAM in Protocol Control
    signal rec_dram_word        :in  std_logic_vector(31 downto 0);
    signal rec_dram_addr        :out natural range 0 to 15;
    
    ------------------------------------
    --Status signals of recieve buffer
    ------------------------------------
    
    --Actual size of synthetised message buffer (in 32 bit words)
    signal rx_buf_size          :out std_logic_vector(12 downto 0);
    
    --Signal whenever buffer is full
    signal rx_full              :out std_logic;
    
    --Signal whenever buffer is empty
    signal rx_empty             :out std_logic;
    
    --Number of messaged stored in recieve buffer
    signal rx_message_count     :out std_logic_vector(10 downto 0);
    
    --Number of free 32 bit wide words
    signal rx_mem_free          :out std_logic_vector(12 downto 0);
    
    --Position of read pointer
    signal rx_read_pointer_pos  :out std_logic_vector(11 downto 0);
    
    --Position of write pointer
    signal rx_write_pointer_pos :out std_logic_vector(11 downto 0);
    
    --Message was discarded since Memory is full
    signal rx_message_disc      :out std_logic;
    
    --Some data were discarded, register
    signal rx_data_overrun      :out std_logic;
    
    -- Signals start of frame for timestamp storing
    signal sof_pulse            :in  std_logic;
    
    signal timestamp            :in std_logic_vector(63 downto 0);
    
    ------------------------------------
    --User registers interface
    ------------------------------------
    
    --Actually loaded data for reading
    signal rx_read_buff         :out std_logic_vector(31 downto 0);
    
    --Driving bus from registers
    signal drv_bus              :in std_logic_vector(1023 downto 0)
  );
    
  -----------------------------
  --Driving bus signal aliases
  -----------------------------
	
	--Erase command from driving registers
  signal drv_erase_rx           :std_logic;
  
  --Command to load increase the reading pointer
  signal drv_read_start         :std_logic;
  
  --Clear data OverRun Flag
  signal drv_clr_ovr            :std_logic;
  
  -- Receive Timestamp options
  signal drv_rtsopt             :std_logic;
    
  ------------------
  --FIFO  Memory   
  ------------------
  constant data_width           :natural := 32;  --Word data width
  
  type rx_memory is array(0 to buff_size - 1) of 
                    std_logic_vector(data_width - 1 downto 0); --Memory type
  
  --Memory declaration inferred in SRAM
  signal memory                 :rx_memory;
  
  --Read Pointer for data
  signal read_pointer           :natural range 0 to buff_size-1;
  
  --Write pointer
  signal write_pointer          :natural range 0 to buff_size-1;
  
  --Registered value of read command for eedge detection. 
  signal prev_read              :std_logic;
  --Note :1->0 edge detection is used here, to move to the next 
  --data when the previous read is finished!
  
  --Recieved message was lost because RX Buffer was full
  signal data_overrun_r         :std_logic;
  
  --Counter used for copying Recieved data to recieve buffer
  signal copy_counter           :natural range 0 to 31;
  
  -- Internal data size decoded from received frame
  signal data_size              :natural range 0 to 31; 
  
  -- Combinationally decoded data size from the received frame DLC
  -- (the size is in 32-bit word)
  signal data_size_comb         :natural range 0 to 31;
  
  -- Combinational decoding of the memory words
  signal frame_form_w           :std_logic_vector(31 downto 0);
  
  -- Internal empty Buffer
  signal rx_empty_int           :std_logic;
  
  -- Internal number of free memory words
  signal rx_mem_free_int        :std_logic_vector(12 downto 0);
  
  
  -- Signal that whole frame is stored in the RX Buffer. Active
  -- for one clock cycle only
  signal commit_rx_frame        :std_logic;
  
  -- Number of frames currently stored in the RX Buffer
  -- Smallest frame length stored is 4 (FRAME_FORMAT +  IDENTIFIER + 2 * TIMESTAMP)
  -- Since we need to store 0 and also buff_size/4 values we need one value more
  -- than can fit into buff/size/4 width counter. Use one bit wider counter.
  signal message_count          :natural range 0 to (buff_size / 2) - 1; 
  
  -- Counter for reading the frame. When whole frame is read,
  -- number of frames must be decremented
  signal read_frame_counter     :natural range 0 to 31;
  
  --- Internal timestamp captured for storing
  signal timestamp_capture      :std_logic_vector(63 downto 0);
  
end entity;


architecture rtl of rxBuffer is
begin
  --Driving bus aliases
  drv_erase_rx          <= drv_bus(DRV_ERASE_RX_INDEX);
  drv_read_start        <= drv_bus(DRV_READ_START_INDEX);
  drv_clr_ovr           <= drv_bus(DRV_CLR_OVR_INDEX);
  drv_rtsopt            <= drv_bus(DRV_RTSOPT_INDEX);
  rx_buf_size           <= std_logic_vector(to_unsigned(buff_size, 13));
  
  --Propagating status registers on output
  rx_read_pointer_pos   <= std_logic_vector(to_unsigned(read_pointer, 12));
  rx_write_pointer_pos  <= std_logic_vector(to_unsigned(write_pointer, 12));
  rx_data_overrun       <= data_overrun_r;
  
 
  -- When buffer is empty the word which is on the output is not valid,
  -- provide zeroes instead
  rx_read_buff          <= memory(read_pointer) when (rx_empty_int = '0')
                                                else
                           (OTHERS => '0');
              

  -- Address for the Receive data RAM in the CAN Core! Comparator is temporary 
  -- before the data order will be reversed!
  rec_dram_addr         <= 18-copy_counter when (copy_counter>2
	                                             and
                                                 copy_counter<19)
                                           else
                           0;
  
  -- Receive data size (in words) decoder
  with rec_dlc_in select data_size_comb <=
      3 when "0000", --Zero bits
      4 when "0001", --1 byte
      4 when "0010", --2 bytes
      4 when "0011", --3 bytes
      4 when "0100", --4 bytes
      5 when "0101", --5 bytes
      5 when "0110", --6 bytes
      5 when "0111", --7 bytes
      5 when "1000", --8 bytes
      6 when "1001", --12 bytes
      7 when "1010", --16 bytes
      8 when "1011", --20 bytes
      9 when "1100", --24 bytes
      11 when "1101", --32 bytes
      15 when "1110", --48 bytes
      19 when "1111", --64 bytes
      0  when others;
  
  -- Frame format word assignment
  frame_form_w(DLC_H downto DLC_L)      <= rec_dlc_in;
  frame_form_w(RTR_IND)                 <= rec_is_rtr;
  frame_form_w(ID_TYPE_IND)             <= rec_ident_type_in;
  frame_form_w(FR_TYPE_IND)             <= rec_frame_type_in;
  frame_form_w(TBF_IND)                 <= '1'; -- All frames have the timestamp
  frame_form_w(BRS_IND)                 <= rec_brs;
  frame_form_w(ESI_RESVD_IND)           <= rec_esi;
  frame_form_w(RWCNT_H downto RWCNT_L)  <=
          std_logic_vector(to_unsigned(data_size_comb, (RWCNT_H - RWCNT_L + 1)));
  frame_form_w(31 downto 16)            <= (OTHERS => '0');
  
  
  ------------------------------------------------------------------------------
  -- Capturing timestamp at begining or end of the frame depending on config
  ------------------------------------------------------------------------------
  capt_ts_proc:process(clk_sys, res_n)
  begin
    if (res_n = ACT_RESET) then
      timestamp_capture       <= (OTHERS => '0');
    elsif (rising_edge(clk_sys))then
      timestamp_capture       <= timestamp_capture;
      
      if ( (drv_rtsopt = RTS_END and rec_message_valid = '1') or
           (drv_rtsopt = RTS_BEG and sof_pulse = '1')) 
      then  
          timestamp_capture   <= timestamp;
      end if;

    end if;
  end process;
  
  
  ------------------------------------------------------------------------------
  -- Reading the Frame by user
  ------------------------------------------------------------------------------
  read_frame_proc:process(clk_sys, res_n, drv_erase_rx)
  begin
    if (res_n = ACT_RESET or drv_erase_rx = '1') then
      message_count             <= 0;
      read_frame_counter        <= 0;
    elsif (rising_edge(clk_sys))then
      
      message_count             <= message_count;
      read_frame_counter        <= read_frame_counter;
      
      -- We can start reading only when there already is some frame
      -- committed in the buffer !!
      if ( (drv_read_start = '1') and (rx_empty_int = '0')) then
        
        -- During the read of FRAME_FORMAT word store the length
        -- of the frame to "read_frame_counter", thus we know how much
        -- we have to read before decrementing the "message_count".
        if (read_frame_counter = 0) then
          read_frame_counter    <= 
              to_integer(unsigned(memory(read_pointer)(RWCNT_H downto RWCNT_L)));
        
        -- The last word is read during decrement from 1 to 0. We can decrease
        -- number of frames then, NOT earlier! If decremented earlier, reading of
        -- last frame would get stuck, since read_pointer in memory access is
        -- incremented only with non-zero message count! If "commit_frame_counter"
        -- is '1' we dont decrement since new frame has arrived.
        elsif (read_frame_counter = 1) then  
          if (commit_rx_frame = '0') then
            message_count       <= message_count - 1;
          end if; 
          read_frame_counter    <= read_frame_counter - 1;
        
        -- Just count down during the read of all the remaining words of the frame
        else
           if (commit_rx_frame = '1') then
            message_count       <= message_count + 1;
          end if;
          read_frame_counter    <= read_frame_counter - 1;
          
        end if;
                
      elsif (commit_rx_frame = '1') then 
        message_count           <= message_count + 1;
      end if;
      
    end if;    
  end process;
  
  
  ------------------------------------------------------------------------------
  --Storing data from CANCore and loading data into reading buffer
  ------------------------------------------------------------------------------
  memory_acess:process(clk_sys, res_n, drv_erase_rx)
		
		--Length variable for frame stored into reading buffer (in 32 bit words)
    variable data_length    : natural range 0 to 16;
    
    --Amount of free words
    variable mem_free       : natural range 0 to 2 * buff_size - 1 := buff_size;
    
  begin     
    if (res_n=ACT_RESET) or (drv_erase_rx='1') then
      write_pointer     <= 0;
      read_pointer      <= 0;
      rx_full           <= '0';
      mem_free          := buff_size;
      rx_mem_free_int   <= std_logic_vector(to_unsigned(buff_size, 13));
      commit_rx_frame   <= '0';
      
      --Nulling output signals
      rec_message_ack   <= '0';
      rx_message_disc   <= '0';
      data_overrun_r    <= '0';
      prev_read         <= '0';
      copy_counter      <= 16; 
      data_size         <= 0;
      
      -- Memory can be initialized to zeroes in the simulation
      -- pragma translate_off    
      memory            <= (OTHERS => (OTHERS => '0'));
      -- pragma translate_on
      
    elsif rising_edge(clk_sys) then
      
      prev_read         <= drv_read_start;
      read_pointer      <= read_pointer;
      commit_rx_frame   <= '0';
      
      --Clearing the overRun flag
      if(drv_clr_ovr='1')then
       data_overrun_r   <= '0';
      else  
       data_overrun_r   <= data_overrun_r;
      end if;
      
      --------------------------------------------------------------------------
      --Moving to next word by reading (if there is sth to read)
      --------------------------------------------------------------------------
      if ((drv_read_start = '1') and (rx_empty_int = '0'))then 
        
        --Increase the reading pointer
        read_pointer                <= (read_pointer+1) mod buff_size;
      
        -- Increase amount of free memory
        mem_free                    := mem_free+1;
 
      end if;
       
      
      --------------------------------------------------------------------------
      --Storing recieved message	
      --------------------------------------------------------------------------
      if(rec_message_valid='1')then
          rec_message_ack <= '1'; --Acknowledge message reception for CAN Core
          
          --If frame is RTR we dont CARE about recieved DLC, that can be arbit-
          --rary due to the rtr preffered behaviour! RTR frame, automatically no
          --data are stored!!!
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
        
          if (mem_free > (data_length + 3)) then --Checking if message can be stored
            
            --Writing Frame format Word
            rx_message_disc             <= '0';
            memory(write_pointer)       <= frame_form_w;
            
           --Increasing write pointer
           write_pointer                <= (write_pointer+1) mod buff_size;
           --mem_free                   := mem_free-4-data_length;
           mem_free                     := mem_free-1;
           
           if(rec_is_rtr='1' and rec_frame_type_in='0')then
             data_size    <= 3;
           else
             data_size    <= data_size_comb;
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
        
        ------------------------------------------------------------------------
        --copy_counter decodes which part of received frame to store
        ------------------------------------------------------------------------
        if(copy_counter=0)then
            memory(write_pointer)       <= "000"&rec_ident_in;
        elsif(copy_counter=1)then
            memory(write_pointer)       <= timestamp(31 downto 0);
        elsif(copy_counter=2)then
            memory(write_pointer)       <= timestamp(63 downto 32);
        end if;
        
        write_pointer                   <= (write_pointer+1) mod buff_size;
        copy_counter                    <= copy_counter+1;
        data_size                       <= data_size;
        rec_message_ack                 <= '0';
        
        mem_free                        := mem_free-1;
      
      elsif(copy_counter<data_size)then -- Here the data words are stored
        
        --Optimized implementation of the storing with auxiliarly receive 
        --data RAM
        memory(write_pointer)           <= rec_dram_word;
        
        write_pointer                   <= (write_pointer+1) mod buff_size;
        copy_counter                    <= copy_counter+1;
        data_size                       <= data_size;
        rec_message_ack                 <= '0';
        mem_free                        := mem_free-1;
      
      -- Note that we get here if either all words were stored
      elsif (copy_counter = data_size) then
        commit_rx_frame                 <= '1';
        copy_counter                    <= copy_counter + 1;
        
      else
        rx_message_disc                 <= '0';
        rec_message_ack                 <= '0';
        write_pointer                   <= write_pointer;
        copy_counter                    <= 16;
        data_size                       <= 0;
      end if;
  
  rx_mem_free_int                       <= std_logic_vector(
                                            to_unsigned(mem_free, 13));
  
  --Assigning output whenever memory is full
  if (mem_free=0) then 
    rx_full                             <= '1';
  else
    rx_full                             <= '0';
  end if;
 
  end if;
end process memory_acess;

  --Memory empty output
  rx_empty_int <= '1' when (message_count = 0)
                      else
                  '0'; 
 
  rx_message_count          <= std_logic_vector(to_unsigned(message_count, 11)); 
  rx_mem_free               <= rx_mem_free_int;
  rx_empty                  <= rx_empty_int;

end architecture;