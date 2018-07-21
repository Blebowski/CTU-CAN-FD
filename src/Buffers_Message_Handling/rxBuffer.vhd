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
--    11.5.2018   1. Changed storing of frame to last during the whole frame.
--                2. Added FSM for control of storing
--                3. Added "write_pointer_raw" and "write_pointer_extra_ts" to
--                   execute writes of timestamp at the end of the frame.
--                4. Commit operation moves "raw" pointer to real pointer. Data
--                   overrun restarts "raw" pointer to value of committed
--                   pointer.
--    19.5.2018   1. Added assertion for number of "store_data" commands during
--                   storing of CAN Frame.
--                2. Separated "write_raw_increment" to "write_raw_intent" and
--                   "write_raw_OK" which is valid only when there is enough
--                   space in the buffer and overrun did not occur before!
--    7.6.2018    Changed detection of buffer full to equality of 
--                "read_pointer" and "write_pointer_raw" and nonzero amount
--                of frames stored.
--   10.7.2018    Changed decoding of RWCNT field. Added exception for CAN 
--                frames with DLC higher than 8, to store only 8 bytes (2 data
--                words).
--------------------------------------------------------------------------------

Library ieee;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.ALL;
use work.CANconstants.all;
use work.CAN_FD_frame_format.all;
use work.CAN_FD_register_map.all;

entity rxBuffer is
    generic(

        -- Only 2^k are allowed as buff_size. Memory adressing is in modular 
        -- arithmetic, synthesis of modulo by number other than 2^k might not
        -- play nicely (will consume lot of LUTs)!!
        buff_size                   :       natural range 32 to 4096 := 32
    );
    port(
        ------------------------------------------------------------------------
        -- Clocks and reset 
        ------------------------------------------------------------------------
        signal clk_sys              :in     std_logic; --System clock
        signal res_n                :in     std_logic; --Async. reset
        

        ------------------------------------------------------------------------
        -- Metadata from CAN Core (provided discrete)
        ------------------------------------------------------------------------
        
        -- Message Identifier
        signal rec_ident_in         :in     std_logic_vector(28 downto 0);
        
        -- Data length code
        signal rec_dlc_in           :in     std_logic_vector(3 downto 0);
        
        -- Recieved identifier type (0-BASE Format, 1-Extended Format);
        signal rec_ident_type_in    :in     std_logic;
        
        -- Recieved frame type (0-Normal CAN, 1- CAN FD)
        signal rec_frame_type_in    :in     std_logic;
        
        -- Recieved frame is RTR Frame(0-No, 1-Yes)
        signal rec_is_rtr           :in     std_logic;
        
        -- Whenever frame was recieved with BIT Rate shift 
        signal rec_brs              :in     std_logic;

        -- Recieved error state indicator
        signal rec_esi              :in     std_logic;


        ------------------------------------------------------------------------
        -- Control signals from CAN Core which control storing of CAN Frame.
        ------------------------------------------------------------------------

        -- After control field of CAN frame, metadata are valid and can be stored.
        -- This command starts the RX FSM for storing.
        signal store_metadata       :in     std_logic;
       
        -- Signal that one word of data can be stored (TX_DATA_X_W). This signal
        -- is active when 4 bytes were received or data reception has finished 
        -- on 4 byte unaligned number of frames! (Thus allowing to store also
        -- data which are not 4 byte aligned!
        signal store_data           :in     std_logic;

        -- Data word which should be stored when "store_data" is active!
        signal store_data_word      :in     std_logic_vector(31 downto 0);

        -- When frame reception is succesfull, in the end of EOF field, CAN Core
        -- gives acknowledge by this signal. This means that frame was received
        -- OK, and Error frame can no longer occur in it.
        signal rec_message_valid    :in     std_logic;
        
        -- If error frame occurred, CAN Core activates this signal.
        -- "write_pointer_raw" will be restarted to last committed value in
        -- "write_pointer".
        signal rec_abort            :in     std_logic;

        -- Signals start of frame. If timestamp on RX frame should be captured
        -- in the beginning of the frame, this pulse captures the timestamp!
        signal sof_pulse            :in     std_logic;


        ------------------------------------
        -- Status signals of recieve buffer
        ------------------------------------
        
        -- Actual size of synthetised message buffer (in 32 bit words)
        signal rx_buf_size          :out    std_logic_vector(12 downto 0);
        
        -- Signal whenever buffer is full (no free memory words)
        signal rx_full              :out    std_logic;
        
        -- Signal whenever buffer is empty (no frame (message) is stored)
        signal rx_empty             :out    std_logic;
        
        -- Number of frames (messages) stored in recieve buffer
        signal rx_message_count     :out    std_logic_vector(10 downto 0);
        
        -- Number of free 32 bit wide words
        signal rx_mem_free          :out    std_logic_vector(12 downto 0);
        
        -- Position of read pointer
        signal rx_read_pointer_pos  :out    std_logic_vector(11 downto 0);
        
        -- Position of write pointer
        signal rx_write_pointer_pos :out    std_logic_vector(11 downto 0);
        
        -- Overrun occurred, data were discarded!
        -- (This is a flag and persists until it is cleared by SW)! 
        signal rx_data_overrun      :out    std_logic;
        
        -- External timestamp input
        signal timestamp            :in     std_logic_vector(63 downto 0);


        ------------------------------------
        -- User registers interface
        ------------------------------------
        
        -- Actually loaded data for reading
        signal rx_read_buff         :out    std_logic_vector(31 downto 0);
        
        -- Driving bus from registers
        signal drv_bus              :in     std_logic_vector(1023 downto 0)
    );

    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Driving bus signal aliases
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    -- Erase command from driving registers. Resets FIFO pointers!
    signal drv_erase_rx             :       std_logic;

    -- Command to load increase the reading pointer
    signal drv_read_start           :       std_logic;

    -- Clear data OverRun Flag
    signal drv_clr_ovr              :       std_logic;

    -- Receive Timestamp options
    signal drv_rtsopt               :       std_logic;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- FIFO  Memory - Declaration and access
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    -- Memory Word data width
    constant data_width             :       natural := 32;

    -- Memory type
    type rx_memory_type is array(0 to buff_size - 1) of 
                    std_logic_vector(data_width - 1 downto 0); 

    -- Memory declaration inferred in SRAM
    signal memory                   :       rx_memory_type;

    -- Read Pointer (access from SW)
    signal read_pointer             :       natural range 0 to buff_size - 1;

    -- Write pointer (committed, available to SW, after frame was stored)
    signal write_pointer            :       natural range 0 to buff_size - 1;

    -- Write pointer RAW. Changing during frame, as frame is continously stored
    -- to the buffer. When frame is sucesfully received, it is updated to
    -- write pointer!
    signal write_pointer_raw        :       natural range 0 to buff_size - 1;

    -- Extra write pointer which is used for storing timestamp at the end of
    -- data frame!
    signal write_pointer_extra_ts   :       natural range 0 to buff_size - 1;

    -- Final pointer to memory. "write_pointer_raw" and 
    -- "write_pointer_extra_ts" are multiplexed based on RX FSM! 
    signal memory_write_pointer     :       natural range 0 to buff_size - 1;

    -- Data that will be written to the RX Buffer memory!
    signal memory_write_data        :       std_logic_vector(31 downto 0);


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- FIFO  Memory - Free words, Overrun status
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    -- Data overrun flag. Recieved message was lost, because there was not
    -- enough space in FIFO during storing! Available for SW!
    signal data_overrun_r           :       std_logic;

    -- Internal data overrun flag. This flag is not available to SW, but it
    -- is restarted automatically at the beginning of each new frame reception!
    -- This allows to accept next frames when overrun occurred on previous ones!
    signal data_overrun_int         :       std_logic;

    -- Combinationally decoded overrun condition. Active when there is intention
    -- to store word to the memory, but there is not enough free space! 
    signal overrun_condition        :       boolean;

     -- RX Buffer is empty (no frame is stored in it)
    signal rx_empty_int             :       std_logic;

    -- Number of free memory words available to SW after frame was committed.
    signal rx_mem_free_int          :       natural range 0 to buff_size;

    -- Number of free memory words calculated during frame storing, before
    -- commit.
    signal rx_mem_free_raw          :       natural range 0 to buff_size;

    -- Indicator of at least one free word in RX FIFO!
    signal is_free_word             :       boolean;

    -- Number of frames currently stored in RX Buffer. Smallest frame length
    -- stored is 4 (FRAME_FORMAT +  IDENTIFIER + 2 * TIMESTAMP). Since we need
    -- to store 0 and also buff_size / 4 values we need one value more than can
    -- fit into buff_size / 4 width counter. Use one bit wider counter.
    signal message_count            :       natural range 0 to (buff_size / 2) - 1; 

    -- Counter for reading the frame. When whole frame is read,
    -- number of frames must be decremented.
    signal read_frame_counter       :       natural range 0 to 31;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- FIFO  Memory - Commands which manipulate pointers, or indicate intent
    --  to write or read from the memory.
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    -- When commit of RX Frame is signalled by CAN Core (rec_message_valid)
    -- "commit_rx_frame" is set to indicate that frame was sucesfully
    -- stored. Note that "rec_message_valid" is not enough to indicate that
    -- frame was stored sucesfully! If frame storing fails at some point due
    -- to lack of memory in FIFO, Protocol control will still finish the frame
    -- and provide "rec_message_valid"! Thus RX FSM sets "commit_rx_frame" only
    -- if "data_overrun" did not occur during the frame!
    signal commit_rx_frame          :       std_logic;

    -- When overrun occurred at any point in the frame and some word was not
    -- stored, frame can not be committed, and write_pointer must be moved
    -- back to last committed value!
    signal commit_overrun_abort     :       std_logic;

    -- Indicates that read occurred, and that it is valid (there is something
    -- to read), thus read pointer can be incremented.
    signal read_increment           :       boolean;

    -- Indicates that FSM is in a state which would like to perform write of a
    -- word to RX Buffer memory!
    signal write_raw_intent         :       boolean;

    -- Indicates that "write_raw_intent" is OK (no overrun) and data can be
    -- truly written to the memory and raw pointer can be updated!
    signal write_raw_OK             :       boolean;

    -- Indicates that FSM is in one of states for writing timestmap from end of
    -- frame to the memory.
    signal write_extra_ts           :       boolean;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- RX FSM, Timestamp capturing, combinationally decoded words
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    -- Combinationally decoded size of the frame (without Frame format word)
    -- from received DLC (the size is in 32-bit words).
    signal rwcnt_com                :       natural range 0 to 31;

    -- Combinational decoded frame format word from metadata.
    signal frame_form_w             :       std_logic_vector(31 downto 0);

    -- FSM for storing the frame during the reception.
    signal rx_fsm                   :       rx_buf_fsm_type;

    -- Internal timestamp captured for storing. Captured either in the
    -- beginning or end of frame. 
    signal timestamp_capture        :       std_logic_vector(63 downto 0);

end entity;


architecture rtl of rxBuffer is
begin

    ----------------------------------------------------------------------------
    -- Driving bus aliases
    ----------------------------------------------------------------------------
    drv_erase_rx          <= drv_bus(DRV_ERASE_RX_INDEX);
    drv_read_start        <= drv_bus(DRV_READ_START_INDEX);
    drv_clr_ovr           <= drv_bus(DRV_CLR_OVR_INDEX);
    drv_rtsopt            <= drv_bus(DRV_RTSOPT_INDEX);
    

    ----------------------------------------------------------------------------
    -- Propagating status registers on output
    ----------------------------------------------------------------------------
    rx_read_pointer_pos   <= std_logic_vector(to_unsigned(read_pointer, 12));
    rx_write_pointer_pos  <= std_logic_vector(to_unsigned(write_pointer, 12));
    rx_data_overrun       <= data_overrun_r;
    rx_buf_size           <= std_logic_vector(to_unsigned(buff_size, 13));

    rx_empty_int          <= '1' when (message_count = 0)
                                 else
                             '0'; 

    rx_full               <= '1' when (rx_mem_free_int = 0)
                                 else
                             '0';

    rx_message_count      <= std_logic_vector(to_unsigned(message_count, 11)); 
    rx_mem_free           <= std_logic_vector(to_unsigned(rx_mem_free_int, 13));
    rx_empty              <= rx_empty_int;



    ----------------------------------------------------------------------------
    -- Signalling that read which came is valid (there is sth to read)
    ----------------------------------------------------------------------------
    read_increment        <= true when (drv_read_start = '1' and
                                        rx_empty_int = '0')
                                  else
                             false;


    ----------------------------------------------------------------------------
    -- Signalling that FSM is writing the data to the memory
    ----------------------------------------------------------------------------
    write_raw_intent      <= true 
                             when 
                              ((rx_fsm = rxb_store_frame_format) or
                               (rx_fsm = rxb_store_identifier) or
                               (rx_fsm = rxb_store_beg_ts_low) or
                               (rx_fsm = rxb_store_beg_ts_high) or
                               ((rx_fsm = rxb_store_data) and store_data = '1'))
                             else
                             false;

    write_extra_ts        <= true when ((rx_fsm = rxb_store_end_ts_low) or
                                        (rx_fsm = rxb_store_end_ts_high)) and
                                        (data_overrun_int = '0')
                                  else
                             false;

    ----------------------------------------------------------------------------
    -- Signalling that FSM may progress with the write (there is enough space
    -- in the buffer, nor any previous data were lost due to overrun)
    ----------------------------------------------------------------------------
    write_raw_OK         <= true when (write_raw_intent and
                                       overrun_condition = false and
                                       data_overrun_int = '0')
                                 else
                            false;

    ----------------------------------------------------------------------------
    -- Store of new word can be executed only if there is space in the buffer.
    -- We don't need exact amount of words. We only need to know if there is
    -- space! When "read_pointer" and "write_pointer_raw" are equal, then 
    -- memory is either empty, or full! If there is no frame stored and pointers
    -- are equal, then memory is empty! If there is at least one frame and
    -- pointers are equal, then memory must be full!
    ----------------------------------------------------------------------------
    is_free_word          <= false when (read_pointer = write_pointer_raw and
                                        message_count > 0)
                                  else
                             true;

    ----------------------------------------------------------------------------
    -- Overrun condition. Following conditions must be met:
    --  1. FSM wants to write to memory either to the position of
    --      "write_pointer_raw". Note that "write_pointer_extra_ts" writes to
    --      words which were already written, thus there is no need to watch
    --      for overrun!
    --  2. There is no free word in the memory remaining!
    ----------------------------------------------------------------------------
    overrun_condition    <= true when (write_raw_intent and 
                                       (is_free_word = false))
                                 else
                            false;


    ----------------------------------------------------------------------------
    -- When buffer is empty the word on address of read pointer is not valid,
    -- provide zeroes instead
    ----------------------------------------------------------------------------
    rx_read_buff          <= memory(read_pointer) when (rx_empty_int = '0')
                                                  else
                             (OTHERS => '0');


    ----------------------------------------------------------------------------
    -- Final write pointer is multiplexed between "write_pointer_raw" for
    -- regular writes and "write_pointer_extra_ts" for writes of timestamp
    -- in the end of frame!
    ----------------------------------------------------------------------------
    memory_write_pointer   <= write_pointer_extra_ts when 
                                    ((rx_fsm = rxb_store_end_ts_low) or
                                     (rx_fsm = rxb_store_end_ts_high))
                                                    else
                             write_pointer_raw;


    ----------------------------------------------------------------------------
    -- Memory data which are written depend on state of the FSM
    ----------------------------------------------------------------------------
    with rx_fsm select memory_write_data <=
        frame_form_w                     when rxb_store_frame_format,
        "000" & rec_ident_in             when rxb_store_identifier,
        timestamp_capture(31 downto 0)   when rxb_store_beg_ts_low,
        timestamp_capture(31 downto 0)   when rxb_store_end_ts_low,
        timestamp_capture(63 downto 32)  when rxb_store_beg_ts_high,
        timestamp_capture(63 downto 32)  when rxb_store_end_ts_high,
        store_data_word                  when rxb_store_data,
        (OTHERS => '0')                  when others;



    ----------------------------------------------------------------------------
    -- Receive data size (in words) decoder
    ----------------------------------------------------------------------------
    with rec_dlc_in select rwcnt_com <=
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


    ----------------------------------------------------------------------------
    -- Frame format word assignment
    ----------------------------------------------------------------------------
    frame_form_w(DLC_H downto DLC_L)      <= rec_dlc_in;
    frame_form_w(4)                       <= '0';
    frame_form_w(RTR_IND)                 <= rec_is_rtr;
    frame_form_w(ID_TYPE_IND)             <= rec_ident_type_in;
    frame_form_w(FR_TYPE_IND)             <= rec_frame_type_in;
    frame_form_w(TBF_IND)                 <= '1'; -- All frames have the timestamp
    frame_form_w(BRS_IND)                 <= rec_brs;
    frame_form_w(ESI_RESVD_IND)           <= rec_esi;

    ----------------------------------------------------------------------------
    -- RWCNT (Read word count is calculated like so:
    --  1. For RTR Frames -> 3 (Only ID + 2 Timestamp words)
    --  2. For Normal CAN Frames with DLC > 8 max. 8 bytes -> RWCNT = 5
    --  3. Otherwise Number of data bytes is matching Received DLC!
    ----------------------------------------------------------------------------
    frame_form_w(RWCNT_H downto RWCNT_L)  <=
        "00011" when (rec_is_rtr = RTR_FRAME) else
        "00101" when ((rec_frame_type_in = NORMAL_CAN) and (rec_dlc_in(3) = '1')) else
         std_logic_vector(to_unsigned(rwcnt_com, (RWCNT_H - RWCNT_L + 1)));

    frame_form_w(31 downto 16)            <= (OTHERS => '0');


    ----------------------------------------------------------------------------
    -- Capturing timestamp. Done at the beginning or end of frame based on
    -- SW configuration.
    ----------------------------------------------------------------------------
    capt_ts_proc : process(clk_sys, res_n)
    begin
        if (res_n = ACT_RESET) then
            timestamp_capture       <= (OTHERS => '0');

        elsif (rising_edge(clk_sys)) then
            
            if ((drv_rtsopt = RTS_END and rec_message_valid = '1') or
                (drv_rtsopt = RTS_BEG and sof_pulse = '1')) 
            then  
                timestamp_capture   <= timestamp;
            end if;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Reading counter (read_frame_counter) which is loaded by RWCNT 
    -- during read of frame format word. Then each next read decreases
    -- the counter. When read counter reaches zero, message count is
    -- decreased. If "commit_rx_frame" comes from RX FSM, "message_count"
    -- is incremented. If both occur at the same time, "message_count"
    -- does not change.
    ----------------------------------------------------------------------------
    read_frame_proc : process(clk_sys, res_n, drv_erase_rx)
    begin
        if (res_n = ACT_RESET or drv_erase_rx = '1') then
            message_count                <= 0;
            read_frame_counter           <= 0;

        elsif (rising_edge(clk_sys)) then

            message_count                <= message_count;
            read_frame_counter           <= read_frame_counter;

            --------------------------------------------------------------------
            -- Reading frame by user when there is active read and there is
            -- something to read
            --------------------------------------------------------------------
            if (read_increment) then

                ----------------------------------------------------------------
                -- During the read of FRAME_FORMAT word store the length
                -- of the frame to "read_frame_counter", thus we know how much
                -- we have to read before decrementing the "message_count".
                ----------------------------------------------------------------
                if (read_frame_counter = 0) then
                    read_frame_counter    <=
                        to_integer(unsigned(memory(read_pointer)
                                                  (RWCNT_H downto RWCNT_L)));

                else
                    read_frame_counter <= read_frame_counter - 1;
                end if;
            end if;

            --------------------------------------------------------------------
            -- Manipulation of "message_count". When last word is read from
            -- frame (read_frame_counter = 1 and read_increment), "message_count"
            -- is decreased, when new frame is committed, message count
            -- is increased. If both at the same time, no change since one frame
            -- is added, next is removed!
            --------------------------------------------------------------------
            if (read_increment and (read_frame_counter = 1)) then
                if (commit_rx_frame = '0') then
                    message_count           <= message_count - 1;
                end if;

            elsif (commit_rx_frame = '1') then
                message_count               <= message_count + 1;
            end if;

        end if;    
    end process;


    ----------------------------------------------------------------------------
    -- FSM for controlling storing of the frame to RX Buffer FIFO.
    ----------------------------------------------------------------------------
    rx_buf_fsm : process(clk_sys, res_n, drv_erase_rx)
    begin     
        if (res_n = ACT_RESET) or (drv_erase_rx = '1') then
            rx_fsm              <= rxb_idle;
          
        elsif rising_edge(clk_sys) then
  
            case rx_fsm is

            --------------------------------------------------------------------
            -- Idle, waiting for "store_metada" to start storing first 4 words.
            --------------------------------------------------------------------
            when rxb_idle =>
                if (store_metadata = '1') then
                    rx_fsm      <= rxb_store_frame_format;
                end if;


            --------------------------------------------------------------------
            -- Storing FRAME_FORM_W. Proceed execpt if error ocurrs.
            --------------------------------------------------------------------
            when rxb_store_frame_format =>
                if (rec_abort = '1') then
                    rx_fsm      <= rxb_idle;
                else
                    rx_fsm      <= rxb_store_identifier;
                end if;


            --------------------------------------------------------------------
            -- Storing IDENTIFIER_W.
            -- Move to storing timestamp words. Note that if SW configured
            -- timestamp from end of the frame, we dont have it yet! We store
            -- invalid timestamp and later (if the frame is received OK), we
            -- repeat the writes with timestamp captured at the end of frame!
            --------------------------------------------------------------------
            when rxb_store_identifier =>
                if (rec_abort = '1') then
                    rx_fsm      <= rxb_idle;
                else
                    rx_fsm      <= rxb_store_beg_ts_low;
                end if;


            --------------------------------------------------------------------
            -- Store TIMESTAMP_L_W from beginning of frame.
            --------------------------------------------------------------------
            when rxb_store_beg_ts_low =>
                if (rec_abort = '1') then
                    rx_fsm      <= rxb_idle;
                else
                    rx_fsm      <= rxb_store_beg_ts_high;
                end if;


            --------------------------------------------------------------------
            -- Store first TIMESTAMP_U_W from beginning of frame.
            --------------------------------------------------------------------
            when rxb_store_beg_ts_high =>
                if (rec_abort = '1') then
                    rx_fsm      <= rxb_idle;
                else                
                    rx_fsm      <= rxb_store_data;
                end if;

            --------------------------------------------------------------------
            -- Store DATA_W. If error ocurrs, abort the storing. If storing is
            -- finished, go to idle or again timestamp storing depending on the
            -- timestamp option configuration. Note that now timestamp storing
            -- is realized via different states!
             --------------------------------------------------------------------
            when rxb_store_data =>
                if (rec_abort = '1') then 
                    rx_fsm          <= rxb_idle;
                elsif (rec_message_valid = '1') then
                    if (drv_rtsopt = RTS_END) then
                        rx_fsm      <= rxb_store_end_ts_low;
                    else
                        rx_fsm      <= rxb_idle; 
                    end if;
                end if;


            --------------------------------------------------------------------
            -- Store TIMESTAMP_L_W from end of frame.
            --------------------------------------------------------------------
            when rxb_store_end_ts_low =>
                rx_fsm      <= rxb_store_end_ts_high;


            --------------------------------------------------------------------
            -- Store first TIMESTAMP_U_W from end of frame.
            --------------------------------------------------------------------
            when rxb_store_end_ts_high =>
                rx_fsm      <= rxb_idle;

            when others =>
                report "Invalid RX Buffer state" severity error;
            end case;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Commit RX Frame when last word was written and overrun did not occur!
    -- This can be either from "rxb_store_data" state or "rxb_store_end_ts_high"
    ----------------------------------------------------------------------------
    commit_proc : process(res_n, clk_sys)
    begin
        if (res_n = ACT_RESET) then
            commit_rx_frame       <= '0';
            commit_overrun_abort  <= '0';

        elsif (rising_edge(clk_sys)) then

            if (((rec_message_valid = '1' and drv_rtsopt = RTS_BEG) or
                 (rx_fsm = rxb_store_end_ts_high)))
            then
                if (data_overrun_int = '0') then
                    commit_rx_frame         <= '1';
                else
                    commit_overrun_abort    <= '1';
                end if;
            else
                commit_rx_frame             <= '0';
                commit_overrun_abort        <= '0';
            end if;

        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Process for pointer manipulation. Takes care of:
    --   1. Incrementing "read_pointer" during read.
    --   2. Incrementing "write_pointer_raw" during write.
    --   3. Committing "write_pointer_raw" to "write_pointer" and vice versa.
    --   4. Setting "write_pointer_extra_ts" for write of timestamp from end
    --      of frame.
    --   5. Update of "mem_free".
    ----------------------------------------------------------------------------
    pointer_proc : process(clk_sys, res_n, drv_erase_rx)
    begin
        if (res_n = ACT_RESET or drv_erase_rx = '1') then
            read_pointer            <= 0;
            write_pointer           <= 0;
            write_pointer_raw       <= 0;
            write_pointer_extra_ts  <= 0;

            rx_mem_free_int         <= buff_size;
            rx_mem_free_raw         <= buff_size;

        elsif (rising_edge(clk_sys)) then

            --------------------------------------------------------------------
            -- Moving to next word by reading (if there is sth to read).
            --------------------------------------------------------------------
            if (read_increment) then
                read_pointer        <= (read_pointer + 1) mod buff_size;
            end if;

            --------------------------------------------------------------------
            -- Loading "write_pointer_raw" to  "write_pointer" when frame is
            -- committed.
            --------------------------------------------------------------------
            if (commit_rx_frame = '1') then
                write_pointer       <= write_pointer_raw;
            end if;

            --------------------------------------------------------------------
            -- Updating "write_pointer_raw":
            --      1. Increment when word is written to memory.
            --      2. Reset when "rec_abort" is active (Error frame) or 
            --         frame finished and overrun occurred meanwhile. Reset to
            --         value of last commited write pointer.
            --------------------------------------------------------------------
            if (write_raw_OK) then
                write_pointer_raw   <= (write_pointer_raw + 1) mod buff_size;
                        
            elsif (rec_abort = '1' or commit_overrun_abort = '1') then
                write_pointer_raw   <= write_pointer;

            else
                write_pointer_raw   <= write_pointer_raw;
            end if;

            --------------------------------------------------------------------
            -- Setting extra write pointer for write of timestamp from end of
            -- frame...
            -- First timestamp word is offset by 3 from committed write pointer.
            -- Second one is offset by 4.
            --------------------------------------------------------------------
            if (rx_fsm = rxb_store_data) then
                write_pointer_extra_ts    <= (write_pointer + 2) mod buff_size;

            elsif (rx_fsm = rxb_store_end_ts_low) then
                write_pointer_extra_ts    <= (write_pointer_extra_ts + 1) mod
                                              buff_size;

            else
                write_pointer_extra_ts    <= write_pointer_extra_ts;
            end if;


            --------------------------------------------------------------------
            -- Calculate free memory internally (raw)
            --------------------------------------------------------------------
            if (read_increment) then
                if (rec_abort = '1' or commit_overrun_abort = '1') then
                    rx_mem_free_raw <= rx_mem_free_int + 1;
                elsif (not write_raw_OK) then
                    rx_mem_free_raw <= rx_mem_free_raw + 1;
                end if;
            else
                if (rec_abort = '1' or commit_overrun_abort = '1') then
                    rx_mem_free_raw <= rx_mem_free_int;
                elsif (write_raw_OK) then
                    rx_mem_free_raw <= rx_mem_free_raw - 1;
                end if;
            end if;

            --------------------------------------------------------------------
            -- Calculate free memory for user:
            --      1. Increment when user reads the frame.
            --      2. Load RAW value when comitt occurs
            --------------------------------------------------------------------
            if (read_increment) then
                if (commit_rx_frame = '1') then        
                    rx_mem_free_int     <= rx_mem_free_raw + 1;
                else
                    rx_mem_free_int     <= rx_mem_free_int + 1;
                end if;
    
            elsif (commit_rx_frame = '1') then
                rx_mem_free_int         <= rx_mem_free_raw;
            end if;

        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Calculation of data overrun flag. If FSM would like to write to the
    -- memory, and there is not enough free space, data overrun flag will be
    -- set, and no further writes will be executed.
    ----------------------------------------------------------------------------
    dor_proc : process(res_n, clk_sys, drv_erase_rx)
    begin
        if (res_n = ACT_RESET or drv_erase_rx = '1') then
            data_overrun_r      <= '0';
            data_overrun_int    <= '0';

        elsif (rising_edge(clk_sys)) then

            --------------------------------------------------------------------
            -- Internal Data overrun flag -> cleared by new frame!
            --------------------------------------------------------------------
            if (rx_fsm = rxb_idle) then
                data_overrun_int    <= '0';

            elsif (overrun_condition) then
                data_overrun_int    <= '1';
            else
                data_overrun_int    <= data_overrun_int;
            end if;

            --------------------------------------------------------------------
            -- SW overrun flag -> Cleared from SW!
            --------------------------------------------------------------------
            if (drv_clr_ovr = '1') then
                data_overrun_r  <= '0';
 
            elsif (overrun_condition) then
                data_overrun_r  <= '1';
            else
                data_overrun_r  <= data_overrun_r;
            end if;

        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Memory access process
    ----------------------------------------------------------------------------
    mem_acc_proc : process(res_n, clk_sys)
    begin
        if (res_n = ACT_RESET) then
            -- Memory is initialized to zeroes in simulation!
            -- pragma translate_off    
            memory            <= (OTHERS => (OTHERS => '0'));
            -- pragma translate_on

        elsif (rising_edge(clk_sys)) then
            
            -- Write to memory either by regular pointer or by "extra timestamp"
            -- pointer.
            if (write_raw_OK or write_extra_ts) then
                memory(memory_write_pointer) <= memory_write_data;    
            end if;

        end if;
    end process;



    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Assertions
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------


    ----------------------------------------------------------------------------
    -- RX Buffer size can be only powers of 2. Since modulo arithmetics is used
    -- on memory pointers, using non power of 2 value would result in increased
    -- logic usage!
    ----------------------------------------------------------------------------
    assert ((buff_size = 32) or
            (buff_size = 64) or
            (buff_size = 128) or
            (buff_size = 256) or
            (buff_size = 512) or
            (buff_size = 1024) or
            (buff_size = 2048) or
            (buff_size = 4096))
    report "Unsupported RX Buffer size! RX Buffer must be power of 2!"
    severity failure;


    ----------------------------------------------------------------------------
    -- Verification of commands applied on RX Buffer. Following must be always
    -- kept, otherwise CAN Core would corrupt storing protocol:
    --  1. "store_metadata" comes during "rx_fsm_idle".
    --  2. "rec_message_valid" and "rec_data" may come ONLY during "store_data"!
    --  3. "sof_pulse" should come only during "rx_fsm_idle".
    --  4. "store_metadata", "store_data", "rec_message_valid", "rec_abort"
    --      should be mutually exclusive!
    ----------------------------------------------------------------------------
    cmd_assert_proc : process(clk_sys)
        variable cmd_join  : std_logic_vector(3 downto 0);
    begin
        -- pragma translate_off
        if (rising_edge(clk_sys) and now /= 0 fs) then
            if (store_metadata = '1' and rx_fsm /= rxb_idle) then
            	-- LCOV_EXCL_START
                report "RX Buffer: Store metadata command did NOT come during " &
                       "'rx_buf_idle'!"severity error;
            	-- LCOV_EXCL_STOP
            end if;

            if ((rec_message_valid = '1' or store_data = '1') and
                 rx_fsm /= rxb_store_data)
            then
            	-- LCOV_EXCL_START
                report "RX Buffer: Store data or finish storing did NOT come " &
                        "during 'rec_data'" severity error; 
            	-- LCOV_EXCL_STOP            
            end if;

            if (sof_pulse = '1' and rx_fsm /= rxb_idle) then
                -- LCOV_EXCL_START
                report "RX Buffer: SOF pulse should come during 'rx_fsm_idle'"
                severity error;
            	-- LCOV_EXCL_STOP
            end if;

            cmd_join := store_metadata & store_data & rec_message_valid &
                        rec_abort; 
            if (cmd_join /= "0000" and cmd_join /= "0001" and cmd_join /= "0010"
                and cmd_join /= "0100" and cmd_join /= "1000")
            then
                -- LCOV_EXCL_START
                report "RX Buffer: One-hot coding on RX Buffer commands " &
                        "corrupted!" severity error;
                -- LCOV_EXCL_STOP
            end if;
        end if;
        -- pragma translate_on
    end process;


    ----------------------------------------------------------------------------
    -- Storing sequence is like so:
    --  1. Store metadata.
    --  2. Store data "n" times, n = ceil(data_length / 4). De-facto RWCNT field
    --     contains number of remaining words (apart from FRAME_FORMAT_W). Thus,
    --     RWCNT - 3 = number of expected data words.
    --  3. Get "rec_abort" or "rec_message_valid" command.
    --
    --  This process verifies that "rec_data" command comes expected number of
    --  times (RWCNT - 3). This verifies consistency of storing protocol by
    --  CAN Core, as well as RWCNT field! 
    ----------------------------------------------------------------------------
    rwcnt_assert_proc : process(clk_sys)
        variable exp_data_stores    : natural := 0;
        variable act_data_stores   : natural := 0;
    begin
        -- pragma translate_off
        if (rising_edge(clk_sys) and now /= 0 fs) then

            -- Calculate number of expected "store_data" commands from CAN Core.
            if (rec_abort = '1') then
                exp_data_stores := 0;
                act_data_stores := 0;

            elsif (store_metadata = '1') then

                exp_data_stores := to_integer(unsigned(
                                    frame_form_w(RWCNT_H downto RWCNT_L))) - 3;
                act_data_stores := 0;
            end if;

            -- Count actual number of "store_data" commands.
            if (store_data = '1') then
                act_data_stores := act_data_stores + 1;
            end if;

            -- Check when frame was received that proper number of "store_data"
            -- commands did arrive.
            if (rec_message_valid = '1' and 
                act_data_stores /= exp_data_stores)
            then
                report "'store_data' count corrupted by CAN Core! " &
                       "Expected: " & integer'image(exp_data_stores) &
                       "  Actual: " & integer'image(act_data_stores)
                severity error;
            end if;
        end if;
        -- pragma translate_on
    end process;


    ----------------------------------------------------------------------------
    -- Checking consistency of "mem_free" calculation.
    -- If buffer is idle (no storing is in progress), and message_count is 0,
    -- then the buffer should be completely empty, thus, mem_free should be
    -- equal to size of buffer.
    ----------------------------------------------------------------------------
    --mem_free_calc_process : process(clk_sys)
    --begin
        -- pragma translate_off
    --    if (rising_edge(clk_sys) and now /= 0 fs) then

--            if (rx_fsm = rxb_idle and message_count = 0 and rx_mem_free_int /=
--                buff_size and commit_rx_frame = '0')
--            then
--                report "Buffer should be empty, but 'rx_mem_free_raw' is " &
--                       "not equal to 'buff_size'" severity error;
--            end if;
--            
--        end if;
        -- pragma translate_on
--    end process;

end architecture;
