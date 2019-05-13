--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018
-- 
-- Authors:
--     Ondrej Ille <ondrej.ille@gmail.com>
--     Martin Jerabek <martin.jerabek01@gmail.com>
-- 
-- Project advisors: 
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 
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
--  buffer is sequential operation started with valid rec_valid for one
--  clock cycle. In following up to 20 clock cycles recieved data has to be va-
--  lid to be fully stored
--
--  Note:This is guaranteed from CAN Core. rec_valid is active in the 
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
--   31.8.2018    Changed reading of RAM to be synchronous to achieve BRAM
--                inferrence on Xilinx! Added register "read_mem_word" to which
--                RAM word is read.
--  15.12.2018    1. Moved RX Buffer FSM to separate entity.
--                2. Moved handling of memory pointers to separate entity
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library work;
use work.id_transfer.all;
use work.can_constants.all;
use work.can_components.all;
use work.can_types.all;
use work.cmn_lib.all;
use work.drv_stat_pkg.all;
use work.reduce_lib.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

entity rx_buffer is
    generic(
        -- Reset polarity
        G_RESET_POLARITY            :       std_logic := '0';
        
        -- RX Buffer size
        G_RX_BUFF_SIZE              :       natural range 32 to 4096 := 32
    );
    port(
        ------------------------------------------------------------------------
        -- Clocks and Asynchronous reset 
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              :in     std_logic;
        
        -- Async. reset
        res_n                :in     std_logic;

        ------------------------------------------------------------------------
        -- Metadata from CAN Core
        ------------------------------------------------------------------------
        -- Frame Identifier
        rec_ident            :in     std_logic_vector(28 downto 0);
        
        -- Data length code
        rec_dlc              :in     std_logic_vector(3 downto 0);
        
        -- Recieved identifier type (0-BASE Format, 1-Extended Format);
        rec_ident_type       :in     std_logic;
        
        -- Recieved frame type (0-Normal CAN, 1- CAN FD)
        rec_frame_type       :in     std_logic;
        
        -- Recieved frame is RTR Frame(0-No, 1-Yes)
        rec_is_rtr           :in     std_logic;
        
        -- Whenever frame was recieved with BIT Rate shift 
        rec_brs              :in     std_logic;

        -- Recieved error state indicator
        rec_esi              :in     std_logic;

        ------------------------------------------------------------------------
        -- Control signals from CAN Core which control storing of CAN Frame.
        -- (Filtered by Frame Filters)
        ------------------------------------------------------------------------
        -- After control field of CAN frame, metadata are valid and can be stored.
        -- This command starts the RX FSM for storing.
        store_metadata_f     :in     std_logic;
       
        -- Signal that one word of data can be stored (TX_DATA_X_W). This signal
        -- is active when 4 bytes were received or data reception has finished 
        -- on 4 byte unaligned number of frames! (Thus allowing to store also
        -- data which are not 4 byte aligned!
        store_data_f         :in     std_logic;

        -- Data word which should be stored when "store_data" is active!
        store_data_word      :in     std_logic_vector(31 downto 0);

        -- Received frame valid (commit RX Frame)
        rec_valid_f          :in     std_logic;
        
        -- Abort storing of RX Frame to RX Buffer.
        rec_abort_f          :in     std_logic;

        -- Signals start of frame. If timestamp on RX frame should be captured
        -- in the beginning of the frame, this pulse captures the timestamp!
        sof_pulse            :in     std_logic;

        -----------------------------------------------------------------------
        -- Status signals of RX buffer
        -----------------------------------------------------------------------
        -- Actual size of synthetised message buffer (in 32 bit words)
        rx_buf_size          :out    std_logic_vector(12 downto 0);
        
        -- Signal whenever buffer is full (no free memory words)
        rx_full              :out    std_logic;
        
        -- Signal whenever buffer is empty (no frame (message) is stored)
        rx_empty             :out    std_logic;
        
        -- Number of frames (messages) stored in recieve buffer
        rx_message_count     :out    std_logic_vector(10 downto 0);
        
        -- Number of free 32 bit wide words
        rx_mem_free          :out    std_logic_vector(12 downto 0);
        
        -- Position of read pointer
        rx_read_pointer_pos  :out    std_logic_vector(11 downto 0);
        
        -- Position of write pointer
        rx_write_pointer_pos :out    std_logic_vector(11 downto 0);
        
        -- Overrun occurred, data were discarded!
        -- (This is a flag and persists until it is cleared by SW)! 
        rx_data_overrun      :out    std_logic;
        
        -- External timestamp input
        timestamp            :in     std_logic_vector(63 downto 0);

        -----------------------------------------------------------------------
        -- Memory registers interface
        -----------------------------------------------------------------------
        -- Actually loaded data for reading
        rx_read_buff         :out    std_logic_vector(31 downto 0);
        
        -- Driving bus from registers
        drv_bus              :in     std_logic_vector(1023 downto 0)
    );
end entity;

architecture rtl of rx_buffer is

    ----------------------------------------------------------------------------
    -- Driving bus aliases
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
    -- FIFO  Memory - Pointers
    ----------------------------------------------------------------------------
    
    -- Read Pointer (access from SW)
    signal read_pointer             :       integer range 0 to G_RX_BUFF_SIZE - 1;

    -- Read pointer incremented by 1 (combinationally)
    signal read_pointer_inc_1       :       integer range 0 to G_RX_BUFF_SIZE - 1;

    -- Write pointer (committed, available to SW, after frame was stored)
    signal write_pointer            :       integer range 0 to G_RX_BUFF_SIZE - 1;

    -- Write pointer RAW. Changing during frame, as frame is continously stored
    -- to the buffer. When frame is sucesfully received, it is updated to
    -- write pointer!
    signal write_pointer_raw        :       integer range 0 to G_RX_BUFF_SIZE - 1;

    -- Extra write pointer which is used for storing timestamp at the end of
    -- data frame!
    signal write_pointer_extra_ts   :       integer range 0 to G_RX_BUFF_SIZE - 1;

    -- Final pointer to memory. "write_pointer_raw" and 
    -- "write_pointer_extra_ts" are multiplexed based on RX FSM! 
    signal memory_write_pointer     :       integer range 0 to G_RX_BUFF_SIZE - 1;

    -- Data that will be written to the RX Buffer memory!
    signal memory_write_data        :       std_logic_vector(31 downto 0);

    -- Number of free memory words available to SW after frame was committed.
    signal rx_mem_free_int          :       integer range -1 to G_RX_BUFF_SIZE + 1;


    ----------------------------------------------------------------------------
    -- FIFO  Memory - Free words, Overrun status
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

    -- Indicator of at least one free word in RX FIFO!
    signal is_free_word             :       boolean;

    -- Number of frames currently stored in RX Buffer. Smallest frame length
    -- stored is 4 (FRAME_FORMAT +  IDENTIFIER + 2 * TIMESTAMP). Since we need
    -- to store 0 and also G_RX_BUFF_SIZE / 4 values we need one value more than can
    -- fit into G_RX_BUFF_SIZE / 4 width counter. Use one bit wider counter.
    signal message_count            :       natural range 0 to (G_RX_BUFF_SIZE / 2) - 1; 

    -- Counter for reading the frame. When whole frame is read,
    -- number of frames must be decremented.
    signal read_frame_counter       :       natural range 0 to 31;


    ----------------------------------------------------------------------------
    -- FIFO  Memory - Commands which manipulate pointers, or indicate intent
    --  to write or read from the memory.
    ----------------------------------------------------------------------------
    
    -- When commit of RX Frame is signalled by CAN Core (rec_valid)
    -- "commit_rx_frame" is set to indicate that frame was sucesfully
    -- stored. Note that "rec_valid" is not enough to indicate that
    -- frame was stored sucesfully! If frame storing fails at some point due
    -- to lack of memory in FIFO, Protocol control will still finish the frame
    -- and provide "rec_valid"! Thus RX FSM sets "commit_rx_frame" only
    -- if "data_overrun" did not occur during the frame!
    signal commit_rx_frame          :       std_logic;

    -- When overrun occurred at any point in the frame and some word was not
    -- stored, frame can not be committed, and write_pointer must be moved
    -- back to last committed value!
    signal commit_overrun_abort     :       std_logic;

    -- Indicates that read occurred, and that it is valid (there is something
    -- to read), thus read pointer can be incremented.
    signal read_increment           :       std_logic;

    -- Indicates that "write_raw_intent" is OK (no overrun) and data can be
    -- truly written to the memory and raw pointer can be updated!
    signal write_raw_OK             :       std_logic;


    ----------------------------------------------------------------------------
    -- RX Buffer FSM outputs
    ----------------------------------------------------------------------------
    
    -- Indicates that FSM is in a state which would like to perform write of a
    -- word to RX Buffer memory!
    signal write_raw_intent         :       std_logic;

    -- Indicates that FSM is in one of states for writing timestmap from end of
    -- frame to the memory.
    signal write_extra_ts           :       std_logic;

    -- Storing of extra timestamp is at the end.
    signal store_extra_ts_end       :       std_logic;

    -- Data write selector
    signal data_selector            :       std_logic_vector(6 downto 0);

    -- Signals that write pointer should be stored to extra write pointer
    signal store_extra_wr_ptr       :       std_logic;

    -- Increment extra write pointer
    signal inc_extra_wr_ptr         :       std_logic;

    -- Restart overrun flag upon start of new frame
    signal reset_overrun_flag       :       std_logic;


    ----------------------------------------------------------------------------
    -- RX FSM, Timestamp capturing, combinationally decoded words
    ----------------------------------------------------------------------------
    
    -- Combinationally decoded size of the frame (without Frame format word)
    -- from received DLC (the size is in 32-bit words).
    signal rwcnt_com                :       natural range 0 to 31;

    -- Combinational decoded frame format word from metadata.
    signal frame_form_w             :       std_logic_vector(31 downto 0);

    -- Internal timestamp captured for storing. Captured either in the
    -- beginning or end of frame. 
    signal timestamp_capture        :       std_logic_vector(63 downto 0);


    ----------------------------------------------------------------------------
    -- RAM wrapper signals
    ----------------------------------------------------------------------------
    
    -- Write control signal    
    signal RAM_write                :       std_logic;

    -- Data output from port B     
    signal RAM_data_out             :       std_logic_vector(31 downto 0);

    -- Write address (connected to write pointer)
    signal RAM_write_address        :       std_logic_vector(11 downto 0);

    -- Read address (connected to read pointer)
    signal RAM_read_address         :       std_logic_vector(11 downto 0);

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
    rx_buf_size           <= std_logic_vector(to_unsigned(G_RX_BUFF_SIZE, 13));

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
    -- RX Buffer FSM component
    ----------------------------------------------------------------------------
    rx_buffer_fsm_inst : rx_buffer_fsm
    generic map(
        G_RESET_POLARITY    => G_RESET_POLARITY
    )
    port map(
        clk_sys             => clk_sys,             -- IN
        res_n               => res_n,               -- IN
        store_metadata_f    => store_metadata_f,    -- IN
        store_data_f        => store_data_f,        -- IN
        rec_valid_f         => rec_valid_f,         -- IN
        rec_abort_f         => rec_abort_f,         -- IN
        sof_pulse           => sof_pulse,           -- IN
        drv_bus             => drv_bus,             -- IN
        
        write_raw_intent    => write_raw_intent,    -- OUT
        write_extra_ts      => write_extra_ts,      -- OUT
        store_extra_ts_end  => store_extra_ts_end,  -- OUT
        data_selector       => data_selector,       -- OUT
        store_extra_wr_ptr  => store_extra_wr_ptr,  -- OUT
        inc_extra_wr_ptr    => inc_extra_wr_ptr,    -- OUT
        reset_overrun_flag  => reset_overrun_flag   -- OUT
    );


    ----------------------------------------------------------------------------
    -- RX Buffer Memory pointers
    ----------------------------------------------------------------------------
    rx_buffer_pointers_inst : rx_buffer_pointers
    generic map(
        G_RESET_POLARITY        => G_RESET_POLARITY,
        G_RX_BUFF_SIZE          => G_RX_BUFF_SIZE
    )
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n,                   -- IN
        rec_abort_f             => rec_abort_f,             -- IN
        commit_rx_frame         => commit_rx_frame,         -- IN
        write_raw_OK            => write_raw_OK,            -- IN
        commit_overrun_abort    => commit_overrun_abort,    -- IN
        store_extra_wr_ptr      => store_extra_wr_ptr,      -- IN
        inc_extra_wr_ptr        => inc_extra_wr_ptr,        -- IN
        read_increment          => read_increment,          -- IN
        drv_bus                 => drv_bus,                 -- IN
        
        read_pointer            => read_pointer,            -- OUT
        read_pointer_inc_1      => read_pointer_inc_1,      -- OUT
        write_pointer           => write_pointer,           -- OUT
        write_pointer_raw       => write_pointer_raw,       -- OUT
        write_pointer_extra_ts  => write_pointer_extra_ts,  -- OUT
        rx_mem_free_int         => rx_mem_free_int          -- OUT
    );


    ----------------------------------------------------------------------------
    -- Final write pointer is multiplexed between "write_pointer_raw" for
    -- regular writes and "write_pointer_extra_ts" for writes of timestamp
    -- in the end of frame!
    ----------------------------------------------------------------------------
    memory_write_pointer   <= write_pointer_extra_ts when (write_extra_ts = '1')
                                                     else
                              write_pointer_raw;


    ----------------------------------------------------------------------------
    -- Memory data which are written depend on state of the FSM
    ----------------------------------------------------------------------------
    with data_selector select memory_write_data <=
        frame_form_w                     when "0000001",
        "000" & rec_ident                when "0000010",
        timestamp_capture(31 downto 0)   when "0000100",
        timestamp_capture(31 downto 0)   when "0001000",
        timestamp_capture(63 downto 32)  when "0010000",
        timestamp_capture(63 downto 32)  when "0100000",
        store_data_word                  when "1000000",
        (OTHERS => '0')                  when others;


    ----------------------------------------------------------------------------
    -- Signalling that read which came is valid (there is sth to read)
    ----------------------------------------------------------------------------
    read_increment        <= '1' when (drv_read_start = '1' and
                                        rx_empty_int = '0')
                                  else
                             '0';


    ----------------------------------------------------------------------------
    -- Signalling that FSM may progress with the write (there is enough space
    -- in the buffer, nor any previous data were lost due to overrun)
    ----------------------------------------------------------------------------
    write_raw_OK         <= '1' when (write_raw_intent = '1' and
                                       overrun_condition = false and
                                       data_overrun_int = '0')
                                else
                            '0';

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
    overrun_condition    <= true when (write_raw_intent = '1' and 
                                       (is_free_word = false))
                                 else
                            false;


    ----------------------------------------------------------------------------
    -- When buffer is empty the word on address of read pointer is not valid,
    -- provide zeroes instead
    ----------------------------------------------------------------------------
    rx_read_buff          <= RAM_data_out when (rx_empty_int = '0')
                                          else
                             (OTHERS => '0');


    ----------------------------------------------------------------------------
    -- Receive data size (in words) decoder
    ----------------------------------------------------------------------------
    with rec_dlc select rwcnt_com <=
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
    frame_form_w(DLC_H downto DLC_L)      <= rec_dlc;
    frame_form_w(4)                       <= '0';
    frame_form_w(RTR_IND)                 <= rec_is_rtr;
    frame_form_w(IDE_IND)                 <= rec_ident_type;
    frame_form_w(FDF_IND)                 <= rec_frame_type;
    frame_form_w(TBF_IND)                 <= '1'; -- All frames have the timestamp
    frame_form_w(BRS_IND)                 <= rec_brs;
    frame_form_w(ESI_RSV_IND)             <= rec_esi;


    ----------------------------------------------------------------------------
    -- RWCNT (Read word count is calculated like so:
    --  1. For RTR Frames -> 3 (Only ID + 2 Timestamp words)
    --  2. For Normal CAN Frames with DLC > 8 max. 8 bytes -> RWCNT = 5
    --  3. Otherwise Number of data bytes is matching Received DLC!
    ----------------------------------------------------------------------------
    frame_form_w(RWCNT_H downto RWCNT_L)  <=
        "00011" when (rec_is_rtr = RTR_FRAME) else
        "00101" when ((rec_frame_type = NORMAL_CAN) and (rec_dlc(3) = '1')) else
         std_logic_vector(to_unsigned(rwcnt_com, (RWCNT_H - RWCNT_L + 1)));

    frame_form_w(31 downto 16)            <= (OTHERS => '0');


    ----------------------------------------------------------------------------
    -- Capturing timestamp. Done at the beginning or end of frame based on
    -- SW configuration.
    ----------------------------------------------------------------------------
    capt_ts_proc : process(clk_sys, res_n)
    begin
        if (res_n = G_RESET_POLARITY) then
            timestamp_capture       <= (OTHERS => '0');

        elsif (rising_edge(clk_sys)) then
            
            if ((drv_rtsopt = RTS_END and rec_valid_f = '1') or
                (drv_rtsopt = RTS_BEG and sof_pulse = '1')) 
            then  
                timestamp_capture   <= timestamp;
            end if;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Reading counter (read_frame_counter) which is loaded by RWCNT during read
    -- of frame format word. Then each next read decreases the counter. When 
    -- read counter reaches zero, message count is decreased. If "commit_rx_frame" 
    -- comes, "message_count" is incremented. If both occur at the same time
    -- , "message_count" does not change.
    ----------------------------------------------------------------------------
    read_frame_proc : process(clk_sys, res_n, drv_erase_rx)
    begin
        if (res_n = G_RESET_POLARITY or drv_erase_rx = '1') then
            read_frame_counter           <= 0;

        elsif (rising_edge(clk_sys)) then

            --------------------------------------------------------------------
            -- Reading frame by user when there is active read and there is
            -- something to read
            --------------------------------------------------------------------
            if (read_increment = '1') then

                ----------------------------------------------------------------
                -- During the read of FRAME_FORMAT word store the length
                -- of the frame to "read_frame_counter", thus we know how much
                -- we have to read before decrementing the "message_count".
                ----------------------------------------------------------------
                if (read_frame_counter = 0) then
                    read_frame_counter    <=
                        to_integer(unsigned(RAM_data_out(RWCNT_H downto
                                                            RWCNT_L)));
                else
                    read_frame_counter <= read_frame_counter - 1;
                end if;
            end if;

        end if;    
    end process;


    ---------------------------------------------------------------------------
    -- Manipulation of "message_count". When last word is read from frame
    -- (read_frame_counter = 1 and read_increment), "message_count" is
    -- decreased, when new frame is committed, message count is increased.
    -- If both at the same time, no change since one frame is added, next is 
    -- removed!
    ---------------------------------------------------------------------------
    message_count_ctr_proc : process(res_n, clk_sys, drv_erase_rx)
    begin
        if (res_n = G_RESET_POLARITY or drv_erase_rx = '1') then
            message_count <= 0;

        elsif (rising_edge(clk_sys)) then

            -- Read of last word, but no new commit
            if ((read_increment = '1') and (read_frame_counter = 1)) then
                if (commit_rx_frame = '0') then
                    message_count           <= message_count - 1;
                end if;

            -- Commit of new frame
            elsif (commit_rx_frame = '1') then
                message_count               <= message_count + 1;
            end if;

        end if;
    end process;
    

    ----------------------------------------------------------------------------
    -- Commit RX Frame when last word was written and overrun did not occur!
    -- This can be either from "rxb_store_data" state or "rxb_store_end_ts_high"
    ----------------------------------------------------------------------------
    commit_proc : process(res_n, clk_sys, drv_erase_rx)
    begin
        if (res_n = G_RESET_POLARITY or drv_erase_rx = '1') then
            commit_rx_frame       <= '0';
            commit_overrun_abort  <= '0';

        elsif (rising_edge(clk_sys)) then

            if (((rec_valid_f = '1' and drv_rtsopt = RTS_BEG) or
                 (store_extra_ts_end = '1')))
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
    -- Calculation of data overrun flag. If FSM would like to write to the
    -- memory, and there is not enough free space, data overrun flag will be
    -- set, and no further writes will be executed.
    ----------------------------------------------------------------------------
    dor_proc : process(res_n, clk_sys, drv_erase_rx)
    begin
        if (res_n = G_RESET_POLARITY or drv_erase_rx = '1') then
            data_overrun_r      <= '0';
            data_overrun_int    <= '0';

        elsif (rising_edge(clk_sys)) then

            --------------------------------------------------------------------
            -- Internal Data overrun flag -> cleared by new frame!
            --------------------------------------------------------------------
            if (reset_overrun_flag = '1') then
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
    -- RAM Memory of RX Buffer
    ----------------------------------------------------------------------------
    rx_buf_RAM_inst : inf_RAM_wrapper 
    generic map (
        G_WORD_WIDTH           => 32,
        G_DEPTH                => G_RX_BUFF_SIZE,
        G_ADDRESS_WIDTH        => RAM_write_address'length,
        G_RESET_POLARITY       => G_RESET_POLARITY,
        G_SIMULATION_RESET     => true,
        G_SYNC_READ            => true
    )
    port map(
        clk_sys              => clk_sys,                -- IN
        res_n                => res_n,                  -- IN
        addr_A               => RAM_write_address,      -- IN
        write                => RAM_write,              -- IN
        data_in              => memory_write_data,      -- IN
        addr_B               => RAM_read_address,       -- IN
        
        data_out             => RAM_data_out            -- OUT
    );

    -- Memory written either on regular write or Extra timestamp write
    RAM_write  <= '1' when (write_raw_OK = '1' or
                           (write_extra_ts = '1' and data_overrun_int = '0' and
                            overrun_condition = false))
                      else
                  '0';

    -- Write address is given by write pointer
    RAM_write_address   <= std_logic_vector(to_unsigned(
                            memory_write_pointer, RAM_write_address'length));


    ----------------------------------------------------------------------------
    -- RAM read address is given by read pointers. If no transaction for read
    -- of RX DATA is in progress, read pointer is given by its real value.
    -- During transaction, Incremented Read pointer is chosen to avoid one clock
    -- cycle delay caused by increment on read pointer!
    ----------------------------------------------------------------------------
   RAM_read_address <= std_logic_vector(to_unsigned(
                        read_pointer_inc_1, RAM_read_address'length))
                       when (read_increment = '1') else
                       std_logic_vector(to_unsigned(
                        read_pointer, RAM_read_address'length));


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
    assert ((G_RX_BUFF_SIZE = 32) or
            (G_RX_BUFF_SIZE = 64) or
            (G_RX_BUFF_SIZE = 128) or
            (G_RX_BUFF_SIZE = 256) or
            (G_RX_BUFF_SIZE = 512) or
            (G_RX_BUFF_SIZE = 1024) or
            (G_RX_BUFF_SIZE = 2048) or
            (G_RX_BUFF_SIZE = 4096))
    report "Unsupported RX Buffer size! RX Buffer must be power of 2!"
        severity failure;


    ----------------------------------------------------------------------------
    -- Storing sequence is like so:
    --  1. Store metadata.
    --  2. Store data "n" times, n = ceil(data_length / 4). De-facto RWCNT field
    --     contains number of remaining words (apart from FRAME_FORMAT_W). Thus,
    --     RWCNT - 3 = number of expected data words.
    --  3. Get "rec_abort" or "rec_valid" command.
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
            if (rec_abort_f = '1') then
                exp_data_stores := 0;
                act_data_stores := 0;

            elsif (store_metadata_f = '1') then

                exp_data_stores := to_integer(unsigned(
                                    frame_form_w(RWCNT_H downto RWCNT_L))) - 3;
                act_data_stores := 0;
            end if;

            -- Count actual number of "store_data" commands.
            if (store_data_f = '1') then
                act_data_stores := act_data_stores + 1;
            end if;

            -- Check when frame was received that proper number of "store_data"
            -- commands did arrive.
            if (rec_valid_f = '1' and 
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
    ----------------------------------------------------------------------------
    -- Functional coverage
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);
    --
    -- psl rx_buf_empty_cov : 
    --      cover (rx_empty = '1');
    --
    -- psl rx_buf_not_empty_to_empty_cov :
    --      cover {rx_empty = '0'; rx_empty = '1'};
    --
    -- psl rx_buf_rx_full_cov :
    --      cover (rx_full = '1');
    -- 
    -- psl rx_buf_rx_full_to_not_full_cov :
    --      cover {(rx_full = '1'); (rx_full = '0')};
    --
    -- psl rx_buf_overrun_cov :
    --      cover (overrun_condition = true);
    --
    -- psl rx_buf_commit_overrun_abort_cov :
    --      cover (commit_overrun_abort = '1');
    --
    -- psl rx_buf_overrun_flags_cov :
    --      cover (data_overrun_int = '1' and data_overrun_r = '1');
    --
    -- psl rx_buf_overrun_clear_cov :
    --      cover (drv_clr_ovr = '1');
    --
    -- psl rx_buf_write_extra_ts_cov :
    --      cover (write_extra_ts = '1');
    -- 
    -- psl rx_buf_release_receive_buffer_cov :
    --      cover (drv_erase_rx = '1'); 
    --
    -- psl rx_buf_commit_and_read_cov :
    --      cover (read_increment = '1' and commit_rx_frame = '1'); 
    --
    -- psl rx_buf_commit_after_read_cov :
    --      cover {read_increment = '1'; commit_rx_frame = '1'}; 
    --
    -- psl rx_buf_read_after_commit_cov :
    --      cover {commit_rx_frame = '1'; read_increment = '1'};
    --
    -- psl rx_buf_write_and_read_cov :
    --      cover (write_raw_intent = '1' and read_increment = '1');
    --
    -- psl rx_buf_read_after_write_cov :
    --      cover {write_raw_intent = '1'; read_increment = '1'};
    --
    -- psl rx_buf_write_after_read_cov :
    --      cover {read_increment = '1'; write_raw_intent = '1'};
    --
    -- psl rx_buf_sof_timestamp :
    --      cover (drv_rtsopt = RTS_BEG and commit_rx_frame = '1');
    --
    -- psl rx_buf_eof_timestamp :
    --      cover (drv_rtsopt = RTS_END and commit_rx_frame = '1');
    --
    -- psl rx_buf_burst_read_short_cov :
    --      cover {(read_increment = '1')[*4]};
    --
    -- psl rx_buf_burst_read_max_cov :
    --      cover {(read_increment = '1')[*20]};
    --
    -- psl rx_buf_frame_abort_cov :
    --      cover (rec_abort_f = '1');
    --
    -- psl rx_buf_store_rtr_cov :
    --      cover (rec_is_rtr = '1' and commit_rx_frame = '1');
    --
    -- psl rx_buf_store_empty_frame_cov :
    --      cover (rec_dlc = "0000" and rec_is_rtr = '0' and commit_rx_frame = '1');
    --
    -- psl rx_buf_store_1_byte_frame_cov :
    --      cover (rec_dlc = "0001" and rec_is_rtr = '0' and commit_rx_frame = '1');
    --
    -- psl rx_buf_store_2_byte_frame_cov :
    --      cover (rec_dlc = "0010" and rec_is_rtr = '0' and commit_rx_frame = '1');
    --
    -- psl rx_buf_store_3_byte_frame_cov :
    --      cover (rec_dlc = "0011" and rec_is_rtr = '0' and commit_rx_frame = '1');
    --
    -- psl rx_buf_store_4_byte_frame_cov :
    --      cover (rec_dlc = "0100" and rec_is_rtr = '0' and commit_rx_frame = '1');
    --
    -- psl rx_buf_store_5_byte_frame_cov :
    --      cover (rec_dlc = "0101" and rec_is_rtr = '0' and commit_rx_frame = '1');
    --
    -- psl rx_buf_store_8_byte_frame_cov :
    --      cover (rec_dlc = "1000" and rec_is_rtr = '0' and commit_rx_frame = '1');
    --
    -- psl rx_buf_store_64_byte_frame_cov :
    --      cover (rec_dlc = "1111" and rec_is_rtr = '0' and commit_rx_frame = '1');
    
end architecture;