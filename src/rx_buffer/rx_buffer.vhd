--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core 
-- Copyright (C) 2021-present Ondrej Ille
-- 
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to use, copy, modify, merge, publish, distribute the Component for
-- educational, research, evaluation, self-interest purposes. Using the
-- Component for commercial purposes is forbidden unless previously agreed with
-- Copyright holder.
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
-- -------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core 
-- Copyright (C) 2015-2020 MIT License
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
-- Module:
--  RX Buffer
--
-- Purpose:
--  Stores RX CAN frame during its reception into RX Buffer RAM. Controlled by
--  Protocol control FSM. RX Frame is read word by word from Memory registers. 
--  RX Buffer is continously stored as it is being received. At the end of frame
--  it is committed to memory, and becomes available to the user. If Overrun
--  or Release receive Buffer occured meanwhile, frame is reverted.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_components_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.common_blocks_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

use ctu_can_fd_rtl.can_registers_pkg.all;

entity rx_buffer is
    generic(
        -- RX Buffer size
        G_RX_BUFF_SIZE              :       natural range 32 to 4096 := 32;
        
        -- Technology type
        G_TECHNOLOGY                :       natural := C_TECH_ASIC
    );
    port(
        ------------------------------------------------------------------------
        -- Clocks and Asynchronous reset 
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              :in     std_logic;
        
        -- Async. reset
        res_n                :in     std_logic;

        -----------------------------------------------------------------------
        -- DFT support
        -----------------------------------------------------------------------
        scan_enable         : in     std_logic;

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
        rx_frame_count       :out    std_logic_vector(10 downto 0);
        
        -- Number of free 32 bit wide words
        rx_mem_free          :out    std_logic_vector(12 downto 0);
        
        -- Position of read pointer
        rx_read_pointer      :out    std_logic_vector(11 downto 0);
        
        -- Position of write pointer
        rx_write_pointer     :out    std_logic_vector(11 downto 0);
        
        -- Overrun occurred, data were discarded!
        -- (This is a flag and persists until it is cleared by SW)! 
        rx_data_overrun      :out    std_logic;
        
        -- Middle of frame indication
        rx_mof               :out    std_logic;
        
        -- External timestamp input
        timestamp            :in     std_logic_vector(63 downto 0);

        -----------------------------------------------------------------------
        -- Memory registers interface
        -----------------------------------------------------------------------
        -- Actually loaded data for reading
        rx_read_buff         :out    std_logic_vector(31 downto 0);
        
        -- Driving bus from registers
        drv_bus              :in     std_logic_vector(1023 downto 0);
        
        -- Test registers
        test_registers_out   :in     test_registers_out_t;
        
        -- RX buffer RAM test output
        tst_rdata_rx_buf     :out    std_logic_vector(31 downto 0)
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
    signal read_pointer             : std_logic_vector(11 downto 0);

    -- Read pointer incremented by 1 (combinationally)
    signal read_pointer_inc_1       : std_logic_vector(11 downto 0);

    -- Write pointer (committed, available to SW, after frame was stored)
    signal write_pointer            : std_logic_vector(11 downto 0);

    -- Write pointer RAW. Changing during frame, as frame is continously stored
    -- to the buffer. When frame is sucesfully received, it is updated to
    -- write pointer!
    signal write_pointer_raw        : std_logic_vector(11 downto 0);

    -- Timestamp write pointer which is used for storing timestamp at the end of
    -- data frame!
    signal write_pointer_ts         : std_logic_vector(11 downto 0);

    -- Number of free memory words available to SW after frame was committed.
    signal rx_mem_free_i            : std_logic_vector(12 downto 0);

    -- Data that will be written to the RX Buffer memory!
    signal memory_write_data        : std_logic_vector(31 downto 0);

    -- RX Buffer mem free
    constant C_RX_BUF_MEM_FREE_ZEROES : std_logic_vector(12 downto 0) :=
        (OTHERS => '0');

    constant C_RX_BUF_PTR_ZEROES : std_logic_vector(11 downto 0) :=
        (OTHERS => '0');

    ----------------------------------------------------------------------------
    -- FIFO  Memory - Free words, Overrun status
    ----------------------------------------------------------------------------

    -- Data overrun flag. Recieved message was lost, because there was not
    -- enough space in FIFO during storing! Available for SW!
    signal data_overrun_flg           :       std_logic;

    -- Internal data overrun flag. This flag is not available to SW, but it
    -- is restarted automatically at the beginning of each new frame reception!
    -- This allows to accept next frames when overrun occurred on previous ones!
    signal data_overrun_i           :       std_logic;

    -- Combinationally decoded overrun condition. Active when there is intention
    -- to store word to the memory, but there is not enough free space! 
    signal overrun_condition        :       std_logic;

     -- RX Buffer is empty (no frame is stored in it)
    signal rx_empty_i               :       std_logic;

    -- Indicator of at least one free word in RX FIFO!
    signal is_free_word             :       std_logic;

    -- Number of frames currently stored in RX Buffer. Smallest frame length
    -- stored is 4 (FRAME_FORMAT +  IDENTIFIER + 2 * TIMESTAMP). Since we need
    -- to store 0 and also G_RX_BUFF_SIZE / 4 values we need one value more than can
    -- fit into G_RX_BUFF_SIZE / 4 width counter. Use one bit wider counter.
    signal frame_count              :       natural range 0 to (G_RX_BUFF_SIZE / 2) - 1; 

    -- Counter for reading the frame. When whole frame is read,
    -- number of frames must be decremented.
    signal read_counter_d           :       unsigned(4 downto 0);
    signal read_counter_q           :       unsigned(4 downto 0);


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

    -- Indicates that FSM is in one of states for writing timestamp
    signal write_ts                 :       std_logic;

    -- Storing of timestamp is at the end.
    signal stored_ts                :       std_logic;

    -- Data write selector
    signal data_selector            :       std_logic_vector(4 downto 0);

    -- Signals that write pointer should be stored to timestamp write pointer
    signal store_ts_wr_ptr          :       std_logic;

    -- Increment timestamp write pointer
    signal inc_ts_wr_ptr            :       std_logic;

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
    
    -- Clock enable signal for timestamp capture register 
    signal timestamp_capture_ce     :       std_logic;


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

    ----------------------------------------------------------------------------
    -- Common reset signal
    ----------------------------------------------------------------------------
    signal rx_buf_res_n_d           :       std_logic;
    signal rx_buf_res_n_q           :       std_logic;
    signal rx_buf_res_n_q_scan      :       std_logic;

    ----------------------------------------------------------------------------
    -- Clock gating for memory
    ----------------------------------------------------------------------------
    signal rx_buf_ram_clk_en        :       std_logic;
    signal clk_ram                  :       std_logic;

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
    rx_read_pointer      <= read_pointer;
    rx_write_pointer     <= write_pointer;
    rx_data_overrun      <= data_overrun_flg;
    rx_buf_size          <= std_logic_vector(to_unsigned(G_RX_BUFF_SIZE, 13));

    rx_empty_i           <= '1' when (frame_count = 0)
                                else
                            '0'; 

    rx_full              <= '1' when (rx_mem_free_i = C_RX_BUF_MEM_FREE_ZEROES)
                                else
                            '0';

    rx_frame_count       <= std_logic_vector(to_unsigned(frame_count, 11)); 
    rx_mem_free          <= rx_mem_free_i;
    rx_empty             <= rx_empty_i;

    ----------------------------------------------------------------------------
    -- Common reset signal. Whole buffer can be reset by two ways:
    --  1. Asynchronous reset - res_n
    --  2. Release Receive Buffer command - drv_erase_rx.
    ----------------------------------------------------------------------------
    rx_buf_res_n_d <= '0' when (drv_erase_rx = '1' or res_n = '0') else
                      '1';

    ----------------------------------------------------------------------------
    -- Register reset to avoid glitches
    ----------------------------------------------------------------------------
    res_reg_inst : dff_arst
    generic map(
        G_RESET_POLARITY   => '0',
        
        -- Reset to the same value as is polarity of reset so that other DFFs
        -- which are reset by output of this one will be reset too!
        G_RST_VAL          => '0'
    )
    port map(
        arst               => res_n,                -- IN
        clk                => clk_sys,              -- IN
        input              => rx_buf_res_n_d,       -- IN

        output             => rx_buf_res_n_q        -- OUT
    );
    
    ----------------------------------------------------------------------------
    -- Mux for gating reset in scan mode
    ----------------------------------------------------------------------------
    mux2_res_tst_inst : mux2
    port map(
        a                  => rx_buf_res_n_q, 
        b                  => '1',
        sel                => scan_enable,

        -- Output
        z                  => rx_buf_res_n_q_scan
    );


    ----------------------------------------------------------------------------
    -- RX Buffer FSM component
    ----------------------------------------------------------------------------
    rx_buffer_fsm_inst : rx_buffer_fsm
    port map(
        clk_sys             => clk_sys,             -- IN
        res_n               => res_n,               -- IN
        store_metadata_f    => store_metadata_f,    -- IN
        store_data_f        => store_data_f,        -- IN
        rec_valid_f         => rec_valid_f,         -- IN
        rec_abort_f         => rec_abort_f,         -- IN
        
        write_raw_intent    => write_raw_intent,    -- OUT
        write_ts            => write_ts,            -- OUT
        stored_ts           => stored_ts,           -- OUT
        data_selector       => data_selector,       -- OUT
        store_ts_wr_ptr     => store_ts_wr_ptr,     -- OUT
        inc_ts_wr_ptr       => inc_ts_wr_ptr,       -- OUT
        reset_overrun_flag  => reset_overrun_flag   -- OUT
    );


    ----------------------------------------------------------------------------
    -- RX Buffer Memory pointers
    ----------------------------------------------------------------------------
    rx_buffer_pointers_inst : rx_buffer_pointers
    generic map(
        G_RX_BUFF_SIZE          => G_RX_BUFF_SIZE
    )
    port map(
        clk_sys                 => clk_sys,                 -- IN
        rx_buf_res_n_q_scan     => rx_buf_res_n_q_scan,     -- IN

        rec_abort_f             => rec_abort_f,             -- IN
        commit_rx_frame         => commit_rx_frame,         -- IN
        write_raw_OK            => write_raw_OK,            -- IN
        commit_overrun_abort    => commit_overrun_abort,    -- IN
        store_ts_wr_ptr         => store_ts_wr_ptr,         -- IN
        inc_ts_wr_ptr           => inc_ts_wr_ptr,           -- IN
        read_increment          => read_increment,          -- IN
        
        read_pointer            => read_pointer,            -- OUT
        read_pointer_inc_1      => read_pointer_inc_1,      -- OUT
        write_pointer           => write_pointer,           -- OUT
        write_pointer_raw       => write_pointer_raw,       -- OUT
        write_pointer_ts        => write_pointer_ts,        -- OUT
        rx_mem_free_i           => rx_mem_free_i            -- OUT
    );


    ----------------------------------------------------------------------------
    -- Memory data which are written depend on state of the FSM
    ----------------------------------------------------------------------------
    with data_selector select memory_write_data <=
        frame_form_w                     when "00001",
        "000" & rec_ident                when "00010",
        store_data_word                  when "00100",
        timestamp_capture(31 downto 0)   when "01000",
        timestamp_capture(63 downto 32)  when "10000",
        (OTHERS => '0')                  when others;


    ----------------------------------------------------------------------------
    -- Signalling that read which came is valid (there is sth to read)
    ----------------------------------------------------------------------------
    read_increment <= '1' when (drv_read_start = '1' and rx_empty_i = '0') else
                      '0';

    ----------------------------------------------------------------------------
    -- Signalling that FSM may progress with the write (there is enough space
    -- in the buffer, nor any previous data were lost due to overrun)
    ----------------------------------------------------------------------------
    write_raw_OK         <= '1' when (write_raw_intent = '1' and
                                      overrun_condition = '0' and
                                      data_overrun_i = '0')
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
    is_free_word          <= '0' when (read_pointer = write_pointer_raw and
                                       frame_count > 0)
                                 else
                             '1';

    ----------------------------------------------------------------------------
    -- Overrun condition. Following conditions must be met:
    --  1. FSM wants to write to memory either to the position of
    --      "write_pointer_raw". Note that "write_pointer_ts" writes to
    --      words which were already written, thus there is no need to watch
    --      for overrun!
    --  2. There is no free word in the memory remaining!
    ----------------------------------------------------------------------------
    overrun_condition <= '1' when (write_raw_intent = '1' and 
                                  (is_free_word = '0'))
                             else
                         '0';


    ----------------------------------------------------------------------------
    -- When buffer is empty the word on address of read pointer is not valid,
    -- provide zeroes instead
    ----------------------------------------------------------------------------
    rx_read_buff <= RAM_data_out when (rx_empty_i = '0') else
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
    frame_form_w(8)                       <= '1'; -- Reserved
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
    timestamp_capture_ce <= '1' when (drv_rtsopt = RTS_END and rec_valid_f = '1')
                                else
                            '1' when (drv_rtsopt = RTS_BEG and sof_pulse = '1')
                                else
                            '0';
                            
    capt_ts_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            timestamp_capture       <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then
            if (timestamp_capture_ce = '1') then
                timestamp_capture   <= timestamp;
            end if;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Reading counter (read_counter_q) which is loaded by RWCNT during read
    -- of frame format word. Then each next read decreases the counter. When 
    -- read counter reaches zero, message count is decreased. If "commit_rx_frame" 
    -- comes, "frame_count" is incremented. If both occur at the same time
    -- , "frame_count" does not change.
    ----------------------------------------------------------------------------
    
    ---------------------------------------------------------------------------
    -- During the read of FRAME_FORMAT word store the length of the frame to
    -- "read_counter", thus we know how much we have to read before 
    -- decrementing the "frame_count".
    ---------------------------------------------------------------------------
    read_counter_d <= read_counter_q - 1 when (read_counter_q > "00000") else
                      unsigned(RAM_data_out(RWCNT_H downto RWCNT_L));
    
    read_frame_proc : process(clk_sys, rx_buf_res_n_q_scan)
    begin
        if (rx_buf_res_n_q_scan = '0') then
            read_counter_q <= (OTHERS => '0');

        elsif (rising_edge(clk_sys)) then

            --------------------------------------------------------------------
            -- Reading frame by user when there is active read and there is
            -- something to read
            --------------------------------------------------------------------
            if (read_increment = '1') then
                read_counter_q <= read_counter_d;
            end if;
        end if;    
    end process;


    ---------------------------------------------------------------------------
    -- Manipulation of "frame_count". When last word is read from frame
    -- (read_counter_q = 1 and read_increment), "frame_count" is
    -- decreased, when new frame is committed, message count is increased.
    -- If both at the same time, no change since one frame is added, next is 
    -- removed!
    ---------------------------------------------------------------------------
    frame_count_ctr_proc : process(clk_sys, rx_buf_res_n_q_scan)
    begin
        if (rx_buf_res_n_q_scan = '0') then
            frame_count <= 0;

        elsif (rising_edge(clk_sys)) then

            -- Read of last word, but no new commit
            if ((read_increment = '1') and (read_counter_q = "00001")) then
                if (commit_rx_frame = '0') then
                    frame_count           <= frame_count - 1;
                end if;

            -- Commit of new frame
            elsif (commit_rx_frame = '1') then
                frame_count               <= frame_count + 1;
            end if;

        end if;
    end process;
    

    ----------------------------------------------------------------------------
    -- Commit RX Frame when last word was written and overrun did not occur!
    -- This can be either from "rxb_store_data" state or "rxb_store_end_ts_high"
    ----------------------------------------------------------------------------
    commit_proc : process(clk_sys, rx_buf_res_n_q_scan)
    begin
        if (rx_buf_res_n_q_scan = '0') then
            commit_rx_frame       <= '0';
            commit_overrun_abort  <= '0';

        elsif (rising_edge(clk_sys)) then

            if (stored_ts = '1') then
                if (data_overrun_i = '0') then
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
    -- Calculation of data overrun flag for user. If FSM would like to write to
    -- the memory, and there is not enough free space, data overrun flag will be
    -- set, and no further writes will be executed. Data Overrun flag can be
    -- cleared from Memory registers via Driving bus.
    ----------------------------------------------------------------------------
    sw_dor_proc : process(clk_sys, rx_buf_res_n_q_scan)
    begin
        if (rx_buf_res_n_q_scan = '0') then
            data_overrun_flg      <= '0';
            
        elsif (rising_edge(clk_sys)) then

            --------------------------------------------------------------------
            -- SW overrun flag -> Cleared from SW!
            --------------------------------------------------------------------
            if (drv_clr_ovr = '1') then
                data_overrun_flg  <= '0';
 
            elsif (overrun_condition = '1') then
                data_overrun_flg  <= '1';
            else
                data_overrun_flg  <= data_overrun_flg;
            end if;

        end if;
    end process;
    
    ----------------------------------------------------------------------------
    -- Internal data overrun flag. This will be set by two conditions:
    --  1. When FSM attempts to write to full RAM.
    --  2. When RRB command is issued and frame storing is in progress! If such
    --     situation occurs, pointers are erased RX Buffer FSM is erased, while
    --     Protocol control continues storing the frame (increments Raw write
    --     pointer). Commiting such a frame would result in inconsistend state
    --     of RX Buffer. So if RRB during storing occurs, all pointers are 
    --     erased, RX Buffer FSM keeps storing, and overrun flag is set. At the
    --     end of storing, flag is erased and raw write pointer is reverted to
    --     commited pointer (which is zero because it was erased).
    -- 
    -- Cleared at the end of frame storing! Note that this register can't be
    -- reset by RRB, only by res_n! 
    ----------------------------------------------------------------------------
    internal_dor_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            data_overrun_i        <= '0';
        elsif (rising_edge(clk_sys)) then
            if (overrun_condition = '1' or drv_erase_rx = '1') then
                data_overrun_i    <= '1';
            elsif (reset_overrun_flag = '1') then
                data_overrun_i    <= '0';
            else
                data_overrun_i    <= data_overrun_i;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Clock gating for RAM. Enable when:
    -- 1. CAN Core is writing
    -- 2. Reading occurs from register map.
    -- 3. Permanently when Memory testing is enabled, or in scan mode
    ----------------------------------------------------------------------------
    rx_buf_ram_clk_en <= '1' when (RAM_write = '1' or drv_read_start = '1')
                             else
                         '1' when (test_registers_out.tst_control(TMAENA_IND) = '1' or
                                   scan_enable = '1')
                             else
                         '0';

    clk_gate_rx_buffer_ram_comp : clk_gate
    generic map(
        G_TECHNOLOGY       => G_TECHNOLOGY
    )
    port map(
        clk_in             => clk_sys,
        clk_en             => rx_buf_ram_clk_en,

        clk_out            => clk_ram
    );

    ----------------------------------------------------------------------------
    -- RAM Memory of RX Buffer
    ----------------------------------------------------------------------------
    rx_buffer_ram_inst : rx_buffer_ram
    generic map(
        G_RX_BUFF_SIZE       => G_RX_BUFF_SIZE
    )
    port map(
        -- Clocks and Asynchronous reset 
        clk_sys              => clk_ram,                -- IN
        res_n                => res_n,                  -- IN

        -- Memory testability
        test_registers_out   => test_registers_out,     -- IN
        tst_rdata_rx_buf     => tst_rdata_rx_buf,       -- OUT

        -- Port A - Write (from CAN Core)
        port_a_address       => RAM_write_address,      -- IN
        port_a_data_in       => memory_write_data,      -- IN
        port_a_write         => RAM_write,              -- IN

        -- Port B - Read (from Memory registers)
        port_b_address       => RAM_read_address,       -- IN
        port_b_data_out      => RAM_data_out            -- OUT
    );

    -- Memory written either on regular write or timestamp write
    RAM_write  <= '1' when (write_raw_OK = '1' or
                           (write_ts = '1' and data_overrun_i = '0' and
                            overrun_condition = '0'))
                      else
                  '0';

    ----------------------------------------------------------------------------
    -- Memory write address is multiplexed between "write_pointer_raw" for
    -- regular writes and "write_pointer_ts" for writes of timestamp!
    ----------------------------------------------------------------------------
    RAM_write_address   <= write_pointer_ts when (write_ts = '1') else
                           write_pointer_raw;

    ----------------------------------------------------------------------------
    -- RAM read address is given by read pointers. If no transaction for read
    -- of RX DATA is in progress, read pointer is given by its real value.
    -- During transaction, Incremented Read pointer is chosen to avoid one clock
    -- cycle delay caused by increment on read pointer!
    ----------------------------------------------------------------------------
    RAM_read_address <= read_pointer_inc_1 when (read_increment = '1') else
                              read_pointer;
                              

    ----------------------------------------------------------------------------
    -- RX buffer middle of frame
    ----------------------------------------------------------------------------
    rx_mof <= '0' when (read_counter_q = "00000") else
              '1';
    
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

    -- <RELEASE_OFF>

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
    -- pragma translate_off
    rwcnt_assert_proc : process(clk_sys)
        variable exp_data_stores    : natural := 0;
        variable act_data_stores   : natural := 0;
    begin
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
    end process;
    -- pragma translate_on


    ----------------------------------------------------------------------------
    -- Assertions
    ----------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);
    
    -- psl read_counter_lt_rwcnt_asrt : assert never
    --  (read_counter_q > 19)
    -- report "Read counter higher than longest RWCNT!"
    -- severity error;
    
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Functional coverage
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- psl rx_buf_empty_cov : 
    --      cover {rx_empty = '1'};
    --
    -- psl rx_buf_not_empty_to_empty_cov :
    --      cover {rx_empty = '0'; rx_empty = '1'};
    --
    -- psl rx_buf_rx_full_cov :
    --      cover {rx_full = '1'};
    -- 
    -- psl rx_buf_rx_full_to_not_full_cov :
    --      cover {(rx_full = '1'); (rx_full = '0')};
    --
    -- psl rx_buf_overrun_cov :
    --      cover {overrun_condition = '1'};
    --
    -- psl rx_buf_commit_overrun_abort_cov :
    --      cover {commit_overrun_abort = '1'};
    --
    -- psl rx_buf_overrun_flags_cov :
    --      cover {data_overrun_i = '1' and data_overrun_flg = '1'};
    --
    -- psl rx_buf_overrun_clear_cov :
    --      cover {drv_clr_ovr = '1'};
    --
    -- psl rx_buf_write_ts_cov :
    --      cover {write_ts = '1'};
    -- 
    -- psl rx_buf_release_receive_buffer_cov :
    --      cover {drv_erase_rx = '1'}; 
    --
    -- psl rx_buf_commit_and_read_cov :
    --      cover {read_increment = '1' and commit_rx_frame = '1'}; 
    --
    -- psl rx_buf_commit_after_read_cov :
    --      cover {read_increment = '1'; commit_rx_frame = '1'};
    --
    -- psl rx_buf_read_after_commit_cov :
    --      cover {commit_rx_frame = '1'; read_increment = '1'};
    --
    -- psl rx_buf_write_and_read_cov :
    --      cover {write_raw_intent = '1' and read_increment = '1'};
    --
    -- psl rx_buf_read_after_write_cov :
    --      cover {write_raw_intent = '1'; read_increment = '1'};
    --
    -- psl rx_buf_write_after_read_cov :
    --      cover {read_increment = '1'; write_raw_intent = '1'};
    --
    -- psl rx_buf_sof_timestamp :
    --      cover {drv_rtsopt = RTS_BEG and commit_rx_frame = '1'};
    --
    -- psl rx_buf_eof_timestamp :
    --      cover {drv_rtsopt = RTS_END and commit_rx_frame = '1'};
    --
    -- psl rx_buf_burst_read_short_cov :
    --      cover {(read_increment = '1')[*4]};
    --
    -- psl rx_buf_burst_read_max_cov :
    --      cover {(read_increment = '1')[*16]};
    -- Note: SW reads the frame like so: Read metadata one by one and then 16 data words.
    --       Therefore highest burst achievable is 16 with current TB!
    
    -- psl rx_buf_frame_abort_cov :
    --      cover {rec_abort_f = '1'};
    --
    -- psl rx_buf_store_rtr_cov :
    --      cover {rec_is_rtr = '1' and commit_rx_frame = '1'};
    --
    -- psl rx_buf_store_empty_frame_cov :
    --      cover {rec_dlc = "0000" and rec_is_rtr = '0' and commit_rx_frame = '1'};
    --
    -- psl rx_buf_store_1_byte_frame_cov :
    --      cover {rec_dlc = "0001" and rec_is_rtr = '0' and commit_rx_frame = '1'};
    --
    -- psl rx_buf_store_2_byte_frame_cov :
    --      cover {rec_dlc = "0010" and rec_is_rtr = '0' and commit_rx_frame = '1'};
    --
    -- psl rx_buf_store_3_byte_frame_cov :
    --      cover {rec_dlc = "0011" and rec_is_rtr = '0' and commit_rx_frame = '1'};
    --
    -- psl rx_buf_store_4_byte_frame_cov :
    --      cover {rec_dlc = "0100" and rec_is_rtr = '0' and commit_rx_frame = '1'};
    --
    -- psl rx_buf_store_5_byte_frame_cov :
    --      cover {rec_dlc = "0101" and rec_is_rtr = '0' and commit_rx_frame = '1'};
    --
    -- psl rx_buf_store_8_byte_frame_cov :
    --      cover {rec_dlc = "1000" and rec_is_rtr = '0' and commit_rx_frame = '1'};
    --
    -- psl rx_buf_store_64_byte_frame_cov :
    --      cover {rec_dlc = "1111" and rec_is_rtr = '0' and commit_rx_frame = '1'};

    ---------------------------------------------------------------------------
    -- "reset_overrun_flag = '1'" only in "s_rxb_idle" state. Therefore we can
    -- use this signal to check that FSM is in s_rxb_idle state!
    ---------------------------------------------------------------------------
    
    -- psl sof_pulse_asrt_asrt : assert never
    --   (sof_pulse = '1' and reset_overrun_flag = '0')
    -- report "RX Buffer: SOF pulse should come when RX Buffer is idle!"
    -- severity error;

    -- <RELEASE_ON>

end architecture;