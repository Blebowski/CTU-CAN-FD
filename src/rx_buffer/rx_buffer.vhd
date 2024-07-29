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

use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

use ctu_can_fd_rtl.can_registers_pkg.all;

entity rx_buffer is
    generic (
        -- RX Buffer size
        G_RX_BUFF_SIZE              :     natural range 32 to 4096;

        -- Width of RX Buffer pointers
        G_RX_BUFF_PTR_WIDTH         :     natural range 5 to 12;

        -- Width of RX Buffer frame counter
        G_RX_BUF_FRAME_CNT_WIDTH    :     natural range 3 to 11;

        -- Add parity to RX Buffer RAM
        G_SUP_PARITY                :     boolean;

        -- Reset RX Buffer RAM
        G_RESET_RX_BUF_RAM          :     boolean;

        -- Technology type
        G_TECHNOLOGY                :     natural
    );
    port (
        -------------------------------------------------------------------------------------------
        -- Clocks and Asynchronous reset
        -------------------------------------------------------------------------------------------
        clk_sys                 : in  std_logic;
        res_n                   : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- DFT support
        -------------------------------------------------------------------------------------------
        scan_enable             : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Metadata from CAN Core
        -------------------------------------------------------------------------------------------
        -- Frame Identifier
        rec_ident               : in  std_logic_vector(28 downto 0);

        -- Data length code
        rec_dlc                 : in  std_logic_vector(3 downto 0);

        -- Recieved identifier type (0-BASE Format, 1-Extended Format);
        rec_ident_type          : in  std_logic;

        -- Recieved frame type (0-Normal CAN, 1- CAN FD)
        rec_frame_type          : in  std_logic;

        -- Received Loopback frame
        rec_lbpf                : in  std_logic;

        -- Recieved frame is RTR Frame(0-No, 1-Yes)
        rec_is_rtr              : in  std_logic;

        -- Whenever frame was recieved with BIT Rate shift
        rec_brs                 : in  std_logic;

        -- Recieved error state indicator
        rec_esi                 : in  std_logic;

        -- Received identifier is valid
        rec_ivld                : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Control signals from CAN Core which control storing of CAN Frame. (Filtered by Frame
        -- Filters)
        -------------------------------------------------------------------------------------------
        -- After control field of CAN frame, metadata are valid and can be stored. This command
        -- starts the RX FSM for storing.
        store_metadata_f        : in  std_logic;

        -- Signal that one word of data can be stored (TX_DATA_X_W). This signal is active when 4
        -- bytes were received or data reception has finished on 4 byte unaligned number of frames!
        -- (Thus allowing to store also data which are not 4 byte aligned!
        store_data_f            : in  std_logic;

        -- Data word which should be stored when "store_data" is active!
        store_data_word         : in  std_logic_vector(31 downto 0);

        -- Received frame valid (commit RX Frame)
        rec_valid_f             : in  std_logic;

        -- Abort storing of RX Frame to RX Buffer.
        rec_abort_f             : in  std_logic;

        -- Signals start of frame. If timestamp on RX frame should be captured in the beginning of
        -- the frame, this pulse captures the timestamp!
        sof_pulse               : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Status signals of RX buffer
        -------------------------------------------------------------------------------------------
        -- Signal whenever buffer is full (no free memory words)
        rx_full                 : out std_logic;

        -- Signal whenever buffer is empty (no frame (message) is stored)
        rx_empty                : out std_logic;

        -- Number of frames (messages) stored in recieve buffer
        rx_frame_count          : out std_logic_vector(G_RX_BUF_FRAME_CNT_WIDTH - 1 downto 0);

        -- Number of free 32 bit wide words
        rx_mem_free             : out std_logic_vector(G_RX_BUFF_PTR_WIDTH downto 0);

        -- Position of read pointer
        rx_read_pointer         : out std_logic_vector(G_RX_BUFF_PTR_WIDTH - 1 downto 0);

        -- Position of write pointer
        rx_write_pointer        : out std_logic_vector(G_RX_BUFF_PTR_WIDTH - 1 downto 0);

        -- Overrun occurred, data were discarded!
        -- (This is a flag and persists until it is cleared by SW).
        rx_data_overrun         : out std_logic;

        -- Middle of frame indication
        rx_mof                  : out std_logic;

        -- RX Buffer Parity error
        -- (This is a flag and persists until it is cleared by SW).
        rx_parity_error         : out std_logic;

        -- Actually loaded data for reading
        rxb_port_b_data_out     : out std_logic_vector(31 downto 0);

        -------------------------------------------------------------------------------------------
        -- External timestamp input
        -------------------------------------------------------------------------------------------
        timestamp               : in  std_logic_vector(63 downto 0);

        -------------------------------------------------------------------------------------------
        -- TX Arbitrator interface
        -------------------------------------------------------------------------------------------
        -- TXT Buffer index that is:
        --   - Currently validated (when no transmission is in progress)
        --   - Used for transmission (when transmission is in progress)
        curr_txtb_index         : in std_logic_vector(2 downto 0);

        -------------------------------------------------------------------------------------------
        -- Memory registers interface
        -------------------------------------------------------------------------------------------
        mr_mode_rxbam           : in  std_logic;
        mr_command_cdo          : in  std_logic;
        mr_command_crxpe        : in  std_logic;
        mr_command_rrb          : in  std_logic;
        mr_command_rxrpmv       : in  std_logic;
        mr_rx_data_read         : in  std_logic;
        mr_rx_settings_rtsop    : in  std_logic;
        mr_settings_pchke       : in  std_logic;

        -- Memory testability
        mr_tst_control_tmaena   : in  std_logic;
        mr_tst_control_twrstb   : in  std_logic;
        mr_tst_dest_tst_addr    : in  std_logic_vector(15 downto 0);
        mr_tst_dest_tst_mtgt    : in  std_logic_vector(3 downto 0);
        mr_tst_wdata_tst_wdata  : in  std_logic_vector(31 downto 0);

        mr_tst_rdata_tst_rdata  : out std_logic_vector(31 downto 0)
    );
end entity;

architecture rtl of rx_buffer is

    -----------------------------------------------------------------------------------------------
    -- FIFO  Memory - Pointers
    -----------------------------------------------------------------------------------------------

    -- Read Pointer (access from SW)
    signal read_pointer                 : std_logic_vector(G_RX_BUFF_PTR_WIDTH - 1 downto 0);

    -- Read pointer incremented by 1 (combinationally)
    signal read_pointer_inc_1           : std_logic_vector(G_RX_BUFF_PTR_WIDTH - 1 downto 0);

    -- Write pointer (committed, available to SW, after frame was stored)
    signal write_pointer                : std_logic_vector(G_RX_BUFF_PTR_WIDTH - 1 downto 0);

    -- Write pointer RAW. Changing during frame, as frame is continously stored
    -- to the buffer. When frame is sucesfully received, it is updated to
    -- write pointer!
    signal write_pointer_raw            : std_logic_vector(G_RX_BUFF_PTR_WIDTH - 1 downto 0);

    -- Timestamp write pointer which is used for storing timestamp at the end of
    -- data frame!
    signal write_pointer_ts             : std_logic_vector(G_RX_BUFF_PTR_WIDTH - 1 downto 0);

    -- Number of free memory words available to SW after frame was committed.
    signal rx_mem_free_i                : std_logic_vector(G_RX_BUFF_PTR_WIDTH downto 0);

    -- RX Buffer mem free
    constant C_RX_BUF_MEM_FREE_ZEROES   : std_logic_vector(G_RX_BUFF_PTR_WIDTH downto 0) :=
        (others => '0');
    constant C_RX_BUF_PTR_ZEROES        : std_logic_vector(G_RX_BUFF_PTR_WIDTH - 1 downto 0) :=
        (others => '0');

    -----------------------------------------------------------------------------------------------
    -- FIFO  Memory - Free words, Overrun status
    -----------------------------------------------------------------------------------------------

    -- Data overrun flag. Recieved message was lost, because there was not enough space in FIFO
    -- during storing! Available for SW!
    signal data_overrun_flg             : std_logic;

    -- Internal data overrun flag. This flag is not available to SW, but it is restarted
    -- automatically at the beginning of each new frame reception! This allows to accept next frames
    -- when overrun occurred on previous ones!
    signal data_overrun_i               : std_logic;

    -- Combinationally decoded overrun condition. Active when there is intention to store word to
    -- the memory, but there is not enough free space!
    signal overrun_condition            : std_logic;

     -- RX Buffer is empty (no frame is stored in it)
    signal rx_empty_i                   : std_logic;

    -- Indicator of at least one free word in RX FIFO!
    signal is_free_word                 : std_logic;

    -- Number of frames currently stored in RX Buffer. Smallest frame length stored is 4
    -- (FRAME_FORMAT +  IDENTIFIER + 2 * TIMESTAMP). Since we need to store 0 and also
    -- G_RX_BUFF_SIZE / 4 values we need one value more than can fit into G_RX_BUFF_SIZE / 4 width
    -- counter. Use one bit wider counter.
    signal frame_count                  : unsigned(G_RX_BUF_FRAME_CNT_WIDTH - 1 downto 0);

    -- Frame read counter. When whole frame is read, number of frames must be decremented.
    signal read_counter_d               : unsigned(4 downto 0);
    signal read_counter_q               : unsigned(4 downto 0);


    -----------------------------------------------------------------------------------------------
    -- FIFO  Memory - Commands which manipulate pointers, or indicate intent to write or read from
    -- the memory.
    -----------------------------------------------------------------------------------------------

    -- When commit of RX Frame is signalled by CAN Core (rec_valid) "commit_rx_frame" is set to
    -- indicate that frame was sucesfully stored. Note that "rec_valid" is not enough to indicate
    -- that frame was stored sucesfully! If frame storing fails at some point due to lack of memory
    -- in FIFO, Protocol control will still finish the frame and provide "rec_valid"! Thus RX FSM
    -- sets "commit_rx_frame" only if "data_overrun" did not occur during the frame!
    signal commit_rx_frame              : std_logic;

    -- When overrun occurred at any point in the frame and some word was not stored, frame can not
    -- be committed, and write_pointer must be moved back to last committed value!
    signal commit_overrun_abort         : std_logic;

    -- Indicates that read occurred, and that it is valid (there is something to read), thus read
    -- pointer can be incremented.
    signal read_increment               : std_logic;

    -- Indicates that "write_raw_intent" is OK (no overrun) and data can be truly written to the
    -- memory and raw pointer can be updated!
    signal write_raw_OK                 : std_logic;


    -----------------------------------------------------------------------------------------------
    -- RX Buffer FSM outputs
    -----------------------------------------------------------------------------------------------

    -- Indicates that FSM is in a state which would like to perform write of a word to RX Buffer
    -- memory!
    signal write_raw_intent             : std_logic;

    -- Indicates that FSM is in one of states for writing timestamp
    signal write_ts                     : std_logic;

    -- Storing of timestamp is at the end.
    signal stored_ts                    : std_logic;

    -- Data write selector
    signal data_selector                : std_logic_vector(4 downto 0);

    -- Signals that write pointer should be stored to timestamp write pointer
    signal store_ts_wr_ptr              : std_logic;

    -- Increment timestamp write pointer
    signal inc_ts_wr_ptr                : std_logic;

    -- Restart overrun flag upon start of new frame
    signal reset_overrun_flag           : std_logic;

    -- Trying to read from RX Buffer FIFO
    signal read_attempt                 : std_logic;

    -----------------------------------------------------------------------------------------------
    -- RX FSM, Timestamp capturing, combinationally decoded words
    -----------------------------------------------------------------------------------------------

    -- Combinationally decoded size of the frame (without Frame format word) from received DLC (the
    -- size is in 32-bit words).
    signal rwcnt_com                    : natural range 0 to 31;

    -- Combinational decoded frame format word from metadata.
    signal frame_form_w                 : std_logic_vector(31 downto 0);

    -- Internal timestamp captured for storing. Captured either in the beginning or end of frame.
    signal timestamp_capture            : std_logic_vector(63 downto 0);

    -- Clock enable signal for timestamp capture register
    signal timestamp_capture_ce         : std_logic;


    -----------------------------------------------------------------------------------------------
    -- RAM wrapper signals
    -----------------------------------------------------------------------------------------------
    signal rxb_port_a_write             : std_logic;
    signal rxb_port_a_address           : std_logic_vector(G_RX_BUFF_PTR_WIDTH - 1 downto 0);
    signal rxb_port_a_data_in           : std_logic_vector(31 downto 0);

    signal rxb_port_b_address           : std_logic_vector(G_RX_BUFF_PTR_WIDTH - 1 downto 0);
    signal rxb_port_b_data_out_i        : std_logic_vector(31 downto 0);

    -----------------------------------------------------------------------------------------------
    -- Common reset signal
    -----------------------------------------------------------------------------------------------
    signal rx_buf_res_n_d               : std_logic;
    signal rx_buf_res_n_q_scan          : std_logic;

    -----------------------------------------------------------------------------------------------
    -- Clock gating for memory
    -----------------------------------------------------------------------------------------------
    signal rx_buf_ram_clk_en            : std_logic;
    signal clk_ram                      : std_logic;

    -----------------------------------------------------------------------------------------------
    -- Parity error detection
    -----------------------------------------------------------------------------------------------
    signal rx_parity_mismatch_comb      : std_logic;

begin

    -----------------------------------------------------------------------------------------------
    -- Common reset signal. Whole buffer can be reset by two ways:
    --  1. Asynchronous reset - res_n
    --  2. Release Receive Buffer command.
    -----------------------------------------------------------------------------------------------
    rx_buf_res_n_d <= '0' when (mr_command_rrb = '1' or res_n = '0')
                          else
                      '1';

    -----------------------------------------------------------------------------------------------
    -- Register reset to avoid glitches
    -----------------------------------------------------------------------------------------------
    rst_reg_inst : entity ctu_can_fd_rtl.rst_reg
    generic map (
        G_RESET_POLARITY        => '0'
    )
    port map (
        -- Clock and Reset
        clk                     => clk_sys,                 -- IN
        arst                    => res_n,                   -- IN

        -- Flip flop input / output
        d                       => rx_buf_res_n_d,          -- IN
        q                       => rx_buf_res_n_q_scan,     -- OUT

        -- Scan mode control
        scan_enable             => scan_enable              -- IN
    );

    -----------------------------------------------------------------------------------------------
    -- RX Buffer FSM component
    -----------------------------------------------------------------------------------------------
    rx_buffer_fsm_inst : entity ctu_can_fd_rtl.rx_buffer_fsm
    port map (
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n,                   -- IN
        store_metadata_f        => store_metadata_f,        -- IN
        store_data_f            => store_data_f,            -- IN
        rec_valid_f             => rec_valid_f,             -- IN
        rec_abort_f             => rec_abort_f,             -- IN

        write_raw_intent        => write_raw_intent,        -- OUT
        write_ts                => write_ts,                -- OUT
        stored_ts               => stored_ts,               -- OUT
        data_selector           => data_selector,           -- OUT
        store_ts_wr_ptr         => store_ts_wr_ptr,         -- OUT
        inc_ts_wr_ptr           => inc_ts_wr_ptr,           -- OUT
        reset_overrun_flag      => reset_overrun_flag       -- OUT
    );


    -----------------------------------------------------------------------------------------------
    -- RX Buffer Memory pointers
    -----------------------------------------------------------------------------------------------
    rx_buffer_pointers_inst : entity ctu_can_fd_rtl.rx_buffer_pointers
    generic map (
        G_RX_BUFF_SIZE          => G_RX_BUFF_SIZE,
        G_RX_BUFF_PTR_WIDTH     => G_RX_BUFF_PTR_WIDTH
    )
    port map (
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

    -----------------------------------------------------------------------------------------------
    -- Memory data which are written depend on state of the FSM
    -----------------------------------------------------------------------------------------------
    with data_selector select rxb_port_a_data_in <=
        frame_form_w                     when "00001",
        "000" & rec_ident                when "00010",
        store_data_word                  when "00100",
        timestamp_capture(31 downto 0)   when "01000",
        timestamp_capture(63 downto 32)  when "10000",
        (others => '0')                  when others;

    -----------------------------------------------------------------------------------------------
    -- Signalling that read which came is valid (there is sth to read)
    -----------------------------------------------------------------------------------------------
    read_attempt <= mr_rx_data_read when (mr_mode_rxbam = RXBAM_ENABLED)
                                    else
                    mr_command_rxrpmv;

    read_increment <= '1' when (read_attempt = '1' and rx_empty_i = '0')
                          else
                      '0';

    -----------------------------------------------------------------------------------------------
    -- Signalling that FSM may progress with the write (there is enough space in the buffer, nor
    -- any previous data were lost due to overrun)
    -----------------------------------------------------------------------------------------------
    write_raw_OK <= '1' when (write_raw_intent = '1' and overrun_condition = '0' and
                              data_overrun_i = '0')
                        else
                    '0';

    -----------------------------------------------------------------------------------------------
    -- Store of new word can be executed only if there is space in the buffer. We don't need exact
    -- amount of words. We only need to know if there is space! When "read_pointer" and
    -- "write_pointer_raw" are equal, then memory is either empty, or full! If there is no frame
    -- stored and pointers are equal, then memory is empty! If there is at least one frame and
    -- pointers are equal, then memory must be full!
    -----------------------------------------------------------------------------------------------
    is_free_word <= '0' when (read_pointer = write_pointer_raw and frame_count > 0)
                        else
                    '1';

    -----------------------------------------------------------------------------------------------
    -- Overrun condition. Following conditions must be met:
    --  1. FSM wants to write to memory either to the position of "write_pointer_raw". Note that
    --     "write_pointer_ts" writes to words which were already written, thus there is no need to
    --      watch for overrun!
    --  2. There is no free word in the memory remaining!
    -----------------------------------------------------------------------------------------------
    overrun_condition <= '1' when (write_raw_intent = '1' and is_free_word = '0')
                             else
                         '0';


    -----------------------------------------------------------------------------------------------
    -- When buffer is empty the word on address of read pointer is not valid, provide zeroes
    -- instead. This guarantees that zeroes are read even if memory content is not defined ater
    -- power-up!
    -----------------------------------------------------------------------------------------------
    rxb_port_b_data_out <= rxb_port_b_data_out_i when (rx_empty_i = '0')
                                                 else
                                 (others => '0');


    -----------------------------------------------------------------------------------------------
    -- Receive data size (in words) decoder
    -----------------------------------------------------------------------------------------------
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
        19 when others; --64 bytes


    -----------------------------------------------------------------------------------------------
    -- Frame format word assignment
    -----------------------------------------------------------------------------------------------
    frame_form_w(DLC_H downto DLC_L)      <= rec_dlc;
    frame_form_w(4)                       <= '0';       -- TODO: Drive "ERF" bit
    frame_form_w(RTR_IND)                 <= rec_is_rtr;
    frame_form_w(IDE_IND)                 <= rec_ident_type;
    frame_form_w(FDF_IND)                 <= rec_frame_type;
    frame_form_w(LBPF_IND)                <= rec_lbpf;
    frame_form_w(BRS_IND)                 <= rec_brs;
    frame_form_w(ESI_RSV_IND)             <= rec_esi;
    frame_form_w(IVLD_IND)                <= rec_ivld;

    -----------------------------------------------------------------------------------------------
    -- RWCNT (Read word count is calculated like so:
    --  1. For RTR Frames -> 3 (Only ID + 2 Timestamp words)
    --  2. For Normal CAN Frames with DLC > 8 max. 8 bytes -> RWCNT = 5
    --  3. Otherwise Number of data bytes is matching Received DLC!
    -----------------------------------------------------------------------------------------------
    frame_form_w(RWCNT_H downto RWCNT_L)  <=
        "00011" when (rec_is_rtr = RTR_FRAME) else
        "00101" when ((rec_frame_type = NORMAL_CAN) and (rec_dlc(3) = '1')) else
         std_logic_vector(to_unsigned(rwcnt_com, (RWCNT_H - RWCNT_L + 1)));

    frame_form_w(23 downto 16) <= (others => '0'); -- TODO: Drive ERF_*

    frame_form_w(LBTBI_H downto LBTBI_L) <= curr_txtb_index when (rec_lbpf = LBPF_LOOPBACK)
                                                            else
                                            (others => '0');

    frame_form_w(31 downto 28) <= (others => '0');

    -----------------------------------------------------------------------------------------------
    -- Capturing timestamp. Done at the beginning or end of frame based on SW configuration.
    -----------------------------------------------------------------------------------------------
    timestamp_capture_ce <= '1' when (mr_rx_settings_rtsop = RTS_END and rec_valid_f = '1')
                                else
                            '1' when (mr_rx_settings_rtsop = RTS_BEG and sof_pulse = '1')
                                else
                            '0';

    capt_ts_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            timestamp_capture <= (others => '0');
        elsif (rising_edge(clk_sys)) then
            if (timestamp_capture_ce = '1') then
                timestamp_capture   <= timestamp;
            end if;
        end if;
    end process;


    -----------------------------------------------------------------------------------------------
    -- Reading counter (read_counter_q) which is loaded by RWCNT during read of frame format word.
    -- Then each next read decreases the counter. When read counter reaches zero, message count is
    -- decreased. If "commit_rx_frame" comes, "frame_count" is incremented. If both occur at the
    -- same time , "frame_count" does not change.
    -----------------------------------------------------------------------------------------------

    -----------------------------------------------------------------------------------------------
    -- During the read of FRAME_FORMAT word store the length of the frame to "read_counter", thus
    -- we know how much we have to read before decrementing the "frame_count".
    -----------------------------------------------------------------------------------------------
    read_counter_d <= read_counter_q - 1 when (read_counter_q > "00000")
                                         else
                      unsigned(rxb_port_b_data_out_i(RWCNT_H downto RWCNT_L));

    read_frame_proc : process(clk_sys, rx_buf_res_n_q_scan)
    begin
        if (rx_buf_res_n_q_scan = '0') then
            read_counter_q <= (others => '0');
        elsif (rising_edge(clk_sys)) then
            -- Reading frame by user when there is active read and there is
            -- something to read
            if (read_increment = '1') then
                read_counter_q <= read_counter_d;
            end if;
        end if;
    end process;


    -----------------------------------------------------------------------------------------------
    -- Manipulation of "frame_count". When last word is read from frame (read_counter_q = 1 and
    -- read_increment), "frame_count" is decreased, when new frame is committed, message count is
    -- increased. If both at the same time, no change since one frame is added, next is removed!
    -----------------------------------------------------------------------------------------------
    frame_count_ctr_proc : process(clk_sys, rx_buf_res_n_q_scan)
    begin
        if (rx_buf_res_n_q_scan = '0') then
            frame_count <= (others => '0');
        elsif (rising_edge(clk_sys)) then

            -- Read of last word, but no new commit
            if ((read_increment = '1') and (read_counter_q = "00001")) then
                if (commit_rx_frame = '0') then
                    frame_count <= frame_count - 1;
                end if;

            -- Commit of new frame
            elsif (commit_rx_frame = '1') then
                frame_count <= frame_count + 1;
            end if;

        end if;
    end process;


    -----------------------------------------------------------------------------------------------
    -- Commit RX Frame when last word was written and overrun did not occur! This can be either
    -- from "rxb_store_data" state or "rxb_store_end_ts_high"
    -----------------------------------------------------------------------------------------------
    commit_proc : process(clk_sys, rx_buf_res_n_q_scan)
    begin
        if (rx_buf_res_n_q_scan = '0') then
            commit_rx_frame       <= '0';
            commit_overrun_abort  <= '0';

        elsif (rising_edge(clk_sys)) then

            if (stored_ts = '1') then
                if (data_overrun_i = '0') then
                    commit_rx_frame <= '1';
                else
                    commit_overrun_abort <= '1';
                end if;
            else
                commit_rx_frame <= '0';
                commit_overrun_abort <= '0';
            end if;

        end if;
    end process;


    -----------------------------------------------------------------------------------------------
    -- Calculation of data overrun flag for user. If FSM would like to write to the memory, and
    -- there is not enough free space, data overrun flag will be set, and no further writes will
    -- be executed. Data Overrun flag can be cleared from Memory registers.
    -----------------------------------------------------------------------------------------------
    sw_dor_proc : process(clk_sys, rx_buf_res_n_q_scan)
    begin
        if (rx_buf_res_n_q_scan = '0') then
            data_overrun_flg <= '0';
        elsif (rising_edge(clk_sys)) then

            -- SW overrun flag -> Cleared from SW!
            if (mr_command_cdo = '1') then
                data_overrun_flg <= '0';
            elsif (overrun_condition = '1') then
                data_overrun_flg <= '1';
            else
                data_overrun_flg <= data_overrun_flg;
            end if;

        end if;
    end process;

    -----------------------------------------------------------------------------------------------
    -- Internal data overrun flag. This will be set by two conditions:
    --  1. When FSM attempts to write to full RAM.
    --  2. When RRB command is issued and frame storing is in progress! If such situation occurs,
    --     pointers are erased RX Buffer FSM is erased, while Protocol control continues storing
    --     the frame (increments Raw write pointer). Commiting such a frame would result in
    --     inconsistent state of RX Buffer. So if RRB during storing occurs, all pointers are
    --     erased, RX Buffer FSM keeps storing, and overrun flag is set. At the end of storing,
    --     flag is erased and raw write pointer is reverted to commited pointer (which is zero
    --     because it was erased).
    --
    -- Cleared at the end of frame storing! Note that this register can't be reset by RRB, only
    -- by res_n!
    -----------------------------------------------------------------------------------------------
    internal_dor_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            data_overrun_i <= '0';
        elsif (rising_edge(clk_sys)) then
            if (overrun_condition = '1' or mr_command_rrb = '1') then
                data_overrun_i <= '1';
            elsif (reset_overrun_flag = '1') then
                data_overrun_i <= '0';
            else
                data_overrun_i <= data_overrun_i;
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------------------------------
    -- Clock gating for RAM. Enable when:
    -- 1. CAN Core is writing
    -- 2. Reading occurs from register map.
    -- 3. Permanently when Memory testing is enabled, or in scan mode
    -----------------------------------------------------------------------------------------------
    rx_buf_ram_clk_en <= '1' when (rxb_port_a_write = '1' or read_attempt = '1')
                             else
                         '1' when (mr_tst_control_tmaena = '1')
                             else
                         '0';

    clk_gate_rx_buffer_ram_comp : entity ctu_can_fd_rtl.clk_gate
    generic map(
        G_TECHNOLOGY            => G_TECHNOLOGY
    )
    port map(
        clk_in                  => clk_sys,                 -- IN
        clk_en                  => rx_buf_ram_clk_en,       -- IN
        scan_enable             => scan_enable,             -- IN

        clk_out                 => clk_ram                  -- OUT
    );

    -----------------------------------------------------------------------------------------------
    -- RAM Memory of RX Buffer
    -----------------------------------------------------------------------------------------------
    rx_buffer_ram_inst : entity ctu_can_fd_rtl.rx_buffer_ram
    generic map(
        G_RX_BUFF_SIZE          => G_RX_BUFF_SIZE,
        G_RX_BUFF_PTR_WIDTH     => G_RX_BUFF_PTR_WIDTH,
        G_SUP_PARITY            => G_SUP_PARITY,
        G_RESET_RX_BUF_RAM      => G_RESET_RX_BUF_RAM
    )
    port map(
        -- Clocks and Asynchronous reset
        clk_sys                 => clk_ram,                 -- IN
        res_n                   => res_n,                   -- IN

        -- Memory testability
        mr_tst_control_tmaena   => mr_tst_control_tmaena,   -- IN
        mr_tst_control_twrstb   => mr_tst_control_twrstb,   -- IN
        mr_tst_dest_tst_addr    => mr_tst_dest_tst_addr,    -- IN
        mr_tst_dest_tst_mtgt    => mr_tst_dest_tst_mtgt,    -- IN
        mr_tst_wdata_tst_wdata  => mr_tst_wdata_tst_wdata,  -- IN

        mr_tst_rdata_tst_rdata  => mr_tst_rdata_tst_rdata,  -- OUT

        -- Port A - Write (from CAN Core)
        rxb_port_a_address      => rxb_port_a_address,      -- IN
        rxb_port_a_data_in      => rxb_port_a_data_in,      -- IN
        rxb_port_a_write        => rxb_port_a_write,        -- IN

        -- Port B - Read (from Memory registers)
        rxb_port_b_address      => rxb_port_b_address,      -- IN
        rxb_port_b_data_out     => rxb_port_b_data_out_i,   -- OUT

        -- Parity error detection
        parity_mismatch         => rx_parity_mismatch_comb  -- OUT
    );

    -- Memory written either on regular write or timestamp write
    rxb_port_a_write  <= '1' when (write_raw_OK = '1' or
                                  (write_ts = '1' and data_overrun_i = '0' and
                                   overrun_condition = '0'))
                             else
                         '0';

    -----------------------------------------------------------------------------------------------
    -- Memory write address is multiplexed between "write_pointer_raw" for regular writes and
    -- "write_pointer_ts" for writes of timestamp!
    -----------------------------------------------------------------------------------------------
    rxb_port_a_address   <= write_pointer_ts when (write_ts = '1')
                                             else
                           write_pointer_raw;

    -----------------------------------------------------------------------------------------------
    -- RAM read address is given by read pointers. If no transaction for read of RX DATA is in
    -- progress, read pointer is given by its real value. During transaction, Incremented Read
    -- pointer is chosen to avoid one clock cycle delay caused by increment on read pointer.
    -----------------------------------------------------------------------------------------------
    rxb_port_b_address <= read_pointer_inc_1 when (read_increment = '1')
                                             else
                                read_pointer;


    -----------------------------------------------------------------------------------------------
    -- RX buffer middle of frame
    -----------------------------------------------------------------------------------------------
    rx_mof <= '0' when (read_counter_q = "00000")
                  else
              '1';

    -----------------------------------------------------------------------------------------------
    -- Parity error flag
    -- Set when reading RX Buffer RAM. When read is in porgress is set, then RX Buffer RAM already
    -- has read data available on output, therefore, RX parity error detection is valid!
    -----------------------------------------------------------------------------------------------
    parity_flag_proc : process(res_n, clk_sys)
    begin
        if (res_n = '0') then
            rx_parity_error <= '0';
        elsif (rising_edge(clk_sys)) then
            if (read_attempt = '1' and rx_parity_mismatch_comb = '1' and mr_settings_pchke = '1')
            then
                rx_parity_error <= '1';
            elsif (mr_command_crxpe = '1') then
                rx_parity_error <= '0';
            end if;
        end if;
    end process;


    -----------------------------------------------------------------------------------------------
    -- Propagating status registers on output
    -----------------------------------------------------------------------------------------------
    rx_read_pointer  <= read_pointer;
    rx_write_pointer <= write_pointer;
    rx_data_overrun  <= data_overrun_flg;

    rx_empty_i       <= '1' when (frame_count = 0)
                            else
                        '0';

    rx_full          <= '1' when (rx_mem_free_i = C_RX_BUF_MEM_FREE_ZEROES)
                            else
                        '0';

    rx_frame_count   <= std_logic_vector(frame_count);
    rx_mem_free      <= rx_mem_free_i;
    rx_empty         <= rx_empty_i;


    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    -- Assertions
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------


    -----------------------------------------------------------------------------------------------
    -- RX Buffer size can be only powers of 2. Since modulo arithmetics is used on memory pointers,
    -- using non power of 2 value would result in increased logic usage!
    -----------------------------------------------------------------------------------------------
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

    -----------------------------------------------------------------------------------------------
    -- Storing sequence is like so:
    --  1. Store metadata.
    --  2. Store data "n" times, n = ceil(data_length / 4). De-facto RWCNT field contains number of
    --     remaining words (apart from FRAME_FORMAT_W). Thus, RWCNT - 3 = number of expected data
    --     words.
    --  3. Get "rec_abort" or "rec_valid" command.
    --
    --  This process verifies that "rec_data" command comes expected number of times (RWCNT - 3).
    --  This verifies consistency of storing protocol by CAN Core, as well as RWCNT field!
    -----------------------------------------------------------------------------------------------
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

                exp_data_stores := to_integer(unsigned(frame_form_w(RWCNT_H downto RWCNT_L))) - 3;
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


    -----------------------------------------------------------------------------------------------
    -- Assertions
    -----------------------------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);

    -- psl read_counter_lt_rwcnt_asrt : assert never
    --  (read_counter_q > 19)
    -- report "Read counter higher than longest RWCNT!";

    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    -- Functional coverage
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
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
    --      cover {mr_command_cdo = '1'};
    --
    -- psl rx_buf_write_ts_cov :
    --      cover {write_ts = '1'};
    --
    -- psl rx_buf_release_receive_buffer_cov :
    --      cover {mr_command_rrb = '1'};
    --
    -- psl rx_buf_commit_and_read_cov :
    --      cover {read_increment = '1' and read_counter_q = "00001" and commit_rx_frame = '1'}
    --      report "RX Buffer Commit and Frame read finish - Simultaneous!";
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
    --      cover {mr_rx_settings_rtsop = RTS_BEG and commit_rx_frame = '1'};
    --
    -- psl rx_buf_eof_timestamp :
    --      cover {mr_rx_settings_rtsop = RTS_END and commit_rx_frame = '1'};
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
    --
    -- psl rx_parity_err_cov :
    --      cover {rx_parity_error = '1'};
    --
    -- psl rx_parity_err_clr_cov :
    --      cover {rx_parity_error = '1' and mr_command_crxpe = '1'};
    --
    -- psl rx_lbpf_cov :
    --      cover {rec_lbpf = '1'};

    -----------------------------------------------------------------------------------------------
    -- "reset_overrun_flag = '1'" only in "s_rxb_idle" state. Therefore we can
    -- use this signal to check that FSM is in s_rxb_idle state!
    -----------------------------------------------------------------------------------------------

    -- psl sof_pulse_asrt_asrt : assert never
    --   (sof_pulse = '1' and reset_overrun_flag = '0')
    -- report "RX Buffer: SOF pulse should come when RX Buffer is idle!";

    -- <RELEASE_ON>

end architecture;