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
--  TX Arbitrator
--
-- Purpose:
--  Circuit for selecting the valid frame for CAN Core from generic number of
--  TXT buffer inputs. Compares priorities of each buffer (SW selected) and
--  picks the highest priority buffer whose input is valid. Timestamp of high-
--  est priority frame is selected and compared with external timestamp. The
--  frame is marked as valid for CAN Core only if this timestamp is lower than
--  value of external Timestamp. This realizes the functionality of transmission
--  at exact time! Metadata of transmitted frame are loaded to output of TX
--  Arbitrator as part of selection process.
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

entity tx_arbitrator is
    generic (
        -- Number of TXT Buffers
        G_TXT_BUFFER_COUNT      : natural range 1 to 8
    );
    port (
        -------------------------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -------------------------------------------------------------------------------------------
        clk_sys                 : in  std_logic;
        res_n                   : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- TXT Buffers interface
        -------------------------------------------------------------------------------------------
        -- Data words from TXT Buffers RAM memories
        txtb_port_b_data_out    : in  t_txt_bufs_output(G_TXT_BUFFER_COUNT - 1 downto 0);

        -- TXT Buffers are available, can be selected by TX Arbitrator
        txtb_available          : in  std_logic_vector(G_TXT_BUFFER_COUNT - 1 downto 0);

        -- TXT Buffer is in state in which it can have backup buffer
        txtb_allow_bb           : in  std_logic_vector(G_TXT_BUFFER_COUNT - 1 downto 0);

        -- Pointer to TXT Buffer
        txtb_port_b_address     : out std_logic_vector(4 downto 0);

        -- Clock enable to TXT Buffer port B
        txtb_port_b_clk_en      : out std_logic;

        -- Parity check valid
        txtb_parity_check_valid : out std_logic;

        -- Parity Mismatch in TXT Buffer
        txtb_parity_mismatch    : in  std_logic_vector(G_TXT_BUFFER_COUNT - 1 downto 0);

        -- TXT Buffer index
        txtb_index_muxed        : out natural range 0 to G_TXT_BUFFER_COUNT - 1;

        -- TXT Buffer is operating as backup buffer
        txtb_is_bb              : out std_logic_vector(G_TXT_BUFFER_COUNT - 1 downto 0);

        -------------------------------------------------------------------------------------------
        -- CAN Core Interface
        -------------------------------------------------------------------------------------------
        -- TXT Buffer memory word
        tran_word               : out std_logic_vector(31 downto 0);

        -- TX Data length code
        tran_dlc                : out std_logic_vector(3 downto 0);

        -- TX Remote transmission request flag
        tran_is_rtr             : out std_logic;

        -- TX Identifier type (0-Basic,1-Extended);
        tran_ident_type         : out std_logic;

        -- TX Frame type (0-CAN 2.0, 1-CAN FD)
        tran_frame_type         : out std_logic;

        -- TX Frame Bit rate shift Flag
        tran_brs                : out std_logic;

        -- TX Identifier
        tran_identifier         : out std_logic_vector(28 downto 0);

        -- TX Frame test
        tran_frame_test         : out t_frame_test_w;

        -- There is valid frame selected, can be locked for transmission
        tran_frame_valid        : out std_logic;

        -- Parity error occured during read of data words from TXT Buffer
        tran_frame_parity_error : out std_logic;

        -- HW Commands from CAN Core for manipulation with TXT Buffers
        txtb_hw_cmd             : in  t_txtb_hw_cmd;

        -- Selected TXT Buffer changed in comparison to previous transmission
        txtb_changed            : out std_logic;

        -- Index of the TXT Buffer for which the actual HW command is valid
        txtb_hw_cmd_index       : out natural range 0 to G_TXT_BUFFER_COUNT - 1;

        -- Pointer to TXT Buffer given by CAN Core. Used for reading data words.
        txtb_ptr                : in  natural range 0 to 20;

        -- TXT Buffer clock enable (from Protocol control)
        txtb_clk_en             : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Memory registers interface
        -------------------------------------------------------------------------------------------
        mr_mode_tttm            : in  std_logic;
        mr_mode_txbbm           : in  std_logic;
        mr_settings_pchke       : in  std_logic;
        mr_tx_priority          : in  t_txt_bufs_priorities(G_TXT_BUFFER_COUNT - 1 downto 0);

        -------------------------------------------------------------------------------------------
        -- Timestamp
        -------------------------------------------------------------------------------------------
        timestamp               : in  std_logic_vector(63 downto 0)
  );
end entity;

architecture rtl of tx_arbitrator is

    -----------------------------------------------------------------------------------------------
    -- Internal signals
    -----------------------------------------------------------------------------------------------

    -- Indicates the highest selected buffer and its validity from combinational priority decoder
    signal select_buf_avail           : std_logic;
    signal select_buf_index           : natural range 0 to G_TXT_BUFFER_COUNT - 1;

    -- Input word from TXT Buffer !!!
    signal txtb_selected_input        : std_logic_vector(31 downto 0);

    -- TXT Buffer timestamp joined combinationally. Given by ts_low_internal and upper timestamp
    -- word, out the output of RAM
    signal txtb_timestamp             : std_logic_vector(63 downto 0);

    -- Comparison of loaded timestamp from TXT Buffer.
    signal timestamp_valid            : std_logic;

    -- The output of priority decoder (selected TXT Buffer) has changed, pulse for one clock cycle
    signal select_index_changed       : std_logic;

    -- Signal that there is "Validated" TXT Buffer
    signal validated_buffer           : std_logic;

    -- TXT Buffer clock enable (one cycle later)
    signal txtb_clk_en_q              : std_logic;

    -- Metadata words read by TX Arbitrator are valid
    signal tx_arb_parity_check_valid  : std_logic;

    -----------------------------------------------------------------------------------------------
    -- Internal registers
    -----------------------------------------------------------------------------------------------

    -- Registered values for detection of change
    signal select_buf_index_reg       : natural range 0 to G_TXT_BUFFER_COUNT - 1;

    -- Lower timestamp loaded from TXT Buffer
    signal ts_low_internal            : std_logic_vector(31 downto 0);

    -- Internal index of TXT Buffer stored at the time of buffer selection
    signal int_txtb_index             : natural range 0 to G_TXT_BUFFER_COUNT - 1;

    -- TXT Buffer internal index of last buffer that was locked From buffer change, Protocol control
    -- can erase retransmitt counter
    signal last_txtb_index            : natural range 0 to G_TXT_BUFFER_COUNT - 1;

    -- TXT Buffer index validated or used for transmission
    signal txtb_index_muxed_i         : natural range 0 to G_TXT_BUFFER_COUNT - 1;

    -- Pointer to TXT Buffer for loading CAN frame metadata and timstamp during the selection of
    -- TXT Buffer.
    signal txtb_pointer_meta_q        : natural range 0 to 20;
    signal txtb_pointer_meta_d        : natural range 0 to 20;

    -- Double buffer registers for metadata
    signal tran_dlc_dbl_buf           : std_logic_vector(3 downto 0);
    signal tran_is_rtr_dbl_buf        : std_logic;
    signal tran_ident_type_dbl_buf    : std_logic;
    signal tran_frame_type_dbl_buf    : std_logic;
    signal tran_brs_dbl_buf           : std_logic;

    -- Double buffer register for frame testability
    signal tran_frame_test_dbl_buf    : t_frame_test_w;

    -- Comitted values of internal signals
    signal tran_dlc_com               : std_logic_vector(3 downto 0);
    signal tran_is_rtr_com            : std_logic;
    signal tran_ident_type_com        : std_logic;
    signal tran_frame_type_com        : std_logic;
    signal tran_brs_com               : std_logic;
    signal tran_frame_valid_com       : std_logic;
    signal tran_identifier_com        : std_logic_vector(28 downto 0);

    -----------------------------------------------------------------------------------------------
    -- TX Arbitrator FSM outputs
    -----------------------------------------------------------------------------------------------

    -- Load Timestamp lower word to metadata pointer
    signal load_ts_lw_addr            : std_logic;

    -- Load Timestamp upper word to metadata pointer
    signal load_ts_uw_addr            : std_logic;

    -- Load Frame format word to metadata pointer
    signal load_ffmt_w_addr           : std_logic;

    -- Load identifier word to metadata pointer
    signal load_ident_w_addr          : std_logic;

    -- Load frame test word to metadata pointer
    signal load_frame_test_w_addr     : std_logic;

    -- Store timestamp lower word
    signal store_ts_l_w               : std_logic;

    -- Commit content of double buffer registers to registers visible by CAN Core
    signal commit_dbl_bufs            : std_logic;

    -- Store metadata (Frame format word) to double buffer registers.
    signal buffer_md_w                : std_logic;

    -- Store frame test word to double buffer registers
    signal buffer_frame_test_w        : std_logic;

    -- Store last locked TXT Buffer index
    signal store_last_txtb_index      : std_logic;

    -- Set valid selected buffer on TX Arbitrator output.
    signal frame_valid_com_set        : std_logic;

    -- Clear valid selected buffer on TX Arbitrator output.
    signal frame_valid_com_clear      : std_logic;

    -- TX Arbitrator is locked
    signal tx_arb_locked              : std_logic;

    -- TXT Buffer clock enable for reading metadata
    signal txtb_meta_clk_en           : std_logic;

    -- TXT Buffer Backup mode enable
    signal mr_tx_priority_txbbm       : t_txt_bufs_priorities(G_TXT_BUFFER_COUNT - 1 downto 0);

    -- Parity mismatches in TXT Buffers:
    --  1. Mismatch during TXT Buffer validation
    --  2. Mismatch during transmission
    signal txtb_parity_mismatch_vld   : std_logic;
    signal txtb_parity_mismatch_tx    : std_logic;

    -----------------------------------------------------------------------------------------------
    -- Comparing procedure for two 64 bit std logic vectors
    -----------------------------------------------------------------------------------------------
    function less_than(
        signal   a       : in std_logic_vector(63 downto 0);
        signal   b       : in std_logic_vector(63 downto 0)
    )return std_logic is
    begin
        if (unsigned(a(63 downto 32)) < unsigned(b(63 downto 32))) or
            ((a(63 downto 32) = b(63 downto 32)) and
            (unsigned(a(31 downto 0)) < unsigned(b(31 downto 0))))then
            return '1';
        else
           return '0';
        end if;
    end function;

begin

    -----------------------------------------------------------------------------------------------
    -- Priority decoder on TXT Buffers
    -----------------------------------------------------------------------------------------------
    priority_decoder_inst : entity ctu_can_fd_rtl.priority_decoder
    generic map (
        G_TXT_BUFFER_COUNT    => G_TXT_BUFFER_COUNT
    )
    port map (
        prio                       => mr_tx_priority_txbbm,        -- IN
        prio_valid                 => txtb_available,              -- IN

        output_valid               => select_buf_avail,            -- OUT
        output_index               => select_buf_index             -- OUT
    );


    -----------------------------------------------------------------------------------------------
    -- TX Arbitrator FSM
    -----------------------------------------------------------------------------------------------
    tx_arbitrator_fsm_inst : entity ctu_can_fd_rtl.tx_arbitrator_fsm
    port map (
        clk_sys                     => clk_sys,                     -- IN
        res_n                       => res_n,                       -- IN

        select_buf_avail            => select_buf_avail,            -- IN
        select_index_changed        => select_index_changed,        -- IN
        timestamp_valid             => timestamp_valid,             -- IN
        txtb_hw_cmd                 => txtb_hw_cmd,                 -- IN
        txtb_parity_mismatch_vld    => txtb_parity_mismatch_vld,    -- IN

        load_ts_lw_addr             => load_ts_lw_addr,             -- OUT
        load_ts_uw_addr             => load_ts_uw_addr,             -- OUT
        load_ffmt_w_addr            => load_ffmt_w_addr,            -- OUT
        load_ident_w_addr           => load_ident_w_addr,           -- OUT
        load_frame_test_w_addr      => load_frame_test_w_addr,      -- OUT
        txtb_meta_clk_en            => txtb_meta_clk_en,            -- OUT

        store_ts_l_w                => store_ts_l_w,                -- OUT
        commit_dbl_bufs             => commit_dbl_bufs,             -- OUT
        buffer_frame_test_w         => buffer_frame_test_w,         -- OUT
        buffer_md_w                 => buffer_md_w,                 -- OUT
        tx_arb_locked               => tx_arb_locked,               -- OUT
        store_last_txtb_index       => store_last_txtb_index,       -- OUT
        frame_valid_com_set         => frame_valid_com_set,         -- OUT
        frame_valid_com_clear       => frame_valid_com_clear,       -- OUT
        tx_arb_parity_check_valid   => tx_arb_parity_check_valid    -- OUT
    );

    -----------------------------------------------------------------------------------------------
    -- TXT Buffer differences fo Shuffle priorites of TXT Buffers, in TXT Buffer Backup mode,
    -- replace priority of "Backup" buffer with priority of "original" buffer
    -----------------------------------------------------------------------------------------------
    txtb_priority_gen : for i in 0 to G_TXT_BUFFER_COUNT - 1 generate

        -- Original Buffers
        txtb_priority_even_gen : if ((i mod 2) = 0) generate
            mr_tx_priority_txbbm(i) <= mr_tx_priority(i);
            txtb_is_bb(i) <= '0';
        end generate;

        -- Backup buffers
        txtb_priority_odd_gen : if ((i mod 2) = 1) generate

            mr_tx_priority_txbbm(i) <= mr_tx_priority(i) when (mr_mode_txbbm = '0')
                                                         else
                                       mr_tx_priority(i - 1);

            txtb_is_bb(i) <= '1' when (mr_mode_txbbm = '1' and int_txtb_index = i-1 and
                                       txtb_allow_bb(i - 1) = '1')
                                 else
                             '0';
        end generate;
    end generate;


    -----------------------------------------------------------------------------------------------
    -- Comparing timestamp with external timestamp. This assumes that Upper timestamp is on the
    -- output of buffer and lower timestamp is stored in "ts_low_internal".
    -----------------------------------------------------------------------------------------------
    timestamp_valid <= less_than(txtb_timestamp, timestamp) when (mr_mode_tttm = TTTM_ENABLED)
                                                            else
                                                        '1';

    -----------------------------------------------------------------------------------------------
    -- When TXT Buffer which is currently "Validated" suddenly becomes "Unavailable" (e.g. due to
    -- Set Abort command), this must be signalled to Protocol control in the same clock cycle!
    -- During transmission, CAN Core is reading metadata from outputs. Since the frame is valid,
    -- it is logical to also have "tran_frame_valid" active!
    -----------------------------------------------------------------------------------------------
    validated_buffer <= '1' when (txtb_available(int_txtb_index) = '1') and
                                 (tran_frame_valid_com = '1')
                            else
                        '0';

    tran_frame_valid <= '1' when (validated_buffer = '1') or (tx_arb_locked = '1')
                            else
                        '0';

    -----------------------------------------------------------------------------------------------
    -- Parity mismatch during validation of TXT Buffer, must be indexed using "raw" /
    -- "combinatorial" index of buffer which is currently being validated.
    ------------------------------------------------------------------------------
    txtb_parity_mismatch_vld <= '1' when (txtb_parity_mismatch(select_buf_index) = '1' and
                                          mr_settings_pchke = '1')
                                    else
                                '0';

    -----------------------------------------------------------------------------------------------
    -- Protocol controller reads from TXT Buffer, so if there is parity mismatch one clock cycle
    -- later, this means there was error in data word. In such case, signal to Protocol controller
    -- to start error frame! Buffer one which parity mismatch is checked, must be selected by
    -- registered index of TXT Buffer which is used for transmission.
    -----------------------------------------------------------------------------------------------
    txtb_parity_mismatch_tx <= '1' when (txtb_parity_mismatch(int_txtb_index) = '1' and
                                         mr_settings_pchke = '1')
                                   else
                               '0';

    tran_frame_parity_error <= '1' when (txtb_parity_mismatch_tx = '1' and txtb_clk_en_q = '1')
                                   else
                               '0';

    -----------------------------------------------------------------------------------------------
    -- Selecting TXT Buffer output word based on TXT Buffer index. During transmission, use last
    -- stored TXT buffer. Otherwise use combinatorially selected TXT buffer (during validation).
    -----------------------------------------------------------------------------------------------
    txtb_index_muxed_i <= int_txtb_index when (tx_arb_locked = '1')
                                         else
                          select_buf_index;

    -- Parity check. When Protocol controller wants to enable clock for TXT Buffer, it reads data
    -- words from it for transmission. When this occurs, data will we available on the output of
    -- TXT Buffer RAM one clock cycle later. At this cycle, TXT Buffer needs to check for parity
    -- error.
    txtb_parity_check_valid <= txtb_clk_en_q when (tx_arb_locked = '1')
                                             else
                               tx_arb_parity_check_valid;

    -- Select read data based on index of TXT buffer which should be accessed
    txtb_selected_input <= txtb_port_b_data_out(txtb_index_muxed_i);

    -- Transmitted data word taken from TXT Buffer output
    tran_word <= txtb_selected_input;
    txtb_index_muxed <= txtb_index_muxed_i;

    -----------------------------------------------------------------------------------------------
    -- Joined timestamp from TXT Buffer. Note that it is not always valid! Only when the TXT Buffer
    -- is addressed with upper timestamp word address!
    -----------------------------------------------------------------------------------------------
    txtb_timestamp <= txtb_selected_input & ts_low_internal;

    -----------------------------------------------------------------------------------------------
    -- Output frame metadata and Identifier for CAN Core
    -----------------------------------------------------------------------------------------------
    tran_dlc         <= tran_dlc_com;
    tran_is_rtr      <= tran_is_rtr_com;
    tran_ident_type  <= tran_ident_type_com;
    tran_frame_type  <= tran_frame_type_com;
    tran_brs         <= tran_brs_com;
    tran_identifier  <= tran_identifier_com;

    -----------------------------------------------------------------------------------------------
    -- During Buffer selection, TX Arbitrator is addressing TXT Buffers.
    -- During Transmission, the Core is addressing TXT Buffers. The same goes for clock enable.
    -----------------------------------------------------------------------------------------------
    -- TODO: Convert pointers to std_logic!
    txtb_port_b_address <= std_logic_vector(to_unsigned(txtb_ptr, 5)) when (tx_arb_locked = '1')
                                                                      else
                           std_logic_vector(to_unsigned(txtb_pointer_meta_q, 5));

    txtb_port_b_clk_en <= txtb_clk_en when (tx_arb_locked = '1')
                                      else
                          txtb_meta_clk_en;

    txtb_hw_cmd_index <= int_txtb_index;


    -----------------------------------------------------------------------------------------------
    -- Register for TXT Buffer clock enable
    -----------------------------------------------------------------------------------------------
    txtb_clk_en_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            txtb_clk_en_q <= '0';
        elsif (rising_edge(clk_sys)) then
            txtb_clk_en_q <= txtb_clk_en;
        end if;
    end process;


    -----------------------------------------------------------------------------------------------
    -- Register for loading lower 32 bits of CAN Frame timestamp
    -----------------------------------------------------------------------------------------------
    low_ts_reg_proc : process(res_n, clk_sys)
    begin
        if (res_n = '0') then
            ts_low_internal <= (others => '0');
        elsif (rising_edge(clk_sys)) then
            if (store_ts_l_w = '1') then
                ts_low_internal <= txtb_selected_input;
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------------------------------
    -- Double buffer registers for Metadata.
    -----------------------------------------------------------------------------------------------
    dbl_buf_reg_ffmt_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            tran_dlc_dbl_buf           <= (others => '0');
            tran_is_rtr_dbl_buf        <= '0';
            tran_ident_type_dbl_buf    <= '0';
            tran_frame_type_dbl_buf    <= '0';
            tran_brs_dbl_buf           <= '0';
        elsif (rising_edge(clk_sys)) then
            if (buffer_md_w = '1') then
                tran_dlc_dbl_buf           <= txtb_selected_input(DLC_H downto DLC_L);
                tran_is_rtr_dbl_buf        <= txtb_selected_input(RTR_IND);
                tran_ident_type_dbl_buf    <= txtb_selected_input(IDE_IND);
                tran_frame_type_dbl_buf    <= txtb_selected_input(FDF_IND);
                tran_brs_dbl_buf           <= txtb_selected_input(BRS_IND);
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------------------------------
    -- Double buffer registers for Frame test word
    -----------------------------------------------------------------------------------------------
    dbl_buf_reg_ftw_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            tran_frame_test_dbl_buf.fstc <= '0';
            tran_frame_test_dbl_buf.fcrc <= '0';
            tran_frame_test_dbl_buf.sdlc <= '0';
            tran_frame_test_dbl_buf.tprm <= (others => '0');
        elsif (rising_edge(clk_sys)) then
            if (buffer_frame_test_w = '1') then
                tran_frame_test_dbl_buf.fstc <= txtb_selected_input(FSTC_IND);
                tran_frame_test_dbl_buf.fcrc <= txtb_selected_input(FCRC_IND);
                tran_frame_test_dbl_buf.sdlc <= txtb_selected_input(SDLC_IND);
                tran_frame_test_dbl_buf.tprm <= txtb_selected_input(TPRM_H downto TPRM_L);
            end if;
        end if;
    end process;


    -----------------------------------------------------------------------------------------------
    -- Capture registers for metadata commited to output of TX Arbitrator. Takenfrom double buffer
    -- register.
    -----------------------------------------------------------------------------------------------
    meta_data_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            tran_dlc_com         <= (others => '0');
            tran_is_rtr_com      <= '0';
            tran_ident_type_com  <= '0';
            tran_frame_type_com  <= '0';
            tran_brs_com         <= '0';
        elsif (rising_edge(clk_sys)) then
            if (commit_dbl_bufs = '1') then
                tran_frame_type_com  <= tran_frame_type_dbl_buf;
                tran_ident_type_com  <= tran_ident_type_dbl_buf;
                tran_dlc_com         <= tran_dlc_dbl_buf;
                tran_is_rtr_com      <= tran_is_rtr_dbl_buf;
                tran_brs_com         <= tran_brs_dbl_buf;
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------------------------------
    -- Capture registers for Identifier commited to output of TX Arbitrator.
    -----------------------------------------------------------------------------------------------
    identifier_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            tran_identifier_com <= (others => '0');
        elsif (rising_edge(clk_sys)) then
            if (commit_dbl_bufs = '1') then
                tran_identifier_com <= txtb_selected_input(28 downto 0);
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------------------------------
    -- Capture registers for Frame test word commited to output of TX Arbitrator.
    -----------------------------------------------------------------------------------------------
    frame_test_w_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            tran_frame_test.fstc <= '0';
            tran_frame_test.fcrc <= '0';
            tran_frame_test.sdlc <= '0';
            tran_frame_test.tprm <= (others => '0');
        elsif (rising_edge(clk_sys)) then
            if (commit_dbl_bufs = '1') then
                tran_frame_test <= tran_frame_test_dbl_buf;
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------------------------------
    -- Register for "committed" valid frame output for CAN Core
    -----------------------------------------------------------------------------------------------
    tran_frame_valid_com_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            tran_frame_valid_com        <= '0';
        elsif (rising_edge(clk_sys)) then
            if (frame_valid_com_set = '1') then
                tran_frame_valid_com    <= '1';
            elsif (frame_valid_com_clear = '1') then
                tran_frame_valid_com    <= '0';
            end if;
        end if;
    end process;


    -----------------------------------------------------------------------------------------------
    -- Storing values of selected TXT Buffer index when selection process ends. Storing TXT Buffer
    -- at the time of LOCK from CAN Core. Two values are needed to determine change of selected TXT
    -- Buffer for CAN Core. CAN Core needs this information for erasing retransmitt limit counter.
    -----------------------------------------------------------------------------------------------
    store_indices_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            last_txtb_index             <= 0;
            int_txtb_index              <= 0;

        elsif (rising_edge(clk_sys)) then
            -- At the time of lock, the last index is stored from the last stored index.
            if (store_last_txtb_index = '1') then
                last_txtb_index         <= int_txtb_index;
            end if;

            -- Combinationally selected index (select_buf_index) is stored when metadata are stored.
            if (commit_dbl_bufs = '1') then
                int_txtb_index          <= select_buf_index;
            end if;

        end if;
    end process;

    txtb_changed  <= '1' when (last_txtb_index /= int_txtb_index and store_last_txtb_index = '1')
                         else
                     '0';


    -----------------------------------------------------------------------------------------------
    -- Registering value of combinationally selected index by priority decoder to determine change
    -- and signal restarting selection process to TX Arbitrator FSM.
    -----------------------------------------------------------------------------------------------
    sel_index_change_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            select_buf_index_reg  <= 0;
        elsif (rising_edge(clk_sys)) then
            select_buf_index_reg <= select_buf_index;
        end if;
    end process;

    select_index_changed <= '0' when (select_buf_index = select_buf_index_reg)
                                else
                            '1';


    -----------------------------------------------------------------------------------------------
    -- Metadata pointer to address TXT Buffer Timestamp and Metadata Words.
    -----------------------------------------------------------------------------------------------
    txtb_pointer_meta_d <=
        to_integer(unsigned(TIMESTAMP_L_W_ADR(11 downto 2))) when (load_ts_lw_addr = '1') else
        to_integer(unsigned(TIMESTAMP_U_W_ADR(11 downto 2))) when (load_ts_uw_addr = '1') else
        to_integer(unsigned(FRAME_FORMAT_W_ADR(11 downto 2))) when (load_ffmt_w_addr = '1') else
        to_integer(unsigned(IDENTIFIER_W_ADR(11 downto 2))) when (load_ident_w_addr = '1') else
        to_integer(unsigned(FRAME_TEST_W_ADR(11 downto 2))) when (load_frame_test_w_addr = '1') else
        txtb_pointer_meta_q;

    store_meta_data_ptr_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            txtb_pointer_meta_q <= to_integer(unsigned(TIMESTAMP_L_W_ADR(11 downto 2)));
        elsif (rising_edge(clk_sys)) then
            txtb_pointer_meta_q <= txtb_pointer_meta_d;
        end if;
    end process;


    -- <RELEASE_OFF>
    -----------------------------------------------------------------------------------------------
    -- Functional coverage
    -----------------------------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);

    -- psl txt_lock_cov : cover
    --    {txtb_hw_cmd.lock = '1'};
    --
    -- psl txt_unlock_cov : cover
    --    {txtb_hw_cmd.unlock = '1'};
    --
    --
    -- Lock Commands
    --
    -- psl txt_lock_buf_1_cov : cover
    --    {txtb_hw_cmd_index = 0 and txtb_hw_cmd.lock = '1'};
    -- psl txt_lock_buf_2_cov : cover
    --    {txtb_hw_cmd_index = 1 and txtb_hw_cmd.lock = '1'};
    -- psl txt_lock_buf_3_cov : cover
    --    {txtb_hw_cmd_index = 2 and txtb_hw_cmd.lock = '1'};
    -- psl txt_lock_buf_4_cov : cover
    --    {txtb_hw_cmd_index = 3 and txtb_hw_cmd.lock = '1'};
    -- psl txt_lock_buf_5_cov : cover
    --    {txtb_hw_cmd_index = 4 and txtb_hw_cmd.lock = '1'};
    -- psl txt_lock_buf_6_cov : cover
    --    {txtb_hw_cmd_index = 5 and txtb_hw_cmd.lock = '1'};
    -- psl txt_lock_buf_7_cov : cover
    --    {txtb_hw_cmd_index = 6 and txtb_hw_cmd.lock = '1'};
    -- psl txt_lock_buf_8_cov : cover
    --    {txtb_hw_cmd_index = 7 and txtb_hw_cmd.lock = '1'};

    -- Unlock Commands
    --
    -- psl txt_unlock_buf_1_cov : cover
    --    {txtb_hw_cmd_index = 0 and txtb_hw_cmd.unlock = '1'};
    -- psl txt_unlock_buf_2_cov : cover
    --    {txtb_hw_cmd_index = 1 and txtb_hw_cmd.unlock = '1'};
    -- psl txt_unlock_buf_3_cov : cover
    --    {txtb_hw_cmd_index = 2 and txtb_hw_cmd.unlock = '1'};
    -- psl txt_unlock_buf_4_cov : cover
    --    {txtb_hw_cmd_index = 3 and txtb_hw_cmd.unlock = '1'};
    -- psl txt_unlock_buf_5_cov : cover
    --    {txtb_hw_cmd_index = 4 and txtb_hw_cmd.unlock = '1'};
    -- psl txt_unlock_buf_6_cov : cover
    --    {txtb_hw_cmd_index = 5 and txtb_hw_cmd.unlock = '1'};
    -- psl txt_unlock_buf_7_cov : cover
    --    {txtb_hw_cmd_index = 6 and txtb_hw_cmd.unlock = '1'};
    -- psl txt_unlock_buf_8_cov : cover
    --    {txtb_hw_cmd_index = 7 and txtb_hw_cmd.unlock = '1'};

    -- Modes
    -- Note: We use gating by tran_frame_valid to avoid falsly covered scenarios,
    --       where reset value has the mode disabled!
    --
    -- psl txtb_ttm_ena_cov : cover
    --    {mr_mode_tttm = '1' and tran_frame_valid = '1'};
    -- psl txtb_ttm_dis_cov : cover
    --    {mr_mode_tttm = '0' and tran_frame_valid = '1'};

    -- psl txtb_txbbm_ena_cov : cover
    --    {mr_mode_txbbm = '1' and tran_frame_valid = '1'};
    -- psl txtb_txbbm_dis_cov : cover
    --    {mr_mode_txbbm = '0' and tran_frame_valid = '1'};


    -- Change of priority when there is validated TXT buffer
    --
    -- psl txt_prio_change_cov : cover
    --    {select_buf_avail = '1';
    --     select_buf_avail = '1' and (select_buf_index /= select_buf_index'LAST_VALUE);
    --     select_buf_avail = '1'};


    -- Change of buffer from available to not available but not due to lock
    --  (e.g. set abort). Again one buffer is enough!
    -- psl buf_ready_to_not_ready_cov : cover
    --    {txtb_available(0) = '1' and select_buf_index = 0 and
    --     txtb_hw_cmd.lock = '0'; txtb_available(0) = '0'}
    --    report "Buffer became non-ready but not due to lock command";
    --
    --
    -- psl txt_buf_change_cov : cover
    --    {txtb_changed = '1' and txtb_hw_cmd.lock = '1'}
    --    report "TX Buffer changed between two frames";
    --
    -- psl txt_buf_sim_chng_and_lock_cov : cover
    --    {select_index_changed = '1' and txtb_hw_cmd.lock = '1'};


    -----------------------------------------------------------------------------------------------
    -- Functional coverage for Priority decoder!
    --
    -- Cover points for each combination of inputs in first stage
    -----------------------------------------------------------------------------------------------

    -- First comparator cell
    txtb_stage_1_cell_1_gen : if G_TXT_BUFFER_COUNT > 1 generate
    begin
        -- psl prio_dec_stage_1_cell_1_cov_1 : cover
        --  {(unsigned(mr_tx_priority(0)) > unsigned(mr_tx_priority(1))) and
        --   txtb_available(0) = '1' and txtb_available(1) = '1'};

        -- psl prio_dec_stage_1_cell_1_cov_2 : cover
        --  {(unsigned(mr_tx_priority(0)) < unsigned(mr_tx_priority(1))) and
        --   txtb_available(0) = '1' and txtb_available(1) = '1'};

        -- psl prio_dec_stage_1_cell_1_cov_3 : cover
        --  {(unsigned(mr_tx_priority(0)) = unsigned(mr_tx_priority(1))) and
        --   txtb_available(0) = '1' and txtb_available(1) = '1'};

        -- psl prio_dec_stage_1_cell_1_cov_4 : cover
        --  {txtb_available(0) = '0' and txtb_available(1) = '1'};

        -- psl prio_dec_stage_1_cell_1_cov_5 : cover
        --  {txtb_available(0) = '1' and txtb_available(1) = '0'};

        -- psl txt_buf_eq_priority_stage_1_cell_1_cov : cover
        --    {txtb_available(0) = '1' and txtb_available(1) = '1' and
        --     mr_tx_priority(0) = mr_tx_priority(1)};

        -- psl txt_buf_all_available_stage_1_cov : cover
        --    {txtb_available(0) = '1' and txtb_available(1) = '1'};

    end generate txtb_stage_1_cell_1_gen;

    -- Second comparator cell
    txtb_stage_1_cell_2_gen : if G_TXT_BUFFER_COUNT > 3 generate
    begin
        -- psl prio_dec_stage_1_cell_2_cov_1 : cover
        --  {(unsigned(mr_tx_priority(2)) > unsigned(mr_tx_priority(3))) and
        --   txtb_available(2) = '1' and txtb_available(3) = '1'};

        -- psl prio_dec_stage_1_cell_2_cov_2 : cover
        --  {(unsigned(mr_tx_priority(2)) < unsigned(mr_tx_priority(3))) and
        --   txtb_available(2) = '1' and txtb_available(3) = '1'};

        -- psl prio_dec_stage_1_cell_2_cov_3 : cover
        --  {(unsigned(mr_tx_priority(2)) = unsigned(mr_tx_priority(3))) and
        --   txtb_available(2) = '1' and txtb_available(3) = '1'};

        -- psl prio_dec_stage_1_cell_2_cov_4 : cover
        --  {txtb_available(2) = '0' and txtb_available(3) = '1'};

        -- psl prio_dec_stage_1_cell_2_cov_5 : cover
        --  {txtb_available(2) = '1' and txtb_available(3) = '0'};

        -- psl txt_buf_eq_priority_stage_1_cell_2_cov : cover
        --    {txtb_available(2) = '1' and txtb_available(3) = '1' and
        --     mr_tx_priority(2) = mr_tx_priority(3)};

        -- psl txt_buf_all_available_stage_2_cov : cover
        --    {txtb_available(0) = '1' and txtb_available(1) = '1' and
        --     txtb_available(2) = '1' and txtb_available(3) = '1'};
    end generate txtb_stage_1_cell_2_gen;

    -- Third comparator cell
    txtb_stage_1_cell_3_gen : if G_TXT_BUFFER_COUNT > 5 generate
    begin
        -- psl prio_dec_stage_1_cell_3_cov_1 : cover
        --  {(unsigned(mr_tx_priority(4)) > unsigned(mr_tx_priority(5))) and
        --   txtb_available(4) = '1' and txtb_available(5) = '1'};

        -- psl prio_dec_stage_1_cell_3_cov_2 : cover
        --  {(unsigned(mr_tx_priority(4)) < unsigned(mr_tx_priority(5))) and
        --   txtb_available(4) = '1' and txtb_available(5) = '1'};

        -- psl prio_dec_stage_1_cell_3_cov_3 : cover
        --  {(unsigned(mr_tx_priority(4)) = unsigned(mr_tx_priority(5))) and
        --   txtb_available(4) = '1' and txtb_available(5) = '1'};

        -- psl prio_dec_stage_1_cell_3_cov_4 : cover
        --  {txtb_available(4) = '0' and txtb_available(5) = '1'};

        -- psl prio_dec_stage_1_cell_3_cov_5 : cover
        --  {txtb_available(4) = '1' and txtb_available(5) = '0'};

        -- psl txt_buf_eq_priority_stage_1_cell_3_cov : cover
        --    {txtb_available(4) = '1' and txtb_available(5) = '1' and
        --     mr_tx_priority(4) = mr_tx_priority(5)};

        -- psl txt_buf_all_available_stage_3_cov : cover
        --    {txtb_available(0) = '1' and txtb_available(1) = '1' and
        --     txtb_available(2) = '1' and txtb_available(3) = '1' and
        --     txtb_available(4) = '1' and txtb_available(5) = '1'};
    end generate txtb_stage_1_cell_3_gen;

    -- Fourth comparator cell
    txtb_stage_1_cell_4_gen : if G_TXT_BUFFER_COUNT > 7 generate
    begin
        -- psl prio_dec_stage_1_cell_4_cov_1 : cover
        --  {(unsigned(mr_tx_priority(6)) > unsigned(mr_tx_priority(7))) and
        --   txtb_available(6) = '1' and txtb_available(7) = '1'};

        -- psl prio_dec_stage_1_cell_4_cov_2 : cover
        --  {(unsigned(mr_tx_priority(6)) < unsigned(mr_tx_priority(7))) and
        --   txtb_available(6) = '1' and txtb_available(7) = '1'};

        -- psl prio_dec_stage_1_cell_4_cov_3 : cover
        --  {(unsigned(mr_tx_priority(6)) = unsigned(mr_tx_priority(7))) and
        --   txtb_available(6) = '1' and txtb_available(7) = '1'};

        -- psl prio_dec_stage_1_cell_4_cov_4 : cover
        --  {txtb_available(6) = '0' and txtb_available(7) = '1'};

        -- psl prio_dec_stage_1_cell_4_cov_5 : cover
        --  {txtb_available(6) = '1' and txtb_available(7) = '0'};

        -- psl txt_buf_eq_priority_stage_1_cell_4_cov : cover
        --    {txtb_available(6) = '1' and txtb_available(7) = '1' and
        --     mr_tx_priority(6) = mr_tx_priority(7)};

        -- psl txt_buf_all_available_stage_4_cov : cover
        --    {txtb_available(0) = '1' and txtb_available(1) = '1' and
        --     txtb_available(2) = '1' and txtb_available(3) = '1' and
        --     txtb_available(4) = '1' and txtb_available(5) = '1' and
        --     txtb_available(6) = '1' and txtb_available(7) = '1'};
    end generate txtb_stage_1_cell_4_gen;

    -----------------------------------------------------------------------------------------------
    -- Assertions
    -----------------------------------------------------------------------------------------------
    -- When TXT Buffer is not ready for more than one cycle, LOCK command might not occur. If it is
    -- not ready for one clock cycle, it might still be due to set abort and LOCK command applied
    -- simultaneously. This is OK. But as soon as buffer is not ready for second cycle, LOCK command
    -- can't be active!
    --
    -- psl txtb_no_lock_when_not_ready_asrt : assert never
    --   {tran_frame_valid = '0';
    --    tran_frame_valid = '0' and txtb_hw_cmd.lock = '1'}
    --   report "NO TXT Buffer ready and lock occurs!";
    -----------------------------------------------------------------------------------------------
    -- <RELEASE_ON>

end architecture;