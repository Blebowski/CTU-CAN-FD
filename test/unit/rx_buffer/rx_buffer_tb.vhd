--------------------------------------------------------------------------------
--
-- CTU CAN FD IP Core
-- Copyright (C) 2021-2023 Ondrej Ille
-- Copyright (C) 2023-     Logic Design Services Ltd.s
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to use, copy, modify, merge, publish, distribute the Component for
-- non-commercial purposes. Using the Component for commercial purposes is
-- forbidden unless previously agreed with Copyright holder.
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
-- @TestInfoStart
--
-- @Purpose:
--  Unit test for the RX Buffer circuit.
--
-- @Verifies:
--  @1. Storing protocol to RX Buffer (store metadata, store data word, reception
--      valid, reception abort).
--  @2. Storing of Metadata, Identifier and Data words to RX Buffer.
--  @2. Reading protocol from RX Buffer. Reading of CAN frame from RX Buffer.
--  @3. Over-run detection by RX Buffer (frame is discarded when overrun is
--      detected).
--  @4. Simultaneous commit and finishing read of frame from RX Buffer.
--
-- @Test sequence:
--  @1. Generate random CAN frames on input of RX Buffer. Emulate storing protocol
--      as if coming from CAN Core. Randomize whether abort will be issued.
--      (As if error frame was occured). Randomize time between frames. Randomize
--      timestamp capturing in SOF or EOF.
--  @2. If Overrun is signalled or frame abort is issued, discard the frame. If
--      frame storing finished succesfully, store the frame also to auxiliarly
--      memory (Input memory). This memory contains what all has been stored to
--      RX Buffer.
--  @3. Read frames from RX Buffer with random gaps between (emulate read
--      protocol). If frame is read, store it to auxiliarly memory (Output memory).
--      Output memory contains what all has been read from RX Buffer.
--  @4. When Input memory is filled, stop generating CAN frames to RX Buffer.
--      Wait until all frames are read from RX Buffer and compare contents of
--      Input memory and Output memory (everything what was succesfully stored
--      to RX Buffer must be also in the same order read from RX Buffer). This
--      verifies proper pointer handling.

-- @Notes:
--  Following test instantiates RX Buffer. Stimuli generator generates input
--  frames as CAN Core would do. Then it checks whether frame was stored into
--  the buffer! Another process reads the data as user would do by memory access.
--  Both, data written into the buffer, and data read from the buffer are stored
--  into test memories (in_mem,out_mem). When test memory is full content of
--  both memories is compared! When mismatch occurs test fails. Each time memory
--  is filled test moves to the next iteration.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    1.6.2016   Created file
--   22.6.2016   Updated testbench to cover also the modified functionality of
--               RX Buffer. Now ESI bit is also stored and compared. Also RTR
--               frame of CAN normal frame does not store any data words into
--               the buffer.
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;
use ieee.std_logic_textio.all;
use STD.textio.all;

-- Only top level uses Vunit. This allows keeping CTU CAN FD VIP Vunit-less,
-- when integrating RTL and VIP into other TB!
library vunit_lib;
context vunit_lib.vunit_context;

-- Common contexts
Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;
context ctu_can_fd_tb.tb_agents_context;
context ctu_can_fd_tb.rtl_context;

library ctu_can_fd_tb_unit;
use ctu_can_fd_tb_unit.random_unit_pkg.all;

entity rx_buffer_tb is
    generic (
        runner_cfg      : string         := runner_cfg_default;
        test_name       : string         := "dummy";
        finish_on_error : integer        := 0;
        iterations      : natural        := 1;
        error_tol       : natural        := 0;
        timeout         : string         := "0 ms";
        seed            : natural        := 0
    );
end entity;

architecture test of rx_buffer_tb is

    -- Port width determining constants
    constant C_RX_BUFF_SIZE             : natural := 256;
    constant C_RX_BUFF_PTR_WIDTH        : natural := integer(ceil(log2(real(C_RX_BUFF_SIZE))));
    constant C_RX_BUF_FRAME_CNT_WIDTH   : natural := integer(ceil(log2(real(C_RX_BUFF_SIZE)))) - 1;

    -- Common test specific data
    signal error_ctr                : integer := 0;
    signal loop_ctr                 : integer := 0;

    -- System clock and reset
    signal clk_sys                  : std_logic := '0';
    signal res_n                    : std_logic := '0';

    -- Metadata and idntifier
    signal rec_ident                : std_logic_vector(28 downto 0) := (others => '0');
    signal rec_dlc                  : std_logic_vector(3 downto 0) := (OTHERS => '0');
    signal rec_ident_type           : std_logic := '0';
    signal rec_frame_type           : std_logic := '0';
    signal rec_is_rtr               : std_logic := '0';
    signal rec_brs                  : std_logic := '0';
    signal rec_esi                  : std_logic := '0';
    signal rec_lbpf                 : std_logic := '0';
    signal rec_ivld                 : std_logic := '0';

    -- Control signals from CAN Core
    signal store_metadata_f         : std_logic := '0';
    signal store_data_f             : std_logic := '0';
    signal store_data_word          : std_logic_vector(31 downto 0) := (OTHERS => '0');
    signal rec_valid_f              : std_logic := '0';
    signal rec_abort_f              : std_logic := '0';
    signal sof_pulse                : std_logic := '0';

    signal timestamp                : std_logic_vector(63 downto 0) := (OTHERS => '0');

    signal rx_full                  : std_logic;
    signal rx_empty                 : std_logic;
    signal rx_frame_count           : std_logic_vector(C_RX_BUF_FRAME_CNT_WIDTH-1 downto 0);
    signal rx_mem_free              : std_logic_vector(C_RX_BUFF_PTR_WIDTH downto 0);
    signal rx_read_pointer          : std_logic_vector(C_RX_BUFF_PTR_WIDTH-1 downto 0);
    signal rx_write_pointer         : std_logic_vector(C_RX_BUFF_PTR_WIDTH-1 downto 0);
    signal rx_data_overrun          : std_logic;

    signal rxb_port_b_data_out      : std_logic_vector(31 downto 0);

    -- Memory registers signals
    signal mr_mode_rxbam            : std_logic := '1';
    signal mr_command_cdo           : std_logic := '0';
    signal mr_command_crxpe         : std_logic := '0';
    signal mr_command_rrb           : std_logic := '0';
    signal mr_command_rxrpmv        : std_logic := '0';
    signal mr_rx_data_read          : std_logic := '0';
    signal mr_rx_settings_rtsop     : std_logic := RTS_END;
    signal mr_settings_pchke        : std_logic := '0';

    -- Memory testability
    signal mr_tst_control_tmaena    : std_logic;
    signal mr_tst_control_twrstb    : std_logic;
    signal mr_tst_dest_tst_addr     : std_logic_vector(15 downto 0);
    signal mr_tst_dest_tst_mtgt     : std_logic_vector(3 downto 0);
    signal mr_tst_wdata_tst_wdata   : std_logic_vector(31 downto 0);
    signal mr_tst_rdata_tst_rdata   : std_logic_vector(31 downto 0);

    ----------------------------------------------------------------------------
    -- Test specific signals
    ----------------------------------------------------------------------------

    signal iteration_done           : boolean     := false;
    signal in_mem_full              : boolean     := false;
    signal out_mem_full             : boolean     := false;

    -- Error counters
    signal stim_errs                : natural     := 0;
    signal read_errs                : natural     := 0;
    signal status_errs              : natural     := 0;
    signal cons_errs                : natural     := 0;

    ----------------------------------------------------------------------------
    -- Memory declarations for memories where data are read out
    ----------------------------------------------------------------------------
    type eval_mem_test is array (0 to 1023) of
        std_logic_vector(31 downto 0);

    signal in_mem                   : eval_mem_test := (OTHERS => (OTHERS => '0'));
    signal out_mem                  : eval_mem_test := (OTHERS => (OTHERS => '0'));

    signal in_pointer               : natural := 0;
    signal out_pointer              : natural := 0;
    signal mod_pointer              : natural := 0;

    signal ts_preset                : std_logic_vector(2 downto 1) := "00";
    signal ts_preset_val            : std_logic_vector(63 downto 0) := (OTHERS => '0');

    ----------------------------------------------------------------------------
    -- Insert frame to test memory
    ----------------------------------------------------------------------------
    procedure insert_frame_test_mem(
        constant frame              :in     t_ctu_frame;
        signal   memory             :inout  eval_mem_test;
        signal   in_pointer         :inout  natural
    )is
        variable rwcnt_vect         :       std_logic_vector(4 downto 0);
        variable length             :       natural;
        variable hw_id              :       std_logic_vector(28 downto 0);
    begin
        -- FRAME_FORMAT_W
        rwcnt_vect           := std_logic_vector(to_unsigned(frame.rwcnt, 5));
        memory(in_pointer)   <= "0000000100000000" &
                              rwcnt_vect &
                              frame.esi &
                              frame.brs &
                              frame.lbpf &
                              frame.frame_format &
                              frame.ident_type &
                              frame.rtr &
                              '0' &  -- No need to support error frames in unit test
                              frame.dlc;
        -- IDENTIFIER_W
        id_sw_to_hw(frame.identifier, frame.ident_type, hw_id);
        memory(in_pointer + 1) <= "000" & hw_id;


        -- TIMESTAMP_U_W and TIMESTAMP_L_W
        memory(in_pointer + 2) <= frame.timestamp(31 downto 0);

        -- Note that here we have to store timestamp increased by two, because
        -- timestamp is in this test increasing by one every clock cycle!!
        -- thus when timestamp is acutally stored into RX buffer it is two
        -- clock cycles later!!!
        memory(in_pointer + 3) <= std_logic_vector(unsigned(
                                   frame.timestamp(63 downto 32)));

        in_pointer <= in_pointer + 4;

        wait for 0 ns;

        if (frame.rtr = RTR_FRAME) then
            length := 0;
        else
            dlc_to_length(frame.dlc, length);
        end if;

        -- Store the data
        if (length > 0) then
            for i in 0 to (length - 1) / 4 loop
                memory(in_pointer)   <= frame.data((i * 4) + 3) &
                                        frame.data((i * 4) + 2) &
                                        frame.data((i * 4) + 1) &
                                        frame.data((i * 4));
                in_pointer           <= in_pointer + 1;
                wait for 0 ns;
            end loop;
        end if;

        -- At the end we need to move one more time
        -- in_pointer              <= in_pointer+1;
        wait for 0 ns;

    end procedure;


    ----------------------------------------------------------------------------
    -- Generates random abort condition as IF coming from CAN Core
    ----------------------------------------------------------------------------
    procedure generate_random_abort(
        signal   rec_abort_f            :out    std_logic;
        signal   clk_sys              :in     std_logic;
        variable abort_present        :out    boolean;
        constant chances              :in     real
    )is
        variable rand_val             :       std_logic;
    begin
        rand_logic_v(rand_val, chances);
        abort_present := false;

        if (rand_val = '1') then
            rec_abort_f  <= '1';
            wait until rising_edge(clk_sys);
            info_m("Data storing was aborted!");

            rec_abort_f  <= '0';
            wait until rising_edge(clk_sys);
            abort_present := true;
        end if;
    end procedure;


    ----------------------------------------------------------------------------
    -- Executes following steps:
    --  1. Generates random CAN frame.
    --  2. Inserts the frame to RX Buffer as CAN Core. Randomized abort of
    --     storing is generated (as if error frame was generated)!
    --  3. Checks for data overrun flag during storing. If overrun appeared, or
    --     error frame was generated, data are not stored in test memory.
    --  4. If storing was not aborted, nor data overrun was generated, data
    --     are stored to "input memory"!
    ----------------------------------------------------------------------------
    procedure insert_frame_to_RX_Buffer(
        signal   clk_sys                :in     std_logic;

        -- Received Metadata and identifier
        signal   rec_ident              :out    std_logic_vector(28 downto 0);
        signal   rec_dlc                :out    std_logic_vector(3 downto 0);
        signal   rec_frame_type         :out    std_logic;
        signal   rec_ident_type         :out    std_logic;
        signal   rec_brs                :out    std_logic;
        signal   rec_esi                :out    std_logic;
        signal   rec_lbpf               :out    std_logic;
        signal   rec_ivld               :out    std_logic;
        signal   rec_rtr                :out    std_logic;

        -- Storing protocol between RX Buffer and CAN Core
        signal   sof_pulse              :out    std_logic;
        signal   store_metadata_f       :out    std_logic;
        signal   store_data_f           :out    std_logic;
        signal   store_data_word        :out    std_logic_vector(31 downto 0);
        signal   rec_abort_f            :out    std_logic;
        signal   rec_valid_f            :out    std_logic;

        signal   mr_rx_settings_rtsop   :in     std_logic;
        signal   mr_command_cdo         :inout  std_logic;

        signal   memory                 :inout  eval_mem_test;
        signal   in_pointer             :inout  natural;
        signal   timestamp              :in     std_logic_vector(63 downto 0)
   )is
        variable can_frame          :       t_ctu_frame;
        variable stored_ts          :       std_logic_vector(63 downto 0);
        variable rand_val           :       natural;
        variable abort_present      :       boolean := false;
        variable id_out             :       std_logic_vector(28 downto 0);
    begin

        generate_can_frame(can_frame);
        stored_ts := (OTHERS => '0');

        ------------------------------------------------------------------------
        -- Initiate frame storing by clearing possible overrun from before.
        -- It might have happened that Overrun was generated at the same time
        -- as there was intent abort. In that case, the frame was aborted,
        -- overrun was not cleared and stayed till next frame. Storing of
        -- next frame then evaluated overrun as present and did not store the
        -- frame to input memory!
        ------------------------------------------------------------------------
        mr_command_cdo <= '1';
        wait until rising_edge(clk_sys);
        mr_command_cdo <= '0';
        wait for 1 ns;

        -- Check that overrun was cleared
        check_m(rx_data_overrun = '0', "Overrun not cleared!");

        ------------------------------------------------------------------------
        -- Initiate Frame by SOF pulse and store timestamp!
        ------------------------------------------------------------------------
        sof_pulse           <= '1';
        if (mr_rx_settings_rtsop = RTS_BEG) then
            stored_ts   := std_logic_vector(to_unsigned(
                            to_integer(unsigned(timestamp)) + 1, 64));
        end if;
        wait until rising_edge(clk_sys);
        sof_pulse           <= '0';
        wait until rising_edge(clk_sys);

        ------------------------------------------------------------------------
        -- Wait Random time (to emulate CAN ID). No real need to emulate real
        -- length of Identifier! Emulate random error also during this time,
        -- error frame may come also before any storing started and can not FUCK
        -- UP the buffer.
        ------------------------------------------------------------------------
        wait_rand_cycles(clk_sys, 10, 50);

        generate_random_abort(rec_abort_f, clk_sys, abort_present, 0.1);

        if (abort_present) then
            wait until rising_edge(clk_sys);
            wait until rising_edge(clk_sys);
            return;
        end if;

        wait_rand_cycles(clk_sys, 10, 50);

        -- Put metadata on input of RX Buffer!
        id_sw_to_hw(can_frame.identifier, can_frame.ident_type, id_out);
        rec_ident          <= id_out;
        rec_dlc            <= can_frame.dlc;
        rec_frame_type     <= can_frame.frame_format;
        rec_ident_type     <= can_frame.ident_type;
        rec_brs            <= can_frame.brs;
        rec_esi            <= can_frame.esi;
        rec_lbpf           <= can_frame.lbpf;
        rec_ivld           <= '1'; -- For data frame IVLD is always 1
        rec_rtr            <= can_frame.rtr;

        info_m("Storing metadata");
        wait until rising_edge(clk_sys);

        -- Send signal to store metadata
        store_metadata_f     <= '1';
        wait until rising_edge(clk_sys);
        store_metadata_f     <= '0';
        wait until rising_edge(clk_sys);

        ------------------------------------------------------------------------
        -- Store data words
        ------------------------------------------------------------------------
        if (can_frame.data_length > 0) then
            for i in 0 to ((can_frame.data_length - 1) / 4) loop

                -- Wait random time between store of individual data bytes!
                wait_rand_cycles(clk_sys, 10, 50);

                -- Send signal to store data
                store_data_word <= can_frame.data((i * 4) + 3) &
                                   can_frame.data((i * 4) + 2) &
                                   can_frame.data((i * 4) + 1) &
                                   can_frame.data((i * 4));

                store_data_f      <= '1';
                info_m("Storing data word");
                wait until rising_edge(clk_sys);
                store_data_f      <= '0';
                wait until rising_edge(clk_sys);

                generate_random_abort(rec_abort_f, clk_sys, abort_present, 0.05);
                if (abort_present) then
                    wait until rising_edge(clk_sys);
                    wait until rising_edge(clk_sys);
                    return;
                end if;
            end loop;
        end if;

        wait_rand_cycles(clk_sys, 30, 100);

        ------------------------------------------------------------------------
        -- If we got here, no abort was generated, thus frame was stored OK!
        -- We commit frame to the buffer and store it to test memories!
        ------------------------------------------------------------------------
        rec_valid_f <= '1';
        info_m("Frame valid!");
        wait until rising_edge(clk_sys);

        ------------------------------------------------------------------------
        -- Timestamp must be marked, if we are interested in END OF Frame
        -- Timestamp!
        ------------------------------------------------------------------------
        if (mr_rx_settings_rtsop = RTS_END) then
            can_frame.timestamp  := timestamp;
        else
            can_frame.timestamp  := stored_ts;
        end if;
        rec_valid_f <= '0';

        ------------------------------------------------------------------------
        -- Check that during whole storing of this frame data overrun did not
        -- occur!
        ------------------------------------------------------------------------
        if (rx_data_overrun = '1') then
            info_m("Data overrun appeared!");

        ------------------------------------------------------------------------
        -- If overrun did not happend, insert frame to input test memory!
        ------------------------------------------------------------------------
        else
            insert_frame_test_mem(can_frame, memory, in_pointer);
        end if;

        wait until rising_edge(clk_sys);
        wait until rising_edge(clk_sys);
        wait until rising_edge(clk_sys);

    end procedure;


    ----------------------------------------------------------------------------
    -- Read frame from the RX buffer and store it into the common model
    -- and output memory!
    ----------------------------------------------------------------------------
    procedure read_frame(
        signal buff_out        :in    std_logic_vector(31 downto 0);
        signal mr_rx_data_read :inout std_logic;
        signal clk_sys         :in    std_logic;
        signal out_mem         :out   eval_mem_test;
        signal in_mem          :in    eval_mem_test;
        signal out_pointer     :inout natural
    )is
        variable rwcnt         :      natural;
    begin

        -- RWCNT field in first word gives us number of words per frame without
        -- frame format word!
        rwcnt := to_integer(unsigned(buff_out(RWCNT_H downto RWCNT_L)));

        -- Reading all words in cycle and storing to output memory!
        for i in 0 to rwcnt loop
            mr_rx_data_read       <= '1';
            out_mem(out_pointer)  <= buff_out;

            -------------------------------------------------------------------
            -- Check that word is exactly matching the word in in_mem at the
            -- same position!
            -------------------------------------------------------------------
            info_m("Buffer output: " & to_hstring(buff_out));
            info_m("Model output: " & to_hstring(in_mem(out_pointer)));
            info_m("Word nr. :" & integer'image(i));
            check_m(buff_out = in_mem(out_pointer),
                    "Buffer inconsistency, index: " & integer'image(out_pointer) &
                    " Expected: " & to_string(in_mem(out_pointer)) &
                    " Observed: " & to_string(buff_out));

            out_pointer           <= out_pointer + 1;
            wait until rising_edge(clk_sys);
            mr_rx_data_read       <= '0';
            wait until rising_edge(clk_sys);
        end loop;

    end procedure;


    ----------------------------------------------------------------------------
    -- Compare contents of input and output memory, if data stored to the buffer
    -- are equal to data read from the buffer!
    ----------------------------------------------------------------------------
    procedure compare_data(
        signal in_mem          :in   eval_mem_test;
        signal out_mem         :in   eval_mem_test;
        variable cons_res      :out  boolean
    )is
    begin
        cons_res := true;
        for i in 0 to in_mem'length - 1 loop
            if (in_mem(i) /= out_mem(i)) then
                info_m("Consistency mismatch at index: " & integer'image(i) &
                       " IN_MEM:  " & to_string(in_mem(i)) &
                       " OUT_MEM: " & to_string(out_mem(i)));
                cons_res := false;
            end if;
        end loop;
    end procedure;

begin

    ----------------------------------------------------------------------------
    -- Buffer component
    ----------------------------------------------------------------------------
    i_rx_buffer : entity ctu_can_fd_rtl.rx_buffer
    generic map(
        G_RX_BUFF_SIZE              => C_RX_BUFF_SIZE,
        G_RX_BUFF_PTR_WIDTH         => C_RX_BUFF_PTR_WIDTH,
        G_RX_BUF_FRAME_CNT_WIDTH    => C_RX_BUF_FRAME_CNT_WIDTH,
        G_SUP_PARITY                => true,
        G_RESET_RX_BUF_RAM          => false,
        G_TECHNOLOGY                => C_TECH_FPGA
    )
    port map(
        clk_sys                  => clk_sys,
        res_n                    => res_n,
        scan_enable              => '0',

        rec_ident                => rec_ident,
        rec_dlc                  => rec_dlc,
        rec_ident_type           => rec_ident_type,
        rec_frame_type           => rec_frame_type,
        rec_is_rtr               => rec_is_rtr,
        rec_brs                  => rec_brs,
        rec_esi                  => rec_esi,
        rec_lbpf                 => rec_lbpf,
        rec_ivld                 => rec_ivld,

        err_capt_err_type        => (others => '0'),
        err_capt_err_pos         => (others => '0'),
        err_capt_err_erp         => '0',
        curr_txtb_index          => (others => '0'),

        store_metadata_f         => store_metadata_f,
        store_data_f             => store_data_f,
        store_data_word          => store_data_word,
        rec_valid_f              => rec_valid_f,
        rec_abort_f              => rec_abort_f,

        sof_pulse                => sof_pulse,
        timestamp                => timestamp,

        mr_mode_rxbam            => mr_mode_rxbam,
        mr_command_cdo           => mr_command_cdo,
        mr_command_crxpe         => mr_command_crxpe,
        mr_command_rrb           => mr_command_rrb,
        mr_command_rxrpmv        => mr_command_rxrpmv,
        mr_rx_data_read          => mr_rx_data_read,
        mr_rx_settings_rtsop     => mr_rx_settings_rtsop,
        mr_settings_pchke        => mr_settings_pchke,
        mr_mode_erfm             => '0',

        -- Actually loaded data for reading
        rxb_port_b_data_out     => rxb_port_b_data_out,

        -- Memory testability
        mr_tst_control_tmaena   => '0',
        mr_tst_control_twrstb   => '0',
        mr_tst_dest_tst_addr    => (others => '0'),
        mr_tst_dest_tst_mtgt    => (others => '0'),
        mr_tst_wdata_tst_wdata  => (others => '0'),
        mr_tst_rdata_tst_rdata  => open,

        rx_full                  => rx_full,
        rx_empty                 => rx_empty,
        rx_frame_count           => rx_frame_count,
        rx_mem_free              => rx_mem_free,
        rx_read_pointer          => rx_read_pointer,
        rx_write_pointer         => rx_write_pointer,
        rx_data_overrun          => rx_data_overrun
    );


    ----------------------------------------------------------------------------
    -- Clock and timestamp generation
    ----------------------------------------------------------------------------
    p_clock_gen : process
    begin
        clk_sys       <= '1';
        wait for 5 ns;
        clk_sys       <= '0';
        wait for 5 ns;
    end process;

    p_timestamp_gen : process
        variable ts_lo    : natural := 0;
        variable tmp      : natural := 0;
        variable ts_hi    : natural := 0;
    begin
        loop
            -- falling edge, because on rising edge, the value must stay stable
            -- even after `wait for 0 ns`
            wait until falling_edge(clk_sys);
            tmp := ts_lo + 1;
            if tmp < ts_lo then
                ts_hi := ts_hi + 1;
            end if;
            ts_lo := tmp;
            timestamp <= std_logic_vector(  to_unsigned(ts_hi, 32)
                                          & to_unsigned(ts_lo, 32));
        end loop;
    end process;

    -- Overall amount of errors is sum of errors from all processes
    error_ctr   <=  stim_errs + read_errs + status_errs + cons_errs;

	-- Common input memory is not filled totally so that one iteration
	-- of test won't take too long!
    in_mem_full <= true when in_pointer + C_RX_BUFF_SIZE + 1 > 300 else
                   false;

    out_mem_full <= true when out_pointer + C_RX_BUFF_SIZE + 1 > 300 else
                    false;

    ----------------------------------------------------------------------------
    -- Stimuli generator - Main test process
    ----------------------------------------------------------------------------
    p_stim_gen : process
        -- Size of generated frame in 32 bit words
        variable gen_size     : natural := 0;
        variable enough_space : boolean := true;
        variable was_inserted : boolean := false;
    begin
        test_runner_setup(runner, runner_cfg);
        info_m("Restarting RX Buffer test!");
        wait for 5 ns;
        res_n <= '1';

        apply_rand_seed(seed);

        info_m("Restarted RX Bufrer test");

        ------------------------------------------------------------------------
        -- Main loop of the test
        ------------------------------------------------------------------------
        info_m("Starting RX buffer main loop");

        while (loop_ctr < iterations)
        loop

            --------------------------------------------------------------------
            -- Change setting for timestamp options (store timestamp
            --  at beginning or end of frame)
            --------------------------------------------------------------------
            if (mr_rx_settings_rtsop = RTS_BEG) then
                mr_rx_settings_rtsop <= RTS_END;
            else
                mr_rx_settings_rtsop <= RTS_BEG;
            end if;

            --------------------------------------------------------------------
            -- Start generating the frames on Input as long as there is enough
            -- space available in the common memory.
            --------------------------------------------------------------------
            while (not in_mem_full) loop
                -- Now buffer has for sure space. Frame is inserted into the
                -- RX Buffer, Model and stored also into common memory
                insert_frame_to_RX_Buffer(clk_sys, rec_ident,
                    rec_dlc, rec_frame_type, rec_ident_type, rec_brs,
                    rec_esi, rec_lbpf, rec_ivld, rec_is_rtr, sof_pulse, store_metadata_f, store_data_f,
                    store_data_word, rec_abort_f, rec_valid_f, mr_rx_settings_rtsop,
                    mr_command_cdo, in_mem, in_pointer, timestamp);
            end loop;

            -- Now input memory is full
            -- We need to wait for Data reader to read all frames into common
            -- memory from rx buffer. Then it checks data
            -- consistency and next iteration can start
            wait until iteration_done = true;

            -- Now common input memory is erased
            in_mem      <= (OTHERS => (OTHERS => '0'));
            in_pointer  <= 0;
            loop_ctr    <= loop_ctr + 1;

            wait for 10 ns;
        end loop;

        -- Finish test succesfully -> Failure will abort immediately
        test_runner_cleanup(runner);
        std.env.finish;
    end process;


    ----------------------------------------------------------------------------
    -- Data reader
    ----------------------------------------------------------------------------
    p_data_reader : process
        variable sanity_check   : boolean  :=  true;
        variable sanity_counter : natural  :=  0;
    begin
        -- Offset in time only in first clock cycle
        if (loop_ctr = 0) then
            wait for 5 ns;
        end if;

        ------------------------------------------------------------------------
        -- Read frames as long as Output memory is not filled. Wait random time
        -- in between, to allow for data overrun to occur!
        ------------------------------------------------------------------------
        while (out_mem_full = false) loop
            if (rx_empty = '0') then
                read_frame(rxb_port_b_data_out, mr_rx_data_read, clk_sys, out_mem,
                           in_mem, out_pointer);
                wait_rand_cycles(clk_sys, 200, 250);
            end if;
            wait until rising_edge(clk_sys);
        end loop;

        -- Now output memory is full. We need to wait for Status block to
        -- compare the data consistency
        wait until iteration_done = true;

        sanity_check    :=  true;
        sanity_counter  :=  0;

        -- Erase common memories
        out_mem         <= (OTHERS => (OTHERS => '0'));
        out_pointer     <= 0;

        wait for 10 ns;
    end process;


    ----------------------------------------------------------------------------
    -- Data consistency checker
    ----------------------------------------------------------------------------
    p_cons_check : process
        variable cons_res : boolean  := false;
        variable clk_time : time     := 10 ns;
    begin

        iteration_done <= false;

        -- Wait until data we inserted into input memory and read back by
        -- data reader.
        wait until (in_mem_full = true) and (out_mem_full = true);

        wait for 3000 ns;

        -- Now compare the data
        cons_res := false;
        compare_data(in_mem, out_mem, cons_res);

        check_m(cons_res, "Data consistency check !");

        -- Now we can tell to the other circuits that one iteration is over
        iteration_done <= true;
        wait for 20 ns;
    end process;


    ---------------------------------------------------------------------------
    -- Spawn watchdog
    ---------------------------------------------------------------------------
    process
    begin
        wait for time'value(timeout);
        report "Timeout reached!" severity failure;
    end process;


end architecture;