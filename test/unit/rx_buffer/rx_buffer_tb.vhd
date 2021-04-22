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

library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer.all;
use ctu_can_fd_rtl.can_constants.all;
use ctu_can_fd_rtl.can_components.all;
use ctu_can_fd_rtl.can_types.all;
use ctu_can_fd_rtl.cmn_lib.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.reduce_lib.all;
use ctu_can_fd_rtl.can_config.all;
use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

library ctu_can_fd_tb_unit;
use ctu_can_fd_tb_unit.can_unit_test_pkg.all;
use ctu_can_fd_tb_unit.random_unit_pkg.all;

library vunit_lib;
context vunit_lib.vunit_context;

architecture rx_buffer_unit_test of CAN_test is

    -- System clock and reset
    signal clk_sys                  :    std_logic := '0';
    signal res_n                    :    std_logic := '0';

    -- Metadata and idntifier
    signal rec_ident             :    std_logic_vector(28 downto 0) :=
                                            (OTHERS => '0');
    signal rec_dlc               :    std_logic_vector(3 downto 0) :=
                                            (OTHERS => '0');
    signal rec_ident_type        :    std_logic := '0';
    signal rec_frame_type        :    std_logic := '0';
    signal rec_is_rtr               :    std_logic := '0';
    signal rec_brs                  :    std_logic := '0';
    signal rec_esi                  :    std_logic := '0';

    -- Control signals from CAN Core
    signal store_metadata_f           :    std_logic := '0';
    signal store_data_f               :    std_logic := '0';
    signal store_data_word          :    std_logic_vector(31 downto 0) :=
                                            (OTHERS => '0');
    signal rec_valid_f        :    std_logic := '0';
    signal rec_abort_f                :    std_logic := '0';
    signal sof_pulse                :    std_logic := '0';

    signal timestamp                :    std_logic_vector(63 downto 0) :=
                                            (OTHERS => '0');

    -- Control and status signals to/from SW
    signal drv_bus                  :    std_logic_vector(1023 downto 0) :=
                                            (OTHERS => '0');

    signal rx_buf_size              :    std_logic_vector(12 downto 0);
    signal rx_full                  :    std_logic;
    signal rx_empty                 :    std_logic;
    signal rx_frame_count           :    std_logic_vector(10 downto 0);
    signal rx_mem_free              :    std_logic_vector(12 downto 0);
    signal rx_read_pointer          :    std_logic_vector(11 downto 0);
    signal rx_write_pointer         :    std_logic_vector(11 downto 0);
    signal rx_data_overrun          :    std_logic;

    signal rx_read_buff             :    std_logic_vector(31 downto 0);

    -- Driving bus aliases
    signal drv_rtsopt               :    std_logic   := RTS_END;
    signal drv_read_start           :    std_logic   := '0';
    signal drv_clr_ovr              :    std_logic   := '0';


    ----------------------------------------------------------------------------
    -- Test specific signals
    ----------------------------------------------------------------------------

    signal iteration_done           :    boolean     := false;
    signal in_mem_full              :    boolean     := false;
    signal out_mem_full             :    boolean     := false;

    -- Error counters
    signal stim_errs                :    natural     := 0;
    signal read_errs                :    natural     := 0;
    signal status_errs              :    natural     := 0;
    signal cons_errs                :    natural     := 0;

    -- Dummy signals
    signal exit_imm_d               :    boolean     := false;
    signal exit_imm_d_2             :    boolean     := false;
    signal exit_imm_d_3             :    boolean     := false;

    -- Additional random counter
    signal rand_ctr_3               :    natural range 0 to RAND_POOL_SIZE := 0;

    ----------------------------------------------------------------------------
    -- Memory declarations for memories where data are read out
    ----------------------------------------------------------------------------
    type eval_mem_test is array (0 to 1023) of
        std_logic_vector(31 downto 0);

    signal in_mem                   :    eval_mem_test :=
                                         (OTHERS => (OTHERS => '0'));

    signal out_mem                  :    eval_mem_test :=
                                         (OTHERS => (OTHERS => '0'));

    signal in_pointer               :    natural := 0;
    signal out_pointer              :    natural := 0;
    signal mod_pointer              :    natural := 0;

    constant C_RX_BUFF_SIZE              :    natural := 32;
    
    signal ts_preset        : std_logic_vector(2 downto 1) := "00";
    signal ts_preset_val    : std_logic_vector(63 downto 0) := (OTHERS => '0');


    ----------------------------------------------------------------------------
    -- Insert frame to test memory
    ----------------------------------------------------------------------------
    procedure insert_frame_test_mem(
        constant frame              :in     SW_CAN_frame_type;
        signal   memory             :inout  eval_mem_test;
        signal   in_pointer         :inout  natural
    )is
        variable rwcnt_vect         :       std_logic_vector(4 downto 0);
        variable length             :       natural;
        variable hw_id              :       std_logic_vector(28 downto 0);
    begin
        -- FRAME_FORMAT_W
        rwcnt_vect           := std_logic_vector(to_unsigned(frame.rwcnt, 5));
        memory(in_pointer)   <= "0000000000000000" & rwcnt_vect &
                              frame.esi &
                              frame.brs &
                              '1' &
                              frame.frame_format &
                              frame.ident_type &
                              frame.rtr &
                              '0' &
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
            decode_dlc(frame.dlc, length);
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
        signal   rand_ctr             :inout  natural range 0 to RAND_POOL_SIZE;
        signal   rec_abort_f            :out    std_logic;
        signal   clk_sys              :in     std_logic;
        variable abort_present        :out    boolean;
        constant chances              :in     real;
        signal   log_level            :in     log_lvl_type
    )is
        variable rand_val             :       std_logic;
    begin
        rand_logic_v(rand_ctr, rand_val, chances);
        abort_present := false;

        if (rand_val = '1') then
            rec_abort_f  <= '1';
            wait until rising_edge(clk_sys);
            info("Data storing was aborted!");

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
        signal   rand_ctr           :inout  natural range 0 to RAND_POOL_SIZE;
        signal   clk_sys            :in     std_logic;

        -- Received Metadata and identifier
        signal   rec_ident       :out    std_logic_vector(28 downto 0);
        signal   rec_dlc         :out    std_logic_vector(3 downto 0);
        signal   rec_frame_type     :out    std_logic;
        signal   rec_ident_type     :out    std_logic;
        signal   rec_brs            :out    std_logic;
        signal   rec_esi            :out    std_logic;
        signal   rec_rtr            :out    std_logic;

        -- Storing protocol between RX Buffer and CAN Core
        signal   sof_pulse          :out    std_logic;
        signal   store_metadata_f     :out    std_logic;
        signal   store_data_f         :out    std_logic;
        signal   store_data_word    :out    std_logic_vector(31 downto 0);
        signal   rec_abort_f          :out    std_logic;
        signal   rec_valid_f  :out    std_logic;

        signal   drv_rtsopt         :in     std_logic;
        signal   drv_clr_ovr        :inout  std_logic;

        signal   memory             :inout  eval_mem_test;
        signal   in_pointer         :inout  natural;
        signal   timestamp          :in     std_logic_vector(63 downto 0);
        signal   log_level          :in     log_lvl_type
   )is
        variable CAN_frame          :       SW_CAN_frame_type;
        variable stored_ts          :       std_logic_vector(63 downto 0);
        variable rand_val           :       natural;
        variable abort_present      :       boolean := false;
        variable id_out             :       std_logic_vector(28 downto 0);
    begin

        CAN_generate_frame(rand_ctr, CAN_frame);
        stored_ts := (OTHERS => '0');

        ------------------------------------------------------------------------
        -- Initiate frame storing by clearing possible overrun from before.
        -- It might have happened that Overrun was generated at the same time
        -- as there was intent abort. In that case, the frame was aborted,
        -- overrun was not cleared and stayed till next frame. Storing of
        -- next frame then evaluated overrun as present and did not store the
        -- frame to input memory!
        ------------------------------------------------------------------------
        drv_clr_ovr <= '1';
        wait until rising_edge(clk_sys);
        drv_clr_ovr <= '0';
        wait for 1 ns;

        -- Check that overrun was cleared
        check(rx_data_overrun = '0', "Overrun not cleared!");

        ------------------------------------------------------------------------
        -- Initiate Frame by SOF pulse and store timestamp!
        ------------------------------------------------------------------------
        sof_pulse           <= '1';
        if (drv_rtsopt = RTS_BEG) then
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
        wait_rand_cycles(rand_ctr, clk_sys, 10, 50);

        generate_random_abort(rand_ctr, rec_abort_f, clk_sys, abort_present, 0.1,
                              log_level);

        if (abort_present) then
            wait until rising_edge(clk_sys);
            wait until rising_edge(clk_sys);
            return;
        end if;

        wait_rand_cycles(rand_ctr, clk_sys, 10, 50);

        -- Put metadata on input of RX Buffer!
        id_sw_to_hw(CAN_frame.identifier, CAN_frame.ident_type, id_out);
        rec_ident       <= id_out;
        rec_dlc         <= CAN_frame.dlc;
        rec_frame_type     <= CAN_frame.frame_format;
        rec_ident_type     <= CAN_frame.ident_type;
        rec_brs            <= CAN_frame.brs;
        rec_esi            <= CAN_frame.esi;
        rec_rtr            <= CAN_frame.rtr;

        info("Storing metadata");
        wait until rising_edge(clk_sys);

        -- Send signal to store metadata
        store_metadata_f     <= '1';
        wait until rising_edge(clk_sys);
        store_metadata_f     <= '0';
        wait until rising_edge(clk_sys);

        ------------------------------------------------------------------------
        -- Store data words
        ------------------------------------------------------------------------
        if (CAN_frame.data_length > 0) then
            for i in 0 to ((CAN_frame.data_length - 1) / 4) loop

                -- Wait random time between store of individual data bytes!
                wait_rand_cycles(rand_ctr, clk_sys, 10, 50);

                -- Send signal to store data
                store_data_word <= CAN_frame.data((i * 4) + 3) &
                                   CAN_frame.data((i * 4) + 2) &
                                   CAN_frame.data((i * 4) + 1) &
                                   CAN_frame.data((i * 4));

                store_data_f      <= '1';
                info("Storing data word");
                wait until rising_edge(clk_sys);
                store_data_f      <= '0';
                wait until rising_edge(clk_sys);

                generate_random_abort(rand_ctr, rec_abort_f, clk_sys, abort_present,
                                      0.05, log_level);
                if (abort_present) then
                    wait until rising_edge(clk_sys);
                    wait until rising_edge(clk_sys);
                    return;
                end if;
            end loop;
        end if;

        wait_rand_cycles(rand_ctr, clk_sys, 30, 100);

        ------------------------------------------------------------------------
        -- If we got here, no abort was generated, thus frame was stored OK!
        -- We commit frame to the buffer and store it to test memories!
        ------------------------------------------------------------------------
        rec_valid_f <= '1';
        info("Frame valid!");
        wait until rising_edge(clk_sys);

        ------------------------------------------------------------------------
        -- Timestamp must be marked, if we are interested in END OF Frame
        -- Timestamp!
        ------------------------------------------------------------------------
        if (drv_rtsopt = RTS_END) then
            CAN_frame.timestamp  := timestamp;
        else
            CAN_frame.timestamp  := stored_ts;
        end if;
        rec_valid_f <= '0';

        ------------------------------------------------------------------------
        -- Check that during whole storing of this frame data overrun did not
        -- occur!
        ------------------------------------------------------------------------
        if (rx_data_overrun = '1') then
            info("Data overrun appeared!");

        ------------------------------------------------------------------------
        -- If overrun did not happend, insert frame to input test memory!
        ------------------------------------------------------------------------
        else
            insert_frame_test_mem(CAN_frame, memory, in_pointer);
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
        signal drv_read_start  :inout std_logic;
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
            drv_read_start        <= '1';
            out_mem(out_pointer)  <= buff_out;

            -------------------------------------------------------------------
            -- Check that word is exactly matching the word in in_mem at the
            -- same position!
            -------------------------------------------------------------------
            info("Buffer output: " & to_hstring(buff_out));
            info("Model output: " & to_hstring(in_mem(out_pointer)));
            info("Word nr. :" & integer'image(i));
            check(buff_out = in_mem(out_pointer),
                  "Buffer FUCKED UP, index: " & integer'image(out_pointer));

            out_pointer           <= out_pointer + 1;
            wait until rising_edge(clk_sys);
            drv_read_start        <= '0';
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
                cons_res := false;
            end if;
        end loop;
    end procedure;

  for rx_Buffer_inst : rx_buffer use entity ctu_can_fd_rtl.rx_buffer(rtl);

begin

    ----------------------------------------------------------------------------
    -- Buffer component
    ----------------------------------------------------------------------------
    rx_buffer_inst : rx_buffer
    generic map(
        G_RESET_POLARITY       => '0',
        G_RX_BUFF_SIZE         => C_RX_BUFF_SIZE
    )
    port map(
        clk_sys                  => clk_sys,
        res_n                    => res_n,
        rec_ident                => rec_ident,
        rec_dlc                  => rec_dlc,
        rec_ident_type           => rec_ident_type,
        rec_frame_type           => rec_frame_type,
        rec_is_rtr               => rec_is_rtr,
        rec_brs                  => rec_brs,
        rec_esi                  => rec_esi,
        store_metadata_f         => store_metadata_f,
        store_data_f             => store_data_f,
        store_data_word          => store_data_word,
        rec_valid_f              => rec_valid_f,
        rec_abort_f              => rec_abort_f,
        sof_pulse                => sof_pulse,
        timestamp                => timestamp,
        drv_bus                  => drv_bus,
        rx_buf_size              => rx_buf_size,
        rx_full                  => rx_full,
        rx_empty                 => rx_empty,
        rx_frame_count           => rx_frame_count,
        rx_mem_free              => rx_mem_free,
        rx_read_pointer          => rx_read_pointer,
        rx_write_pointer         => rx_write_pointer,
        rx_data_overrun          => rx_data_overrun,
        rx_read_buff             => rx_read_buff
    );


    ----------------------------------------------------------------------------
    -- Clock and timestamp generation
    ----------------------------------------------------------------------------
    clock_gen_proc(period => f100_Mhz, duty => 50, epsilon_ppm => 0,
                   out_clk => clk_sys);
    timestamp_gen_proc(clk_sys, timestamp, ts_preset(1), ts_preset_val);

    -- Overall amount of errors is sum of errors from all processes
    error_ctr   <=  stim_errs + read_errs + status_errs + cons_errs;

	-- Common input memory is not filled totally so that one iteration
	-- of test won't take too long!
    in_mem_full <= true when in_pointer + C_RX_BUFF_SIZE + 1 > 300 else
                   false;

    out_mem_full <= true when out_pointer + C_RX_BUFF_SIZE + 1 > 300 else
                 false;

    drv_bus(DRV_READ_START_INDEX)   <= drv_read_start;
    drv_bus(DRV_RTSOPT_INDEX)       <= drv_rtsopt;
    drv_bus(DRV_CLR_OVR_INDEX)      <= drv_clr_ovr;


    ----------------------------------------------------------------------------
    -- Stimuli generator - Main test process
    ----------------------------------------------------------------------------
    stim_gen : process
        -- Size of generated frame in 32 bit words
        variable gen_size     : natural := 0;
        variable enough_space : boolean := true;
        variable was_inserted : boolean := false;
    begin
        info("Restarting RX Buffer test!");
        wait for 5 ns;
        reset_test(res_n, status, run, stim_errs);
        apply_rand_seed(seed, 0, rand_ctr);
        info("Restarted RX Bufrer test");
        print_test_info(iterations, log_level, error_beh, error_tol);

        ------------------------------------------------------------------------
        -- Main loop of the test
        ------------------------------------------------------------------------
        info("Starting RX buffer main loop");

        while (loop_ctr < iterations or exit_imm)
        loop

            --------------------------------------------------------------------
            -- Change setting for timestamp options (store timestamp
            --  at beginning or end of frame)
            --------------------------------------------------------------------
            if (drv_rtsopt = RTS_BEG) then
                drv_rtsopt <= RTS_END;
            else
                drv_rtsopt <= RTS_BEG;
            end if;

            --------------------------------------------------------------------
            -- Start generating the frames on Input as long as there is enough
            -- space available in the common memory.
            --------------------------------------------------------------------
            while (in_mem_full = false) loop
                -- Now buffer has for sure space. Frame is inserted into the
                -- RX Buffer, Model and stored also into common memory
                insert_frame_to_RX_Buffer(rand_ctr, clk_sys, rec_ident,
                    rec_dlc, rec_frame_type, rec_ident_type, rec_brs,
                    rec_esi, rec_is_rtr, sof_pulse, store_metadata_f, store_data_f,
                    store_data_word, rec_abort_f, rec_valid_f, drv_rtsopt,
                    drv_clr_ovr, in_mem, in_pointer, timestamp, log_level);
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

        -- This is the main process loop so we evaluate test here
        evaluate_test(error_tol, error_ctr, status);
    end process;


    ----------------------------------------------------------------------------
    -- Data reader
    ----------------------------------------------------------------------------
    data_reader : process
        variable sanity_check   : boolean  :=  true;
        variable sanity_counter : natural  :=  0;
    begin
        -- Offset in time only in first clock cycle
        if (loop_ctr = 0) then
            wait for 5 ns;
        end if;

        if (res_n = C_RESET_POLARITY) then
            apply_rand_seed(seed, 1, rand_ctr_3);
        end if;

        ------------------------------------------------------------------------
        -- Read frames as long as Output memory is not filled. Wait random time
        -- in between, to allow for data overrun to occur!
        ------------------------------------------------------------------------
        while (out_mem_full = false) loop
            if (rx_empty = '0') then
                read_frame(rx_read_buff, drv_read_start, clk_sys, out_mem,
                           in_mem, out_pointer);
                wait_rand_cycles(rand_ctr_3, clk_sys, 200, 250);
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
    cons_check : process
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

        check(cons_res, "Data consistency check failed !");

        -- Now we can tell to the other circuits that one iteration is over
        iteration_done <= true;
        wait for 20 ns;
    end process;

    errors <= error_ctr;

end architecture;