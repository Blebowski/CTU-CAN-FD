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
--  Unit test for the RX Buffer circuit.
--
--  Following test instantiates RX Buffer. Stimuli generator generates input
--  frames as CAN_Core would do. Then it checks whether frame was stored into
--  the buffer! Another process reads the data as user would do by memory access.
--  Both, data written into the buffer, and data read from the buffer are stored
--  into test memories (in_mem,out_mem). When test memory is full content of
--  both memories is compared! When mismatch occurs test fails. Each time memory
--  is filled test moves to the next iteration.
--
--------------------------------------------------------------------------------
-- Revision History:
--    1.6.2016   Created file
--   22.6.2016   Updated testbench to cover also the modified functionality of
--               RX Buffer. Now ESI bit is also stored and compared. Also RTR
--               frame of CAN normal frame does not store any data words into
--               the buffer.
--
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
USE ieee.math_real.ALL;
use work.CANconstants.all;
use work.CANcomponents.ALL;
USE work.CANtestLib.All;
USE work.randomLib.All;
use work.ID_transfer.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

architecture rx_buf_unit_test of CAN_test is

    -- System clock and reset
    signal clk_sys                  :    std_logic := '0';
    signal res_n                    :    std_logic := '0';

    -- Metadata and idntifier
    signal rec_ident_in             :    std_logic_vector(28 downto 0) :=
                                            (OTHERS => '0');
    signal rec_dlc_in               :    std_logic_vector(3 downto 0) :=
                                            (OTHERS => '0');
    signal rec_ident_type_in        :    std_logic := '0';
    signal rec_frame_type_in        :    std_logic := '0';
    signal rec_is_rtr               :    std_logic := '0';
    signal rec_brs                  :    std_logic := '0';
    signal rec_esi                  :    std_logic := '0';

    -- Control signals from CAN Core
    signal store_metadata           :    std_logic := '0';
    signal store_data               :    std_logic := '0';
    signal store_data_word          :    std_logic_vector(31 downto 0) :=
                                            (OTHERS => '0');
    signal rec_message_valid        :    std_logic := '0';
    signal rec_abort                :    std_logic := '0';
    signal sof_pulse                :    std_logic := '0';

    signal timestamp                :    std_logic_vector(63 downto 0) :=
                                            (OTHERS => '0');

    -- Control and status signals to/from SW
    signal drv_bus                  :    std_logic_vector(1023 downto 0) :=
                                            (OTHERS => '0');

    signal rx_buf_size              :    std_logic_vector(12 downto 0);
    signal rx_full                  :    std_logic;
    signal rx_empty                 :    std_logic;
    signal rx_message_count         :    std_logic_vector(10 downto 0);
    signal rx_mem_free              :    std_logic_vector(12 downto 0);
    signal rx_read_pointer_pos      :    std_logic_vector(11 downto 0);
    signal rx_write_pointer_pos     :    std_logic_vector(11 downto 0);
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

    constant buff_size              :    natural := 32;


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
        signal   rec_abort            :out    std_logic;
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
            rec_abort  <= '1';
            wait until rising_edge(clk_sys);
            log("Data storing was aborted!", info_l, log_level);

            rec_abort  <= '0';
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
        signal   rec_ident_in       :out    std_logic_vector(28 downto 0);
        signal   rec_dlc_in         :out    std_logic_vector(3 downto 0);
        signal   rec_frame_type     :out    std_logic;
        signal   rec_ident_type     :out    std_logic;
        signal   rec_brs            :out    std_logic;
        signal   rec_esi            :out    std_logic;
        signal   rec_rtr            :out    std_logic;

        -- Storing protocol between RX Buffer and CAN Core
        signal   sof_pulse          :out    std_logic;
        signal   store_metadata     :out    std_logic;
        signal   store_data         :out    std_logic;
        signal   store_data_word    :out    std_logic_vector(31 downto 0);
        signal   rec_abort          :out    std_logic;
        signal   rec_message_valid  :out    std_logic;

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
        if (rx_data_overrun = '1') then
            -- LCOV_EXCL_START
            log("Overrun not cleared!", error_l, log_level);
            -- LCOV_EXCL_STOP
        end if;

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

        generate_random_abort(rand_ctr, rec_abort, clk_sys, abort_present, 0.1,
                              log_level);

        if (abort_present) then
            wait until rising_edge(clk_sys);
            wait until rising_edge(clk_sys);
            return;
        end if;

        wait_rand_cycles(rand_ctr, clk_sys, 10, 50);

        -- Put metadata on input of RX Buffer!
        id_sw_to_hw(CAN_frame.identifier, CAN_frame.ident_type, id_out);
        rec_ident_in       <= id_out;
        rec_dlc_in         <= CAN_frame.dlc;
        rec_frame_type     <= CAN_frame.frame_format;
        rec_ident_type     <= CAN_frame.ident_type;
        rec_brs            <= CAN_frame.brs;
        rec_esi            <= CAN_frame.esi;
        rec_rtr            <= CAN_frame.rtr;

        log("Storing metadata", info_l, log_level);
        wait until rising_edge(clk_sys);

        -- Send signal to store metadata
        store_metadata     <= '1';
        wait until rising_edge(clk_sys);
        store_metadata     <= '0';
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

                store_data      <= '1';
                log("Storing data word", info_l, log_level);
                wait until rising_edge(clk_sys);
                store_data      <= '0';
                wait until rising_edge(clk_sys);

                generate_random_abort(rand_ctr, rec_abort, clk_sys, abort_present,
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
        rec_message_valid <= '1';
        log("Frame valid!", info_l, log_level);
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
        rec_message_valid <= '0';

        ------------------------------------------------------------------------
        -- Check that during whole storing of this frame data overrun did not
        -- occur!
        ------------------------------------------------------------------------
        if (rx_data_overrun = '1') then
            log("Data overrun appeared!", info_l, log_level);

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

            -- Check that word is exactly matching the word in in_mem at the
            -- same position
            if (buff_out /= in_mem(out_pointer)) then
                -- LCOV_EXCL_START
                log("Buffer FUCKED UP, index: " & integer'image(out_pointer),
                    error_l, log_level);
                -- LCOV_EXCL_STOP
            end if;

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

  for rx_Buffer_comp : rxBuffer use entity work.rxBuffer(rtl);

begin

    ----------------------------------------------------------------------------
    -- Buffer component
    ----------------------------------------------------------------------------
    rx_Buffer_comp : rxBuffer
    generic map(
        buff_size                => buff_size
    )
    port map(
        clk_sys                  => clk_sys,
        res_n                    => res_n,
        rec_ident_in             => rec_ident_in,
        rec_dlc_in               => rec_dlc_in,
        rec_ident_type_in        => rec_ident_type_in,
        rec_frame_type_in        => rec_frame_type_in,
        rec_is_rtr               => rec_is_rtr,
        rec_brs                  => rec_brs,
        rec_esi                  => rec_esi,
        store_metadata           => store_metadata,
        store_data               => store_data,
        store_data_word          => store_data_word,
        rec_message_valid        => rec_message_valid,
        rec_abort                => rec_abort,
        sof_pulse                => sof_pulse,
        timestamp                => timestamp,
        drv_bus                  => drv_bus,
        rx_buf_size              => rx_buf_size,
        rx_full                  => rx_full,
        rx_empty                 => rx_empty,
        rx_message_count         => rx_message_count,
        rx_mem_free              => rx_mem_free,
        rx_read_pointer_pos      => rx_read_pointer_pos,
        rx_write_pointer_pos     => rx_write_pointer_pos,
        rx_data_overrun          => rx_data_overrun,
        rx_read_buff             => rx_read_buff
    );


    ----------------------------------------------------------------------------
    -- Clock and timestamp generation
    ----------------------------------------------------------------------------
    clock_gen_proc(period => f100_Mhz, duty => 50, epsilon_ppm => 0,
                   out_clk => clk_sys);
    timestamp_gen_proc(clk_sys, timestamp);

    -- Overall amount of errors is sum of errors from all processes
    error_ctr   <=  stim_errs + read_errs + status_errs + cons_errs;

	-- Common input memory is not filled totally so that one iteration
	-- of test won't take too long!
    in_mem_full <= true when in_pointer + buff_size + 1 > 300 else
                   false;

    out_mem_full <= true when out_pointer + buff_size + 1 > 300 else
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
        log("Restarting RX Buffer test!", info_l, log_level);
        wait for 5 ns;
        reset_test(res_n, status, run, stim_errs);
        apply_rand_seed(seed, 0, rand_ctr);
        log("Restarted RX Bufrer test", info_l, log_level);
        print_test_info(iterations, log_level, error_beh, error_tol);

        ------------------------------------------------------------------------
        -- Main loop of the test
        ------------------------------------------------------------------------
        log("Starting RX buffer main loop", info_l, log_level);

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
                insert_frame_to_RX_Buffer(rand_ctr, clk_sys, rec_ident_in,
                    rec_dlc_in, rec_frame_type_in, rec_ident_type_in, rec_brs,
                    rec_esi, rec_is_rtr, sof_pulse, store_metadata, store_data,
                    store_data_word, rec_abort, rec_message_valid, drv_rtsopt,
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

        if (res_n = ACT_RESET) then
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

        if (cons_res = false) then
            -- LCOV_EXCL_START
            process_error(cons_errs, error_beh, exit_imm_d_3);
            log("Data consistency check failed !", error_l, log_level);
            -- LCOV_EXCL_STOP
        end if;

        -- Now we can tell to the other circuits that one iteration is over
        iteration_done <= true;
        wait for 20 ns;
    end process;

    errors <= error_ctr;

end architecture;
