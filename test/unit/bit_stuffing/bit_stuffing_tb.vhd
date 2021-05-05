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
--  Unit test for bit stuffing and bit destuffing circuits.
--
-- @Verifies:
--  @1. Functionality of Bit Stuffing and Bit Destuffing modules.
--  @1. Insertion of stuff bits after 5 bits of equal consecutive value.
--  @2. Discarding of stuffing bits after 5 bits of equal consecutive value.
--  @3. Fixed bit stuffing and Fixed bit destuffing. Stuff bit is inserted
--      after 4 bits regardless of bit value.
--  @4. Stuff Error detection if 6-th equal consecutive bit is received.
--  @5. Recursive behaviour of bit stuffing (stuff bit counts as first bit in
--      the next sequence of equal consecutive bits).
--  @6. Insertion of Stuff bit upon change from non-fixed to fixed bit stuffing
--      (modeling insertion of stuff bit before stuff count field of CAN FD
--      frame).
--
-- @Test sequence:
--  @1. Generate random bit sequence. Randomize whether at some point of the
--      sequence, switch to fixed bit stuffing will occur.
--  @2. Calculate expected sequence after bit stuffing (SW model).
--  @3. Apply sequence on inputs of Bit Stuffing. Compare output of bit
--      stuffing with expected bit coming out of SW model.
--  @4. Stuffed sequence is destuffed with bit destuffing and compared with
--      original generated sequence. Random stuff error is inserted. A check
--      if bit error is detected is executed.
-- 
-- @Notes:
--  Unit test makes use of bit stuffing/destuffing symmetry. Random data are
--  generated on input of bit stuffing. Data are stuffed and compared with
--  reference SW model. Then data are destuff_enable and output is compared with
--  generated data (before bit stuffing). Additional error might be generated
--  on input of bit de-stuffing forcing n + 1 consecutive bits of equal value.
--
--  SW model emulates behaviour of CAN Frame:
--      1. Bit stuffing is enabled (non fixed) as in original CAN Frame.
--      2. Some bits are stuffed.
--      3. With random chance bit stuffing is switched to fixed stuffing as
--         if in real CAN Frame.
--      4. Extra stuff bit is inserted as special stuff bit in begining of
--         CAN FD CRC.
--      5. Next bits are stuffed with fixed stuffing.
--
--   Output of SW model is always compared with output of bit stuffing DUT.
--   Note that upon stuff error, step is finished immediately (as if error
--   frame transmission started). Note that SW model covers recursive behaviour
--   of bit stuffing.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--
--    13.6.2016   Created file
--    23.5.2018   Reimplemented Bit stuffing testbench. Generated bitsream
--                before bit stuffing. Added SW model. Calculated expected stuff
--                sequence. Comparing TX/RX mismatch, stuff output expected/
--                actual, stuff error/expected actual.
--                Stuff error randomly generated upon inserted stuff bit.
--                Verification of "special" fixed stuff bit in transition
--                between "non fixed" and "fixed" stuffing added.
--                CAN frame emulated. First stuffing is enabled non fixed, then
--                transits to fixed stuffing as during CAN FD Frame.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;
use ieee.std_logic_textio.all;
use STD.textio.all;

library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_components_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.common_blocks_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;
use ctu_can_fd_rtl.can_config_pkg.all;
use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

library ctu_can_fd_tb_unit;
use ctu_can_fd_tb_unit.can_unit_test_pkg.all;
use ctu_can_fd_tb_unit.random_unit_pkg.all;

library vunit_lib;
context vunit_lib.vunit_context;

architecture bit_stuffing_unit_test of CAN_test is

    ----------------------------------------------------------------------------
    -- System clock and async. reset
    ----------------------------------------------------------------------------
    signal clk_sys              :   std_logic := '0';
    signal res_n                :   std_logic := '0';

    ----------------------------------------------------------------------------
    -- Triggering signals
    ----------------------------------------------------------------------------

    -- Trigger with intention to transmitt the data.
    signal tx_trig_intent       :   std_logic := '0';

    -- Trigger on which data are actually transmitted! New data are not
    -- transmitted if bit was stuffed and "data_halt" occurred!
    signal tx_trig_ack          :   std_logic := '0';

    -- Bit stuffing trigger (one clock delayed from tx_trig)
    signal bst_trigger              :   std_logic := '0';

    -- Bit destuffin trigger
    signal bds_trigger              :   std_logic := '0';

    -- Intent to receive data
    signal rx_trig_intent       :   std_logic := '0';

    -- Trigger on which data are really sampled! New data are not received if
    -- data are destuff_enable ("destuff_enable" is active)!
    signal rx_trig_ack          :   std_logic := '0';

    ----------------------------------------------------------------------------
    -- Datapath
    ----------------------------------------------------------------------------
    -- Transmitted data (before bit stuffing)
    signal tx_data              :   std_logic := '1';

    -- Stuffed data (after bit stuffing)
    signal stuffed_data         :   std_logic := '1';

    -- Data for bit destuffing (stuffed_data and err_data)
    signal joined_data          :   std_logic := '0';

    -- Error data should be inserted
    signal err_data             :   std_logic := '0';

    -- Received data (after bit-destuffing)
    signal rx_data              :   std_logic := '1';

    ----------------------------------------------------------------------------
    -- Control signals
    ----------------------------------------------------------------------------
    signal stuff_enable            :   std_logic := '0';
    signal destuff_enable            :   std_logic := '0';
    signal fixed_stuff          :   std_logic := '0';  -- Common for both
    signal stuff_length            :   std_logic_vector(2 downto 0) := "100";
    signal destuff_length            :   std_logic_vector(2 downto 0) := "100";

    ----------------------------------------------------------------------------
    -- Status signals
    ----------------------------------------------------------------------------
    signal data_halt            :   std_logic := '0';
    signal destuffed            :   std_logic := '0';
    signal stuff_err          :   std_logic := '0';
    signal bst_ctr               :   std_logic_vector(2 downto 0);
    signal dst_ctr               :   std_logic_vector(2 downto 0);


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Testbench signals
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    ----------------------------------------------------------------------------
    -- Bit stuffing settings. Includes everything necessary for one iteration
    -- of Bit stuffing unit test.
    ----------------------------------------------------------------------------
    type bs_test_settings_type is record

        -- Number of bits to wait in "NON FIXED" mode after enabling.
        bc_non_fixed            :   natural;

        -- Length of "NON FIXED" stuffing rule.
        stuff_length_non_fixed  :   std_logic_vector(2 downto 0);

        -- Whether change to "FIXED" bit stuffing should be performed.
        change_to_fixed         :   boolean;

        -- Number of bits to wait in FIXED mode after enabling.
        bc_fixed                :   natural;

        -- Number of to wait in "FIXED" mode after change from NON FIXED to
        -- FIXED bit stuffing.
        stuff_length_fixed      :   std_logic_vector(2 downto 0);

        -- TX Data sequence before bit stuffing and RX after bit destuffing.
        -- (Make sure that largest possible CAN FD frame fits)
        tx_data_seq             :   std_logic_vector(799 downto 0);
        rx_data_seq             :   std_logic_vector(799 downto 0);

        -- Stuffed data sequence which should come out of bit stuffing after
        -- "tx_data_seq" was applied on input of the circuit.
        stuffed_data_seq        :   std_logic_vector(799 downto 0);

        -- Bit at matching position in "stuffed_data_seq" is stuffed
        stuffed_bits_mark       :   std_logic_vector(799 downto 0);

        -- Stuffing counters
        stuff_counter_fixed     :   natural;
        stuff_counter_non_fixed :   natural;
    end record;

    ----------------------------------------------------------------------------
    -- Error counters
    ----------------------------------------------------------------------------
    signal  stuf_e_err_ctr      :   natural := 0;
    signal  stat_err_ctr        :   natural := 0;

    ----------------------------------------------------------------------------
    -- Other signals
    ----------------------------------------------------------------------------
    signal rand_set_ctr         :   natural range 0 to RAND_POOL_SIZE := 0;
    signal rand_st_err_ctr      :   natural range 0 to RAND_POOL_SIZE := 0;
    signal exit_imm_1           :   boolean := false;


    -- Signal that no trigger is active and bit stuffing settings can be
    -- changed
    signal no_trigger           :   boolean := false;

    -- Bit stuffing step settings
    signal set                  :   bs_test_settings_type :=
                                    (0, "000", false, 0, "000", (OTHERS => '1'),
                                     (OTHERS => '1'), (OTHERS => '1'),
                                     (OTHERS => '1'), 0, 0);

    signal nbs_index            :   natural := 0;
    signal wbs_index            :   natural := 0;

    -- Expected value on output of Bit stuffing circuit (as calculated by
    --  SW model)
    signal exp_stuffed          :   std_logic := '1';
    signal should_be_stuffed    :   std_logic := '0';

    ----------------------------------------------------------------------------
    -- Calculates expected sequence on output of bit stuffing circuit
    -- from input sequence based on bit stuffing settings.
    -- Includes: non - fixed stuffing, extra stuff bit + fixed stuffing!
    ----------------------------------------------------------------------------
    procedure calc_sw_bit_stuf_sequence(
        signal set              : inout bs_test_settings_type
    ) is
        variable i              :       integer := 0;
        variable in_ptr         :       integer := 0;
        variable out_ptr        :       integer := 0;
        variable stuff_ctr      :       integer := 1;
        variable st_length      :       integer := 0;
        variable prev_bit       :       std_logic := '1';
    begin

        -- Check settings validity for circuits as well as for
        -- "stuffed_data_seq" length
        if (to_integer(unsigned(set.stuff_length_fixed))     < 3 or
            to_integer(unsigned(set.stuff_length_non_fixed)) < 3)
        then
            -- LCOV_EXCL_START
            report "Invalid bit stuffing settings!" severity failure;
            -- LCOV_EXCL_STOP
        end if;

        -- Set previous bit to actual input of Bit stuffing. This corresponds
        -- to previously propagated bit value when bit stuffing was not yet
        -- enabled! 
        prev_bit := tx_data;

        --------------------------------------
        -- Calculate non-fixed stuffing
        --------------------------------------
        st_length := to_integer(unsigned(set.stuff_length_non_fixed));
        while (i < (set.bc_non_fixed - 1)) loop

            -- Insert stuff bit
            if (stuff_ctr = st_length) then
                set.stuffed_data_seq(out_ptr) <= not set.tx_data_seq(in_ptr - 1);
                set.stuffed_bits_mark(out_ptr) <= '1';
                set.stuff_counter_non_fixed <= set.stuff_counter_non_fixed + 1;
                out_ptr := out_ptr + 1;
                stuff_ctr := 1;
                prev_bit := not set.tx_data_seq(in_ptr - 1);
            end if;

            -- Check if next bit is equal to previous one
            if (set.tx_data_seq(in_ptr) = prev_bit) then
                stuff_ctr := stuff_ctr + 1;
            else
                stuff_ctr := 1;
                prev_bit := set.tx_data_seq(in_ptr);
            end if;

            -- Insert bit from input sequence
            set.stuffed_data_seq(out_ptr) <= set.tx_data_seq(in_ptr);
            in_ptr  := in_ptr + 1;
            out_ptr := out_ptr + 1;
            i       := i + 1;
        end loop;


        --------------------------------------
        -- Calculate fixed bit stuffing
        --------------------------------------
        if (set.change_to_fixed) then
            stuff_ctr := 0;

            -- Insert one extra stuff bit as in beginning of CAN FD CRC!
            set.stuffed_data_seq(out_ptr)  <= not set.tx_data_seq(in_ptr - 1);
            set.stuffed_bits_mark(out_ptr) <= '1';
            set.stuff_counter_fixed <= set.stuff_counter_fixed + 1;
            out_ptr := out_ptr + 1;

            st_length := to_integer(unsigned(set.stuff_length_fixed));
            i := 0;
            while (i < (set.bc_fixed - 1)) loop

                -- Insert stuff bit
                if (stuff_ctr = st_length) then
                    stuff_ctr := 1;
                    set.stuffed_data_seq(out_ptr) <=
                        not set.tx_data_seq(in_ptr - 1);
                    set.stuffed_bits_mark(out_ptr) <= '1';
                    set.stuff_counter_fixed <=
                        set.stuff_counter_fixed + 1;
                    out_ptr := out_ptr + 1;
                    --wait for 0 ns;

                -- Stuff counter is incremented regardless of bit values since
                -- stuffing is fixed!
                else
                    stuff_ctr := stuff_ctr + 1;
                end if;

                -- Insert bit from input sequence
                set.stuffed_data_seq(out_ptr) <= set.tx_data_seq(in_ptr);
                in_ptr  := in_ptr + 1;
                out_ptr := out_ptr + 1;
                i       := i + 1;
            end loop;
        end if;

    end procedure;



    procedure generate_bs_settings(
        signal rand_ctr         : inout natural range 0 to RAND_POOL_SIZE;
        signal set              : inout bs_test_settings_type
    )is
        variable tmp            :       natural;
        variable tmp_vect       :       std_logic_vector(799 downto 0);
    begin
        ------------------------------------------------------------------------
        -- Generate random bit stuffing rule lengths. This is not necessary
        -- for CAN protocol, but why not to be a litte more general? Stuffing
        -- length constrained to lengths from 3 to 7.
        ------------------------------------------------------------------------
        rand_int_v(rand_ctr, 3, tmp);
        set.stuff_length_non_fixed <=
            std_logic_vector(to_unsigned(tmp + 3, 3));
        wait for 0 ns;

        rand_int_v(rand_ctr, 3, tmp);
        set.stuff_length_fixed <=
            std_logic_vector(to_unsigned(tmp + 3, 3));

        ------------------------------------------------------------------------
        -- Generate duration of random intervals for non fixed and for
        -- fixed parts of bit stuffing. Generate if fixed bit stuffing should
        -- even be used!
        ------------------------------------------------------------------------
        rand_int_s(rand_ctr, 550, set.bc_non_fixed, true);
        set.bc_non_fixed <= set.bc_non_fixed + 20;

        rand_int_s(rand_ctr, 20, set.bc_fixed, true);
        set.bc_fixed <= set.bc_fixed + 15;

        rand_int_v(rand_ctr, 10, tmp);
        if (tmp > 5) then
            set.change_to_fixed <= true;
        else
            set.change_to_fixed <= false;
        end if;
        wait for 0 ns;

        ------------------------------------------------------------------------
        -- Generate random bit sequences in TX Data, use only data up to
        -- generated length
        ------------------------------------------------------------------------
        rand_logic_vect_cons_v(rand_ctr, tmp_vect, 0.85);
        set.tx_data_seq(set.bc_non_fixed - 1 downto 0) <=
            tmp_vect(set.bc_non_fixed - 1 downto 0);

        if (set.change_to_fixed) then
            rand_logic_vect_cons_v(rand_ctr, tmp_vect, 0.8);
            set.tx_data_seq(set.bc_fixed + set.bc_non_fixed - 1 downto
                            set.bc_non_fixed) <=
                tmp_vect(set.bc_fixed - 1 downto 0);
        end if;

        wait for 0 ns;

        -- Calculate expected stuff sequence!
        calc_sw_bit_stuf_sequence(set);

    end procedure;



    ----------------------------------------------------------------------------
    -- Executes test step. Sequence is as following:
    --  1. Generate random settings for bit stuffing, bit destuffing. Generate
    --     random bit streams, calculates expected sequence after bit stuffing!
    --  2. Enable bit stuffing with "fixed_stuff = 0" as if CAN frame started.
    --  3. Transmit TX data , receive RX data. Sample data after bit stuffing.
    --     In each step, evaluate if data after bit stuffing match expected data
    --     in "set.stuffed_data". Throw an error if not.
    --  4. After "set.bc_non_fixed" change to fixed bit stuffing if
    --     "set.change_fixed = true".
    --  5. TX, RX and sample stuffed data for rest of the sequence with fixed
    --     bit stuffing. Evaluate if TX data = RX data and if stuffed sequence
    --     matches the expected sequence.
    --  6. During points 3 and 5, if "err_data" is active, finish the step.
    --     "gen_st_error" process is looking for detection of stuff error.
    --     Stuff error means error frame on CAN Bus, no sense in comparing the
    --     data further!
    ----------------------------------------------------------------------------
    procedure exec_bs_test_step(
        signal rand_ctr     : inout natural range 0 to RAND_POOL_SIZE;
        signal set          : inout bs_test_settings_type;

        -- Triggering signals
        signal tx_trig_ack  : in    std_logic;
        signal rx_trig_ack  : in    std_logic;
        signal bst_trigger      : in    std_logic;
        signal bds_trigger      : in    std_logic;
        signal no_trigger   : in    boolean;

        -- Data signals
        signal tx_data      : inout std_logic;
        signal rx_data      : in    std_logic;
        signal stuffed_data : in    std_logic;
        signal err_data     : in    std_logic;

        -- Bit stuffing and destuffing configuration
        signal stuff_enable    : inout std_logic;
        signal destuff_enable    : inout std_logic;
        signal fixed_stuff  : inout std_logic;
        signal stuff_length    : inout std_logic_vector(2 downto 0);
        signal destuff_length    : inout std_logic_vector(2 downto 0);

        -- Pointers to input data
        signal nbs_index    : out   natural;
        signal wbs_index    : out   natural;

        -- Error behaviour stuff
        signal err_ctr      : inout natural;
        signal log_lvl      : in    log_lvl_type;
        signal error_beh    : in    err_beh_type;
        signal exit_imm     : inout boolean
    ) is
        variable nbs_ptr    :       natural := 0;
        variable wbs_ptr    :       natural := 0;
        variable msg1       :       line;
        variable msg2       :       line;
        variable msg3       :       line;
        variable msg4       :       line;
    begin
        wait until no_trigger;
        wait until rising_edge(clk_sys);

        set <= (0, "000", false, 0, "000", (OTHERS => '0'), (OTHERS => '0'),
                (OTHERS => '0'), (OTHERS => '0'), 0, 0);

        -- Generate step settings
        generate_bs_settings(rand_ctr, set);
        wait for 0 ns;

        if (log_lvl = info_l) then
            -- LCOV_EXCL_START
            info("TX Data NON fixed: ");
            write(msg1, set.tx_data_seq(set.bc_non_fixed - 1 downto 0));
            writeline(output, msg1);

            info("TX Data fixed: ");
            write(msg2, set.tx_data_seq(set.bc_non_fixed + set.bc_fixed - 1
                                       downto set.bc_non_fixed));
            writeline(output, msg2);
            -- LCOV_EXCL_STOP
        end if;

        if (log_lvl = info_l) then
            -- LCOV_EXCL_START
            info("Stuffed data NON fixed: ");
            write(msg3, set.stuffed_data_seq(
                        set.stuff_counter_non_fixed + set.bc_non_fixed - 1
                        downto 0));
            writeline(output, msg3);

            info("Stuffed data fixed: ");
            write(msg4, set.stuffed_data_seq(
                        set.stuff_counter_non_fixed + set.bc_non_fixed +
                        set.stuff_counter_fixed + set.bc_fixed - 1 downto
                        set.stuff_counter_non_fixed + set.bc_non_fixed));
            writeline(output, msg4);
            -- LCOV_EXCL_STOP
        end if;

        info("Non-fixed length: " & integer'image(set.bc_non_fixed));
        if (set.change_to_fixed) then
            info("Change to fixed should occur!");
            info("Fixed length: " & integer'image(set.bc_fixed));
        end if;

        -- Enable Bit stuffing and destuffing. Set no fixed stuffing.
        stuff_enable    <= '1';
        destuff_enable    <= '1';
        fixed_stuff  <= '0';

        -- Set non fixed length
        stuff_length    <= set.stuff_length_non_fixed;
        destuff_length    <= set.stuff_length_non_fixed;

        -- Transmitt, receive and sample stuffed data!
        while true loop
            wait until rising_edge(clk_sys);

            -- Stuff error was forced -> stop sampling.
            if (err_data = '1') then
                exit;
            end if;

            -- Transmitting
            if (tx_trig_ack = '1') then
                tx_data <= set.tx_data_seq(nbs_ptr);
            end if;

            -- Receiving
            if (rx_trig_ack = '1') then
                wait for 1 ns;
                check(rx_data = tx_data, "TX, RX data mismatch");
                nbs_ptr := nbs_ptr + 1;
                nbs_index <= nbs_ptr;
            end if;

            -- Report if recursive bit stuffing occurred. This is just to
            -- verify that circuit works recursively well!
            -- (enough to detect one polarity to verify proper operation)
            if (set.tx_data_seq(nbs_ptr) = '0' and
                set.tx_data_seq(nbs_ptr + 1) = '0' and
                set.tx_data_seq(nbs_ptr + 2) = '0' and
                set.tx_data_seq(nbs_ptr + 3) = '0' and
                set.tx_data_seq(nbs_ptr + 4) = '0' and
                set.tx_data_seq(nbs_ptr + 5) = '1' and
                set.tx_data_seq(nbs_ptr + 6) = '1' and
                set.tx_data_seq(nbs_ptr + 7) = '1' and
                set.tx_data_seq(nbs_ptr + 8) = '1' and
                bst_trigger = '1')
            then
                info("Recursive stuff ocurred!");
            end if;

            -- Checking Bit stuffed sequence and that bit was stuffed when
            -- supposed to!
            if (bst_trigger = '1') then
                wait for 1 ns;
                check(stuffed_data = set.stuffed_data_seq(wbs_ptr),
                      "Stuffed data mismatch");
                check(set.stuffed_bits_mark(wbs_ptr) = data_halt,
                      "Stuff bit not inserted!");
                wbs_ptr := wbs_ptr + 1;
                wbs_index <= wbs_ptr;
            end if;

            -- Check for change to fixed stuffing
            if (no_trigger and (nbs_ptr = set.bc_non_fixed - 1) and
                fixed_stuff = '0')
            then

                -- Quit if fixed stuffing is not set for this step.
                if (set.change_to_fixed = false) then
                    info("End of data sequence.");
                    exit;
                end if;

                info("Change to fixed stuffing.");

                fixed_stuff  <= '1';
                stuff_length    <= set.stuff_length_fixed;
                destuff_length    <= set.stuff_length_fixed;
            end if;

            -- Check if we are in the end
            if (no_trigger and (nbs_ptr = set.bc_non_fixed + set.bc_fixed - 2)
                and fixed_stuff = '1')
            then
                info("End of data sequence.");
                exit;
            end if;

        end loop;

        -- Finish test step by turning bit stuffing off!
        wait for 50 ns;
        stuff_enable    <= '0';
        destuff_enable    <= '0';
        wbs_ptr      := 0;
        nbs_ptr      := 0;
        nbs_index    <= 0;
        wbs_index    <= 0;
        wait for 500 ns;
        wait until no_trigger;

    end procedure;


begin

    bit_stuffing_comp : bit_stuffing
    port map(
        clk_sys            =>  clk_sys,
        res_n              =>  res_n,
        bst_trigger        =>  bst_trigger,
        stuff_enable       =>  stuff_enable,
        data_in            =>  tx_data,
        fixed_stuff        =>  fixed_stuff,
        tx_frame_no_sof    =>  '0',
        data_halt          =>  data_halt,
        stuff_length       =>  stuff_length,
        bst_ctr            =>  bst_ctr,
        data_out           =>  stuffed_data
    );


    bit_destuffing_comp : bit_destuffing
    port map(
        clk_sys            =>  clk_sys,
        res_n              =>  res_n,
        data_in            =>  joined_data,
        bds_trigger        =>  bds_trigger,
        stuff_err          =>  stuff_err,
        data_out           =>  rx_data,
        destuff_enable     =>  destuff_enable,
        fixed_stuff        =>  fixed_stuff,
        destuff_length     =>  destuff_length,
        destuffed          =>  destuffed,
        dst_ctr            =>  dst_ctr
    );


    ----------------------------------------------------------------------------
    -- Clock generation
    ----------------------------------------------------------------------------
    clock_gen_proc(period => f100_Mhz, duty => 50, epsilon_ppm => 0,
                   out_clk => clk_sys);


    ----------------------------------------------------------------------------
    -- Trigger generation
    ----------------------------------------------------------------------------
    stuff_gen : process
    begin
        wait until rising_edge(clk_sys);
        tx_trig_intent       <= '1';
        wait until rising_edge(clk_sys);
        tx_trig_intent       <= '0';
        bst_trigger              <= '1';
        wait until rising_edge(clk_sys);
        bst_trigger              <= '0';
        bds_trigger              <= '1';
        wait until rising_edge(clk_sys);
        bds_trigger              <= '0';
        rx_trig_intent       <= '1';
        wait until rising_edge(clk_sys);
        rx_trig_intent       <= '0';
        wait until rising_edge(clk_sys);
        wait until rising_edge(clk_sys);
        wait until rising_edge(clk_sys);
        wait until rising_edge(clk_sys);
    end process;


    -- Generation of real TX, RX triggers based on bus throttling by
    -- bit stuffing and destuffing!
    tx_trig_ack <= tx_trig_intent and (not data_halt);
    rx_trig_ack <= rx_trig_intent and (not destuffed);

    no_trigger <= true when (tx_trig_intent = '0' and bst_trigger = '0' and
                             bds_trigger = '0' and rx_trig_intent = '0')
                       else
                  false;

    -- Assigning only to have waveform of expected data and presence of
    -- stuff bit!
    sw_mod_prop_proc : process
    begin
        wait until (rising_edge(clk_sys) and bst_trigger = '1');
        exp_stuffed         <= set.stuffed_data_seq(wbs_index);
        should_be_stuffed   <= set.stuffed_bits_mark(wbs_index);
    end process;
    
    ----------------------------------------------------------------------------
    -- When bit stuff occurs, value of next bit should be oposite of previous
    -- one! Bit destuffing is thus expecting bit of opposite polarity that
    -- should be destuff_enable. If this does not occur, stuff error should be
    -- detected!
    --
    -- With some random chance, if stuff bit occurs force stuff error and
    -- check whether stuff error was fired!
    ----------------------------------------------------------------------------
    gen_st_error : process
        variable tmp        : real := 0.0;
    begin
        
        -- Wait until bit was stuffed
        while (data_halt /= '1') loop
            wait until rising_edge(clk_sys);
        end loop;
            
        -- Generate random stuff error;
        rand_real_v(rand_st_err_ctr, tmp);
        if (tmp < 0.02) then
            err_data <= '1';
            wait for 11 ns; -- One cycle + little bit more

            info("Stuff error inserted");

            -------------------------------------------------------------------
            -- Now stuff error should be fired by bit destuffing, since bit
            -- value was forced to be the same as previous bits! Do this check
            -- only when destuffing is enabled! Due to TB, it can happend that
            -- this if condition will enter right after bit sequence end!
            -- In this case, Stuff error is not fired because bit stuffing is
            -- already disabled (which is OK!)
            -------------------------------------------------------------------
            --if (destuff_enable = '1') then
            --    if (stuff_err /= '1') then
            --        wait for 150 ns;
            --        error("Stuff error not fired");
            --    end if;
                check(stuff_err = '1', "Stuff error not fired!");
            --end if;

            wait until rising_edge(clk_sys);
            err_data <= '0';
        end if;
        
        while (data_halt /= '0') loop
            wait until rising_edge(clk_sys);
        end loop;

    end process;


    error_ctr     <=  stat_err_ctr + stuf_e_err_ctr;


    -- Stuffed data are inverted if error should be forced! Bit destuffing will
    -- receive inversion of stuffed bits (the same as original value), thus
    -- causing error!
    joined_data   <=  stuffed_data when (err_data = '0') else
                      not stuffed_data;

    errors        <= error_ctr;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Main Test process
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    test_proc : process
    begin
        info("Restarting Bit stuffing-destuffing test!");
        wait for 5 ns;
        reset_test(res_n, status, run, stat_err_ctr);
        apply_rand_seed(seed, 0, rand_ctr);
        info("Restarted Bit stuffing-destuffing test");
        print_test_info(iterations, log_level, error_beh, error_tol);

        -------------------------------
        -- Main loop of the test
        -------------------------------
        info("Starting Bit stuffing-destuffing main loop");

        while (loop_ctr < iterations  or  exit_imm or exit_imm_1)
        loop
            info("Starting loop nr " & integer'image(loop_ctr));

            exec_bs_test_step(rand_ctr, set, tx_trig_ack, rx_trig_ack, bst_trigger,
                              bds_trigger, no_trigger, tx_data, rx_data,
                              stuffed_data, err_data, stuff_enable, destuff_enable,
                              fixed_stuff, stuff_length, destuff_length, nbs_index,
                              wbs_index, stat_err_ctr, log_level, error_beh,
                              exit_imm);
            loop_ctr <= loop_ctr + 1;
        end loop;

        evaluate_test(error_tol, error_ctr, status);
    end process;

end architecture;
