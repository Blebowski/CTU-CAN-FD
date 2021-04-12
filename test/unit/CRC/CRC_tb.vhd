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
--  Unit test for the CRC circuit.
--
-- @Verifies:
--  @1. Calculation of CRC15, CRC17 and CRC21 according to 11898-1:2015.
--  @2. Calculation of CRC15, CRC17 and CRC21 according to CAN FD specification
--      1.0 (non-iso version with different intialization vectors).
--
-- @Test sequence:
--  @1. Generate random bit sequence between 10 and 620 bits long. Generate
--      random setting of ISO/NON-ISO.
--  @2. Calculate expected CRC results for this bit sequence (SW model).
--  @3. Apply generated bit sequence to inputs of CRC module.
--  @4. Compare results of SW model and CRC value on DUT outputs.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    4.6.2016   Created file
--------------------------------------------------------------------------------


--------------------------------------------------------------------------------
-- Test implementation
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

architecture CRC_unit_test of CAN_test is

    signal data_in     :   std_logic := '0'; --Serial data input
    signal clk_sys     :   std_logic := '0'; --System clock input
    signal trig        :   std_logic := '0'; --Trigger to sample the input value
    signal res_n       :   std_logic := '0'; --Asynchronous reset
    signal enable      :   std_logic := '0';
    signal drv_bus     :   std_logic_vector(1023 downto 0) := (OTHERS => '0');

    signal crc_15       :   std_logic_vector(14 downto 0) := (OTHERS => '0');
    signal crc_17       :   std_logic_vector(16 downto 0) := (OTHERS => '0');
    signal crc_21       :   std_logic_vector(20 downto 0) := (OTHERS => '0');

    --TX trigger for crc, corresponds to trigger
    -- signal delayed by one from Protocol control trigger
    signal tx_trig     :   std_logic := '0';
    signal rx_trig     :   std_logic := '0';
    signal rnd_ctr_tr  :   natural range 0 to RAND_POOL_SIZE := 0;
    signal drv_fd_type :   std_logic := '0';

    -- Load CRC Initialization vector
    signal load_init_vect     :   std_logic := '0';

    ----------------------------------------------------------------------------
    -- Generate bit sequence for CRC calculation
    ----------------------------------------------------------------------------
    procedure gen_bit_sequence(
        signal   rand_ctr    :inout   natural range 0 to RAND_POOL_SIZE;
        constant min_length  :in      natural range 2  to 10;
        constant max_length  :in      natural range 11 to 640;
        variable bit_seq     :out     std_logic_vector(660 downto 0);
        variable act_length  :inout   natural
    ) is
        variable rand_nr       :real;
    begin
        rand_real_v(rand_ctr, rand_nr);
        act_length := min_length +
                      integer(real(max_length - min_length) * rand_nr);
        rand_logic_vect_v(rand_ctr, bit_seq, 0.5);

        ------------------------------------------------------------------------
        -- From length until the end of vector we should have zeroes
        -- Here we left enough space at the end of the vector for
        -- right alignment by zeroes from right!
        ------------------------------------------------------------------------
        for i in 0 to 660 - act_length loop
            bit_seq(i) := '0';
        end loop;
    end procedure;


    ----------------------------------------------------------------------------
    -- Calculate the CRC in behavioral way
    ----------------------------------------------------------------------------
    procedure calc_crc(
        variable bit_seq    :in     std_logic_vector(660 downto 0);
        variable act_length :in     natural;
        constant crc_15_pol :in     std_logic_vector(15 downto 0);
        constant crc_17_pol :in     std_logic_vector(19 downto 0);
        constant crc_21_pol :in     std_logic_vector(23 downto 0);
        signal   drv_fd_type:in     std_logic;
        variable crc_15_res :inout  std_logic_vector(14 downto 0);
        variable crc_17_res :inout  std_logic_vector(16 downto 0);
        variable crc_21_res :inout  std_logic_vector(20 downto 0)
    ) is
        variable crc_15_next  :       std_logic;
        variable crc_17_next  :       std_logic;
        variable crc_21_next  :       std_logic;
        variable iterator     :       natural;
    begin

        --Set up the initial polynomials
        crc_15_res := (OTHERS => '0');
        crc_17_res := (OTHERS => '0');
        crc_21_res := (OTHERS => '0');
        if (drv_fd_type = ISO_FD) then
           crc_17_res(16) := '1';
           crc_21_res(20) := '1';
        end if;


        ------------------------------------------------------------------------
        -- Calculate CRC in behavioral way use the same algorithm as in the
        -- actual circuit! I was too lazy to debug the algoryhtm with polynomial
        -- division. FOllowing algorythm is the same as in CAN FD spec.
        ------------------------------------------------------------------------

        iterator := 0;
        while (iterator < act_length) loop

            -- Check whether we go XORing
            crc_15_next := crc_15_res(14) xor bit_seq(660 - iterator);
            crc_17_next := crc_17_res(16) xor bit_seq(660 - iterator);
            crc_21_next := crc_21_res(20) xor bit_seq(660 - iterator);

            -- Shift left by one
            crc_15_res := crc_15_res(13 downto 0) & '0';
            crc_17_res := crc_17_res(15 downto 0) & '0';
            crc_21_res := crc_21_res(19 downto 0) & '0';

            -- Now do the XOR if we have to
            if (crc_15_next = '1') then
                crc_15_res := crc_15_res xor crc_15_pol(14 downto 0);
            end if;

            if (crc_17_next = '1') then
                crc_17_res := crc_17_res xor crc_17_pol(16 downto 0);
            end if;

            if (crc_21_next = '1') then
                crc_21_res := crc_21_res xor crc_21_pol(20 downto 0);
            end if;

            iterator := iterator + 1;
        end loop;
    end procedure;


    ----------------------------------------------------------------------------
    -- Puts the CRC on the input of the
    ----------------------------------------------------------------------------
    procedure put_to_dut(
        signal    sync         :in     std_logic;
        signal    sample       :in     std_logic;
        signal    enable       :out    std_logic;
        signal    data_in      :out    std_logic;
        signal    clk_sys      :in     std_logic;
        variable  bit_seq      :in     std_logic_vector(660 downto 0);
        variable  act_length   :in     natural
    ) is
        variable iterator: natural := 0;
    begin
        wait until rising_edge(sync);
        enable <= '1';
        wait until falling_edge(clk_sys);


        iterator := 0;
        while (iterator < act_length) loop
            log("Putting bit nr " & integer'image(iterator));
            data_in <= bit_seq(660 - iterator);
            wait until rising_edge(sample);
            iterator := iterator + 1;
            wait until rising_edge(sync);
        end loop;

        wait until falling_edge(clk_sys);
        enable <= '0';
    end procedure;


    ----------------------------------------------------------------------------
    -- Compare results
    ----------------------------------------------------------------------------
    procedure compare_results(
        signal    crc_15_dut  :in     std_logic_vector(14 downto 0);
        signal    crc_17_dut  :in     std_logic_vector(16 downto 0);
        signal    crc_21_dut  :in     std_logic_vector(20 downto 0);
        variable  crc_15_mod  :in     std_logic_vector(14 downto 0);
        variable  crc_17_mod  :in     std_logic_vector(16 downto 0);
        variable  crc_21_mod  :in     std_logic_vector(20 downto 0);
        variable  c15_mism    :out    boolean;
        variable  c17_mism    :out    boolean;
        variable  c21_mism    :out    boolean
    )is
    begin
		c15_mism := false;

        if (crc_15_dut /= crc_15_mod) then
			-- LCOV_EXCL_START
            c15_mism := true;
			-- LCOV_EXCL_STOP
        end if;

        if (crc_17_dut /= crc_17_mod) then
			-- LCOV_EXCL_START
            c17_mism := true;
			-- LCOV_EXCL_STOP
        end if;

        if (crc_21_dut /= crc_21_mod) then
			-- LCOV_EXCL_START
            c21_mism := true;
			-- LCOV_EXCL_STOP
        end if;
  end procedure;

begin

    ----------------------------------------------------------------------------
    -- DUT
    ----------------------------------------------------------------------------
    can_crc_comp : can_crc
    generic map(
        G_RESET_POLARITY    => '0',
        G_CRC15_POL         => C_CRC15_POL,
        G_CRC17_POL         => C_CRC17_POL,
        G_CRC21_POL         => C_CRC21_POL
    )
    port map(
        -- System clock and Asynchronous Reset
        clk_sys          => clk_sys,
        res_n            => res_n,

        -- Memory registers interface
        drv_bus          => drv_bus,

        -- Data inputs for CRC calculation. Use the same data input for each
        -- CRC calculation as workaround!
        data_tx_wbs      => data_in,
        data_tx_nbs      => data_in,
        data_rx_wbs      => data_in,
        data_rx_nbs      => data_in,

        -- Trigger signals to process the data on each CRC input.
        trig_tx_wbs      => trig,
        trig_tx_nbs      => trig,
        trig_rx_wbs      => trig,
        trig_rx_nbs      => trig,

        -- Control signals. So far don't use speculative calculation
        crc_enable       => enable,
        crc_spec_enable  => '0',
        crc_calc_from_rx => '0',
        load_init_vect   => load_init_vect,

        -- CRC Outputs
        crc_15           => crc_15,
        crc_17           => crc_17,
        crc_21           => crc_21
    );

    -- Flexible data rate type
    drv_bus(DRV_FD_TYPE_INDEX) <= drv_fd_type;


    ----------------------------------------------------------------------------
    -- Clock generation
    ----------------------------------------------------------------------------
    clock_gen_proc(period => f100_Mhz, duty => 50, epsilon_ppm => 0,
                   out_clk => clk_sys);


    ----------------------------------------------------------------------------
    -- Sampling signals generation
    ----------------------------------------------------------------------------
    sample_gen : process
        variable min_diff : natural := 0;
    begin
        if (res_n = C_RESET_POLARITY) then
            apply_rand_seed(seed, 1, rnd_ctr_tr);
        end if;
        generate_simple_trig(rnd_ctr_tr, tx_trig, rx_trig, clk_sys, min_diff);
    end process;

    -- Propagate the trigger to the CRC circuit...
    trig <= rx_trig;


    ----------------------------------------------------------------------------
    -- Main test process
    ----------------------------------------------------------------------------
    test_proc : process
        variable bit_seq    : std_logic_vector(660 downto 0);
        variable gen_length : natural;
        variable crc_15_mod : std_logic_vector(14 downto 0) := (OTHERS => '0');
        variable crc_17_mod : std_logic_vector(16 downto 0) := (OTHERS => '0');
        variable crc_21_mod : std_logic_vector(20 downto 0) := (OTHERS => '0');
        variable c15_mism   : boolean:=false;
        variable c17_mism   : boolean:=false;
        variable c21_mism   : boolean:=false;
    begin
        info("Restarting CRC test!");
        wait for 5 ns;
        reset_test(res_n, status, run, error_ctr);
        apply_rand_seed(seed, 0, rand_ctr);
        info("Restarted CRC test");
        print_test_info(iterations, log_level, error_beh, error_tol);

        while (loop_ctr < iterations or exit_imm)
        loop
            info("Starting loop nr " & integer'image(loop_ctr));

            --Generate random ISO, non ISO
            rand_logic_s(rand_ctr, drv_fd_type, 0.5);

            --Generate bit sequence
            info("Generating random bit sequence");
            gen_bit_sequence(rand_ctr, 10, 620, bit_seq, gen_length);

            info("Calculating software CRC");
            calc_crc(bit_seq, gen_length, C_CRC15_POL, C_CRC17_POL, C_CRC21_POL,
                   drv_fd_type, crc_15_mod, crc_17_mod, crc_21_mod);
                   
            info("Preloading initialization vector!");
            load_init_vect <= '1';
            wait until rising_edge(clk_sys);
            wait until rising_edge(clk_sys);
            load_init_vect <= '0';
            wait until rising_edge(clk_sys);

            info("Putting bit sequence to DUT");
            put_to_dut(tx_trig, rx_trig, enable, data_in, clk_sys, bit_seq,
                       gen_length);

            info("Comparing SW CRC and DUT output");
            compare_results(crc_15, crc_17, crc_21, crc_15_mod, crc_17_mod,
                            crc_21_mod, c15_mism, c17_mism, c21_mism);

            check_false(c15_mism, "Mismatch in CRC 15");
            check_false(c17_mism, "Mismatch in CRC 17");
            check_false(c21_mism, "Mismatch in CRC 21");

            loop_ctr <= loop_ctr + 1;
        end loop;

        evaluate_test(error_tol, error_ctr, status);
    end process;

end architecture;
