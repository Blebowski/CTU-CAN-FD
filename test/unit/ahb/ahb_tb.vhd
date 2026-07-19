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
--    Unit test of ctu_can_fd_top_ahb, performing register read/write via AHB.
--
-- @Verifies:
--  @1. Single (non-back-to-back) read and write transactions to CTU CAN FD via
--      AHB.
--  @2. Back-to-back (pipelined) burst read and write transactions, including
--      bursts over a non-incrementing (scattered) address set.
--  @3. Byte, half-word and word sized accesses (HSIZE / byte-lane decoding).
--  @4. Known-answer register content right after reset (DEVICE_ID, BTR, MODE,
--      STATUS).
--  @5. The dedicated pipeline hazard of ahb_ifc: a read whose address phase
--      immediately follows a write's data phase gets exactly one extra wait
--      state (HREADY deasserted for one cycle).
--
-- @Test sequence:
--  @1. Reset the DUT and check known-answer register values (DEVICE_ID, BTR,
--      MODE, STATUS reset values).
--  @2. Repeat for "iterations" random passes: single word write/read-back to
--      BTR, and a back-to-back burst write/read-back over the Filter A/B/C
--      mask/value registers (consecutive R/W registers).
--  @3. Byte-wise and half-word-wise partial writes to BTR, verifying byte-lane
--      decoding (HSIZE + HADDR(1:0)) and that untouched bytes are preserved.
--  @4. Manually pipelined write followed immediately by a read (address phase
--      of the read overlapping the data phase of the write) to exercise the
--      dedicated read-after-write wait state in ahb_ifc.
--  @5. Back-to-back burst write/read over a scattered (non-incrementing)
--      address set, to verify the interface does not assume/require
--      address auto-increment.
--  @6. Write a known pattern, apply a mid-test hardware reset, and check that
--      the control register is restored to its reset value.
--
-- @Notes:
--  This is a from-scratch AHB master BFM (no VIP agent is used - CTU CAN FD's
--  AHB interface is exercised directly at the signal level, per AMBA2 AHB
--  timing). HREADY is fed back directly from HREADYOUT (single-slave, no
--  interconnect/mux in this TB). Pipeline timing (address phase N, data phase
--  N+1, one extra wait state on read-after-write) was cross-checked against
--  doc/system_architecture/timing_diagrams/ahb_access.json.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    17.7.2026   First implementation
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;
use ieee.std_logic_textio.all;
use STD.textio.all;

library vunit_lib;
context vunit_lib.vunit_context;

-- Common contexts
Library ctu_can_fd_tb;
context ctu_can_fd_tb.ieee_context;
context ctu_can_fd_tb.tb_common_context;
context ctu_can_fd_tb.tb_agents_context;
context ctu_can_fd_tb.rtl_context;

entity ahb_tb is
    generic (
        runner_cfg      : string         := runner_cfg_default;
        iterations      : natural        := 1;
        timeout         : string         := "0 us";
        error_tol       : natural        := 0;
        seed            : natural        := 0
    );
end entity;

architecture test of ahb_tb is

    ----------------------------------------------------------------------------
    -- AHB protocol encodings (AMBA2 AHB)
    ----------------------------------------------------------------------------
    constant C_HTRANS_IDLE     : std_logic_vector(1 downto 0) := "00";
    constant C_HTRANS_BUSY     : std_logic_vector(1 downto 0) := "01";
    constant C_HTRANS_NONSEQ   : std_logic_vector(1 downto 0) := "10";
    constant C_HTRANS_SEQ      : std_logic_vector(1 downto 0) := "11";

    constant C_HBURST_SINGLE   : std_logic_vector(2 downto 0) := "000";
    constant C_HBURST_INCR     : std_logic_vector(2 downto 0) := "001";

    constant C_HSIZE_BYTE      : std_logic_vector(2 downto 0) := "000";
    constant C_HSIZE_HALFWORD  : std_logic_vector(2 downto 0) := "001";
    constant C_HSIZE_WORD      : std_logic_vector(2 downto 0) := "010";

    ----------------------------------------------------------------------------
    -- Known-answer reset values, built from the register map's own per-field
    -- reset constants (rather than hand-computed magic numbers) so the check
    -- tracks the register map if it ever changes.
    ----------------------------------------------------------------------------
    constant C_BTR_RSTVAL : std_logic_vector(31 downto 0) :=
        SJW_RSTVAL & BRP_RSTVAL & PH2_RSTVAL & PH1_RSTVAL & PROP_RSTVAL;

    constant C_MODE_RSTVAL : std_logic_vector(11 downto 0) := (
        RST_IND   => RST_RSTVAL,
        BMM_IND   => BMM_RSTVAL,
        STM_IND   => STM_RSTVAL,
        AFM_IND   => AFM_RSTVAL,
        FDE_IND   => FDE_RSTVAL,
        TTTM_IND  => TTTM_RSTVAL,
        ROM_IND   => ROM_RSTVAL,
        ACF_IND   => ACF_RSTVAL,
        TSTM_IND  => TSTM_RSTVAL,
        RXBAM_IND => RXBAM_RSTVAL,
        TXBBM_IND => TXBBM_RSTVAL,
        SAM_IND   => SAM_RSTVAL
    );

    constant C_STATUS_RSTVAL : std_logic_vector(11 downto 0) := (
        RXNE_IND  => RXNE_RSTVAL,
        DOR_IND   => DOR_RSTVAL,
        TXNF_IND  => TXNF_RSTVAL,
        EFT_IND   => EFT_RSTVAL,
        RXS_IND   => RXS_RSTVAL,
        TXS_IND   => TXS_RSTVAL,
        EWL_IND   => EWL_RSTVAL,
        IDLE_IND  => IDLE_RSTVAL,
        PEXS_IND  => PEXS_RSTVAL,
        RXPE_IND  => RXPE_RSTVAL,
        TXPE_IND  => TXPE_RSTVAL,
        TXDPE_IND => TXDPE_RSTVAL
    );

    ----------------------------------------------------------------------------
    -- Address / data arrays for burst procedures
    ----------------------------------------------------------------------------
    type t_ahb_addr_array is array (natural range <>) of std_logic_vector(31 downto 0);
    type t_ahb_data_array is array (natural range <>) of std_logic_vector(31 downto 0);

    ----------------------------------------------------------------------------
    -- Zero-extend a 12 bit register-map address to the 32 bit AHB address bus
    ----------------------------------------------------------------------------
    function f_reg_addr(constant reg_adr : std_logic_vector(11 downto 0))
        return std_logic_vector is
    begin
        return x"00000" & reg_adr;
    end function;

    ----------------------------------------------------------------------------
    -- AHB signals
    ----------------------------------------------------------------------------
    signal hclk         : std_logic := '0';
    signal hresetn      : std_logic := '0';

    signal haddr        : std_logic_vector(31 downto 0) := (others => '0');
    signal hwdata        : std_logic_vector(31 downto 0) := (others => '0');
    signal hsel          : std_logic := '0';
    signal hwrite         : std_logic := '0';
    signal hsize         : std_logic_vector(2 downto 0) := C_HSIZE_WORD;
    signal hburst        : std_logic_vector(2 downto 0) := C_HBURST_SINGLE;
    signal hprot        : std_logic_vector(3 downto 0) := (others => '0');
    signal htrans        : std_logic_vector(1 downto 0) := C_HTRANS_IDLE;
    signal hmastlock     : std_logic := '0';
    signal hready        : std_logic;
    signal hreadyout     : std_logic;
    signal hresp         : std_logic_vector(1 downto 0);
    signal hrdata        : std_logic_vector(31 downto 0);

begin

    ----------------------------------------------------------------------------
    -- Single-slave, direct-connect testbench: HREADY seen by the master is
    -- simply the DUT's own HREADYOUT (no interconnect/mux involved).
    ----------------------------------------------------------------------------
    hready <= hreadyout;

    ----------------------------------------------------------------------------
    -- DUT
    ----------------------------------------------------------------------------
    i_dut : entity ctu_can_fd_rtl.ctu_can_fd_top_ahb
        generic map (
            G_RX_BUF_SIZE       => 128,
            G_TXT_BUF_COUNT     => 4,
            G_FILT_A_EN         => true,
            G_FILT_B_EN         => true,
            G_FILT_C_EN         => true,
            G_FILT_RANGE_EN     => false,
            G_TEST_REGS_EN      => true
        )
        port map (
            hresetn          => hresetn,
            hclk             => hclk,
            haddr            => haddr,
            hwdata           => hwdata,
            hsel             => hsel,
            hwrite           => hwrite,
            hsize            => hsize,
            hburst           => hburst,
            hprot            => hprot,
            htrans           => htrans,
            hmastlock        => hmastlock,
            hready           => hready,
            hreadyout        => hreadyout,
            hresp            => hresp,
            hrdata           => hrdata,

            rst_n_out        => open,

            can_tx           => open,
            can_rx           => '1',

            timestamp        => (others => '0'),

            scan_mode        => '0',

            int              => open
        );

    hclk <= not hclk after 5 ns;

    ----------------------------------------------------------------------------
    -- Main test process
    ----------------------------------------------------------------------------
    p_main : process
        variable rdata            : std_logic_vector(31 downto 0);
        variable rand_word        : std_logic_vector(31 downto 0);

        -- Filter A/B/C mask/value registers (0x3C - 0x50): consecutive R/W
        -- registers, only bits 28 downto 0 implemented.
        variable filt_addr_arr    : t_ahb_addr_array(0 to 5);
        variable filt_wdata_arr   : t_ahb_data_array(0 to 5);
        variable filt_rdata_arr   : t_ahb_data_array(0 to 5);

        variable scat_addr_arr    : t_ahb_addr_array(0 to 2);
        variable scat_wdata_arr   : t_ahb_data_array(0 to 2);
        variable scat_rdata_arr   : t_ahb_data_array(0 to 2);

        --------------------------------------------------------------------
        -- Single, non-back-to-back AHB write. Address phase for one clock,
        -- then a data phase during which HTRANS is IDLE - the bus returns
        -- to idle before the procedure returns, so consecutive calls never
        -- pipeline with each other.
        --------------------------------------------------------------------
        procedure ahb_write(
            constant addr : std_logic_vector(31 downto 0);
            constant data : std_logic_vector(31 downto 0);
            constant size : std_logic_vector(2 downto 0) := C_HSIZE_WORD
        ) is
        begin
            -- Address phase
            haddr     <= addr;
            hwrite    <= '1';
            hsize     <= size;
            hburst    <= C_HBURST_SINGLE;
            htrans    <= C_HTRANS_NONSEQ;
            hprot     <= (others => '0');
            hmastlock <= '0';
            hsel      <= '1';

            wait until rising_edge(hclk);
            wait for 1 ps;
            while (hready /= '1') loop
                wait until rising_edge(hclk);
                wait for 1 ps;
            end loop;

            -- Data phase - go back to idle since this write is not chained
            -- with any following transfer
            htrans <= C_HTRANS_IDLE;
            hsel   <= '0';
            hwdata <= data;

            wait until rising_edge(hclk);
            wait for 1 ps;
            while (hready /= '1') loop
                wait until rising_edge(hclk);
                wait for 1 ps;
            end loop;

            hwdata <= (others => '0');
            haddr  <= (others => '0');
            hwrite <= '0';
        end procedure;

        --------------------------------------------------------------------
        -- Single, non-back-to-back AHB read. Result is written to "data".
        --------------------------------------------------------------------
        procedure ahb_read(
            constant addr : std_logic_vector(31 downto 0);
            variable data : out std_logic_vector(31 downto 0);
            constant size : std_logic_vector(2 downto 0) := C_HSIZE_WORD
        ) is
        begin
            -- Address phase
            haddr     <= addr;
            hwrite    <= '0';
            hsize     <= size;
            hburst    <= C_HBURST_SINGLE;
            htrans    <= C_HTRANS_NONSEQ;
            hprot     <= (others => '0');
            hmastlock <= '0';
            hsel      <= '1';

            wait until rising_edge(hclk);
            wait for 1 ps;

            -- Data phase - go back to idle; HRDATA becomes valid one clock
            -- after the address phase was accepted
            htrans <= C_HTRANS_IDLE;
            hsel   <= '0';
            haddr <= (others => '0');

            while (hready /= '1') loop
                wait until rising_edge(hclk);
                wait for 1 ps;
            end loop;

            data := hrdata;
            wait for 1 ps;
        end procedure;

        --------------------------------------------------------------------
        -- Back-to-back (pipelined) burst write. addr_arr/data_arr may be an
        -- arbitrary (not necessarily incrementing) set of addresses - each
        -- beat's address phase overlaps the previous beat's data phase, with
        -- no idle cycles in between.
        --------------------------------------------------------------------
        procedure ahb_burst_write(
            constant addr_arr : t_ahb_addr_array;
            constant data_arr : t_ahb_data_array;
            constant size     : std_logic_vector(2 downto 0) := C_HSIZE_WORD
        ) is
            variable n : natural := addr_arr'length;
        begin
            assert (data_arr'length = n)
                report "ahb_burst_write: address/data array length mismatch"
                severity failure;

            for i in 0 to n - 1 loop
                haddr     <= addr_arr(addr_arr'low + i);
                hwrite    <= '1';
                hsize     <= size;
                hburst    <= C_HBURST_INCR;
                hprot     <= (others => '0');
                hmastlock <= '0';
                hsel      <= '1';

                if (i = 0) then
                    htrans <= C_HTRANS_NONSEQ;
                else
                    htrans <= C_HTRANS_SEQ;
                    -- Data phase of the previous beat, concurrent with the
                    -- address phase of this beat
                    hwdata <= data_arr(data_arr'low + i - 1);
                end if;

                wait until rising_edge(hclk);
                wait for 1 ps;
                while (hready /= '1') loop
                    wait until rising_edge(hclk);
                    wait for 1 ps;
                end loop;
            end loop;

            -- Drain: data phase of the last beat, no further address phase
            htrans <= C_HTRANS_IDLE;
            hsel   <= '0';
            hwdata <= data_arr(data_arr'low + n - 1);

            wait until rising_edge(hclk);
            wait for 1 ps;
            while (hready /= '1') loop
                wait until rising_edge(hclk);
                wait for 1 ps;
            end loop;

            hwdata <= (others => '0');
            haddr  <= (others => '0');
            hwrite <= '0';
        end procedure;

        --------------------------------------------------------------------
        -- Back-to-back (pipelined) burst read. Results are written to
        -- data_arr, one entry per address in addr_arr.
        --------------------------------------------------------------------
        procedure ahb_burst_read(
            constant addr_arr : t_ahb_addr_array;
            variable data_arr : out t_ahb_data_array;
            constant size     : std_logic_vector(2 downto 0) := C_HSIZE_WORD
        ) is
            variable n : natural := addr_arr'length;
        begin
            assert (data_arr'length = n)
                report "ahb_burst_read: address/data array length mismatch"
                severity failure;

            hwrite    <= '0';
            hsize     <= size;
            hburst    <= C_HBURST_INCR;
            hprot     <= (others => '0');
            hmastlock <= '0';
            hsel      <= '1';

            for i in 0 to n - 1 loop
                haddr <= addr_arr(addr_arr'low + i);

                if (i = 0) then
                    htrans <= C_HTRANS_NONSEQ;
                else
                    htrans <= C_HTRANS_SEQ;
                end if;

                wait until rising_edge(hclk);
                wait for 1 ps;
                if (i < n - 1) then
                    haddr <= addr_arr(addr_arr'low + i + 1);
                end if;

                while (hready = '0') loop
                    wait until rising_edge(hclk);
                    wait for 1 ps;
                end loop;
                data_arr(i) := hrdata;
            end loop;

            -- Drain: sample data phase of the last beat
            htrans <= C_HTRANS_IDLE;
            hsel   <= '0';
            haddr  <= (others => '0');

        end procedure;

    begin
        test_runner_setup(runner, runner_cfg);
        apply_rand_seed(seed);

        info_m("Restarting AHB test");

        hresetn <= '0';
        wait for 25 ns;
        hresetn <= '1';

        wait until rising_edge(hclk);
        wait until rising_edge(hclk);
        wait until rising_edge(hclk);

        ------------------------------------------------------------------------
        -- Known-answer register checks right after reset
        ------------------------------------------------------------------------
        info_m("Checking known-answer registers after reset");

        ahb_read(f_reg_addr(DEVICE_ID_ADR), rdata);
        check_m(rdata(15 downto 0) = CTU_CAN_FD_ID, "DEVICE_ID mismatch after reset");

        ahb_read(f_reg_addr(BTR_ADR), rdata);
        check_m(rdata = C_BTR_RSTVAL, "BTR reset value mismatch");

        ahb_read(f_reg_addr(MODE_ADR), rdata);
        check_m(rdata(11 downto 0) = C_MODE_RSTVAL, "MODE reset value mismatch");

        ahb_read(f_reg_addr(STATUS_ADR), rdata);
        check_m(rdata(11 downto 0) = C_STATUS_RSTVAL, "STATUS reset value mismatch");

        ------------------------------------------------------------------------
        -- Randomized single access and burst access regression
        ------------------------------------------------------------------------
        for iter in 1 to iterations loop
            info_m("AHB test iteration: " & integer'image(iter));

            -- Single (non-back-to-back) write/read-back to BTR
            rand_logic_vect_v(rand_word, 0.5);
            ahb_write(f_reg_addr(BTR_ADR), rand_word, C_HSIZE_WORD);
            ahb_read(f_reg_addr(BTR_ADR), rdata);
            check_m(rdata = rand_word, "Single BTR write/read-back mismatch, iteration " &
                    integer'image(iter));

            -- Back-to-back burst write/read-back over the Filter A/B/C
            -- mask/value registers (consecutive R/W registers)
            filt_addr_arr(0) := f_reg_addr(FILTER_A_MASK_ADR);
            filt_addr_arr(1) := f_reg_addr(FILTER_A_VAL_ADR);
            filt_addr_arr(2) := f_reg_addr(FILTER_B_MASK_ADR);
            filt_addr_arr(3) := f_reg_addr(FILTER_B_VAL_ADR);
            filt_addr_arr(4) := f_reg_addr(FILTER_C_MASK_ADR);
            filt_addr_arr(5) := f_reg_addr(FILTER_C_VAL_ADR);

            for i in 0 to 5 loop
                rand_logic_vect_v(filt_wdata_arr(i), 0.5);
                filt_wdata_arr(i)(31 downto 29) := "000";
            end loop;

            ahb_burst_write(filt_addr_arr, filt_wdata_arr, C_HSIZE_WORD);
            ahb_burst_read(filt_addr_arr, filt_rdata_arr, C_HSIZE_WORD);

            for i in 0 to 5 loop
                info_m( "Burst WDATA: " & to_hstring(filt_wdata_arr(i)) & "   " &
                        "Burst RDATA: " & to_hstring(filt_rdata_arr(i)));
            end loop;

            for i in 0 to 5 loop
                check_m(filt_rdata_arr(i) = filt_wdata_arr(i),
                        "Filter register burst readback mismatch, word " & integer'image(i) &
                        ", iteration " & integer'image(iter));
            end loop;
        end loop;

        ------------------------------------------------------------------------
        -- Byte / half-word sized accesses (HSIZE + HADDR(1:0) byte-lane
        -- decoding). Value is replicated to every byte lane of HWDATA so the
        -- check does not depend on assuming which lane ahb_ifc will pick -
        -- only the *expected* final word depends on that (sourced directly
        -- from ahb_ifc's own byte-enable decode table).
        ------------------------------------------------------------------------
        info_m("Checking byte / half-word sized accesses");

        ahb_write(std_logic_vector(unsigned(f_reg_addr(BTR_ADR)) + 0), x"11111111", C_HSIZE_BYTE);
        ahb_write(std_logic_vector(unsigned(f_reg_addr(BTR_ADR)) + 1), x"22222222", C_HSIZE_BYTE);
        ahb_write(std_logic_vector(unsigned(f_reg_addr(BTR_ADR)) + 2), x"33333333", C_HSIZE_BYTE);
        ahb_write(std_logic_vector(unsigned(f_reg_addr(BTR_ADR)) + 3), x"44444444", C_HSIZE_BYTE);
        ahb_read(f_reg_addr(BTR_ADR), rdata);
        check_m(rdata = x"44332211", "Byte-wise BTR write mismatch");

        ahb_write(std_logic_vector(unsigned(f_reg_addr(BTR_ADR)) + 0), x"BEEFBEEF", C_HSIZE_HALFWORD);
        ahb_write(std_logic_vector(unsigned(f_reg_addr(BTR_ADR)) + 2), x"CAFECAFE", C_HSIZE_HALFWORD);
        ahb_read(f_reg_addr(BTR_ADR), rdata);
        check_m(rdata = x"CAFEBEEF", "Half-word-wise BTR write mismatch");

        ------------------------------------------------------------------------
        -- Read-immediately-after-write pipeline hazard: ahb_ifc must insert
        -- exactly one wait state when a read's address phase overlaps the
        -- data phase of the preceding write (see ahb_ifc.vhd hreadyout logic
        -- and doc/system_architecture/timing_diagrams/ahb_access.json, phase
        -- "Read after Write"). Driven manually (not through ahb_write/
        -- ahb_read) since those procedures always idle the bus in between.
        ------------------------------------------------------------------------
        info_m("Checking read-after-write pipeline hazard");

        -- Beat 0: write BTR (address phase)
        haddr     <= f_reg_addr(BTR_ADR);
        hwrite    <= '1';
        hsize     <= C_HSIZE_WORD;
        hburst    <= C_HBURST_SINGLE;
        htrans    <= C_HTRANS_NONSEQ;
        hprot     <= (others => '0');
        hmastlock <= '0';
        hsel      <= '1';

        wait until rising_edge(hclk);
        wait for 1 ps;
        check_m(hready = '1', "Unexpected wait state on write address phase");

        -- Beat 1: read DEVICE_ID - address phase overlaps write's data phase
        haddr  <= f_reg_addr(DEVICE_ID_ADR);
        hwrite <= '0';
        htrans <= C_HTRANS_NONSEQ;
        hwdata <= x"01234567";

        wait until rising_edge(hclk);
        wait for 1 ps;
        check_m(hready = '0',
                "Expected a 1 cycle wait state on read-after-write pipeline hazard");

        wait until rising_edge(hclk);
        wait for 1 ps;
        check_m(hready = '1', "Read-after-write hazard did not resolve after one wait state");

        -- Drain: data phase of the read
        htrans <= C_HTRANS_IDLE;
        hsel   <= '0';
        check_m(hrdata(15 downto 0) = CTU_CAN_FD_ID,
                "DEVICE_ID readback wrong during read-after-write hazard test");

        wait until rising_edge(hclk);

        -- Confirm the pipelined write actually committed correctly
        ahb_read(f_reg_addr(BTR_ADR), rdata);
        check_m(rdata = x"01234567",
                "Write data lost/corrupted during read-after-write pipeline hazard");

        ------------------------------------------------------------------------
        -- Write-immediately-after-read pipeline hazard: ahb_ifc now stalls
        -- the following address phase for one cycle after *every* accepted
        -- read (not just a following read), so a write whose address phase
        -- overlaps the previous read's data phase also sees the wait state.
        -- Driven manually (not through ahb_write/ahb_read) since those
        -- procedures always idle the bus in between.
        ------------------------------------------------------------------------
        info_m("Checking write-after-read pipeline hazard");

        -- Beat 0: read DEVICE_ID (address phase)
        haddr     <= f_reg_addr(DEVICE_ID_ADR);
        hwrite    <= '0';
        hsize     <= C_HSIZE_WORD;
        hburst    <= C_HBURST_SINGLE;
        htrans    <= C_HTRANS_NONSEQ;
        hprot     <= (others => '0');
        hmastlock <= '0';
        hsel      <= '1';

        wait until rising_edge(hclk);
        wait for 1 ps;
        check_m(hready = '0', "Wait state in write-after-read beat 1");

        -- Beat 1: write BTR - address phase overlaps read's data phase
        haddr  <= f_reg_addr(BTR_ADR);
        hwrite <= '1';
        htrans <= C_HTRANS_NONSEQ;

        wait until rising_edge(hclk);
        wait for 1 ps;
        check_m(hready = '1', "No Wait state in write-after-read beat 2");
        check_m(hrdata(15 downto 0) = CTU_CAN_FD_ID,
                "DEVICE_ID readback wrong during write-after-read hazard test");

        wait until rising_edge(hclk);
        wait for 1 ps;
        check_m(hready = '1', "No Wait state in write-after-read beat 3");

        hwdata <= x"5AA55AA5";
        htrans <= C_HTRANS_IDLE;
        hsel   <= '0';

        wait until rising_edge(hclk);
        check_m(hready = '1', "No Wait state in write-after-read beat 4");
        hwdata <= (others => '0');
        haddr  <= (others => '0');
        hwrite <= '0';
        wait until rising_edge(hclk);

        -- Confirm the pipelined write actually committed correctly
        ahb_read(f_reg_addr(BTR_ADR), rdata);
        check_m(rdata = x"5AA55AA5",
                "Write data lost/corrupted during write-after-read pipeline hazard");

        ------------------------------------------------------------------------
        -- Burst over a scattered (non-incrementing) address set - the
        -- interface must not assume/require addresses to auto-increment.
        ------------------------------------------------------------------------
        info_m("Checking burst over a scattered address set");

        scat_addr_arr(0) := f_reg_addr(FILTER_A_MASK_ADR);
        scat_addr_arr(1) := f_reg_addr(BTR_ADR);
        scat_addr_arr(2) := f_reg_addr(BTR_FD_ADR);

        for i in 0 to 2 loop
            rand_logic_vect_v(scat_wdata_arr(i), 0.5);
        end loop;
        scat_wdata_arr(0)(31 downto 29) := "000";
        scat_wdata_arr(2)(6) := '0';
        scat_wdata_arr(2)(12) := '0';
        scat_wdata_arr(2)(18) := '0';

        ahb_burst_write(scat_addr_arr, scat_wdata_arr, C_HSIZE_WORD);
        ahb_burst_read(scat_addr_arr, scat_rdata_arr, C_HSIZE_WORD);

        for i in 0 to 2 loop
            check_m(scat_rdata_arr(i) = scat_wdata_arr(i),
                    "Scattered-address burst readback mismatch, beat " & integer'image(i));
        end loop;

        ------------------------------------------------------------------------
        -- Mid-test hardware reset must restore the known reset value
        ------------------------------------------------------------------------
        info_m("Checking hardware reset mid-test");

        ahb_write(f_reg_addr(BTR_ADR), x"DEADBEEF", C_HSIZE_WORD);

        hresetn <= '0';
        wait until rising_edge(hclk);
        hresetn <= '1';

        wait until rising_edge(hclk);
        wait until rising_edge(hclk);
        wait until rising_edge(hclk);

        ahb_read(f_reg_addr(BTR_ADR), rdata);
        check_m(rdata = C_BTR_RSTVAL, "BTR not restored to reset value after HW reset");

        wait until rising_edge(hclk);
        wait until rising_edge(hclk);

        test_runner_cleanup(runner);
        std.env.finish;
    end process;

    ----------------------------------------------------------------------------
    -- Watchdog
    ----------------------------------------------------------------------------
    p_watchdog : process
    begin
        if (time'value(timeout) > 0 ns) then
            wait for time'value(timeout);
            report "AHB unit test: timeout reached!" severity failure;
            std.env.finish;
        else
            wait;
        end if;
    end process;

end architecture;
