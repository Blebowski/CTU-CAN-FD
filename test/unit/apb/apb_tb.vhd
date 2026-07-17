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
--    Simple test of ctu_can_fd_top_apb, performing register read/write via APB.
--
-- @Verifies:
--  @1. Read transactions to CTU CAN FD via APB.
--  @2. Write transactions to CTU CAN FD via APB.
--  @3. 32 bit and 16 bit transactions.
--
-- @TestInfoEnd
--------------------------------------------------------------------------------
-- Revision History:
--    April 2018   First Implementation - Martin Jerabek
--      May 2018   Converted from AXI to APB, using CAN_test intrface
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;
use ieee.std_logic_textio.all;
use STD.textio.all;

library ctu_can_fd_rtl;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.can_config_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

library ctu_can_fd_tb_unit;
use ctu_can_fd_tb_unit.can_unit_test_pkg.all;
use ctu_can_fd_tb_unit.random_unit_pkg.all;

library vunit_lib;
context vunit_lib.vunit_context;


architecture apb_unit_test of CAN_test is
    signal s_apb_paddr      : std_logic_vector(31 downto 0);
    signal s_apb_penable    : std_logic;
    signal s_apb_pprot      : std_logic_vector(2 downto 0);
    signal s_apb_prdata     : std_logic_vector(31 downto 0);
    signal s_apb_pready     : std_logic;
    signal s_apb_psel       : std_logic;
    signal s_apb_pslverr    : std_logic;
    signal s_apb_pstrb      : std_logic_vector(3 downto 0);
    signal s_apb_pwdata     : std_logic_vector(31 downto 0);
    signal s_apb_pwrite     : std_logic;

    signal aclk : std_logic := '0';
    signal arstn : std_logic := '0';

begin
    i_can : entity ctu_can_fd_rtl.ctu_can_fd_top_apb
        generic map (
            G_RX_BUF_SIZE       => 128,
            G_TXT_BUF_COUNT     => 4,
            G_FILT_A_EN         => false,
            G_FILT_B_EN         => false,
            G_FILT_C_EN         => false,
            G_FILT_RANGE_EN     => false,
            G_TEST_REGS_EN      => true
        )
        port map (
            CAN_rx           => '1',
            timestamp        => (others => '0'),
            aclk             => aclk,
            arstn            => arstn,
            rst_n_out        => open,

            scan_mode        => '0',

            -- APB ports
            s_apb_paddr    => s_apb_paddr,
            s_apb_penable  => s_apb_penable,
            s_apb_pprot    => s_apb_pprot,
            s_apb_prdata   => s_apb_prdata,
            s_apb_pready   => s_apb_pready,
            s_apb_psel     => s_apb_psel,
            s_apb_pslverr  => s_apb_pslverr,
            s_apb_pstrb    => s_apb_pstrb,
            s_apb_pwdata   => s_apb_pwdata,
            s_apb_pwrite   => s_apb_pwrite
        );

    aclk <= not aclk after 100 ns;

    assert s_apb_pslverr = '0' or arstn = '0' or now = 0 fs report "Slave error!" severity error;

    p_main : process
        procedure apb_write(
            constant addr : std_logic_vector(11 downto 0);
            constant data : std_logic_vector(31 downto 0);
            constant be   : std_logic_vector(3 downto 0)
        ) is
            variable i : natural := 0;
        begin
            s_apb_paddr <= (others => '0');
            s_apb_paddr(11 downto 0) <= addr;
            s_apb_pwdata <= data;
            s_apb_pwrite <= '1';
            s_apb_pstrb <= be;
            s_apb_psel <= '1';
            s_apb_penable <= '0';

            wait until rising_edge(aclk);
            s_apb_penable <= '1';
            L: loop
                wait until rising_edge(aclk);
                exit L when s_apb_pready = '1';
                i := i + 1;
                check(i < 5, "Peripheral is stalling for too many cycles for APB write.");
            end loop;
            s_apb_penable <= '0';
            s_apb_psel <= '0';
        end procedure;

        procedure apb_read(constant addr : std_logic_vector(11 downto 0)) is
            variable i : natural := 0;
        begin
            s_apb_paddr <= (others => '0');
            s_apb_paddr(11 downto 0) <= addr;
            s_apb_pwrite <= '0';
            s_apb_pstrb <= (others => '1');
            s_apb_psel <= '1';
            s_apb_penable <= '0';

            wait until rising_edge(aclk);
            s_apb_penable <= '1';
            L: loop
                wait until rising_edge(aclk);
                exit L when s_apb_pready = '1';
                i := i + 1;
                check(i < 5, "Peripheral is stalling for too many cycles for APB read.");
            end loop;
            s_apb_penable <= '0';
            s_apb_psel <= '0';
        end procedure;

        procedure apb_test_pattern(constant addr : std_logic_vector(11 downto 0);
                                   constant data : std_logic_vector(31 downto 0)) is
        begin
            apb_write(addr, data, b"1111");
            apb_read(addr);
            check(s_apb_prdata = data, "pattern " & to_hstring(data) & " mismatch: got " & to_hstring(s_apb_prdata));
        end procedure;
    begin
        s_apb_penable  <= '0';
        s_apb_pprot    <= (others => 'X');
        s_apb_psel     <= '0';

        info("Restarting APB test");
        status         <= waiting;
        if not run then wait until run; end if;
        print_test_info(iterations, log_level, error_beh, error_tol);
        wait until rising_edge(aclk); -- some time in reset
        arstn     <= '1';
        status    <= running;
        error_ctr <= 0;

        wait until rising_edge(aclk);
        wait until rising_edge(aclk);
        wait until rising_edge(aclk);

        apb_read(DEVICE_ID_ADR);
        check(s_apb_prdata = x"0204CAFD", "CAN ID reg mismatch (just after HW reset)");

        apb_write(BTR_ADR, x"FFFFFFFF", b"1111");
        apb_read(DEVICE_ID_ADR);
        check(s_apb_prdata = x"0204CAFD", "CAN ID reg mismatch");
        apb_read(BTR_ADR);
        check(s_apb_prdata = x"FFFFFFFF", "readback mismatch");

        apb_write(BTR_ADR, x"00000000", b"0011");
        apb_read(BTR_ADR);
        check(s_apb_prdata = x"FFFF0000", "write low word: readback mismatch");

        apb_write(BTR_ADR, x"FFFFFFFF", b"1111");
        apb_write(BTR_ADR, x"00000000", b"1100");
        apb_read(BTR_ADR);
        check(s_apb_prdata = x"0000FFFF", "write high word: readback mismatch");

        apb_test_pattern(BTR_ADR, x"AAAAAAAA");
        apb_test_pattern(BTR_ADR, x"55555555");
        apb_test_pattern(BTR_ADR, x"92492492");
        apb_test_pattern(BTR_ADR, x"49249249");
        apb_test_pattern(BTR_ADR, x"24924924");

        apb_write(BTR_ADR, x"87654321", b"1111");
        apb_read(BTR_ADR);
        check(s_apb_prdata = x"87654321", "readback mismatch");
        apb_write(BTR_ADR, x"000055aa", b"0011");
        apb_read(BTR_ADR);
        check(s_apb_prdata = x"876555aa", "write low word: readback mismatch");

        -- write after HW reset
        arstn     <= '0';
        wait until rising_edge(aclk);
        arstn     <= '1';

        wait until rising_edge(aclk);
        wait until rising_edge(aclk);

        apb_write(BTR_ADR, x"DEADBEEF", b"1111");
        apb_read(BTR_ADR);
        check(s_apb_prdata = x"DEADBEEF", "Readback for write-after-HW-reset mismatch.");

        wait until aclk;
        wait until aclk;
        wait until aclk;
        wait until aclk;

        evaluate_test(error_tol, error_ctr, status);
    end process;
end architecture;
