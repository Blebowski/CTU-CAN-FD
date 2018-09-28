--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018
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
-- Purpose:
--    Simple test of can_top_apb, performing register read/write via APB.
--------------------------------------------------------------------------------
-- Revision History:
--    April 2018   First Implementation - Martin Jerabek
--      May 2018   Converted from AXI to APB, using CAN_test intrface
--------------------------------------------------------------------------------

library ieee;
library work;
use ieee.std_logic_1164.all;
USE work.CANtestLib.All;
USE work.CANconstants.All;
use work.CAN_FD_register_map.all;

architecture apb_unit_test of CAN_test is
    component CTU_CAN_FD_v1_0 is
        generic(
            use_logger       : boolean                := true;
            rx_buffer_size   : natural range 4 to 512 := 128;
            use_sync         : boolean                := true;
            sup_filtA        : boolean                := true;
            sup_filtB        : boolean                := true;
            sup_filtC        : boolean                := true;
            sup_range        : boolean                := true;
            logger_size      : natural range 0 to 512 := 8
        );
        port(
            aclk             : in  std_logic;
            arstn            : in  std_logic;

            irq              : out std_logic;
            CAN_tx           : out std_logic;
            CAN_rx           : in  std_logic;
            time_quanta_clk  : out std_logic;
            timestamp        : in std_logic_vector(63 downto 0);

            -- Ports of APB4
            s_apb_paddr      : in  std_logic_vector(31 downto 0);
            s_apb_penable    : in  std_logic;
            s_apb_pprot      : in  std_logic_vector(2 downto 0);
            s_apb_prdata     : out std_logic_vector(31 downto 0);
            s_apb_pready     : out std_logic;
            s_apb_psel       : in  std_logic;
            s_apb_pslverr    : out std_logic;
            s_apb_pstrb      : in  std_logic_vector(3 downto 0);
            s_apb_pwdata     : in  std_logic_vector(31 downto 0);
            s_apb_pwrite     : in  std_logic
    );
    end component CTU_CAN_FD_v1_0;

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
    can: CTU_CAN_FD_v1_0
        generic map (
            use_logger  => false,
            use_sync    => false,
            sup_filtA   => false,
            sup_filtB   => false,
            sup_filtC   => false,
            sup_range   => false
        )
        port map (
            CAN_rx           => '1',
            timestamp        => (others => '0'),
            aclk           => aclk,
            arstn          => arstn,

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

    main:process
        procedure apb_write(constant addr : std_logic_vector(11 downto 0);
                            constant data : std_logic_vector(31 downto 0);
                            constant be   : std_logic_vector(3 downto 0)) is
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
                assert i < 5 report "Peripheral is stalling for too many cycles for APB write." severity error;
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
                assert i < 5 report "Peripheral is stalling for too many cycles for APB read." severity error;
            end loop;
            s_apb_penable <= '0';
            s_apb_psel <= '0';
        end procedure;

        procedure apb_test_pattern(constant addr : std_logic_vector(11 downto 0);
                                   constant data : std_logic_vector(31 downto 0)) is
        begin
            apb_write(addr, data, b"1111");
            apb_read(addr);
            assert s_apb_prdata = data report "pattern " & to_hstring(data) & " mismatch: got " & to_hstring(s_apb_prdata);
        end procedure;
    begin
        s_apb_penable  <= '0';
        s_apb_pprot    <= (others => 'X');
        s_apb_psel     <= '0';

        log("Restarting APB test", info_l, log_level);
        status         <= waiting;
        if not run then wait until run; end if;
        print_test_info(iterations, log_level, error_beh, error_tol);
        wait until rising_edge(aclk); -- some time in reset
        arstn     <= '1';
        status    <= running;
        error_ctr <= 0;

        -- do not wait 2 cycles after HW reset - the APB interface should account for it

        -- read just after HW reset
        apb_read(DEVICE_ID_ADR);
        assert s_apb_prdata = x"0201CAFD" report "CAN ID reg mismatch (just after HW reset)";

        apb_write(BTR_ADR, x"FFFFFFFF", b"1111");
        apb_read(DEVICE_ID_ADR);
        assert s_apb_prdata = x"0201CAFD" report "CAN ID reg mismatch";
        apb_read(BTR_ADR);
        assert s_apb_prdata = x"FFFFFFFF" report "readback mismatch";

        apb_write(BTR_ADR, x"00000000", b"0011");
        apb_read(BTR_ADR);
        assert s_apb_prdata = x"FFFF0000" report "write low word: readback mismatch";

        apb_write(BTR_ADR, x"FFFFFFFF", b"1111");
        apb_write(BTR_ADR, x"00000000", b"1100");
        apb_read(BTR_ADR);
        assert s_apb_prdata = x"0000FFFF" report "write high word: readback mismatch";

        apb_test_pattern(BTR_ADR, x"AAAAAAAA");
        apb_test_pattern(BTR_ADR, x"55555555");
        apb_test_pattern(BTR_ADR, x"92492492");
        apb_test_pattern(BTR_ADR, x"49249249");
        apb_test_pattern(BTR_ADR, x"24924924");

        apb_write(BTR_ADR, x"87654321", b"1111");
        apb_read(BTR_ADR);
        assert s_apb_prdata = x"87654321" report "readback mismatch";
        apb_write(BTR_ADR, x"000055aa", b"0011");
        apb_read(BTR_ADR);
        assert s_apb_prdata = x"876555aa" report "write low word: readback mismatch";

        -- write after HW reset
        arstn     <= '0';
        wait until rising_edge(aclk);
        arstn     <= '1';
        apb_write(BTR_ADR, x"DEADBEEF", b"1111");
        apb_read(BTR_ADR);
        assert s_apb_prdata = x"DEADBEEF" report "Readback for write-after-HW-reset mismatch.";

        wait until aclk;
        wait until aclk;
        wait until aclk;
        wait until aclk;

        evaluate_test(error_tol, error_ctr, status);
    end process;
end architecture;
