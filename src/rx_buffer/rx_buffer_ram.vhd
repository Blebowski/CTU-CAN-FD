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
--  RX Buffer RAM
--
-- Purpose:
--  Wrapper for dual port RAM on RX Buffer! Port A - write only, Port B - read
--  only.
--
-- Memory parameters:
--  Depth: G_RX_BUFF_SIZE
--  Word size: 32 bits
--  Read: Synchronous
--  Write: Synchronous
--  Port A: Write Only
--  Port B: Read only
--
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

entity rx_buffer_ram is
    generic (
        -- RX Buffer size
        G_RX_BUFF_SIZE          :       natural range 32 to 4096;

        -- Width of RX Buffer pointers
        G_RX_BUFF_PTR_WIDTH     :       natural range 5 to 12;

        -- Add parity to RX Buffer RAM
        G_SUP_PARITY            :       boolean;

        -- Reset RX Buffer RAM
        G_RESET_RX_BUF_RAM      :       boolean
    );
    port(
        -------------------------------------------------------------------------------------------
        -- Clocks and Asynchronous reset
        -------------------------------------------------------------------------------------------
        res_n                   : in  std_logic;
        clk_sys                 : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Memory testability
        -------------------------------------------------------------------------------------------
        mr_tst_control_tmaena   : in  std_logic;
        mr_tst_control_twrstb   : in  std_logic;
        mr_tst_dest_tst_addr    : in  std_logic_vector(15 downto 0);
        mr_tst_dest_tst_mtgt    : in  std_logic_vector(3 downto 0);
        mr_tst_wdata_tst_wdata  : in  std_logic_vector(31 downto 0);

        mr_tst_rdata_tst_rdata  : out std_logic_vector(31 downto 0);

        -------------------------------------------------------------------------------------------
        -- Port A - Write (from CAN Core)
        -------------------------------------------------------------------------------------------
        rxb_port_a_address      : in  std_logic_vector(G_RX_BUFF_PTR_WIDTH - 1 downto 0);
        rxb_port_a_data_in      : in  std_logic_vector(31 downto 0);
        rxb_port_a_write        : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Port B - Read (from Memory registers)
        -------------------------------------------------------------------------------------------
        rxb_port_b_address      : in  std_logic_vector(G_RX_BUFF_PTR_WIDTH - 1 downto 0);
        rxb_port_b_data_out     : out std_logic_vector(31 downto 0);

        -------------------------------------------------------------------------------------------
        -- Parity mismatch
        -------------------------------------------------------------------------------------------
        parity_mismatch         : out std_logic
    );
end entity;

architecture rtl of rx_buffer_ram is

    signal rxb_port_a_address_i     : std_logic_vector(G_RX_BUFF_PTR_WIDTH - 1 downto 0);
    signal rxb_port_a_write_i       : std_logic;
    signal rxb_port_a_data_in_i     : std_logic_vector(31 downto 0);
    signal rxb_port_b_address_i     : std_logic_vector(G_RX_BUFF_PTR_WIDTH - 1 downto 0);
    signal rxb_port_b_data_out_i    : std_logic_vector(31 downto 0);

    signal mr_tst_dest_tst_addr_pad : std_logic_vector(15 downto 0);

    signal tst_ena                  : std_logic;

    signal parity_word              : std_logic_vector(G_RX_BUFF_SIZE - 1 downto 0);
    signal parity_write             : std_logic;
    signal parity_read_real         : std_logic;
    signal parity_read_exp          : std_logic;

begin

    -------------------------------------------------------------------------------------------
    -- RAM is implemented as synchronous inferred RAM for FPGAs. Synchronous RAM is chosen
    -- since some FPGA families does not provide inferred RAM for asynchronously read data (in
    -- the same clock cycle).
    -------------------------------------------------------------------------------------------
    rx_buf_ram_inst : entity ctu_can_fd_rtl.inf_ram_wrapper
    generic map (
        G_WORD_WIDTH            => 32,
        G_DEPTH                 => G_RX_BUFF_SIZE,
        G_ADDRESS_WIDTH         => G_RX_BUFF_PTR_WIDTH,
        G_SYNC_READ             => true,
        G_RESETABLE             => G_RESET_RX_BUF_RAM
    )
    port map(
        clk_sys                 => clk_sys,                 -- IN
        res_n                   => res_n,                   -- IN

        addr_A                  => rxb_port_a_address_i,    -- IN
        write                   => rxb_port_a_write_i,      -- IN
        data_in                 => rxb_port_a_data_in_i,    -- IN
        be                      => "1111",                  -- IN

        addr_B                  => rxb_port_b_address_i,    -- IN
        data_out                => rxb_port_b_data_out_i    -- OUT
    );
    rxb_port_b_data_out <= rxb_port_b_data_out_i;

    -- Note: If you want to replace RAM by dedicated memory macro,
    --       place it instead of "inf_RAM_wrapper"!

    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    -- Parity protection
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    parity_true_gen : if (G_SUP_PARITY) generate
    begin

        -------------------------------------------------------------------------------------------
        -- Parity encoding
        -------------------------------------------------------------------------------------------
        parity_calculator_write_inst : entity ctu_can_fd_rtl.parity_calculator
        generic map (
            G_WIDTH             => 32,
            G_PARITY_TYPE       => C_PARITY_TYPE
        )
        port map(
            data_in             => rxb_port_a_data_in,
            parity              => parity_write
        );

        -------------------------------------------------------------------------------------------
        -- Parity memory (vector with per word bit of RAM)
        -------------------------------------------------------------------------------------------
        parity_mem_proc : process(clk_sys, res_n)
        begin
            if (res_n = '0') then
                parity_word <= (others => '0');
            elsif (rising_edge(clk_sys)) then
                if (rxb_port_a_write = '1') then
                    parity_word(to_integer(unsigned(rxb_port_a_address))) <= parity_write;
                end if;
            end if;
        end process;

        -------------------------------------------------------------------------------------------
        -- Parity decoding
        -------------------------------------------------------------------------------------------
        parity_calculator_read_inst : entity ctu_can_fd_rtl.parity_calculator
        generic map (
            G_WIDTH             => 32,
            G_PARITY_TYPE       => C_PARITY_TYPE
        )
        port map(
            data_in             => rxb_port_b_data_out_i,
            parity              => parity_read_real
        );

        -------------------------------------------------------------------------------------------
        -- Parity check
        --
        -- When reading from RX Buffer RAM, read data are obtained one clock cycle later!
        -------------------------------------------------------------------------------------------
        parity_check_proc : process(clk_sys, res_n)
        begin
            if (res_n = '0') then
                parity_read_exp <= '0';
            elsif (rising_edge(clk_sys)) then
                parity_read_exp <= parity_word(to_integer(unsigned(rxb_port_b_address)));
            end if;
        end process;

        parity_mismatch <= '1' when (parity_read_real /= parity_read_exp)
                               else
                           '0';

    end generate parity_true_gen;

    parity_false_gen : if (not G_SUP_PARITY) generate
        parity_mismatch <= '0';
        parity_read_real <= '0';
        parity_read_exp <= '0';
        parity_word <= (others => '0');
        parity_write <= '0';
    end generate;


    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    -- Memory testability
    --
    -- When memory test is enabled, control by Test registers.
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    process (mr_tst_dest_tst_addr)
    begin
        mr_tst_dest_tst_addr_pad <=
            std_logic_vector(unsigned(mr_tst_dest_tst_addr) mod G_RX_BUFF_SIZE);
    end process;

    tst_ena <= '1' when (mr_tst_control_tmaena = '1') and (mr_tst_dest_tst_mtgt = TMTGT_RXBUF)
                   else
               '0';

    -- Write port
    rxb_port_a_address_i <= rxb_port_a_address when (tst_ena = '0')
                                               else
                            mr_tst_dest_tst_addr_pad(G_RX_BUFF_PTR_WIDTH - 1 downto 0);

    rxb_port_a_write_i <= rxb_port_a_write when (tst_ena = '0')
                                           else
                          mr_tst_control_twrstb;

    rxb_port_a_data_in_i <= rxb_port_a_data_in when (tst_ena = '0')
                                               else
                            mr_tst_wdata_tst_wdata;

    -- Read port
    rxb_port_b_address_i <= rxb_port_b_address when (tst_ena = '0')
                                               else
                            mr_tst_dest_tst_addr_pad(G_RX_BUFF_PTR_WIDTH - 1 downto 0);

    mr_tst_rdata_tst_rdata <= rxb_port_b_data_out_i when (tst_ena = '1')
                                                    else
                              (others => '0');


    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    -- Assertions and functional coverage
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------

    -- psl default clock is rising_edge(clk_sys);
    --
    -- psl rx_ram_port_a_no_addr_overflow : assert never
    --  to_integer(unsigned(rxb_port_a_address)) >= G_RX_BUFF_SIZE
    --  report "RX Buffer RAM - Port A address overflow";
    --
    -- psl rx_ram_port_b_no_addr_overflow : assert never
    --  to_integer(unsigned(rxb_port_b_address)) >= G_RX_BUFF_SIZE
    --  report "RX Buffer RAM - Port B address overflow";
    --
    -- psl rx_ram_test_cov : cover
    --   {tst_ena = '0' and mr_tst_control_twrstb = '1'};

end architecture;