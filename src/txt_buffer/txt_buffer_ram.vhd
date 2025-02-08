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
--  TXT Buffer RAM
--
-- Purpose:
--  Wrapper for dual port RAM in TXT Buffer.
--
-- Memory parameters:
--  Depth: 20
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

entity txt_buffer_ram is
    generic (
        -- TXT buffer ID
        G_ID                    :     natural;

        -- TXT Buffer parity
        G_SUP_PARITY            :     boolean;

        -- TXT Buffer RAM is resettable
        G_RESET_TXT_BUF_RAM     :     boolean
    );
    port (
        -------------------------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -------------------------------------------------------------------------------------------
        clk_sys                 : in  std_logic;
        res_n                   : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Parity configuration
        -------------------------------------------------------------------------------------------
        mr_settings_pchke       : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Memory Testability
        -------------------------------------------------------------------------------------------
        mr_tst_control_tmaena   : in  std_logic;
        mr_tst_control_twrstb   : in  std_logic;
        mr_tst_dest_tst_addr    : in  std_logic_vector(4 downto 0);
        mr_tst_dest_tst_mtgt    : in  std_logic_vector(3 downto 0);
        mr_tst_wdata_tst_wdata  : in  std_logic_vector(31 downto 0);

        mr_tst_rdata_tst_rdata  : out std_logic_vector(31 downto 0);

        -------------------------------------------------------------------------------------------
        -- Port A - Write (from Memory registers)
        -------------------------------------------------------------------------------------------
        txtb_port_a_address     : in  std_logic_vector(4 downto 0);
        txtb_port_a_data_in     : in  std_logic_vector(31 downto 0);
        txtb_port_a_parity      : in  std_logic;
        txtb_port_a_write       : in  std_logic;
        txtb_port_a_be          : in  std_logic_vector(3 downto 0);

        -------------------------------------------------------------------------------------------
        -- Port B - Read (from CAN Core)
        -------------------------------------------------------------------------------------------
        txtb_port_b_address     : in  std_logic_vector(4 downto 0);
        txtb_port_b_data_out    : out std_logic_vector(31 downto 0);

        -------------------------------------------------------------------------------------------
        -- Parity mismatch
        -------------------------------------------------------------------------------------------
        parity_mismatch         : out std_logic
    );
end entity;

architecture rtl of txt_buffer_ram is

    -----------------------------------------------------------------------------------------------
    -- FRAME_FORMAT_W
    -- IDENTIFIER_W
    -- TIMESTAMP_U_W
    -- TIMESTAMP_L_W
    -- DATA_*_*  (16 words)
    -- FRAME_TEST_W
    -----------------------------------------------------------------------------------------------
    constant C_TXT_BUF_DEPTH            : natural := 21;

    signal txtb_port_a_address_i        : std_logic_vector(4 downto 0);
    signal txtb_port_a_write_i          : std_logic;
    signal txtb_port_a_data_i           : std_logic_vector(31 downto 0);

    signal txtb_port_b_address_i        : std_logic_vector(4 downto 0);
    signal txtb_port_b_data_out_i       : std_logic_vector(31 downto 0);

    signal tst_ena                      : std_logic;

    signal parity_word                  : std_logic_vector(C_TXT_BUF_DEPTH - 1 downto 0);
    signal parity_read_real             : std_logic;
    signal parity_read_exp              : std_logic;

begin

    -----------------------------------------------------------------------------------------------
    -- RAM is implemented as synchronous inferred RAM for FPGAs. Synchronous RAM is chosen since
    -- some FPGA families does not provide inferred RAM for asynchronously read data (in the same
    -- clock cycle).
    -----------------------------------------------------------------------------------------------
    txt_buf_ram_inst : entity ctu_can_fd_rtl.inf_ram_wrapper
    generic map (
        G_WORD_WIDTH            => 32,
        G_DEPTH                 => C_TXT_BUF_DEPTH,
        G_ADDRESS_WIDTH         => txtb_port_a_address'length,
        G_SYNC_READ             => true,
        G_RESETABLE             => G_RESET_TXT_BUF_RAM
    )
    port map(
        clk_sys                 => clk_sys,                             -- IN
        res_n                   => res_n,                               -- IN

        addr_a                  => txtb_port_a_address_i,               -- IN
        write                   => txtb_port_a_write_i,                 -- IN
        data_in                 => txtb_port_a_data_i,                  -- IN
        be                      => txtb_port_a_be,                      -- IN

        addr_b                  => txtb_port_b_address_i,               -- IN
        data_out                => txtb_port_b_data_out_i               -- OUT
    );
    txtb_port_b_data_out <= txtb_port_b_data_out_i;

    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    -- Parity protection
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    parity_true_gen : if (G_SUP_PARITY) generate

        -------------------------------------------------------------------------------------------
        -- Storing Parity word
        -------------------------------------------------------------------------------------------
        parity_word_proc : process(res_n, clk_sys)
        begin
            if (res_n = '0') then
                parity_word <= (others => '0');
            elsif rising_edge(clk_sys) then
                if (txtb_port_a_write = '1') then
                    parity_word(to_integer(unsigned(txtb_port_a_address))) <= txtb_port_a_parity;
                end if;
            end if;
        end process;

        -------------------------------------------------------------------------------------------
        -- Parity decoding
        -------------------------------------------------------------------------------------------
        parity_calculator_read_inst : entity ctu_can_fd_rtl.parity_calculator
        generic map (
            G_WIDTH         => 32,
            G_PARITY_TYPE   => C_PARITY_TYPE
        )
        port map(
            data_in         => txtb_port_b_data_out_i,
            parity          => parity_read_real
        );

        -------------------------------------------------------------------------------------------
        -- Parity check
        --
        -- When reading from TXT Buffer RAM, read data are obtained one clock cycle later!
        -------------------------------------------------------------------------------------------
        parity_check_proc : process(clk_sys, res_n)
        begin
            if (res_n = '0') then
                parity_read_exp <= '0';
            elsif (rising_edge(clk_sys)) then
                parity_read_exp <= parity_word(to_integer(unsigned(txtb_port_b_address)));
            end if;
        end process;

        parity_mismatch <= '1' when (parity_read_real /= parity_read_exp) and (mr_settings_pchke = '1')
                               else
                           '0';

    end generate;

    parity_false_gen : if (not G_SUP_PARITY) generate
        parity_mismatch <= '0';
        parity_read_exp <= '0';
        parity_read_real <= '0';
        parity_word <= (others => '0');
    end generate;

    -----------------------------------------------------------------------------------------------
    -- Memory testability
    -----------------------------------------------------------------------------------------------
    tst_ena <= '1' when (mr_tst_control_tmaena = '1') and
                        (mr_tst_dest_tst_mtgt = std_logic_vector(to_unsigned(G_ID + 2, 4)))
                   else
               '0';

    -- Write port
    txtb_port_a_address_i <= txtb_port_a_address when (tst_ena = '0')
                                                 else
                             mr_tst_dest_tst_addr(4 downto 0);

    txtb_port_a_write_i <= txtb_port_a_write when (tst_ena = '0')
                                             else
                           mr_tst_control_twrstb;

    txtb_port_a_data_i <= txtb_port_a_data_in when (tst_ena = '0')
                                              else
                          mr_tst_wdata_tst_wdata;

    -- Read port
    txtb_port_b_address_i <= txtb_port_b_address when (tst_ena = '0')
                                                 else
                             mr_tst_dest_tst_addr(4 downto 0);

    mr_tst_rdata_tst_rdata <= txtb_port_b_data_out_i when (tst_ena = '1')
                                                     else
                                     (others => '0');

    -----------------------------------------------------------------------------------------------
    -- Assertions and functional coverage
    -----------------------------------------------------------------------------------------------

    -- psl default clock is rising_edge(clk_sys);

    -- psl txt_ram_0_test_cov : cover
    --   {tst_ena = '0' and mr_tst_control_twrstb = '1'};

end architecture;