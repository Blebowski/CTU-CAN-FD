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
--  Inferred RAM wrapper.
--
-- Purpose:
--  Dual port Memory wrapper for inferrence of RAM blocks in Intel and Xilinx
--  FPGAs. Port A is write only. Port B is read only. Port A interface is
--  synchronous. Read interface is either combinatorial (asynchronous) or
--  registered (synchronous). Clock is shared between the two ports. If used
--  on ASIC or FPGA without memories, synthesized as DFFs without Set or Reset.
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;

entity inf_ram_wrapper is
    generic (
        -- Width of memory word (in bits)
        G_WORD_WIDTH           :     natural := 32;

        -- Memory depth (in words)
        G_DEPTH                :     natural := 32;

        -- Address width (in bits)
        G_ADDRESS_WIDTH        :     natural := 8;

        -- Synchronous read
        G_SYNC_READ            :     boolean := true;

        -- If true, res_n causes RAM to be reset
        G_RESETABLE            :     boolean := false
    );
  port (
        -------------------------------------------------------------------------------------------
        -- Clock and Reset
        -------------------------------------------------------------------------------------------
        clk_sys     : in  std_logic;
        res_n       : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Port A - Data input
        -------------------------------------------------------------------------------------------
        addr_A      : in  std_logic_vector(G_ADDRESS_WIDTH - 1 downto 0);
        write       : in  std_logic;
        data_in     : in  std_logic_vector(G_WORD_WIDTH - 1 downto 0);
        be          : in  std_logic_vector(G_WORD_WIDTH / 8 - 1 downto 0);

        -------------------------------------------------------------------------------------------
        -- Port B - Data output
        -------------------------------------------------------------------------------------------
        addr_B      : in  std_logic_vector(G_ADDRESS_WIDTH - 1 downto 0);
        data_out    : out std_logic_vector(G_WORD_WIDTH - 1 downto 0)
    );
end entity;

architecture rtl of inf_RAM_wrapper is

    -----------------------------------------------------------------------------------------------
    -- Memory definition
    -----------------------------------------------------------------------------------------------
    type memory_type is array(0 to G_DEPTH - 1) of
            std_logic_vector(G_WORD_WIDTH - 1 downto 0);
    signal ram_memory        : memory_type;

    signal int_read_data     : std_logic_vector(G_WORD_WIDTH - 1 downto 0);
    signal byte_we           : std_logic_vector(G_WORD_WIDTH/8 - 1 downto 0);

begin

    -----------------------------------------------------------------------------------------------
    -- Memory Write access process - per byte
    -----------------------------------------------------------------------------------------------
    byte_gen : for i in 0 to G_WORD_WIDTH/8 - 1 generate
        byte_we(i) <= '1' when (write = '1' and be(i) = '1')
                          else
                      '0';
    end generate;

    -----------------------------------------------------------------------------------------------
    -- RAM memory (non-resetable version)
    -----------------------------------------------------------------------------------------------
    ram_rst_false_gen : if (not G_RESETABLE) generate

        ram_write_process : process(clk_sys)
        begin
            if (rising_edge(clk_sys)) then
                for i in 0 to G_WORD_WIDTH/8 - 1 loop
                    if (byte_we(i) = '1') then
                        ram_memory(to_integer(unsigned(addr_A)))(i * 8 + 7 downto i * 8)
                            <= data_in(i * 8 + 7 downto i * 8);
                    end if;
                end loop;
            end if;
        end process;

    end generate;

    -----------------------------------------------------------------------------------------------
    -- RAM memory (resetable version)
    -----------------------------------------------------------------------------------------------
    ram_rst_true_gen : if (G_RESETABLE) generate

        ram_write_process : process(clk_sys, res_n)
        begin
            if (res_n = '0') then
                ram_memory <= (others => (others => '0'));
            elsif (rising_edge(clk_sys)) then
                for i in 0 to G_WORD_WIDTH/8 - 1 loop
                    if (byte_we(i) = '1') then
                        ram_memory(to_integer(unsigned(addr_A)))(i * 8 + 7 downto i * 8)
                            <= data_in(i * 8 + 7 downto i * 8);
                    end if;
                end loop;
            end if;
        end process;

    end generate;

    -----------------------------------------------------------------------------------------------
    -- Memory read access
    -----------------------------------------------------------------------------------------------
    int_read_data <= ram_memory(to_integer(unsigned(addr_B)));

    -- Synchronous read
    sync_read_gen : if (G_SYNC_READ) generate
        ram_read_process : process(clk_sys)
        begin
            if (rising_edge(clk_sys)) then
                data_out <= int_read_data;
            end if;
        end process;
    end generate;

    -- Asynchronous read
    async_read_gen : if (not G_SYNC_READ) generate
        data_out <= int_read_data;
    end generate;


    -----------------------------------------------------------------------------------------------
    -- Assertions on size
    -----------------------------------------------------------------------------------------------
    assert ((G_WORD_WIDTH = 8) or
            (G_WORD_WIDTH = 16) or
            (G_WORD_WIDTH = 32) or
            (G_WORD_WIDTH = 64) or
            (G_WORD_WIDTH = 128))
    report "Unsupported inferred RAM word width! " &
           "Only 8, 16, 32, 64 and 128 are allowed!"
        severity failure;

end architecture;
