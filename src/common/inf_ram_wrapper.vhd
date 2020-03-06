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
    generic(
        -- Reset polarity
        G_RESET_POLARITY       :     std_logic := '1';
        
        -- Width of memory word (in bits)
        G_WORD_WIDTH           :     natural := 32;

        -- Memory depth (in words)
        G_DEPTH                :     natural := 32;

        -- Address width (in bits)
        G_ADDRESS_WIDTH        :     natural := 8;

        -- Synchronous read
        G_SYNC_READ            :     boolean := true
    );
  port(
        ------------------------------------------------------------------------
        -- Clock and Reset
        ------------------------------------------------------------------------
        clk_sys     :in   std_logic;
        res_n       :in   std_logic;

        ------------------------------------------------------------------------
        -- Port A - Data input
        ------------------------------------------------------------------------
        -- Address
        addr_A      :in   std_logic_vector(G_ADDRESS_WIDTH - 1 downto 0);
        
        -- Write signal
        write       :in   std_logic;
        
        -- Data input
        data_in     :in   std_logic_vector(G_WORD_WIDTH - 1 downto 0);

        ------------------------------------------------------------------------   
        -- Port B - Data output
        ------------------------------------------------------------------------
        -- Address
        addr_B      :in   std_logic_vector(G_ADDRESS_WIDTH - 1 downto 0);
        
        -- Data output
        data_out    :out  std_logic_vector(G_WORD_WIDTH - 1 downto 0)
    );
end entity;

architecture rtl of inf_RAM_wrapper is

    ----------------------------------------------------------------------------
    -- Memory definition
    ----------------------------------------------------------------------------
    type memory_type is array(0 to G_DEPTH - 1) of
            std_logic_vector(G_WORD_WIDTH - 1 downto 0);
    signal ram_memory        :     memory_type;

    signal int_read_data     :     std_logic_vector(G_WORD_WIDTH - 1 downto 0);

begin

    ----------------------------------------------------------------------------
    -- Memory Write access process 
    ----------------------------------------------------------------------------
    ram_write_process : process(res_n, clk_sys)
    begin
        if (rising_edge(clk_sys)) then
            if (write = '1') then
                ram_memory(to_integer(unsigned(addr_A))) <= data_in;
            end if;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Memory read access
    ----------------------------------------------------------------------------
    int_read_data <= ram_memory(to_integer(unsigned(addr_B)));

    -- Synchronous read
    sync_read_gen : if (G_SYNC_READ) generate
        ram_read_process : process(res_n, clk_sys)
        begin
            if (res_n = G_RESET_POLARITY) then                        
                data_out <= (OTHERS => '0');
            elsif (rising_edge(clk_sys)) then
                data_out <= int_read_data;
            end if;
        end process;
    end generate;

    -- Asynchronous read
    async_read_gen : if (not G_SYNC_READ) generate
        data_out <= int_read_data;
    end generate;


    ----------------------------------------------------------------------------
    -- Assertions on size
    ----------------------------------------------------------------------------
    assert ((G_WORD_WIDTH = 8) or
            (G_WORD_WIDTH = 16) or
            (G_WORD_WIDTH = 32) or
            (G_WORD_WIDTH = 64) or
            (G_WORD_WIDTH = 128))
    report "Unsupported inferred RAM word width! " &
           "Only 8, 16, 32, 64 and 128 are allowed!"
        severity failure;

end architecture;
