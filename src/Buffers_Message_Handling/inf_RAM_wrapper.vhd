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
--  RAM memory wrapper intended for use with inferred memories in FPGA
--  technologies. Supports RAM inference in Xilinx and Intel FPGAs.
--  Synchronous dual port memory with shared clock. Port A is used for
--  writes. Port B is used for reads!
--------------------------------------------------------------------------------
-- Revision History:
--    27.9.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;

entity inf_RAM_wrapper is
    generic(

        -- Width of memory word (in bits)
        constant word_width           :     natural := 32;

        -- Memory depth (in words)
        constant depth                :     natural := 32;

        -- Address width (in bits)
        constant address_width        :     natural := 8;

        -- Polarity of reset
        constant reset_polarity       :     std_logic := '1';

        -- RAM content reset upon reset
        constant simulation_reset     :     boolean := true;

        -- Synchronous read
        constant sync_read            :     boolean := true
    );
  port(
        ------------------------------------------------------------------------
        -- Clock and reset
        ------------------------------------------------------------------------
        signal clk_sys                :in   std_logic;
        signal res_n                  :in   std_logic;

        ------------------------------------------------------------------------
        -- Port A - Data input
        ------------------------------------------------------------------------
        signal addr_A                 :in   std_logic_vector(address_width -1
                                                downto 0);
        signal write                  :in   std_logic;
        signal data_in                :in   std_logic_vector(word_width - 1
                                                downto 0);

        ------------------------------------------------------------------------   
        -- Port B - Data output
        ------------------------------------------------------------------------
        signal addr_B                 :in   std_logic_vector(address_width - 1
                                                downto 0);
        signal data_out               :out  std_logic_vector(word_width - 1
                                                downto 0)
    );
             
end entity;


architecture rtl of inf_RAM_wrapper is

    ----------------------------------------------------------------------------
    -- Memory definition
    ----------------------------------------------------------------------------
    type memory_type is array(0 to depth - 1) of
            std_logic_vector(word_width - 1 downto 0);
    signal ram_memory                 :     memory_type;

    signal int_read_data              :     std_logic_vector(word_width - 1
                                                downto 0);

begin

    ----------------------------------------------------------------------------
    -- Memory Write access process 
    ----------------------------------------------------------------------------
    ram_write_process : process(res_n, clk_sys)
    begin
        if (res_n = reset_polarity) then

            -- pragma translate_off
            if (simulation_reset) then
                ram_memory <= (OTHERS => (OTHERS => '0'));
            end if;            
            -- pragma translate_on
           
        elsif (rising_edge(clk_sys)) then

            -- Store the data into the RAM memory
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
    sync_read_gen : if (sync_read) generate
        ram_read_process : process(res_n, clk_sys)
        begin
            if (res_n = reset_polarity) then
                data_out <= (OTHERS => '0');
               
            elsif (rising_edge(clk_sys)) then
                data_out <= int_read_data;
                
            end if;
        end process;
    end generate;

    -- Asynchronous read
    async_read_gen : if (not sync_read) generate
        data_out <= int_read_data;
    end generate;


    ----------------------------------------------------------------------------
    -- Assertions on size
    ----------------------------------------------------------------------------
    assert ((word_width = 8) or
            (word_width = 16) or
            (word_width = 32) or
            (word_width = 64) or
            (word_width = 128))
    report "Unsupported inferred RAM word width! " &
           "Only 8, 16, 32, 64 and 128 are allowed!"
        severity failure;

end architecture;
