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
--   Definition of a single memory register. Supports optional auto-clear
--   of register in the next cycle. Supports implementation of only some
--   bits within a register. Supports indication of write to the register via
--   pulse on "reg_written" for one clock cycle!
--------------------------------------------------------------------------------
-- Revision History:
--    14.10.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;

entity memory_reg is
    generic(

        -- Width of register data
        constant data_width           :     natural := 32;

        -- Data mask. Each logic 1 indicates present bit, logic 0 indicates
        -- reserved bit in register bits. Reserved bit always returns 0.
        constant data_mask            :     std_logic_vector;

        -- Reset polarity
        constant reset_polarity       :     std_logic := '0';

        -- Reset value of register
        constant reset_value          :     std_logic_vector;

        -- If given bit of the register should be cleared automatically one
        -- clock cycle after writing.
        constant auto_clear           :     std_logic_vector
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and reset
        ------------------------------------------------------------------------
        signal clk_sys                :in   std_logic;
        signal res_n                  :in   std_logic;

        ------------------------------------------------------------------------
        -- Address bus
        ------------------------------------------------------------------------
        signal data_in                :in   std_logic_vector(
                                                data_width - 1 downto 0);
        signal write                  :in   std_logic;
        signal cs                     :in   std_logic;
        signal w_be                   :in   std_logic_vector(
                                                data_width / 8 - 1 downto 0);

        ------------------------------------------------------------------------
        -- Register outputs
        ------------------------------------------------------------------------
        signal reg_value              :out  std_logic_vector(
                                                data_width - 1 downto 0)
    );
             
end entity memory_reg;


architecture rtl of txtBuffer is

    -- Register implementation itself!
    signal reg_value            :   std_logic_vector(data_width - 1 downto 0);

    -- Write selector. Indicates that given bit should be written!
    signal wr_select            :   std_logic_vector(data_width / 8 - 1 downto 0);

begin

    ----------------------------------------------------------------------------
    -- Write selector. Takes "write", "byte enable" and creates write select
    -- for each byte!
    ----------------------------------------------------------------------------    
    for i in 0 to (data_width / 8 - 1) loop
        wr_select(i) <= write and cs and wr_be(i);
    end loop;


    ----------------------------------------------------------------------------
    -- Register instance
    ----------------------------------------------------------------------------
    for i in 0 to data_width - 1 generate
    
        ------------------------------------------------------------------------
        -- Register implementation itself
        ------------------------------------------------------------------------
        reg_present_gen : if (data_mask(i) = '1') generate

            reg_access_proc : process(clk_sys, res_n)
            begin
                if (res_n = reset_polarity) then
                    reg_value(i)  <= reset_value(i)

                elsif (rising_edge(clk_sys)) then

                    -- Write to the register
                    if (wr_select(i / 8) = '1') then
                        reg_value(i)  <= data_in(i)

                    -- Clear the register if autoclear is set and register is
                    -- set
                    elsif (auto_clear(i) = '1' and reg_value(i) = '1') then
                        reg_value(i)  <= reset_value(i);
                    end if;

                end if;
            end process;

        end generate reg_present_gen;


        -----------------------------------------------------------------------
        -- Registers which are not present are stuck at reset value
        -----------------------------------------------------------------------
        reg_not_present_gen : if (data_mask(i) = '0') generate
            reg_value(i)    <=  reset_value(i);
        end generate reg_not_present_gen;

        end if;

    end process;

    -- Propagate to the output
    reg_written <= reg_written_r;


    ----------------------------------------------------------------------------
    -- Monitoring register sizes!
    ----------------------------------------------------------------------------
    assert ((data_width = 8) or
            (data_width = 16) or
            (data_width = 32) or
            (data_width = 64) or
            (data_width = 128))
    report "Unsupported Memory register width! " &
           "Only 8, 16, 32, 64 and 128 are allowed!"
        severity failure;

end architecture;