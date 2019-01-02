--------------------------------------------------------------------------------
-- 
-- Register map generation tool
--
-- Copyright (C) 2018 Ondrej Ille <ondrej.ille@gmail.com>
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this SW component and associated documentation files (the "Component"),
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
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Purpose:
--   Definition of a single memory register. Supports optional auto-clear
--   of register in the next cycle. Supports implementation of only some
--   bits within a register.
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
        constant data_mask            :     std_logic_vector(data_width - 1 downto 0);

        -- Reset polarity
        constant reset_polarity       :     std_logic := '0';

        -- Reset value of register
        constant reset_value          :     std_logic_vector(data_width - 1 downto 0);

        -- If given bit of the register should be cleared automatically one
        -- clock cycle after writing.
        constant auto_clear           :     std_logic_vector(data_width - 1 downto 0)
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
        signal data_in                :in   std_logic_vector(data_width - 1 downto 0);
        signal write                  :in   std_logic;
        signal cs                     :in   std_logic;
        signal w_be                   :in   std_logic_vector(data_width / 8 - 1 downto 0);

        ------------------------------------------------------------------------
        -- Register outputs
        ------------------------------------------------------------------------
        signal reg_value              :out  std_logic_vector(data_width - 1 downto 0)
    );
             
end entity memory_reg;


architecture rtl of memory_reg is

    -- Register implementation itself!
    signal reg_value_r          :   std_logic_vector(data_width - 1 downto 0);

    -- Write selector. Indicates that given bit should be written!
    signal wr_select            :   std_logic_vector(data_width / 8 - 1 downto 0);

    -- Expanded write register select with the same width as register
    signal wr_select_expanded   :   std_logic_vector(data_width - 1 downto 0);

begin

    ----------------------------------------------------------------------------
    -- Write selector. Takes "write", "byte enable" and creates write select
    -- for each byte!
    ----------------------------------------------------------------------------    
    wr_sel_gen : for i in 0 to (data_width / 8 - 1) generate
        wr_select(i) <= write and cs and w_be(i);

        -- Expand the signal for each index
        wr_sel_exp_gen : for j in 0 to 7 generate
            wr_select_expanded(i * 8 + j) <= wr_select(i);
        end generate;
    end generate wr_sel_gen;

    ----------------------------------------------------------------------------
    -- Register instance
    ----------------------------------------------------------------------------
    bit_gen : for i in 0 to data_width - 1 generate
    
        ------------------------------------------------------------------------
        -- Register implementation itself
        ------------------------------------------------------------------------
        reg_present_gen : if (data_mask(i) = '1') generate

            reg_access_proc : process(clk_sys, res_n)
            begin
                if (res_n = reset_polarity) then
                    reg_value_r(i)  <= reset_value(i);

                elsif (rising_edge(clk_sys)) then

                    -- Write to the register
                    if (wr_select_expanded(i) = '1') then
                        reg_value_r(i)  <= data_in(i);

                    -- Clear the register if autoclear is set and register is
                    -- set
                    elsif (auto_clear(i) = '1' and reg_value_r(i) = '1') then
                        reg_value_r(i)  <= reset_value(i);
                    end if;

                end if;
            end process;

        end generate reg_present_gen;


        -----------------------------------------------------------------------
        -- Registers which are not present are stuck at reset value
        -----------------------------------------------------------------------
        reg_not_present_gen : if (data_mask(i) = '0') generate
            reg_value_r(i)    <=  reset_value(i);
        end generate reg_not_present_gen;

    end generate bit_gen;

    ----------------------------------------------------------------------------
    -- Register to output propagation
    ----------------------------------------------------------------------------
    reg_value <= reg_value_r;

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
