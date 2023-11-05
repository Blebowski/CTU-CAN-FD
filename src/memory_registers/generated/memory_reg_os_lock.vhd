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
--   One Shot register with lock support
--------------------------------------------------------------------------------
-- Revision History:
--    14.10.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;

entity memory_reg_os_lock is
    generic(

        -- Width of register data
        constant data_width                 :     natural := 32;

        -- Reset value of register
        constant reset_value                :     std_logic_vector
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

        ------------------------------------------------------------------------
        -- Lock control
        ------------------------------------------------------------------------
        signal lock                   :in   std_logic;

        ------------------------------------------------------------------------
        -- Register outputs
        ------------------------------------------------------------------------
        signal reg_value              :out  std_logic_vector(data_width - 1 downto 0)
    );

end entity memory_reg_os_lock;


architecture rtl of memory_reg_os_lock is

    ---------------------------------------------------------------------------
    -- Create new constants for reset value, implemented etc.
    -- This is important because generic can't be directly passed to if-generate
    -- condition. In instance of the module, generic is filled like so:
    --   reset_value                       => "0000000110011111"
    -- Some tools interpret this vector as 'downto' (GHDL), other tools as 'to'.
    -- (Vivado). We want to keep the module generic therefore we will not give
    -- range (and direction) to generic "reset_value", but we must tell the tool
    -- how to interpret this constant that is passed without "to/downto"!
    -- So we re-declare constants internally and give direction to them.
    -- Tool should assign the constant the same way as it was passed.
    ---------------------------------------------------------------------------
    constant reset_value_i : std_logic_vector(data_width - 1 downto 0) := reset_value;

    -- Register implementation itself!
    signal reg_value_r          :   std_logic_vector(data_width - 1 downto 0);

    -- Write enable
    signal wr_en                :   std_logic;

begin

    ----------------------------------------------------------------------------
    -- -- Write enable
    ----------------------------------------------------------------------------
    wr_en <= write and cs and (not lock);

    ----------------------------------------------------------------------------
    -- Register instance
    ----------------------------------------------------------------------------
    bit_gen : for i in 0 to data_width - 1 generate

        reg_value_r(i) <= data_in(i) when (wr_en = '1')
                                        else
                            reset_value_i(i);

    end generate bit_gen;

    ----------------------------------------------------------------------------
    -- Register to output propagation
    ----------------------------------------------------------------------------
    reg_value <= reg_value_r;

end architecture;
