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
--  Template with entity with of basic memory interface.
--  
--  WARNING:
--    Ports and generics must be declared on single line for parser to work
--    properly
--------------------------------------------------------------------------------
-- Revision History:
--    03.11.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;

entity memory_bus_template is
    generic(
        -- Width of Data input
        constant data_width           :     natural := 32;

        -- Width of Address input
        constant address_width        :     natural := 8;

        -- Registered read output. When true, data are returned combinationally
        -- in the same clock cycle, when false data are one clock delayed
        constant registered_read      :     boolean := true;

        -- Clear / Keep r_data signal after read. When true, data are cleared.
        -- When false, data are kept.
        constant clear_read_data      :     boolean := true;

        -- Reset polarity
        constant reset_polarity       :     std_logic := '0'
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and reset
        ------------------------------------------------------------------------
        signal clk_sys                :in   std_logic;
        signal res_n                  :in   std_logic;

        ------------------------------------------------------------------------
        -- Address input
        ------------------------------------------------------------------------
        signal address                :in   std_logic_vector(address_width - 1 downto 0);

        ------------------------------------------------------------------------
        -- Write Data / Read Data
        ------------------------------------------------------------------------
        signal w_data                 :in   std_logic_vector(data_width - 1 downto 0);
        signal r_data                 :out  std_logic_vector(data_width - 1 downto 0);

        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        signal cs                     :in   std_logic;
        signal read                   :in   std_logic;
        signal write                  :in   std_logic;
        signal be                     :in   std_logic_vector(data_width / 8 - 1 downto 0);

        ------------------------------------------------------------------------
        -- Lock signals
        ------------------------------------------------------------------------
        signal lock_1                 :in   std_logic;
        signal lock_2                 :in   std_logic
    );

end entity memory_bus_template;

