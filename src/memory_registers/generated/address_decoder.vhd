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
--   Generic address decoder! 
--------------------------------------------------------------------------------
-- Revision History:
--    14.10.2018   Created file
--    07.12.2018   Added enable signal. Active only when enable is in logic 1,
--                 otherwise disabled.
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;

entity address_decoder is
    generic(

        -- Width of address input
        constant address_width         :     natural;

        -- Number of address entries to decode
        constant address_entries       :     natural;

        -- Addresses to be decoded joined to single address vector. This is
        -- beneficial since there can be gaps in addresses between extra logic!
        constant addr_vect             :     std_logic_vector;

        -- Choose betweed registered/ non-registered output
        constant registered_out        :     boolean := false;

        -- Reset polarity
        constant reset_polarity        :     std_logic := '0'
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
        -- Enable input
        ------------------------------------------------------------------------
        signal enable                 :in   std_logic;

        ------------------------------------------------------------------------
        -- Output, one-hot coded. In logic 1 for each valid address
        ------------------------------------------------------------------------
        signal addr_dec               :out  std_logic_vector(address_entries - 1 downto 0)
    );

end entity address_decoder;


architecture rtl of address_decoder is

    -- Internal one-hot coded signal of address decoder
    signal addr_dec_i                 :   std_logic_vector(
                                                address_entries - 1 downto 0);
    
    -- Address after masking by enable input
    signal addr_dec_enabled_i          :   std_logic_vector(
                                                address_entries - 1 downto 0);

begin

    ---------------------------------------------------------------------------
    -- Combinational Address decoder
    ---------------------------------------------------------------------------
    addr_dec_gen : for i in 0 to address_entries - 1 generate
        constant l_ind : natural := address_width * i;
        constant h_ind : natural := (address_width * (i + 1)) - 1;
    begin
        addr_dec_i(i) <= '1' when (address = addr_vect(h_ind downto l_ind))
                             else
                         '0';
    end generate addr_dec_gen;


    ---------------------------------------------------------------------------
    -- Address decoder enabled / disabled - masking
    ---------------------------------------------------------------------------
    addr_dec_enabled_i <= addr_dec_i when (enable = '1') else
                          (OTHERS => '0');


    ---------------------------------------------------------------------------
    -- Registering / Not-registering output
    ---------------------------------------------------------------------------
    addr_dec_reg_true_gen : if (registered_out) generate
        addr_dec_reg_proc : process(res_n, clk_sys)
        begin
            if (res_n = reset_polarity) then
                addr_dec <= (OTHERS => '0');

            elsif (rising_edge(clk_sys)) then
                addr_dec <= addr_dec_enabled_i;

            end if;
        end process;
    end generate addr_dec_reg_true_gen;

    addr_dec_reg_false_gen : if (not registered_out) generate
        addr_dec <= addr_dec_enabled_i;
    end generate addr_dec_reg_false_gen;


    ---------------------------------------------------------------------------
    -- Check that input vector length is correct.
    ---------------------------------------------------------------------------    
    assert (addr_vect'length = address_width * address_entries)
        report "Invalid length of address vector: " &
                integer'image(addr_vect'length) &
               " Length should be: " &
                integer'image(address_width * address_entries)
        severity failure;
    
end architecture;
