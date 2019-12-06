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
--   Common package for register map generator. Contains following components:
--      Address decoder
--      Data multiplexor
--      Memory register 
--      Access signaller
--
--------------------------------------------------------------------------------
-- Revision history:
--  25.11.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
USE ieee.numeric_std.ALL;

package cmn_reg_map_pkg is


--------------------------------------------------------------------------------
-- Address decoder
--------------------------------------------------------------------------------
component address_decoder is
    generic(
        constant address_width         :     natural;
        constant address_entries       :     natural;
        constant addr_vect             :     std_logic_vector;
        constant registered_out        :     boolean := false;
        constant reset_polarity        :     std_logic := '0'
    );
    port(
        signal clk_sys                :in   std_logic;
        signal res_n                  :in   std_logic;
        signal address                :in   std_logic_vector(address_width - 1 downto 0);
        signal enable                 :in   std_logic;
        signal addr_dec               :out  std_logic_vector(address_entries - 1 downto 0)
    );
end component address_decoder;


--------------------------------------------------------------------------------
-- Data multiplexor
--------------------------------------------------------------------------------
component data_mux is
    generic(
        constant data_out_width        :     natural := 32;
        constant data_in_width         :     natural := 256;
        constant sel_width             :     natural := 8;
        constant registered_out        :     boolean := false;
        constant reset_polarity        :     std_logic := '0'
    );
    port(
        signal clk_sys                :in   std_logic;
        signal res_n                  :in   std_logic;
        signal data_selector          :in   std_logic_vector(sel_width - 1 downto 0);
        signal data_in                :in   std_logic_vector(data_in_width - 1 downto 0);
        signal data_mask_n            :in   std_logic_vector(data_out_width - 1 downto 0);        
        signal enable                 :in   std_logic;
        signal data_out               :out  std_logic_vector(data_out_width - 1 downto 0)
    );

end component data_mux;


--------------------------------------------------------------------------------
-- Memory register
--------------------------------------------------------------------------------
component memory_reg is
    generic(
        constant data_width           :     natural := 32;
        constant data_mask            :     std_logic_vector;
        constant reset_polarity       :     std_logic := '0';
        constant reset_value          :     std_logic_vector;
        constant auto_clear           :     std_logic_vector;
        constant is_lockable          :     boolean     
    );
    port(
        signal clk_sys                :in   std_logic;
        signal res_n                  :in   std_logic;
        signal data_in                :in   std_logic_vector(data_width - 1 downto 0);
        signal write                  :in   std_logic;
        signal cs                     :in   std_logic;
        signal w_be                   :in   std_logic_vector(data_width / 8 - 1 downto 0);
        signal reg_value              :out  std_logic_vector(data_width - 1 downto 0);
        signal lock                   :in   std_logic
    );
end component memory_reg;


--------------------------------------------------------------------------------
-- Access signaller
--------------------------------------------------------------------------------
component access_signaller is
    generic(
        constant reset_polarity       :     std_logic := '0';
        constant data_width           :     natural := 32;
        constant read_signalling      :     boolean := false;
        constant write_signalling     :     boolean := false;
        constant read_signalling_reg  :     boolean := false;
        constant write_signalling_reg :     boolean := false
    );
    port(
        signal clk_sys                :in   std_logic;
        signal res_n                  :in   std_logic;
        signal cs                     :in   std_logic;
        signal read                   :in   std_logic;
        signal write                  :in   std_logic;
        signal be                     :in   std_logic_vector(data_width / 8 - 1 downto 0);
        signal write_signal           :out  std_logic;
        signal read_signal            :out  std_logic
    );

end component access_signaller;

end package;
