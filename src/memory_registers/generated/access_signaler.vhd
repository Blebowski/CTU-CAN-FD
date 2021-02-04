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
--   Access signaller indicating write or read access to a given register.
--   Optional registering of access signalling is present.
--------------------------------------------------------------------------------
-- Revision History:
--    25.10.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;

entity access_signaller is
    generic(

        -- Reset polarity
        constant reset_polarity       :     std_logic := '0';

        -- Width of memory register whose access is being signalled
        constant data_width           :     natural := 32;

        -- Signal read
        constant read_signalling      :     boolean := false;

        -- Signal write
        constant write_signalling     :     boolean := false;

        -- Registered read sigalling
        constant read_signalling_reg  :     boolean := false;

        -- Registered write sigalling
        constant write_signalling_reg :     boolean := false
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and reset
        ------------------------------------------------------------------------
        signal clk_sys                :in   std_logic;
        signal res_n                  :in   std_logic;

        ------------------------------------------------------------------------
        -- Chip select input (from address decoder)
        ------------------------------------------------------------------------
        signal cs                     :in   std_logic;

        ------------------------------------------------------------------------
        -- Memory access signals
        ------------------------------------------------------------------------
        signal read                   :in   std_logic;
        signal write                  :in   std_logic;
        signal be                     :in   std_logic_vector(data_width / 8 - 1 downto 0);

        ------------------------------------------------------------------------
        -- Signalling outputs
        ------------------------------------------------------------------------
        signal write_signal           :out  std_logic;
        signal read_signal            :out  std_logic
    );

end entity access_signaller;


architecture rtl of access_signaller is

    -- Byte enable zeros
    constant BE_ZEROES : std_logic_vector(data_width / 8 - 1 downto 0) := (OTHERS => '0');

    -- Any of byte enables for given register is active
    signal be_active   : std_logic;

    -- Internal write / read signals
    signal access_in            : std_logic_vector(1 downto 0);
    signal access_active        : std_logic_vector(1 downto 0);
    signal access_active_reg    : std_logic_vector(1 downto 0);

    -- Signalling configuration
    type t_access_cfg is array (0 to 1) of boolean;
    constant access_cfg         : t_access_cfg := (read_signalling, write_signalling);
    constant acces_reg_cfg      : t_access_cfg := (read_signalling_reg, write_signalling_reg);

begin

    ---------------------------------------------------------------------------
    -- Byte enable detection. At least one byte enable is non-zero
    ---------------------------------------------------------------------------
    be_active    <= '1' when (be /= BE_ZEROES)
                        else
                    '0';

    access_in(0) <= read and cs;
    access_in(1) <= write and cs;

    ---------------------------------------------------------------------------
    -- Write / Read signalling
    ---------------------------------------------------------------------------
    signalling_gen : for i in 0 to 1 generate
        -- Signalling present
        signal_pres_gen : if (access_cfg(i)) generate
            access_active(i) <= be_active and access_in(i);
        end generate signal_pres_gen;

        -- Signalling not present
        signal_not_pres_gen : if (not access_cfg(i)) generate
            access_active(i) <= '0';
        end generate signal_not_pres_gen;

        -- Registering signalling
        signal_reg_pres_gen : if (acces_reg_cfg(i)) generate

            res_sign_proc : process(res_n, clk_sys)
            begin
                if (res_n = reset_polarity) then
                    access_active_reg(i) <= '0';
                elsif (rising_edge(clk_sys)) then

                    -----------------------------------------------------------
                    -- Mark signalling to a DFF when it is active. Clear when 
                    -- it was activated. Thisway DFF won't tick every clock 
                    -- cycle but only upon set-clear!
                    -----------------------------------------------------------
                    if (access_active(i) = '1') then
                        access_active_reg(i) <= '1';
                    elsif (access_active_reg(i) = '1') then
                        access_active_reg(i) <= '0';
                    end if;

                end if;
            end process res_sign_proc;

        end generate signal_reg_pres_gen;

        -- Non-registering write signalling
        signal_reg_not_pres_gen : if (not acces_reg_cfg(i)) generate
            access_active_reg(i)   <=  access_active(i);
        end generate signal_reg_not_pres_gen;

    end generate signalling_gen;

    ---------------------------------------------------------------------------
    -- Propagation to outputs
    ---------------------------------------------------------------------------
    read_signal  <= access_active_reg(0);
    write_signal <= access_active_reg(1);


    ---------------------------------------------------------------------------
    -- Assertions on correct input configuration. It is not allowed to
    -- register read or write signalling when read or write signalling itself
    -- is turned off!
    ---------------------------------------------------------------------------
    assert (not (read_signalling = False and read_signalling_reg = True))
        report "Invalid read signalling settings!" severity failure;

    assert (not (write_signalling = False and write_signalling_reg = True))
        report "Invalid write signalling settings!" severity failure;

end architecture;
