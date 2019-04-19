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
--  Range Filter for CAN identifiers. Interprets input value as decimal value
--  and compares if it is in range given by upper and lower threshold.
--------------------------------------------------------------------------------
-- Revision History:
--    14.11.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library work;
use work.id_transfer.all;
use work.can_constants.all;
use work.can_components.all;
use work.can_types.all;
use work.cmn_lib.all;
use work.drv_stat_pkg.all;
use work.endian_swap.all;
use work.reduce_lib.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

entity range_filter is
    generic(
        -- Filter width
        G_WIDTH              :   natural;

        -- Filter presence
        G_IS_PRESENT         :   boolean        
    );
    port(
        -- Upper threshold of a filter
        g_filter_upp_th      : in    std_logic_vector(G_WIDTH - 1 downto 0);

        -- Lower threshold of a filter
        g_filter_low_th      : in    std_logic_vector(G_WIDTH - 1 downto 0);

        -- Filter input
        g_filter_input       : in    std_logic_vector(G_WIDTH - 1 downto 0);

        -- Filter enable (output is stuck at zero when disabled)
        g_enable             : in    std_logic;

        -- Filter output
        g_valid              : out   std_logic
    );
end entity;
  
architecture rtl of range_filter is

    -- Upper and lower threshold converted to unsigned values
    signal upper_th_dec             :   natural range 0 to (2 ** G_WIDTH - 1);
    signal lower_th_dec             :   natural range 0 to (2 ** G_WIDTH - 1);

    -- Filter input converted to unsigned value
    signal value_dec                :   natural range 0 to (2 ** G_WIDTH - 1);

begin

    -- Conversion procedures
    ID_reg_to_decimal(filter_input, value_dec);

    ID_reg_to_decimal(filter_upp_th, upper_th_dec);
    ID_reg_to_decimal(filter_low_th, lower_th_dec);

    -- Filter implementation
    gen_filt_pos : if (G_IS_PRESENT = true) generate
        valid  <= '1' when ((value_dec <= upper_th_dec) and
                            (value_dec >= lower_th_dec) and
                            (enable = '1'))
                      else
                  '0';
    end generate;
  
    gen_filtRan_neg : if (G_IS_PRESENT = false) generate
        valid <= '0';
    end generate;

end architecture;