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
--  Bit Filter for CAN identifiers. Output is valid if masked filter input 
--  equals masked value to be compared. Output is combinational.
--------------------------------------------------------------------------------
-- Revision History:
--    14.11.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

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

entity bit_filter is
    generic(
        -- Filter width
        constant width              :   natural;

        -- Filter presence
        constant is_present         :   boolean
    );
    port(
        -- Filter mask
        signal filter_mask          : in  std_logic_vector(width - 1 downto 0);

        -- Filter value
        signal filter_value         : in  std_logic_vector(width - 1 downto 0);

        -- Filter input
        signal filter_input         : in  std_logic_vector(width - 1 downto 0);

        -- Filter enable (output is stuck at zero when disabled)
        signal enable               : in  std_logic;

        -- Filter output
        signal valid                : out std_logic
    );
end entity;
  
architecture rtl of bit_filter is

    signal masked_input             :   std_logic_vector(width - 1 downto 0);
    signal masked_value             :   std_logic_vector(width - 1 downto 0);

begin

    masked_input <= filter_input and filter_mask;
    masked_value <= filter_value and filter_mask;

    -- Filter A input frame type filtering 
    gen_filt_pos : if (is_present = true) generate
        valid <= '1' when (masked_input = masked_value) 
                          AND
                          (enable = '1')
                     else
                 '0';
    end generate;
    
    gen_filt_neg : if (is_present = false) generate
        valid <= '0';
    end generate;  
  
end architecture;
