--------------------------------------------------------------------------------
--
-- CTU CAN FD IP Core
-- Copyright (C) 2021-2023 Ondrej Ille
-- Copyright (C) 2023-     Logic Design Services Ltd.s
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to use, copy, modify, merge, publish, distribute the Component for
-- non-commercial purposes. Using the Component for commercial purposes is
-- forbidden unless previously agreed with Copyright holder.
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
-- -------------------------------------------------------------------------------
--
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2020 MIT License
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
-- Module:
--  Range Filter for CAN identifiers.
--
-- Purpose:
--  Filters out CAN identifier based on its decimal value. A CAN Identifier
--  passes the filter when decimal value of Identifier is Higher or Equal than
--  Lower thresholds and Lower or Equal than Higher threshold.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity ff_range_filter is
    generic(
        -- Filter width
        G_WIDTH             :   natural;

        -- Filter presence
        G_IS_PRESENT        :   boolean
    );
    port(
        -- Upper threshold of a filter
        filter_upp_th       : in  std_logic_vector(G_WIDTH - 1 downto 0);

        -- Lower threshold of a filter
        filter_low_th       : in  std_logic_vector(G_WIDTH - 1 downto 0);

        -- Filter input
        filter_input        : in  std_logic_vector(G_WIDTH - 1 downto 0);

        -- Filter enable (output is stuck at zero when disabled)
        enable              : in  std_logic;

        -- Filter output
        valid               : out std_logic
    );
end entity;

architecture rtl of ff_range_filter is

    -- Upper and lower threshold converted to unsigned values
    signal upper_th_dec : natural range 0 to (2 ** G_WIDTH - 1);
    signal lower_th_dec : natural range 0 to (2 ** G_WIDTH - 1);

    -- Filter input converted to unsigned value
    signal value_dec    : natural range 0 to (2 ** G_WIDTH - 1);

    procedure ID_reg_to_decimal(
        signal ID_reg   : in    std_logic_vector(28 downto 0);
        signal ID_dec   : out   natural range 0 to (2 ** 29 - 1)
    ) is
        variable base : std_logic_vector(10 downto 0);
        variable ext  : std_logic_vector(17 downto 0);
        variable conc : std_logic_vector(28 downto 0);
    begin
        base   := ID_reg(IDENTIFIER_BASE_H downto IDENTIFIER_BASE_L);
        ext    := ID_reg(IDENTIFIER_EXT_H downto IDENTIFIER_EXT_L);
        conc   := base&ext;
        ID_dec <= to_integer(unsigned(conc));
    end procedure ID_reg_to_decimal;

begin

    -- Conversion procedures
    ID_reg_to_decimal(filter_input, value_dec);

    ID_reg_to_decimal(filter_upp_th, upper_th_dec);
    ID_reg_to_decimal(filter_low_th, lower_th_dec);

    -- Filter implementation
    g_filt_pos : if (G_IS_PRESENT = true) generate
        valid  <= '1' when ((value_dec <= upper_th_dec) and
                            (value_dec >= lower_th_dec) and
                            (enable = '1'))
                      else
                  '0';
    end generate;

    g_filt_neg : if (G_IS_PRESENT = false) generate
        valid <= '0';
    end generate;

end architecture;
