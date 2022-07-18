--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core 
-- Copyright (C) 2021-present Ondrej Ille
-- 
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to use, copy, modify, merge, publish, distribute the Component for
-- educational, research, evaluation, self-interest purposes. Using the
-- Component for commercial purposes is forbidden unless previously agreed with
-- Copyright holder.
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
--  Register which is used to drive asynchronous reset of another flip-flop.
--
--  Output is gated into inactive value in scan mode.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.can_constants_pkg.all;

entity rst_reg is
    generic (
        G_RESET_POLARITY    : std_logic := '0'
    );
    port (
        -----------------------------------------------------------------------
        -- Clock and Reset
        -----------------------------------------------------------------------
        clk         : in    std_logic;
        arst        : in    std_logic;

        -----------------------------------------------------------------------
        -- Flip flop input / output
        -----------------------------------------------------------------------
        d           : in    std_logic;
        q           : out   std_logic;

        -----------------------------------------------------------------------
        -- Scan mode control
        -----------------------------------------------------------------------
        scan_enable : in    std_logic
    );
end rst_reg;

architecture rtl of rst_reg is
    
    signal q_i : std_logic;

    constant C_NOT_RESET_POLARITY : std_logic := not G_RESET_POLARITY;

begin

    rx_shift_res_reg_inst : entity ctu_can_fd_rtl.dff_arst
    generic map(
        G_RESET_POLARITY   => G_RESET_POLARITY,

        -- Reset to the same value as is polarity of reset so that other DFFs
        -- which are reset by output of this one will be reset too!
        G_RST_VAL          => G_RESET_POLARITY
    )
    port map(
        arst               => arst,         -- IN
        clk                => clk,          -- IN
        reg_d              => d,            -- IN

        reg_q              => q_i           -- OUT
    );

    ---------------------------------------------------------------------------
    -- Registering reset to avoid glitches
    ---------------------------------------------------------------------------
    mux2_res_tst_inst : entity ctu_can_fd_rtl.mux2
    port map(
        a                  => q_i, 
        b                  => C_NOT_RESET_POLARITY,
        sel                => scan_enable,

        -- Output
        z                  => q
    );

end rtl;