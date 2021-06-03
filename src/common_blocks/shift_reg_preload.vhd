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
--  Shift register with asynchronous reset and synchronous pre-load.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

entity shift_reg_preload is
    generic (
        -- Reset polarity
        G_RESET_POLARITY     :       std_logic;
        
        -- Reset value
        G_RESET_VALUE        :       std_logic_vector;
        
        -- Shift register width
        G_WIDTH              :       natural
    );
    port (
        -----------------------------------------------------------------------
        -- Clock and reset
        -----------------------------------------------------------------------
        clk                  : in    std_logic;
        res_n                : in    std_logic;

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- Pre-load shift register
        preload              : in    std_logic;

        -- Value to be pre-load to the shift register
        preload_val          : in    std_logic_vector(G_WIDTH - 1 downto 0);

        -- Enable for shift register. When enabled, shifted each clock, when
        -- disabled, register keeps its state.
        enable               : in    std_logic;

        -- Input to a shift register 
        input                : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- Shift register value
        reg_stat             : out   std_logic_vector(G_WIDTH - 1 downto 0);

        -- Shift register output
        output               : out   std_logic
    );
end shift_reg_preload;

architecture rtl of shift_reg_preload is

    -- Internal shift register DFFs
    signal shift_regs               :  std_logic_vector(G_WIDTH - 1 downto 0);

    -- Combinational next value of shift register
    signal next_shift_reg_val       :  std_logic_vector(G_WIDTH - 1 downto 0);
begin

    ---------------------------------------------------------------------------
    -- Calculation of next shift register value
    ---------------------------------------------------------------------------
    next_shift_reg_val  <= shift_regs(G_WIDTH - 2 downto 0) & input;
    output              <= shift_regs(G_WIDTH - 1);

    ---------------------------------------------------------------------------
    -- Implementation of a shift register
    ---------------------------------------------------------------------------
    shift_down_proc : process (res_n, clk)
    begin
        if (res_n = G_RESET_POLARITY) then
            shift_regs <= G_RESET_VALUE;

        elsif (rising_edge(clk)) then
            if (preload = '1') then
                shift_regs <= preload_val;
            elsif (enable = '1') then
                shift_regs <= next_shift_reg_val;
            end if;
        end if;
    end process;


    ---------------------------------------------------------------------------
    -- Propagation of shift register to the outputs
    ---------------------------------------------------------------------------
    reg_stat <= shift_regs;

    ---------------------------------------------------------------------------
    -- Assertion for correct length of reset value
    ---------------------------------------------------------------------------
    assert (G_RESET_VALUE'length = G_WIDTH) report "Invalid length of shift " &
         "register reset value" severity error;

end rtl;
