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
--  Generic shift register.
--------------------------------------------------------------------------------
-- Revision History:
--    23.11.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

entity shift_reg is
    generic (
        constant reset_polarity     :       std_logic;
        constant reset_value        :       std_logic_vector;
        constant width              :       natural;

        -- When 'true', values are shifted from highest index and output taken
        -- from lowest index. When 'false' value are shifter from lowest index
        -- and output taken from highest index.
        constant shift_down         :       boolean
    );
    port (

        -----------------------------------------------------------------------
        -- Clock and reset
        -----------------------------------------------------------------------
        signal clk                  : in    std_logic;
        signal res_n                : in    std_logic;

        -- Input to a shift register        
        signal input                : in    std_logic;

        -- Enable for shift register. When enabled, shifted each clock, when
        -- disabled, register keeps its state.
        signal enable               : in    std_logic;

        -- Register parallel output
        signal reg_stat             : out   std_logic_vector(width - 1 downto 0);

        -- Register output
        signal output               : out   std_logic
    );
end shift_reg;

architecture rtl of shift_reg is

    -- Internal shift register DFFs
    signal shift_regs               :       std_logic_vector(width - 1 downto 0);

    -- Combinational next value of shift register
    signal next_shift_reg_val       :       std_logic_vector(width - 1 downto 0);
begin

    ---------------------------------------------------------------------------
    -- Calculation of next shift register value
    ---------------------------------------------------------------------------
    shift_down_gen : if (shift_down) generate
        next_shift_reg_val  <= input & shift_regs(width - 1 downto 1);
        output              <= shift_regs(0);
    end generate shift_down_gen;

    shift_up_gen : if (not shift_down) generate
        next_shift_reg_val  <= shift_regs(width - 2 downto 0) & input;
        output              <= shift_regs(width - 1);
    end generate shift_up_gen;


    ---------------------------------------------------------------------------
    -- Implementation of a shift register
    ---------------------------------------------------------------------------
    shift_down_proc : process (clk)
    begin
        if (res_n = reset_polarity) then
            shift_regs <= reset_value;

        elsif (rising_edge(clk)) then
            if (enable = '1') then
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
    assert (reset_value'length = width) report "Invalid length of shift " &
         "register reset value" severity error;

end rtl;
