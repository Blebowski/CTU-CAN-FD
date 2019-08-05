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
-- Module:
--  Single Flip-flop with asynchronous reset and clock enable.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

entity dff_arst_ce is
    generic (
        -- Reset polarity
        G_RESET_POLARITY   :       std_logic;
        
        -- Reset value
        G_RST_VAL          :       std_logic
    );    
    port (
        -- Asynchronous reset
        arst               : in    std_logic;
        
        -- Clock
        clk                : in    std_logic;

        -- Data input (D)
        input              : in    std_logic;
        
        -- Clock enable (CE)
        ce                 : in    std_logic;
        
        -- Data output (Q)
        output             : out   std_logic
    );
end dff_arst_ce;

architecture rtl of dff_arst_ce is
begin

    -- DFF process
    dff_proc : process (clk, arst)
    begin
        if (arst = G_RESET_POLARITY) then
            output     <= G_RST_VAL;

        elsif (rising_edge(clk)) then
            if (ce = '1') then
                output <= input;
            end if;
        end if;
    end process;

end rtl;
