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
--  Asynchronous reset synchroniser with two DFFs.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

entity rst_sync is
    generic (
        -- Reset polarity
        G_RESET_POLARITY    :       std_logic
    );    
    port (
        -- Clock
        clk                 : in    std_logic;
        
        -- Asynchronous reset
        arst                : in    std_logic;
        
        -- Synchronous reset
        rst                 : out   std_logic
    );
end rst_sync;

architecture rtl of rst_sync is

    -- Synchroniser registers
    signal rff                      :       std_logic;

begin

    -- Reset synchroniser process
    rst_sync_proc : process (clk, arst)
    begin
        if (arst = G_RESET_POLARITY) then
            rff     <= G_RESET_POLARITY;
            rst     <= G_RESET_POLARITY;
        elsif (rising_edge(clk)) then
            rff     <= not (G_RESET_POLARITY);
            rst     <= rff;
        end if;
    end process;

end rtl;
