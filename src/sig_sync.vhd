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
--  Two Flip-flop asynchronous signal synchroniser. Synthesizes as two DFFs.
--  Simulation behaviour is like so:
--   1. If t_setup and t_hold are not corrupted, two clock cycle delay is
--      introduced.
--   2. If t_setup or t_hold are corrupted, 
--------------------------------------------------------------------------------
-- Revision History:
--    16.11.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

entity sig_sync is
    generic (
        constant t_setup            :       time := 0 ps;
        constant t_hold             :       time := 0 ps;
        constant timing_check       :       boolean := true;
    );
    port (
        signal clk                  : in    std_logic;
        signal async                : in    std_logic;
        signal sync                 : out   std_logic
    );
end rst_sync;

architecture rtl of rst_sync is

    -- Synchroniser registers
    signal rff                      :       std_logic;

begin

    -- Signal synchroniser process.
    rst_sync_proc : process (clk)
    begin
        if (rising_edge(clk)) then
            rff     <= async;
            sync    <= rff;
        end if;
    end process;

    -- Check for timing violations on second DFF
    timing_check_proc : proces
    begin
        
    end process;

end rtl;
