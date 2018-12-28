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
--    Bit stream generator. Generates custom bit sequence. Input bit sequence
--    defined as list of <length> <value> where length is number of clock
--    cycles that <value> should last.
--    
--------------------------------------------------------------------------------
-- Revision History:
--    21.6.2018   First Implementation - Ondrej Ille
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use work.CANtestLib.all;
use work.can_constants.all;

entity bit_generator is
    generic (
        constant prescaler  :        natural := 1;
        constant invert     :        boolean := true
    );
    port (
        signal bit_seq      : in     bit_seq_type;
        signal clk_sys      : in     std_logic;

        -- Control signals
        signal run          : in     boolean := false;
        signal done         : buffer boolean := false;

        signal output       : out    std_logic
    );
end entity;


architecture behav of bit_generator is
    
    signal tick             :       std_logic := '0';

    signal counter          :       natural := 0;

begin
    
    ----------------------------------------------------------------------------
    -- Prescaler equal to 1 -> Each clock cycle is processed.
    ----------------------------------------------------------------------------
    presc_1_gen : if (prescaler = 1) generate
        tick <= '1';
    end generate;

    ----------------------------------------------------------------------------
    -- Prescaler higher than 1 -> After N cycles are processed.
    ----------------------------------------------------------------------------
    presc_2_gen : if (prescaler > 1) generate    
        tick_gen : process
        begin
            tick <= '0';
            for i in 1 to prescaler - 1 loop
                wait until rising_edge(clk_sys);
            end loop;
            tick <= '1';
            wait until rising_edge(clk_sys);
        end process;
    end generate;


    ----------------------------------------------------------------------------
    -- Bit generator process
    ----------------------------------------------------------------------------
    bit_gen : process(clk_sys)
        variable index      :   natural := 1;
    begin
        if (rising_edge(clk_sys) and tick = '1') then
            if (run and done = false) then
                
                -- Output driver
                if (invert) then
                    output      <= not bit_seq.bit_values(index);
                else
                    output      <= bit_seq.bit_values(index);
                end if;

                -- Internal counter of equal consecutive values
                if (counter = bit_seq.bit_durations(index)) then
                    if (index < bit_seq.length) then
                        index   := index + 1;
                    else
                        done    <= true;
                    end if;
                    counter     <= 0;
                else
                    counter     <= counter + 1;
                end if;

            else
                index           := 1;
                output          <= RECESSIVE;
                counter         <= 0;
                done            <= false;
            end if;
        end if;
    end process;

end;



