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
-- Purpose:
--    Delay a signal by non-static time.
--    Maintains a FIFO of (time&value) of event (change) on input signal
--    and replays it on the delayed signal with the specified delay.
--------------------------------------------------------------------------------
-- Revision History:
--    February 2018   First Implementation - Martin Jerabek
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

entity signal_delayer_vec is
    generic (
        NSAMPLES : positive;
        DWIDTH : positive
    );
    port (
        input : in std_logic_vector(DWIDTH-1 downto 0);
        delayed : out std_logic_vector(DWIDTH-1 downto 0);
        delay : in time
    );
end entity;

architecture tb of signal_delayer_vec is
    type data_type is array(0 to NSAMPLES-1) of time;
    type dataval_type is array(0 to NSAMPLES-1) of std_logic_vector(DWIDTH-1 downto 0);
    signal first : boolean := true;

    signal nonempty : boolean;
    signal pop : boolean;-- sensitive to edge, not level!
    signal top : time;
    signal top_val : std_logic_vector(DWIDTH-1 downto 0);
begin
    p_fifo: process
        variable data : data_type;
        variable dataval : dataval_type;
        variable rdidx : natural := 0;
        variable wridx : natural := 0;
    begin
        if first then
            first <= false;
            top_val <= input;
        end if;
        wait until (input'event or pop'event);
        if input'event then
            assert (wridx - rdidx) < NSAMPLES report "FIFO full!" severity failure;
            data(wridx mod NSAMPLES) := now;
            dataval(wridx mod NSAMPLES) := input;
            wridx := wridx + 1;
        elsif pop'event then
            assert (wridx - rdidx) > 0 report "FIFO empty!" severity failure;
            top_val <= dataval(rdidx mod NSAMPLES);
            rdidx := rdidx + 1;
        end if;
        top <= data(rdidx mod NSAMPLES);
        nonempty <= (wridx - rdidx) > 0;
        wait for 0 ns;
    end process;

    p_delay: process
        variable towait : time;
        variable first : boolean := true;
    begin
        if delay < 0 ns then
            wait until delay >= 0 ns;
        end if;
        if first then
            first := false;
            delayed <= input;
        end if;
        if not nonempty then
            wait until nonempty;
        end if;
        towait := top + delay - now;
        --report "Waiting for " & time'image(towait);
        wait for towait;
        delayed <= top_val;
        if not nonempty then
            wait until nonempty;
        end if;
        pop <= not pop;
    end process;
end;

library ieee;
use ieee.std_logic_1164.all;

entity signal_delayer is
    generic (
        NSAMPLES : positive
    );
    port (
        input : in std_logic;
        delayed : out std_logic;
        delay : in time
    );
end entity;

architecture tb of signal_delayer is
    signal input_v, delayed_v : std_logic_vector(0 downto 0);
begin
    i_sdv: entity work.signal_delayer_vec
    generic map (
        NSAMPLES => NSAMPLES,
        DWIDTH => 1
    )
    port map (
        input => input_v,
        delayed => delayed_v,
        delay => delay
    );
    input_v <= (0 => input);
    delayed <= delayed_v(0);
end architecture;
