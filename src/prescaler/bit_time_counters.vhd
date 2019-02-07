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
--  Contains two counters:
--      1. Time Quanta counter.
--      2. Bit time counter.
--
--  Time Quanta counter counts duration of Time quanta segment and provides
--- Time Quanta edge signal.
--  Bit Time counter counts with granularity of Time Quanta and provides value
--  of Bit Time counter to the output.
--
--------------------------------------------------------------------------------
-- Revision History:
--    15.02.2019   Created file
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

entity bit_time_counters is
    generic (
        -- Reset polarity
        reset_polarity  : std_logic := '0';
        
        -- Bit Time counter width
        bt_width        : natural := 8;
        
        -- Time Qunata counter width
        tq_width        : natural := 8
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and reset
        -----------------------------------------------------------------------
        signal clk_sys          : in    std_logic;
        signal res_n            : in    std_logic;

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        
        -- Prescaler value
        signal prescaler        : in    std_logic_vector(tq_width - 1 downto 0);
        
        -- Time Quanta reset (synchronous)
        signal tq_reset         : in    std_logic;
        
        -- Bit Time reset (synchronous)
        signal bt_reset         : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- Time Quanta edge
        signal tq_edge          : out   std_logic;
        
        -- Bit Time counter
        signal bt_counter       : out   std_logic_vector(bt_width - 1 downto 0)
    );
end entity;

architecture rtl of bit_time_counters is
    
    -- Time Quanta Counter
    signal tq_counter_d         : std_logic_vector(tq_width - 1 downto 0);
    signal tq_counter_q         : std_logic_vector(tq_width - 1 downto 0);
    signal tq_counter_ce        : std_logic;

    signal tq_edge_i            : std_logic;

    constant tq_zeroes : std_logic_vector(tq_width - 1 downto 0) :=
        (OTHERS => '0');
    constant tq_run_th : std_logic_vector(tq_width - 1 downto 0) :=
        (0 => '1', OTHERS => '0');
    
    -- Bit Time counter
    signal bt_counter_d         : std_logic_vector(bt_width - 1 downto 0);
    signal bt_counter_q         : std_logic_vector(bt_width - 1 downto 0);
    
    constant bt_zeroes : std_logic_vector(bt_width - 1 downto 0) :=
        (OTHERS => '0');
    constant bt_ones : std_logic_vector(bt_width - 1 downto 0) :=
        (OTHERS => '1');

begin    

    ---------------------------------------------------------------------------
    -- If prescaler is defined as 0 or 1, there is no need to run the counter!
    -- Run it only when Prescaler is higher than 1! 
    ---------------------------------------------------------------------------
    tq_counter_ce <= '1' when (prescaler > tq_run_th) else
                     '0';

    ---------------------------------------------------------------------------
    -- Time quanta counter next value:
    --  1. Erase when reaching value of prescaler.
    --  2. Erase when re-started.
    --  3. Add 1 ohterwise!
    ---------------------------------------------------------------------------
    tq_counter_d <=
        (OTHERS => '0') when (unsigned(tq_counter_q) = unsigned(prescaler) - 1)
                        else
        (OTHERS => '0') when (tq_reset = '1')
                        else
        std_logic_vector(unsigned(tq_counter_q) + 1);

    tq_proc : process(clk_sys, res_n)
    begin
        if (res_n = reset_polarity) then
            tq_counter_q <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then
            if (tq_counter_ce = '1') then
                tq_counter_q <= tq_counter_d;
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Time quanta edge
    ---------------------------------------------------------------------------
    tq_edge_i <= '1' when (tq_counter_ce = '0' or tq_counter_q = tq_zeroes) else
                 '0';
    tq_edge <= tq_edge_i;

    ---------------------------------------------------------------------------
    -- Bit time counter
    ---------------------------------------------------------------------------
    bt_counter_d <= bt_zeroes when (bt_reset = '1') else
                    std_logic_vector(unsigned(bt_counter_q) + 1);

    bt_counter_proc : process(clk_sys, res_n)
    begin
        if (res_n = reset_polarity) then
            bt_counter_q <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then
            if (tq_edge_i = '1') then
                bt_counter_q <= bt_counter_d;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);
    --
    -- psl no_bt_overflow_asrt : assert never
    --      (bt_counter_q = bt_ones and tq_edge = '1' and bt_reset = '0');

end architecture rtl;