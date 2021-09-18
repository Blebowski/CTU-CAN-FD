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
--  Bit time counters.
--
-- Purpose:
--  Contains counters:
--      1. Time Quanta counter.
--      2. Segment counter.
--
--  Time Quanta counter counts duration of Time quanta segment and provides
--- Time Quanta edge signal. Segment counter counts with granularity of Time 
--  Quanta and measures length of Bit segment (TSEG1, TSEG2).
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;

use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity bit_time_counters is
    generic (
        -- Bit Time counter width
        G_BT_WIDTH        : natural := 8;
        
        -- Baud rate prescaler width
        G_BRP_WIDTH       : natural := 8
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys          : in    std_logic;
        
        -- Asynchrnous reset
        res_n            : in    std_logic;

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- Baud rate Prescaler
        brp              : in    std_logic_vector(G_BRP_WIDTH - 1 downto 0);
        
        -- Time Quanta Counter reset (synchronous)
        tq_reset         : in    std_logic;
        
        -- Bit Time counter reset (synchronous)
        bt_reset         : in    std_logic;
        
        -- Counters enabled
        ctrs_en          : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- Time Quanta edge
        tq_edge         : out   std_logic;
       
        -- Segment counter
        segm_counter      : out   std_logic_vector(G_BT_WIDTH - 1 downto 0)
    );
end entity;

architecture rtl of bit_time_counters is
    
    -- Time Quanta Counter
    signal tq_counter_d         : std_logic_vector(G_BRP_WIDTH - 1 downto 0);
    signal tq_counter_q         : std_logic_vector(G_BRP_WIDTH - 1 downto 0);
    signal tq_counter_ce        : std_logic;
    
    signal tq_counter_allow     : std_logic;

    signal tq_edge_i            : std_logic;

    constant C_TQ_RUN_TH : unsigned(G_BRP_WIDTH - 1 downto 0) :=
        to_unsigned(1, G_BRP_WIDTH);
    
    -- Bit Time counter
    signal segm_counter_d         : std_logic_vector(G_BT_WIDTH - 1 downto 0);
    signal segm_counter_q         : std_logic_vector(G_BT_WIDTH - 1 downto 0);
    signal segm_counter_ce        : std_logic;
    
    constant C_BT_ZEROES : std_logic_vector(G_BT_WIDTH - 1 downto 0) :=
        (OTHERS => '0');

begin

    ---------------------------------------------------------------------------
    -- If prescaler is defined as 0 or 1, there is no need to run the counter!
    -- Run it only when Prescaler is higher than 1! 
    ---------------------------------------------------------------------------
    tq_counter_allow <= '1' when (unsigned(brp) > C_TQ_RUN_TH) else
                        '0';

    tq_counter_ce <= '1' when (tq_counter_allow = '1' and ctrs_en = '1')
                         else
                     '0';

    ---------------------------------------------------------------------------
    -- Time quanta counter next value:
    --  1. Erase when reaching value of prescaler.
    --  2. Erase when re-started.
    --  3. Add 1 ohterwise!
    ---------------------------------------------------------------------------
    tq_counter_d <=
        (OTHERS => '0') when (unsigned(tq_counter_q) = unsigned(brp) - 1)
                        else
        (OTHERS => '0') when (tq_reset = '1')
                        else
        std_logic_vector(unsigned(tq_counter_q) + 1);

    tq_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
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
    tq_edge_i <= '1' when (tq_counter_allow = '0' or 
                           unsigned(tq_counter_q) = unsigned(brp) - 1)
                     else
                 '0';

    ---------------------------------------------------------------------------
    -- Segment counter
    ---------------------------------------------------------------------------
    segm_counter_d <= C_BT_ZEROES when (bt_reset = '1') else
                      std_logic_vector(unsigned(segm_counter_q) + 1);

    segm_counter_ce <= '1' when (bt_reset = '1')
                           else
                       '1' when (tq_edge_i = '1' and ctrs_en = '1')
                           else
                       '0';

    segm_counter_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            segm_counter_q <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then
            if (segm_counter_ce = '1') then
                segm_counter_q <= segm_counter_d;
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Internal signals to output propagation
    ---------------------------------------------------------------------------
    segm_counter <= segm_counter_q;
    tq_edge <= tq_edge_i;

end architecture rtl;