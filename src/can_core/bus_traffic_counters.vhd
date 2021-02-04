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
--  Bus traffic counters.
--
-- Purpose:
--  Counts number of transmitted and received frames on CAN Bus.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer.all;
use ctu_can_fd_rtl.can_constants.all;
use ctu_can_fd_rtl.can_components.all;
use ctu_can_fd_rtl.can_types.all;
use ctu_can_fd_rtl.cmn_lib.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.reduce_lib.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity bus_traffic_counters is
    generic(
        -- Reset polarity
        G_RESET_POLARITY       : std_logic := '0'
    );
    port(
        ------------------------------------------------------------------------
        -- System clock and Asynchronous Reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys                :in   std_logic;
        
        -- Asynchronous Reset
        res_n                  :in   std_logic;

        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- Clear RX Traffic counter (Glitch free)
        clear_rx_ctr           :in   std_logic;
        
        -- Clear TX Traffic counter (Glitch free)
        clear_tx_ctr           :in   std_logic;

        -- Increment TX Traffic Counter
        inc_tx_ctr             :in   std_logic;
        
        -- Increment RX Traffic Counter
        inc_rx_ctr             :in   std_logic;

        ------------------------------------------------------------------------
        -- Counter outputs
        ------------------------------------------------------------------------
        -- TX Traffic counter
        tx_ctr                 :out  std_logic_vector(31 downto 0);
        
        -- RX Traffic counter
        rx_ctr                 :out  std_logic_vector(31 downto 0)
    );
end entity;

architecture rtl of bus_traffic_counters is

    signal tx_ctr_i          :     std_logic_vector(31 downto 0);
    signal rx_ctr_i          :     std_logic_vector(31 downto 0);

    -- Selected value to increment
    signal sel_value           :     unsigned(31 downto 0);

    -- Incremented value by 1
    signal inc_value           :     unsigned(31 downto 0);
    
    -- Reset signals for counters (registered, to avoid glitches)
    signal tx_ctr_rst_d        :     std_logic;
    signal tx_ctr_rst_q        :     std_logic;
    
    signal rx_ctr_rst_d        :     std_logic;
    signal rx_ctr_rst_q        :     std_logic;

begin

    tx_ctr <= tx_ctr_i;
    rx_ctr <= rx_ctr_i;

    -- Multiplexor between TX and RX value to increment
    sel_value <= unsigned(tx_ctr_i) when (inc_tx_ctr = '1') else
                 unsigned(rx_ctr_i);

    -- Incremented value of either TX or RX counter
    inc_value <= sel_value + 1;
    
    ----------------------------------------------------------------------------
    -- Reset registers
    ----------------------------------------------------------------------------
    tx_ctr_rst_d <= G_RESET_POLARITY when (clear_tx_ctr = '1') else
                    (not G_RESET_POLARITY);
    
    rx_ctr_rst_d <= G_RESET_POLARITY when (clear_rx_ctr = '1') else
                    (not G_RESET_POLARITY);                
    
    tx_ctr_res_inst : dff_arst
    generic map(
        G_RESET_POLARITY   => G_RESET_POLARITY,
        
        -- Reset to the same value as is polarity of reset so that other DFFs
        -- which are reset by output of this one will be reset too!
        G_RST_VAL          => G_RESET_POLARITY
    )
    port map(
        arst               => res_n,                -- IN
        clk                => clk_sys,              -- IN
        input              => tx_ctr_rst_d,         -- IN

        output             => tx_ctr_rst_q          -- OUT
    );
    
    rx_ctr_res_inst : dff_arst
    generic map(
        G_RESET_POLARITY   => G_RESET_POLARITY,
        
        -- Reset to the same value as is polarity of reset so that other DFFs
        -- which are reset by output of this one will be reset too!
        G_RST_VAL          => G_RESET_POLARITY
    )
    port map(
        arst               => res_n,                -- IN
        clk                => clk_sys,              -- IN
        input              => rx_ctr_rst_d,         -- IN
        
        output             => rx_ctr_rst_q          -- OUT
    );

    ----------------------------------------------------------------------------
    -- TX Counter register
    ----------------------------------------------------------------------------
    tx_ctr_proc : process(clk_sys, tx_ctr_rst_q)
    begin
        if (tx_ctr_rst_q = G_RESET_POLARITY) then
            tx_ctr_i        <= (OTHERS => '0');

        elsif rising_edge(clk_sys) then
            if (inc_tx_ctr = '1') then
                tx_ctr_i <= std_logic_vector(inc_value);
            end if;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- RX Counter register
    ----------------------------------------------------------------------------
    rx_ctr_proc : process(clk_sys, rx_ctr_rst_q)
    begin
        if (rx_ctr_rst_q = G_RESET_POLARITY) then
            rx_ctr_i        <= (OTHERS => '0');

        elsif rising_edge(clk_sys) then
            if (inc_rx_ctr = '1') then
                rx_ctr_i <= std_logic_vector(inc_value);
            end if;
        end if;
    end process;

    -- <RELEASE_OFF>
    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);
    
    -- psl no_simul_inc_tx_rx_asrt : assert never
    -- (inc_tx_ctr = '1' and inc_rx_ctr = '1')
    -- report "Simultaneous increment of TX and RX error traffic counter"
    -- severity error;

    -- <RELEASE_ON>
end architecture;
