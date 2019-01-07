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
--  Counters for TX and RX frames. Two 32-bit counters are held. Single adder
--  is used and muxed between these two registers since these counters are
--  never supposed to be incremented simultaneously!
--------------------------------------------------------------------------------
-- Revision History:
--    24.12.2018  Created file
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

entity bus_traffic_counters is
    port(
        ------------------------------------------------------------------------
        -- System clock and Reset
        ------------------------------------------------------------------------
        signal clk_sys                :in   std_logic;
        signal res_n                  :in   std_logic;

        -- Clear signals (used as async. reset, not preload to lower resource
        -- usage)
        signal clear_rx_ctr           :in   std_logic;
        signal clear_tx_ctr           :in   std_logic;

        -- Increment signals (upon sucesfull transmission or reception of frame)
        signal inc_tx_ctr             :in   std_logic;
        signal inc_rx_ctr             :in   std_logic;

        -- Counter outputs
        signal tx_ctr                 :out  std_logic_vector(31 downto 0);
        signal rx_ctr                 :out  std_logic_vector(31 downto 0)
    );
end entity;

architecture rtl of bus_traffic_counters is

    signal tx_ctr_int                :     std_logic_vector(31 downto 0);
    signal rx_ctr_int                :     std_logic_vector(31 downto 0);

    -- Input selector
    signal sel                        :     std_logic;   

    -- Selected value to increment
    signal sel_value                  :     std_logic_vector(31 downto 0);

    -- Incremented value by 1
    signal inc_value                  :     std_logic_vector(31 downto 0);

begin

    tx_ctr <= tx_ctr_int;
    rx_ctr <= rx_ctr_int;

    -- Input selector
    sel <= '1' when (inc_tx_ctr = '1') else
           '0';

    -- Multiplexor between TX and RX value to increment
    sel_value <= tx_ctr_int when (sel = '1') else
                 rx_ctr_int;

    -- Incremented value of either TX or RX counter
    inc_value <= std_logic_vector(to_unsigned(
                    to_integer(unsigned(sel_value)) + 1, sel_value'length));

    ----------------------------------------------------------------------------
    -- TX Counter register
    ----------------------------------------------------------------------------
    tx_ctr_proc : process(clk_sys, res_n)
    begin
        if (res_n = ACT_RESET or clear_tx_ctr = '1') then
            tx_ctr_int        <= (OTHERS => '0');

        elsif rising_edge(clk_sys) then
            if (inc_tx_ctr = '1') then
                tx_ctr_int <= inc_value;
            end if;
        end if;
    end process;


    ----------------------------------------------------------------------------
    -- RX Counter register
    ----------------------------------------------------------------------------
    rx_ctr_proc : process(clk_sys, res_n)
    begin
        if (res_n = ACT_RESET or clear_rx_ctr = '1') then
            rx_ctr_int        <= (OTHERS => '0');

        elsif rising_edge(clk_sys) then
            if (inc_rx_ctr = '1') then
                rx_ctr_int <= inc_value;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Assertion that both inputs are not active at the same time since only
    -- single adder is used. This would corrupt counter values.
    ---------------------------------------------------------------------------
    assert not (inc_tx_ctr = '1' and inc_rx_ctr = '1') report
        "RX frame counter and TX frame counter can't be incremented at once"
        severity error;

end architecture;
