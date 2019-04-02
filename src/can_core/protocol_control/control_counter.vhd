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
--  Control counter.
--------------------------------------------------------------------------------
-- Revision History:
--    29.3.2019   Created file
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

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

entity control_counter is
    generic(
        -- Reset polarity
        G_RESET_POLARITY        :     std_logic := '0';
        
        -- Width of control counter
        G_CTRL_CTR_WIDTH        :     natural := 9 
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys         :in   std_logic;

        -- Asynchronous reset
        res_n           :in   std_logic;

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- RX Trigger (Decrements the counter)
        rx_trigger            :in   std_logic;

        -- Control counter counting is enabled
        ctrl_ctr_ena          :in   std_logic;

        -- Pre-load control counter
        ctrl_ctr_pload        :in   std_logic;
  
        -- Pre-load value for control counter
        ctrl_ctr_pload_val    :in   std_logic_vector(G_CTRL_CTR_WIDTH - 1 downto 0);

        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- Control counter is equal to zero
        ctrl_ctr_zero         :out std_logic;
        
        -- Control counter is equal to one
        ctrl_ctr_one          :out std_logic;

        -- Control counter is divisible by 8 (aligned to byte)
        ctrl_ctr_mod_8        :out std_logic
    );
end entity;

architecture rtl of control_counter is

    -- Control counter
    signal ctrl_ctr_d : unsigned(G_CTRL_CTR_WIDTH - 1 downto 0);
    signal ctrl_ctr_q : unsigned(G_CTRL_CTR_WIDTH - 1 downto 0);

    -- Clock enable
    signal ctrl_ctr_ce : std_logic;

begin

    -- Next value
    ctrl_ctr_d <= ctrl_ctr_pload_val when (ctrl_ctr_pload = '1') else
                  (ctrl_ctr_q - 1)   when (rx_trigger = '1') else
                  ctrl_ctr_q;
                 
    -- Clock enable
    ctrl_ctr_ce <= '1' when (rx_trigger = '1' and ctrl_ctr_ena = '1') else
                   '1' when (ctrl_ctr_pload = '1') else
                   '0';

    ctrl_ctr_zero <= '1' when (ctrl_ctr_q = 0) else
                     '0';

    ctrl_ctr_one <= '1' when (ctrl_ctr_q = 1) else
                    '0';

    ctrl_ctr_mod_8 <= '1' when (ctrl_ctr_q(2 downto 0) = "000")
                          else
                      '0';

    ---------------------------------------------------------------------------
    -- Counter register
    ---------------------------------------------------------------------------                   
    retr_ctr_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = G_RESET_POLARITY) then
            ctrl_ctr_q <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then
            if (ctrl_ctr_ce = '1') then
                ctrl_ctr_q <= ctrl_ctr_d;
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);

    -- psl retr_ctr_simul_set_and_clear_asrt : assert never
    --  (rx_trigger = '1' and ctrl_ctr_ena = '1' and ctrl_ctr_pload = '0')
    -- report "Control counter underflow!"
    -- severity error;

end architecture;