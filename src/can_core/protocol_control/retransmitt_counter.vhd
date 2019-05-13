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
--  Retransmitt counter.
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
use work.reduce_lib.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

entity retransmitt_counter is
    generic(
        -- Reset polarity
        G_RESET_POLARITY        :     std_logic := '0';
        
        -- Width of Retransmitt limit counter
        G_RETR_LIM_CTR_WIDTH    :     natural := 4 
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys        :in   std_logic;

        -- Asynchronous reset
        res_n          :in   std_logic;

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- Selected TXT Buffer changed in comparison to previous transmission
        txtb_changed   :in   std_logic;

        -- Clear the counter
        retr_ctr_clear :in   std_logic;
        
        -- Increment the counter by 1
        retr_ctr_add   :in   std_logic;
        
        -- Retransmitt limit
        retr_limit     :in   std_logic_vector(G_RETR_LIM_CTR_WIDTH - 1 downto 0);

        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- Retransmitt limit was reached
        retr_limit_reached  :out  std_logic
    );
end entity;

architecture rtl of retransmitt_counter is

    -- Retransmitt limit counter
    signal retr_ctr_d : unsigned(G_RETR_LIM_CTR_WIDTH - 1 downto 0);
    signal retr_ctr_q : unsigned(G_RETR_LIM_CTR_WIDTH - 1 downto 0);

    -- Clock enable
    signal retr_ctr_ce : std_logic;
    
begin

    -- Next value
    retr_ctr_d <= (retr_ctr_q + 1) when (retr_ctr_add = '1') else
                   (OTHERS => '0') when (retr_ctr_clear = '1' or 
                                         txtb_changed = '1') else
                   retr_ctr_q;
                 
    -- Clock enable
    retr_ctr_ce <= '1' when (txtb_changed = '1' or
                             retr_ctr_clear = '1' or
                             retr_ctr_add = '1')
                       else
                   '0';

    ---------------------------------------------------------------------------
    -- Counter register
    ---------------------------------------------------------------------------                   
    retr_ctr_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = G_RESET_POLARITY) then
            retr_ctr_q <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then
            if (retr_ctr_ce = '1') then
                retr_ctr_q <= retr_ctr_d;
            end if;
        end if;
    end process;
    
    -- Retransmitt limit reached indication
    retr_limit_reached <= '1' when (unsigned(retr_limit) = retr_ctr_q) else
                          '0';

    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    
    -- psl default clock is rising_edge(clk_sys);

    -- psl retr_ctr_simul_set_and_clear_asrt : assert never
    --  (retr_ctr_add = '1' and retr_ctr_clear = '1');
    -- report "Retransmitt counter, simultaneous increment and clear!"
    -- severity error;
    
    -- psl_retr_ctr_no_overflow : assert never
    --  (retr_limit_reached = '1' and retr_ctr_add = '1')
    -- report "Retransmitt counter overflow"
    -- severity error;
    
end architecture;
