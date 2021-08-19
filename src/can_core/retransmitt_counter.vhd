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
--  Retransmitt counter.
--
-- Purpose:
--  Counts number of retransmissions on a TX frame from single TXT Buffer.
--  Signals reaching Retransmitt limit to Protocol Control FSM. Cleared when
--  selected TXT Buffer changes or transmission was succesfull. Incremented by
--  1 when Error frame occurs or Arbitration is lost.
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

entity retransmitt_counter is
    generic(        
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
        retr_limit_reached  :out  std_logic;
        
        -- Status of retransmit counter (for observation purpose)
        retr_ctr            :out  std_logic_vector(G_RETR_LIM_CTR_WIDTH - 1 downto 0)
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
        if (res_n = '0') then
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

    -- Counter status propagation to output
    retr_ctr <= std_logic_vector(retr_ctr_q);

    -- <RELEASE_OFF>
    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    
    -- psl default clock is rising_edge(clk_sys);

    -- psl retr_ctr_simul_set_and_clear_asrt : assert never
    --  (retr_ctr_add = '1' and retr_ctr_clear = '1');
    -- report "Retransmitt counter, simultaneous increment and clear!";
    
    -- psl_retr_ctr_no_overflow : assert never
    --  (retr_limit_reached = '1' and retr_ctr_add = '1')
    -- report "Retransmitt counter overflow";
    
    -- <RELEASE_ON>
end architecture;