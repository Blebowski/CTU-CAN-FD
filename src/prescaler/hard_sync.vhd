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
--  Information processing Time checker.
--
--   Checks length of Information processing time after Sample point between
--   PH1 and PH2. Functions like a half-handshake. When 'ipt_req' comes, interna
--   shift register is preloaded. This shift register shifts each clock cycle
--   and after input value was shifted till the very end, 'ipt_gnt' is set
--   high and remains high till the next 'ipt_req'.
---------------------------------------------------------------------------------------------------------------------------------------------
-- Revision History:
--    03.02.2019   Created file
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

entity ipt_checker is
    generic (
        -- Reset polarity
        reset_polarity : std_logic := '0';

        -- Length of Information processing time in clock cycles.
        ipt_length     : natural := 4
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and reset
        -----------------------------------------------------------------------
        signal clk_sys          : in    std_logic;
        signal res_n            : in    std_logic;

        -----------------------------------------------------------------------
        -- Control interface (Handshake-like)
        -----------------------------------------------------------------------
        signal ipt_req          : in    std_logic;
        signal ipt_gnt          : out   std_logic
    );
end entity;


architecture rtl of ipt_checker is

    -- Internal Shift register
    signal ipt_sr       : std_logic_vector(ipt_length - 1 downto 0);
    signal ipt_sr_nxt   : std_logic_vector(ipt_length - 1 downto 0);

    -- Clock enable for internal shift register. 
    signal ipt_sr_ce  :  std_logic;
    
    -- IPT shift register is empty 
    signal ipt_empty  :  std_logic;
    
    ---------------------------------------------------------------------------    
    -- IPT constants
    ---------------------------------------------------------------------------
    constant IPT_ZEROES : std_logic_vector(ipt_length - 1 downto 0) :=
        (OTHERS => '0');
        
    constant IPT_ONES : std_logic_vector(ipt_length - 1 downto 0) :=
        (OTHERS => '1');
    
begin

    ---------------------------------------------------------------------------
    -- Shift register clock enable. Tick when:
    --  1. There is a request to measure IPT till grant (shift reg preload)
    --  2. Shift register is not empty, shifting is in progress.
    ---------------------------------------------------------------------------
    ipt_sr_ce <= '1' when (ipt_req = '1') else
                 '1' when (ipt_empty = '0') else
                 '0';

    -- Is shift register empty??
    ipt_empty <= '1' when (ipt_sr = IPT_ZEROES) else
                 '0';
    
    ---------------------------------------------------------------------------
    -- IPT Shift register. Next value:
    --  1. Preload upon request
    --  2. Shift to the right
    ---------------------------------------------------------------------------
    ipt_sr_nxt <= IPT_ONES when (ipt_req = '1') else
                  '0' & ipt_sr(ipt_length - 1 downto 1);
    
    
    ---------------------------------------------------------------------------
    -- IPT Shift register. Register assignment
    ---------------------------------------------------------------------------
    ipt_sr_proc : process(res_n, clk_sys)
    begin
        if (res_n = reset_polarity) then
            ipt_sr <= IPT_ZEROES;
        elsif (rising_edge(clk_sys)) then
            if (ipt_sr_ce = '1') then
                ipt_sr <= ip_sr_nxt;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Grant computation. We grant only if the shift register has shifted
    -- till the very end!
    ---------------------------------------------------------------------------
    ipt_gnt <= '1' when (ipt_empty = '1' and ipt_req = '0') else
               '0';
    
    ---------------------------------------------------------------------------
    -- Check that no next IPT request will come till grant to the first
    -- request has been given. This should not occur since there should not
    -- be sample points so close to each other.
    ---------------------------------------------------------------------------
    -- psl ipt_half_handshake_asrt :
    --      assert (not (ipt_empty = '0' and ipt_req = '1'));

end architecture rtl;