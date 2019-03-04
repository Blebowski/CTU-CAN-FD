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
--  Bit time FSM.
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

entity bit_time_fsm is
    generic (
        -- Reset polarity
        reset_polarity  : std_logic := '0'
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and reset
        -----------------------------------------------------------------------
        signal clk_sys          : in    std_logic;
        signal res_n            : in    std_logic;

        -----------------------------------------------------------------------
        -- Control interface 
        -----------------------------------------------------------------------
        -- Signalling segment end (either due to re-sync, or reaching expected
        -- length of segment)
        signal segm_end         : in    std_logic;
        signal h_sync_valid     : in    std_logic;

        -- Core is enabled
        signal drv_ena          : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Status signals 
        -----------------------------------------------------------------------
        -- Bit time is in TSEG1
        signal is_tseg1         : out   std_logic;
        
        -- Bit time is in TSEG2
        signal is_tseg2         : out   std_logic;
        
        -- Sample point request (to sample point generator)
        signal sample_req       : out   std_logic;
        
        -- Sync signal request
        signal sync_req         : out   std_logic;
        
        -- Bit time FSM output
        signal bt_FSM_out       : out   bit_time_type
    );
end entity;

architecture rtl of bit_time_fsm is

    -- Bit time FSM
    signal current_state    : bit_time_type;
    signal next_state       : bit_time_type;

    -- Bit time FSM clock enable
    signal bt_fsm_ce        : std_logic;
    
begin

    ----------------------------------------------------------------------------
    -- Next state process (combinational)
    ----------------------------------------------------------------------------
    next_state_proc : process(current_state, h_sync_valid, segm_end, drv_ena)
    begin
        next_state <= current_state;
    
        if (drv_ena = CTU_CAN_DISABLED) then
            next_state <= reset;
        elsif (h_sync_valid = '1') then
            next_state <= tseg1;
        else
            case current_state is
            when tseg1 =>
                if (segm_end = '1') then
                    next_state <= tseg2;
                end if;
            when tseg2 =>
                if (segm_end = '1') then
                    next_state <= tseg1;
                end if;
            when reset =>
                next_state <= tseg1;
            end case;
        end if;
    end process;
    
    -- State register to output propagation
    bt_FSM_out <= current_state;
    
    ----------------------------------------------------------------------------
    -- Current state process (combinational)
    ----------------------------------------------------------------------------
    curr_state_proc : process(current_state, segm_end)
    begin
        -- Default values
        is_tseg1       <= '0';
        is_tseg2       <= '0';
        sample_req     <= '0';
        sync_req       <= '0';
        
        case current_state is
        when reset =>
        when tseg1 =>
            is_tseg1 <= '1';
            if (segm_end = '1') then
                sample_req <= '1';
            end if;
            
        when tseg2 =>
            is_tseg2 <= '1';
            if (segm_end = '1') then
                sync_req <= '1';
            end if;
            
        end case;
    end process;
    
    ----------------------------------------------------------------------------
    -- State register assignment
    ----------------------------------------------------------------------------
    state_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = reset_polarity) then
            current_state <= reset;
        elsif (rising_edge(clk_sys)) then
            if (bt_fsm_ce = '1') then
                current_state <= next_state;
            end if;
        end if;
    end process;
    
    bt_fsm_ce <= '1' when (next_state /= current_state) else
                 '0'; 

end architecture rtl;