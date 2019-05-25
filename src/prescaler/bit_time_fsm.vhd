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
--
--  Bit Time FSM has three states:
--      1. Reset
--      2. TSEG1
--      3. TSEG2
--
--  Output of Bit time FSM are SYNC and SAMPLE requests for SYNC and SAMPLE
--  trigger generator.    
--------------------------------------------------------------------------------
-- Revision History:
--    15.02.2019   Created file
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

entity bit_time_fsm is
    generic (
        -- Reset polarity
        G_RESET_POLARITY    : std_logic := '0'
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys             : in    std_logic;
        
        -- Asynchronous reset
        res_n               : in    std_logic;

        -----------------------------------------------------------------------
        -- Control interface 
        -----------------------------------------------------------------------
        -- Segment end (either due to re-sync, or reaching expected length)
        segm_end            : in    std_logic;
        
        -- Hard synchronisation is valid
        h_sync_valid        : in    std_logic;

        -- CTU CAN FD is enabled
        drv_ena             : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Status signals 
        -----------------------------------------------------------------------
        -- Bit time FSM is in TSEG1
        is_tseg1            : out   std_logic;
        
        -- Bit time FSM is in TSEG2
        is_tseg2            : out   std_logic;
        
        -- Sample signal request (to sample point generator)
        sample_req          : out   std_logic;
        
        -- Sync signal request
        sync_req            : out   std_logic;
        
        -- Bit Tim FSM Output
        bt_fsm              : out   t_bit_time 
    );
end entity;

architecture rtl of bit_time_fsm is

    -- Bit time FSM
    signal current_state    : t_bit_time;
    signal next_state       : t_bit_time;

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
            next_state <= s_bt_reset;
        elsif (h_sync_valid = '1') then
            next_state <= s_bt_tseg1;
        else
            case current_state is
            when s_bt_tseg1 =>
                if (segm_end = '1') then
                    next_state <= s_bt_tseg2;
                end if;
            when s_bt_tseg2 =>
                if (segm_end = '1') then
                    next_state <= s_bt_tseg1;
                end if;
            when s_bt_reset =>
                next_state <= s_bt_tseg1;
            end case;
        end if;
    end process;
    
    ----------------------------------------------------------------------------
    -- Current state process (combinational)
    ----------------------------------------------------------------------------
    curr_state_proc : process(current_state, segm_end, drv_ena)
    begin
        -- Default values
        is_tseg1       <= '0';
        is_tseg2       <= '0';
        sample_req     <= '0';
        sync_req       <= '0';
        
        case current_state is
        when s_bt_reset =>
            if (drv_ena = CTU_CAN_ENABLED) then
                sync_req <= '1';    
            end if;
            
        when s_bt_tseg1 =>
            is_tseg1 <= '1';
            if (segm_end = '1') then
                sample_req <= '1';
            end if;
            
        when s_bt_tseg2 =>
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
        if (res_n = G_RESET_POLARITY) then
            current_state <= s_bt_reset;
        elsif (rising_edge(clk_sys)) then
            if (bt_fsm_ce = '1') then
                current_state <= next_state;
            end if;
        end if;
    end process;
    
    bt_fsm_ce <= '1' when (next_state /= current_state) else
                 '0'; 
                 
    bt_fsm <= current_state;

end architecture rtl;