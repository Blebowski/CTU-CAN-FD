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
-- Module:
--  Operation control FSM.
-- 
-- Purpose:
--  Controls operation state of the node (Transmitter, Receiver, Idle).
--  Controlled by Protocol control FSM.
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

entity operation_control is
    generic(
        -- Reset polarity
        G_RESET_POLARITY     :     std_logic    
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              :in   std_logic;
        
        -- Asynchronous reset
        res_n                :in   std_logic;

        ------------------------------------------------------------------------
        -- Memory registers Interface
        ------------------------------------------------------------------------
        -- Driving bus
        drv_bus              :in   std_logic_vector(1023 downto 0);

        ------------------------------------------------------------------------
        -- Protocol Control Interface
        ------------------------------------------------------------------------
        -- Arbitration lost
        arbitration_lost     :in   std_logic;

        -- Set unit to be transmitter (in SOF)
        set_transmitter      :in   std_logic; 

        -- Set unit to be receiver
        set_receiver         :in   std_logic;

        -- Set unit to be idle
        set_idle             :in   std_logic;

        -- Status outputs
        is_transmitter       :out  std_logic;
        
        -- Unit is receiver
        is_receiver          :out  std_logic;
        
        -- Unit is idle
        is_idle              :out  std_logic
    );
end entity;

architecture rtl of operation_control is
    
    -- CTU CAN FD is enabled
    signal drv_ena           :     std_logic;
    
    -- Operation control FSM
    signal curr_state        :     t_operation_control_state;
    signal next_state        :     t_operation_control_state;

begin
  
    -- Driving bus aliases
    drv_ena <= drv_bus(DRV_ENA_INDEX);

    ---------------------------------------------------------------------------
    -- Next state
    ---------------------------------------------------------------------------
    next_state_proc : process(curr_state, set_idle, set_transmitter,
        set_receiver, arbitration_lost, drv_ena)
    begin
        next_state <= curr_state;
        
        if (drv_ena = CTU_CAN_DISABLED) then
            next_state <= s_oc_off; 
        else
            case curr_state is
            when s_oc_off =>
                if (set_idle = '1') then
                    next_state <= s_oc_idle;
                end if;
            when s_oc_idle =>
                if (set_transmitter = '1') then
                    next_state <= s_oc_transmitter;
                elsif (set_receiver = '1') then
                    next_state <= s_oc_receiver;
                end if;
            when s_oc_transmitter =>
                if (set_idle = '1') then
                    next_state <= s_oc_idle;
                elsif (set_receiver = '1' or arbitration_lost = '1') then
                    next_state <= s_oc_receiver;
                end if;
            when s_oc_receiver =>
                if (set_idle = '1') then
                    next_state <= s_oc_idle;
                elsif (set_transmitter = '1') then
                    next_state <= s_oc_transmitter;
                end if;
            end case;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Current state
    ---------------------------------------------------------------------------
    curr_state_proc : process(curr_state)
    begin
        is_idle <= '0';
        is_transmitter <= '0';
        is_receiver <= '0';
        
        case curr_state is
        when s_oc_off =>
        when s_oc_idle =>
            is_idle <= '1';
        when s_oc_transmitter =>
            is_transmitter <= '1';
        when s_oc_receiver =>
            is_receiver <= '1';
        end case;
    end process;

    ---------------------------------------------------------------------------
    -- State register
    ---------------------------------------------------------------------------
    state_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = G_RESET_POLARITY) then
            curr_state <= s_oc_off;
        elsif (rising_edge(clk_sys)) then
            curr_state <= next_state;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);
    
    -- psl valid_arb_lost_asrt : assert never
    --  (arbitration_lost = '1' and curr_state /= s_oc_transmitter)
    -- report "Unit which is not transmitter lost arbitration"
    -- severity error;
    
    -- psl no_tx_rx_req_in_idle_asrt : assert never
    --  (set_transmitter = '1' or set_receiver = '1') and (curr_state = s_oc_off)
    -- report "Unit which is OFF can't be set to Transmitter or Receiver"
    -- severity error;
    
    -- psl no_simul_tx_rx_set_asrt : assert never
    --  (set_transmitter = '1' and set_receiver = '1')
    -- report "Unit can't be set to transmitter and receiver simultaneously!"
    -- severity error;
    
    -- psl no_simul_tx_idle_set_asrt : assert never
    --  (set_transmitter = '1' and set_idle = '1')
    -- report "Unit can't be set to transmitter and idle simultaneously!"
    -- severity error;

end architecture;
