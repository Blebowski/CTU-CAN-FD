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
--  Operation control FSM.
-- 
-- Purpose:
--  Controls operation state of the node (Transmitter, Receiver, Idle).
--  Controlled by Protocol control FSM.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_components_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.common_blocks_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity operation_control is
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              :in   std_logic;
        
        -- Asynchronous reset
        res_n                :in   std_logic;

        ------------------------------------------------------------------------
        -- Prescaler Interface
        ------------------------------------------------------------------------
        -- RX Trigger
        rx_trigger           :in   std_logic;

        ------------------------------------------------------------------------
        -- Fault confinement Interface
        ------------------------------------------------------------------------
        -- Unit is Bus-off
        is_bus_off           :in   std_logic;

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

    -- Unit is off the bus
    signal go_to_off         :     std_logic;

begin

    -- Unit should go to off when it turned Error Passive or when it is disabled.
    -- Gated by RX Trigger (delayed till next sample point) to avoid transiting
    -- back to off directly after end of integration/reintegration.
    go_to_off <= '1' when (is_bus_off = '1') and (rx_trigger = '1')
                     else
                 '0';

    ---------------------------------------------------------------------------
    -- Next state
    ---------------------------------------------------------------------------
    next_state_proc : process(curr_state, set_idle, set_transmitter,
        set_receiver, arbitration_lost, go_to_off)
    begin
        next_state <= curr_state;
        
        case curr_state is
        when s_oc_off =>
            if (set_idle = '1') then
                next_state <= s_oc_idle;
            end if;
            
        when s_oc_idle =>
            if (go_to_off = '1') then
                next_state <= s_oc_off; 
            elsif (set_transmitter = '1') then
                next_state <= s_oc_transmitter;
            elsif (set_receiver = '1') then
                next_state <= s_oc_receiver;
            end if;
            
        when s_oc_transmitter =>
            if (go_to_off = '1') then
                next_state <= s_oc_off; 
            elsif (set_idle = '1') then
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
        if (res_n = '0') then
            curr_state <= s_oc_off;
        elsif (rising_edge(clk_sys)) then
            curr_state <= next_state;
        end if;
    end process;

    -- <RELEASE_OFF>
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
    
    -- psl never_to_off_as_receiver_asrt : assert never
    --  (go_to_off = '1' and curr_state = s_oc_receiver)
    --  report "Unit should not become Bus off while receiver!";
    
    -- psl op_fsm_transmitter_cov : cover
    --  {curr_state = s_oc_transmitter};

    -- psl op_fsm_received_cov : cover
    --  {curr_state = s_oc_receiver};

    -- <RELEASE_ON>
end architecture;