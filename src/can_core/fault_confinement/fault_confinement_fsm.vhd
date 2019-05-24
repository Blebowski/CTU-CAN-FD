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
--  Fault confinement FSM.
--------------------------------------------------------------------------------
-- Revision History:
--    27.3.2019  Created file
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

entity fault_confinement_fsm is
    generic(
        -- Reset polarity
        G_RESET_POLARITY        :     std_logic := '0'
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys                 :in   std_logic;
        
        -- Asynchronous reset
        res_n                   :in   std_logic;

        -----------------------------------------------------------------------
        -- Control signals
        -----------------------------------------------------------------------
        -- Error warning limit
        ewl                     :in   std_logic_vector(8 downto 0);
        
        -- Error passive threshold
        erp                     :in   std_logic_vector(8 downto 0);

        -- Set unit to be error active
        set_err_active          :in   std_logic;
       
        -----------------------------------------------------------------------
        -- Error counters
        -----------------------------------------------------------------------
        -- TX Error counter
        tx_err_ctr              :in   std_logic_vector(8 downto 0);
        
        -- RX Error counter
        rx_err_ctr              :in   std_logic_vector(8 downto 0);

        -----------------------------------------------------------------------
        -- Fault confinement State indication
        -----------------------------------------------------------------------
        -- Unit is error active
        is_err_active           :out  std_logic;
        
        -- Unit is error passive
        is_err_passive          :out  std_logic;
        
        -- Unit is Bus-off
        is_bus_off              :out  std_logic;

        -----------------------------------------------------------------------
        -- Status outputs
        -----------------------------------------------------------------------
        -- Error passive state changed
        error_passive_changed   :out  std_logic;

        -- Error warning limit was reached
        error_warning_limit     :out  std_logic
    );
end entity;

architecture rtl of fault_confinement_fsm is

    signal tx_err_ctr_mt_erp : std_logic;
    signal rx_err_ctr_mt_erp : std_logic;
    
    signal tx_err_ctr_mt_ewl : std_logic;
    signal rx_err_ctr_mt_ewl : std_logic;

    signal tx_err_ctr_mt_255 : std_logic;
    
    ---------------------------------------------------------------------------
    -- Fault confinement FSM
    ---------------------------------------------------------------------------
    signal curr_state : t_fault_conf_state;
    signal next_state : t_fault_conf_state;
    
begin
    
    -- TX Error counter more than Error Passive Limit
    tx_err_ctr_mt_erp <= '1' when (unsigned(tx_err_ctr) > unsigned(erp)) else
                         '0';

    -- RX Error counter more than Error Passive Limit
    rx_err_ctr_mt_erp <= '1' when (unsigned(rx_err_ctr) > unsigned(erp)) else
                         '0';

    -- TX Error counter more than 255
    tx_err_ctr_mt_255 <= '1' when (unsigned(tx_err_ctr) > 255) else
                         '0';

    -- TX Error counter more than Error Passive Limit
    tx_err_ctr_mt_ewl <= '1' when (unsigned(tx_err_ctr) > unsigned(ewl)) else
                         '0';

    -- RX Error counter more than Error Passive Limit
    rx_err_ctr_mt_ewl <= '1' when (unsigned(rx_err_ctr) > unsigned(ewl)) else
                         '0';

    error_warning_limit <= '1' when (tx_err_ctr_mt_ewl = '1' or
                                     rx_err_ctr_mt_ewl = '1')
                               else
                           '0';
    
    ---------------------------------------------------------------------------
    -- Next state process
    ---------------------------------------------------------------------------
    fc_fsm_next_state_proc : process(curr_state, tx_err_ctr_mt_255,
        tx_err_ctr_mt_erp, rx_err_ctr_mt_erp, set_err_active)
    begin
        next_state <= curr_state;

        case curr_state is
        when s_fc_error_active =>
            if (tx_err_ctr_mt_erp = '1' or rx_err_ctr_mt_erp = '1') then
                next_state <= s_fc_error_passive;   
            end if;

        when s_fc_error_passive =>
            if (tx_err_ctr_mt_255 = '1') then
                next_state <= s_fc_bus_off;
            elsif (tx_err_ctr_mt_erp = '0' and rx_err_ctr_mt_erp = '0') then
                next_state <= s_fc_error_active;
            end if;

        when s_fc_bus_off =>
            if (set_err_active = '1') then
                next_state <= s_fc_error_active;
            end if;
        end case;
        
    end process;

    ---------------------------------------------------------------------------
    -- State register
    ---------------------------------------------------------------------------
    fault_conf_state_reg : process(clk_sys, res_n)
    begin
        if (res_n = G_RESET_POLARITY) then
            curr_state <= s_fc_bus_off;
        elsif (rising_edge(clk_sys)) then
            curr_state <= next_state;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Current state
    ---------------------------------------------------------------------------
    fc_fsm_curr_state_proc : process(curr_state, tx_err_ctr_mt_255,
        tx_err_ctr_mt_erp, rx_err_ctr_mt_erp, set_err_active)
    begin
        is_err_active     <= '0';
        is_err_passive    <= '0';
        is_bus_off        <= '0';
        error_passive_changed  <= '0';
        
        case curr_state is
        when s_fc_error_active =>
            is_err_active <= '1';
            if (tx_err_ctr_mt_erp = '1' or rx_err_ctr_mt_erp = '1') then
                error_passive_changed <= '1';
            end if;
       
        when s_fc_error_passive =>
            is_err_passive <= '1';

        when s_fc_bus_off =>
            is_bus_off <= '1';
        end case;
    end process;

end architecture;