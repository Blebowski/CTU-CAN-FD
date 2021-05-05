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
--  Fault confinement FSM.
--
-- Purpose:
--  Implements Fault confinement state of a node. State is changed when Error
--  Counters reach according threshold. Error warning limit interrupt is 
--  supported. Error warning limit and Error passive threshold are configurable
--  from Driving Bus. Protocol control sets node to error active upon the end
--  of Intefration and Re-integration.
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

entity fault_confinement_fsm is
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
        
        -- Unit enabled
        drv_ena                 :in   std_logic;
       
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
        -- Fault confinement state changed
        fcs_changed             :out  std_logic;

        -- Error warning limit was reached
        err_warning_limit       :out  std_logic
    );
end entity;

architecture rtl of fault_confinement_fsm is

    signal tx_err_ctr_mt_erp : std_logic;
    signal rx_err_ctr_mt_erp : std_logic;
    
    signal tx_err_ctr_mt_ewl : std_logic;
    signal rx_err_ctr_mt_ewl : std_logic;

    signal tx_err_ctr_mt_255 : std_logic;
    
    -- Error warning limit register (to detect change)
    signal err_warning_limit_d : std_logic;
    signal err_warning_limit_q : std_logic;
    
    ---------------------------------------------------------------------------
    -- Fault confinement FSM
    ---------------------------------------------------------------------------
    signal curr_state : t_fault_conf_state;
    signal next_state : t_fault_conf_state;
    
    -- Reset DFF
    signal fc_fsm_res_d : std_logic;
    signal fc_fsm_res_q : std_logic;
    
begin
    
    -- TX Error counter more than Error Passive Limit
    tx_err_ctr_mt_erp <= '1' when (unsigned(tx_err_ctr) >= unsigned(erp)) else
                         '0';

    -- RX Error counter more than Error Passive Limit
    rx_err_ctr_mt_erp <= '1' when (unsigned(rx_err_ctr) >= unsigned(erp)) else
                         '0';

    -- TX Error counter more than 255
    tx_err_ctr_mt_255 <= '1' when (unsigned(tx_err_ctr) > 255) else
                         '0';

    -- TX Error counter more than Error Passive Limit
    tx_err_ctr_mt_ewl <= '1' when (unsigned(tx_err_ctr) >= unsigned(ewl)) else
                         '0';

    -- RX Error counter more than Error Passive Limit
    rx_err_ctr_mt_ewl <= '1' when (unsigned(rx_err_ctr) >= unsigned(ewl)) else
                         '0';

    err_warning_limit_d <= '1' when (tx_err_ctr_mt_ewl = '1' or
                                     rx_err_ctr_mt_ewl = '1')
                               else
                           '0';

    ---------------------------------------------------------------------------
    -- Error warning limit register
    ---------------------------------------------------------------------------
    ewl_reg_proc : process(res_n, clk_sys)
    begin
        if (res_n = '0') then
            err_warning_limit_q <= '0';
        elsif (rising_edge(clk_sys)) then
            err_warning_limit_q <= err_warning_limit_d;
        end if;
    end process;
    
    err_warning_limit <= '1' when (err_warning_limit_d /= err_warning_limit_q)
                             else
                         '0';
                         
    -- Reset DFF, keep Bus-off when disabled!
    fc_fsm_res_d <= drv_ena;
    
    dff_fc_reset_inst : dff_arst
    generic map(
        G_RESET_POLARITY   => '0',
        
        -- Reset to the same value as is polarity of reset so that other DFFs
        -- which are reset by output of this one will be reset too!
        G_RST_VAL          => '0'
    )
    port map(
        arst               => res_n,                -- IN
        clk                => clk_sys,              -- IN
        input              => fc_fsm_res_d,         -- IN

        output             => fc_fsm_res_q          -- OUT
    );
    
    ---------------------------------------------------------------------------
    -- Next state process
    ---------------------------------------------------------------------------
    fc_fsm_next_state_proc : process(curr_state, tx_err_ctr_mt_255,
        tx_err_ctr_mt_erp, rx_err_ctr_mt_erp, set_err_active)
    begin
        next_state <= curr_state;

        case curr_state is
        when s_fc_err_active =>
            if (tx_err_ctr_mt_erp = '1' or rx_err_ctr_mt_erp = '1') then
                next_state <= s_fc_err_passive;   
            end if;

        when s_fc_err_passive =>
            if (tx_err_ctr_mt_255 = '1') then
                next_state <= s_fc_bus_off;
            elsif (tx_err_ctr_mt_erp = '0' and rx_err_ctr_mt_erp = '0') then
                next_state <= s_fc_err_active;
            end if;

        when s_fc_bus_off =>
            if (set_err_active = '1') then
                next_state <= s_fc_err_active;
            end if;
        end case;
        
    end process;

    ---------------------------------------------------------------------------
    -- State register
    ---------------------------------------------------------------------------
    fault_conf_state_reg : process(clk_sys, fc_fsm_res_q)
    begin
        if (fc_fsm_res_q = '0') then
            curr_state <= s_fc_bus_off;
        elsif (rising_edge(clk_sys)) then
            curr_state <= next_state;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Current state
    ---------------------------------------------------------------------------
    fc_fsm_curr_state_proc : process(curr_state)
    begin
        is_err_active     <= '0';
        is_err_passive    <= '0';
        is_bus_off        <= '0';
        
        case curr_state is
        when s_fc_err_active =>
            is_err_active <= '1';

        when s_fc_err_passive =>
            is_err_passive <= '1';

        when s_fc_bus_off =>
            is_bus_off <= '1';
        end case;
    end process;

    -- Fault confinement state changed when current state is not equal to
    -- Next state.
    fcs_changed <= '1' when (curr_state /= next_state) else
                   '0';

end architecture;