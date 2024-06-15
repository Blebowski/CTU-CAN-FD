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
--  Fault confinement
--
-- Sub-modules:
--  1. Fault confinement rules
--  2. Error counters
--  3. Fault confinement FSM.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;

use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity fault_confinement is
    port (
        -------------------------------------------------------------------------------------------
        -- Clock and Asynchronous Reset
        -------------------------------------------------------------------------------------------
        clk_sys                 : in  std_logic;
        res_n                   : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- DFT support
        -------------------------------------------------------------------------------------------
        scan_enable            : in   std_logic;

        -------------------------------------------------------------------------------------------
        -- Memory registers interface
        -------------------------------------------------------------------------------------------
        mr_mode_rom             : in  std_logic;
        mr_ewl_ew_limit         : in  std_logic_vector(7 downto 0);
        mr_erp_erp_limit        : in  std_logic_vector(7 downto 0);
        mr_ctr_pres_ctpv        : in  std_logic_vector(8 downto 0);
        mr_ctr_pres_ptx         : in  std_logic;
        mr_ctr_pres_prx         : in  std_logic;
        mr_ctr_pres_enorm       : in  std_logic;
        mr_ctr_pres_efd         : in  std_logic;
        mr_status_ewl           : out std_logic;

        -------------------------------------------------------------------------------------------
        -- Error signalling for interrupts
        -------------------------------------------------------------------------------------------
        -- Fault confinement state changed
        fcs_changed             : out std_logic;

        -- Error warning limit was reached
        err_warning_limit_pulse : out std_logic;

        -------------------------------------------------------------------------------------------
        -- Operation control Interface
        -------------------------------------------------------------------------------------------
        -- Unit is transmitter
        is_transmitter          : in  std_logic;

        -- Unit is receiver
        is_receiver             : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Protocol control Interface
        -------------------------------------------------------------------------------------------
        -- Sample control (Nominal, Data, Secondary)
        sp_control              : in  std_logic_vector(1 downto 0);

        -- Set unit to error active (after re-integration). Erases eror counters to 0.
        set_err_active          : in  std_logic;

        -- Error is detected
        err_detected            : in  std_logic;

        -- Error counter should remain unchanged
        err_ctrs_unchanged      : in  std_logic;

        -- Primary Error
        primary_err             : in  std_logic;

        -- Active Error Flag or Overload flag is being tranmsmitted
        act_err_ovr_flag        : in  std_logic;

        -- Error delimiter too late
        err_delim_late          : in  std_logic;

        -- Transmission of frame valid
        tran_valid              : in  std_logic;

        -- Decrement receive Error counter
        decrement_rec           : in  std_logic;

        -- Bit Error after ACK error in Passive Error flag
        bit_err_after_ack_err   : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Fault confinement State indication
        -------------------------------------------------------------------------------------------
        -- Unit is error active
        is_err_active           : out std_logic;

        -- Unit is error passive
        is_err_passive          : out std_logic;

        -- Unit is Bus-off
        is_bus_off              : out std_logic;

        -------------------------------------------------------------------------------------------
        -- Error counters
        -------------------------------------------------------------------------------------------
        -- TX Error counter
        tx_err_ctr              : out std_logic_vector(8 downto 0);

        -- RX Error counter
        rx_err_ctr              : out std_logic_vector(8 downto 0);

        -- Error counter in Nominal Bit-rate
        norm_err_ctr            : out std_logic_vector(15 downto 0);

        -- Error counter in Data Bit-rate
        data_err_ctr            : out std_logic_vector(15 downto 0)
    );
end entity;

architecture rtl of fault_confinement is

    -- Internal TX/RX Error counter values
    signal tx_err_ctr_i         : std_logic_vector(8 downto 0);
    signal rx_err_ctr_i         : std_logic_vector(8 downto 0);

    -- Increment decrement commands
    signal inc_one              : std_logic;
    signal inc_eight            : std_logic;
    signal dec_one              : std_logic;

begin

    -----------------------------------------------------------------------------------------------
    -- Fault confinement FSM
    -----------------------------------------------------------------------------------------------
    fault_confinement_fsm_inst : entity ctu_can_fd_rtl.fault_confinement_fsm
    port map(
        res_n                   => res_n,                    -- IN
        clk_sys                 => clk_sys,                  -- IN

        mr_ewl_ew_limit         => mr_ewl_ew_limit,          -- IN
        mr_erp_erp_limit        => mr_erp_erp_limit,         -- IN
        mr_status_ewl           => mr_status_ewl,            -- OUT

        set_err_active          => set_err_active,           -- IN
        tx_err_ctr              => tx_err_ctr_i,             -- IN
        rx_err_ctr              => rx_err_ctr_i,             -- IN

        is_err_active           => is_err_active,            -- OUT
        is_err_passive          => is_err_passive,           -- OUT
        is_bus_off              => is_bus_off,               -- OUT

        fcs_changed             => fcs_changed,              -- OUT
        err_warning_limit_pulse => err_warning_limit_pulse   -- OUT
    );


    -----------------------------------------------------------------------------------------------
    -- Error counters
    -----------------------------------------------------------------------------------------------
    err_counters_inst : entity ctu_can_fd_rtl.err_counters
    port map(
        clk_sys                 => clk_sys,                  -- IN
        res_n                   => res_n,                    -- IN
        scan_enable             => scan_enable,              -- IN

        sp_control              => sp_control,               -- IN
        inc_one                 => inc_one,                  -- IN
        inc_eight               => inc_eight,                -- IN
        dec_one                 => dec_one,                  -- IN
        set_err_active          => set_err_active,           -- IN
        is_transmitter          => is_transmitter,           -- IN
        is_receiver             => is_receiver,              -- IN

        mr_ctr_pres_ctpv        => mr_ctr_pres_ctpv,         -- IN
        mr_ctr_pres_ptx         => mr_ctr_pres_ptx,          -- IN
        mr_ctr_pres_prx         => mr_ctr_pres_prx,          -- IN
        mr_ctr_pres_enorm       => mr_ctr_pres_enorm,        -- IN
        mr_ctr_pres_efd         => mr_ctr_pres_efd,          -- IN

        rx_err_ctr              => rx_err_ctr_i,             -- OUT
        tx_err_ctr              => tx_err_ctr_i,             -- OUT
        norm_err_ctr            => norm_err_ctr,             -- OUT
        data_err_ctr            => data_err_ctr              -- OUT
    );

    -----------------------------------------------------------------------------------------------
    -- Fault confinement rules
    -----------------------------------------------------------------------------------------------
    fault_confinement_rules_inst : entity ctu_can_fd_rtl.fault_confinement_rules
    port map(
        clk_sys                 => clk_sys,                  -- IN

        is_transmitter          => is_transmitter,           -- IN
        is_receiver             => is_receiver,              -- IN
        err_detected            => err_detected,             -- IN
        err_ctrs_unchanged      => err_ctrs_unchanged,       -- IN
        primary_err             => primary_err,              -- IN
        act_err_ovr_flag        => act_err_ovr_flag,         -- IN
        err_delim_late          => err_delim_late,           -- IN
        tran_valid              => tran_valid,               -- IN
        decrement_rec           => decrement_rec,            -- IN
        bit_err_after_ack_err   => bit_err_after_ack_err,    -- IN

        mr_mode_rom             => mr_mode_rom,              -- IN

        inc_one                 => inc_one,                  -- OUT
        inc_eight               => inc_eight,                -- OUT
        dec_one                 => dec_one                   -- OUT
    );

    -----------------------------------------------------------------------------------------------
    -- Internal signals to output propagation
    -----------------------------------------------------------------------------------------------
    tx_err_ctr <= tx_err_ctr_i;
    rx_err_ctr <= rx_err_ctr_i;

end architecture;
