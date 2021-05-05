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
-- Package:
--  CAN types
-- 
-- Purpose:
--  Package with type definitions for CTU CAN FD.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.can_constants.all;
use ctu_can_fd_rtl.can_config.all;

package can_types is

    ----------------------------------------------------------------------------
    -- State Machine types
    ----------------------------------------------------------------------------

    -- Fault confinement state of node
    type t_fault_conf_state is (
        s_fc_err_active,
        s_fc_err_passive,
        s_fc_bus_off
    );

    -- Operation mode of the Node
    type t_operation_control_state is (
        s_oc_off,
        s_oc_idle,
        s_oc_transmitter,
        s_oc_receiver
    );

    -- Protocol control FSM
    type t_protocol_control_state is (
        s_pc_off,
        s_pc_integrating,
        s_pc_idle,
        s_pc_sof,
        s_pc_base_id,
        s_pc_rtr_srr_r1,
        s_pc_ide,
        s_pc_ext_id,
        s_pc_rtr_r1,
        s_pc_edl_r1,
        s_pc_r0_ext,
        s_pc_r0_fd,
        s_pc_edl_r0,
        s_pc_brs,
        s_pc_esi,
        s_pc_dlc,
        s_pc_data,
        s_pc_stuff_count,
        s_pc_crc,
        s_pc_crc_delim,
        s_pc_ack,
        s_pc_ack_fd_1,
        s_pc_ack_fd_2,
        s_pc_ack_delim,
        s_pc_eof,
        s_pc_intermission,
        s_pc_suspend,
        s_pc_reintegrating_wait,
        s_pc_reintegrating,
        s_pc_act_err_flag,
        s_pc_pas_err_flag,
        s_pc_err_delim_wait,
        s_pc_err_flag_too_long,
        s_pc_ovr_flag_too_long,
        s_pc_err_delim,
        s_pc_ovr_flag,
        s_pc_ovr_delim_wait,
        s_pc_ovr_delim
    );

    type t_bit_time is (
        s_bt_tseg1,
        s_bt_tseg2,
        s_bt_reset
    );

    -- Logger state machine type 
    type logger_state_type is (
        config,
        ready,
        running
    );

    -- RX Buffer loader type
    type t_rx_buf_state is (
        s_rxb_idle,
        s_rxb_store_frame_format,
        s_rxb_store_identifier,
        s_rxb_skip_ts_low,
        s_rxb_skip_ts_high,
        s_rxb_store_end_ts_low,
        s_rxb_store_end_ts_high,
        s_rxb_store_data
    );

    -- TX arbitrator state type
    type t_tx_arb_state is (
        s_arb_idle,
        s_arb_sel_low_ts,
        s_arb_sel_upp_ts,
        s_arb_sel_ffw,
        s_arb_sel_idw,
        s_arb_validated,
        s_arb_locked
    );
  
    -- TXT buffer state type
    type t_txt_buf_state is (
        s_txt_empty,
        s_txt_ready,
        s_txt_tx_prog,
        s_txt_ab_prog,
        s_txt_ok,
        s_txt_failed,
        s_txt_aborted
    );

    ----------------------------------------------------------------------------
    -- TXT Buffer types
    ----------------------------------------------------------------------------
    
    -- Priorities of TXT Buffers
    type t_txt_bufs_priorities is array (integer range <>) of
        std_logic_vector(2 downto 0);

    -- Memory outputs of TXT Buffers
    type t_txt_bufs_output is array (integer range <>) of
        std_logic_vector(31 downto 0);

    -- States of Buffers
    type t_txt_bufs_state is array (integer range <>) of
        std_logic_vector(3 downto 0);
    
    -- SW commands
    type t_txtb_sw_cmd is record
        set_rdy   : std_logic;
        set_ety   : std_logic;
        set_abt   : std_logic;
    end record;

    -- HW commands
    type t_txtb_hw_cmd is record
        lock      : std_logic;
        unlock    : std_logic;
        valid     : std_logic;
        err       : std_logic;
        arbl      : std_logic;
        failed    : std_logic;
    end record;


    ---------------------------------------------------------------------------- 
    -- DLC Types
    ----------------------------------------------------------------------------
    type dlc_type is array (0 to 15) of std_logic_vector(3 downto 0);
    type length_type is array (0 to 15) of natural;
    constant dlc_codes : dlc_type := ("0000", "0001", "0010", "0011",
                                      "0100", "0101", "0110", "0111",
                                      "1000", "1001", "1010", "1011",
                                      "1100", "1101", "1110","1111");
    constant dlc_length : length_type := (0, 1, 2, 3, 4, 5, 6, 7, 8,
                                          12, 16, 20, 24, 32, 48, 64);

    ---------------------------------------------------------------------------- 
    -- Debug types
    ----------------------------------------------------------------------------
    type t_ctu_can_fd_test_probe is record
        rx_trigger_nbs      : std_logic;
        rx_trigger_wbs      : std_logic;
        tx_trigger          : std_logic;
    end record;

end package;
