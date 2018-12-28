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
--  Package with type definitions for CTU CAN FD.
--------------------------------------------------------------------------------
-- Revision History:
--   28.12.2018   Created file, separated from "can_constants" package.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

package can_types is

    ----------------------------------------------------------------------------
    -- State Machine types
    ----------------------------------------------------------------------------

    -- Error state of node
    type error_state_type is (
        error_active,
        error_passive,
        bus_off
    );

    -- Operation mode of the Node
    type oper_mode_type is (
        integrating,
        idle,
        transciever,
        reciever
    );

    -- Protocol control FSM
    type protocol_type is (
        sof,
        arbitration,
        control,
        data,
        crc,
        delim_ack,
        eof,
        interframe,
        overload,
        error,
        off
    );

    -- Note: two bits are two bits between Base and Extended identifier
    -- Note: one bit is the last remaining bit after the identifier extension
    type arb_type is (
        base_id,
        two_bits,
        ext_id,
        one_bit
    );

    -- Within ISO CAN FD new field stuff count is needed!
    type crc_type is(
        stuff_count,
        real_crc
    );

    -- Intermission field sub-State
    type interm_spc_type is (
        intermission,
        suspend,
        interm_idle
    );

    -- Error frame subtype
    type err_frame_type is (
        err_flg_sup,
        err_delim
    );

    -- Overload frame subtype
    type ovr_frame_type is (
        ovr_flg_sup,
        ovr_delim
    );

    type bit_time_type is (
        sync,
        prop,
        ph1,
        ph2,
        h_sync,
        reset
    );

    -- Logger state machine type 
    type logger_state_type is (
        config,
        ready,
        running
    );

    -- RX Buffer loader type
    type rx_buf_fsm_type is (
        rxb_idle,
        rxb_store_frame_format,
        rxb_store_identifier,
        rxb_store_beg_ts_low,
        rxb_store_beg_ts_high,
        rxb_store_end_ts_low,
        rxb_store_end_ts_high,
        rxb_store_data
    );

    -- TX arbitrator state type
    type tx_arb_state_type is (
        arb_sel_low_ts,
        arb_sel_upp_ts,
        arb_sel_ffw,
        arb_locked
    );
  
    -- TXT buffer state type
    type txt_fsm_type is (
        txt_empty,
        txt_ready,
        txt_tx_prog,
        txt_ab_prog,
        txt_ok,
        txt_error,
        txt_aborted
    );

    ----------------------------------------------------------------------------
    -- TXT Buffer types
    ----------------------------------------------------------------------------

    -- Priorities of TXT Buffers
    type txtb_priorities_type is array (0 to TXT_BUFFER_COUNT - 1) of
        std_logic_vector(2 downto 0);

    -- Memory outputs of TXT Buffer
    type txtb_output_type is array (0 to TXT_BUFFER_COUNT - 1) of
        std_logic_vector(31 downto 0);

    -- States of Buffer
    type txtb_state_type is array (0 to TXT_BUFFER_COUNT - 1) of
        std_logic_vector(3 downto 0);

    -- SW commands
    type txt_sw_cmd_type is record
        set_rdy   : std_logic;
        set_ety   : std_logic;
        set_abt   : std_logic;
    end record;

    -- HW commands
    type txt_hw_cmd_type is record
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

end package;
