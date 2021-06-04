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
--  Fault confinement rules.
--
-- Purpose:
--  Implement fault confinement rules for incrementing and decrementing Fault
--  confinement error counters. Controlled by Protocol control via standardized
--  interface as described in ISO 11898-1 2015.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;

use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity fault_confinement_rules is
    port(
        -----------------------------------------------------------------------
        -- Operation control interface
        ------------------------------------------------------------------------
        -- Unit is transmitter
        is_transmitter          :in   std_logic;

        -- Unit is receiver
        is_receiver             :in   std_logic;

        -----------------------------------------------------------------------
        -- Protocol Control interface
        -----------------------------------------------------------------------
        -- Error is detected
        err_detected            :in   std_logic;
        
        -- Error counter should remain unchanged
        err_ctrs_unchanged      :in   std_logic;
        
        -- Primary Error
        primary_err             :in   std_logic;
        
        -- Active Error Flag or Overload flag is being tranmsmitted
        act_err_ovr_flag        :in   std_logic;
        
        -- Error delimiter too late
        err_delim_late          :in   std_logic;
        
        -- Transmission of frame valid
        tran_valid              :in   std_logic;
        
        -- Decrement receive Error counter
        decrement_rec           :in   std_logic;

        -- Bit Error in passive error flag after ACK error
        bit_err_after_ack_err   :in   std_logic;

        -----------------------------------------------------------------------
        -- Control registers interface
        -----------------------------------------------------------------------
        -- ROM mode enabled
        drv_rom_ena             :in   std_logic;

        -----------------------------------------------------------------------
        -- Output signals to error counters
        -----------------------------------------------------------------------
        -- Increment Error counter by 1
        inc_one                 :out  std_logic;

        -- Increment Error counter by 8
        inc_eight               :out  std_logic;
        
        -- Decrement Error counter by 1
        dec_one                 :out  std_logic
    );
end entity;

architecture rtl of fault_confinement_rules is

    signal inc_one_i    : std_logic;
    signal inc_eight_i  : std_logic;

begin
    
    ---------------------------------------------------------------------------
    -- Increment RX Error counter by 1 when Receiver detects an error which
    -- is not during Active Error flag or overload flag!
    ---------------------------------------------------------------------------
    inc_one_i <= '1' when (err_detected = '1' and act_err_ovr_flag = '0' and
                           is_receiver = '1')
                     else
                 '0';

    ---------------------------------------------------------------------------
    -- Increment by 8:
    --  - Receiver detects DOMINANT bit as first bit after sending and Error
    --    flag (rule "b")
    --  - Transmitter/Receiver detect a bit error while sending Active Error
    --    flag or an overload flag! Note that other than bit error can't be
    --    signalled in Error Flag on 'err_detected'! (rules "d" and "e")
    --  - Transmitter sends Error flag but non of the exceptions are valid
    --     (rule "c")
    --  - Error delimiter comes too late (more than 14 consecutive bits),
    --    (rule "f")
    --  - ACK Error followed by bit error during passive error frame!
    ---------------------------------------------------------------------------
    inc_eight_i <= '1' when (primary_err = '1' and is_receiver = '1') else
                   '1' when (act_err_ovr_flag = '1' and err_detected = '1') else
                   '1' when (is_transmitter = '1' and 
                             err_detected = '1' and
                             err_ctrs_unchanged = '0') else
                   '1' when (err_delim_late = '1' or bit_err_after_ack_err = '1') else
                   '0';         

    ---------------------------------------------------------------------------
    -- Decrement by 1 when either transmission or reception is valid
    ---------------------------------------------------------------------------
    dec_one <= '1' when (decrement_rec = '1' or tran_valid = '1') else
               '0';

    ---------------------------------------------------------------------------
    -- Gating by ROM mode. In ROM mode, Error counters shall not increment
    -- in ROM mode. Not that decrement does not need to be gated since the
    -- counter will stay at 0!
    ---------------------------------------------------------------------------
    inc_one <= '1' when (inc_one_i = '1' and drv_rom_ena = ROM_DISABLED) else
               '0';
    inc_eight <= '1' when (inc_eight_i = '1' and drv_rom_ena = ROM_DISABLED) else
                 '0';

end architecture;
