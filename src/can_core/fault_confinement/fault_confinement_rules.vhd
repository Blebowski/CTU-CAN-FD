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
--  Fault confinement rules.
--------------------------------------------------------------------------------
-- Revision History:
--    02.4.2019  Created file
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
        
        -- Reception of frame valid
        rec_valid               :in   std_logic;

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

begin
    
    ---------------------------------------------------------------------------
    -- Increment RX Error counter by 1 when Receiver detects an error which
    -- is not during Error flag or overload flag!
    ---------------------------------------------------------------------------
    inc_one <= '1' when (err_detected = '1' and act_err_ovr_flag = '0' and
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
    ---------------------------------------------------------------------------
    inc_eight <= '1' when (primary_err = '1' and is_receiver = '1') else
                 '1' when (act_err_ovr_flag = '1' and err_detected = '1') else
                 '1' when (is_transmitter = '1' and 
                           err_detected = '1' and
                           err_ctrs_unchanged = '0') else
                 '1' when (err_delim_late = '1') else
                 '0';         

    ---------------------------------------------------------------------------
    -- Decrement by 1 when either transmission or reception is valid
    ---------------------------------------------------------------------------
    dec_one <= '1' when (rec_valid = '1' or tran_valid = '1') else
               '0';

end architecture;