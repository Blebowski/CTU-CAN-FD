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
--  Trigger signals generator.
--
-- Purpose:
--  Trigger signals are active for one clock cycle. There are two trigger
--  types in CTU CAN FD implementation:
--      1. RX Triggers - 2 Triggers - Pipeline stages: Destuff and Process
--      2. TX Trigger - 1 Trigger - Pipeline stage: Stuff
--  TX trigger is active at the start of bit time and it is used to transmitt
--  Data. RX trigger is active in last cycle of TSEG1 and it represents
--  sample point! Both triggers are always aligned with Time Quanta!
--  Trigger signals are then used for data processing pipeline in CAN 
--  Datapath (e.g. Bit Stuffing, Bit Destuffing, Processing by CAN Core).
--  Trigger signals are demonstrated in following diagram:
--
--             +------+--------------+-----------+----------+
--             | SYNC |     PROP     |    PH1    |    PH2   |
--             +------+--------------+-----------+----------+
--    TX   __¯¯____________________________________________¯¯____
--         ______________________________________________________
--         ____________________________________¯¯________________
--    RX   ______________________________________¯¯______________
--         ______________________________________________________
--    Clock _¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_

--  Note that trigger signal sequence should always be completed. Due to
--  Hard Synchronisation mechanism, Trigger request for e.g. TX Trigger might
--  occur still during pipelined RX Trigger signal active. This occurs when
--  Hard synchronisation occurs just one clock cycle after Sample point.
--  The main task of trigger generator is to generate Triggers from Trigger
--  Requests. If a trigger request occurs during previous trigger active,
--  Trigger generator buffers the request and processes it only after the
--  previous trigger sequence ends. Overall length of bit is maintained, only
--  next TX trigger is throttled by one clock cycle.
-- 
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

entity trigger_generator is
    generic (
        -- Number of signals in Sample trigger
        G_SAMPLE_TRIGGER_COUNT    : natural range 2 to 8 := 3
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys          : in    std_logic;
        
        -- Asynchronous reset
        res_n            : in    std_logic;

        -----------------------------------------------------------------------
        -- Control signal
        -----------------------------------------------------------------------
        -- RX Trigger request (Sample point)
        rx_trig_req      : in    std_logic;

        -- TX Trigger request (Sync)
        tx_trig_req      : in    std_logic;

        -----------------------------------------------------------------------
        -- Trigger outputs
        -----------------------------------------------------------------------
        -- RX Triggers (Two in two following clock cycles)
        rx_triggers     : out std_logic_vector(G_SAMPLE_TRIGGER_COUNT - 1 downto 0);
        
        -- TX Trigger
        tx_trigger      : out std_logic
    );
end entity;

architecture rtl of trigger_generator is

    -- Register to create delayed version of Sample Trigger by one clock cycle.     
    signal rx_trig_req_q           : std_logic;

    ---------------------------------------------------------------------------
    -- Trigger request flag. Set when a request for Sync trigger arrives and
    -- another Sample is still in progress
    ---------------------------------------------------------------------------
    signal tx_trig_req_flag_d    : std_logic;
    signal tx_trig_req_flag_q    : std_logic;
    signal tx_trig_req_flag_dq   : std_logic;

begin
    
    ---------------------------------------------------------------------------
    -- Sync trigger capture register
    ---------------------------------------------------------------------------
    tx_trig_req_flag_d <= '1' when (rx_trig_req_q = '1' and tx_trig_req = '1') else
                          '0' when (rx_trig_req_q = '0') else
                          tx_trig_req_flag_q;

    tx_trig_req_flag_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            tx_trig_req_flag_q <= '0';
        elsif (rising_edge(clk_sys)) then
            tx_trig_req_flag_q <= tx_trig_req_flag_d;
        end if;
    end process;

    tx_trig_req_flag_dq <= tx_trig_req or tx_trig_req_flag_q;

    ---------------------------------------------------------------------------
    -- Register to create delayed version of RX Trigger (for processing by
    -- Protocol Control)
    ---------------------------------------------------------------------------
    rx_trig_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            rx_trig_req_q <= '0';
        elsif (rising_edge(clk_sys)) then
            rx_trig_req_q <= rx_trig_req;
        end if;
    end process;


    ---------------------------------------------------------------------------
    -- RX Trigger, driven directly. Since Sync Trigger lasts only one
    -- clock cycle, and trigger request might never occur at once, we don't
    -- have to do any capturing!
    ---------------------------------------------------------------------------
    rx_triggers(1) <= rx_trig_req;
    rx_triggers(0) <= rx_trig_req_q;

    ---------------------------------------------------------------------------
    -- TX Trigger is active when either direct trigger or flag is active. But
    -- it must be gated when RX Trigger 1 is active. In this case TX Trigger
    -- request flag was set and TX Trigger will be shifted by one clock cycle.
    ---------------------------------------------------------------------------
    tx_trigger <= '0' when (rx_trig_req_q = '1') else
                  tx_trig_req_flag_dq;

    -- <RELEASE_OFF>
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    -- psl default clock is rising_edge(clk_sys);
    
    ---------------------------------------------------------------------------
    -- Sync request and Sample request should never be active at the same time.
    -- This should be handled by Scanner FSM.
    --
    -- psl sync_sample_trig_no_simul_asrt : assert never
    --  (rx_trig_req = '1' and tx_trig_req = '1')
    --  report "Sync and Sample trigger should no be requested at once!";
    ---------------------------------------------------------------------------
    
    -- psl tx_trigger_throttling_cov : cover
    --  {rx_trig_req_q = '1' and tx_trig_req = '1'}
    --  report "TX trigger throtlled!";

    -- <RELEASE_ON>
end architecture rtl;