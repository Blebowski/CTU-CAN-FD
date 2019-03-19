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
--  Trigger signals generator.
--
--  Trigger signals are active for one clock cycle. There are two trigger
--  signals in CTU CAN FD implementation:
--      1. Sync trigger
--      2. Sample trigger
--  Sync trigger is active at the start of bit time and it is used to transmitt
--  Data. Sample trigger is active in last cycle of TSEG1 and it represents
--  sample point! Both triggers are always aligned with Time Quanta!
--  Each trigger is pipelined to several consecutive clock cycles. Trigger
--  signals are then used for data processing pipeline in CAN Datapath (e.g.
--  Bit Stuffing, Bit Destuffing, Processing by CAN Core).
--  Trigger signals are demonstrated in following diagram:
--
--             +------+--------------+-----------+----------+
--             | SYNC |     PROP     |    PH1    |    PH2   |
--             +------+--------------+-----------+----------+
--    Sync __¯¯____________________________________________¯¯____
--         ____¯¯____________________________________________¯¯__
--         ____________________________________¯¯________________
--    Sample_____________________________________¯¯______________
--         ________________________________________¯¯____________
--    Clock _¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_¯_

--  Note that trigger signal sequence should always be completed. Due to
--  Hard Synchronisation mechanism, Trigger request for e.g. Sync Trigger might
--  occur still during pipelined Sample Trigger signals active.
--  The main task of trigger generator is to generate Triggers from Trigger
--  Requests. If a trigger request occurs during previous trigger active,
--  Trigger generator buffers the request and processes it only after the
--  previous trigger sequence ends.
-- 
--------------------------------------------------------------------------------
-- Revision History:
--    23.02.2019   Created file
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library work;
use work.id_transfer.all;
use work.can_constants.all;
use work.can_components.all;
use work.can_types.all;
use work.cmn_lib.all;
use work.drv_stat_pkg.all;
use work.endian_swap.all;
use work.reduce_lib.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

entity trigger_generator is
    generic (
        -- Reset polarity
        reset_polarity          : std_logic := '0';
        
        -- Number of signals in Sync trigger
        sync_trigger_count      : natural range 2 to 8 := 2;
        
        -- Number of signals in Sample trigger
        sample_trigger_count    : natural range 2 to 8 := 3
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and reset
        -----------------------------------------------------------------------
        signal clk_sys          : in    std_logic;
        signal res_n            : in    std_logic;

        -----------------------------------------------------------------------
        -- Trigger requests
        -----------------------------------------------------------------------
        signal sample_req       : in    std_logic;
        signal sync_req         : in    std_logic;

        -- Sampling type (Nominal, Data)
        signal sp_control       : in    std_logic_vector(1 downto 0);
        
        -----------------------------------------------------------------------
        -- Trigger outputs
        -----------------------------------------------------------------------
        -- Sample
        signal sample_nbt : out std_logic_vector(sample_trigger_count - 1 downto 0);
        signal sample_dbt : out std_logic_vector(sample_trigger_count - 1 downto 0);
        
        -- Sync
        signal sync_nbt : out std_logic_vector(sync_trigger_count - 1 downto 0);
        signal sync_dbt : out std_logic_vector(sync_trigger_count - 1 downto 0)
    );
end entity;

architecture rtl of trigger_generator is

    -- Shift registers for Trigger generation
    signal sync_sr      :   std_logic_vector(sync_trigger_count - 2 downto 0);
    signal sample_sr    :   std_logic_vector(sample_trigger_count - 2 downto 0);

    -- Shift registers for Trigger generation are equal to zeroes
    signal sync_sr_empty    : std_logic;
    signal sample_sr_empty  : std_logic;

    -- Trigger request flags. Set when a request for trigger arrives and
    -- another trigger is still in progress
    signal sync_req_flag_d    : std_logic;
    signal sync_req_flag_q    : std_logic;
    signal sync_req_flag_dq   : std_logic;

    signal sample_req_flag_d  : std_logic;
    signal sample_req_flag_q  : std_logic;
    signal sample_req_flag_dq : std_logic;

    -- Trigger signals (internal values)
    signal sync_trig_i      : std_logic;
    signal sample_trig_i    : std_logic;

    -- Constants with zero values for shift registers
    constant sync_sr_zeroes : std_logic_vector(sync_trigger_count - 2 downto 0) :=
        (OTHERS => '0');
    constant sample_sr_zeroes : std_logic_vector(sample_trigger_count - 2 downto 0) :=
        (OTHERS => '0');

begin
    
    ---------------------------------------------------------------------------
    -- Sync trigger capture register
    ---------------------------------------------------------------------------
    sync_req_flag_d <= '1' when (sample_sr_empty = '0' and sync_req = '1') else
                       '0' when (sample_sr_empty = '1') else
                       sync_req_flag_q;

    sync_req_flag_proc : process(clk_sys, res_n)
    begin
        if (res_n = reset_polarity) then
            sync_req_flag_q <= '0';
        elsif (rising_edge(clk_sys)) then
            sync_req_flag_q <= sync_req_flag_d;
        end if;
    end process;

    sync_req_flag_dq <= sync_req or sync_req_flag_q;


    ---------------------------------------------------------------------------
    -- Sample trigger capture register
    ---------------------------------------------------------------------------
    sample_req_flag_d <= '1' when (sync_sr_empty = '0' and sample_req = '1') else
                         '0' when (sync_sr_empty = '1') else
                         sample_req_flag_q;

    sample_req_flag_proc : process(clk_sys, res_n)
    begin
        if (res_n = reset_polarity) then
            sample_req_flag_q <= '0';
        elsif (rising_edge(clk_sys)) then
            sample_req_flag_q <= sample_req_flag_d;
        end if;
    end process;

    sample_req_flag_dq <= sample_req or sample_req_flag_q;
    

    ---------------------------------------------------------------------------
    -- Decoding internal value of triggers. Allow for trigger to propagate
    -- only if the other trigger has already passed!
    ---------------------------------------------------------------------------
    sync_trig_i <= '1' when (sync_req_flag_dq = '1' and sample_sr_empty = '1') else
                   '0';
    sample_trig_i <= '1' when (sample_req_flag_dq = '1' and sync_sr_empty = '1') else
                     '0';

    ---------------------------------------------------------------------------
    -- Decoding if pipeline trigger registers are empty or not
    ---------------------------------------------------------------------------
    sync_sr_empty <= '1' when (sync_sr = sync_sr_zeroes) else
                     '0';
    sample_sr_empty <= '1' when (sample_sr = sample_sr_zeroes) else
                       '0';

    ---------------------------------------------------------------------------
    -- Shift register instances
    ---------------------------------------------------------------------------
    
    -- Highest bit of shift register
    trig_sr_highest_bit_proc : process(res_n, clk_sys)
    begin
        if (res_n = reset_polarity) then
            sync_sr(sync_sr'length - 1)       <= '0';
            sample_sr(sample_sr'length - 1)   <= '0';
        elsif (rising_edge(clk_sys)) then
            sync_sr(sync_sr'length - 1)     <= sync_trig_i;
            sample_sr(sample_sr'length - 1) <= sample_trig_i;
        end if;
    end process;
    
    -- Next bits, instantiated only when there are more than 2 triggers
    sync_trig_sr_gen : if (sync_trigger_count > 2) generate
        sync_trig_sr_proc : process(clk_sys, res_n)
        begin
            if (res_n = reset_polarity) then
                sync_sr(sync_sr'length - 2 downto 0) <= (OTHERS => '0');
            elsif (rising_edge(clk_sys)) then
                sync_sr(sync_sr'length - 2 downto 0) <=
                    sync_sr(sync_sr'length - 1 downto 1);
            end if;
        end process;
    end generate sync_trig_sr_gen;

    sample_trig_sr_gen : if (sample_trigger_count > 2) generate
        sample_trig_sr_proc : process(clk_sys, res_n)
        begin
            if (res_n = reset_polarity) then
                sample_sr(sample_sr'length - 2 downto 0) <= (OTHERS => '0');
            elsif (rising_edge(clk_sys)) then
                sample_sr(sample_sr'length - 2 downto 0) <=
                    sample_sr(sample_sr'length - 1 downto 1);
            end if;
        end process;
    end generate sample_trig_sr_gen;

    ---------------------------------------------------------------------------
    -- Propagation of trigger signals to output!
    ---------------------------------------------------------------------------

    -- First triggers (direct output, not from shift register)
    sync_nbt(sync_trigger_count - 1) <=
        sync_trig_i when (sp_control = NOMINAL_SAMPLE) else
        '0';

    sync_dbt(sync_trigger_count - 1) <=
        sync_trig_i when (sp_control = DATA_SAMPLE or
                          sp_control = SECONDARY_SAMPLE)
                    else
        '0';
    
    sample_nbt(sample_trigger_count - 1) <=
        sample_trig_i when (sp_control = NOMINAL_SAMPLE) else
        '0';

    sample_dbt(sample_trigger_count - 1) <=
        sample_trig_i when (sp_control = DATA_SAMPLE or
                            sp_control = SECONDARY_SAMPLE)
                      else
        '0';
        
    -- Next triggers (outputs from shift registers)
    sync_nbt(sync_trigger_count - 2 downto 0) <=
        sync_sr(sync_trigger_count - 2 downto 0)
        when (sp_control = NOMINAL_SAMPLE) else
        (OTHERS => '0');
        
    sync_dbt(sync_trigger_count - 2 downto 0) <=
        sync_sr(sync_trigger_count - 2 downto 0) 
        when (sp_control = DATA_SAMPLE or sp_control = SECONDARY_SAMPLE) else
        (OTHERS => '0');

    sample_nbt(sample_trigger_count - 2 downto 0) <=
        sample_sr(sample_trigger_count - 2 downto 0)
        when (sp_control = NOMINAL_SAMPLE) else
        (OTHERS => '0');
        
    sample_dbt(sample_trigger_count - 2 downto 0) <=
        sample_sr(sample_trigger_count - 2 downto 0)
        when (sp_control = DATA_SAMPLE or sp_control = SECONDARY_SAMPLE) else
        (OTHERS => '0');

    
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
    --  (sample_req = '1' and sync_req = '1')
    --  report "Sync and Sample trigger should no be requested at once!"
    --  severity error;
    ---------------------------------------------------------------------------
    
    ---------------------------------------------------------------------------
    -- If any request is captured, there should not come another request of
    -- the same type! Only other trigger type request might come!
    --
    -- psl sync_req_flag_overrun_asrt : assert never
    --  (sync_req = '1' and sync_req_flag_q = '1')
    --  report "Sync request should not occur when previous is not finished!"
    --  severity error;
    --
    -- psl sample_req_flag_overrun_asrt : assert never
    --  (sample_req = '1' and sample_req_flag_q = '1')
    --  report "Sample request should not occur when previous is not finished!"
    --  severity error;
    --
    ---------------------------------------------------------------------------
    
    ---------------------------------------------------------------------------
    -- If any trigger is not yet fully generated (there are non-zeroes in its
    -- shift register), there should never come other request of the same type.
    --
    -- psl sync_req_and_sync_sr_non_zero_asrt : assert never
    --  (sync_req = '1' and sync_sr_empty = '0')
    --  report "Sync request should not occur if its shift register is not empty!"
    --  severity error;
    --
    -- psl sample_req_and_sync_sr_non_zero_asrt : assert never
    --  (sample_req = '1' and sample_sr_empty = '0')
    --  report "Sample request should not occur if its shift register is not empty!"
    --  severity error;
    ---------------------------------------------------------------------------
        
end architecture rtl;