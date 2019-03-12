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
--  Prescaler model.
--
--  Models Bit time State (TSEG1 and TSEG2). At the moment does not model
--  SYNC and SAMPLE sequences. Takes into account:
--   1. Nominal Bit Time and Data Bit time
--   2. Bit rate shift (both ways)
--   3. Re-synchronisation (also during Bit rate shift)
--   4. Hard synchronisation
--
--  Model consists of three processes:
--   1. TSEG1 process
--   2. TSEG2 NBT process
--   3. TSEG2 DBT process
--  TSEG1 process communicates with TSEG1 and TSEG2 via handshake mechanism.
--  TSEG1 manippulates Bit time State. When TSEG1 ends, it changes Bit time
--  state and issues request to both TSEG2 processes. Both TSEG2 processes count
--  in parallel and issue an ACK when they finish. Both TSEG2 issue ACK only
--  when they finish in correct Sampling Type (sp_control).
--
--  Each process holds its own flags about Hard sync. edge occurence and
--  edge occurence during its bit segment. These flags are set by process,
--  and cleared when Bit time is entering TSEG2 (Sample point)!
--  These flags are also set for TSEG1 process when edge occured in TSEG2, so
--  that resync edge would not be considered more times between two sample
--  points!
--
--  TODO: Model does not implement trigger sequences so far!
--                                                                 
--------------------------------------------------------------------------------
-- Revision History:
--
--    04.03.2019  First implemenation.
--    09.03.2019  Debugged, added info prints!  
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library work;
use work.can_constants.all;
use work.can_types.all;
use work.drv_stat_pkg.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

library vunit_lib;
context vunit_lib.vunit_context;

entity prescaler_model is
    generic(
      -- Reset polarity
      reset_polarity        :   std_logic := '0';
      
      -- Clock period
      clock_period          :   time := 10 ns
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and async reset
        -----------------------------------------------------------------------
        signal clk_sys              :in std_logic;
        signal res_n                :in std_logic;
    
        -----------------------------------------------------------------------
        -- Bus synch Interface
        -----------------------------------------------------------------------
        signal sync_edge            :in std_logic;
        signal OP_State             :in oper_mode_type;
        
        -- Driving Bus
        signal drv_bus              :in std_logic_vector(1023 downto 0); 
        
        -- Bit time FSM output
        signal bt_FSM_out           :out bit_time_type;
    
        -- What is actual node transmitting on the bus
        signal data_tx              :in   std_logic;
    
        -----------------------------------------------------------------------
        -- Bit time and Synchronisation config
        -----------------------------------------------------------------------
        signal sp_control           :in std_logic_vector(1 downto 0);
        signal sync_control         :in std_logic_vector(1 downto 0)
  );
end entity;

architecture func of prescaler_model is

    ---------------------------------------------------------------------------
    -- Bit timing configuration
    ---------------------------------------------------------------------------
    -- Nominal
    signal brp_nbt      : integer := 1;
    signal tseg1_nbt    : integer := 1;
    signal tseg2_nbt    : integer := 1;
    signal sjw_nbt      : integer := 1;
    
    -- Data
    signal brp_dbt      : integer := 1;
    signal tseg1_dbt    : integer := 1;
    signal tseg2_dbt    : integer := 1;
    signal sjw_dbt      : integer := 1;

    -- Selected
    signal brp_s      : integer;
    signal tseg1_s    : integer;
    signal tseg2_s    : integer;
    signal sjw_s      : integer;

    signal bt_fsm       : bit_time_type := reset;

    -- Time quanta counters
    signal tq_ctr_nbt   : integer := 0;
    signal tq_ctr_dbt   : integer := 0;

    -- Time quanta edges
    signal tq_edge_nbt : std_logic := '0';
    signal tq_edge_dbt : std_logic := '0';

    signal tq_edge     : std_logic;
    
    ---------------------------------------------------------------------------
    -- Handshake between main Bit time process and TSEG2 processes for
    -- Nominal and Data processes
    ---------------------------------------------------------------------------
    signal tseg2_nbt_req : boolean := false;
    signal tseg2_dbt_req : boolean := false;
            
    signal tseg2_nbt_ack : boolean := false;
    signal tseg2_dbt_ack : boolean := false;
    
    signal drv_ena : std_logic;
    
    ---------------------------------------------------------------------------
    -- Signals for processes that are used to count duration of
    -- segment lengths
    ---------------------------------------------------------------------------
    signal tseg1_i           : integer := 0;
    signal tseg1_exp_length  : integer := 0;
    
    signal tseg2_nbt_i           : integer := 0;
    signal tseg2_nbt_exp_length  : integer := 0;
    
    signal tseg2_dbt_i           : integer := 0;
    signal tseg2_dbt_exp_length  : integer := 0;
    
    ---------------------------------------------------------------------------
    -- Declare protected boolean
    ---------------------------------------------------------------------------
    type p_boolean is protected
        impure function get return boolean;
        procedure set(set_val : boolean);
    end protected p_boolean;
    
    type p_boolean is protected body
    
        variable val : boolean := false;
        
        impure function get return boolean is
        begin
            return val;
        end function get;
        
        procedure set(set_val : boolean) is
        begin
            val := set_val;
        end procedure;
        
    end protected body p_boolean;
    
    -- Edge occurence for TSEG1
    shared variable h_sync_occured_tseg1  :   p_boolean;
    shared variable edge_occured_tseg1    :   p_boolean;
    
    -- Edge occurence for TSEG2 NBT
    shared variable h_sync_occured_tseg2_nbt : p_boolean;
    shared variable edge_occured_tseg2_nbt   : p_boolean;
    
    -- Edge occurence for TSEG2 DBT
    shared variable h_sync_occured_tseg2_dbt : p_boolean;
    shared variable edge_occured_tseg2_dbt   : p_boolean;
    
    ---------------------------------------------------------------------------
    -- Segment counting. Counts duration of TSEG1 or TSEG2.
    -- Holds info about occured edge, or valid Hard synchronisation edge.
    ---------------------------------------------------------------------------
    procedure count_segment (
        signal   clk_sys         : in     std_logic;
        signal   tq_edge         : in     std_logic;
        signal   sp_control      : in     std_logic_vector(1 downto 0);
        signal   sync_edge       : in     std_logic;
        signal   sync_control    : in     std_logic_vector(1 downto 0);
        signal   bt_fsm          : in     bit_time_type;
        signal   nom_dur         : in     integer;
        signal   brp             : in     integer;
        signal   sjw             : in     integer;
        signal   exp_duration    : inout  integer;
        signal   i               : inout  integer;
                 h_sync_occured  : inout  p_boolean;
                 edge_occured    : inout  p_boolean
    ) is
        variable e_tseg2 : integer;
    begin
        
        exp_duration <= nom_dur;
        i <= 0;
        wait for 0 ns;
        
        while (i < exp_duration) loop
            wait until (rising_edge(clk_sys) and tq_edge = '1') or
                         (drv_ena = '0') or (bt_fsm'active);

            i <= i + 1;

            if (drv_ena = '0') then
                info("model: Prescaler turned off, Exiting time segment counting!");
                exit;
            end if;
            
            -- Finish if current bit segment is finished
            -- (e.g. due to another process)
            if (bt_fsm'active) then
                exit;
            end if;
            
            -------------------------------------------------------------------
            -- If edge occured in the last time quanta.
            -------------------------------------------------------------------
            if (sync_edge'last_event < brp * clock_period) then

                info("model: Sync edge processing entered!");
                 
                -- Don't consider it for NO_SYNC
                if (sync_control = NO_SYNC) then
                    info("model: Ignoring edge, NO_SYNC!");
                    wait for 0 ns;
                    next;
                end if;

                -- Quit immediately for HARD_SYNC
                if (sync_control = HARD_SYNC and h_sync_occured.get = false and
                    edge_occured.get = false)
                then
                    info("model: Processing as Hard-sync!");
                    h_sync_occured.set(true);
                    edge_occured.set(true);
                    wait for 0 ns;
                    exit;
                end if;
                
                if (sync_control = RE_SYNC) then
                    -- Skip if Positive re-sync is not forbidden
                    if (bt_fsm = tseg1 and data_tx = DOMINANT and
                        OP_state = transciever)
                    then
                        info("model: No pos resync for transmitter!");
                        wait for 0 ns;
                        next;
                    end if;
    
                    info("model: RE-SYNC");
                    info("model: Checking if SYNC edge occured!");
    
                    -- Skip if there was already previous edge since sample point! 
                    if (edge_occured.get) then
                        info("model: Skipping re-sync, already resynchronised!");
                        wait for 0 ns;
                        next;
                    end if;
                    
                    info("model: Edge not occured, processing!");
                    edge_occured.set(true);
    
                    -- Now we can truly re-synchronise!
                    if (bt_fsm = tseg1) then
                        if (i > sjw) then
                            exp_duration <= exp_duration + sjw;
                        else
                            exp_duration <= exp_duration + i;
                        end if;
                    else
                        e_tseg2 := exp_duration - i;
                        if (e_tseg2 > sjw) then
                            exp_duration <= exp_duration - sjw;
                        else
                            exp_duration <= exp_duration - e_tseg2;
                        end if;
                    end if;
                    
                    wait for 0 ns;
                end if;
                
            end if;
            wait for 0 ns;
            
            -------------------------------------------------------------------
            -- Here restart the hard-synchronisation If we counted till the end!
            -- This-way, even if hard sync. occured once, we are able to continue
            -- with TSEG2 properly!
            -------------------------------------------------------------------
            if (i = exp_duration) then
                h_sync_occured.set(false);
            end if;
    
        end loop;
        
    end procedure;
    
    ---------------------------------------------------------------------------
    -- Counts Time quanta
    ---------------------------------------------------------------------------
    procedure count_tq(
      signal tq_ctr :  inout integer;
      signal brp    :  in    integer  
    ) is
    begin
        if (rising_edge(clk_sys)) then
            if (tq_ctr = brp - 1 or brp = 1) then
                tq_ctr <= 0;
            else
                tq_ctr <= tq_ctr + 1;
            end if;
        end if;
        
        if (bt_fsm'event) then
            tq_ctr <= 0;
        end if;

        if (drv_ena /= '1') then
            tq_ctr <= 0;
        end if;
    end procedure;
    
    ---------------------------------------------------------------------------
    -- Set time quanta edges
    ---------------------------------------------------------------------------
    procedure set_tq_edge(
        signal tq_edge  : inout std_logic;
        signal tq_ctr   : in    integer;
        signal brp      : in    integer
    ) is
    begin
        if (bt_fsm'event and brp /= 1) then
            tq_edge <= '0';
        end if;
        
        if (rising_edge(clk_sys)) then
            if ((tq_ctr = brp - 2) or brp = 1) then
                tq_edge <= '1';
            else
                tq_edge <= '0';
            end if;
        end if;

        if (drv_ena /= '1') then
            if (brp > 1) then
                tq_edge <= '0';
            else
                tq_edge <= '1';    
            end if;
        end if;
    end procedure;
        
begin

    drv_ena <= drv_bus(DRV_ENA_INDEX);

    ---------------------------------------------------------------------------
    -- Bit time settings
    ---------------------------------------------------------------------------
    brp_nbt <= to_integer(unsigned(drv_bus(DRV_TQ_NBT_HIGH downto DRV_TQ_NBT_LOW)));
    brp_dbt <= to_integer(unsigned(drv_bus(DRV_TQ_DBT_HIGH downto DRV_TQ_DBT_LOW)));
    
    tseg1_nbt <= to_integer(unsigned(drv_bus(DRV_PRS_NBT_HIGH downto DRV_PRS_NBT_LOW))) +
                 to_integer(unsigned(drv_bus(DRV_PH1_NBT_HIGH downto DRV_PH1_NBT_LOW))) + 1;
    
    tseg2_nbt <= to_integer(unsigned(drv_bus(DRV_PH2_NBT_HIGH downto DRV_PH2_NBT_LOW)));
    
    
    tseg1_dbt <= to_integer(unsigned(drv_bus(DRV_PRS_DBT_HIGH downto DRV_PRS_DBT_LOW))) +
                 to_integer(unsigned(drv_bus(DRV_PH1_DBT_HIGH downto DRV_PH1_DBT_LOW))) + 1;
    
    tseg2_dbt <= to_integer(unsigned(drv_bus(DRV_PH2_DBT_HIGH downto DRV_PH2_DBT_LOW)));
    
    sjw_nbt <= to_integer(unsigned(drv_bus(DRV_SJW_HIGH downto DRV_SJW_LOW)));
    sjw_dbt <= to_integer(unsigned(drv_bus(DRV_SJW_DBT_HIGH downto DRV_SJW_DBT_LOW)));
    
    
    ---------------------------------------------------------------------------
    -- Selection based on Sample control (Nominal, Data)
    ---------------------------------------------------------------------------
    brp_s <= brp_nbt when (sp_control = NOMINAL_SAMPLE) else
             brp_dbt;
           
    tseg1_s <= tseg1_nbt when (sp_control = NOMINAL_SAMPLE) else
               tseg1_dbt;
    
    tseg2_s <= tseg2_nbt when (sp_control = NOMINAL_SAMPLE) else
               tseg2_dbt;
    
    sjw_s <= sjw_nbt when (sp_control = NOMINAL_SAMPLE) else
             sjw_dbt;
    
    
    ---------------------------------------------------------------------------
    -- Time quanta counter Nominal
    ---------------------------------------------------------------------------
    tq_nbt_edge_proc : process(clk_sys, bt_fsm, drv_ena, brp_nbt)
    begin
        set_tq_edge(tq_edge_nbt, tq_ctr_nbt, brp_nbt);
    end process;
    
    tq_nbt_proc : process(clk_sys, bt_fsm, drv_ena, brp_nbt)
    begin
        count_tq(tq_ctr_nbt, brp_nbt);
    end process;


    ---------------------------------------------------------------------------
    -- Time quanta counter data
    ---------------------------------------------------------------------------
    tq_dbt_edge_proc : process(clk_sys, bt_fsm, drv_ena, brp_dbt)
    begin
        set_tq_edge(tq_edge_dbt, tq_ctr_dbt, brp_dbt);
    end process;

    tq_dbt_proc : process(clk_sys, bt_fsm, drv_ena, brp_dbt)
    begin
        count_tq(tq_ctr_dbt, brp_dbt);
    end process;
    
    
    tq_edge <= tq_edge_nbt when (sp_control = NOMINAL_SAMPLE) else
               tq_edge_dbt;    


    ---------------------------------------------------------------------------
    -- Bit time process
    -- Counts TSEG1 and starts TSEG2 processes for NBT and for DBT.
    ---------------------------------------------------------------------------
    bt_proc : process
        variable exp_dur      : integer := 0;
        variable i            : integer;
    begin
        
        if (drv_ena = '1') then

            bt_fsm <= tseg1;
           
            -- Execute segment 1 
            count_segment (
                clk_sys => clk_sys,
                tq_edge => tq_edge,
                sp_control => sp_control,
                sync_edge => sync_edge,
                sync_control => sync_control,
                bt_fsm => bt_fsm,
                nom_dur => tseg1_s,
                brp => brp_s,
                sjw => sjw_s,
                exp_duration => tseg1_exp_length,
                i => tseg1_i,
                h_sync_occured => h_sync_occured_tseg1,
                edge_occured => edge_occured_tseg1
            );
    
            -- Go to the second segment only if Hard sync did not occur, 
            -- if it occur, we restart from TSEG1!
            if (h_sync_occured_tseg1.get = false and drv_ena = '1') then   
            
                -- Here we are at the moment of sample point. We have to 
                -- reset flags about occured synchronisation edges for 
                -- each process!
                edge_occured_tseg1.set(false);
                edge_occured_tseg2_nbt.set(false);
                edge_occured_tseg2_dbt.set(false);
                
                h_sync_occured_tseg2_nbt.set(false);
                h_sync_occured_tseg2_dbt.set(false);

                bt_fsm <= tseg2;
            
                tseg2_nbt_req <= true;
                tseg2_dbt_req <= true;
                
                -- Wait for response from TSEG2 counters
                -- (both nominal and data)
                wait until (tseg2_nbt_ack or tseg2_dbt_ack);

                ---------------------------------------------------------------
                -- Here we have to mark for TSEG1 process that edge occured
                -- since in either of TSEG2 processes.
                ---------------------------------------------------------------
                if (sp_control = NOMINAL_SAMPLE) then
                    edge_occured_tseg1.set(edge_occured_tseg2_nbt.get);
                    h_sync_occured_tseg1.set(h_sync_occured_tseg2_nbt.get);
                else
                    edge_occured_tseg1.set(edge_occured_tseg2_dbt.get);
                    h_sync_occured_tseg1.set(h_sync_occured_tseg2_dbt.get);
                end if;

                tseg2_nbt_req <= false;
                tseg2_dbt_req <= false;
            end if;
        end if;
        
        if (drv_ena /= '1') then
            tseg2_nbt_req <= false;
            tseg2_dbt_req <= false;
            bt_fsm <= reset;
            wait until drv_ena = '1';
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Time segment 2 process - Nominal
    ---------------------------------------------------------------------------
    tseg2_nbt_proc : process
        variable exp_dur : integer := 0;
        variable i       : integer := 0;
    begin
        if (drv_ena = '1') then
            wait until tseg2_nbt_req = true;
            tseg2_nbt_ack <= false;
            
            count_segment (
                clk_sys => clk_sys,
                tq_edge => tq_edge_nbt,
                sp_control => sp_control,
                sync_edge => sync_edge,
                sync_control => sync_control,
                bt_fsm => bt_fsm,
                nom_dur => tseg2_nbt,
                brp => brp_nbt,
                sjw => sjw_nbt,
                exp_duration => tseg2_nbt_exp_length,
                i => tseg2_nbt_i,
                h_sync_occured => h_sync_occured_tseg2_nbt,
                edge_occured => edge_occured_tseg2_nbt
            );
            
            if (drv_ena = '1' and sp_control = NOMINAL_SAMPLE) then
                tseg2_nbt_ack <= true;
            end if;
        
            if (drv_ena = '1' and tseg2_nbt_req /= false) then
                wait until tseg2_nbt_req = false or drv_ena = '0';
            end if;
        
        end if;
        
        if (drv_ena /= '1') then
            tseg2_nbt_ack <= false;
            wait until drv_ena = '1';
        end if;
    end process;
     
     
    ---------------------------------------------------------------------------
    -- Time segment 2 process - Data
    ---------------------------------------------------------------------------
    tseg2_dbt_proc : process
    begin
        if (drv_ena = '1') then
            wait until tseg2_dbt_req = true;
            tseg2_dbt_ack <= false;
            
            count_segment (
                clk_sys => clk_sys,
                tq_edge => tq_edge_dbt,
                sp_control => sp_control,
                sync_edge => sync_edge,
                sync_control => sync_control,
                bt_fsm => bt_fsm,
                nom_dur => tseg2_dbt,
                brp => brp_dbt,
                sjw => sjw_dbt,
                exp_duration => tseg2_dbt_exp_length,
                i => tseg2_dbt_i,
                h_sync_occured => h_sync_occured_tseg2_dbt,
                edge_occured => edge_occured_tseg2_dbt
            );
            if (drv_ena = '1' and (sp_control = DATA_SAMPLE or
                                   sp_control = SECONDARY_SAMPLE))
            then
                tseg2_dbt_ack <= true;
            end if;
            
            if (drv_ena = '1' and tseg2_dbt_req /= false) then
                wait until tseg2_dbt_req = false or drv_ena = '0';
            end if;
        end if;
        
        if (drv_ena /= '1') then
            tseg2_dbt_ack <= false;
            wait until drv_ena = '1';
        end if;
    end process;
    
    bt_FSM_out <= bt_fsm;
    
end architecture;