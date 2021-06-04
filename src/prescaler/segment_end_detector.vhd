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
--  End of segment detector.
--
-- Purpose:
--  Detects end of current segment (TSEG1 or TSEG2) as a result of Hard-sync.,
--  or request from Bit segment meter. Provides signal for clearing Bit Time 
--  counters. Only requests from Bit segment meter module which matches current
--  Bit-rate is considered (Nominal resynchronisation is considered in Nominal
--  Bit-rate, Data resynchronisation is considered in Data Bit-rate).
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

entity segment_end_detector is
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys            : in    std_logic;
        
        -- Asynchronous reset
        res_n              : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Control interface
        -----------------------------------------------------------------------
        -- Sample control (Nominal, Data, Secondary)
        sp_control         : in    std_logic_vector(1 downto 0);
        
        -- Hard synchronisation edge is valid
        h_sync_edge_valid  : in    std_logic;
        
        -- Segment end request (Nominal)
        exit_segm_req_nbt  : in    std_logic;
        
        -- Segment end request (Data)
        exit_segm_req_dbt  : in    std_logic;

        -- Bit time FSM is in TSEG1
        is_tseg1           : in    std_logic;
        
        -- Bit time FSM is in TSEG2
        is_tseg2           : in    std_logic;

        -- Nominal Time quanta is active
        tq_edge_nbt        : in    std_logic;
        
        -- Data Time quanta is active
        tq_edge_dbt        : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Status signals
        -----------------------------------------------------------------------
        -- Segment end
        segm_end           : out   std_logic;
        
        -- Hard Synchronisation is valid
        h_sync_valid       : out   std_logic;
        
        -- Clear Bit time counters
        bt_ctr_clear       : out   std_logic
    );
end entity;

architecture rtl of segment_end_detector is

    ---------------------------------------------------------------------------
    -- Registers to capture requests for Hard-sync (0),
    -- NBT end of segment (1), DBT end of segment (2)
    ---------------------------------------------------------------------------
    signal req_input                : std_logic_vector(2 downto 1);
    signal segm_end_req_capt_d      : std_logic_vector(2 downto 1);
    signal segm_end_req_capt_q      : std_logic_vector(2 downto 1);
    signal segm_end_req_capt_ce     : std_logic_vector(2 downto 1);
    signal segm_end_req_capt_clr    : std_logic_vector(2 downto 1);

    -- ORed flags and combinational requests
    signal segm_end_req_capt_dq     : std_logic_vector(2 downto 1);
    
    -- Valid requests to end segment for each Sample type (Nominal, Data)
    signal segm_end_nbt_valid       : std_logic;
    signal segm_end_dbt_valid       : std_logic;
    signal segm_end_nbt_dbt_valid   : std_logic;
 
    -- Combinational requests to finish segment.
    signal tseg1_end_req_valid      : std_logic;
    signal tseg2_end_req_valid      : std_logic;
    signal h_sync_valid_i           : std_logic;

    -- End of segment, internal value
    signal segment_end_i            : std_logic;
    
    -- Nominal / Data Time quanta are active
    signal nbt_tq_active            : std_logic;
    signal dbt_tq_active            : std_logic;
    
    -- Bit time clear - internal value
    signal bt_ctr_clear_i           : std_logic;

begin

    ----------------------------------------------------------------------------
    -- End of segment request capturing:
    --  1. NBT Resynchronisation requests segment end
    --  2. DBT Resynchronisation requests segment end
    ----------------------------------------------------------------------------
    req_input(1) <= exit_segm_req_nbt;
    req_input(2) <= exit_segm_req_dbt;
    
    ----------------------------------------------------------------------------
    -- Clearing requests:
    --  2. Segment end.
    --  3. Segment end.
    ----------------------------------------------------------------------------
    segm_end_req_capt_clr(1) <= segment_end_i;
    segm_end_req_capt_clr(2) <= segment_end_i;

    segm_end_req_capture : for i in 1 to 2 generate
    begin
        
        -- Clear the flag upon real end of segment!
        segm_end_req_capt_d(i) <= '0' when (segm_end_req_capt_clr(i) = '1') else
                                  req_input(i);
        
        segm_end_req_capt_ce(i) <=
            '1' when (segm_end_req_capt_clr(i) = '1' or req_input(i) = '1') else
            '0';
        
        end_of_segm_req_proc : process(clk_sys, res_n)
        begin
            if (res_n = '0') then
                segm_end_req_capt_q(i) <= '0';
            elsif (rising_edge(clk_sys)) then
                if (segm_end_req_capt_ce(i) = '1') then
                    segm_end_req_capt_q(i) <= segm_end_req_capt_d(i);
                end if;
            end if;
        end process;
        
    end generate;
    
    ---------------------------------------------------------------------------
    -- Segment end request from NBT and DBT resynchronisation is valid
    -- for each Bit segment differently.
    -- For TSEG1:
    --  1. Combinational is valid! Here the request hangs (it is always due
    --     to comparison with Bit time counter, so it does not have to be
    --     captured)!
    -- For TSEG2:
    --  2. Combinational is valid, or captured request is valid. This accounts
    --     for edge in the same clock cycle, as well as immediate exit occured
    --     in previous clock cycle during previous Time quanta which was
    --     captured and is not present anymore! 
    ---------------------------------------------------------------------------
    segm_end_req_capt_dq(1) <= req_input(1) when (is_tseg1 = '1') else
                               req_input(1) OR segm_end_req_capt_q(1);

    segm_end_req_capt_dq(2) <= req_input(2) when (is_tseg1 = '1') else
                               req_input(2) OR segm_end_req_capt_q(2);

    ---------------------------------------------------------------------------
    -- Nominal and Data Time Quanta are active only when corresponding Sample
    -- type is set!
    ---------------------------------------------------------------------------
    nbt_tq_active <= '1' when (sp_control = NOMINAL_SAMPLE and tq_edge_nbt = '1') else
                     '0';
 
    dbt_tq_active <= '1' when (tq_edge_dbt = '1' and (sp_control = DATA_SAMPLE or
                                                      sp_control = SECONDARY_SAMPLE))
                         else
                     '0';
 
    ---------------------------------------------------------------------------
    -- Request to finish from either Nominal Bit-Rate re-synchronisation
    -- or Data Re-synchronisation is valid when Sample control has Nominal
    -- or Data, Secondary sampling set!
    ---------------------------------------------------------------------------
    segm_end_nbt_valid <=
        '1' when (segm_end_req_capt_dq(1) = '1' and nbt_tq_active = '1')
            else
        '0';
 
    segm_end_dbt_valid <=
        '1' when (segm_end_req_capt_dq(2) = '1' and dbt_tq_active = '1')
            else
        '0';
    
    segm_end_nbt_dbt_valid <=
        '1' when (segm_end_nbt_valid = '1' or segm_end_dbt_valid = '1')
            else
        '0';
 
    ---------------------------------------------------------------------------
    -- Time segment end requests.
    ---------------------------------------------------------------------------
    tseg1_end_req_valid <=
        '1' when (is_tseg1 = '1' and segm_end_nbt_dbt_valid = '1') else
        '0';

    tseg2_end_req_valid <=
        '1' when (is_tseg2 = '1' and segm_end_nbt_dbt_valid = '1')
            else
        '0';
    
    ---------------------------------------------------------------------------
    -- Align Hard synchronisation request with Time Quanta. Note that Hard sync.
    -- is only allowed in Nominal Bit-rat, thus use only Nominal Time Quanta edge!
    ---------------------------------------------------------------------------
    h_sync_valid_i <=
        '1' when ((h_sync_edge_valid = '1') and
                  (nbt_tq_active = '1'))
            else
        '0';

    ---------------------------------------------------------------------------
    -- Overall segment end request occurs due to following conditions:
    --  1. Nominal Bit Time Resynchronisation signals end of segment, Nominal
    --     Time Quanta edge and Sample control is NOMINAL_SAMPLE.
    --  2. Data Bit Time Resynchronisation signals end of segment, Data
    --     Time Quanta edge and Sample control is either DATA_SAMPLE or
    --     SECONDARY_SAMPLE!
    --  3. Hard synchronisation induced end of segment in TSEG2! In TSEG1
    --     segment is not ended, only Bit Time counter is restarted!
    ---------------------------------------------------------------------------
    segment_end_i <= '1' when ((tseg1_end_req_valid = '1' and h_sync_valid_i = '0') or
                               tseg2_end_req_valid = '1' or
                               (h_sync_valid_i = '1' and is_tseg2 = '1'))
                         else
                     '0';

    ---------------------------------------------------------------------------
    -- Bit time counter clear:
    --  1. Segment end.
    --  2. Hard sync is valid. This covers the case when Hard-sync edge
    --     occurs in TSEG1 and TSEG1 does not end, it just gets re-started
    --     (bit time counter will be cleared)!
    ---------------------------------------------------------------------------
    bt_ctr_clear_i <= '1' when (segment_end_i = '1' or h_sync_valid_i = '1')
                          else
                      '0';

    bt_ctr_clear    <= bt_ctr_clear_i;
    segm_end        <= segment_end_i;
    h_sync_valid    <= h_sync_valid_i;
 
 
    -- <RELEASE_OFF>
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    -- psl default clock is rising_edge(clk_sys);

    -- psl no_h_sync_not_in_time_quanta : assert never
    --  (h_sync_edge_valid = '1' and tq_edge_nbt = '0');

    -- psl segm_end_req_1_capt_cov : cover
    --  {segm_end_req_capt_q(1) = '1'};
    
    -- psl segm_end_req_2_capt_cov : cover
    --  {segm_end_req_capt_q(2) = '1'};
    
    -- <RELEASE_ON>
 
end architecture rtl;