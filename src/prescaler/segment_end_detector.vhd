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
--  End of segment detector. Detects end of current segment (TSEG1 or TSEG2)
--  as a result of Hard-synchronisation, or request from Re-synchronisation.
--  Provides signal for clearing Bit Time counters.
--
--------------------------------------------------------------------------------
-- Revision History:
--    15.02.2019   Created file
--    08.03.2019   Separated Segment end and Bit time counter clear. This-way
--                 we can distuinguish between first and second hard-sync
--                 edge in TSEG1.
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

entity segment_end_detector is
    generic (
        -- Reset polarity
        reset_polarity          :       std_logic := '0'
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and reset
        -----------------------------------------------------------------------
        signal clk_sys            : in    std_logic;
        signal res_n              : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Control interface
        -----------------------------------------------------------------------
        -- Sample point type
        signal sp_control         : in    std_logic_vector(1 downto 0);
        
        -- Hard synchronisation edge is valid
        signal h_sync_edge_valid  : in    std_logic;
        
        -- Nominal and Data Bit Time segment end request
        signal exit_segm_req_nbt  : in    std_logic;
        signal exit_segm_req_dbt  : in    std_logic;

        -- Bit time segments indication
        signal is_tseg1           : in    std_logic;
        signal is_tseg2           : in    std_logic;

        -- Time Quanta edges (Nominal and Data)
        signal tq_edge_nbt        : in    std_logic;
        signal tq_edge_dbt        : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Outputs - Decision that current segment should end!
        -----------------------------------------------------------------------
        signal segm_end           : out   std_logic;
        signal h_sync_valid       : out   std_logic;
        signal bt_ctr_clear       : out   std_logic
    );
end entity;

architecture rtl of segment_end_detector is

    ---------------------------------------------------------------------------
    -- Registers to capture requests for Hard-sync (0),
    -- NBT end of segment (1), DBT end of segment (2)
    ---------------------------------------------------------------------------
    signal req_input                : std_logic_vector(2 downto 0);
    signal segm_end_req_capt_d      : std_logic_vector(2 downto 0);
    signal segm_end_req_capt_q      : std_logic_vector(2 downto 0);
    signal segm_end_req_capt_ce     : std_logic_vector(2 downto 0);
    signal segm_end_req_capt_clr    : std_logic_vector(2 downto 0);

    -- ORed flags and combinational requests
    signal segm_end_req_capt_dq     : std_logic_vector(2 downto 0);
    
    -- Valid requests to end segment for each Sample type (Nominal, Data)
    signal segm_end_nbt_valid       : std_logic;
    signal segm_end_dbt_valid       : std_logic;
    signal segm_end_nbt_dbt_valid   : std_logic;
    
    -- Internally generated sample point
    signal sample_point             : std_logic;
 
    -- Combinational requests to finish segment.
    signal tseg1_end_req_valid      : std_logic;
    signal tseg2_end_req_valid      : std_logic;
    signal hsync_end_req_valid      : std_logic;

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
    --  1. Due to Hard sync.
    --  2. NBT Resynchronisation requests segment end
    --  3. DBT Resynchronisation requests segment end
    ----------------------------------------------------------------------------
    req_input(0) <= h_sync_edge_valid;
    req_input(1) <= exit_segm_req_nbt;
    req_input(2) <= exit_segm_req_dbt;
    
    ----------------------------------------------------------------------------
    -- Clearing requests:
    --  1. Upon any bit time clear (which is either Segment end or Hard-sync).
    --  2. Segment end.
    --  3. Segment end.
    ----------------------------------------------------------------------------
    segm_end_req_capt_clr(0) <= bt_ctr_clear_i;
    segm_end_req_capt_clr(1) <= segment_end_i;
    segm_end_req_capt_clr(2) <= segment_end_i;

    segm_end_req_capture : for i in 0 to 2 generate
    begin
        
        -- Clear the flag upon real end of segment!
        segm_end_req_capt_d(i) <= '0' when (segm_end_req_capt_clr(i) = '1') else
                                  req_input(i);
        
        segm_end_req_capt_ce(i) <=
            '1' when (segm_end_req_capt_clr(i) = '1' or req_input(i) = '1') else
            '0';
        
        end_of_segm_req_proc : process(clk_sys, res_n)
        begin
            if (res_n = reset_polarity) then
                segm_end_req_capt_q(i) <= '0';
            elsif (rising_edge(clk_sys)) then
                if (segm_end_req_capt_ce(i) = '1') then
                    segm_end_req_capt_q(i) <= segm_end_req_capt_d(i);
                end if;
            end if;
        end process;
        
    end generate;
 
    ---------------------------------------------------------------------------
    -- Hard synchronisation induced end of segment request is valid when
    -- both: combinational and captured requests are valid. This accounts
    -- for h-sync edge in the same clock cycle as well as captured from
    -- previous clock cycles in last time quanta!
    ---------------------------------------------------------------------------
    segm_end_req_capt_dq(0) <= req_input(0) OR segm_end_req_capt_q(0);
    
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
    hsync_end_req_valid <=
        '1' when ((segm_end_req_capt_dq(0) = '1') and
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
    segment_end_i <= '1' when ((tseg1_end_req_valid = '1' and hsync_end_req_valid = '0') or
                               tseg2_end_req_valid = '1' or
                               (hsync_end_req_valid = '1' and is_tseg2 = '1'))
                         else
                     '0';

    ---------------------------------------------------------------------------
    -- Bit time counter clear:
    --  1. Segment end.
    --  2. Hard sync is valid. This covers the case when Hard-sync edge
    --     occurs in TSEG1 and TSEG1 does not end, it just gets re-started
    --     (bit time counter will be cleared)!
    ---------------------------------------------------------------------------
    bt_ctr_clear_i <= '1' when (segment_end_i = '1' or hsync_end_req_valid = '1')
                          else
                      '0';

    bt_ctr_clear    <= bt_ctr_clear_i;
    segm_end        <= segment_end_i;
    h_sync_valid    <= hsync_end_req_valid;
 
end architecture rtl;