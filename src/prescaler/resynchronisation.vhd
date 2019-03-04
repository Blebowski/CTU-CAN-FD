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
--  Re-Synchronisation implementation. This module takes care of measuring
--  segment length (TSEG1 or TSEG2) with respect to Re-synchronisation.
--  Expected segment length is loaded upon the end of segment. Upon Re-
--  synchronisation, new length of segment is loaded. End of segment is
--  requested when input Bit Time counter reaches expected length of segment.
--  
--  To cover immediate re-synchronisation in PH2, where Bit should end
--  immediately, there is special "immediate" resynchronisation implemented.
--  This re-synchronisation signals end of segment in the same clock cycle.
--
--  Re-synchronisation data-path is depicted in following diagrams:
--
--      TSEG1 +-+
--      +---->+  +
--            |   +
--            |   +---+      +-+              Expected
--      TSEG2 |   +   +----->+  +             segment
--      +---->+  +    |      |   +   +-----+  length
--            +-+   +-+---+  |   +--->D   Q+-------->
--                  |     |  |   +   |     |
--                  | +/- +->+  +    |     |
--       SJW  +-+   |     |  +-+     | ENA |
--      +---->+  +  +-+---+          +--+--+
--            |   +   ^                 ^
--       Bit  |   +---+               +-+--+
--     Counter|   +                   | OR |
--      +---->+  +                    ++--++
--            +-+       Segment End    ^  |
--                     +---------------+  |
--                      Resynchronisation |
--                     +------------------+
--                       valid
--
--   In this first part, Expected length of segment is calculated. Note that
--   upon end of segment either TSEG1 or TSEG2 is loaded to measure duration
--   of next segment (TSEG1 or TSEG2) mux, its output is then selected to D
--   input of Expected segment length register.
--   Re-synchronisation can be broken down to following parts:
--      1. Phase error > 0 (PROP or PH1) and Phase err > SJW.
--      2. Phase error > 0 (PROP or PH1) and Phase err < SJW.
--      3. Phase error < 0 (PH2) and Phase err > SJW.
--      4. Phase error < 0 (PH2) and Phase err < SJW.
--   Following values are loaded to expected segment length in these cases:
--      1. TSEG1 + SJW (TSEG1 prolonged by SJW)
--      2. TSEG1 + Bit Counter (in this case value in Bit counter is equal
--           to phase error).
--      3. TSEG2 - SJW (TSEG1 is shortened by SJW).
--      4. This case is not handled here, since this requires immediate
--         end of segment in acutal Time quanta. If Time quanta is 1 this
--         needs to be in current cycle and we can't afford 1 clock cycle
--         delay of "expected segment length" register. This is handled
--         by immediate segment exit described further.
--
--  Calculation oh phase error and Detection of immediate segment exit
--  is shown in following diagram:
--     
--       Synchronisation jump width
--     +--------------------------------+        Phase error more
--               Bit                    |        than SJW
--             Counter   +-+            v     |---------------------->
--           +---------->+  +  Phase +--+--+  |
--           |           |   + error |     |  |   +-----+
--           |  +-----+  |   +------>+  >  +----->+     | Immediate
--     +-----+->+     |  |   +       |     |      |     |   exit
--      TSEG2   |  -  +->+  +        +-----+ +--->+ AND +--------->
--     +------->+     |  +-+     Is TSEG2?   |    |     |
--              +-----+         +------------+ +->+     |
--                           Resynchronisation |  +-----+
--                         +-------------------+
--                                 valid
--      
--   Phase error detection functions like so:
--      1. In TSEG1, phase error is given just by value of Bit counter
--      2. In TSEG2, phase error = TSEG2 - Bit Counter.
--
--   Phase error is compared with SJW to find out whether Phase error > SJW
--   or not. This signal is then used to drive muxes in diagram nr. 1.
--   Immediate segment end is detected when Phase error <= SJW and at the same
--   time we are in TSEG2 and re-synchronisation occurs.
--
--  End of segment requesting is depicted in following diagram:
--                             
--      Immediate exit +----+ 
--     +-------------->+    | 
--        Bit          |    |  Segment end request
--      Counter +---+  | OR +---------------------->
--     +------->+   |  |    | 
--              | = +->+    | 
--     +------->+   |  +----+ 
--     Expected +---+         
--     segment                
--     length                 
--
--  End of segment request provides a request to finish current Bit time
--  segment either due to Immediate Re-synchronisation or Reaching expected
--  length of segment (nominal or post-resynchronisation).                             
--
--  All three above described diagrams create a single re-synchronisation
--  datapath.
--
--  All data-path inputs are unsigned (direct code)!!
--     
--------------------------------------------------------------------------------
-- Revision History:
--    10.02.2019   Created file
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

entity resynchronisation is
    generic (
        -- Reset polarity
        reset_polarity          :       std_logic := '0';
        
        -- SJW width
        sjw_width               :       natural := 4;
        
        -- TSEG1 width
        tseg1_width             :       natural := 8;
        
        -- TSEG2 width
        tseg2_width             :       natural := 8;
        
        -- Bit counter width
        bt_width                :       natural := 8
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and reset
        -----------------------------------------------------------------------
        signal clk_sys          : in    std_logic;
        signal res_n            : in    std_logic;

        -----------------------------------------------------------------------
        -- Control interface
        -----------------------------------------------------------------------
        -- There is a valid re-synchronisation edge
        signal resync_edge_valid    : in    std_logic;
        
        -- Information processing time OK, PH2 may end
        signal ipt_ok               : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Bit Time FSM interface
        -----------------------------------------------------------------------        
        -- Bit time is in SYNC, PROP or PH1
        signal is_tseg1         : in    std_logic;
        
        -- Bit time is in PH2
        signal is_tseg2         : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Bit Time config capture interface
        -----------------------------------------------------------------------
        -- Time segment 1 (SYNC + PROP + PH1)
        signal tseg_1       : in    std_logic_vector(tseg1_width - 1 downto 0);
        
        -- Time segment 2 (PH2)
        signal tseg_2       : in    std_logic_vector(tseg2_width - 1 downto 0);
        
        -- Synchronisation Jump Width
        signal sjw          : in    std_logic_vector(sjw_width - 1 downto 0);
        
        -- Circuit operation has started -> load expected segment length reg.
        signal start_edge   : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Bit Time counter interface
        -----------------------------------------------------------------------
        -- Bit time counter
        signal bt_counter   : in    std_logic_vector(bt_width - 1 downto 0);

        -----------------------------------------------------------------------
        -- End of segment detector
        -----------------------------------------------------------------------
        -- End of segment (either TSEG1 or TSEG2)
        signal segm_end         : in    std_logic;
        
        -- Hard synchronisation valid
        signal h_sync_valid     : in    std_logic;

        -----------------------------------------------------------------------
        -- Output interface (signalling end of segment)
        -----------------------------------------------------------------------
        -- End of segment request
        signal exit_segm_req    : out   std_logic
    );
end entity;

architecture rtl of resynchronisation is

    function max(
        a : natural;
        b : natural)
    return natural is
    begin
        if (a > b) then
            return a;
        else
            return b;
        end if;
    end function max;

    ---------------------------------------------------------------------------
    -- Internal constants, calculation of data-path widths.
    ---------------------------------------------------------------------------
    constant bs_width : natural := max(tseg1_width, tseg1_width);
    constant ext_width : natural := max(bt_width, sjw_width);
    constant exp_width : natural := max(bs_width, ext_width) + 1;
    
    constant e_width     : natural := max(bt_width, tseg2_width);
    constant e_sjw_width : natural := max(e_width, sjw_width);

    -- Selector between TSEG1 and TSEG2
    signal sel_tseg1            : std_logic;
    
    -- Length of TSEG1 or TSEG2 (without resynchronisation)
    signal basic_segm_length    : unsigned(bs_width - 1 downto 0);
    
    -- Length which should be added to the basic segment length (positive only)
    signal segm_extension       : unsigned(ext_width - 1 downto 0);
    
    -- Base length of segment +/- segment extension
    signal segm_ext_add         : unsigned(exp_width - 1 downto 0);
    signal segm_ext_sub         : unsigned(exp_width - 1 downto 0);
    
    -- Expected segment length register
    signal exp_seg_length_d     : unsigned(exp_width - 1 downto 0);
    signal exp_seg_length_q     : unsigned(exp_width - 1 downto 0);
    signal exp_seg_length_ce    : std_logic;
    
    -- Expected length of segment after re-synchronisation
    signal sync_segm_length     : unsigned(exp_width - 1 downto 0);


    -- Negative phase error (in PH2)
    signal neg_phase_err        : unsigned(e_width - 1 downto 0);

    -- Phase error
    signal phase_err            : unsigned(e_width - 1 downto 0);

    -- Phase error higher than SJW
    signal phase_err_mt_sjw     : std_logic;
    
    -- Exit PH2 immediately
    signal exit_ph2_immediate   : std_logic;
    
    -- Regular exit of Bit segment
    signal exit_segm_regular    : std_logic;
     
begin

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Re-synchronisation data-path
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    sel_tseg1 <= '1' when (h_sync_valid = '1' or start_edge = '1') else
                 '1' when (segm_end = '1' and is_tseg2 = '1') else
                 '0';

    basic_segm_length <= 
        resize(unsigned(tseg_1), bs_width) when (sel_tseg1 = '1') else
        resize(unsigned(tseg_2), bs_width);

    segm_extension <= 
        resize(unsigned(sjw), ext_width) when (phase_err_mt_sjw = '1') else
        resize(unsigned(bt_counter), ext_width);

    segm_ext_add <= resize(basic_segm_length, exp_width) +
                    resize(segm_extension, exp_width);

    segm_ext_sub <= resize(basic_segm_length, exp_width) -
                    resize(segm_extension, exp_width);

    sync_segm_length <= segm_ext_sub when (is_tseg2 = '1') else
                        segm_ext_add;
    
    ---------------------------------------------------------------------------
    -- Expected length of segment register. Load:
    --  1. Nominal length of next segment
    --  2. Value post-resynchronisation.
    ---------------------------------------------------------------------------
    exp_seg_length_d <=
        resize(basic_segm_length, exp_width) when (segm_end = '1' or start_edge = '1') else
        resize(sync_segm_length, exp_width);

    exp_seg_length_ce <= '1' when (segm_end = '1' or resync_edge_valid = '1' or
                                   start_edge = '1')
                             else
                         '0';

    exp_seg_length_proc : process(res_n, clk_sys)
    begin
        if (res_n = reset_polarity) then
            exp_seg_length_q <= (others => '1');
        elsif (rising_edge(clk_sys)) then
            if (exp_seg_length_ce = '1') then
                exp_seg_length_q <= exp_seg_length_d;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Phase error calculation:
    --  1. For TSEG2: TSEG2 - Bit Time counter
    --  2. For TSEG1: Only Bit Time counter
    -- Note that subtraction in unsigned type is safe here since bt_counter
    -- is never higher than tseg_2 in tseg_2. If we are in tseg_1 neg_phase
    -- err underflows, but we don't care since we don't use it then!
    ---------------------------------------------------------------------------
    neg_phase_err  <= resize(unsigned(tseg_2), e_width) -
                      resize(unsigned(bt_counter), e_width); 

    phase_err <= resize(neg_phase_err, e_width) when (is_tseg2 = '1') else
                 resize(unsigned(bt_counter), e_width);

    phase_err_mt_sjw <= '1' when (resize(phase_err, e_sjw_width) >
                                  resize(unsigned(sjw), e_sjw_width))
                            else
                        '0';

    ---------------------------------------------------------------------------
    -- Immediate exit occurs during PH2 when resync edge occurs.
    ---------------------------------------------------------------------------
    exit_ph2_immediate <= '1' when (phase_err_mt_sjw = '0' and is_tseg2 = '1' and
                                    resync_edge_valid = '1') 
                              else
                          '0';

    ---------------------------------------------------------------------------
    -- Regular end occurs when Bit time counter reaches expected length of
    -- segment.
    ---------------------------------------------------------------------------
    exit_segm_regular <= '1' when (resize(unsigned(bt_counter), exp_width) >=
                                   resize(unsigned(exp_seg_length_q) - 1, exp_width))
                             else
                         '0';

    ---------------------------------------------------------------------------
    -- Capture request to end of segment. Re-synchronisation is not Time Quanta
    -- aligned, so we must capture the flag.
    --  1. Immediate exit of PH2, we still need to capture in case that this
    --     resynchronisation is delayed till IPT_OK.
    --  2. PH2, regular segment exit.
    --  3. PROP or PH1 regular segment exit.
    ---------------------------------------------------------------------------
    exit_segm_req <= '1' when (exit_ph2_immediate = '1') else
                     '1' when (is_tseg2 = '1' and exit_segm_regular = '1') else
                     '1' when (is_tseg1 = '1' and exit_segm_regular = '1') else
                     '0';

end architecture rtl;