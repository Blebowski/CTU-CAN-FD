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
--  Bit segment meter.
--
-- Purpose:
--  This module measures segment length (TSEG1 or TSEG2) with respect to 
--  Re-synchronisation. Expected segment length is loaded upon the end of 
--  previous segment. Upon Resynchronisation, new length of segment is loaded
--  (shortened or lengthened). End of segment is requested when Segment counter 
--  reaches expected length of segment.
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
--      1. Phase error > 0 (TSEG1) and Phase err > SJW.
--      2. Phase error > 0 (TSEG1) and Phase err < SJW.
--      3. Phase error < 0 (TSEG2) and Phase err > SJW.
--      4. Phase error < 0 (TSEG2) and Phase err < SJW.
--   Following values are loaded to expected segment length in these cases:
--      1. TSEG1 + SJW (TSEG1 prolonged by SJW)
--      2. TSEG1 + Segment Counter (in this case value in Segment counter 
--           is equal to phase error).
--      3. TSEG2 - SJW (TSEG1 is shortened by SJW).
--      4. This case is not handled here, since this requires immediate
--         end of segment in acutal Time quanta. If Time quanta is 1 this
--         needs to be in current cycle and we can't afford 1 clock cycle
--         delay of "expected segment length" register. This is handled
--         by immediate segment exit described further.
--
--  If Hard synchronisation occured, Length of TSEG1 - 1 is loaded to expected
--  segment length register.
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
--      TSEG2   |     +->+  +        +-----+ +--->+ AND +--------->
--     +------->+  -  |  +-+     Is TSEG2?   |    |     |
--        1     |     |         +------------+ +->+     |
--     +------->|     |      Resynchronisation |  +-----+
--              +-----+    +-------------------+
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
--   End of segment request is depicted in following diagram:
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

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_components_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.common_blocks_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity bit_segment_meter is
    generic (
        -- SJW width
        G_SJW_WIDTH               :       natural := 4;
        
        -- TSEG1 width
        G_TSEG1_WIDTH             :       natural := 8;
        
        -- TSEG2 width
        G_TSEG2_WIDTH             :       natural := 8;
        
        -- Bit counter width
        G_BT_WIDTH                :       natural := 8
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
        -- Control interface
        -----------------------------------------------------------------------
        -- There is a valid re-synchronisation edge.
        resync_edge_valid    : in    std_logic;

        -----------------------------------------------------------------------
        -- Bit Time FSM interface
        -----------------------------------------------------------------------        
        -- Bit time is in SYNC, PROP or PH1
        is_tseg1         : in    std_logic;
        
        -- Bit time is in PH2
        is_tseg2         : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Bit Time config capture interface
        -----------------------------------------------------------------------
        -- Time segment 1 (SYNC + PROP + PH1)
        tseg_1       : in    std_logic_vector(G_TSEG1_WIDTH - 1 downto 0);
        
        -- Time segment 2 (PH2)
        tseg_2       : in    std_logic_vector(G_TSEG2_WIDTH - 1 downto 0);
        
        -- Synchronisation Jump Width
        sjw          : in    std_logic_vector(G_SJW_WIDTH - 1 downto 0);
        
        -- Circuit operation has started -> load expected segment length reg.
        start_edge   : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Bit Time counter interface
        -----------------------------------------------------------------------
        -- Bit time counter
        segm_counter : in    std_logic_vector(G_BT_WIDTH - 1 downto 0);

        -----------------------------------------------------------------------
        -- End of segment detector
        -----------------------------------------------------------------------
        -- End of segment (either TSEG1 or TSEG2)
        segm_end         : in    std_logic;
        
        -- Hard synchronisation valid
        h_sync_valid     : in    std_logic;

        -----------------------------------------------------------------------
        -- Output interface (signalling end of segment)
        -----------------------------------------------------------------------
        -- End of segment request
        exit_segm_req    : out   std_logic
    );
end entity;

architecture rtl of bit_segment_meter is

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
    constant C_BS_WIDTH : natural := max(G_TSEG1_WIDTH, G_TSEG2_WIDTH);
    constant C_EXT_WIDTH : natural := max(G_BT_WIDTH, G_SJW_WIDTH);
    constant C_EXP_WIDTH : natural := max(C_BS_WIDTH, C_EXT_WIDTH) + 1;
    
    constant C_E_WIDTH     : natural := max(G_BT_WIDTH, G_TSEG2_WIDTH);
    constant C_E_SJW_WIDTH : natural := max(C_E_WIDTH, G_SJW_WIDTH);

    -- Selector between TSEG1 and TSEG2
    signal sel_tseg1            : std_logic;
    
    -- Length of TSEG1 or TSEG2 (without resynchronisation)
    signal basic_segm_length    : unsigned(C_BS_WIDTH - 1 downto 0);
    
    -- Length which should be added to the basic segment length (positive only)
    signal segm_extension       : unsigned(C_EXT_WIDTH - 1 downto 0);
    
    -- Base length of segment +/- segment extension
    signal segm_ext_add         : unsigned(C_EXP_WIDTH - 1 downto 0);
    signal segm_ext_sub         : unsigned(C_EXP_WIDTH - 1 downto 0);
    
    -- Expected segment length register
    signal exp_seg_length_d     : unsigned(C_EXP_WIDTH - 1 downto 0);
    signal exp_seg_length_q     : unsigned(C_EXP_WIDTH - 1 downto 0);
    signal exp_seg_length_ce    : std_logic;
    
    -- Expected length of segment after re-synchronisation
    signal sync_segm_length     : unsigned(C_EXP_WIDTH - 1 downto 0);


    -- Negative phase error (in PH2)
    signal neg_phase_err        : unsigned(C_E_WIDTH - 1 downto 0);

    -- Phase error
    signal phase_err            : unsigned(C_E_WIDTH - 1 downto 0);

    -- Phase error higher than SJW
    signal phase_err_mt_sjw     : std_logic;
    signal phase_err_eq_sjw     : std_logic;
    
    -- Exit PH2 immediately
    signal exit_ph2_immediate   : std_logic;
    
    -- Regular exit of Bit segment
    signal exit_segm_regular    : std_logic;
     
    -- Regular exit for TSEG1, TSEG2
    signal exit_segm_regular_tseg1 : std_logic;
    signal exit_segm_regular_tseg2 : std_logic;
    
    -- SJW more than 0
    signal sjw_mt_zero             : std_logic;
    
    -- Choose basic segment length
    signal use_basic_segm_length   : std_logic;
    
    -- Phase error = SJW + 1 -> Used for immediate segment end in case of
    -- negative resync, since at point where edge is evaluated, bit time
    -- should be ended!
    signal phase_err_sjw_by_one    : std_logic;
    
    -- Shorten following Tseg1 by 1 due to negative resync with Phase err <= SJW
    signal shorten_tseg1_after_tseg2 : std_logic;
    
begin

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Re-synchronisation data-path
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    sel_tseg1 <= '1' when (h_sync_valid = '1' or start_edge = '1' or
                           shorten_tseg1_after_tseg2 = '1') else
                 '1' when (segm_end = '1' and is_tseg2 = '1') else
                 '1' when (segm_end = '0' and is_tseg1 = '1') else
                 '0';

    basic_segm_length <= 
        resize(unsigned(tseg_1), C_BS_WIDTH) when (sel_tseg1 = '1') else
        resize(unsigned(tseg_2), C_BS_WIDTH);

    segm_extension <= 
               to_unsigned(1, C_EXT_WIDTH) when (h_sync_valid = '1' or
                                                 shorten_tseg1_after_tseg2 = '1') else
        resize(unsigned(sjw), C_EXT_WIDTH) when (phase_err_mt_sjw = '1') else
        resize(unsigned(segm_counter), C_EXT_WIDTH);

    segm_ext_add <= resize(basic_segm_length, C_EXP_WIDTH) +
                    resize(segm_extension, C_EXP_WIDTH);

    segm_ext_sub <= resize(basic_segm_length, C_EXP_WIDTH) -
                    resize(segm_extension, C_EXP_WIDTH);

    sync_segm_length <= segm_ext_sub when (is_tseg2 = '1' or h_sync_valid = '1' or
                                           exit_ph2_immediate = '1')
                                     else
                        segm_ext_add;

    ---------------------------------------------------------------------------
    -- Use basic segment length:
    --  1. Circuit start
    --  2. Segment end, but not due to hard-sync. When segment end due to hard
    --     sync occurs, we must take TSEG1 - 1 which is calculated in synced
    --     segment length! This also applies when there is negative resync.
    --     due to Phase error <= SJW.
    ---------------------------------------------------------------------------
    use_basic_segm_length <= '1' when (start_edge = '1')
                                 else
                             '1' when (segm_end = '1' and
                                       h_sync_valid = '0' and
                                       shorten_tseg1_after_tseg2 = '0') 
                                 else
                             '0';
    
    ---------------------------------------------------------------------------
    -- Expected length of segment register. Load:
    --  1. Nominal length of next segment
    --  2. Value post-synchronisation.
    ---------------------------------------------------------------------------
    exp_seg_length_d <=
        resize(basic_segm_length, C_EXP_WIDTH) when (use_basic_segm_length = '1')
                                               else
        resize(sync_segm_length, C_EXP_WIDTH);

    exp_seg_length_ce <= '1' when (segm_end = '1' or 
                                   resync_edge_valid = '1' or
                                   h_sync_valid = '1' or
                                   start_edge = '1')
                             else
                         '0';

    exp_seg_length_proc : process(res_n, clk_sys)
    begin
        if (res_n = '0') then
            exp_seg_length_q <= (others => '1');
        elsif (rising_edge(clk_sys)) then
            if (exp_seg_length_ce = '1') then
                exp_seg_length_q <= exp_seg_length_d;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Phase error calculation:
    --  1. For TSEG2: TSEG2 - Bit Time counter - 1
    --  2. For TSEG1: Only Bit Time counter
    --
    -- Note that subtraction in unsigned type is safe here since segm_counter
    -- is never higher than tseg_2 in tseg_2. If we are in tseg_1 neg_phase
    -- err underflows, but we don't care since we don't use it then!
    ---------------------------------------------------------------------------
    neg_phase_err  <= resize(unsigned(tseg_2), C_E_WIDTH) -
                      resize(unsigned(segm_counter), C_E_WIDTH);

    phase_err <= resize(neg_phase_err, C_E_WIDTH) when (is_tseg2 = '1') else
                 resize(unsigned(segm_counter), C_E_WIDTH);

    phase_err_mt_sjw <= '1' when (resize(phase_err, C_E_SJW_WIDTH) >
                                  resize(unsigned(sjw), C_E_SJW_WIDTH))
                            else
                        '0';

    phase_err_eq_sjw <= '1' when (resize(phase_err, C_E_SJW_WIDTH) =
                                  resize(unsigned(sjw), C_E_SJW_WIDTH))
                            else
                        '0'; 

    phase_err_sjw_by_one <= '1' when (resize(phase_err, C_E_SJW_WIDTH) =
                                      (resize(unsigned(sjw), C_E_SJW_WIDTH) +
                                       to_unsigned(1, C_E_SJW_WIDTH)))
                                else
                            '0';

    sjw_mt_zero <= '1' when (unsigned(sjw) > 0) else
                   '0';

    ---------------------------------------------------------------------------
    -- Immediate exit occurs during PH2 when resync edge occurs.
    -- This occurs in two cases:
    --  1. Phase error <= SJW. Also, consecutive TSEG1 is shortened.
    --  2. Phase error = SJW + 1. In this case immediate end also occurs,
    --     because at the end of TQ we flip to Phase error of 5 and we must
    --     shorten TSEG2 by SJW. In this case consecutive TSEG1 is NOT
    --     shortened!
    ---------------------------------------------------------------------------
    exit_ph2_immediate <= '1' when ((phase_err_mt_sjw = '0' or phase_err_sjw_by_one = '1') and
                                    is_tseg2 = '1' and
                                    resync_edge_valid = '1')
                              else
                          '0';


    ---------------------------------------------------------------------------
    -- When negative resynchronisation occurs due to Phase error <= SJW,
    -- we exit immediately! But we also exit immediately when Phase error
    ---------------------------------------------------------------------------
    shorten_tseg1_after_tseg2 <= '1' when (exit_ph2_immediate = '1' and
                                           phase_err_sjw_by_one = '0')
                                     else
                                 '0';

    ---------------------------------------------------------------------------
    -- Regular end occurs when Bit time counter reaches expected length of
    -- segment.
    ---------------------------------------------------------------------------
    exit_segm_regular <= '1' when (resize(unsigned(segm_counter), C_EXP_WIDTH) >=
                                   resize(unsigned(exp_seg_length_q) - 1, C_EXP_WIDTH))
                             else
                         '0';


    ---------------------------------------------------------------------------
    -- TSEG1 is finished when Bit time counter reached value, but not when
    -- resync-edge is there at the same time! If we did not consider resync
    -- edge, we would ignore resync edge which arrives just at the same clock
    -- cycle as bit time!
    ---------------------------------------------------------------------------
    exit_segm_regular_tseg1 <=  '0' when (is_tseg1 = '1' and 
                                          resync_edge_valid = '1' and
                                          sjw_mt_zero = '1')
                                    else
                                '1' when (is_tseg1 = '1' and exit_segm_regular = '1')
                                    else
                                '0';

    ---------------------------------------------------------------------------
    -- TSEG2 is finished when Bit time counter reached expected value!
    ---------------------------------------------------------------------------
    exit_segm_regular_tseg2 <= '1' when (is_tseg2 = '1' and exit_segm_regular = '1')
                                   else
                               '0';

    ---------------------------------------------------------------------------
    -- Capture request to end of segment. Re-synchronisation is not Time Quanta
    -- aligned, so we must capture the flag.
    --  1. Immediate exit of PH2.
    --  2. PH2, regular segment exit.
    --  3. PROP or PH1 regular segment exit.
    ---------------------------------------------------------------------------
    exit_segm_req <= '1' when (exit_ph2_immediate = '1') else
                     '1' when (exit_segm_regular_tseg1 = '1' or
                               exit_segm_regular_tseg2 = '1') else
                     '0';

    -- <RELEASE_OFF>
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    -- psl default clock is rising_edge(clk_sys);

    -- Positive resynchronization with E < SJW
    -- psl pos_resync_e_less_than_sjw_cov : cover
    --  {exp_seg_length_ce = '1' and use_basic_segm_length = '0' and is_tseg1 = '1'
    --   and resync_edge_valid = '1' and
    --   resize(phase_err, C_E_SJW_WIDTH) < resize(unsigned(sjw), C_E_SJW_WIDTH)};

    -- Positive resynchronization with E > SJW
    -- psl pos_resync_e_more_than_sjw_cov : cover
    --  {exp_seg_length_ce = '1' and use_basic_segm_length = '0' and is_tseg1 = '1'
    --   and resync_edge_valid = '1' and
    --   resize(phase_err, C_E_SJW_WIDTH) > resize(unsigned(sjw), C_E_SJW_WIDTH)};

    -- Positive resynchronization with E = SJW
    -- psl pos_resync_e_equal_sjw_cov : cover
    --  {exp_seg_length_ce = '1' and use_basic_segm_length = '0' and is_tseg1 = '1'
    --   and resync_edge_valid = '1' and
    --   resize(phase_err, C_E_SJW_WIDTH) = resize(unsigned(sjw), C_E_SJW_WIDTH)};

    -- Negative resynchronization with E < SJW
    -- psl neg_resync_e_less_than_sjw_cov : cover
    --  {exp_seg_length_ce = '1' and resync_edge_valid = '1' and is_tseg2 = '1' and
    --   resize(phase_err, C_E_SJW_WIDTH) < resize(unsigned(sjw), C_E_SJW_WIDTH)};

    -- Negative resynchronization with E > SJW
    -- psl neg_resync_e_more_than_sjw_cov : cover
    --  {exp_seg_length_ce = '1' and resync_edge_valid = '1' and is_tseg2 = '1' and
    --   resize(phase_err, C_E_SJW_WIDTH) > resize(unsigned(sjw), C_E_SJW_WIDTH)};

    -- Negative resynchronization with E = SJW
    -- psl neg_resync_e_equal_sjw_cov : cover
    --  {exp_seg_length_ce = '1' and resync_edge_valid = '1' and is_tseg2 = '1' and
    --   resize(phase_err, C_E_SJW_WIDTH) = resize(unsigned(sjw), C_E_SJW_WIDTH)};

    -- psl exit_segm_immediate_cov : cover
    --  {exit_segm_req = '1' and exit_ph2_immediate = '1'};

    -- psl exit_segm_regular_tseg1_cov : cover
    --  {exit_segm_req = '1' and exit_segm_regular_tseg1 = '1' and exit_segm_regular_tseg2 = '0'};

    -- psl exit_segm_regular_tseg2_cov : cover
    --  {exit_segm_req = '1' and exit_segm_regular_tseg1 = '0' and exit_segm_regular_tseg2 = '1'};

    -- <RELEASE_ON>

end architecture rtl;