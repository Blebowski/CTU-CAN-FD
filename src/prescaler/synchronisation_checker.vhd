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
--  Synchronisation Checker.
--
-- Purpose:
--  Holds flag that Re-synchronisation or Hard synchronisation occured. 
--  Valid Hard synchronisation or Re-synchronisation is signalled on the output.
--  Synchronisation flag is cleared in the end of TSEG1 (Sample point).
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

entity synchronisation_checker is
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys          : in    std_logic;
        
        -- Asynchronous Reset
        res_n            : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Control interface
        -----------------------------------------------------------------------
        -- Synchronisation control (No synchronisation, Hard Synchronisation,
        -- Resynchronisation
        sync_control     : in    std_logic_vector(1 downto 0);
        
        -- Synchronisation edge (from Bus sampling)
        sync_edge        : in    std_logic;
        
        -- No re-synchronisation should be executed due to positive phase
        -- error
        no_pos_resync    : in    std_logic;
        
        -- End of segment
        segment_end      : in    std_logic;
        
        -- Bit time FSM is in TSEG1
        is_tseg1         : in    std_logic;
        
        -- Bit time FSM is in TSEG2
        is_tseg2         : in    std_logic;
        
        -----------------------------------------------------------------------
        -- Status
        -----------------------------------------------------------------------
        -- Resynchronisation edge is valid
        resync_edge_valid    : out std_logic;
        
        -- Hard synchronisation edge is valid
        h_sync_edge_valid    : out std_logic
    );
end entity;

architecture rtl of synchronisation_checker is

    -- Synchronisation edges
    signal resync_edge          : std_logic;
    signal h_sync_edge          : std_logic;
    signal h_or_re_sync_edge    : std_logic;

    -- Flag that synchronisation has occurred (either hard sync or re-sync)
    signal sync_flag            : std_logic;
    signal sync_flag_ce         : std_logic;
    signal sync_flag_nxt        : std_logic;
    
begin
    
    ---------------------------------------------------------------------------
    -- Re-synchronisation, Hard synchronisation is distinguished by
    -- Sample control given by Protocol Control.
    ---------------------------------------------------------------------------
    resync_edge <= '1' when (sync_edge = '1' and sync_control = RE_SYNC) else
                   '0';

    h_sync_edge <= '1' when (sync_edge = '1' and sync_control = HARD_SYNC) else
                   '0';

    h_or_re_sync_edge <= '1' when (resync_edge = '1' or h_sync_edge = '1') else
                         '0';

    ---------------------------------------------------------------------------
    -- Synchronisation flag register.
    --  1. Set when Hard-sync or Resync occurs.
    --  2. Cleared in the end oof PH1 (sample point)
    -- Takes care of not synchronising twice between two sample points.
    ---------------------------------------------------------------------------    
    sync_flag_ce <= '1' when (h_or_re_sync_edge = '1') else
                    '1' when (segment_end = '1' and is_tseg1 = '1') else
                    '0';
                    
    sync_flag_nxt <= '1' when (h_or_re_sync_edge = '1') else
                     '0';
    
    sync_flag_proc : process(res_n, clk_sys)
    begin
        if (res_n = '0') then
            sync_flag <= '0';
        elsif (rising_edge(clk_sys)) then
            if (sync_flag_ce = '1') then
                sync_flag <= sync_flag_nxt;
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Re-synchronisation is valid when following conditions are met:
    --  1. There is resynchronisation edge
    --  2. Synchronisation flag dit not occur yet!
    -- This has two sub-cases:
    --  1. TSEG2, any synchronisation
    --  2. TSEG1, only when 'no_pos_resync' is not set! This takes care of
    --     no synchronisation for transmitter as result of positive phase
    --     error!
    ---------------------------------------------------------------------------
    resync_edge_valid <= '1' when (resync_edge = '1' and sync_flag = '0' and
                                   ((is_tseg2 = '1') or
                                    (is_tseg1 = '1' and no_pos_resync = '0')))
                             else
                         '0';

    ---------------------------------------------------------------------------
    -- Hard synchronisation is valid at any time, only if there was no
    -- synchronisation before!
    ---------------------------------------------------------------------------
    h_sync_edge_valid <= '0' when (no_pos_resync = '1' and is_tseg1 = '1') else
                         '1' when (h_sync_edge = '1' and sync_flag = '0') else
                         '0';

    -- <RELEASE_OFF>
    -----------------------------------------------------------------------
    -----------------------------------------------------------------------
    -- Assertions
    -----------------------------------------------------------------------
    -----------------------------------------------------------------------
    
    -- psl default clock is rising_edge(clk_sys);

    -- psl h_sync_ignored_due_to_previous_sync_cov : cover
    --  {sync_flag = '1' and h_sync_edge = '1'}
    --  report "Hard synchronisation ignored due to previous synchronization";

    -- psl re_sync_ignored_due_to_previous_sync_cov : cover
    --  {sync_flag = '1' and resync_edge = '1'}
    --  report "Re-synchronisation ignored due to previous synchronization";

    -- psl h_sync_in_tseg_1_cov : cover
    --  {h_sync_edge_valid = '1' and is_tseg1 = '1'}
    --  report "Hard synchronization in TSEG1"; 

    -- psl h_sync_in_tseg_2_cov : cover
    --  {h_sync_edge_valid = '1' and is_tseg2 = '1'}
    --  report "Hard synchronization in TSEG2";

    -- Hard synchronization in TSEG1
    -- psl re_sync_in_tseg_1_cov : cover
    --  {resync_edge_valid = '1' and is_tseg1 = '1'};

    -- psl re_sync_in_tseg_2_cov : cover
    --  {resync_edge_valid = '1' and is_tseg2 = '1'}
    --  report "Hard synchronization in TSEG2";

    -- <RELEASE_ON>

end architecture rtl;