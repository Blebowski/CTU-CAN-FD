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
-- Module:
--  Prescaler circuit.
--
-- Sub-modules:
--  1. Bit time config capture
--  2. Synchronisation checker
--  3. Bit time counters
--  4. Resynchronisation
--  5. Segment end detector
--  6. Bit time FSM.
--  7. Trigger generator.                                          
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

entity prescaler is
    generic(
        -- Reset polarity
        G_RESET_POLARITY        :   std_logic := '0';

        -- TSEG1 Width
        G_TSEG1_WIDTH           :   natural := 8;
        
        -- TSEG2 Width
        G_TSEG2_WIDTH           :   natural := 8;
        
        -- Baud rate prescaler Width
        G_BRP_WIDTH             :   natural := 8;
        
        -- Synchronisation Jump width Width
        G_SJW_WIDTH             :   natural := 5;
      
        -- Number of signals in Sample trigger
        G_SAMPLE_TRIGGER_COUNT  :   natural range 2 to 8 := 2
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys              :in std_logic;
        
        -- Asynchronous reset
        res_n                :in std_logic;
        
        -----------------------------------------------------------------------
        -- Memory registers interface
        -----------------------------------------------------------------------
        -- Driving Bus
        drv_bus              :in std_logic_vector(1023 downto 0); 
        
        -----------------------------------------------------------------------
        -- Control Interface
        -----------------------------------------------------------------------
        -- Synchronisation edge (from Bus sampling)
        sync_edge            :in std_logic;
        
        -- Sample control (Nominal, Data, Secondary)
        sp_control           :in std_logic_vector(1 downto 0);
        
        -- Synchronisation control (No synchronisation, Hard Synchronisation,
        -- Resynchronisation
        sync_control         :in std_logic_vector(1 downto 0);
        
        -- No re-synchronisation should be executed due to positive phase
        -- error
        no_pos_resync        :in std_logic;
        
        -- Enable Bit time counters.
        bt_ctrs_en           :in std_logic;
        
        -- Bit rate shifted
        br_shifted           :in std_logic;
                
        -----------------------------------------------------------------------
        -- Trigger signals
        -----------------------------------------------------------------------
        -- RX Triggers
        rx_triggers     : out std_logic_vector(G_SAMPLE_TRIGGER_COUNT - 1 downto 0);
        
        -- TX Trigger
        tx_trigger      : out std_logic;
        
        -----------------------------------------------------------------------
        -- Status outputs
        -----------------------------------------------------------------------        
        -- Bit Time FSM state
        bt_fsm          : out t_bit_time 
  );
end entity;

architecture rtl of prescaler is

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
    -- Driving bus aliases
    ---------------------------------------------------------------------------
    signal drv_ena   :  std_logic;

    ---------------------------------------------------------------------------
    -- Segment lengths
    ---------------------------------------------------------------------------
    signal tseg1 :  std_logic_vector(G_TSEG1_WIDTH - 1 downto 0);
    signal tseg2 :  std_logic_vector(G_TSEG2_WIDTH - 1 downto 0);
    signal brp   :  std_logic_vector(G_BRP_WIDTH - 1 downto 0);
    signal sjw   :  std_logic_vector(G_SJW_WIDTH - 1 downto 0);
    
    -- End of segment is detected (by segment end detector)
    signal segment_end          : std_logic;
    
    -- Valid hard synchronisation occurred
    signal h_sync_valid         : std_logic;
    
    -- Signalling of each segment (by Bit Time FSM)
    signal is_tseg1             : std_logic;
    signal is_tseg2             : std_logic;
    
    -- Hard/Re-Synchronisation edges are valid. This only signals that sync.
    -- edge is there, sync control is set accordingly and there was no previous
    -- synchronisation from sample point till now!
    signal resync_edge_valid    : std_logic;
    signal h_sync_edge_valid    : std_logic;

    -- Size of internal Segment counters.
    constant C_SEGM_CTR_WIDTH       : natural :=
        max(G_TSEG1_WIDTH, G_TSEG2_WIDTH) + 1;
   
    -- Segment counter
    signal segm_counter           : std_logic_vector(C_SEGM_CTR_WIDTH - 1 downto 0);
    
    -- Exit segment request from re-synchronisation circuit
    signal exit_segm_req        : std_logic;
    
    -- Time quanta edges
    signal tq_edge              : std_logic;

    -- Sample trigger request (in sample point)
    signal rx_trig_req           : std_logic;
    
    -- Sync trigger request (in beginning of SYNC segment)
    signal tx_trig_req           : std_logic;   

    -- Signal that expected semgent length should be loaded after restart!
    signal start_edge            : std_logic;
    
    -- Bit time counter clear
    signal bt_ctr_clear          : std_logic;

begin

    drv_ena <= drv_bus(DRV_ENA_INDEX);
    
    ---------------------------------------------------------------------------
    -- Bit time config capture
    ---------------------------------------------------------------------------
    bit_time_cfg_capture_inst : bit_time_cfg_capture
    generic map (
        G_RESET_POLARITY    => G_RESET_POLARITY,
        G_TSEG1_WIDTH       => G_TSEG1_WIDTH,
        G_TSEG2_WIDTH       => G_TSEG2_WIDTH,
        G_BRP_WIDTH         => G_BRP_WIDTH,
        G_SJW_WIDTH         => G_SJW_WIDTH
    )
    port map(
        clk_sys    => clk_sys,      -- IN
        res_n      => res_n,        -- IN
        drv_bus    => drv_bus,      -- IN
        sp_control => sp_control,   -- IN
        
        tseg1       => tseg1,       -- OUT
        tseg2       => tseg2,       -- OUT
        brp         => brp,         -- OUT
        sjw         => sjw,         -- OUT        
        start_edge  => start_edge   -- OUT
    );

    ---------------------------------------------------------------------------
    -- Synchronisation checker
    ---------------------------------------------------------------------------
    synchronisation_checker_inst : synchronisation_checker
    generic map(
        G_RESET_POLARITY    => G_RESET_POLARITY
    )
    port map(
        clk_sys           => clk_sys,               -- IN
        res_n             => res_n,                 -- IN
        sync_control      => sync_control,          -- IN
        sync_edge         => sync_edge,             -- IN
        no_pos_resync     => no_pos_resync,         -- IN
        segment_end       => segment_end,           -- IN
        is_tseg1          => is_tseg1,              -- IN
        is_tseg2          => is_tseg2,              -- IN
        
        resync_edge_valid => resync_edge_valid,     -- OUT
        h_sync_edge_valid => h_sync_edge_valid      -- OUT
    );

    
    ---------------------------------------------------------------------------
    -- Re-synchronisation
    ---------------------------------------------------------------------------
    resynchronisation_inst : resynchronisation
    generic map(
        G_RESET_POLARITY       => G_RESET_POLARITY,
        G_SJW_WIDTH            => G_SJW_WIDTH,
        G_TSEG1_WIDTH          => G_TSEG1_WIDTH,
        G_TSEG2_WIDTH          => G_TSEG2_WIDTH,
        G_SEGM_CTR_WIDTH       => C_SEGM_CTR_WIDTH
    )
    port map(
        clk_sys              => clk_sys,            -- IN
        res_n                => res_n,              -- IN
        resync_edge_valid    => resync_edge_valid,  -- IN
        br_shifted           => br_shifted,         -- IN
        is_tseg1             => is_tseg1,           -- IN
        is_tseg2             => is_tseg2,           -- IN

        tseg_1               => tseg1,              -- IN
        tseg_2               => tseg2,              -- IN
        sjw                  => sjw,                -- IN
        start_edge           => start_edge,         -- IN

        segm_counter         => segm_counter,       -- IN
        segm_end             => segment_end,        -- IN
        h_sync_valid         => h_sync_valid,       -- IN

        exit_segm_req        => exit_segm_req       -- OUT
    );
    
    
    ---------------------------------------------------------------------------
    -- Bit Time counter
    ---------------------------------------------------------------------------
    bit_time_counters_inst : bit_time_counters
    generic map(
        G_RESET_POLARITY  => G_RESET_POLARITY,
        G_SEGM_CTR_WIDTH  => C_SEGM_CTR_WIDTH,
        G_BRP_WIDTH       => G_BRP_WIDTH
    )
    port map(
        clk_sys         => clk_sys,         -- IN
        res_n           => res_n,           -- IN
        brp             => brp,             -- IN
        bt_ctr_clear    => bt_ctr_clear,    -- IN
        drv_ena         => drv_ena,         -- IN
        bt_ctrs_en      => bt_ctrs_en,      -- IN

        tq_edge         => tq_edge,         -- OUT
        segm_counter    => segm_counter     -- OUT
    );
    

    ---------------------------------------------------------------------------
    -- End of Segment detector
    ---------------------------------------------------------------------------
    segment_end_detector_inst : segment_end_detector
    generic map(
        g_reset_polarity   => G_RESET_POLARITY
    )
    port map(
        clk_sys            => clk_sys,              -- IN
        res_n              => res_n,                -- IN
        h_sync_edge_valid  => h_sync_edge_valid,    -- IN
        exit_segm_req      => exit_segm_req,        -- IN
        is_tseg1           => is_tseg1,             -- IN
        is_tseg2           => is_tseg2,             -- IN
        tq_edge            => tq_edge,              -- IN

        segm_end           => segment_end,          -- OUT
        h_sync_valid       => h_sync_valid,         -- OUT
        bt_ctr_clear       => bt_ctr_clear          -- OUT
    );


    ---------------------------------------------------------------------------
    -- Bit time FSM
    ---------------------------------------------------------------------------
    bit_time_fsm_inst : bit_time_fsm
    generic map(
        G_RESET_POLARITY => G_RESET_POLARITY
    )
    port map(
        clk_sys          => clk_sys,            -- IN
        res_n            => res_n,              -- IN
        segm_end         => segment_end,        -- IN
        h_sync_valid     => h_sync_valid,       -- IN
        drv_ena          => drv_ena,            -- IN
        is_tseg1         => is_tseg1,           -- OUT
        is_tseg2         => is_tseg2,           -- OUT
        rx_trig_req      => rx_trig_req,        -- IN
        tx_trig_req      => tx_trig_req,        -- IN
        bt_fsm           => bt_fsm              -- OUT
    );
    
    ---------------------------------------------------------------------------
    -- Trigger generator
    ---------------------------------------------------------------------------
    trigger_generator_inst : trigger_generator
    generic map(
        G_RESET_POLARITY        => G_RESET_POLARITY,
        G_SAMPLE_TRIGGER_COUNT  => G_SAMPLE_TRIGGER_COUNT
    )
    port map(
        clk_sys     => clk_sys,         -- IN
        res_n       => res_n,           -- IN
        rx_trig_req => rx_trig_req,     -- IN
        tx_trig_req => tx_trig_req,     -- IN
        sp_control  => sp_control,      -- IN
        
        rx_triggers => rx_triggers,     -- OUT
        tx_trigger  => tx_trigger       -- OUT
    );

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    
    -- psl default clock is rising_edge(clk_sys);
    --

end architecture;