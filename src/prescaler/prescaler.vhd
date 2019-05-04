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
--  Prescaler circuit.
--
--  Implements functionality of Bit Time measurement. Handles Hard Synchroni-
--  sation and Re-synchronisation. Generates SYNC and SAMPLE Triggering signals
--  for CAN TX and CAN RX Datapath processing.
--                                                                          
--------------------------------------------------------------------------------
-- Revision History:
--
--    June 2015   Version 1 of circuit
--    July 2015   Version 2 and 3 of circuit
--    15.2.2019   Version 4 of the circuit. Complete re-work and splitting into
--                sub-modules. Stashed all the previous messages since these
--                are now obsolete!
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
use work.endian_swap.all;
use work.reduce_lib.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

entity prescaler is
    generic(
        -- Reset polarity
        G_RESET_POLARITY        :   std_logic := '0';

        -- TSEG1 Width - Nominal Bit Time
        G_TSEG1_NBT_WIDTH       :   natural := 8;
        
        -- TSEG2 Width - Nominal Bit Time
        G_TSEG2_NBT_WIDTH       :   natural := 8;
        
        -- Baud rate prescaler Width - Nominal Bit Time
        G_BRP_NBT_WIDTH         :   natural := 8;
        
        -- Synchronisation Jump width Width - Nominal Bit Time
        G_SJW_NBT_WIDTH         :   natural := 5;
        
        -- TSEG1 Width - Data Bit Time
        G_TSEG1_DBT_WIDTH       :   natural := 8;
        
        -- TSEG2 Width - Data Bit Time
        G_TSEG2_DBT_WIDTH       :   natural := 8;
        
        -- Baud rate prescaler width - Data Bit Time
        G_BRP_DBT_WIDTH         :   natural := 8;
        
        -- Synchronisation Jump Width width - Data Bit Time
        G_SJW_DBT_WIDTH         :   natural := 5;
      
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
        -- Time quanta clock synchronisation output (debug only)
        time_quanta_clk : out std_logic;
        
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
    -- Nominal Bit-rate
    signal tseg1_nbt :  std_logic_vector(G_TSEG1_NBT_WIDTH - 1 downto 0);
    signal tseg2_nbt :  std_logic_vector(G_TSEG2_NBT_WIDTH - 1 downto 0);
    signal brp_nbt   :  std_logic_vector(G_BRP_NBT_WIDTH - 1 downto 0);
    signal sjw_nbt   :  std_logic_vector(G_SJW_NBT_WIDTH - 1 downto 0);

    -- Data Bit-rate
    signal tseg1_dbt :  std_logic_vector(G_TSEG1_DBT_WIDTH - 1 downto 0);
    signal tseg2_dbt :  std_logic_vector(G_TSEG2_DBT_WIDTH - 1 downto 0);
    signal brp_dbt   :  std_logic_vector(G_BRP_DBT_WIDTH - 1 downto 0);
    signal sjw_dbt   :  std_logic_vector(G_SJW_DBT_WIDTH - 1 downto 0);
    
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

    -- Size of internal Bit time counters.
    constant C_BT_NBT_WIDTH       : natural :=
        max(G_TSEG1_NBT_WIDTH, G_TSEG2_NBT_WIDTH) + 1;
    constant C_BT_DBT_WIDTH       : natural :=
        max(G_TSEG1_DBT_WIDTH, G_TSEG2_DBT_WIDTH) + 1;
   
    -- Bit time counter values. 
    signal bt_counter_nbt       : std_logic_vector(C_BT_NBT_WIDTH - 1 downto 0);
    signal bt_counter_dbt       : std_logic_vector(C_BT_DBT_WIDTH - 1 downto 0);
    
    -- Exit segment requests from re-synchronisation circuits
    signal exit_segm_req_nbt    : std_logic;
    signal exit_segm_req_dbt    : std_logic;
    
    -- Time quanta edges
    signal tq_edge_nbt          : std_logic;
    signal tq_edge_dbt          : std_logic;
    
    -- Sample trigger request (in sample point)
    signal sample_req           : std_logic;
    
    -- Sync trigger request (in beginning of SYNC segment)
    signal sync_req             : std_logic;   

    -- Signal that expected semgent length should be loaded after restart!
    signal start_edge           : std_logic;
    
    -- Bit time counter clear
    signal bt_ctr_clear         : std_logic;
    
    -- Constants defined for PSL assertions only.
    constant C_NBT_ONES   : std_logic_vector(C_BT_NBT_WIDTH - 1 downto 0) :=
        (OTHERS => '1');
    constant C_DBT_ONES   : std_logic_vector(C_BT_DBT_WIDTH - 1 downto 0) :=
        (OTHERS => '1');

begin

    drv_ena <= drv_bus(DRV_ENA_INDEX);
    
    ---------------------------------------------------------------------------
    -- Bit time config capture
    ---------------------------------------------------------------------------
    bit_time_cfg_capture_inst : bit_time_cfg_capture
    generic map (
        G_RESET_POLARITY    => G_RESET_POLARITY,
        G_TSEG1_NBT_WIDTH   => G_TSEG1_NBT_WIDTH,
        G_TSEG2_NBT_WIDTH   => G_TSEG2_NBT_WIDTH,
        G_BRP_NBT_WIDTH     => G_BRP_NBT_WIDTH,
        G_SJW_NBT_WIDTH     => G_SJW_NBT_WIDTH,
        G_TSEG1_DBT_WIDTH   => G_TSEG1_DBT_WIDTH,
        G_TSEG2_DBT_WIDTH   => G_TSEG2_DBT_WIDTH,
        G_BRP_DBT_WIDTH     => G_BRP_DBT_WIDTH,
        G_SJW_DBT_WIDTH     => G_SJW_DBT_WIDTH
    )
    port map(
        clk_sys    => clk_sys,      -- IN
        res_n      => res_n,        -- IN
        drv_bus    => drv_bus,      -- IN
        
        tseg1_nbt  => tseg1_nbt,    -- OUT
        tseg2_nbt  => tseg2_nbt,    -- OUT
        brp_nbt    => brp_nbt,      -- OUT
        sjw_nbt    => sjw_nbt,      -- OUT
        tseg1_dbt  => tseg1_dbt,    -- OUT
        tseg2_dbt  => tseg2_dbt,    -- OUT
        brp_dbt    => brp_dbt,      -- OUT
        sjw_dbt    => sjw_dbt,      -- OUT
        start_edge => start_edge    -- OUT
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
    -- Re-synchronisation (Nominal Bit Time)
    ---------------------------------------------------------------------------
    resynchronisation_nbt_inst : resynchronisation
    generic map(
        G_RESET_POLARITY       => G_RESET_POLARITY,
        G_SJW_WIDTH            => G_SJW_NBT_WIDTH,
        G_TSEG1_WIDTH          => G_TSEG1_NBT_WIDTH,
        G_TSEG2_WIDTH          => G_TSEG2_NBT_WIDTH,
        G_BT_WIDTH             => C_BT_NBT_WIDTH
    )
    port map(
        clk_sys              => clk_sys,            -- IN
        res_n                => res_n,              -- IN
        resync_edge_valid    => resync_edge_valid,  -- IN
        is_tseg1             => is_tseg1,           -- IN
        is_tseg2             => is_tseg2,           -- IN
        tseg_1               => tseg1_nbt,          -- IN
        tseg_2               => tseg2_nbt,          -- IN
        sjw                  => sjw_nbt,            -- IN
        start_edge           => start_edge,         -- IN
        bt_counter           => bt_counter_nbt,     -- IN
        segm_end             => segment_end,        -- IN
        h_sync_valid         => h_sync_valid,       -- IN
        
        exit_segm_req        => exit_segm_req_nbt   -- OUT
    );
    
    
    ---------------------------------------------------------------------------
    -- Bit Time counter (Nominal Bit Time)
    ---------------------------------------------------------------------------
    bit_time_counters_nbt_inst : bit_time_counters
    generic map(
        G_RESET_POLARITY  => G_RESET_POLARITY,
        G_BT_WIDTH        => C_BT_NBT_WIDTH,
        G_TQ_WIDTH        => G_BRP_NBT_WIDTH
    )
    port map(
        clk_sys         => clk_sys,         -- IN
        res_n           => res_n,           -- IN
        prescaler       => brp_nbt,         -- IN
        tq_reset        => bt_ctr_clear,    -- IN
        bt_reset        => bt_ctr_clear,    -- IN
        drv_ena         => drv_ena,         -- IN
        
        tq_edge         => tq_edge_nbt,     -- OUT     
        bt_counter      => bt_counter_nbt   -- OUT
    );
    

    ---------------------------------------------------------------------------
    -- Re-synchronisation (Data Bit Time)
    ---------------------------------------------------------------------------
    resynchronisation_dbt_inst : resynchronisation
    generic map(
        G_RESET_POLARITY       => G_RESET_POLARITY,
        G_SJW_WIDTH            => G_SJW_DBT_WIDTH,
        G_TSEG1_WIDTH          => G_TSEG1_DBT_WIDTH,
        G_TSEG2_WIDTH          => G_TSEG2_DBT_WIDTH,
        G_BT_WIDTH             => C_BT_DBT_WIDTH
    )
    port map(
        clk_sys              => clk_sys,            -- IN
        res_n                => res_n,              -- IN
        resync_edge_valid    => resync_edge_valid,  -- IN
        is_tseg1             => is_tseg1,           -- IN
        is_tseg2             => is_tseg2,           -- IN
        tseg_1               => tseg1_dbt,          -- IN
        tseg_2               => tseg2_dbt,          -- IN
        sjw                  => sjw_dbt,            -- IN
        start_edge           => start_edge,         -- IN
        bt_counter           => bt_counter_dbt,     -- IN
        segm_end             => segment_end,        -- IN
        h_sync_valid         => h_sync_valid,       -- IN
        
        exit_segm_req        => exit_segm_req_dbt   -- OUT
    );
    
    
    ---------------------------------------------------------------------------
    -- Bit Time counter (Data Bit Time)
    ---------------------------------------------------------------------------
    bit_time_counters_dbt_inst : bit_time_counters
    generic map(
        g_reset_polarity  => G_RESET_POLARITY,
        G_BT_WIDTH        => C_BT_DBT_WIDTH,
        G_TQ_WIDTH        => G_BRP_DBT_WIDTH
    )
    port map(
        clk_sys         => clk_sys,         -- IN
        res_n           => res_n,           -- IN
        prescaler       => brp_dbt,         -- IN
        tq_reset        => bt_ctr_clear,    -- IN
        bt_reset        => bt_ctr_clear,    -- IN
        drv_ena         => drv_ena,         -- IN
        
        tq_edge         => tq_edge_dbt,     -- OUT
        bt_counter      => bt_counter_dbt   -- OUT
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
        sp_control         => sp_control,           -- IN
        h_sync_edge_valid  => h_sync_edge_valid,    -- IN
        exit_segm_req_nbt  => exit_segm_req_nbt,    -- IN
        exit_segm_req_dbt  => exit_segm_req_dbt,    -- IN
        is_tseg1           => is_tseg1,             -- IN
        is_tseg2           => is_tseg2,             -- IN
        tq_edge_nbt        => tq_edge_nbt,          -- IN
        tq_edge_dbt        => tq_edge_dbt,          -- IN
        
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
        sample_req       => sample_req,         -- IN
        sync_req         => sync_req,           -- IN
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
        sample_req  => sample_req,      -- IN
        sync_req    => sync_req,        -- IN
        sp_control  => sp_control,      -- IN
        
        rx_triggers => rx_triggers,     -- OUT
        tx_trigger  => tx_trigger       -- OUT
    );
    
    ---------------------------------------------------------------------------
    -- Internal signals to output propagation
    ---------------------------------------------------------------------------
    -- Time quanta clock output
    time_quanta_clk <= tq_edge_nbt when (sp_control = NOMINAL_SAMPLE)
                                   else
                       tq_edge_dbt;

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    
    -- psl default clock is rising_edge(clk_sys);
    --
    -- psl no_nbt_bt_overflow_asrt : assert never
    --      (bt_counter_dbt = nbt_ones and tq_edge_nbt = '1' and
    --       segment_end = '0' and sp_control = NOMINAL_SAMPLE)
    --  report "Nominal Bit time counter overflow!" severity error;
    --
    -- psl no_dbt_bt_overflow_asrt : assert never
    --      (bt_counter_dbt = dbt_ones and tq_edge_dbt = '1' and
    --       segment_end = '0' and
    --       (sp_control = DATA_SAMPLE or sp_control = SECONDARY_SAMPLE))
    --  report "Data Bit time counter overflow!" severity error;

end architecture;