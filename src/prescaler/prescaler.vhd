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
--  Prescaler circuit.
--
-- Sub-modules:
--  1. Bit time config capture
--  2. Synchronisation checker
--  3. Bit time counters (Nominal)
--  4. Bit segment meter (Nominal)
--  5. Bit time counters (Data)
--  6. Bit segment meter (Data)
--  7. Segment end detector
--  8. Bit time FSM.
--  9. Trigger generator.                                          
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

entity prescaler is
    generic(
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
        
        -- Enable Nominal Bit time counters.
        nbt_ctrs_en          :in std_logic;
        
        -- Enable Data Bit time counters.
        dbt_ctrs_en          :in std_logic;
        
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
        bt_fsm          : out t_bit_time;
        
        -- Time quanta edge
        tq_edge         : out std_logic
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
    signal segm_counter_nbt       : std_logic_vector(C_BT_NBT_WIDTH - 1 downto 0);
    signal segm_counter_dbt       : std_logic_vector(C_BT_DBT_WIDTH - 1 downto 0);
    
    -- Exit segment requests from re-synchronisation circuits
    signal exit_segm_req_nbt    : std_logic;
    signal exit_segm_req_dbt    : std_logic;
    
    -- Time quanta edges
    signal tq_edge_nbt          : std_logic;
    signal tq_edge_dbt          : std_logic;

    -- Sample trigger request (in sample point)
    signal rx_trig_req           : std_logic;
    
    -- Sync trigger request (in beginning of SYNC segment)
    signal tx_trig_req             : std_logic;   

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
    -- Bit segment meter (Nominal Bit Time)
    ---------------------------------------------------------------------------
    bit_segment_meter_nbt_inst : bit_segment_meter
    generic map(
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
        segm_counter         => segm_counter_nbt,   -- IN
        segm_end             => segment_end,        -- IN
        h_sync_valid         => h_sync_valid,       -- IN
        
        exit_segm_req        => exit_segm_req_nbt   -- OUT
    );
    
    
    ---------------------------------------------------------------------------
    -- Bit Time counter (Nominal Bit Time)
    ---------------------------------------------------------------------------
    bit_time_counters_nbt_inst : bit_time_counters
    generic map(
        G_BT_WIDTH        => C_BT_NBT_WIDTH,
        G_BRP_WIDTH       => G_BRP_NBT_WIDTH
    )
    port map(
        clk_sys         => clk_sys,         -- IN
        res_n           => res_n,           -- IN
        brp             => brp_nbt,         -- IN
        tq_reset        => bt_ctr_clear,    -- IN
        bt_reset        => bt_ctr_clear,    -- IN
        drv_ena         => drv_ena,         -- IN
        ctrs_en         => nbt_ctrs_en,     -- IN
        
        tq_edge         => tq_edge_nbt,     -- OUT     
        segm_counter    => segm_counter_nbt -- OUT
    );
    

    ---------------------------------------------------------------------------
    -- Bit segment meter (Data Bit Time)
    ---------------------------------------------------------------------------
    bit_segment_meter_dbt_inst : bit_segment_meter
    generic map(
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
        segm_counter         => segm_counter_dbt,   -- IN
        segm_end             => segment_end,        -- IN
        h_sync_valid         => h_sync_valid,       -- IN
        
        exit_segm_req        => exit_segm_req_dbt   -- OUT
    );
    
    
    ---------------------------------------------------------------------------
    -- Bit Time counter (Data Bit Time)
    ---------------------------------------------------------------------------
    bit_time_counters_dbt_inst : bit_time_counters
    generic map(
        G_BT_WIDTH        => C_BT_DBT_WIDTH,
        G_BRP_WIDTH       => G_BRP_DBT_WIDTH
    )
    port map(
        clk_sys         => clk_sys,         -- IN
        res_n           => res_n,           -- IN
        brp             => brp_dbt,         -- IN
        tq_reset        => bt_ctr_clear,    -- IN
        bt_reset        => bt_ctr_clear,    -- IN
        drv_ena         => drv_ena,         -- IN
        ctrs_en         => dbt_ctrs_en,     -- IN
        
        tq_edge         => tq_edge_dbt,     -- OUT
        segm_counter    => segm_counter_dbt -- OUT
    );

    ---------------------------------------------------------------------------
    -- End of Segment detector
    ---------------------------------------------------------------------------
    segment_end_detector_inst : segment_end_detector
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
    port map(
        clk_sys          => clk_sys,            -- IN
        res_n            => res_n,              -- IN
        segm_end         => segment_end,        -- IN
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
        G_SAMPLE_TRIGGER_COUNT  => G_SAMPLE_TRIGGER_COUNT
    )
    port map(
        clk_sys     => clk_sys,         -- IN
        res_n       => res_n,           -- IN
        rx_trig_req => rx_trig_req,     -- IN
        tx_trig_req => tx_trig_req,     -- IN

        rx_triggers => rx_triggers,     -- OUT
        tx_trigger  => tx_trigger       -- OUT
    );
    
    tq_edge <= tq_edge_nbt when (sp_control = NOMINAL_SAMPLE) else
               tq_edge_dbt;

    -- <RELEASE_OFF>
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Assertions
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------

    -- psl default clock is rising_edge(clk_sys);

    -- psl min_lenght_ph1_nbt_asrt : assert always
    --  {drv_ena = '1'} |=>
    --      {to_integer(unsigned(tseg1_nbt)) * to_integer(unsigned(brp_nbt)) > 2}
    --  report "Lenght of TSEG1(NBT) must be more than 2 clock cycles!";

    -- psl min_lenght_ph2_nbt_asrt : assert always
    --  {drv_ena = '1'} |=>
    --      {to_integer(unsigned(tseg2_nbt)) * to_integer(unsigned(brp_nbt)) > 1}
    --  report "Lenght of TSEG2(NBT) must be more than 1 clock cycle!";
    
    -- psl min_lenght_ph1_dbt_asrt : assert always
    --  {drv_ena = '1'} |=>
    --      {to_integer(unsigned(tseg1_dbt)) * to_integer(unsigned(brp_dbt)) > 2}
    --  report "Lenght of TSEG1(DBT) must be more than 2 clock cycles!";

    -- psl min_lenght_ph2_dbt_asrt : assert always
    --  {drv_ena = '1'} |=>
    --      {to_integer(unsigned(tseg2_dbt)) * to_integer(unsigned(brp_dbt)) > 1}
    --  report "Lenght of TSEG2(DBT) must be more than 1 clock cycle!";
    
    -- psl nominal_sample_cov : cover
    --  {sp_control = NOMINAL_SAMPLE};

    -- psl data_sample_cov : cover
    --  {sp_control = DATA_SAMPLE};

    -- psl secondary_sample_cov : cover
    --  {sp_control = SECONDARY_SAMPLE};

    -- psl minimal_bit_time_nbt_cov : cover
    --  {to_integer(unsigned(tseg1_nbt)) = 5 and to_integer(unsigned(tseg2_nbt)) = 3
    --   and to_integer(unsigned(brp_nbt)) = 1};

    -- psl minimal_bit_time_dbt_cov : cover
    --  {to_integer(unsigned(tseg1_dbt)) = 3 and to_integer(unsigned(tseg2_dbt)) = 2
    --   and to_integer(unsigned(brp_dbt)) = 1};

    -- psl brp_bin_1_1_cov : cover
    --  {to_integer(unsigned(brp_nbt)) = 1 and to_integer(unsigned(brp_dbt)) = 1};

    -- psl brp_bin_2_1_cov : cover
    --  {to_integer(unsigned(brp_nbt)) = 2 and to_integer(unsigned(brp_dbt)) = 1};

    -- psl brp_bin_3_1_cov : cover
    --  {to_integer(unsigned(brp_nbt)) = 3 and to_integer(unsigned(brp_dbt)) = 1};

    -- psl brp_bin_4_1_cov : cover
    --  {to_integer(unsigned(brp_nbt)) = 4 and to_integer(unsigned(brp_dbt)) = 1};
    
    -- psl brp_bin_4_2_cov : cover
    --  {to_integer(unsigned(brp_nbt)) = 4 and to_integer(unsigned(brp_dbt)) = 2};
    
    -- psl brp_bin_4_3_cov : cover
    --  {to_integer(unsigned(brp_nbt)) = 4 and to_integer(unsigned(brp_dbt)) = 3};
    
    -- psl brp_bin_1_2_cov : cover
    --  {to_integer(unsigned(brp_nbt)) = 1 and to_integer(unsigned(brp_dbt)) = 2};
    
    -- psl brp_nbt_max_cov : cover
    --  {to_integer(unsigned(brp_nbt)) = 255};
    
    -- psl brp_dbt_max_cov : cover
    --  {to_integer(unsigned(brp_dbt)) = 255};
    
    -- psl pos_resync_in_nominal_bit_rate_cov : cover
    --  {resync_edge_valid = '1' and is_tseg1 = '1' and sp_control = NOMINAL_SAMPLE}
    --  report "Positive resynchronization in Nominal bit rate!";

    -- psl neg_resync_in_nominal_bit_rate_cov : cover
    --  {resync_edge_valid = '1' and is_tseg2 = '1' and sp_control = NOMINAL_SAMPLE}
    --  report "Negative resynchronization in Nominal bit rate!";

    -- psl neg_resync_in_data_bit_rate_cov : cover
    --  {resync_edge_valid = '1' and is_tseg2 = '1' and sp_control = DATA_SAMPLE}
    --  report "Negative resynchronization in Data bit rate!";

    -- psl pos_resync_in_data_bit_rate_cov : cover
    --  {resync_edge_valid = '1' and is_tseg1 = '1' and sp_control = DATA_SAMPLE}
    --  report "Positive resynchronization in Data bit rate!";

    -- <RELEASE_ON>
    
end architecture;