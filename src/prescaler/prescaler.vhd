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

entity prescaler is
    generic(
      -- Reset polarity
      reset_polarity        :   std_logic := '0';
        
      -- Insertion of capture registers
      capt_btr              :   boolean := false;
      capt_tseg_1           :   boolean := true;
      capt_tseg_2           :   boolean := false;
      capt_sjw              :   boolean := false;
      
      -- Width of Bit time segments      
      tseg1_nbt_width       : natural := 8; 
      tseg2_nbt_width       : natural := 6;
      tq_nbt_width          : natural := 8;
      sjw_nbt_width         : natural := 5;
      
      tseg1_dbt_width       : natural := 7;
      tseg2_dbt_width       : natural := 5;
      tq_dbt_width          : natural := 8;
      sjw_dbt_width         : natural := 5;
      
      -- Length of information processing time (in clock cycles)
      ipt_length            : natural := 3;
      
      -- Number of signals in Sync trigger
      sync_trigger_count    : natural range 2 to 8 := 2;
    
      -- Number of signals in Sample trigger
      sample_trigger_count  : natural range 2 to 8 := 3
    );
    port(
    ---------------------------------------------------------------------------
    -- Clock and async reset
    ---------------------------------------------------------------------------
    signal clk_sys              :in std_logic;  --System clock
    signal res_n                :in std_logic;   --Async reset
    
    ---------------------------------------------------------------------------
    -- Bus synch Interface
    ---------------------------------------------------------------------------
    signal sync_edge            :in std_logic;        --Edge for synchronisation
    signal OP_State             :in oper_mode_type;   --Protocol control state
    
    --Driving Bus
    signal drv_bus              :in std_logic_vector(1023 downto 0); 
    
    ---------------------------------------------------------------------------
    -- Generated clock - Nominal bit time
    ---------------------------------------------------------------------------
    --Time quantum clock - Nominal bit time
    signal clk_tq_nbt           :out std_logic;
    
    --Time quantum - Data bit time
    signal clk_tq_dbt           :out std_logic;
    
    ---------------------------------------------------------------------------
    -- Sample signals and delayed signals
    ---------------------------------------------------------------------------
    signal sample_nbt   :out std_logic_vector(sample_trigger_count - 1 downto 0); 
    signal sample_dbt   :out std_logic_vector(sample_trigger_count - 1 downto 0);

    ---------------------------------------------------------------------------
    -- Sync Signals
    ---------------------------------------------------------------------------
    signal sync_nbt     :out std_logic_vector(sync_trigger_count - 1 downto 0);
    signal sync_dbt     :out std_logic_vector(sync_trigger_count - 1 downto 0);
    
    signal bt_FSM_out           :out bit_time_type;
    
    -- What is actual node transmitting on the bus
    signal data_tx              :in   std_logic;
    
    -- Validated hard synchronisation edge to start Protocol control FSM
    -- Note: Sync edge from busSync.vhd cant be used! If it comes during sample 
    --       nbt, sequence it causes errors! It needs to be strictly before or 
    --       strictly after this sequence!!! 
    signal hard_sync_edge_valid :out std_logic; 
    
    ---------------------------------------------------------------------------
    -- Bit timing and Synchronisation control
    ---------------------------------------------------------------------------
    signal sp_control           :in std_logic_vector(1 downto 0);
    signal sync_control         :in std_logic_vector(1 downto 0)
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
    signal tseg1_nbt :  std_logic_vector(tseg1_nbt_width - 1 downto 0);
    signal tseg2_nbt :  std_logic_vector(tseg2_nbt_width - 1 downto 0);
    signal brp_nbt   :  std_logic_vector(tq_nbt_width - 1 downto 0);
    signal sjw_nbt   :  std_logic_vector(sjw_nbt_width - 1 downto 0);

    -- Data Bit-rate
    signal tseg1_dbt :  std_logic_vector(tseg1_dbt_width - 1 downto 0);
    signal tseg2_dbt :  std_logic_vector(tseg2_dbt_width - 1 downto 0);
    signal brp_dbt   :  std_logic_vector(tq_dbt_width - 1 downto 0);
    signal sjw_dbt   :  std_logic_vector(sjw_dbt_width - 1 downto 0);
    
    -- No positive resynchronisation.
    -- TODO: Move this to operation control FSM!
    signal no_pos_resync  :  std_logic;

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

    -- Information processing Time has elapsed, TSEG2 may end
    signal ipt_ok               : std_logic;
     
    -- Size of internal Bit time counters.
    constant bt_width_nbt : natural :=
        max(tseg1_nbt_width, tseg2_nbt_width) + 1;
    constant bt_width_dbt : natural :=
        max(tseg1_dbt_width, tseg2_dbt_width) + 1;
   
    -- Bit time counter values. 
    signal bt_counter_nbt   : std_logic_vector(bt_width_nbt - 1 downto 0);
    signal bt_counter_dbt   : std_logic_vector(bt_width_dbt - 1 downto 0);
    
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
    
    constant nbt_ones   : std_logic_vector(bt_width_nbt - 1 downto 0) :=
        (OTHERS => '1');
    constant dbt_ones   : std_logic_vector(bt_width_nbt - 1 downto 0) :=
        (OTHERS => '1');

    -- Signal that expected semgent length should be loaded after restart!
    signal start_edge           : std_logic;
    
    -- Bit time counter clear
    signal bt_ctr_clear         : std_logic;

begin

    drv_ena <= drv_bus(DRV_ENA_INDEX);
    
    ---------------------------------------------------------------------------
    -- No positive resynchronisation detection.
    -- TODO: This will be moved to Operation control!
    ---------------------------------------------------------------------------
    no_pos_resync <= '1' when (OP_State = transciever and data_tx = DOMINANT)
                         else
                     '0';

    ---------------------------------------------------------------------------
    -- Bit time config capture
    ---------------------------------------------------------------------------
    bit_time_cfg_capture_comp : bit_time_cfg_capture
    generic map (
        reset_polarity  => reset_polarity,
        capt_btr        => capt_btr,
        capt_tseg_1     => capt_tseg_1,
        capt_tseg_2     => capt_tseg_2,
        capt_sjw        => capt_sjw,
        tseg1_nbt_width => tseg1_nbt_width,
        tseg2_nbt_width => tseg2_nbt_width,
        tq_nbt_width    => tq_nbt_width,
        sjw_nbt_width   => sjw_nbt_width,
        tseg1_dbt_width => tseg1_dbt_width,
        tseg2_dbt_width => tseg2_dbt_width,
        tq_dbt_width    => tq_dbt_width,
        sjw_dbt_width   => sjw_dbt_width
    )
    port map(
        clk_sys    => clk_sys,
        res_n      => res_n,
        drv_bus    => drv_bus,
        tseg1_nbt  => tseg1_nbt,
        tseg2_nbt  => tseg2_nbt,
        brp_nbt    => brp_nbt,
        sjw_nbt    => sjw_nbt,
        tseg1_dbt  => tseg1_dbt,
        tseg2_dbt  => tseg2_dbt,
        brp_dbt    => brp_dbt,
        sjw_dbt    => sjw_dbt,
        start_edge => start_edge
    );

    ---------------------------------------------------------------------------
    -- Synchronisation checker
    ---------------------------------------------------------------------------
    synchronisation_checker_comp : synchronisation_checker
    generic map(
        reset_polarity    => reset_polarity
    )
    port map(
        clk_sys           => clk_sys,
        res_n             => res_n,
        sync_control      => sync_control,
        sync_edge         => sync_edge,
        no_pos_resync     => no_pos_resync,
        segment_end       => segment_end,
        is_tseg1          => is_tseg1,
        is_tseg2          => is_tseg2,
        resync_edge_valid => resync_edge_valid,
        h_sync_edge_valid => h_sync_edge_valid
    );

    ---------------------------------------------------------------------------
    -- Information processing time checker
    ---------------------------------------------------------------------------
    ipt_checker_comp : ipt_checker
    generic map(
        reset_polarity => reset_polarity, 
        ipt_length     => ipt_length
    )
    port map(
        clk_sys        => clk_sys,
        res_n          => res_n,
        is_tseg2       => is_tseg2,
        ipt_gnt        => ipt_ok
    );

    ---------------------------------------------------------------------------
    -- Re-synchronisation (Nominal Bit Time)
    ---------------------------------------------------------------------------
    resynchronisation_nbt_comp : resynchronisation
    generic map(
        reset_polarity       => reset_polarity,
        sjw_width            => sjw_nbt_width,
        tseg1_width          => tseg1_nbt_width,
        tseg2_width          => tseg2_nbt_width,
        bt_width             => bt_width_nbt
    )
    port map(
        clk_sys              => clk_sys,
        res_n                => res_n,
        resync_edge_valid    => resync_edge_valid,
        ipt_ok               => ipt_ok,
        is_tseg1             => is_tseg1,
        is_tseg2             => is_tseg2,
        tseg_1               => tseg1_nbt,
        tseg_2               => tseg2_nbt,
        sjw                  => sjw_nbt,
        start_edge           => start_edge,
        bt_counter           => bt_counter_nbt,
        segm_end             => segment_end,
        h_sync_valid         => h_sync_valid,
        exit_segm_req        => exit_segm_req_nbt
    );
    
    
    ---------------------------------------------------------------------------
    -- Bit Time counter (Nominal Bit Time)
    ---------------------------------------------------------------------------
    bit_time_counters_nbt_comp : bit_time_counters
    generic map(
        reset_polarity  => reset_polarity,
        bt_width        => bt_width_nbt,
        tq_width        => tq_nbt_width
    )
    port map(
        clk_sys         => clk_sys,
        res_n           => res_n,
        prescaler       => brp_nbt,
        tq_reset        => bt_ctr_clear,
        bt_reset        => bt_ctr_clear,
        drv_ena         => drv_ena,
        tq_edge         => tq_edge_nbt,
        bt_counter      => bt_counter_nbt
    );
    

    ---------------------------------------------------------------------------
    -- Re-synchronisation (Data Bit Time)
    ---------------------------------------------------------------------------
    resynchronisation_dbt_comp : resynchronisation
    generic map(
        reset_polarity       => reset_polarity,
        sjw_width            => sjw_dbt_width,
        tseg1_width          => tseg1_dbt_width,
        tseg2_width          => tseg2_dbt_width,
        bt_width             => bt_width_dbt
    )
    port map(
        clk_sys              => clk_sys,
        res_n                => res_n,
        resync_edge_valid    => resync_edge_valid,
        ipt_ok               => ipt_ok,
        is_tseg1             => is_tseg1,
        is_tseg2             => is_tseg2,
        tseg_1               => tseg1_dbt,
        tseg_2               => tseg2_dbt,
        sjw                  => sjw_dbt,
        start_edge           => start_edge,
        bt_counter           => bt_counter_dbt,
        segm_end             => segment_end,
        h_sync_valid         => h_sync_valid,
        exit_segm_req        => exit_segm_req_dbt
    );
    
    
    ---------------------------------------------------------------------------
    -- Bit Time counter (Data Bit Time)
    ---------------------------------------------------------------------------
    bit_time_counters_dbt_comp : bit_time_counters
    generic map(
        reset_polarity  => reset_polarity,
        bt_width        => bt_width_dbt,
        tq_width        => tq_dbt_width
    )
    port map(
        clk_sys         => clk_sys,
        res_n           => res_n,
        prescaler       => brp_dbt,
        tq_reset        => bt_ctr_clear,
        bt_reset        => bt_ctr_clear,
        drv_ena         => drv_ena,
        tq_edge         => tq_edge_dbt,
        bt_counter      => bt_counter_dbt
    );

    ---------------------------------------------------------------------------
    -- End of Segment detector
    ---------------------------------------------------------------------------
    segment_end_detector_comp : segment_end_detector
    generic map(
        reset_polarity  => reset_polarity
    )
    port map(
        clk_sys            => clk_sys,
        res_n              => res_n,
        sp_control         => sp_control,
        h_sync_edge_valid  => h_sync_edge_valid, 
        exit_segm_req_nbt  => exit_segm_req_nbt,
        exit_segm_req_dbt  => exit_segm_req_dbt,
        ipt_ok             => ipt_ok,
        is_tseg1           => is_tseg1,
        is_tseg2           => is_tseg2,
        tq_edge_nbt        => tq_edge_nbt,
        tq_edge_dbt        => tq_edge_dbt,
        segm_end           => segment_end,
        h_sync_valid       => h_sync_valid,
        bt_ctr_clear       => bt_ctr_clear
    );
    
    
    ---------------------------------------------------------------------------
    -- Bit time FSM
    ---------------------------------------------------------------------------
    bit_time_fsm_comp : bit_time_fsm
    generic map(
        reset_polarity   => reset_polarity
    )
    port map(
        clk_sys          => clk_sys,
        res_n            => res_n,
        segm_end         => segment_end,
        h_sync_valid     => h_sync_valid,
        drv_ena          => drv_ena,
        is_tseg1         => is_tseg1,
        is_tseg2         => is_tseg2,
        sample_req       => sample_req,
        sync_req         => sync_req,
        bt_FSM_out       => bt_FSM_out
    );
    
    ---------------------------------------------------------------------------
    -- Trigger generator
    ---------------------------------------------------------------------------
    trigger_generator_comp : trigger_generator
    generic map(
        reset_polarity        => reset_polarity,
        sync_trigger_count    => sync_trigger_count,
        sample_trigger_count  => sample_trigger_count
    )
    port map(
        clk_sys     => clk_sys,
        res_n       => res_n,
        sample_req  => sample_req,
        sync_req    => sync_req,
        sp_control  => sp_control,
        sample_nbt  => sample_nbt,
        sample_dbt  => sample_dbt,
        sync_nbt    => sync_nbt,
        sync_dbt    => sync_dbt
    );
    
    ---------------------------------------------------------------------------
    -- Internal signals to output propagation
    ---------------------------------------------------------------------------
    hard_sync_edge_valid <= h_sync_valid;
    
    clk_tq_nbt  <= tq_edge_nbt;
    clk_tq_dbt  <= tq_edge_dbt;
    
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
    --

end architecture;