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
--  Bus sampling
--
-- Sub-modules:
--  1. CAN RX synchronisation chain
--  2. Transceiver Delay measurement
--  3. Data edge detector
--  4. Secondary sampling point shift register.
--  5. TX Data cache.
--  6. Bit Error detector.
--  7. Sample multiplexor.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity bus_sampling is
    generic (
        -- Secondary sampling point Shift registers length
        G_SSP_DELAY_SAT_VAL     :     natural;

        -- Depth of FIFO Cache for TX Data
        G_TX_CACHE_DEPTH        :     natural;

        -- Width (number of bits) in transceiver delay measurement counter
        G_TRV_CTR_WIDTH         :     natural;

        -- Width of SSP position
        G_SSP_POS_WIDTH          :    natural;

        -- Optional usage of saturated value of ssp_delay
        G_USE_SSP_SATURATION    :     boolean;

        -- Width of SSP generator counters (BTMC, SSPC)
        G_SSP_CTRS_WIDTH        :     natural
    );
    port (
        -------------------------------------------------------------------------------------------
        -- Clock and Async reset
        -------------------------------------------------------------------------------------------
        clk_sys                 :in   std_logic;
        res_n                   :in   std_logic;

        -------------------------------------------------------------------------------------------
        -- DFT support
        -------------------------------------------------------------------------------------------
        scan_enable             :in   std_logic;

        -------------------------------------------------------------------------------------------
        -- Physical layer interface
        -------------------------------------------------------------------------------------------
        -- CAN serial stream output
        can_rx                  :in   std_logic;

        -- CAN serial stream input
        can_tx                  :out  std_logic;

        -------------------------------------------------------------------------------------------
        -- Memory registers interface
        -------------------------------------------------------------------------------------------
        mr_settings_ena         :in   std_logic;
        mr_ssp_cfg_ssp_offset   :in   std_logic_vector(7 downto 0);
        mr_ssp_cfg_ssp_src      :in   std_logic_vector(1 downto 0);

        -- Measured Transceiver delay
        trv_delay               :out  std_logic_vector(G_TRV_CTR_WIDTH - 1 downto 0);

        -------------------------------------------------------------------------------------------
        -- Prescaler interface
        -------------------------------------------------------------------------------------------
        -- RX Trigger
        rx_trigger              :in   std_logic;

        -- TX Trigger
        tx_trigger              :in   std_logic;

        -- Valid synchronisation edge appeared (Recessive to Dominant)
        sync_edge               :out  std_logic;

        -- Time quanta edge
        tq_edge                 :in   std_logic;

        -------------------------------------------------------------------------------------------
        -- CAN Core Interface
        -------------------------------------------------------------------------------------------
        -- TX data
        tx_data_wbs             :in   std_logic;

        -- RX data
        rx_data_wbs             :out  std_logic;

        -- Sample control
        sp_control              :in   std_logic_vector(1 downto 0);

        -- Reset for Secondary Sampling point Shift register.
        ssp_reset               :in   std_logic;

        -- Measure transmitter delay
        tran_delay_meas         :in   std_logic;

        -- Bit error detected
        bit_err                 :out  std_logic;

        -- Reset Bit time measurement counter
        btmc_reset              :in    std_logic;

        -- Start Measurement of data bit time (in TX Trigger)
        dbt_measure_start       :in    std_logic;

        -- First SSP generated (in ESI bit)
        gen_first_ssp           :in    std_logic;

        -- Bit error detection is enabled
        bit_err_enable          :in    std_logic
    );
end entity;

architecture rtl of bus_sampling is

    -------------------------------------------------------------------------------------------
    -- Internal registers and signals
    -------------------------------------------------------------------------------------------
    -- CAN RX Data (Synchronised)
    signal data_rx_synced           : std_logic;

    -- Bus sampling and edge detection, Previously sampled value on CAN bus
    signal prev_sample              : std_logic;

    -- Secondary sampling signal (sampling with transciever delay compensation)
    signal sample_sec               : std_logic;

    -- Delayed TX Data from TX Data shift register at position of secondary
    -- sampling point.
    signal data_tx_delayed          : std_logic;

    -- Appropriate edge appeared at recieved data
    signal edge_rx_valid            : std_logic;

    -- Edge appeared at transcieved data
    signal edge_tx_valid            : std_logic;

    --Note: Bit Error is set up at sample point for whole bit
    -- time until next sample point!!!!!

    -- SSP delay. Calculated from trv_delay either directly or by offseting
    -- by ssp_offset.
    signal ssp_delay                : std_logic_vector(7 downto 0);

    -- TX Trigger delayed by 1 clock cycle
    signal tx_trigger_q             : std_logic;

    -- TX Trigger (used for SSP)
    signal tx_trigger_ssp           : std_logic;

    -------------------------------------------------------------------------------------------
    -- Reset for shift registers. This is used instead of shift register with preload to lower
    -- the resource usage! Resetting and preloading to the same value can be merged into just
    -- resetting by OR of sources.
    -------------------------------------------------------------------------------------------
    signal shift_regs_res_d         : std_logic;
    signal shift_regs_res_q_scan    : std_logic;

    -- Enable for secondary sampling point shift register
    signal ssp_enable               : std_logic;

begin

    -------------------------------------------------------------------------------------------
    -- Synchronisation chain for input signal
    -------------------------------------------------------------------------------------------
    can_rx_sig_sync_inst : entity ctu_can_fd_rtl.sig_sync
    generic map (
        G_RESET_POLARITY            => '0',
        G_RESET_VALUE               => RECESSIVE
    )
    port map(
        arst                        => res_n,                   -- IN
        clk                         => clk_sys,                 -- IN
        async                       => can_rx,                  -- IN

        sync                        => data_rx_synced           -- OUT
    );

    -------------------------------------------------------------------------------------------
    -- Unit for measurement of transceiver delay and calculation of secondary sampling point.
    -------------------------------------------------------------------------------------------
    trv_delay_measurement_inst : entity ctu_can_fd_rtl.trv_delay_measurement
    generic map (
        G_TRV_CTR_WIDTH             => G_TRV_CTR_WIDTH,
        G_SSP_POS_WIDTH             => G_SSP_POS_WIDTH,
        G_USE_SSP_SATURATION        => G_USE_SSP_SATURATION,
        G_SSP_SATURATION_LVL        => G_SSP_DELAY_SAT_VAL
    )
    port map (
        clk_sys                     => clk_sys,                  -- IN
        res_n                       => res_n,                    -- IN

        scan_enable                 => scan_enable,              -- IN

        edge_tx_valid               => edge_tx_valid,            -- IN
        edge_rx_valid               => edge_rx_valid,            -- IN
        tran_delay_meas             => tran_delay_meas,          -- IN
        mr_ssp_cfg_ssp_offset       => mr_ssp_cfg_ssp_offset,    -- IN
        mr_ssp_cfg_ssp_src          => mr_ssp_cfg_ssp_src,       -- IN

        trv_delay_shadowed          => trv_delay,                -- OUT
        ssp_delay_shadowed          => ssp_delay                 -- OUT
    );



    -------------------------------------------------------------------------------------------
    -- Edge detector on TX, RX Data
    -------------------------------------------------------------------------------------------
    data_edge_detector_inst : entity ctu_can_fd_rtl.data_edge_detector
    port map (
        clk_sys                     => clk_sys,                 -- IN
        res_n                       => res_n,                   -- IN
        tx_data                     => tx_data_wbs,             -- IN
        rx_data                     => data_rx_synced,          -- IN
        prev_rx_sample              => prev_sample,             -- IN
        tq_edge                     => tq_edge,                 -- IN

        tx_edge                     => edge_tx_valid,           -- OUT
        rx_edge                     => edge_rx_valid,           -- OUT
        sync_edge                   => sync_edge                -- OUT
    );


    -------------------------------------------------------------------------------------------
    -- Reset for shift registers for secondary sampling point
    -------------------------------------------------------------------------------------------
    shift_regs_res_d <= '0' when (ssp_reset = '1') else
                        '1';

    -------------------------------------------------------------------------------------------
    -- Pipeline reset for shift registers to avoid glitches!
    -------------------------------------------------------------------------------------------
    shift_regs_rst_reg_inst : entity ctu_can_fd_rtl.rst_reg
    generic map (
        G_RESET_POLARITY            => '0'
    )
    port map (
        -- Clock and Reset
        clk                         => clk_sys,                 -- IN
        arst                        => res_n,                   -- IN

        -- Flip flop input / output
        d                           => shift_regs_res_d,        -- IN
        q                           => shift_regs_res_q_scan,   -- OUT

        -- Scan mode control
        scan_enable                 => scan_enable              -- IN
    );

    -------------------------------------------------------------------------------------------
    -- Create delayed TX Trigger one clock cycle after Stuff pipeline stage.
    -------------------------------------------------------------------------------------------
    tx_trigger_reg_inst : entity ctu_can_fd_rtl.dff_arst
    generic map (
        G_RESET_POLARITY            => '0',
        G_RST_VAL                   => '0'
    )
    port map (
        arst                        => res_n,                   -- IN
        clk                         => clk_sys,                 -- IN
        reg_d                       => tx_trigger,              -- IN

        reg_q                       => tx_trigger_q             -- OUT
    );

    -------------------------------------------------------------------------------------------
    -- Generator of secondary sampling point
    -------------------------------------------------------------------------------------------
    ssp_generator_inst : entity ctu_can_fd_rtl.ssp_generator
    generic map (
        G_SSP_CTRS_WIDTH            => G_SSP_CTRS_WIDTH
    )
    port map (
        -- Clock and Async reset
        clk_sys                     => clk_sys,                 -- (IN)
        res_n                       => res_n,                   -- (IN)

        -- Control signals
        btmc_reset                  => btmc_reset,              -- (IN)
        dbt_measure_start           => dbt_measure_start,       -- (IN)
        gen_first_ssp               => gen_first_ssp,           -- (IN)
        ssp_delay                   => ssp_delay,               -- (IN)
        ssp_enable                  => ssp_enable,              -- (IN)

        -- Trigger signals
        tx_trigger                  => tx_trigger,              -- (IN)
        sample_sec                  => sample_sec               -- (OUT)
    );

    -- Secondary sampling point shift register clock enable
    ssp_enable <= '1' when (sp_control = SECONDARY_SAMPLE) else
                  '0';

    -------------------------------------------------------------------------------------------
    -- Secondary sampling point input: Delayed TX Trigger gated and available only during
    -- secondary sampling! TX trigger for storing data to TX cache must be delayed since TX
    -- data will be one output of Bit Stuffing only one clock cycle after TX Trigger!
    -------------------------------------------------------------------------------------------
    tx_trigger_ssp <= '1' when (tx_trigger_q = '1' and sp_control = SECONDARY_SAMPLE)
                          else
                      '0';

    -------------------------------------------------------------------------------------------
    -- TX DATA Cache. Stores TX Data when Sample point enters the SSP shift register and reads
    -- data when Sample point steps out of shift register. This gets the TX data which
    -- correspond to the RX Bit in Secondary sampling point.
    -------------------------------------------------------------------------------------------
    tx_data_cache_inst : entity ctu_can_fd_rtl.tx_data_cache
    generic map (
        G_TX_CACHE_DEPTH            => G_TX_CACHE_DEPTH,
        G_TX_CACHE_RST_VAL          => RECESSIVE
    )
    port map (
        clk_sys                     => clk_sys,                 -- IN
        res_n                       => shift_regs_res_q_scan,   -- IN
        write                       => tx_trigger_ssp,          -- IN
        read                        => sample_sec,              -- IN
        data_in                     => tx_data_wbs,             -- IN

        data_out                    => data_tx_delayed          -- OUT
    );


    -------------------------------------------------------------------------------------------
    -- Bit error detector
    -------------------------------------------------------------------------------------------
    bit_err_detector_inst : entity ctu_can_fd_rtl.bit_err_detector
    port map (
        clk_sys                     => clk_sys,                 -- IN
        res_n                       => res_n,                   -- IN
        mr_settings_ena             => mr_settings_ena,         -- IN
        sp_control                  => sp_control,              -- IN
        rx_trigger                  => rx_trigger,              -- IN
        sample_sec                  => sample_sec,              -- IN
        bit_err_enable              => bit_err_enable,          -- IN
        data_tx                     => tx_data_wbs,             -- IN
        data_tx_delayed             => data_tx_delayed,         -- IN
        data_rx_synced              => data_rx_synced,          -- IN

        bit_err                     => bit_err                  -- OUT
    );

    -------------------------------------------------------------------------------------------
    -- Sampling of bus value
    -------------------------------------------------------------------------------------------
    sample_mux_inst : entity ctu_can_fd_rtl.sample_mux
    port map (
        clk_sys                     => clk_sys,                 -- IN
        res_n                       => res_n,                   -- IN
        mr_settings_ena             => mr_settings_ena,         -- IN
        sp_control                  => sp_control,              -- IN
        rx_trigger                  => rx_trigger,              -- IN
        sample_sec                  => sample_sec,              -- IN
        data_rx_synced              => data_rx_synced,          -- IN

        prev_sample                 => prev_sample              -- OUT
    );

    -- Output data propagation - Pipe directly - no delay
    can_tx      <= tx_data_wbs;

    -- RX Data for bit destuffing - Output of re-synchroniser.
    rx_data_wbs <= data_rx_synced;

    -- <RELEASE_OFF>
    -------------------------------------------------------------------------------------------
    -- Assertions on input signals
    -------------------------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);

    -- psl tx_trigger_ssp_cov : cover
    --  {tx_trigger_ssp = '1'};

    -- <RELEASE_ON>

end architecture;