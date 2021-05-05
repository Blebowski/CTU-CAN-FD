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
--  Secondary Sampling point generator.
--
-- Purpose:
--  Create SSP by delaying TX trigger. First TX Trigger after bit-rate switch
--  is delayed by SSP Offset, each next by length of bit time. Length of bit
--  time is measured in multiples of clock cycles.
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

entity ssp_generator is
    generic(
        -- Width of SSP generator counters (BTMC, SSPC)
        G_SSP_CTRS_WIDTH     :      natural := 15
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Async reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys              :in   std_logic;

        -- Asynchronous reset
        res_n                :in   std_logic;       

        ------------------------------------------------------------------------
        -- Control signals
        ------------------------------------------------------------------------
        -- Reset Bit time measurement counter
        btmc_reset          :in    std_logic;

        -- Start Measurement of data bit time (in TX Trigger)
        dbt_measure_start   :in    std_logic;

        -- First SSP generated (in ESI bit)
        gen_first_ssp       :in    std_logic;

        -- SSP offset
        ssp_delay           :in    std_logic_vector(7 downto 0);

        -- SSP enable (SSP trigger gated when disabled)
        ssp_enable          :in    std_logic;

        -----------------------------------------------------------------------
        -- Trigger signals
        -----------------------------------------------------------------------
        -- TX Trigger
        tx_trigger          :in    std_logic;
        
        -- RX Trigger
        sample_sec          :out   std_logic
    );
end entity;

architecture rtl of ssp_generator is

    -- Bit time measuremend counter
    signal btmc_d           :   std_logic_vector(G_SSP_CTRS_WIDTH - 1 downto 0);
    signal btmc_q           :   std_logic_vector(G_SSP_CTRS_WIDTH - 1 downto 0);
    signal btmc_add         :   std_logic_vector(G_SSP_CTRS_WIDTH - 1 downto 0);
    signal btmc_ce          :   std_logic;

    -- Measurement running flag
    signal btmc_meas_running_d   : std_logic;
    signal btmc_meas_running_q   : std_logic;

    -- SSP counter
    signal sspc_d           :   std_logic_vector(G_SSP_CTRS_WIDTH - 1 downto 0);
    signal sspc_q           :   std_logic_vector(G_SSP_CTRS_WIDTH - 1 downto 0);
    signal sspc_ce          :   std_logic;
    signal sspc_expired     :   std_logic;
    signal sspc_threshold   :   std_logic_vector(G_SSP_CTRS_WIDTH - 1 downto 0);
    signal sspc_add         :   std_logic_vector(G_SSP_CTRS_WIDTH - 1 downto 0);
    
    constant C_SSPC_RST_VAL :   std_logic_vector(G_SSP_CTRS_WIDTH - 1 downto 0)
        := std_logic_vector(to_unsigned(1, G_SSP_CTRS_WIDTH));
    
    -- First SSP flag
    signal first_ssp_d      :   std_logic;
    signal first_ssp_q      :   std_logic;

    -- SSP running flag
    signal sspc_ena_d       :   std_logic;
    signal sspc_ena_q       :   std_logic;
    
    -- Padded SSP delay
    signal ssp_delay_padded :   std_logic_vector(G_SSP_CTRS_WIDTH - 1 downto 0);

begin

    ---------------------------------------------------------------------------
    -- Bit time measurement control:
    --  1. Reset on measurement restart.
    --  2. Start when commanded by Protocol control (ESI bit) and TX Trigger
    --     occurs!
    --  3. Stop on next TX Trigger (measurement should not be set then)
    ---------------------------------------------------------------------------
    btmc_meas_running_d <=
        '0' when (btmc_reset = '1') else
        '1' when (dbt_measure_start = '1' and tx_trigger = '1') else
        '0' when (tx_trigger = '1') else
        btmc_meas_running_q;

    btmc_meas_flag_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            btmc_meas_running_q <= '0';
        elsif (rising_edge(clk_sys)) then
            btmc_meas_running_q <= btmc_meas_running_d;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Bit time counter measurement:
    --  1. Reset
    --  2. Increment when measurement is running
    --  3. Keep value otherwise
    ---------------------------------------------------------------------------
    btmc_d <= (OTHERS => '0') when (btmc_reset = '1') else
                     btmc_add when (btmc_meas_running_q = '1') else
                       btmc_q;
    
    btmc_add <= std_logic_vector(unsigned(btmc_q) + 1);

    btmc_ce <= '1' when (btmc_d /= btmc_q) else
               '0';

    btmc_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            btmc_q <= (OTHERS => '0');    
        elsif (rising_edge(clk_sys)) then
            if (btmc_ce = '1') then
                btmc_q <= btmc_d;
            end if;
        end if;
    end process;
    
    --------------------------------------------------------------------------
    -- First SSP flag:
    --  1. Set in TX trigger of ESI (first SSP delay starts).
    --  2. Cleared when first SSP is reached.
    ---------------------------------------------------------------------------
    first_ssp_d <= '1' when (gen_first_ssp = '1'and tx_trigger = '1') else
                   '0' when (sspc_expired = '1') else
                   first_ssp_q;
                   
    first_ssp_flag_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            first_ssp_q <= '0'; 
        elsif (rising_edge(clk_sys)) then
            first_ssp_q <= first_ssp_d;
        end if;
    end process;


    --------------------------------------------------------------------------
    -- SSP counting:
    --  1. Enable on TX Trigger of ESI
    --  2. Disable when SSP gets disabled
    --------------------------------------------------------------------------
    sspc_ena_d <= '1' when (gen_first_ssp = '1' and tx_trigger = '1') else
                  '0' when (ssp_enable = '0') else
                  sspc_ena_q;

    sspc_run_flag_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            sspc_ena_q <= '0';
        elsif (rising_edge(clk_sys)) then
            sspc_ena_q <= sspc_ena_d;
        end if;
    end process;

    --------------------------------------------------------------------------
    --------------------------------------------------------------------------
    -- SSP Counter
    --------------------------------------------------------------------------    
    --------------------------------------------------------------------------    

    -- Pad SSP delay to size of SSP counters
    ssp_delay_padded(ssp_delay'high downto 0) <= ssp_delay;
    ssp_delay_padded(G_SSP_CTRS_WIDTH - 1 downto ssp_delay'high + 1) <=
        (OTHERS => '0');

    --------------------------------------------------------------------------
    -- SSP measurement threshold:
    --  1. Count till SSP offset in first SSP.
    --  2. Count till bit time length in further SSPs (measured by BTMC)
    ---------------------------------------------------------------------------
    sspc_threshold <= ssp_delay_padded when (first_ssp_q = '1') else
                      btmc_q;

    sspc_expired <= '1' when (sspc_q >= sspc_threshold) else
                    '0';

    sspc_add <= std_logic_vector(unsigned(sspc_q) + 1);

    --------------------------------------------------------------------------
    -- SSP counter:
    --  1. Reset before, or when expired (auto reset)
    --  2. Count when enabled
    --------------------------------------------------------------------------
    sspc_d <= C_SSPC_RST_VAL when (btmc_reset = '1' or sspc_expired = '1') else
                    sspc_add when (sspc_ena_q = '1') else
                      sspc_q;

    sspc_ce <= '1' when (sspc_d /= sspc_q) else
               '0';

    sspc_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            sspc_q <= C_SSPC_RST_VAL;
        elsif (rising_edge(clk_sys)) then
            if (sspc_ce = '1') then
                sspc_q <= sspc_d;
            end if;
        end if;
    end process;
    
    --------------------------------------------------------------------------
    -- Generation of SSP trigger
    --------------------------------------------------------------------------
    sample_sec <= '1' when (sspc_expired = '1' and sspc_ena_q = '1') else
                  '0';

    -- <RELEASE_OFF>
    ----------------------------------------------------------------------------
    -- Assertions
    ----------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);

    -- psl no_sspc_overflow : assert never
    --  (unsigned(sspc_d) < unsigned(sspc_q)) and (btmc_reset = '0' and sspc_expired = '0')
    -- report "SSPC overflow";

    -- psl no_btmc_overflow : assert never
    --  ((unsigned(btmc_d) < unsigned(btmc_q)) and (btmc_reset /= '1'))
    --  report "BTMC overflow";
    
    -- psl first_ssp_cov : cover
    --  {first_ssp_q = '1'};

    -- psl sspc_ena_q_cov : cover
    --  {sspc_ena_q = '1'};

    -- <RELEASE_ON>

end architecture;