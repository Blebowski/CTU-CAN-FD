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
--  Transcevier delay measurement.
--
-- Purpose:
--  Measures Transceiver delay and calculates position of secondary sampling
--  point.
-- 
--  Measurement is started by and edge on TX Data and Stopped by an edge on 
--  RX Data. Measurement is performed only when it is enabled by Protocol
--  Control, otherwise measured values remain unchanged.
-- 
--  Secondary sampling point is shadowed during measurement and propagated to 
--  output after the measurement. Data are loaded to output register upon the 
--  end of transceiver delay measurement and kept stable till the end of next
--  measurement. Configurable offset "ssp_offset" is implemented.
--
--  Shadowed value is muxed based on "ssp_delay_select":
--    1. Measured value + configured offset.
--    2. Configured offset only.
--
-- Circuit has following diagram:
--
--            SSP Delay select
--  ------------------------------------+
--                                      |   
--  Start Measurement                   |
--  ------------|                       |
--              |                       |
--   Stop       |                       |
--  Measur.     |                       |
--  ------      |                       |
--       |      |                       |
--       v      v                       |   Shadow registers
--   +--------------+                   |        load
--   |  Measurement +-------------------------------------------+
--   |     Flag     |                   |                       |
--   +------+-------+                   |                       |
--          |                           |                       |
--          | Measurement progress      |                       v
--          |                           |                  |----------|  TRV
--          |               ------------|------------------|  Shadow  | Delay
--          |               |           |                  | register |-------->
--   +------v-------+  Transceiver  XX  |                  |----------|
--   |  Transceiver |    Delay      | X v                       |
--   |     Delay    +-------+-----> |  X                        |
--   |    Counter   |       |       |   X                       v
--   +--------------+    +--v--+    |    X  |------------|  |----------|  SSP
--    SSP offset         |     |    |    X  |            |  |  Shadow  | Offset
--  -------------------> |  +  +--> |    X+-| Saturation |->+ register |------->
--                       |     |    |    X  |            |  |          |
--                       +-----+    |   X   |------------|  |----------|
--                                  |  X                    
--                                  | X                     
--                                  XX 
--
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.id_transfer.all;
use ctu_can_fd_rtl.can_constants.all;
use ctu_can_fd_rtl.can_components.all;
use ctu_can_fd_rtl.can_types.all;
use ctu_can_fd_rtl.cmn_lib.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.reduce_lib.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity trv_delay_measurement is
    generic(
        -- Reset polarity
        G_RESET_POLARITY         :     std_logic;
        
        -- Width (number of bits) in transceiver delay measurement counter
        G_TRV_CTR_WIDTH          :     natural := 7;
        
        -- Width of SSP position
        G_SSP_POS_WIDTH          :     natural := 8;

        -- Optional usage of saturated value of ssp_delay 
        G_USE_SSP_SATURATION     :     boolean := true;
        
        -- Saturation level for size of SSP_delay. This is to make sure that
        -- if there is smaller shift register for secondary sampling point we
        -- don't address outside of this register.
        G_SSP_SATURATION_LVL     :     natural
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys             :in   std_logic;
        
        -- Asynchronous reset        
        res_n               :in   std_logic;

        ------------------------------------------------------------------------
        -- Transceiver Delay measurement control
        ------------------------------------------------------------------------
        -- Start measurement (on TX Edge)
        edge_tx_valid       :in   std_logic;
        
        -- Stop measurement (on RX Edge)
        edge_rx_valid       :in   std_logic;
        
        -- Transmitter delay measurement enabled (by Protocol control)
        tran_delay_meas     :in   std_logic;

        ------------------------------------------------------------------------
        -- Memory registers interface
        ------------------------------------------------------------------------
        -- Secondary sampling point offset
        ssp_offset          :in   std_logic_vector(G_SSP_POS_WIDTH - 1 downto 0);

        -- Source of secondary sampling point 
        -- (Measured, Offset, Measured and Offset)
        ssp_delay_select    :in   std_logic_vector(1 downto 0);

        ------------------------------------------------------------------------
        -- Status outputs
        ------------------------------------------------------------------------
        
        -- Shadowed value of Transceiver delay. Updated when measurement ends.
        trv_delay_shadowed  :out  std_logic_vector(G_TRV_CTR_WIDTH - 1 downto 0);
                                               
        -- Shadowed value of SSP configuration. Updated when measurement ends.
        ssp_delay_shadowed  :out  std_logic_vector(G_SSP_POS_WIDTH - 1 downto 0)
    );
end entity;

architecture rtl of trv_delay_measurement is
    
    -- Transceiver delay measurement progress register
    signal trv_meas_progress_d    :    std_logic;
    signal trv_meas_progress_q    :    std_logic;
    
    -- Delayed value of trv_meas_progress to detect when measurement has ended
    -- and load ssp_offset shadow register.
    signal trv_meas_progress_del  :    std_logic;

    ---------------------------------------------------------------------------
    -- Transceiver delay counter
    ---------------------------------------------------------------------------
    signal trv_delay_ctr_q   :  std_logic_vector(G_TRV_CTR_WIDTH - 1 downto 0);
    signal trv_delay_ctr_d   :  std_logic_vector(G_TRV_CTR_WIDTH - 1 downto 0);
    signal trv_delay_ctr_add   :  std_logic_vector(G_TRV_CTR_WIDTH - 1 downto 0);
    
    signal trv_delay_ctr_q_padded : std_logic_vector(G_SSP_POS_WIDTH downto 0);

    -- Reset for the counter
    signal trv_delay_ctr_rst_d   :  std_logic;
    signal trv_delay_ctr_rst_q   :  std_logic;

    constant C_TRV_DEL_SAT  :  std_logic_vector(G_TRV_CTR_WIDTH - 1 downto 0) :=
        std_logic_vector(to_unsigned(127, G_TRV_CTR_WIDTH));

    ---------------------------------------------------------------------------
    -- SSP Shadowed register
    ---------------------------------------------------------------------------
    signal ssp_shadowed_nxt    :  std_logic;

    -- Load shadow register to output
    signal ssp_shadow_ce     :  std_logic;

    ---------------------------------------------------------------------------
    -- Shadowed value of transceiver delay counter
    ---------------------------------------------------------------------------
    -- Note that output counter is one bit wider than width of counter since
    -- output value can be addition of two values of trv_ctr_width size and
    -- we want to avoid overflow.
    signal ssp_delay_raw        :  std_logic_vector(G_SSP_POS_WIDTH downto 0);

    -- Saturated value of ssp_delay. If saturation is not used, ssp_delay_raw
    -- is connected directly
    signal ssp_delay_saturated  :  std_logic_vector(G_SSP_POS_WIDTH - 1 downto 0);
                                           
    -- Measured transceiver value + trv_offset
    signal trv_delay_sum        :  std_logic_vector(G_SSP_POS_WIDTH downto 0);

begin
    
    ----------------------------------------------------------------------------
    -- Next value of transceiver delay measurment flag:
    --  1. Clear immediately when measurement is disabled.
    --  2. Start measurement when enabled and "edge_tx_valid"
    --  3. Stop measurement when enabled  and "edge_rx_valid"
    --  4. Keep value otherwise.
    ----------------------------------------------------------------------------
    trv_meas_progress_d <= '0' when (tran_delay_meas = '0') else
                             '1' when (edge_tx_valid = '1') else
                             '0' when (edge_rx_valid = '1') else
                             trv_meas_progress_q;

    ----------------------------------------------------------------------------
    -- Register for transceiver delay measurement progress flag.
    ----------------------------------------------------------------------------
    trv_delay_prog_proc : process(res_n, clk_sys)
    begin
        if (res_n = G_RESET_POLARITY) then
            trv_meas_progress_q     <= '0';
            trv_meas_progress_del   <= '0';
        elsif (rising_edge(clk_sys)) then
            trv_meas_progress_q     <= trv_meas_progress_d;
            trv_meas_progress_del   <= trv_meas_progress_q;
        end if;
    end process;
    
    ----------------------------------------------------------------------------
    -- Reset counter for transceiver delay upon start of measurement.
    ----------------------------------------------------------------------------
    trv_delay_ctr_rst_d <= G_RESET_POLARITY when (tran_delay_meas = '1' and
                                                  edge_tx_valid = '1')
                                            else
                           not G_RESET_POLARITY; 
    
    trv_delay_rst_reg_inst : dff_arst
    generic map(
        G_RESET_POLARITY   => G_RESET_POLARITY,
        
        -- Reset to the same value as is polarity of reset so that other DFFs
        -- which are reset by output of this one will be reset too!
        G_RST_VAL          => G_RESET_POLARITY
    )
    port map(
        arst               => res_n,                -- IN
        clk                => clk_sys,              -- IN
        input              => trv_delay_ctr_rst_d,  -- IN

        output             => trv_delay_ctr_rst_q   -- OUT
    );
    
    ----------------------------------------------------------------------------
    -- Combinationally incremented value of trv_delay counter by 1.
    ----------------------------------------------------------------------------                                             
    trv_delay_ctr_add <= std_logic_vector(unsigned(trv_delay_ctr_q) + 1);

    -- Saturate when counter reaches 127, do not add anymore to avoid overflow!
    trv_delay_ctr_d <= C_TRV_DEL_SAT when (trv_delay_ctr_q = C_TRV_DEL_SAT)
                                     else
                      trv_delay_ctr_add;

    ----------------------------------------------------------------------------
    -- Register for transceiver delay measurement progress flag.
    ----------------------------------------------------------------------------
    trv_del_ctr_proc : process(clk_sys, trv_delay_ctr_rst_q)
    begin
        if (trv_delay_ctr_rst_q = G_RESET_POLARITY) then
            trv_delay_ctr_q(0) <= '1'; 
            trv_delay_ctr_q(G_TRV_CTR_WIDTH - 1 downto 1) <= (OTHERS => '0');

        elsif (rising_edge(clk_sys)) then
            
            -- Increment the counter if the measurement is in progress
            if (trv_meas_progress_q = '1') then
                trv_delay_ctr_q <= trv_delay_ctr_d;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Padding to width of SSP position width + 1 (for addition calculation)
    ---------------------------------------------------------------------------
    trv_delay_ctr_q_padded(G_TRV_CTR_WIDTH - 1 downto 0) <= trv_delay_ctr_q;

    trv_delay_ctr_q_padded(G_SSP_POS_WIDTH downto G_TRV_CTR_WIDTH) <=
        (OTHERS => '0');


    ---------------------------------------------------------------------------
    -- Combinationally adding ssp_offset and trv_delay_ctr_q.
    -- These are one bit wider to cover possible overflow!
    ---------------------------------------------------------------------------
    trv_delay_sum <= std_logic_vector(unsigned(trv_delay_ctr_q_padded) +
                                      ('0' & unsigned(ssp_offset)));

    ----------------------------------------------------------------------------
    -- Multiplexor for selected secondary sampling point delay. Selects:
    --  1. Measured trv_delay + ssp_offset
    --  2. ssp_offset only.
    ----------------------------------------------------------------------------
    with ssp_delay_select select ssp_delay_raw <=
                  trv_delay_sum when SSP_SRC_MEAS_N_OFFSET,
               '0' & ssp_offset when SSP_SRC_OFFSET,
                (OTHERS => '0') when others;

    ----------------------------------------------------------------------------
    -- SSP Delay saturation
    ----------------------------------------------------------------------------
    ssp_delay_sat_block : block
        constant C_SSP_SAT_LVL_VECT : std_logic_vector(G_SSP_POS_WIDTH - 1 downto 0) :=
            std_logic_vector(to_unsigned(G_SSP_SATURATION_LVL, G_SSP_POS_WIDTH));
    begin
        -- Use saturation
        ssp_sat_true : if (G_USE_SSP_SATURATION) generate

            -- Saturate if highest bit of result is set
            ssp_delay_saturated <=
                C_SSP_SAT_LVL_VECT when (ssp_delay_raw(G_SSP_POS_WIDTH) = '1') else
                ssp_delay_raw(G_SSP_POS_WIDTH - 1 downto 0);

        end generate ssp_sat_true;

        -- Don't use saturation
        ssp_sat_false : if (not G_USE_SSP_SATURATION) generate
            ssp_delay_saturated <= ssp_delay_raw(G_SSP_POS_WIDTH - 1 downto 0);
        end generate ssp_sat_false;
        
    end block ssp_delay_sat_block;


    ---------------------------------------------------------------------------
    -- SSP Shadow register. Both values are captured at the end of measurement.
    --  1. Transceiver Delay - Only measured value
    --  2. SSP Offset - Selected between measured, measured + offset,
    --                  offset.
    ---------------------------------------------------------------------------
    ssp_shadow_reg_proc : process(res_n, clk_sys)
    begin
        if (res_n = G_RESET_POLARITY) then
            ssp_delay_shadowed   <= (OTHERS => '0');
            trv_delay_shadowed   <= (OTHERS => '0');
            
        elsif (rising_edge(clk_sys)) then
            if (ssp_shadow_ce = '1') then
                ssp_delay_shadowed <= ssp_delay_saturated;
                trv_delay_shadowed <= trv_delay_ctr_q;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Load shadow register for secondary sampling point after the end of
    -- secondary sampling point measurement. This must be after the measurement
    -- has already ended so that last value is taken, not the value decremented
    -- by 1!
    ---------------------------------------------------------------------------
    ssp_shadow_ce <= '1' when (trv_meas_progress_del = '1') and
                                (trv_meas_progress_q = '0')
                         else
                     '0';

    -- <RELEASE_OFF>
    
    assert (G_TRV_CTR_WIDTH <= G_SSP_POS_WIDTH)
        report "SSP Position width must be higher or equal to trv counter width!"
        severity error;
    
    -- <RELEASE_ON>

end architecture;