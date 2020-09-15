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
--  Bit Time config capture
--
-- Purpose:
--  Re-calculates Bit time settings as defined in Memory registers (SYNC, PROP,
--  PH1, PH2), to internal representation of Prescaler (TSEG1 and TSEG2).
--  TSEG1 = SYNC + PROP + PH1, is captured at moment when core is turned on to
--  avoid long combinational paths. TSEG2, SJW, BRP are passed directly, only
--  width is accustomized.
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

entity bit_time_cfg_capture is
    generic (
        -- Reset polarity
        G_RESET_POLARITY   : std_logic := '0';
        
        -- TSEG1 Width - Nominal Bit Time
        G_TSEG1_NBT_WIDTH  : natural := 8;
        
        -- TSEG2 Width - Nominal Bit Time
        G_TSEG2_NBT_WIDTH  : natural := 8;
        
        -- Baud rate prescaler Width - Nominal Bit Time
        G_BRP_NBT_WIDTH    : natural := 8;
        
        -- Synchronisation Jump width Width - Nominal Bit Time
        G_SJW_NBT_WIDTH    : natural := 5;
        
        -- TSEG1 Width - Data Bit Time
        G_TSEG1_DBT_WIDTH  : natural := 8;
        
        -- TSEG2 Width - Data Bit Time
        G_TSEG2_DBT_WIDTH  : natural := 8;
        
        -- Baud rate prescaler width - Data Bit Time
        G_BRP_DBT_WIDTH    : natural := 8;
        
        -- Synchronisation Jump Width width - Data Bit Time
        G_SJW_DBT_WIDTH    : natural := 5
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys     : in    std_logic;
        
        -- Asynchronous reset
        res_n       : in    std_logic;

        -----------------------------------------------------------------------
        -- Memory Registers interface
        -----------------------------------------------------------------------
        -- Driving Bus
        drv_bus     : in    std_logic_vector(1023 downto 0);

        -----------------------------------------------------------------------
        -- Output values
        -----------------------------------------------------------------------
        -- Time segment 1 - Nominal Bit Time
        tseg1_nbt   : out   std_logic_vector(G_TSEG1_NBT_WIDTH - 1 downto 0);
        
        -- Time segment 2 - Nominal Bit Time
        tseg2_nbt   : out   std_logic_vector(G_TSEG2_NBT_WIDTH - 1 downto 0);
        
        -- Baud Rate Prescaler - Nominal Bit Time
        brp_nbt     : out   std_logic_vector(G_BRP_NBT_WIDTH - 1 downto 0);
        
        -- Synchronisation Jump Width - Nominal Bit Time
        sjw_nbt     : out   std_logic_vector(G_SJW_NBT_WIDTH - 1 downto 0);
        
        -- Time segment 1 - Data Bit Time
        tseg1_dbt   : out   std_logic_vector(G_TSEG1_DBT_WIDTH - 1 downto 0);
        
        -- Time segment 2 - Data Bit Time
        tseg2_dbt   : out   std_logic_vector(G_TSEG2_DBT_WIDTH - 1 downto 0);
        
        -- Baud Rate Prescaler - Data Bit Time
        brp_dbt     : out   std_logic_vector(G_BRP_DBT_WIDTH - 1 downto 0);
        
        -- Synchronisation Jump Width - Data Bit Time
        sjw_dbt     : out   std_logic_vector(G_SJW_DBT_WIDTH - 1 downto 0);
        
        -- Signal to load the expected segment length by Bit time counters
        start_edge  : out   std_logic
    );
end entity;

architecture rtl of bit_time_cfg_capture is
    
    ---------------------------------------------------------------------------
    -- Driving Bus aliases
    ---------------------------------------------------------------------------
    -- Nominal
    signal drv_tq_nbt             :   std_logic_vector(7 downto 0);
    signal drv_prs_nbt            :   std_logic_vector(6 downto 0);
    signal drv_ph1_nbt            :   std_logic_vector(5 downto 0);
    signal drv_ph2_nbt            :   std_logic_vector(5 downto 0);
    signal drv_sjw_nbt            :   std_logic_vector(4 downto 0);
    
    -- Data
    signal drv_tq_dbt             :   std_logic_vector(7 downto 0);
    signal drv_prs_dbt            :   std_logic_vector(5 downto 0);
    signal drv_ph1_dbt            :   std_logic_vector(4 downto 0);
    signal drv_ph2_dbt            :   std_logic_vector(4 downto 0);
    signal drv_sjw_dbt            :   std_logic_vector(4 downto 0);
    
    ---------------------------------------------------------------------------
    -- Next values for configuration.
    ---------------------------------------------------------------------------
    signal tseg1_nbt_d : std_logic_vector(G_TSEG1_NBT_WIDTH - 1 downto 0);
    signal tseg1_dbt_d : std_logic_vector(G_TSEG1_DBT_WIDTH - 1 downto 0);
    
    constant sync_length          :   unsigned(7 downto 0) := x"01";
    
    ---------------------------------------------------------------------------
    -- Drv ena edge detection
    ---------------------------------------------------------------------------
    signal drv_ena                :   std_logic;
    signal drv_ena_reg            :   std_logic;
    signal drv_ena_reg_2          :   std_logic;
        
    -- Capture signal
    signal capture                :   std_logic;
    
begin

    ---------------------------------------------------------------------------
    -- Connect to Driving Bus
    ---------------------------------------------------------------------------
    drv_tq_nbt        <=  drv_bus(DRV_TQ_NBT_HIGH downto DRV_TQ_NBT_LOW);
    drv_tq_dbt        <=  drv_bus(DRV_TQ_DBT_HIGH downto DRV_TQ_DBT_LOW);
    drv_prs_nbt       <=  drv_bus(DRV_PRS_NBT_HIGH downto DRV_PRS_NBT_LOW);
    drv_ph1_nbt       <=  drv_bus(DRV_PH1_NBT_HIGH downto DRV_PH1_NBT_LOW);
    drv_ph2_nbt       <=  drv_bus(DRV_PH2_NBT_HIGH downto DRV_PH2_NBT_LOW);
    drv_prs_dbt       <=  drv_bus(DRV_PRS_DBT_HIGH downto DRV_PRS_DBT_LOW);
    drv_ph1_dbt       <=  drv_bus(DRV_PH1_DBT_HIGH downto DRV_PH1_DBT_LOW);
    drv_ph2_dbt       <=  drv_bus(DRV_PH2_DBT_HIGH downto DRV_PH2_DBT_LOW);
    drv_sjw_nbt       <=  drv_bus(DRV_SJW_HIGH downto DRV_SJW_LOW);
    drv_sjw_dbt       <=  drv_bus(DRV_SJW_DBT_HIGH downto DRV_SJW_DBT_LOW);

    drv_ena           <=  drv_bus(DRV_ENA_INDEX);

    ---------------------------------------------------------------------------
    -- DRV_ENA edge detection
    ---------------------------------------------------------------------------
    drv_ena_reg_proc : process(res_n, clk_sys)
    begin
        if (res_n = G_RESET_POLARITY) then
            drv_ena_reg     <= '0';
            drv_ena_reg_2   <= '0';
        elsif (rising_edge(clk_sys)) then
            drv_ena_reg     <= drv_ena;
            drv_ena_reg_2   <= drv_ena_reg;
        end if;
    end process;

    -- Capture the configuration upon enabbling of the core.
    capture <= '1' when (drv_ena = '1' and drv_ena_reg = '0') else
               '0';

    -- Start edge is generated one clock cycle after the capture so that
    -- resynchronisation will capture correct values already!
    start_edge <= '1' when (drv_ena_reg_2 = '0' and drv_ena_reg = '1') else
                  '0';
    
    ---------------------------------------------------------------------------
    -- Time segment 2 connection 
    ---------------------------------------------------------------------------
    tseg2_nbt <= std_logic_vector(resize(unsigned(drv_ph2_nbt), G_TSEG2_NBT_WIDTH));
    tseg2_dbt <= std_logic_vector(resize(unsigned(drv_ph2_dbt), G_TSEG2_DBT_WIDTH));
    
    ---------------------------------------------------------------------------
    -- Synchronisation jump width connection
    ---------------------------------------------------------------------------
    sjw_nbt <= std_logic_vector(resize(unsigned(drv_sjw_nbt), G_SJW_NBT_WIDTH));
    sjw_dbt <= std_logic_vector(resize(unsigned(drv_sjw_dbt), G_SJW_DBT_WIDTH));
    
    ---------------------------------------------------------------------------
    -- Baud rate prescaler connection 
    ---------------------------------------------------------------------------
    brp_nbt <= std_logic_vector(resize(unsigned(drv_tq_nbt), G_BRP_NBT_WIDTH));
    brp_dbt <= std_logic_vector(resize(unsigned(drv_tq_dbt), G_BRP_DBT_WIDTH));

    ---------------------------------------------------------------------------
    -- Calculation of next values for capture registers
    ---------------------------------------------------------------------------
    tseg1_nbt_d   <= std_logic_vector(
                        resize(unsigned(drv_prs_nbt), G_TSEG1_NBT_WIDTH) +
                        resize(unsigned(drv_ph1_nbt), G_TSEG1_NBT_WIDTH) +
                        resize(sync_length, G_TSEG1_NBT_WIDTH));

    tseg1_dbt_d   <= std_logic_vector(
                        resize(unsigned(drv_prs_dbt), G_TSEG1_DBT_WIDTH) +
                        resize(unsigned(drv_ph1_dbt), G_TSEG1_DBT_WIDTH) +
                        resize(sync_length, G_TSEG1_DBT_WIDTH));

    ---------------------------------------------------------------------------
    -- Capture registers for TSEG1 
    ---------------------------------------------------------------------------
    brp_capt_proc : process(res_n, clk_sys)
    begin
        if (res_n = G_RESET_POLARITY) then
            tseg1_nbt <= (OTHERS => '0');
            tseg1_dbt <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then
            if (capture = '1') then
                tseg1_nbt <= tseg1_nbt_d;
                tseg1_dbt <= tseg1_dbt_d;
            end if;
        end if;
    end process;

end architecture;