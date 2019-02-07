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
--  Bit Time config capture
--
--  Captures Bit Timing configuration from BTR and BTR_FD registers when 
--  Core is enabled (drv_ena:0->1).
--  This has two advantages:
--      1. Register value is shadowed, thus chaning BTR or BTR FD when it is
--         not desired will avoid possible mal-function. Extra cost of few
--         DFFs is not that high.
--      2. This circuit re-calculates PROP+PH1 to TSEG1. The advantage is that
--         adder will not be in combinational path of bit timing calculation
--         and will thus not screw maximal frequency.
--  Each capture register is optinally configurable by generic. E.g. it might
--  not have sense to insert capture register on SJW (since there is no
--  combinational logic).
--------------------------------------------------------------------------------
-- Revision History:
--    03.02.2019   Created file
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

entity bit_time_cfg_capture is
    generic (
        -- Reset polarity
        reset_polarity : std_logic := '0';

        -- Insert Capture register for BTR, BTR_FD
        capt_btr        : boolean := false;
        
        -- Insert Capture register for TSEG1
        capt_tseg_1     : boolean := true;
        
        -- Insert Capture register for TSEG2
        capt_tseg_2     : boolean := false;
        
        -- Insert Capture register for SJW
        capt_sjw        : boolean := false;
        
        -- Nominal bit time widths
        tseg1_nbt_width : natural := 8;
        tseg2_nbt_width : natural := 8;
        tq_nbt_width    : natural := 8;
        sjw_nbt_width   : natural := 5;
        
        -- Data bit time widths
        tseg1_dbt_width : natural := 8;
        tseg2_dbt_width : natural := 8;
        tq_dbt_width    : natural := 8;
        sjw_dbt_width   : natural := 5
    );
    port(
        -----------------------------------------------------------------------
        -- Clock and reset
        -----------------------------------------------------------------------
        signal clk_sys          : in    std_logic;
        signal res_n            : in    std_logic;

        -----------------------------------------------------------------------
        -- Memory Registers interface and control signal
        -----------------------------------------------------------------------
        signal drv_bus          : in    std_logic_vector(1023 downto 0);

        -----------------------------------------------------------------------
        -- Output values
        -----------------------------------------------------------------------
        
        -- Time segment 1 - Nominal Bit Time
        signal tseg1_nbt  : out std_logic_vector(tseg1_nbt_width - 1 downto 0);
        
        -- Time segment 2 - Nominal Bit Time
        signal tseg2_nbt  : out std_logic_vector(tseg2_nbt_width - 1 downto 0);
        
        -- Baud Rate Prescaler - Nominal Bit Time
        signal brp_nbt    : out std_logic_vector(tq_nbt_width - 1 downto 0);
        
        -- Synchronisation Jump Width - Nominal Bit Time
        signal sjw_nbt    : out std_logic_vector(sjw_nbt_width - 1 downto 0);
        
        -- Time segment 1 - Data Bit Time
        signal tseg1_dbt  : out std_logic_vector(tseg1_dbt_width - 1 downto 0);
        
        -- Time segment 2 - Data Bit Time
        signal tseg2_dbt  : out std_logic_vector(tseg2_dbt_width - 1 downto 0);
        
        -- Baud Rate Prescaler - Data Bit Time
        signal brp_dbt    : out std_logic_vector(tq_dbt_width - 1 downto 0);
        
        -- Synchronisation Jump Width - Data Bit Time
        signal sjw_dbt    : out std_logic_vector(sjw_dbt_width - 1 downto 0)
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
    signal tseg1_nbt_nxt : std_logic_vector(tseg1_nbt_width - 1 downto 0);
    signal tseg1_dbt_nxt : std_logic_vector(tseg1_dbt_width - 1 downto 0);
    
    signal tseg2_nbt_nxt : std_logic_vector(tseg2_nbt_width - 1 downto 0);
    signal tseg2_dbt_nxt : std_logic_vector(tseg2_dbt_width - 1 downto 0);
    
    signal brp_nbt_nxt   : std_logic_vector(tq_nbt_width - 1 downto 0);
    signal brp_dbt_nxt   : std_logic_vector(tq_dbt_width - 1 downto 0);
    
    signal sjw_nbt_nxt   : std_logic_vector(sjw_nbt_width - 1 downto 0);
    signal sjw_dbt_nxt   : std_logic_vector(sjw_dbt_width - 1 downto 0);
    
    constant sync_length          :   unsigned(7 downto 0) := x"01";
    
    ---------------------------------------------------------------------------
    -- Drv ena edge detection
    ---------------------------------------------------------------------------
    signal drv_ena                :   std_logic;
    signal drv_ena_reg            :   std_logic;
    signal drv_ena_edge           :   std_logic;
    
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
        if (res_n = reset_polarity) then
            drv_ena_reg <= '0';
        elsif (rising_edge(clk_sys)) then
            drv_ena_reg <= drv_ena;
        end if;
    end process;

    -- Capture the configuration upon enabbling of the core.
    capture <= '1' when (drv_ena = '1' and drv_ena_reg = '0') else
               '0';

    ---------------------------------------------------------------------------
    -- Calculation of next values for capture registers
    ---------------------------------------------------------------------------
    tseg1_nbt_nxt   <= std_logic_vector(
                            resize(unsigned(drv_prs_nbt), tseg1_nbt_width) +
                            resize(unsigned(drv_ph1_nbt), tseg1_nbt_width) +
                            resize(sync_length, tseg1_nbt_width));

    tseg1_dbt_nxt   <= std_logic_vector(
                            resize(unsigned(drv_prs_dbt), tseg1_dbt_width) +
                            resize(unsigned(drv_ph1_dbt), tseg1_dbt_width) +
                            resize(sync_length, tseg1_dbt_width));
    
    tseg2_nbt_nxt <=
        std_logic_vector(resize(unsigned(drv_ph2_nbt), tseg2_nbt_width));
    tseg2_dbt_nxt <=
        std_logic_vector(resize(unsigned(drv_ph2_dbt), tseg2_dbt_width));
    
    brp_nbt_nxt     <=
        std_logic_vector(resize(unsigned(drv_tq_nbt), tq_nbt_width));
    brp_dbt_nxt     <=
        std_logic_vector(resize(unsigned(drv_tq_dbt), tq_dbt_width));
    
    sjw_nbt_nxt     <=
        std_logic_vector(resize(unsigned(drv_sjw_nbt), sjw_nbt_width));
    sjw_dbt_nxt     <=
        std_logic_vector(resize(unsigned(drv_sjw_dbt), sjw_nbt_width));
    
    ---------------------------------------------------------------------------
    -- Capture registers for BRP, BRP_FD 
    ---------------------------------------------------------------------------
    brp_capt_true_gen : if (capt_btr) generate
        brp_capt_proc : process(res_n, clk_sys)
        begin
            if (res_n = reset_polarity) then
                brp_nbt <= (OTHERS => '0');
                brp_dbt <= (OTHERS => '0');
            elsif (rising_edge(clk_sys)) then
                if (capture = '1') then
                    brp_nbt <= brp_nbt_nxt;
                    brp_dbt <= brp_dbt_nxt;
                end if;
            end if;
        end process;
    end generate brp_capt_true_gen;
    
    brp_capt_false_gen : if (not capt_btr) generate
        brp_nbt <= brp_nbt_nxt;
        brp_dbt <= brp_dbt_nxt;
    end generate brp_capt_false_gen;

    ---------------------------------------------------------------------------
    -- Capture registers for TSEG1 
    ---------------------------------------------------------------------------
    tseg1_capt_true_gen : if (capt_tseg_1) generate
        brp_capt_proc : process(res_n, clk_sys)
        begin
            if (res_n = reset_polarity) then
                tseg1_nbt <= (OTHERS => '0');
                tseg1_dbt <= (OTHERS => '0');
            elsif (rising_edge(clk_sys)) then
                if (capture = '1') then
                    tseg1_nbt <= tseg1_nbt_nxt;
                    tseg1_dbt <= tseg1_dbt_nxt;
                end if;
            end if;
        end process;
    end generate tseg1_capt_true_gen;
    
    tseg1_capt_false_gen : if (not capt_tseg_1) generate
        tseg1_nbt <= tseg1_nbt_nxt;
        tseg1_dbt <= tseg1_dbt_nxt;
    end generate tseg1_capt_false_gen;
        

    ---------------------------------------------------------------------------
    -- Capture registers for TSEG2 
    ---------------------------------------------------------------------------
    tseg2_capt_true_gen : if (capt_tseg_2) generate
        brp_capt_proc : process(res_n, clk_sys)
        begin
            if (res_n = reset_polarity) then
                tseg2_nbt <= (OTHERS => '0');
                tseg2_dbt <= (OTHERS => '0');
            elsif (rising_edge(clk_sys)) then
                if (capture = '1') then
                    tseg2_nbt <= tseg2_nbt_nxt;
                    tseg2_dbt <= tseg2_dbt_nxt;
                end if;
            end if;
        end process;
    end generate tseg2_capt_true_gen;
    
    tseg2_capt_false_gen : if (not capt_tseg_2) generate
        tseg2_nbt <= tseg2_nbt_nxt;
        tseg2_dbt <= tseg2_dbt_nxt;
    end generate tseg2_capt_false_gen;


    ---------------------------------------------------------------------------
    -- Capture registers for SJW 
    ---------------------------------------------------------------------------
    sjw_capt_true_gen : if (capt_sjw) generate
        brp_capt_proc : process(res_n, clk_sys)
        begin
            if (res_n = reset_polarity) then
                sjw_nbt <= (OTHERS => '0');
                sjw_dbt <= (OTHERS => '0');
            elsif (rising_edge(clk_sys)) then
                if (capture = '1') then
                    sjw_nbt <= sjw_nbt_nxt;
                    sjw_dbt <= sjw_dbt_nxt;
                end if;
            end if;
        end process;
    end generate sjw_capt_true_gen;
    
    sjw_capt_false_gen : if (not capt_sjw) generate
        sjw_nbt <= sjw_nbt_nxt;
        sjw_dbt <= sjw_dbt_nxt;
    end generate sjw_capt_false_gen;

end architecture;