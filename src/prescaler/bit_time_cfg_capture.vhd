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
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;

use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity bit_time_cfg_capture is
    generic (
        -- TSEG1 Width - Nominal Bit Time
        G_TSEG1_NBT_WIDTH   :     natural;

        -- TSEG2 Width - Nominal Bit Time
        G_TSEG2_NBT_WIDTH   :     natural;

        -- Baud rate prescaler Width - Nominal Bit Time
        G_BRP_NBT_WIDTH     :     natural;

        -- Synchronisation Jump width Width - Nominal Bit Time
        G_SJW_NBT_WIDTH     :     natural;

        -- TSEG1 Width - Data Bit Time
        G_TSEG1_DBT_WIDTH   :     natural;

        -- TSEG2 Width - Data Bit Time
        G_TSEG2_DBT_WIDTH   :     natural;

        -- Baud rate prescaler width - Data Bit Time
        G_BRP_DBT_WIDTH     :     natural;

        -- Synchronisation Jump Width width - Data Bit Time
        G_SJW_DBT_WIDTH     :     natural
    );
    port (
        -------------------------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -------------------------------------------------------------------------------------------
        clk_sys             : in  std_logic;
        res_n               : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Memory Registers interface
        -------------------------------------------------------------------------------------------
        mr_settings_ena     : in  std_logic;

        mr_btr_prop         : in  std_logic_vector(6 downto 0);
        mr_btr_ph1          : in  std_logic_vector(5 downto 0);
        mr_btr_ph2          : in  std_logic_vector(5 downto 0);
        mr_btr_brp          : in  std_logic_vector(7 downto 0);
        mr_btr_sjw          : in  std_logic_vector(4 downto 0);

        mr_btr_fd_prop_fd   : in  std_logic_vector(5 downto 0);
        mr_btr_fd_ph1_fd    : in  std_logic_vector(4 downto 0);
        mr_btr_fd_ph2_fd    : in  std_logic_vector(4 downto 0);
        mr_btr_fd_brp_fd    : in  std_logic_vector(7 downto 0);
        mr_btr_fd_sjw_fd    : in  std_logic_vector(4 downto 0);

        -------------------------------------------------------------------------------------------
        -- Output values
        -------------------------------------------------------------------------------------------
        -- Time segment 1 - Nominal Bit Time
        tseg1_nbt           : out std_logic_vector(G_TSEG1_NBT_WIDTH - 1 downto 0);

        -- Time segment 2 - Nominal Bit Time
        tseg2_nbt           : out std_logic_vector(G_TSEG2_NBT_WIDTH - 1 downto 0);

        -- Baud Rate Prescaler - Nominal Bit Time
        brp_nbt             : out std_logic_vector(G_BRP_NBT_WIDTH - 1 downto 0);

        -- Synchronisation Jump Width - Nominal Bit Time
        sjw_nbt             : out std_logic_vector(G_SJW_NBT_WIDTH - 1 downto 0);

        -- Time segment 1 - Data Bit Time
        tseg1_dbt           : out std_logic_vector(G_TSEG1_DBT_WIDTH - 1 downto 0);

        -- Time segment 2 - Data Bit Time
        tseg2_dbt           : out std_logic_vector(G_TSEG2_DBT_WIDTH - 1 downto 0);

        -- Baud Rate Prescaler - Data Bit Time
        brp_dbt             : out std_logic_vector(G_BRP_DBT_WIDTH - 1 downto 0);

        -- Synchronisation Jump Width - Data Bit Time
        sjw_dbt             : out std_logic_vector(G_SJW_DBT_WIDTH - 1 downto 0);

        -- Signal to load the expected segment length by Bit time counters
        start_edge          : out std_logic
    );
end entity;

architecture rtl of bit_time_cfg_capture is

    -----------------------------------------------------------------------------------------------
    -- Next values for configuration.
    -----------------------------------------------------------------------------------------------
    signal tseg1_nbt_d      : std_logic_vector(G_TSEG1_NBT_WIDTH - 1 downto 0);
    signal tseg1_dbt_d      : std_logic_vector(G_TSEG1_DBT_WIDTH - 1 downto 0);

    constant sync_length    : unsigned(7 downto 0) := x"01";

    -----------------------------------------------------------------------------------------------
    -- Drv ena edge detection
    -----------------------------------------------------------------------------------------------
    signal drv_ena          : std_logic;
    signal drv_ena_reg      : std_logic;
    signal drv_ena_reg_2    : std_logic;

    -- Capture signal
    signal capture          : std_logic;

begin

    -----------------------------------------------------------------------------------------------
    -- SETTINGS[ENA] edge detection
    -----------------------------------------------------------------------------------------------
    drv_ena_reg_proc : process(res_n, clk_sys)
    begin
        if (res_n = '0') then
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

    -- Start edge is generated one clock cycle after the capture so that resynchronisation will
    -- capture correct values already!
    start_edge <= '1' when (drv_ena_reg_2 = '0' and drv_ena_reg = '1') else
                  '0';

    -----------------------------------------------------------------------------------------------
    -- Time segment 2
    -----------------------------------------------------------------------------------------------
    tseg2_nbt <= std_logic_vector(resize(unsigned(mr_btr_ph2),      G_TSEG2_NBT_WIDTH));
    tseg2_dbt <= std_logic_vector(resize(unsigned(mr_btr_fd_ph2_fd),G_TSEG2_DBT_WIDTH));

    -----------------------------------------------------------------------------------------------
    -- Synchronisation jump width
    -----------------------------------------------------------------------------------------------
    sjw_nbt <= std_logic_vector(resize(unsigned(mr_btr_sjw),        G_SJW_NBT_WIDTH));
    sjw_dbt <= std_logic_vector(resize(unsigned(mr_btr_fd_sjw_fd),  G_SJW_DBT_WIDTH));

    -----------------------------------------------------------------------------------------------
    -- Baud rate prescaler
    -----------------------------------------------------------------------------------------------
    brp_nbt <= std_logic_vector(resize(unsigned(mr_btr_brp),        G_BRP_NBT_WIDTH));
    brp_dbt <= std_logic_vector(resize(unsigned(mr_btr_fd_brp_fd),  G_BRP_DBT_WIDTH));

    -----------------------------------------------------------------------------------------------
    -- Calculation of next values for TSEG1 capture registers
    -----------------------------------------------------------------------------------------------
    tseg1_nbt_d   <= std_logic_vector(
                        resize(unsigned(mr_btr_prop),       G_TSEG1_NBT_WIDTH) +
                        resize(unsigned(mr_btr_ph1),        G_TSEG1_NBT_WIDTH) +
                        resize(sync_length,                 G_TSEG1_NBT_WIDTH));

    tseg1_dbt_d   <= std_logic_vector(
                        resize(unsigned(mr_btr_fd_prop_fd), G_TSEG1_DBT_WIDTH) +
                        resize(unsigned(mr_btr_fd_ph1_fd),  G_TSEG1_DBT_WIDTH) +
                        resize(sync_length,                 G_TSEG1_DBT_WIDTH));

    -----------------------------------------------------------------------------------------------
    -- Capture registers for TSEG1
    -----------------------------------------------------------------------------------------------
    brp_capt_proc : process(res_n, clk_sys)
    begin
        if (res_n = '0') then
            -- Matching reset values to what is in Memory registers.
            -- This is to make assertions which check valid bit time config
            -- happy, no impact on functionality!
            tseg1_nbt <= std_logic_vector(to_unsigned(9, G_TSEG1_NBT_WIDTH));
            tseg1_dbt <= std_logic_vector(to_unsigned(7, G_TSEG1_NBT_WIDTH));
        elsif (rising_edge(clk_sys)) then
            if (capture = '1') then
                tseg1_nbt <= tseg1_nbt_d;
                tseg1_dbt <= tseg1_dbt_d;
            end if;
        end if;
    end process;

end architecture;