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
--  CAN CRC
--
-- Sub-modules:
--  CRC 15 - Calculated from data without bit stuffing.
--  CRC 17 - Calculated from data with bit stuffing.
--  CRC 21 - Calculated from data with bit stuffing.
--
-- Purpose:
--  Calculates crc sequences for CAN frame. Transmitter calculates CRC from
--  transmitted serial sequence. Receiver calculates data from RX sequence.
--  Final CRC is multiplexed on output. If node loses arbitration, source of
--  CRC calculation is changed.
--  Pipeline stages in which input is processed:
--   CRC 15 - Process (RX) / Stuff (TX)
--   CRC 17 - Process (RX) / Stuff + 1 clock cycle (TX)
--   CRC 21 - Process (RX) / Stuff + 1 clock cycle (TX)
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

entity can_crc is
    generic (
        -- CRC 15 polynomial
        G_CRC15_POL         :     std_logic_vector(15 downto 0) := x"C599";

        -- CRC 17 polynomial
        G_CRC17_POL         :     std_logic_vector(19 downto 0) := x"3685B";

        -- CRC 15 polynomial
        G_CRC21_POL         :     std_logic_vector(23 downto 0) := x"302899"
    );
    port (
        -------------------------------------------------------------------------------------------
        -- System clock and Asynchronous Reset
        -------------------------------------------------------------------------------------------
        clk_sys             : in  std_logic;
        res_n               : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Memory registers interface
        -------------------------------------------------------------------------------------------
        mr_settings_nisofd  : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Data inputs for CRC calculation
        -------------------------------------------------------------------------------------------
        -- TX Data with Bit Stuffing
        data_tx_wbs         : in  std_logic;

        -- TX Data without Bit Stuffing
        data_tx_nbs         : in  std_logic;

        -- RX Data with Bit Stuffing
        data_rx_wbs         : in  std_logic;

        -- RX Data without Bit Stuffing
        data_rx_nbs         : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Trigger signals to process the data on each CRC input.
        -------------------------------------------------------------------------------------------
        -- Trigger for TX Data with Bit Stuffing
        trig_tx_wbs         : in  std_logic;

        -- Trigger for TX Data without Bit Stuffing
        trig_tx_nbs         : in  std_logic;

        -- Trigger for RX Data with Bit Stuffing
        trig_rx_wbs         : in  std_logic;

        -- Trigger for RX Data without Bit Stuffing
        trig_rx_nbs         : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Control signals
        -------------------------------------------------------------------------------------------
        -- Enable for all CRC circuits.
        crc_enable          : in  std_logic;

        -- CRC calculation - speculative enable
        crc_spec_enable     : in  std_logic;

        -- Use RX Data for CRC calculation
        crc_calc_from_rx    : in  std_logic;

        -- Load CRC Initialization vector
        load_init_vect      : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- CRC Outputs
        -------------------------------------------------------------------------------------------
        crc_15              : out std_logic_vector(14 downto 0);
        crc_17              : out std_logic_vector(16 downto 0);
        crc_21              : out std_logic_vector(20 downto 0)
    );
end entity;

architecture rtl of can_crc is

    -- Initialization vectors
    signal init_vect_msb_17     :     std_logic;
    signal init_vect_msb_21     :     std_logic;

    -------------------------------------------------------------------------------------------
    -- Immediate outputs of CRC circuits
    -------------------------------------------------------------------------------------------

    -- Data inputs to CRC 17 and CRC 21
    signal crc_17_21_data_in    :     std_logic;

    -- Triggers for CRC 17 and 21
    signal crc_17_21_trigger    :     std_logic;

    -- Data inputs to CRC 15
    signal crc_15_data_in       :     std_logic;

    -- Triggers for CRC 15
    signal crc_15_trigger       :     std_logic;

    -- Internal enable signals
    signal crc_ena_15           :     std_logic;
    signal crc_ena_17_21        :     std_logic;

begin

    -----------------------------------------------------------------------------------------------
    -- For CRC 17 and 21, init vector MSB depends on ISO/NON-ISO type:
    --  ISO - Highest bit 1
    --  NON-ISO - All zeroes
    -----------------------------------------------------------------------------------------------
    init_vect_msb_17 <= '1' when (mr_settings_nisofd = ISO_FD)
                            else
                        '0';

    init_vect_msb_21 <= '1' when (mr_settings_nisofd = ISO_FD)
                            else
                        '0';

    -----------------------------------------------------------------------------------------------
    -- Muxes for CRC 17,21. For Receiver choose crc from RX Stream, for Transmitter use CRC from
    -- TX Stream.
    -----------------------------------------------------------------------------------------------
    crc_17_21_data_in <= data_rx_wbs when (crc_calc_from_rx = '1')
                                     else
                         data_tx_wbs;

    crc_17_21_trigger <= trig_rx_wbs when (crc_calc_from_rx = '1')
                                     else
                         trig_tx_wbs;

    -----------------------------------------------------------------------------------------------
    -- Muxes for CRC 15. For Receiver choose crc from RX Stream, for Transmitter use CRC from
    -- TX Stream.
    -----------------------------------------------------------------------------------------------
    crc_15_data_in <= data_rx_nbs when (crc_calc_from_rx = '1')
                                  else
                      data_tx_nbs;

    crc_15_trigger <= trig_rx_nbs when (crc_calc_from_rx = '1')
                                  else
                      trig_tx_nbs;

    -----------------------------------------------------------------------------------------------
    -- CRC circuits calculate data upon trigger when enabled by one of two enable signals:
    --  1  Regular - CRC processes data regardless of data value.
    --  2. Speculative - Processes data value only if it is DOMINANT! This corresponds to processing
    --     DOMINANT bit in IDLE, Suspend or Intermission and considering this bit as SOF! This bit
    --     must be included in CRC calculation, but only when it is DOMINANT!
    -----------------------------------------------------------------------------------------------
    crc_ena_15   <= '1' when (crc_enable = '1')
                        else
                    '1' when (crc_spec_enable = '1' and crc_15_data_in = DOMINANT)
                        else
                    '0';

    crc_ena_17_21  <= '1' when (crc_enable = '1')
                          else
                      '1' when (crc_spec_enable = '1' and crc_17_21_data_in = DOMINANT)
                          else
                      '0';

    -----------------------------------------------------------------------------------------------
    -- CRC 15 (from RX Data, no Bit Stuffing)
    -----------------------------------------------------------------------------------------------
    crc_calc_15_inst : entity ctu_can_fd_rtl.crc_calc
    generic map (
        G_CRC_WIDTH         => 15,
        G_POLYNOMIAL        => G_CRC15_POL
    )
    port map (
        res_n               => res_n,               -- IN
        clk_sys             => clk_sys,             -- IN

        data_in             => crc_15_data_in,      -- IN
        trig                => crc_15_trigger,      -- IN
        enable              => crc_ena_15,          -- IN
        init_vect_msb       => '0',                 -- IN
        load_init_vect      => load_init_vect,      -- IN

        crc                 => crc_15               -- OUT
    );

    -----------------------------------------------------------------------------------------------
    -- CRC 17 (from TX or RX Data, with Bit Stuffing)
    -----------------------------------------------------------------------------------------------
    crc_calc_17_rx_inst : entity ctu_can_fd_rtl.crc_calc
    generic map(
        G_CRC_WIDTH         => 17,
        G_POLYNOMIAL        => G_CRC17_POL
    )
    port map(
        res_n               => res_n,               -- IN
        clk_sys             => clk_sys,             -- IN

        data_in             => crc_17_21_data_in,   -- IN
        trig                => crc_17_21_trigger,   -- IN
        enable              => crc_ena_17_21,       -- IN
        init_vect_msb       => init_vect_msb_17,    -- IN
        load_init_vect      => load_init_vect,      -- IN

        crc                 => crc_17               -- OUT
    );


    -----------------------------------------------------------------------------------------------
    -- CRC 21 (from TX or RX Data, with Bit Stuffing)
    -----------------------------------------------------------------------------------------------
    crc_calc_21_rx_inst : entity ctu_can_fd_rtl.crc_calc
    generic map(
        G_CRC_WIDTH         => 21,
        G_POLYNOMIAL        => G_CRC21_POL
    )
    port map(
        res_n               => res_n,               -- IN
        clk_sys             => clk_sys,             -- IN

        data_in             => crc_17_21_data_in,   -- IN
        trig                => crc_17_21_trigger,   -- IN
        enable              => crc_ena_17_21,       -- IN
        init_vect_msb       => init_vect_msb_21,    -- IN
        load_init_vect      => load_init_vect,      -- IN

        crc                 => crc_21               -- OUT
    );

end architecture;
