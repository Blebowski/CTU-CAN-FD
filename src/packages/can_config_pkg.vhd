--------------------------------------------------------------------------------
--
-- CTU CAN FD IP Core
-- Copyright (C) 2021-2023 Ondrej Ille
-- Copyright (C) 2023-     Logic Design Services Ltd.s
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to use, copy, modify, merge, publish, distribute the Component for
-- non-commercial purposes. Using the Component for commercial purposes is
-- forbidden unless previously agreed with Copyright holder.
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
-- Package:
--  CAN Configuration
--
-- Purpose:
--  Package with Configuration of CTU CAN FD. Contains all contstants which
--  drive top level generics.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;

package can_config_pkg is

    -- IP Core version
    constant C_CTU_CAN_FD_VERSION_MINOR : std_logic_vector(7 downto 0) := x"07";
    constant C_CTU_CAN_FD_VERSION_MAJOR : std_logic_vector(7 downto 0) := x"02";

    -- Number of TXT Buffers
    constant C_TXT_BUFFER_COUNT     : natural := 4;

    -- Number of Interrupts
    constant C_INT_COUNT            : natural := 12;

    -- Width of control counter
    constant C_CTRL_CTR_WIDTH       : natural := 9;

    -- Width of retransmitt counter
    constant C_RETR_LIM_CTR_WIDTH   : natural := 4;

    -- TSEG1 Width
    -- SYNC (1) + PROP_NBT (127) + PH1_NBT (63) = 191 -> Fits into 8 bits
    -- SYNC (1) + PROP_DBT (63) + PH1_DBT (31) = 95 -> Fits into 7 bits
    constant C_TSEG1_WIDTH          : natural := 8;

    -- TSEG2 Width
    -- PH2_NBT (63) -> Fits into 6 bits
    -- PH2_DBT (31) -> Fits into 5 bits
    constant C_TSEG2_WIDTH          : natural := 6;

    -- Baud rate prescaler Width
    -- BRP_NBT (255) -> Fits into 8 bits
    -- BRP_DBT (255) -> Fits into 8 bits
    constant C_BRP_WIDTH            : natural := 8;

    -- Synchronisation Jump width Width
    -- SJW_NBT (31) -> Fits into 5 bits
    -- SJW_DBT (31) -> Fits into 5 bits
    constant C_SJW_WIDTH            : natural := 5;

    -- Secondary sampling point Shift registers length
    constant C_SSP_DELAY_SAT_VAL    : natural := 510;

    -- Depth of FIFO Cache for TX Data
    constant C_TX_CACHE_DEPTH       : natural := 8;

    -- Size of TX Data cache pointer
    constant C_TX_CACHE_PTR_WIDTH   : natural := 4;

    -- Width (number of bits) in transceiver delay measurement counter
    constant C_TRV_CTR_WIDTH        : natural := 8;

    -- Secondary sample point position width
    constant C_SSP_POS_WIDTH        : natural := 9;

    -- Secondary sample point offset width
    constant C_SSP_OFFSET_WIDTH     : natural := 8;

    -- Width of SSP counters
    constant C_SSP_CTRS_WIDTH       : natural := 15;

    -- CRC polynomials
    constant C_CRC15_POL            : std_logic_vector(15 downto 0) := x"C599";
    constant C_CRC17_POL            : std_logic_vector(19 downto 0) := x"3685B";
    constant C_CRC21_POL            : std_logic_vector(23 downto 0) := x"302899";

    -- Device ID
    constant C_CAN_DEVICE_ID        : std_logic_vector(15 downto 0) := x"CAFD";

end package;
