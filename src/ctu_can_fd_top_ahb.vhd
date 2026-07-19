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
-- Purpose:
--    Top-level entity using AHB.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_config_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity ctu_can_fd_top_ahb is
    generic(
        -- RX Buffer RAM size (32 bit words)
        G_RX_BUF_SIZE           : natural range 32 to 4096  := 32;

        -- Number of supported TXT buffers
        G_TXT_BUF_COUNT         : natural range 2 to 8      := C_TXT_BUFFER_COUNT;

        -- Synthesize Filter A
        G_FILT_A_EN             : boolean                   := false;

        -- Synthesize Filter B
        G_FILT_B_EN             : boolean                   := false;

        -- Synthesize Filter C
        G_FILT_C_EN             : boolean                   := false;

        -- Synthesize Range Filter
        G_FILT_RANGE_EN         : boolean                   := false;

        -- Synthesize Test registers
        G_TEST_REGS_EN          : boolean                   := true;

        -- Insert Traffic counters
        G_TRAFFIC_CTRS_EN       : boolean                   := false;

        -- Add parity bit to TXT Buffer and RX Buffer RAMs
        G_PARITY_EN             : boolean                   := false;

        -- Number of active timestamp bits
        G_ACTIVE_TS_BITS        : natural range 0 to 63     := 63;

        -- Reset TXT / RX Buffer RAMs
        G_RESET_BUF_RAMS        : boolean                   := false;

        -- Target technology (ASIC or FPGA)
        G_TECHNOLOGY            : natural                   := C_TECH_FPGA
    );
    port(
        -----------------------------------------------------------------------
        -- AHB interface
        -----------------------------------------------------------------------
        hresetn          : in std_logic;
        hclk             : in std_logic;
        haddr            : in std_logic_vector(31 downto 0);
        hwdata           : in std_logic_vector(31 downto 0);
        hsel             : in std_logic;
        hwrite           : in std_logic;
        hsize            : in std_logic_vector(2 downto 0);
        hburst           : in std_logic_vector(2 downto 0);
        hprot            : in std_logic_vector(3 downto 0);
        htrans           : in std_logic_vector(1 downto 0);
        hmastlock        : in std_logic;
        hready           : in std_logic;
        hreadyout        : out std_logic;
        hresp            : out std_logic_vector(1 downto 0);
        hrdata           : out std_logic_vector(31 downto 0);

        -----------------------------------------------------------------------
        -- Synchronized reset
        -----------------------------------------------------------------------
        rst_n_out        : out std_logic;

        -----------------------------------------------------------------------
        -- CAN Bus
        -----------------------------------------------------------------------
        can_tx           : out std_logic;
        can_rx           : in  std_logic;

        -----------------------------------------------------------------------
        -- Timestamp
        -----------------------------------------------------------------------
        timestamp        : in  std_logic_vector(63 downto 0);

        -----------------------------------------------------------------------
        -- DFT support
        -----------------------------------------------------------------------
        scan_mode        : in  std_logic;

        -----------------------------------------------------------------------
        -- Interrupt
        -----------------------------------------------------------------------
        int              : out std_logic
    );
end entity ctu_can_fd_top_ahb;

architecture rtl of ctu_can_fd_top_ahb is

    signal ctu_can_data_in   : std_logic_vector(31 downto 0);
    signal ctu_can_data_out  : std_logic_vector(31 downto 0);
    signal ctu_can_adress    : std_logic_vector(15 downto 0);

    signal ctu_can_scs       : std_logic;
    signal ctu_can_srd       : std_logic;
    signal ctu_can_swr       : std_logic;
    signal ctu_can_sbe       : std_logic_vector(3 downto 0);

    signal rst_n_out_i       : std_logic;

begin

    i_can : entity ctu_can_fd_rtl.ctu_can_fd_top
    generic map (
        G_RX_BUF_SIZE           => G_RX_BUF_SIZE,
        G_TXT_BUF_COUNT         => G_TXT_BUF_COUNT,
        G_FILT_A_EN             => G_FILT_A_EN,
        G_FILT_B_EN             => G_FILT_B_EN,
        G_FILT_C_EN             => G_FILT_C_EN,
        G_FILT_RANGE_EN         => G_FILT_RANGE_EN,
        G_TEST_REGS_EN          => G_TEST_REGS_EN,
        G_TRAFFIC_CTRS_EN       => G_TRAFFIC_CTRS_EN,
        G_PARITY_EN             => G_PARITY_EN,
        G_ACTIVE_TS_BITS        => G_ACTIVE_TS_BITS,
        G_RESET_BUF_RAMS        => G_RESET_BUF_RAMS,
        G_TECHNOLOGY            => G_TECHNOLOGY
    )
    port map (
        clk_sys         => hclk,
        rst_n           => hresetn,
        rst_n_out       => rst_n_out_i,

        scan_mode       => scan_mode,

        data_in         => ctu_can_data_in,
        data_out        => ctu_can_data_out,
        adress          => ctu_can_adress,
        scs             => ctu_can_scs,
        srd             => ctu_can_srd,
        swr             => ctu_can_swr,
        sbe             => ctu_can_sbe,

        int             => int,

        CAN_tx          => CAN_tx,
        CAN_rx          => CAN_rx,

        timestamp       => timestamp
    );

    i_ahb_ifc : entity ctu_can_fd_rtl.ahb_ifc
    port map(
        -- CTU CAN FD Interface
        data_in          => ctu_can_data_in,
        data_out         => ctu_can_data_out,
        adress           => ctu_can_adress,
        sbe              => ctu_can_sbe,
        scs              => ctu_can_scs,
        swr              => ctu_can_swr,
        srd              => ctu_can_srd,

        -- AHB interface
        hresetn          => rst_n_out_i,
        hclk             => hclk,
        haddr            => haddr,
        hwdata           => hwdata,
        hsel             => hsel,
        hwrite           => hwrite,
        hsize            => hsize,
        hburst           => hburst,
        hprot            => hprot,
        htrans           => htrans,
        hmastlock        => hmastlock,
        hready           => hready,
        hreadyout        => hreadyout,
        hresp            => hresp,
        hrdata           => hrdata
    );

    rst_n_out <= rst_n_out_i;

end architecture rtl;