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
-- Module:
--  Trigger multiplexor.
--
-- Purpose:
--  Creates trigger (clock enable) signals for various control logic in MAC.
--  Creates following TX trigger signals when new bit starts:
--      1. Protocol control TX Trigger - Gated when a bit was stuffed.
--      2. Bit stuffing trigger
--      3. CRC TX No bit stuffing trigger - Gated when a stuff bit was
--         inserted after previous bit.
--      4. CRC TX With bit stuffing trigger - 1 clock cycle after TX trigger.
--         Gated when fixed stuff bit was inserted.
--  Creates following RX trigger signals at sample point:
--      1. Protocol control RX Trigger - Gated when a bit was destuffed.
--      2. Bit destuffing trigger
--      3. CRC RX With bit stuffing trigger - Gated when fixed stuff bit is
--         destuffed since CRC 17, 21 shall not be calculated from fixed stuff
--         bits.
--      4. CRC RX No bit stuffing trigger - Gated when a bit is destuffed.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

library ctu_can_fd_rtl;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity mac_trigger_mux is
    port (
        -------------------------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -------------------------------------------------------------------------------------------
        clk_sys                :in   std_logic;
        rst_n                  :in   std_logic;

        -------------------------------------------------------------------------------------------
        -- Input triggers
        -------------------------------------------------------------------------------------------
        -- RX Trigger
        rx_trigger             :in   std_logic;

        -- TX Trigger
        tx_trigger             :in   std_logic;

        -------------------------------------------------------------------------------------------
        -- Control signals
        -------------------------------------------------------------------------------------------
        -- Stuff bit is inserted, Protocol control operation to be halted for one bit time
        data_halt              :in   std_logic;

        -- Data output is not valid, actual bit is stuff bit.
        destuffed              :in   std_logic;

        -- Fixed bit stuffing method is used
        fixed_stuff            :in   std_logic;

        -------------------------------------------------------------------------------------------
        -- Output triggers
        -------------------------------------------------------------------------------------------
        -- Protocol control TX Trigger
        pc_tx_trigger          :out  std_logic;

        -- Protocol control RX Trigger
        pc_rx_trigger          :out  std_logic;

        -- CRC Trigger RX - No bit stuffing
        crc_trig_rx_nbs        :out  std_logic;

        -- CRC Trigger TX - No bit stuffing
        crc_trig_tx_nbs        :out  std_logic;

        -- CRC Trigger RX - With bit stuffing
        crc_trig_rx_wbs        :out  std_logic;

        -- CRC Trigger TX - With bit stuffing
        crc_trig_tx_wbs        :out  std_logic
    );
end entity;

architecture rtl of mac_trigger_mux is

    signal tx_trigger_q       :      std_logic;

begin

    -----------------------------------------------------------------------------------------------
    -- Protocol control triggers:
    --  1. Protocol control trigger (TX) - shifts TX Shift register, is enabled when stuff bit is
    --     not inserted! Active in Stuff pipeline stage.
    --  2. Protocol control trigger (RX) - shifts RX Shift register, is enabled when stuff bit is
    --     not destuffed! Active in Process pipeline stage.
    -----------------------------------------------------------------------------------------------
    pc_tx_trigger <= '1' when (tx_trigger = '1' and data_halt = '0') else
                     '0';

    pc_rx_trigger <= '1' when (rx_trigger = '1' and destuffed = '0') else
                     '0';

    -----------------------------------------------------------------------------------------------
    -- CRC Triggers for CRC 15 (CRC without stuff bits):
    --  1. CRC RX NBS - Trigger for CRC15 from RX data without bit stuffing. Trigger must be gated
    --     when bit was destuffed, because CRC15 for CAN 2.0 frames shall not take stuff bits into
    --     account!
    --  2. CRC TX NBS - Trigger for CRC15 from TX data without bit stuffing. Must be gated when
    --     stuff bit is inserted!
    -----------------------------------------------------------------------------------------------
    crc_trig_rx_nbs <= '1' when (rx_trigger = '1' and destuffed = '0') else
                       '0';

    crc_trig_tx_nbs <= '1' when (tx_trigger = '1' and data_halt = '0') else
                       '0';

    -----------------------------------------------------------------------------------------------
    -- CRC Trigger for CRC 17, 21 (with bit stuffing):
    --  1. CRC TX WBS - Trigger for CRC17, CRC21 from TX Data with bit stuffing. This trigger must
    --     be gated for fixed stuff bits since CRC17, CRC21 shall not contain fixed stuff bits!
    --     Active one clock cycle after Stuff pipeline stage.
    --  2. CRC RX WBS - Trigger for CRC17, CRC21 from RX Data with bit stuffing. Fixed stuff bits
    --     must be left out! Active in Process pipeline stage. (see next comment).
    -----------------------------------------------------------------------------------------------
    i_crc_trig_tx_wbs_reg : entity ctu_can_fd_rtl.dff_arst
    generic map(
        G_RESET_POLARITY   => '0',
        G_RST_VAL          => '0'
    )
    port map(
        arst               => rst_n,            -- IN
        clk                => clk_sys,          -- IN
        reg_d              => tx_trigger,       -- IN

        reg_q              => tx_trigger_q      -- OUT
    );

    crc_trig_tx_wbs <= '0' when (fixed_stuff = '1' and data_halt = '1') else
                       '1' when (tx_trigger_q = '1') else
                       '0';

    crc_trig_rx_wbs <= '0' when (fixed_stuff = '1' and destuffed = '1') else
                       '1' when (rx_trigger = '1') else
                       '0';

end architecture;