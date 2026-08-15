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
--  Bit Timing Logic
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity btl_top is
    generic (
        G_TSEG1_WIDTH           :     natural;
        G_TSEG2_WIDTH           :     natural;
        G_BRP_WIDTH             :     natural;
        G_SJW_WIDTH             :     natural
    );
    port(
        -------------------------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -------------------------------------------------------------------------------------------
        clk_sys                 : in  std_logic;
        rst_n                   : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Memory registers interface
        -------------------------------------------------------------------------------------------
        mr_btr_prop             : in  std_logic_vector(6 downto 0);
        mr_btr_ph1              : in  std_logic_vector(5 downto 0);
        mr_btr_ph2              : in  std_logic_vector(5 downto 0);
        mr_btr_brp              : in  std_logic_vector(7 downto 0);
        mr_btr_sjw              : in  std_logic_vector(4 downto 0);

        mr_btr_fd_prop_fd       : in  std_logic_vector(5 downto 0);
        mr_btr_fd_ph1_fd        : in  std_logic_vector(4 downto 0);
        mr_btr_fd_ph2_fd        : in  std_logic_vector(4 downto 0);
        mr_btr_fd_brp_fd        : in  std_logic_vector(7 downto 0);
        mr_btr_fd_sjw_fd        : in  std_logic_vector(4 downto 0);

        -------------------------------------------------------------------------------------------
        -- Control Interface to PC FSM
        -------------------------------------------------------------------------------------------
        sync_edge               : in  std_logic;
        bit_rate_d              : in  t_bit_rate;
        bit_rate_q              : in  t_bit_rate;
        sync_control            : in  std_logic_vector(1 downto 0);
        no_pos_resync           : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Trigger signals
        -------------------------------------------------------------------------------------------
        rx_trigger              : out std_logic;
        tx_trigger              : out std_logic;

        -------------------------------------------------------------------------------------------
        -- Status outputs
        -------------------------------------------------------------------------------------------
        tq_edge                 : out std_logic
    );
end entity;

architecture rtl of btl_top is

    signal is_tseg1     : std_logic;
    signal is_tseg2     : std_logic;

begin

    -----------------------------------------------------------------------------------------------
    -- Bit Time Logic FSM
    -----------------------------------------------------------------------------------------------
    i_btl_fsm : entity ctu_can_fd_rtl.btl_fsm
    port map (
        clk_sys                 => clk_sys,             -- IN
        rst_n                   => rst_n,               -- IN

        rx_trigger              => rx_trigger,          -- IN
        tx_trigger              => tx_trigger,          -- IN

        is_tseg1                => is_tseg1,            -- OUT
        is_tseg2                => is_tseg2             -- OUT
    );

    -----------------------------------------------------------------------------------------------
    -- Bit Time Logic Datapath
    -----------------------------------------------------------------------------------------------
    i_btl_datapath : entity ctu_can_fd_rtl.btl_datapath
    generic map (
        G_TSEG1_WIDTH           => G_TSEG1_WIDTH,
        G_TSEG2_WIDTH           => G_TSEG2_WIDTH,
        G_BRP_WIDTH             => G_BRP_WIDTH,
        G_SJW_WIDTH             => G_SJW_WIDTH
    )
    port map (
        -- Clock and Asynchronous reset
        clk_sys                 => clk_sys,                 -- IN
        rst_n                   => rst_n,                   -- IN

        -- Memory registers interface
        mr_btr_prop             => mr_btr_prop,             -- IN
        mr_btr_ph1              => mr_btr_ph1,              -- IN
        mr_btr_ph2              => mr_btr_ph2,              -- IN
        mr_btr_brp              => mr_btr_brp,              -- IN
        mr_btr_sjw              => mr_btr_sjw,              -- IN

        mr_btr_fd_prop_fd       => mr_btr_fd_prop_fd,       -- IN
        mr_btr_fd_ph1_fd        => mr_btr_fd_ph1_fd,        -- IN
        mr_btr_fd_ph2_fd        => mr_btr_fd_ph2_fd,        -- IN
        mr_btr_fd_brp_fd        => mr_btr_fd_brp_fd,        -- IN
        mr_btr_fd_sjw_fd        => mr_btr_fd_sjw_fd,        -- IN

        -- Control Interface from PC FSM
        sync_edge               => sync_edge,               -- IN
        bit_rate_d              => bit_rate_d,              -- IN
        bit_rate_q              => bit_rate_q,              -- IN
        sync_control            => sync_control,            -- IN
        no_pos_resync           => no_pos_resync,           -- IN

        -- Interface to BTL FSM
        is_tseg1                => is_tseg1,                -- IN
        is_tseg2                => is_tseg2,                -- IN

        -- Trigger signals
        rx_trigger              => rx_trigger,              -- OUT
        tx_trigger              => tx_trigger,              -- OUT

        -- Status outputs
        tq_edge                 => tq_edge                  -- OUT
    );

end architecture;