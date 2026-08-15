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
--  Bit Timing Logic - FSM
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity btl_fsm is
    port (
        -------------------------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -------------------------------------------------------------------------------------------
        clk_sys             : in  std_logic;
        rst_n               : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Control interface
        -------------------------------------------------------------------------------------------
        rx_trigger          : in  std_logic;
        tx_trigger          : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Status signals
        -------------------------------------------------------------------------------------------
        is_tseg1            : out std_logic;
        is_tseg2            : out std_logic
    );
end entity;

architecture rtl of btl_fsm is

    -- Bit time FSM
    signal current_state    : t_bit_time;
    signal next_state       : t_bit_time;

begin

    -------------------------------------------------------------------------------------------
    -- Next state process (combinational)
    -------------------------------------------------------------------------------------------
    p_next_state : process(current_state, rx_trigger, tx_trigger)
    begin
        next_state <= current_state;

        case current_state is
        when s_bt_tseg1 =>
            if (rx_trigger = '1') then
                next_state <= s_bt_tseg2;
            end if;
        when s_bt_tseg2 =>
            if (tx_trigger = '1') then
                next_state <= s_bt_tseg1;
            end if;
        when s_bt_reset =>
            next_state <= s_bt_tseg1;
        end case;
    end process;

    -------------------------------------------------------------------------------------------
    -- Current state process (combinational)
    -------------------------------------------------------------------------------------------
    p_curr_state : process(current_state)
    begin
        -- Default values
        is_tseg1 <= '0';
        is_tseg2 <= '0';

        case current_state is
        when s_bt_reset =>
            -- To correctly preload Segment counters after reset!
            is_tseg2 <= '1';

        when s_bt_tseg1 =>
            is_tseg1 <= '1';

        when s_bt_tseg2 =>
            is_tseg2 <= '1';

        end case;
    end process;

    -------------------------------------------------------------------------------------------
    -- State register assignment
    -------------------------------------------------------------------------------------------
    p_state_reg : process(clk_sys, rst_n)
    begin
        if (rst_n = '0') then
            current_state <= s_bt_reset;
        elsif (rising_edge(clk_sys)) then
            current_state <= next_state;
        end if;
    end process;

end architecture rtl;