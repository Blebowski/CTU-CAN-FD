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
--  Bus traffic counters.
--
-- Purpose:
--  Counts number of transmitted and received frames on CAN Bus.
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

entity bus_traffic_counters is
    port (
        -------------------------------------------------------------------------------------------
        -- System clock and Asynchronous Reset
        -------------------------------------------------------------------------------------------
        clk_sys                 : in  std_logic;
        res_n                   : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- DFT support
        -------------------------------------------------------------------------------------------
        scan_enable             : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Control signals
        -------------------------------------------------------------------------------------------

        -- Frame transmission valid
        tran_valid              : in  std_logic;

        -- Frame reception valid
        rec_valid              : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Memory registers interface
        -------------------------------------------------------------------------------------------
        mr_command_rxfcrst      : in  std_logic;
        mr_command_txfcrst      : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Counter values
        -------------------------------------------------------------------------------------------
        -- TX Traffic counter
        tx_frame_ctr            : out std_logic_vector(31 downto 0);

        -- RX Traffic counter
        rx_frame_ctr            : out std_logic_vector(31 downto 0)
    );
end entity;

architecture rtl of bus_traffic_counters is

    signal tx_frame_ctr_i       : std_logic_vector(31 downto 0);
    signal rx_frame_ctr_i       : std_logic_vector(31 downto 0);

    -- Selected value to increment
    signal sel_value            : unsigned(31 downto 0);

    -- Incremented value by 1
    signal inc_value            : unsigned(31 downto 0);

    -- Reset signals for counters (registered, to avoid glitches)
    signal tx_ctr_rst_n_d       : std_logic;
    signal tx_ctr_rst_n_q_scan  : std_logic;

    signal rx_ctr_rst_n_d       : std_logic;
    signal rx_ctr_rst_n_q_scan  : std_logic;

    signal tran_valid_q         : std_logic;
    signal rec_valid_q          : std_logic;

begin

    -----------------------------------------------------------------------------------------------
    -- Register increment command (to relax timing through the counter!)
    -----------------------------------------------------------------------------------------------
    increment_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            tran_valid_q <= '0';
            rec_valid_q <= '0';
        elsif rising_edge(clk_sys) then
            tran_valid_q <= tran_valid;
            rec_valid_q <= rec_valid;
        end if;
    end process;

    tx_frame_ctr <= tx_frame_ctr_i;
    rx_frame_ctr <= rx_frame_ctr_i;

    -- Multiplexor between TX and RX value to increment
    sel_value <= unsigned(tx_frame_ctr_i) when (tran_valid_q = '1')
                                          else
                 unsigned(rx_frame_ctr_i);

    -- Incremented value of either TX or RX counter
    inc_value <= sel_value + 1;

    -----------------------------------------------------------------------------------------------
    -- Reset registers
    -----------------------------------------------------------------------------------------------
    tx_ctr_rst_n_d <= '0' when (mr_command_txfcrst = '1')
                          else
                      '1';

    rx_ctr_rst_n_d <= '0' when (mr_command_rxfcrst = '1')
                          else
                      '1';

    -----------------------------------------------------------------------------------------------
    -- Reset pipeline registers
    -----------------------------------------------------------------------------------------------
    tx_ctr_reg_rst_inst : entity ctu_can_fd_rtl.rst_reg
    generic map (
        G_RESET_POLARITY    => '0'
    )
    port map(
        -- Clock and Reset
        clk                 => clk_sys,                         -- IN
        arst                => res_n,                           -- IN

        -- Flip flop input / output
        d                   => tx_ctr_rst_n_d,                  -- IN
        q                   => tx_ctr_rst_n_q_scan,             -- OUT

        -- Scan mode control
        scan_enable         => scan_enable                      -- IN
    );

    rx_ctr_reg_rst_inst : entity ctu_can_fd_rtl.rst_reg
    generic map (
        G_RESET_POLARITY    => '0'
    )
    port map(
        -- Clock and Reset
        clk                 => clk_sys,                         -- IN
        arst                => res_n,                           -- IN

        -- Flip flop input / output
        d                   => rx_ctr_rst_n_d,                  -- IN
        q                   => rx_ctr_rst_n_q_scan,             -- OUT

        -- Scan mode control
        scan_enable         => scan_enable                      -- IN
    );

    -----------------------------------------------------------------------------------------------
    -- TX Counter register
    -----------------------------------------------------------------------------------------------
    tx_ctr_proc : process(clk_sys, tx_ctr_rst_n_q_scan)
    begin
        if (tx_ctr_rst_n_q_scan = '0') then
            tx_frame_ctr_i <= (others => '0');
        elsif rising_edge(clk_sys) then
            if (tran_valid_q = '1') then
                tx_frame_ctr_i <= std_logic_vector(inc_value);
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------------------------------
    -- RX Counter register
    -----------------------------------------------------------------------------------------------
    rx_ctr_proc : process(clk_sys, rx_ctr_rst_n_q_scan)
    begin
        if (rx_ctr_rst_n_q_scan = '0') then
            rx_frame_ctr_i <= (others => '0');
        elsif rising_edge(clk_sys) then
            if (rec_valid_q = '1') then
                rx_frame_ctr_i <= std_logic_vector(inc_value);
            end if;
        end if;
    end process;


    -- <RELEASE_OFF>
    -----------------------------------------------------------------------------------------------
    -- Assertions
    -----------------------------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);

    -- psl no_simul_inc_tx_rx_asrt : assert never
    -- (tran_valid = '1' and rec_valid = '1')
    -- report "Simultaneous increment of TX and RX error traffic counter";

    -- psl traffic_ctrs_tx_inc_cov : cover
    --  {tran_valid = '1'};

    -- psl traffic_ctrs_rx_inc_cov : cover
    --  {rec_valid = '1'};

    -- <RELEASE_ON>

end architecture;
