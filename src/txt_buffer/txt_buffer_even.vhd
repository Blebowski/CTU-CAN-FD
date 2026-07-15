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
--  TXT Buffer - Even index
--
-- Purpose:
--  Stores single frame for transmission in internal RAM. Accessed from Memory
--  registers via memory bus (to store frame) and SW commands.
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library ctu_can_fd_rtl;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

use ctu_can_fd_rtl.can_registers_pkg.all;

entity txt_buffer_even is
    generic (
        -- Number of TXT Buffers
        G_TXT_BUF_COUNT         :     natural range 2 to 8;

        -- TXT Buffer ID
        G_ID                    :     natural;

        -- Technology type
        G_TECHNOLOGY            :     natural;

        -- Support Parity Error
        G_PARITY_EN             :     boolean;

        -- TXT Buffer RAMs are resetable
        G_RESET_TXT_BUF_RAM     :     boolean
    );
    port (
        -------------------------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -------------------------------------------------------------------------------------------
        clk_sys                 : in  std_logic;
        res_n                   : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- DFT support
        -------------------------------------------------------------------------------------------
        scan_enable             : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Memory Registers Interface
        -------------------------------------------------------------------------------------------
        -- Control signals
        mr_mode_bmm             : in  std_logic;
        mr_mode_rom             : in  std_logic;
        mr_mode_txbbm           : in  std_logic;
        mr_settings_tbfbo       : in  std_logic;
        mr_settings_pchke       : in  std_logic;
        mr_tx_command_txce      : in  std_logic;
        mr_tx_command_txcr      : in  std_logic;
        mr_tx_command_txca      : in  std_logic;
        mr_tx_command_txbi      : in  std_logic;

        -- Memory Testability
        mr_tst_control_tmaena   : in  std_logic;
        mr_tst_control_twrstb   : in  std_logic;
        mr_tst_dest_tst_addr    : in  std_logic_vector(4 downto 0);
        mr_tst_dest_tst_mtgt    : in  std_logic_vector(3 downto 0);
        mr_tst_wdata_tst_wdata  : in  std_logic_vector(31 downto 0);
        mr_tst_rdata_tst_rdata  : out std_logic_vector(31 downto 0);

        -- Port A - Write Bus
        txtb_port_a_data_in     : in  std_logic_vector(31 downto 0);
        txtb_port_a_parity      : in  std_logic;
        txtb_port_a_address     : in  std_logic_vector(4 downto 0);
        txtb_port_a_cs          : in  std_logic;
        txtb_port_a_be          : in  std_logic_vector(3 downto 0);
        txtb_state              : out std_logic_vector(3 downto 0);

        -------------------------------------------------------------------------------------------
        -- Interrupt Manager Interface
        -------------------------------------------------------------------------------------------
        -- HW Command applied
        txtb_hw_cmd_int         : out std_logic;

        -------------------------------------------------------------------------------------------
        -- CAN Core and TX Arbitrator Interface
        -------------------------------------------------------------------------------------------
        -- HW Commands
        txtb_hw_cmd             : in  t_txtb_hw_cmd;

        -- HW commands chip select
        txtb_hw_cmd_cs          : in  std_logic;

        -- TXT Buffer RAM data output
        txtb_port_b_data_out    : out std_logic_vector(31 downto 0);

        -- TXT Buffer RAM address
        txtb_port_b_address     : in  std_logic_vector(4 downto 0);

        -- Clock enable to TXT Buffer port B
        txtb_port_b_clk_en      : in  std_logic;

        -- Unit just turned bus off.
        is_bus_off              : in  std_logic;

        -- TXT Buffer is available to be locked by CAN Core for transmission
        txtb_available          : out std_logic;

        -- TXT Buffer is in state for which its backup buffer can be used
        txtb_allow_bb           : out std_logic;

        -- Parity check valid
        txtb_parity_check_valid : in  std_logic;

        -- Parity error detected
        txtb_parity_mismatch    : out std_logic;

        -- Parity error really occured
        txtb_parity_error_valid : out std_logic;

        -- Index of TXT Buffer which is being read
        txtb_index_muxed        : in  natural range 0 to G_TXT_BUF_COUNT - 1
    );
end entity;

architecture rtl of txt_buffer_even is

    -----------------------------------------------------------------------------------------------
    -- Internal signals
    -----------------------------------------------------------------------------------------------

    -- TXT Buffer memory protection
    signal txtb_user_accessible         : std_logic;

    -- Unmask TXT Buffer RAM output
    signal txtb_unmask_data_ram         : std_logic;

    -- Output of TXT Buffer RAM
    signal txtb_port_b_data_out_i       : std_logic_vector(31 downto 0);

    -- TXT Buffer parity error
    signal txtb_parity_error_valid_i    : std_logic;

    -- TXT Buffer SW commands - registered
    signal mr_tx_command_txce_q         : std_logic;
    signal mr_tx_command_txcr_q         : std_logic;
    signal mr_tx_command_txca_q         : std_logic;

    -- Auxiliarly signals
    signal tx_command_txce_valid        : std_logic;
    signal tx_command_txcr_valid        : std_logic;
    signal abort_applied                : std_logic;
    signal abort_or_skipped             : std_logic;

    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    -- RAM wrapper signals
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    signal txtb_port_a_write            : std_logic;

    -- Clock enabled
    signal txtb_ram_clk_en              : std_logic;

    -- RAM clocks
    signal clk_ram                      : std_logic;

    -- Parity check
    signal parity_mismatch              : std_logic;

begin

    -- TXT Buffer RAM write signal
    txtb_port_a_write <= '1' when (txtb_port_a_cs = '1' and txtb_user_accessible = '1')
                             else
                         '0';

    -----------------------------------------------------------------------------------------------
    -- Output of TXT Buffer RAM is masked when it is not valid. This has several reasons:
    --  1. RAM content is undefined, therefore before filling RAM, XXXs on output if further
    --     comparator logic of TX Arbitrator will yell a lot. This saves from flood of simulation
    --     warnings!
    --  2. CAN Core and TX Arbitrator should not be reading any data from TXT Buffer RAM when it is
    --     not in Ready, TX in Progress or Abort in Progress (SW did not fill them yet). So we make
    --     sure that they are not used somewhere when they might be undefined yet!
    -----------------------------------------------------------------------------------------------
    txtb_port_b_data_out <= txtb_port_b_data_out_i when (txtb_unmask_data_ram = '1')
                                                   else
                                    (others => '0');

    -----------------------------------------------------------------------------------------------
    -- Clock gating for TXT Buffer RAM. Enable when:
    --  1. Read access from CAN core
    --  2. Write access from user
    --  3. Always in memory test mode, or in scan mode
    -----------------------------------------------------------------------------------------------
    txtb_ram_clk_en <= '1' when (txtb_port_b_clk_en = '1' or txtb_port_a_write = '1')
                           else
                       '1' when (mr_tst_control_tmaena = '1')
                           else
                       '0';

    -----------------------------------------------------------------------------------------------
    -- Parity error really occured (and STATUS[TXPE] can be set), only when TX Arbitrator or CAN
    -- Core have really read from the TXT Buffer, otherwise the output might be rubbish (uninited
    -- data, previous value).
    -----------------------------------------------------------------------------------------------
    txtb_parity_error_valid_i <= '1' when (parity_mismatch = '1' and
                                           txtb_parity_check_valid = '1' and
                                           txtb_index_muxed = G_ID)
                                     else
                                 '0';

    txtb_parity_error_valid <= txtb_parity_error_valid_i;


    -----------------------------------------------------------------------------------------------
    -- Register the TXT Buffer commands -> Breaks paths from memory bus
    -----------------------------------------------------------------------------------------------
    p_sw_command_reg : process(res_n, clk_sys)
    begin
        if (res_n = '0') then
            mr_tx_command_txce_q <= '0';
            mr_tx_command_txcr_q <= '0';
            mr_tx_command_txca_q <= '0';
        elsif (rising_edge(clk_sys)) then
            mr_tx_command_txce_q <= mr_tx_command_txce;
            mr_tx_command_txcr_q <= mr_tx_command_txcr;
            mr_tx_command_txca_q <= mr_tx_command_txca;
        end if;
    end process;

    tx_command_txce_valid <= '1' when (mr_tx_command_txce_q = '1' and mr_tx_command_txbi = '1')
                                 else
                             '0';
    tx_command_txcr_valid <= '1' when (mr_tx_command_txcr_q = '1' and mr_tx_command_txbi = '1')
                                 else
                             '0';

    abort_applied <= '1' when (mr_tx_command_txca_q = '1' and mr_tx_command_txbi = '1')
                         else
                     '0';

    -- Even TXT Buffer can't be skipped, only abort applies here
    abort_or_skipped <= abort_applied;

    -----------------------------------------------------------------------------------------------
    -- Clock gater for TXT Buffer RAM
    -----------------------------------------------------------------------------------------------
    i_clk_gate_txt_buffer_ram_comp : entity ctu_can_fd_rtl.clk_gate
    generic map (
        G_TECHNOLOGY            => G_TECHNOLOGY
    )
    port map (
        clk_in                  => clk_sys,                     -- IN
        clk_en                  => txtb_ram_clk_en,             -- IN
        scan_enable             => scan_enable,                 -- IN

        clk_out                 => clk_ram                      -- OUT
    );

    -----------------------------------------------------------------------------------------------
    -- RAM Memory of TXT Buffer
    -----------------------------------------------------------------------------------------------
    i_txt_buffer_ram : entity ctu_can_fd_rtl.txt_buffer_ram
    generic map (
        G_ID                    => G_ID,
        G_PARITY_EN            => G_PARITY_EN,
        G_RESET_TXT_BUF_RAM     => G_RESET_TXT_BUF_RAM
    )
    port map (
        -- Clock and Asynchronous reset
        clk_sys                 => clk_ram,                     -- IN
        res_n                   => res_n,                       -- IN

        -- Parity configuration
        mr_settings_pchke       => mr_settings_pchke,           -- IN

        -- Memory testability
        mr_tst_control_tmaena   => mr_tst_control_tmaena,       -- IN
        mr_tst_control_twrstb   => mr_tst_control_twrstb,       -- IN
        mr_tst_dest_tst_addr    => mr_tst_dest_tst_addr,        -- IN
        mr_tst_dest_tst_mtgt    => mr_tst_dest_tst_mtgt,        -- IN
        mr_tst_wdata_tst_wdata  => mr_tst_wdata_tst_wdata,      -- IN

        mr_tst_rdata_tst_rdata  => mr_tst_rdata_tst_rdata,      -- OUT

        -- Port A - Write (from Memory registers)
        txtb_port_a_address     => txtb_port_a_address,         -- IN
        txtb_port_a_data_in     => txtb_port_a_data_in,         -- IN
        txtb_port_a_parity      => txtb_port_a_parity,          -- IN
        txtb_port_a_write       => txtb_port_a_write,           -- IN
        txtb_port_a_be          => txtb_port_a_be,              -- IN

        -- Port B - Read (from CAN Core)
        txtb_port_b_address     => txtb_port_b_address,         -- IN
        txtb_port_b_data_out    => txtb_port_b_data_out_i,      -- OUT

        -- Parity check
        parity_mismatch         => parity_mismatch              -- OUT
    );

    -----------------------------------------------------------------------------------------------
    -- TXT Buffer FSM
    -----------------------------------------------------------------------------------------------
    i_txt_buffer_fsm : entity ctu_can_fd_rtl.txt_buffer_fsm
    port map (
        clk_sys                 => clk_sys,                     -- IN
        res_n                   => res_n,                       -- IN

        mr_mode_bmm             => mr_mode_bmm,                 -- IN
        mr_mode_rom             => mr_mode_rom,                 -- IN
        mr_settings_tbfbo       => mr_settings_tbfbo,           -- IN

        tx_command_txce_valid   => tx_command_txce_valid,       -- IN
        tx_command_txcr_valid   => tx_command_txcr_valid,       -- IN
        abort_applied           => abort_applied,               -- IN
        abort_or_skipped        => abort_or_skipped,            -- IN

        txtb_hw_cmd             => txtb_hw_cmd,                 -- IN
        txtb_hw_cmd_cs          => txtb_hw_cmd_cs,              -- IN
        is_bus_off              => is_bus_off,                  -- IN
        txtb_parity_error_valid => txtb_parity_error_valid_i,   -- IN

        txtb_allow_bb           => txtb_allow_bb,               -- OUT
        txtb_user_accessible    => txtb_user_accessible,        -- OUT
        txtb_hw_cmd_int         => txtb_hw_cmd_int,             -- OUT
        txtb_state              => txtb_state,                  -- OUT
        txtb_available          => txtb_available,              -- OUT
        txtb_unmask_data_ram    => txtb_unmask_data_ram         -- OUT
    );

    txtb_parity_mismatch <= parity_mismatch;

end architecture;