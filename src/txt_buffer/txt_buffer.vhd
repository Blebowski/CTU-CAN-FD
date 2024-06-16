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
--  TXT Buffer
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
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;

use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

use ctu_can_fd_rtl.can_registers_pkg.all;

entity txt_buffer is
    generic (
        -- Number of TXT Buffers
        G_TXT_BUFFER_COUNT      :     natural range 2 to 8;

        -- TXT Buffer ID
        G_ID                    :     natural;

        -- Technology type
        G_TECHNOLOGY            :     natural;

        -- Support Parity Error
        G_SUP_PARITY            :     boolean;

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

        -- TXT Buffer is backup buffer
        txtb_is_bb              : in  std_logic;

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

        -- Index of TXT Buffer for which HW commands is valid
        txtb_hw_cmd_index       : in  natural range 0 to G_TXT_BUFFER_COUNT - 1;

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

        -- Parity error in Backup buffer
        txtb_bb_parity_error    : out std_logic;

        -- Index of TXT Buffer which is being read
        txtb_index_muxed        : in  natural range 0 to G_TXT_BUFFER_COUNT - 1
    );
end entity;

architecture rtl of txt_buffer is

    -----------------------------------------------------------------------------------------------
    -- Signal aliases
    -----------------------------------------------------------------------------------------------

    -- TXT Buffer memory protection
    signal txtb_user_accessible         : std_logic;

    -- Internal buffer selects for commands. Commands are shared across the buffers so we need
    -- unique identifier
    signal hw_cbs                       : std_logic;

    -- Unmask TXT Buffer RAM output
    signal txtb_unmask_data_ram         : std_logic;

    -- Output of TXT Buffer RAM
    signal txtb_port_b_data_out_i       : std_logic_vector(31 downto 0);

    -- TXT Buffer parity error
    signal txtb_parity_error_valid_i    : std_logic;

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

    -- Command buffer select signals
    hw_cbs <= '1' when (txtb_hw_cmd_index = G_ID)
                  else
              '0';

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
                                           txtb_index_muxed = G_ID and
                                           mr_settings_pchke = '1')
                                     else
                                 '0';

    txtb_parity_error_valid <= txtb_parity_error_valid_i;

    -----------------------------------------------------------------------------------------------
    -- If parity error occurs in Backup Buffer during TXTB modes, then set STATUS[TXDPE] = 1.
    -----------------------------------------------------------------------------------------------
    txtb_bb_parity_error <= '1' when (txtb_parity_error_valid_i = '1' and
                                      (G_ID mod 2) = 1 and
                                      mr_mode_txbbm = '1')
                                else
                            '0';


    clk_gate_txt_buffer_ram_comp : entity ctu_can_fd_rtl.clk_gate
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
    txt_buffer_ram_inst : entity ctu_can_fd_rtl.txt_buffer_ram
    generic map (
        G_ID                    => G_ID,
        G_SUP_PARITY            => G_SUP_PARITY,
        G_RESET_TXT_BUF_RAM     => G_RESET_TXT_BUF_RAM
    )
    port map (
        -- Clock and Asynchronous reset
        clk_sys                 => clk_ram,                     -- IN
        res_n                   => res_n,                       -- IN

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
    txt_buffer_fsm_inst : entity ctu_can_fd_rtl.txt_buffer_fsm
    port map (
        clk_sys                 => clk_sys,                     -- IN
        res_n                   => res_n,                       -- IN

        mr_mode_bmm             => mr_mode_bmm,                 -- IN
        mr_mode_rom             => mr_mode_rom,                 -- IN
        mr_settings_tbfbo       => mr_settings_tbfbo,           -- IN

        mr_tx_command_txce      => mr_tx_command_txce,          -- IN
        mr_tx_command_txcr      => mr_tx_command_txcr,          -- IN
        mr_tx_command_txca      => mr_tx_command_txca,          -- IN
        mr_tx_command_txbi      => mr_tx_command_txbi,          -- IN

        txtb_hw_cmd             => txtb_hw_cmd,                 -- IN
        hw_cbs                  => hw_cbs,                      -- IN
        is_bus_off              => is_bus_off,                  -- IN
        txtb_parity_error_valid => txtb_parity_error_valid_i,   -- IN
        txtb_is_bb              => txtb_is_bb,                  -- IN

        txtb_allow_bb           => txtb_allow_bb,               -- OUT
        txtb_user_accessible    => txtb_user_accessible,        -- OUT
        txtb_hw_cmd_int         => txtb_hw_cmd_int,             -- OUT
        txtb_state              => txtb_state,                  -- OUT
        txtb_available          => txtb_available,              -- OUT
        txtb_unmask_data_ram    => txtb_unmask_data_ram         -- OUT
    );

    txtb_parity_mismatch <= parity_mismatch;

    -- <RELEASE_OFF>
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    -- Functional coverage
    -----------------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------------
    func_cov_block : block
    begin

    -- psl default clock is rising_edge(clk_sys);

    -- Each SW command active
    -- psl txtb_set_ready_cov : cover {mr_tx_command_txcr = '1' and mr_tx_command_txbi = '1'};
    -- psl txtb_set_empty_cov : cover {mr_tx_command_txce = '1' and mr_tx_command_txbi = '1'};
    -- psl txtb_set_abort_cov : cover {mr_tx_command_txca = '1' and mr_tx_command_txbi = '1'};

    -- HW Commands
    -- psl txtb_hw_lock     : cover {txtb_hw_cmd.lock = '1'     and hw_cbs = '1'};
    -- psl txtb_hw_valid    : cover {txtb_hw_cmd.valid = '1'    and hw_cbs = '1'};
    -- psl txtb_hw_err      : cover {txtb_hw_cmd.err = '1'      and hw_cbs = '1'};
    -- psl txtb_hw_arbl     : cover {txtb_hw_cmd.arbl = '1'     and hw_cbs = '1'};
    -- psl txtb_hw_failed   : cover {txtb_hw_cmd.failed = '1'   and hw_cbs = '1'};

    -- psl txtb_double_parity_buf_1_cov : cover
    --    {txtb_bb_parity_error = '1'};

    -- psl txtb_parity_buf_cov : cover
    --    {txtb_parity_error_valid = '1'};

    end block;

    -- <RELEASE_ON>
end architecture;