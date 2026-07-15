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
--  Transmitt Frame Buffer FSM.
--
-- Purpose:
--  Handles HW commands from Protocol control and SW commands from Memory
--  registers.
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

entity txt_buffer_fsm is
    port (
        -------------------------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -------------------------------------------------------------------------------------------
        clk_sys                 : in  std_logic;
        rst_n                   : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Memory registers interface
        -------------------------------------------------------------------------------------------
        mr_mode_bmm             : in  std_logic;
        mr_mode_rom             : in  std_logic;
        mr_settings_tbfbo       : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Control signals
        -------------------------------------------------------------------------------------------
        tx_command_txce_valid   : in  std_logic;
        tx_command_txcr_valid   : in  std_logic;
        abort_applied           : in  std_logic;
        abort_or_skipped        : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- CAN Core interface
        -------------------------------------------------------------------------------------------
        -- HW Commands
        txtb_hw_cmd             : in  t_txtb_hw_cmd;

        -- HW Buffer select
        txtb_hw_cmd_cs          : in  std_logic;

        -- Unit is Bus off
        is_bus_off              : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Parity checking logic
        -------------------------------------------------------------------------------------------
        -- Parity Error
        txtb_parity_error_valid : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Status signals
        -------------------------------------------------------------------------------------------
        -- Buffer accessible from SW
        txtb_user_accessible    : out std_logic;

        -- TXT Buffer is in state for which its backup buffer can be used
        txtb_allow_bb           : out std_logic;

        -- HW Command applied on TXT Buffer.
        txtb_hw_cmd_int         : out std_logic;

        -- Buffer status (FSM state) encoded for reading by SW.
        txtb_state              : out std_logic_vector(3 downto 0);

        -- TXT Buffer is available to be locked by CAN Core for transmission
        txtb_available          : out std_logic;

        -- UnMask content of TXT Buffer RAM
        txtb_unmask_data_ram    : out std_logic
    );
end entity;

architecture rtl of txt_buffer_fsm is

    -- FSM signals
    signal next_state           : t_txt_buf_state;
    signal curr_state           : t_txt_buf_state;

    -- TXT Buffer clock enable
    signal txt_fsm_ce           : std_logic;

    -- Forced transition to failed state
    signal go_to_failed         : std_logic;
    signal transient_state      : std_logic;

    -- Internal HW Command, filtered by HW chip select for this Buffer
    signal txtb_hw_cmd_i         : t_txtb_hw_cmd;

    -- Arbitration lost or error valid fro current TXT Buffer
    signal arbl_or_err           : std_logic;

begin

    transient_state <= '1' when ((curr_state = s_txt_ab_prog) or
                                 (curr_state = s_txt_tx_prog) or
                                 (curr_state = s_txt_ready))
                          else
                      '0';

    go_to_failed <= '1' when ((is_bus_off = '1' and mr_settings_tbfbo = TXTBUF_FAILED_BUS_OFF_ENABLED) or
                              mr_mode_bmm = BMM_ENABLED or
                              mr_mode_rom = ROM_ENABLED)
                        else
                    '0';

    txtb_hw_cmd_i <= txtb_hw_cmd when (txtb_hw_cmd_cs = '1')
                                 else
                     (others => '0');

    arbl_or_err <= '1' when (txtb_hw_cmd_i.err = '1' or txtb_hw_cmd_i.arbl = '1') else
                   '0';

    txtb_allow_bb <= transient_state;

    -----------------------------------------------------------------------------------------------
    -- Next state process
    -----------------------------------------------------------------------------------------------
    p_tx_buf_fsm_next_state : process(curr_state, tx_command_txce_valid, tx_command_txcr_valid,
        txtb_hw_cmd_i, abort_applied, go_to_failed, txtb_parity_error_valid, abort_or_skipped,
        arbl_or_err)
    begin
        next_state <= curr_state;

        case curr_state is

        -------------------------------------------------------------------------------------------
        -- Buffer is empty
        -------------------------------------------------------------------------------------------
        when s_txt_empty =>

            -- "Set_ready"
            if (tx_command_txcr_valid = '1') then
                next_state <= s_txt_ready;
            end if;

        -------------------------------------------------------------------------------------------
        -- Buffer is ready for transmission
        -------------------------------------------------------------------------------------------
        when s_txt_ready =>

            -- Node became bus-off
            if (go_to_failed = '1') then
                next_state <= s_txt_failed;

            -- Parity Error occured
            elsif (txtb_parity_error_valid = '1') then
                next_state <= s_txt_parity_err;

            -- Locking for transmission
            elsif (txtb_hw_cmd_i.lock = '1') then

                -- Simultaneous "lock" and abort -> transmit, but with abort pending
                if (abort_applied = '1') then
                    next_state <= s_txt_ab_prog;
                else
                    next_state <= s_txt_tx_prog;
                end if;

            -- Abort the ready buffer or Skip the original TXT Buffer getting "failed" or "OK".
            elsif (abort_or_skipped = '1') then
                next_state <= s_txt_aborted;
            end if;

        -------------------------------------------------------------------------------------------
        -- Transmission from buffer is in progress
        -------------------------------------------------------------------------------------------
        when s_txt_tx_prog =>

            -- Node became bus-off
            if (go_to_failed = '1') then
                next_state <= s_txt_failed;

            -- Parity Error occured
            elsif (txtb_parity_error_valid = '1') then
                next_state <= s_txt_parity_err;

            -- Retransmitt reached, do not try again
            elsif (txtb_hw_cmd_i.failed = '1') then
                next_state <= s_txt_failed;

            -- Transmission OK
            elsif (txtb_hw_cmd_i.valid = '1') then
                next_state <= s_txt_ok;

            -- Error or arbitration lost
            elsif (arbl_or_err = '1') then
                if (abort_applied = '1') then
                    next_state <= s_txt_aborted;
                else
                    next_state <= s_txt_ready;
                end if;

            -- Request abort during transmission
            elsif (abort_applied = '1') then
                next_state <= s_txt_ab_prog;
            end if;

        -------------------------------------------------------------------------------------------
        -- Transmission is in progress -> abort at nearest error!
        -------------------------------------------------------------------------------------------
        when s_txt_ab_prog =>

            -- Node became bus-off
            if (go_to_failed = '1') then
                next_state <= s_txt_failed;

            -- Parity Error occured
            elsif (txtb_parity_error_valid = '1') then
                next_state <= s_txt_parity_err;

            -- Retransmitt reached, do not try again
            elsif (txtb_hw_cmd_i.failed = '1') then
                next_state <= s_txt_failed;

            -- Transmission OK
            elsif (txtb_hw_cmd_i.valid = '1') then
                next_state <= s_txt_ok;

            -- Error or arbitration lost
            elsif (arbl_or_err = '1') then
                next_state <= s_txt_aborted;
            end if;

        -------------------------------------------------------------------------------------------
        -- Transmission from buffer failed. Retransmitt limit was reached.
        -------------------------------------------------------------------------------------------
        when s_txt_failed =>

            -- "Set_ready"
            if (tx_command_txcr_valid = '1') then
                next_state <= s_txt_ready;

            -- "Set_empty"
            elsif (tx_command_txce_valid = '1') then
                next_state <= s_txt_empty;
            end if;

        -------------------------------------------------------------------------------------------
        -- Transmission was aborted by user command
        -------------------------------------------------------------------------------------------
        when s_txt_aborted =>

            -- "Set_ready"
            if (tx_command_txcr_valid = '1') then
                next_state <= s_txt_ready;

            -- "Set_empty"
            elsif (tx_command_txce_valid = '1') then
                next_state <= s_txt_empty;
            end if;

        -------------------------------------------------------------------------------------------
        -- Transmission was succesfull
        -------------------------------------------------------------------------------------------
        when s_txt_ok =>

            -- "Set_ready"
            if (tx_command_txcr_valid = '1') then
                next_state <= s_txt_ready;

            -- "Set_empty"
            elsif (tx_command_txce_valid = '1') then
                next_state <= s_txt_empty;
            end if;

        -------------------------------------------------------------------------------------------
        -- Parity Error
        -------------------------------------------------------------------------------------------
        when s_txt_parity_err =>

            -- "Set_ready"
            if (tx_command_txcr_valid = '1') then
                next_state <= s_txt_ready;

            -- "Set_empty"
            elsif (tx_command_txce_valid = '1') then
                next_state <= s_txt_empty;
            end if;

        end case;

    end process;

    -----------------------------------------------------------------------------------------------
    -- State register clock enable
    -----------------------------------------------------------------------------------------------
    txt_fsm_ce <= '1' when (next_state /= curr_state) else
                  '0';

    -----------------------------------------------------------------------------------------------
    -- State register
    -----------------------------------------------------------------------------------------------
    p_tx_buf_fsm_state_reg : process(rst_n, clk_sys)
    begin
        if (rst_n = '0') then
            curr_state <= s_txt_empty;
        elsif (rising_edge(clk_sys)) then
            if (txt_fsm_ce = '1') then
                curr_state <= next_state;
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------------------------------
    -- Buffer FSM outputs
    -----------------------------------------------------------------------------------------------
    -- Memory protection of TXT Buffer
    txtb_user_accessible <= '0' when ((curr_state = s_txt_ready)   or
                                      (curr_state = s_txt_tx_prog) or
                                      (curr_state = s_txt_ab_prog))
                                  else
                            '1';

    -- TXT Buffer HW Command generates interrupt upon transition to Failed, Done and Aborted states!
    txtb_hw_cmd_int <= '1' when (txtb_hw_cmd_i.failed = '1') or  -- Fail transition completely
                                (txtb_hw_cmd_i.valid = '1') or   -- Finish transition OK
                                ((txtb_hw_cmd_i.arbl = '1' or
                                  txtb_hw_cmd_i.err = '1') and
                                 (curr_state = s_txt_ab_prog))   -- Not failed completely, but already in
                                                                 -- abort in progress -> No further transmissions
                                                                 -- will continue -> Indicate to user
                           else
                       '1' when (is_bus_off = '1' and next_state = s_txt_failed and
                                 transient_state = '1')
                           else
                       '0';

    -- Buffer is available for selection by TX Arbitrator only in state "Ready" Abort signal must
    -- not be active. If not considered, race condition between HW and SW commands could occur!
    txtb_available   <= '1' when ((curr_state = s_txt_ready) and (abort_applied = '0'))
                            else
                        '0';

    -- Encoding Buffer FSM to output values read from TXT Buffer status register.
    with curr_state select txtb_state <=
        TXT_RDY   when s_txt_ready,
        TXT_TRAN  when s_txt_tx_prog,
        TXT_ABTP  when s_txt_ab_prog,
        TXT_TOK   when s_txt_ok,
        TXT_ERR   when s_txt_failed,
        TXT_ABT   when s_txt_aborted,
        TXT_ETY   when s_txt_empty,
        TXT_PER   when s_txt_parity_err;

    -----------------------------------------------------------------------------------------------
    -- Unmask content of TXT Buffer RAM (make it available for CAN Core and TX Arbitrator) when it
    -- is valid for them to read from there. That is during TXT Buffer selection or during
    -- transmission. During other moments content of TXT Buffer RAM is not needed by CAN Core nor
    -- TX Arbitrator!
    -----------------------------------------------------------------------------------------------
    txtb_unmask_data_ram <= '1' when (transient_state = '1')
                                else
                            '0';

    -----------------------------------------------------------------------------------------------
    -- Assertions
    -----------------------------------------------------------------------------------------------

    -- psl default clock is rising_edge(clk_sys);

    -- HW Lock command should never arrive when the buffer is in other state
    -- than ready
    --
    -- psl txtb_lock_only_in_rdy_asrt : assert always
    --  ((txtb_hw_cmd.lock = '1' and txtb_hw_cmd_cs = '1') -> curr_state = s_txt_ready)
    --  report "TXT Buffer not READY when LOCK command occurred!";
    -----------------------------------------------------------------------------------------------
    -- HW Unlock command is valid only when Buffer is TX in Progress or Abort in
    -- progress.
    --
    -- psl txtb_unlock_only_in_tx_prog_asrt : assert always
    --  ((txtb_hw_cmd_i.valid = '1' or txtb_hw_cmd_i.err = '1' or txtb_hw_cmd_i.arbl = '1' or txtb_hw_cmd_i.failed = '1') ->
    --   (curr_state = s_txt_tx_prog or curr_state = s_txt_ab_prog or curr_state = s_txt_parity_err))
    --  report "TXT Buffer not TX in progress, Abort in progress or Parity Error when unlock received!";
    -----------------------------------------------------------------------------------------------
    -- HW Lock command should never occur when there was abort in previous cycle!
    --
    -- psl txtb_no_lock_after_abort : assert never
    --  {abort_applied = '1';txtb_hw_cmd.lock = '1' and txtb_hw_cmd_cs = '1'}
    --  report "LOCK command after ABORT was applied!";
    -----------------------------------------------------------------------------------------------

end architecture;