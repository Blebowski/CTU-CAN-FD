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
use ctu_can_fd_rtl.id_transfer_pkg.all;
use ctu_can_fd_rtl.can_constants_pkg.all;
use ctu_can_fd_rtl.can_components_pkg.all;
use ctu_can_fd_rtl.can_types_pkg.all;
use ctu_can_fd_rtl.common_blocks_pkg.all;
use ctu_can_fd_rtl.drv_stat_pkg.all;
use ctu_can_fd_rtl.unary_ops_pkg.all;

use ctu_can_fd_rtl.CAN_FD_register_map.all;
use ctu_can_fd_rtl.CAN_FD_frame_format.all;

entity txt_buffer_fsm is
    generic(
        -- TXT Buffer ID
        G_ID                   :     natural
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and Asynchronous reset
        ------------------------------------------------------------------------
        -- System clock
        clk_sys                 :in   std_logic;
        
        -- Asynchronous reset
        res_n                   :in   std_logic;

        ------------------------------------------------------------------------
        -- Memory registers interface
        ------------------------------------------------------------------------
        -- SW commands
        txtb_sw_cmd             :in   t_txtb_sw_cmd;
        
        -- SW buffer select
        sw_cbs                  :in   std_logic;
        
        -- TXT Buffer bus-off behavior
        txt_buf_failed_bof      :in   std_logic;

        -- Restricted operation mode
        drv_rom_ena            :in   std_logic;

        -- Bus monitoring mode
        drv_bus_mon_ena        :in   std_logic;

        ------------------------------------------------------------------------   
        -- CAN Core interface
        ------------------------------------------------------------------------
        -- HW Commands
        txtb_hw_cmd             :in   t_txtb_hw_cmd;  
        
        -- HW Buffer select
        hw_cbs                  :in   std_logic;
    
        -- Unit is Bus off
        is_bus_off              :in   std_logic;

        ------------------------------------------------------------------------
        -- Status signals
        ------------------------------------------------------------------------
        -- Buffer accessible from SW
        txtb_user_accessible    :out  std_logic;

        -- HW Command applied on TXT Buffer.
        txtb_hw_cmd_int         :out  std_logic;

        -- Buffer status (FSM state) encoded for reading by SW.
        txtb_state              :out  std_logic_vector(3 downto 0);

        -- TXT Buffer is available to be locked by CAN Core for transmission
        txtb_available          :out  std_logic;
                
        -- UnMask content of TXT Buffer RAM
        txtb_unmask_data_ram    :out  std_logic
    );             
end entity;

architecture rtl of txt_buffer_fsm is

    -- FSM signals
    signal next_state          : t_txt_buf_state;
    signal curr_state          : t_txt_buf_state;

    -- Abort command applied
    signal abort_applied       : std_logic;
    
    -- TXT Buffer clock enable
    signal txt_fsm_ce          : std_logic;

    -- Forced transition to failed state
    signal go_to_failed        : std_logic;
    signal transient_state     : std_logic;

begin

    abort_applied <= '1' when (txtb_sw_cmd.set_abt = '1' and sw_cbs = '1') else
                     '0';

    transient_state <= '1' when ((curr_state = s_txt_ab_prog) or
                                 (curr_state = s_txt_tx_prog) or
                                 (curr_state = s_txt_ready))
                          else
                      '0';

    go_to_failed <= '1' when (transient_state = '1') and
                             ((is_bus_off = '1' and txt_buf_failed_bof =
                               TXTBUF_FAILED_BUS_OFF_ENABLED) or
                             drv_bus_mon_ena = BMM_ENABLED or
                             drv_rom_ena = ROM_ENABLED)
                        else
                    '0';

    ----------------------------------------------------------------------------
    -- Next state process
    ----------------------------------------------------------------------------
    tx_buf_fsm_next_state_proc : process(curr_state, txtb_sw_cmd, sw_cbs, 
        txtb_hw_cmd, hw_cbs, is_bus_off, abort_applied, go_to_failed)
    begin
        next_state <= curr_state;

        case curr_state is

        --------------------------------------------------------------------
        -- Buffer is empty
        --------------------------------------------------------------------
        when s_txt_empty =>

            -- "Set_ready"
            if (txtb_sw_cmd.set_rdy = '1' and sw_cbs = '1') then
                next_state       <= s_txt_ready;
            end if;

        --------------------------------------------------------------------
        -- Buffer is ready for transmission
        --------------------------------------------------------------------
        when s_txt_ready =>
          
            -- Locking for transmission
            if (txtb_hw_cmd.lock = '1' and hw_cbs = '1') then

                -- Simultaneous "lock" and abort -> transmit, but
                -- with abort pending
                if (abort_applied = '1') then
                    next_state     <= s_txt_ab_prog;
                else
                    next_state     <= s_txt_tx_prog;
                end if;

            -- Abort the ready buffer
            elsif (abort_applied = '1') then
                next_state       <= s_txt_aborted;
            end if;

        --------------------------------------------------------------------
        -- Transmission from buffer is in progress
        --------------------------------------------------------------------
        when s_txt_tx_prog =>
          
            -- Unlock the buffer
            if (txtb_hw_cmd.unlock = '1' and hw_cbs = '1') then

                -- Retransmitt reached, transmitt OK, or try again...
                if (txtb_hw_cmd.failed         = '1') then
                    next_state     <= s_txt_failed;
                elsif (txtb_hw_cmd.valid       = '1') then
                    next_state     <= s_txt_ok;
                elsif (txtb_hw_cmd.err         = '1' or 
                       txtb_hw_cmd.arbl        = '1') then
                    if (abort_applied = '1') then
                        next_state     <= s_txt_aborted;
                    else
                        next_state     <= s_txt_ready;
                    end if;
                end if;

            -- Request abort during transmission
            elsif (abort_applied = '1') then 
                next_state         <= s_txt_ab_prog;
            end if;

        --------------------------------------------------------------------
        -- Transmission is in progress -> abort at nearest error!
        --------------------------------------------------------------------
        when s_txt_ab_prog =>
          
            -- Unlock the buffer
            if (txtb_hw_cmd.unlock = '1' and hw_cbs = '1') then

                -- Retransmitt reached, transmitt OK, or try again... 
                if (txtb_hw_cmd.failed         = '1') then
                    next_state     <= s_txt_failed;
                elsif (txtb_hw_cmd.valid       = '1') then
                    next_state     <= s_txt_ok;
                elsif (txtb_hw_cmd.err         = '1' or 
                       txtb_hw_cmd.arbl        = '1') then
                    next_state     <= s_txt_aborted;
                end if;
            end if;

        --------------------------------------------------------------------
        -- Transmission from buffer failed. Retransmitt limit was reached.
        --------------------------------------------------------------------
        when s_txt_failed =>

            -- "Set_ready"
            if (txtb_sw_cmd.set_rdy = '1' and sw_cbs = '1') then
                next_state       <= s_txt_ready;
            end if;

            -- "Set_empty"
            if (txtb_sw_cmd.set_ety = '1' and sw_cbs = '1') then
                next_state       <= s_txt_empty;
            end if;

        --------------------------------------------------------------------
        -- Transmission was aborted by user command
        --------------------------------------------------------------------
        when s_txt_aborted =>
          
            -- "Set_ready"
            if (txtb_sw_cmd.set_rdy = '1' and sw_cbs = '1') then
                next_state       <= s_txt_ready;
            end if;

            -- "Set_empty"
            if (txtb_sw_cmd.set_ety = '1' and sw_cbs = '1') then
                next_state       <= s_txt_empty;
            end if;

        --------------------------------------------------------------------
        -- Transmission was succesfull
        --------------------------------------------------------------------
        when s_txt_ok =>
          
            -- "Set_ready"
            if (txtb_sw_cmd.set_rdy = '1' and sw_cbs = '1') then
                next_state       <= s_txt_ready;
            end if;

            -- "Set_empty"
            if (txtb_sw_cmd.set_ety = '1' and sw_cbs = '1') then
                next_state       <= s_txt_empty;
            end if;

        end case;

        --------------------------------------------------------------------
        -- If Core is bus-off, TXT Buffer goes to failed from any transient
        -- state.
        --------------------------------------------------------------------
        if (go_to_failed = '1') then
            next_state <= s_txt_failed;
        end if;

    end process;

    ----------------------------------------------------------------------------
    -- State register clock enable
    ----------------------------------------------------------------------------
    txt_fsm_ce <= '1' when (next_state /= curr_state) else
                  '0';

    ----------------------------------------------------------------------------
    -- State register
    ----------------------------------------------------------------------------
    tx_buf_fsm_state_reg_proc : process(res_n, clk_sys)
    begin
        if (res_n = '0') then
            curr_state <= s_txt_empty;
        elsif (rising_edge(clk_sys)) then
            if (txt_fsm_ce = '1') then
                curr_state <= next_state;
            end if; 
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Buffer FSM outputs
    ----------------------------------------------------------------------------
    -- Memory protection of TXT Buffer
    txtb_user_accessible <= '0' when ((curr_state = s_txt_ready)   or
                                      (curr_state = s_txt_tx_prog) or
                                      (curr_state = s_txt_ab_prog))
                                  else
                            '1';

    -- TXT Buffer HW Command generates interrupt upon transition to
    -- Failed, Done and Aborted states!
    txtb_hw_cmd_int <= '1' when (hw_cbs = '1') and ((txtb_hw_cmd.failed = '1') or
                                                   (txtb_hw_cmd.valid = '1') or
                                                   ((txtb_hw_cmd.unlock = '1') and
                                                    (curr_state = s_txt_ab_prog)) or
                                                   ((txtb_sw_cmd.set_abt = '1') and
                                                    (sw_cbs = '1') and
                                                    (curr_state = s_txt_ready)))
                           else
                       '1' when (is_bus_off = '1' and next_state = s_txt_failed and
                                transient_state = '1')
                           else  
                       '0';

    -- Buffer is available for selection by TX Arbitrator only in state "Ready"
    -- Abort signal must not be active. If not considered, race condition
    -- between HW and SW commands could occur!
    txtb_available   <= '1' when ((curr_state = s_txt_ready) and
                                  (abort_applied = '0'))
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
        TXT_ETY   when s_txt_empty;
        
    ---------------------------------------------------------------------------
    -- Unmask content of TXT Buffer RAM (make it available for CAN Core and
    -- TX Arbitrator) when it is valid for them to read from there. That is
    -- during TXT Buffer selection or during transmission. During other moments
    -- content of TXT Buffer RAM is not needed by CAN Core nor TX Arbitrator!
    ---------------------------------------------------------------------------
    txtb_unmask_data_ram <= '1' when (transient_state = '1')
                                else
                            '0';


    -- <RELEASE_OFF>
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Functional coverage
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- psl default clock is rising_edge(clk_sys);

    -- Each FSM state
    -- psl txtb_fsm_empty_cov : cover {curr_state = s_txt_empty};
    -- psl txtb_fsm_ready_cov : cover {curr_state = s_txt_ready};
    -- psl txtb_fsm_tx_prog_cov : cover {curr_state = s_txt_tx_prog};
    -- psl txtb_fsm_ab_prog_cov : cover {curr_state = s_txt_ab_prog};
    -- psl txtb_fsm_error_cov : cover {curr_state = s_txt_failed};
    -- psl txtb_fsm_aborted_cov : cover {curr_state = s_txt_aborted};
    -- psl txtb_fsm_tx_ok_cov : cover {curr_state = s_txt_ok};
    
    -- Simultaneous HW and SW Commands
    --
    -- psl txtb_hw_sw_cmd_txt_ready_hazard_cov : cover
    --  {txtb_hw_cmd.lock = '1' and hw_cbs = '1' and abort_applied = '1' and
    --   curr_state = s_txt_ready};
    --
    -- psl txtb_hw_sw_cmd_txt_tx_prog_hazard_cov : cover
    --  {txtb_hw_cmd.unlock = '1' and hw_cbs = '1' and abort_applied = '1' and
    --   curr_state = s_txt_tx_prog};
    
    ----------------------------------------------------------------------------
    -- Assertions
    ----------------------------------------------------------------------------
    -- HW Lock command should never arrive when the buffer is in other state
    -- than ready
    --
    -- psl txtb_lock_only_in_rdy_asrt : assert always
    --  ((txtb_hw_cmd.lock = '1' and hw_cbs = '1') -> curr_state = s_txt_ready)
    --  report "TXT Buffer not READY when LOCK command occurred!";
    ----------------------------------------------------------------------------
    -- HW Unlock command is valid only when Buffer is TX in Progress or Abort in
    -- progress.
    --
    -- psl txtb_unlock_only_in_tx_prog_asrt : assert always
    --  ((txtb_hw_cmd.unlock = '1' and hw_cbs = '1') ->
    --   (curr_state = s_txt_tx_prog or curr_state = s_txt_ab_prog))
    --  report "TXT Buffer " & integer'image(G_ID) &
    --  " not TX in progress or Abort in progress";
    ----------------------------------------------------------------------------
    -- HW Lock command should never occur when there was abort in previous cycle!
    --
    -- psl txtb_no_lock_after_abort : assert never
    --  {abort_applied = '1';txtb_hw_cmd.lock = '1' and hw_cbs = '1'}
    --  report "LOCK command after ABORT was applied!";
    ----------------------------------------------------------------------------

    -- <RELEASE_ON>
end architecture;