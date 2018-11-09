--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018
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
--  Transmitt Message Buffer FSM.
--------------------------------------------------------------------------------
-- Revision History:
--
--    07.11.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use work.CANconstants.all;
use work.CANcomponents.all;
use work.CAN_FD_register_map.all;

entity txtBuffer_fsm is
    generic(
        constant ID                   :     natural
    );
    port(
        ------------------------------------------------------------------------
        -- Clock and reset
        ------------------------------------------------------------------------
        signal clk_sys                :in   std_logic;
        signal res_n                  :in   std_logic;

        ------------------------------------------------------------------------
        -- SW Interface (Memory registers)
        ------------------------------------------------------------------------
        signal txt_sw_cmd             :in   txt_sw_cmd_type;
        signal sw_cbs                 :in   std_logic;

        ------------------------------------------------------------------------   
        -- HW Interface (CAN Core
        ------------------------------------------------------------------------
        signal txt_hw_cmd             :in   txt_hw_cmd_type;  
        signal hw_cbs                 :in   std_logic;
    
        ------------------------------------------------------------------------
        -- Other control signals
        ------------------------------------------------------------------------
        signal bus_off_start          :in   std_logic;

        ------------------------------------------------------------------------
        -- Status signals
        ------------------------------------------------------------------------

        -- Buffer accessible from SW
        signal txtb_user_accessible   :out  std_logic;

        -- HW Command applied on TXT Buffer -> Interrupt!
        signal txtb_hw_cmd_int        :out  std_logic;

        -- Buffer status (FSM state) encoded for reading by SW from registers!
        signal txtb_state             :out  std_logic_vector(3 downto 0);

        -- TXT Buffer is ready to be locked by CAN Core for transmission
        signal txt_buf_ready          :out  std_logic
    );
             
end entity;


architecture rtl of txtBuffer_fsm is

    -- FSM state of the buffer
    signal buf_fsm                : txt_fsm_type;

begin

    ----------------------------------------------------------------------------
    -- Buffer FSM process
    ----------------------------------------------------------------------------
    tx_buf_fsm_proc : process(res_n, clk_sys)
    begin
        if (res_n = ACT_RESET) then
            buf_fsm         <= txt_empty;

        elsif (rising_edge(clk_sys)) then
        
            buf_fsm           <= buf_fsm;

            case buf_fsm is

            --------------------------------------------------------------------
            -- Buffer is empty
            --------------------------------------------------------------------
            when txt_empty =>

                -- "Set_ready"
                if (txt_sw_cmd.set_rdy = '1' and sw_cbs = '1') then
                    buf_fsm       <= txt_ready;
                end if;


            --------------------------------------------------------------------
            -- Buffer is ready for transmission
            --------------------------------------------------------------------
            when txt_ready =>
              
                -- Locking for transmission
                if (txt_hw_cmd.lock = '1' and hw_cbs = '1') then

                    -- Simultaneous "lock" and abort -> transmit, but
                    -- with abort pending
                    if (txt_sw_cmd.set_abt = '1' and sw_cbs = '1') then
                        buf_fsm     <= txt_ab_prog;
                    else
                        buf_fsm     <= txt_tx_prog;
                    end if;

                -- Abort the ready buffer
                elsif (txt_sw_cmd.set_abt = '1' and sw_cbs = '1') then
                    buf_fsm       <= txt_aborted;
                else
                    buf_fsm       <= buf_fsm;
                end if;


            --------------------------------------------------------------------
            -- Transmission from buffer is in progress
            --------------------------------------------------------------------
            when txt_tx_prog =>
              
                -- Unlock the buffer
                if (txt_hw_cmd.unlock = '1' and hw_cbs = '1') then

                    -- Retransmitt reached, transmitt OK, or try again...
                    if (txt_hw_cmd.failed         = '1') then
                        buf_fsm     <= txt_error;
                    elsif (txt_hw_cmd.valid       = '1') then
                        buf_fsm     <= txt_ok;
                    elsif (txt_hw_cmd.err         = '1' or 
                           txt_hw_cmd.arbl        = '1') then
                        buf_fsm     <= txt_ready;
                    else
                        buf_fsm     <= buf_fsm;
                    end if;

                -- Request abort during transmission
                elsif (txt_sw_cmd.set_abt = '1' and sw_cbs = '1') then 
                    buf_fsm         <= txt_ab_prog;
                else
                    buf_fsm         <= buf_fsm;  
                end if;


            --------------------------------------------------------------------
            -- Transmission is in progress -> abort at nearest error!
            --------------------------------------------------------------------
            when txt_ab_prog =>
              
                -- Unlock the buffer
                if (txt_hw_cmd.unlock = '1' and hw_cbs = '1') then

                    -- Retransmitt reached, transmitt OK, or try again... 
                    if (txt_hw_cmd.failed         = '1') then
                        buf_fsm     <= txt_error;
                    elsif (txt_hw_cmd.valid       = '1') then
                        buf_fsm     <= txt_ok;
                    elsif (txt_hw_cmd.err         = '1' or 
                           txt_hw_cmd.arbl        = '1') then
                        buf_fsm     <= txt_aborted;
                    else
                        buf_fsm     <= buf_fsm;
                    end if;

                else
                    buf_fsm       <= buf_fsm;
                end if;


            --------------------------------------------------------------------
            -- Transmission from buffer failed. Retransmitt limit was reached.
            --------------------------------------------------------------------
            when txt_error =>

                -- "Set_ready"
                if (txt_sw_cmd.set_rdy = '1' and sw_cbs = '1') then
                    buf_fsm       <= txt_ready;
                end if;

                -- "Set_empty"
                if (txt_sw_cmd.set_ety = '1' and sw_cbs = '1') then
                    buf_fsm       <= txt_empty;
                end if;


            --------------------------------------------------------------------
            -- Transmission was aborted by user command
            --------------------------------------------------------------------
            when txt_aborted =>
              
                -- "Set_ready"
                if (txt_sw_cmd.set_rdy = '1' and sw_cbs = '1') then
                    buf_fsm       <= txt_ready;
                end if;

                -- "Set_empty"
                if (txt_sw_cmd.set_ety = '1' and sw_cbs = '1') then
                    buf_fsm       <= txt_empty;
                end if;


            --------------------------------------------------------------------
            -- Transmission was succesfull
            --------------------------------------------------------------------
            when txt_ok =>
              
                -- "Set_ready"
                if (txt_sw_cmd.set_rdy = '1' and sw_cbs = '1') then
                    buf_fsm       <= txt_ready;
                end if;

                -- "Set_empty"
                if (txt_sw_cmd.set_ety = '1' and sw_cbs = '1') then
                    buf_fsm       <= txt_empty;
                end if;
              
            end case;

            --------------------------------------------------------------------
            -- If Core goes to bus-off, TXT Buffer goes to failed, regardless of
            -- any other SW or HW commands
            --------------------------------------------------------------------
            if (bus_off_start = '1') then
                buf_fsm       <= txt_error;
            end if;

        end if;
    end process;


    ----------------------------------------------------------------------------
    -- Buffer FSM outputs
    ----------------------------------------------------------------------------
    -- Memory protection of TXT Buffer
    txtb_user_accessible <= '0' when ((buf_fsm = txt_ready)   or
                                      (buf_fsm = txt_tx_prog) or
                                      (buf_fsm = txt_ab_prog))
                                  else
                            '1';

    -- TXT Buffer HW Command generates interrupt upon transition to
    -- Failed, Done and Aborted states!
    txtb_hw_cmd_int <= '1' when (hw_cbs = '1') and ((txt_hw_cmd.failed = '1') or
                                                   (txt_hw_cmd.valid = '1') or
                                                   ((txt_hw_cmd.unlock = '1') and
                                                    (buf_fsm = txt_ab_prog)) or
                                                   ((txt_sw_cmd.set_abt = '1') and
                                                    (sw_cbs = '1') and
                                                    (buf_fsm = txt_ready)))
                           else
                        '0';

    -- Buffer is ready for selection by TX Arbitrator only in state "Ready"
    -- Abort signal must not be active. If not considered,
    -- race conditions between HW and SW commands could occur.
    txt_buf_ready       <= '1' when ((buf_fsm = txt_ready) and
                                     (txt_sw_cmd.set_abt = '0'))
                               else
                           '0';

    -- Encoding Buffer FSM to output values read from TXT Buffer status register.
    with buf_fsm select txtb_state <= 
        TXT_RDY   when txt_ready,
        TXT_TRAN  when txt_tx_prog,
        TXT_ABTP  when txt_ab_prog,
        TXT_TOK   when txt_ok,
        TXT_ERR   when txt_error,
        TXT_ABT   when txt_aborted,
        TXT_ETY   when txt_empty;


    ----------------------------------------------------------------------------
    -- Monitoring invalid command combinations!
    ----------------------------------------------------------------------------
    lock_check_proc : process(clk_sys)
    begin
        if (rising_edge(clk_sys)) then
            if (txt_hw_cmd.lock       = '1'       and
                buf_fsm              /= txt_ready and
                hw_cbs                = '1') 
            then
                report "Buffer not READY and LOCK occurred on TXT Buffer: " &
                        integer'image(ID) & " Buffer state: " & 
                        txt_fsm_type'image(buf_fsm) severity error;
            end if;
        end if;
    end process;


    unlock_check_proc : process(clk_sys)
    begin
        if (rising_edge(clk_sys)) then
            if (txt_hw_cmd.unlock     = '1'         and
                buf_fsm              /= txt_tx_prog and
                buf_fsm              /= txt_ab_prog and
                hw_cbs                = '1')
            then
                report "Buffer not 'TX_prog' or 'AB_prog' and UNLOCK" &
                       " occurred on Buffer: " & integer'image(ID) severity error;
            end if;
        end if;
    end process;

end architecture;
