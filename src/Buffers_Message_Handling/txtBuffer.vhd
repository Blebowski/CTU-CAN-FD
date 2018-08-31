--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018 Ondrej Ille <ondrej.ille@gmail.com>
-- 
-- Project advisors and co-authors: 
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 	Martin Jerabek <jerabma7@fel.cvut.cz>
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
--  Transmit message buffer. Buffer is accessed via "tran_data" and "tran_addr"
--  signals from user registers. Buffer contains simple FSM for manipulation
--  from HW (HW Commands) as well as SW (SW Commands). Buffer is split into
--  "data" part which is implemented to be inferred in dual port RAM block,
--  and "metadata" part which is available in paralell on the output. "data"
--  part is addressed by second port from Protocol control.
--------------------------------------------------------------------------------
-- Revision History:
--
--    July 2015   Created file
--    30.11.2017  Changed the buffer implementation from parallel into 32*20 
--                buffer of data. Reading so far left parallel. User is directly
--                accessing the buffer and storing the data to it.
--    04.12.2017  Buffer split to "Frame metadata" (txt_buffer_info) and "Data" 
--                (txt_buffer_data). Frame metadata consists of first 4 words 
--                (Frame format, Timestamps and Identifier). Frame metadata are
--                available combinationally at all times. Frame data are accessed
--                directly from CAN Core by new pointer "txt_data_addr". 
--                txt_buffer_data is synthesized as RAM memory and significant
--                reource reduction was achieved.
--     15.2.2018  Implemented TXT Buffer state machine. Replaced "empty", "ack"
--                and "allow" signals with HW commands from Protocol Control and
--                SW commands from User registers. Hardware commands have always
--                higher priority.
--     24.3.2018  1. Changed TXT Buffer implementation to have both, metadata
--                    and data in CAN frame.
--                2. Added memory protection on the TXT Buffer. It can NOT be
--                    written when in "ready", "tx in progress" or "abort in
--                    progress" states.
--     06.4.2018  Changed output from side of CAN Core to synchronous. Async.
--                output did not allow inferrence of RAM in Altera FPGA.
--     30.8.2018  Added "txt_hw_cmd_int" output for Interrupt Manager. TXTB HW
--                command Interrupt generated upon move to Done, Failed or
--                Aborted states.
--------------------------------------------------------------------------------

Library ieee;
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.ALL;
use work.CANconstants.all;

entity txtBuffer is
    generic(
        constant buf_count            :     natural range 1 to 8;
        constant ID                   :     natural := 1
    );
  PORT(
        ------------------------------------------------------------------------
        -- Clock and reset
        ------------------------------------------------------------------------
        signal clk_sys                :in   std_logic;
        signal res_n                  :in   std_logic;

        ------------------------------------------------------------------------
        -- Driving Registers Interface
        ------------------------------------------------------------------------

        -- Data and address for SW access into the RAM of TXT Buffer
        signal tran_data              :in   std_logic_vector(31 downto 0);
        signal tran_addr              :in   std_logic_vector(4 downto 0);
        signal tran_cs                :in   std_logic;

        -- SW commands from user registers
        signal txt_sw_cmd             :in   txt_sw_cmd_type;
        signal txt_sw_buf_cmd_index   :in   std_logic_vector(
                                                buf_count - 1 downto 0);

        ------------------------------------------------------------------------   
        -- Status signals
        ------------------------------------------------------------------------
        signal txtb_state             :out  txt_fsm_type;

        ------------------------------------------------------------------------
        -- Interrupt Manager
        ------------------------------------------------------------------------
        signal txt_hw_cmd_int         :out  std_logic;

        ------------------------------------------------------------------------
        -- CAN Core and TX Arbiter Interface
        ------------------------------------------------------------------------

        -- Commands from the CAN Core for manipulation of the CAN 
        signal txt_hw_cmd             :in   txt_hw_cmd_type;  
        signal txt_hw_cmd_buf_index   :in   natural range 0 to buf_count - 1;

        -- Buffer output and pointer to the RAM memory
        signal txt_word               :out  std_logic_vector(31 downto 0);
        signal txt_addr               :in   natural range 0 to 19;

        signal bus_off_start          :in   std_logic;

        -- Signals to the TX Arbitrator that it can be selected for transmission
        -- (used as input to priority decoder)
        signal txt_buf_ready          :out  std_logic
    );
             
end entity;


architecture rtl of txtBuffer is

    ----------------------------------------------------------------------------
    -- Internal registers
    ----------------------------------------------------------------------------
    type frame_memory is array(0 to 19) of std_logic_vector(31 downto 0);

    ----------------------------------------------------------------------------
    --Signal aliases
    ----------------------------------------------------------------------------

    -- Time transcieve buffer - Data memory
    signal txt_buffer_mem         : frame_memory;

    -- FSM state of the buffer
    signal buf_fsm                : txt_fsm_type;

    -- TXT Buffer memory protection
    signal txtb_user_accessible   : boolean;

    -- Internal buffer selects for commands. Commands are shared across the
    -- buffers so we need unique identifier
    signal hw_cbs                 : std_logic;
    signal sw_cbs                 : std_logic;

begin
    
    -- Buffer is ready for selection by TX Arbitrator only in state "Ready"
    -- Abort signal must not be active. If not considered,
    -- race conditions between HW and SW commands could occur.
    txt_buf_ready       <= '1' when ((buf_fsm = txt_ready) and
                                     (txt_sw_cmd.set_abt = '0'))
                               else
                           '0';
    
    -- Command buffer select signals
    hw_cbs <= '1' when txt_hw_cmd_buf_index = ID
                  else
              '0';
  
    sw_cbs <= '1' when txt_sw_buf_cmd_index(ID) = '1' 
                  else
              '0';

    -- TXT Buffer HW Command generates interrupt upon transition to
    -- Failed, Done and Aborted states!
    txt_hw_cmd_int <= '1' when (hw_cbs = '1') and ((txt_hw_cmd.failed = '1') or
                                                   (txt_hw_cmd.valid = '1') or
                                                   ((txt_hw_cmd.unlock = '1') and
                                                    (buf_fsm = txt_ab_prog)) or
                                                   ((txt_sw_cmd.set_abt = '1') and
                                                    (sw_cbs = '1') and
                                                    (buf_fsm = txt_ready)))
                          else
                       '0';
    
    -- Connect internal buffer state to output
    txtb_state <= buf_fsm;
    
    -- Memory protection of TXT Buffer
    txtb_user_accessible <= false when ((buf_fsm = txt_ready)   or
                                        (buf_fsm = txt_tx_prog) or
                                        (buf_fsm = txt_ab_prog))
                                  else
                            true;
    
    ----------------------------------------------------------------------------
    -- Buffer access process from SW
    ----------------------------------------------------------------------------
    tx_buf_access_proc : process(res_n, clk_sys)
    begin
        if (res_n = ACT_RESET) then

            -- pragma translate_off
            txt_buffer_mem <= (OTHERS => (OTHERS => '0'));
            txt_word       <= (OTHERS => '0');      
            -- pragma translate_on
           
        elsif (rising_edge(clk_sys)) then

            --Store the data into the Buffer during the access
            if (tran_cs = '1' and txtb_user_accessible) then
                txt_buffer_mem(to_integer(unsigned(tran_addr))) <= tran_data;
            end if;

            -- Output data are given by the address from Core
            txt_word            <= txt_buffer_mem(txt_addr);

        end if;
    end process;
    
    
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
    -- Monitoring invalid command combinations!
    ----------------------------------------------------------------------------
    lock_check_proc : process(clk_sys)
    begin
        if (rising_edge(clk_sys)) then
            if (txt_hw_cmd.lock       = '1'       and
                buf_fsm              /= txt_ready and
                txt_hw_cmd_buf_index  = ID) 
            then
                report "Buffer not READY and LOCK occurred on TXT Buffer: " &
                        integer'image(ID) severity error;
            end if;
        end if;
    end process;


    unlock_check_proc : process(clk_sys)
    begin
        if (rising_edge(clk_sys)) then
            if (txt_hw_cmd.unlock     = '1'         and
                buf_fsm              /= txt_tx_prog and
                buf_fsm              /= txt_ab_prog and
                txt_hw_cmd_buf_index  = ID)
            then
                report "Buffer not 'TX_prog' or 'AB_prog' and UNLOCK" &
                       " occurred on Buffer: " & integer'image(ID) severity error;
            end if;
        end if;
    end process;

end architecture;
