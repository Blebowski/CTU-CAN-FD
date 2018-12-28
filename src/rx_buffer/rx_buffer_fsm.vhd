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
--  Receive Buffer FSM. Reacts on commands from CAN Core and controls storing
--  of CAN frame continusly to RX Buffer RAM.
--------------------------------------------------------------------------------
-- Revision History:
--    14.12.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.ALL;
use work.can_constants.all;
use work.CAN_FD_frame_format.all;
use work.CAN_FD_register_map.all;
use work.can_components.all;

entity rx_buffer_fsm is
    port(
        ------------------------------------------------------------------------
        -- Clocks and reset 
        ------------------------------------------------------------------------
        signal clk_sys              :in     std_logic; --System clock
        signal res_n                :in     std_logic; --Async. reset

        ------------------------------------------------------------------------
        -- Control signals from CAN Core which control storing of CAN Frame.
        ------------------------------------------------------------------------

        -- After control field of CAN frame, metadata are valid and can be stored.
        -- This command starts the RX FSM for storing.
        signal store_metadata       :in     std_logic;
       
        -- Signal that one word of data can be stored (TX_DATA_X_W). This signal
        -- is active when 4 bytes were received or data reception has finished 
        -- on 4 byte unaligned number of frames! (Thus allowing to store also
        -- data which are not 4 byte aligned!
        signal store_data           :in     std_logic;

        -- When frame reception is succesfull, in the end of EOF field, CAN Core
        -- gives acknowledge by this signal. This means that frame was received
        -- OK, and Error frame can no longer occur in it.
        signal rec_message_valid    :in     std_logic;
        
        -- If error frame occurred, CAN Core activates this signal.
        -- "write_pointer_raw" will be restarted to last committed value in
        -- "write_pointer".
        signal rec_abort            :in     std_logic;

        -- Signals start of frame. If timestamp on RX frame should be captured
        -- in the beginning of the frame, this pulse captures the timestamp!
        signal sof_pulse            :in     std_logic;

        ------------------------------------
        -- User registers interface
        ------------------------------------
        
        -- Driving bus from registers
        signal drv_bus              :in     std_logic_vector(1023 downto 0);

        ------------------------------------
        -- FSM outputs
        ------------------------------------
        
        -- Intent to write to RX Buffer RAM
        signal write_raw_intent     :out    std_logic;

        -- Extra Timestamp should be written to RX Buffer RAM memory
        signal write_extra_ts       :out    std_logic;

        -- Storing of extra Timestamp from end of frame has ended. In last
        -- state of storing extra timestamp, used for commiting the frame.
        -- Normally the frame is commited by rec_mesage_valid, but in case
        -- of extra timestamp we must wait till also extra timestamp is
        -- stored.
        signal store_extra_ts_end   :out    std_logic;

        -- One-hot coded data selector for selection of memory word to be stored
        -- in RX Buffer RAM
        signal data_selector        :out    std_logic_vector(6 downto 0);

        -- Extra write pointer should be loaded from write pointer
        signal store_extra_wr_ptr   :out    std_logic;

        -- Increment extra write pointer by 1
        signal inc_extra_wr_ptr     :out    std_logic;

        -- Reset internal overrun flag
        signal reset_overrun_flag   :out    std_logic

    );
end entity;


architecture rtl of rx_buffer_fsm is

    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Driving bus signal aliases
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    -- Erase command from driving registers. Resets FIFO pointers!
    signal drv_erase_rx             :       std_logic;

    -- Receive Timestamp options
    signal drv_rtsopt               :       std_logic;

    -- FSM state register
    signal rx_fsm                   :       rx_buf_fsm_type;

begin

    ----------------------------------------------------------------------------
    -- Driving bus aliases
    ----------------------------------------------------------------------------
    drv_erase_rx          <= drv_bus(DRV_ERASE_RX_INDEX);
    drv_rtsopt            <= drv_bus(DRV_RTSOPT_INDEX);    


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- FSM for controlling storing of the frame to RX Buffer FIFO.
    -- Calculation of next state based on Commands from Protocol Control.
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    rx_buf_fsm : process(clk_sys, res_n, drv_erase_rx)
    begin     
        if (res_n = ACT_RESET) or (drv_erase_rx = '1') then
            rx_fsm              <= rxb_idle;
          
        elsif rising_edge(clk_sys) then
  
            case rx_fsm is

            --------------------------------------------------------------------
            -- Idle, waiting for "store_metada" to start storing first 4 words.
            --------------------------------------------------------------------
            when rxb_idle =>
                if (store_metadata = '1') then
                    rx_fsm      <= rxb_store_frame_format;
                end if;


            --------------------------------------------------------------------
            -- Storing FRAME_FORM_W. Proceed execpt if error ocurrs.
            --------------------------------------------------------------------
            when rxb_store_frame_format =>
                if (rec_abort = '1') then
                    rx_fsm      <= rxb_idle;
                else
                    rx_fsm      <= rxb_store_identifier;
                end if;


            --------------------------------------------------------------------
            -- Storing IDENTIFIER_W.
            -- Move to storing timestamp words. Note that if SW configured
            -- timestamp from end of the frame, we dont have it yet! We store
            -- invalid timestamp and later (if the frame is received OK), we
            -- repeat the writes with timestamp captured at the end of frame!
            --------------------------------------------------------------------
            when rxb_store_identifier =>
                if (rec_abort = '1') then
                    rx_fsm      <= rxb_idle;
                else
                    rx_fsm      <= rxb_store_beg_ts_low;
                end if;


            --------------------------------------------------------------------
            -- Store TIMESTAMP_L_W from beginning of frame.
            --------------------------------------------------------------------
            when rxb_store_beg_ts_low =>
                if (rec_abort = '1') then
                    rx_fsm      <= rxb_idle;
                else
                    rx_fsm      <= rxb_store_beg_ts_high;
                end if;


            --------------------------------------------------------------------
            -- Store first TIMESTAMP_U_W from beginning of frame.
            --------------------------------------------------------------------
            when rxb_store_beg_ts_high =>
                if (rec_abort = '1') then
                    rx_fsm      <= rxb_idle;
                else                
                    rx_fsm      <= rxb_store_data;
                end if;

            --------------------------------------------------------------------
            -- Store DATA_W. If error ocurrs, abort the storing. If storing is
            -- finished, go to idle or again timestamp storing depending on the
            -- timestamp option configuration. Note that now timestamp storing
            -- is realized via different states!
             --------------------------------------------------------------------
            when rxb_store_data =>
                if (rec_abort = '1') then 
                    rx_fsm          <= rxb_idle;
                elsif (rec_message_valid = '1') then
                    if (drv_rtsopt = RTS_END) then
                        rx_fsm      <= rxb_store_end_ts_low;
                    else
                        rx_fsm      <= rxb_idle; 
                    end if;
                end if;


            --------------------------------------------------------------------
            -- Store TIMESTAMP_L_W from end of frame.
            --------------------------------------------------------------------
            when rxb_store_end_ts_low =>
                rx_fsm      <= rxb_store_end_ts_high;


            --------------------------------------------------------------------
            -- Store first TIMESTAMP_U_W from end of frame.
            --------------------------------------------------------------------
            when rxb_store_end_ts_high =>
                rx_fsm      <= rxb_idle;

            when others =>
                report "Invalid RX Buffer state" severity error;
            end case;
        end if;
    end process;


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- FSM outputs
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    ----------------------------------------------------------------------------
    -- Signalling write data to memory. Timestamp from begining of frame
    -- and whole frame is stored via this signal.
    ----------------------------------------------------------------------------
    write_raw_intent      <= '1' when 
                              ((rx_fsm = rxb_store_frame_format) or
                               (rx_fsm = rxb_store_identifier) or
                               (rx_fsm = rxb_store_beg_ts_low) or
                               (rx_fsm = rxb_store_beg_ts_high) or
                               ((rx_fsm = rxb_store_data) and store_data = '1'))
                                 else
                             '0';

    ----------------------------------------------------------------------------
    -- Signalling that extra timestamp is stored to memory. Timestamp from
    -- end of frame is stored extra.
    ----------------------------------------------------------------------------
    write_extra_ts        <= '1' when ((rx_fsm = rxb_store_end_ts_low) or
                                       (rx_fsm = rxb_store_end_ts_high))
                                 else
                             '0';


    ----------------------------------------------------------------------------
    -- Memory data which are written depend on state of the FSM
    ----------------------------------------------------------------------------
    with rx_fsm select data_selector <=
        "0000001"       when rxb_store_frame_format,
        "0000010"       when rxb_store_identifier,
        "0000100"       when rxb_store_beg_ts_low,
        "0001000"       when rxb_store_end_ts_low,
        "0010000"       when rxb_store_beg_ts_high,
        "0100000"       when rxb_store_end_ts_high,
        "1000000"       when rxb_store_data,
        "0000000"       when others;


    ----------------------------------------------------------------------------
    -- Signalling that storing extra timestamp has ended and frame can
    -- be committed.
    ----------------------------------------------------------------------------
    store_extra_ts_end <= '1' when (rx_fsm = rxb_store_end_ts_high)
                              else
                          '0';

    ----------------------------------------------------------------------------
    -- Storing extra write pointer is done early enough so that there is
    -- enough time to increment it to be pointing to first(higher) timestamp word
    ----------------------------------------------------------------------------
    store_extra_wr_ptr <= '1' when (rx_fsm = rxb_store_frame_format)
                              else
                          '0';

    ----------------------------------------------------------------------------
    -- Incrementing extra write pointer is done twice so that when lower
    -- timestamp word is beigin stored, extra write pointer is pointing to
    -- lower timestamp word address.
    -- Furthermore, Extra write pointer is incremented once more when lower
    -- timestamp word was stored, to point to higher timestamp word.
    ----------------------------------------------------------------------------
    inc_extra_wr_ptr <= '1' when (rx_fsm = rxb_store_identifier) or
                                 (rx_fsm = rxb_store_beg_ts_low) or
                                 (rx_fsm = rxb_store_end_ts_low)
                            else
                        '0';

    ----------------------------------------------------------------------------
    -- Internal overrun flag is re-started when 
    ----------------------------------------------------------------------------
    reset_overrun_flag  <= '1' when (rx_fsm = rxb_idle)
                               else
                           '0';


    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Assertions
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------

    ----------------------------------------------------------------------------
    -- Verification of commands applied on RX Buffer. Following must be always
    -- kept, otherwise CAN Core would corrupt storing protocol:
    --  1. "store_metadata" comes during "rx_fsm_idle".
    --  2. "rec_message_valid" and "rec_data" may come ONLY during "store_data"!
    --  3. "sof_pulse" should come only during "rx_fsm_idle".
    --  4. "store_metadata", "store_data", "rec_message_valid", "rec_abort"
    --      should be mutually exclusive!
    ----------------------------------------------------------------------------
    cmd_assert_proc : process(clk_sys)
        variable cmd_join  : std_logic_vector(3 downto 0);
    begin
        -- pragma translate_off
        if (rising_edge(clk_sys) and now /= 0 fs) then
            if (store_metadata = '1' and rx_fsm /= rxb_idle) then
            	-- LCOV_EXCL_START
                report "RX Buffer: Store metadata command did NOT come during " &
                       "'rx_buf_idle'!"severity error;
            	-- LCOV_EXCL_STOP
            end if;

            if ((rec_message_valid = '1' or store_data = '1') and
                 rx_fsm /= rxb_store_data)
            then
            	-- LCOV_EXCL_START
                report "RX Buffer: Store data or finish storing did NOT come " &
                        "during 'rec_data'" severity error; 
            	-- LCOV_EXCL_STOP            
            end if;

            if (sof_pulse = '1' and rx_fsm /= rxb_idle) then
                -- LCOV_EXCL_START
                report "RX Buffer: SOF pulse should come during 'rx_fsm_idle'"
                severity error;
            	-- LCOV_EXCL_STOP
            end if;

            cmd_join := store_metadata & store_data & rec_message_valid &
                        rec_abort; 
            if (cmd_join /= "0000" and cmd_join /= "0001" and cmd_join /= "0010"
                and cmd_join /= "0100" and cmd_join /= "1000")
            then
                -- LCOV_EXCL_START
                report "RX Buffer: One-hot coding on RX Buffer commands " &
                        "corrupted!" severity error;
                -- LCOV_EXCL_STOP
            end if;
        end if;
        -- pragma translate_on
    end process;

end architecture;
