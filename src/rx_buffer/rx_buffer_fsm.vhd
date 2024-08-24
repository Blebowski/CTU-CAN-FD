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
--  RX Buffer FSM.
--
-- Purpose:
--  Reacts on commands from CAN Core and controls storing of CAN frame
--  continusly to RX Buffer RAM.
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

entity rx_buffer_fsm is
    port (
        -------------------------------------------------------------------------------------------
        -- Clocks and reset
        -------------------------------------------------------------------------------------------
        clk_sys                 : in  std_logic;
        res_n                   : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Memory registers interface
        -------------------------------------------------------------------------------------------
        mr_mode_erfm            : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- Control signals from CAN Core (Filtered by Frame filters)
        -------------------------------------------------------------------------------------------
        -- Start Storing of Metadata to RX Buffer (first 4 words of frame)
        store_metadata_f        : in  std_logic;

        -- Store Data word to RX Buffer
        store_data_f            : in  std_logic;

        -- Received frame valid
        rec_valid_f             : in  std_logic;

        -- Abort storing of RX Frame to RX Buffer.
        rec_abort_f             : in  std_logic;

        -------------------------------------------------------------------------------------------
        -- FSM outputs
        -------------------------------------------------------------------------------------------
        -- Intent to write to RX Buffer RAM
        write_raw_intent        : out std_logic;

        -- Select Timestamp write pointer
        select_ts_wptr          : out std_logic;

        -- Intend to comit frame to RX Buffer
        commit_intent           : out std_logic;

        -- Data selector for selection of memory word to be stored in RX Buffer
        -- RAM (one hot coded)
        data_selector           : out std_logic_vector(4 downto 0);

        -- Load timestamp write pointer from regular write pointer
        store_ts_wr_ptr         : out std_logic;

        -- Increment timestamp write pointer by 1
        inc_ts_wr_ptr           : out std_logic;

        -- Error frame type
        rec_erf                 : out std_logic;

        -- Reset internal overrun flag
        reset_overrun_flag      : out std_logic
    );
end entity;

architecture rtl of rx_buffer_fsm is

    -- RX Buffer FSM
    signal curr_state   : t_rx_buf_state;
    signal next_state   : t_rx_buf_state;

    -- Clock enable for state register
    signal rx_fsm_ce    : std_logic;

begin

    -----------------------------------------------------------------------------------------------
    -- Next State process
    -----------------------------------------------------------------------------------------------
    next_state_proc : process(curr_state, store_metadata_f, rec_abort_f, rec_valid_f, mr_mode_erfm)
    begin
        next_state <= curr_state;

        case curr_state is

        -------------------------------------------------------------------------------------------
        -- Idle, waiting for "store_metada" to start storing first 4 words.
        -------------------------------------------------------------------------------------------
        when s_rxb_idle =>
            -- TODO: Rec_abort_f can be removed it is in fact unreachable. Rec_abort_f will be
            --       always one cycle after store_metadata_f!
            --       Need to add assertion on mutual exclusivity too!
            if (store_metadata_f = '1' and rec_abort_f = '0') then
                next_state <= s_rxb_store_frame_format;
            elsif (mr_mode_erfm = ERFM_ENABLED and rec_abort_f = '1') then
                next_state <= s_rxb_store_err_frame_format;
            end if;

        -------------------------------------------------------------------------------------------
        -- Storing FRAME_FORM_W. Proceed execpt if error ocurrs.
        -------------------------------------------------------------------------------------------
        when s_rxb_store_frame_format =>
            if (rec_abort_f = '1') then
                if (mr_mode_erfm = ERFM_ENABLED) then
                    next_state <= s_rxb_store_err_frame_format;
                else
                    next_state <= s_rxb_idle;
                end if;
            else
                next_state <= s_rxb_store_identifier;
            end if;

        -------------------------------------------------------------------------------------------
        -- Storing IDENTIFIER_W.
        -- Move to storing timestamp words. Note that if SW configured timestamp from end of the
        -- frame, we dont have it yet! We store invalid timestamp and later (if the frame is
        -- received OK), we repeat the writes with timestamp captured at the end of frame!
        -------------------------------------------------------------------------------------------
        when s_rxb_store_identifier =>
            next_state <= s_rxb_skip_ts_low;

        -------------------------------------------------------------------------------------------
        -- Skip through TIMESTAMP_L_W, store only zeroes.
        -------------------------------------------------------------------------------------------
        when s_rxb_skip_ts_low =>
            next_state <= s_rxb_skip_ts_high;

        -------------------------------------------------------------------------------------------
        -- Skip through TIMESTAMP_U_W, store only zeroes.
        -------------------------------------------------------------------------------------------
        when s_rxb_skip_ts_high =>
            next_state <= s_rxb_store_data;

        -------------------------------------------------------------------------------------------
        -- Store DATA_W. If error ocurrs, abort the storing. If storing is finished, go to idle or
        -- again timestamp storing depending on the timestamp option configuration. Note that now
        -- timestamp storing is realized via different states!
         -------------------------------------------------------------------------------------------
        when s_rxb_store_data =>
            if (rec_abort_f = '1') then
                if (mr_mode_erfm = ERFM_ENABLED) then
                    next_state <= s_rxb_store_err_frame_format;
                else
                    next_state <= s_rxb_idle;
                end if;

            elsif (rec_valid_f = '1') then
                next_state <= s_rxb_store_end_ts_low;
            end if;

        -------------------------------------------------------------------------------------------
        -- Store TIMESTAMP_L_W.
        -------------------------------------------------------------------------------------------
        when s_rxb_store_end_ts_low =>
            next_state <= s_rxb_store_end_ts_high;

        -------------------------------------------------------------------------------------------
        -- Store first TIMESTAMP_U_W.
        -------------------------------------------------------------------------------------------
        when s_rxb_store_end_ts_high =>
            next_state <= s_rxb_idle;

        -------------------------------------------------------------------------------------------
        -- Store FRAME_FORMAT_W of an Error frame
        -------------------------------------------------------------------------------------------
        when s_rxb_store_err_frame_format =>
            next_state <= s_rxb_store_err_identifier;

        -------------------------------------------------------------------------------------------
        -- Store IDENTIFIER_W of an Error frame
        -------------------------------------------------------------------------------------------
        when s_rxb_store_err_identifier =>
            next_state <= s_rxb_store_err_ts_low;

        -------------------------------------------------------------------------------------------
        -- Store TIMESTAMP_L_W of an Error frame
        -------------------------------------------------------------------------------------------
        when s_rxb_store_err_ts_low =>
            next_state <= s_rxb_store_err_ts_high;

        -------------------------------------------------------------------------------------------
        -- Store TIMESTAMP_U_W of an Error frame
        -------------------------------------------------------------------------------------------
        when s_rxb_store_err_ts_high =>
            next_state <= s_rxb_idle;

        end case;
    end process;


    -----------------------------------------------------------------------------------------------
    -- Current State process (outputs)
    -----------------------------------------------------------------------------------------------
    curr_state_proc : process(curr_state, store_data_f, rec_abort_f)
    begin
        write_raw_intent <= '0';
        select_ts_wptr <= '0';
        data_selector <= (others => '0');
        commit_intent <= '0';
        store_ts_wr_ptr <= '0';
        inc_ts_wr_ptr <= '0';
        reset_overrun_flag <= '0';
        rec_erf <= '0';

        case curr_state is
        when s_rxb_idle =>
            reset_overrun_flag <= '1';

        when s_rxb_store_frame_format =>
            if (rec_abort_f = '0') then
                write_raw_intent <= '1';
            end if;
            data_selector <= "00001";

        when s_rxb_store_identifier =>
            write_raw_intent <= '1';
            data_selector <= "00010";

        when s_rxb_skip_ts_low =>
            write_raw_intent <= '1';

            -- Here we capture value of Raw write pointer to Timestamp write pointer
            -- so that it can be later used for Timestamp storing.
            store_ts_wr_ptr <= '1';

        when s_rxb_skip_ts_high =>
            write_raw_intent <= '1';

        when s_rxb_store_data =>
            data_selector <= "00100";

            if (store_data_f = '1') then
                write_raw_intent <= '1';
            end if;

        when s_rxb_store_end_ts_low =>
            data_selector <= "01000";

            -- Raw write pointer points to the last data frame word
            -- when timestamp is stored at the end of frame. Need to
            -- use backed-up timestamp write pointer.
            select_ts_wptr <= '1';

            -- Timestamp write pointer is incremented once more when lower
            -- timestamp word was stored, to point to higher timestamp word.
            inc_ts_wr_ptr <= '1';

        when s_rxb_store_end_ts_high =>
            data_selector <= "10000";

            -- Raw write pointer points to the last data frame word
            -- when timestamp is stored at the end of frame. Need to
            -- use backed-up timestamp write pointer.
            select_ts_wptr <= '1';

            commit_intent <= '1';

        when s_rxb_store_err_frame_format =>
            write_raw_intent <= '1';
            data_selector <= "00001";
            rec_erf <= '1';

        when s_rxb_store_err_identifier =>
            write_raw_intent <= '1';
            data_selector <= "00010";

        when s_rxb_store_err_ts_low =>
            write_raw_intent <= '1';
            data_selector <= "01000";

        when s_rxb_store_err_ts_high =>
            write_raw_intent <= '1';
            data_selector <= "10000";

            commit_intent <= '1';
        end case;

    end process;


    -----------------------------------------------------------------------------------------------
    -- State register process
    -----------------------------------------------------------------------------------------------
    state_reg_proc : process(clk_sys, res_n)
    begin
        if (res_n = '0') then
            curr_state <= s_rxb_idle;
        elsif (rising_edge(clk_sys)) then
            if (rx_fsm_ce = '1') then
                curr_state <= next_state;
            end if;
        end if;
    end process;

    -- Clock enable for State reg
    rx_fsm_ce <= '1' when (next_state /= curr_state) else
                 '0';

    -- <RELEASE_OFF>
    -- pragma translate_off

    -----------------------------------------------------------------------------------------------
    -- Assertions
    -----------------------------------------------------------------------------------------------

    assertions_block : block
        -- Joined commands
        signal cmd_join     : std_logic_vector(3 downto 0);
    begin
        -- psl default clock is rising_edge(clk_sys);

        -- Joined commands, for assertions only
        cmd_join <= store_metadata_f & store_data_f & rec_valid_f & rec_abort_f;

        -- psl store_metadata_in_idle_asrt : assert never
        --  (store_metadata_f = '1' and (curr_state /= s_rxb_idle))
        -- report "RX Buffer: Store metadata command did NOT come when RX buffer is idle!";

        -- psl commit_or_store_data_asrt : assert never
        --  ((rec_valid_f = '1' or store_data_f = '1') and curr_state /= s_rxb_store_data)
        -- report "RX Buffer: Store data or frame commit commands did not come when RX Buffer is receiving data!";

        -- psl rxb_not_storing_err_frm_when_next_occurs_asrt : assert never
        --  (curr_state = s_rxb_store_err_frame_format or
        --   curr_state = s_rxb_store_err_identifier or
        --   curr_state = s_rxb_store_err_ts_low or
        --   curr_state = s_rxb_store_err_ts_high) and (rec_abort_f = '1')
        -- report "Error frame was not yet fully stored when another one occured.";

        process (cmd_join)
        begin
            if (now > 0 ps) then
                if (cmd_join /= "0000" and cmd_join /= "0001" and
                    cmd_join /= "0010" and cmd_join /= "0100" and
                    cmd_join /= "1000")
                then
                    report "RX Buffer: RX Buffer commands should be one-hot encoded!"
                    severity error;
                end if;
            end if;
        end process;

        -- psl rx_no_abort_after_metadata_cov : assert never
        --  (rec_abort_f = '1') and
        --  (curr_state = s_rxb_store_identifier or curr_state = s_rxb_skip_ts_low or
        --   curr_state = s_rxb_skip_ts_high or curr_state = s_rxb_store_end_ts_low or
        --   curr_state = s_rxb_store_end_ts_high)
        --  report "RX Buffer abort not supported storing of Identifier and Timestamp";

        -- psl store_metadata_and_rec_abort_back_to_back_cov : cover
        --  {mr_mode_erfm = ERFM_ENABLED and rec_abort_f = '1' and curr_state = s_rxb_store_frame_format}
        --  report "rec_abort_f = 1 and curr_state = s_rxb_store_frame_format";

        -- psl rec_abort_in_idle_cov : cover
        --  {mr_mode_erfm = ERFM_ENABLED and rec_abort_f = '1' and curr_state = s_rxb_idle}
        --  report "rec_abort_f = 1 and curr_state = s_rxb_idle";

        -- psl rec_abort_in_store_data_cov : cover
        --  {mr_mode_erfm = ERFM_ENABLED and rec_abort_f = '1' and curr_state = s_rxb_store_data}
        --  report "rec_abort_f = 1 and curr_state = s_rxb_store_data";

    end block assertions_block;

    -- pragma translate_on
    -- <RELEASE_ON>

end architecture;