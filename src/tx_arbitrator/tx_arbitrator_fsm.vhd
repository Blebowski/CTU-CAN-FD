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
--  TX Arbitrator FSM. Reacts on change of selected TXT Buffer, drives loading
--  Timestamp (Timestamp words), metadata (Frame format word) from TXT Buffer.
--  Reacts on LOCK command from CAN Core which locks TXT Buffer for
--  transmission.
--  Each FSM state lasts two clock cycles (wait state is inserted), to give
--  time to TXT Buffer RAM for data read.
--------------------------------------------------------------------------------
-- Revision History:
--    10.11.2018   Created file
--------------------------------------------------------------------------------

Library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.ALL;
use ieee.math_real.ALL;

Library work;
use work.id_transfer.all;
use work.can_constants.all;
use work.can_components.all;
use work.can_types.all;
use work.cmn_lib.all;
use work.drv_stat_pkg.all;
use work.endian_swap.all;
use work.reduce_lib.all;

use work.CAN_FD_register_map.all;
use work.CAN_FD_frame_format.all;

entity tx_arbitrator_fsm is
    port( 
        -----------------------------------------------------------------------
        -- Clock and Asynchronous reset
        -----------------------------------------------------------------------
        -- System clock
        clk_sys                :in  std_logic;
        
        -- Asynchronous reset
        res_n                  :in  std_logic;
        
        -----------------------------------------------------------------------
        -- Priority decoder interface
        -----------------------------------------------------------------------
        -- TXT Buffer is valid and selected for transmission
        select_buf_avail       :in  std_logic;
        
        -- Priority decoder output has changed. TXT Arbitrator FSM has to restart
        -- selection process.
        select_index_changed   :in  std_logic;
        
        -----------------------------------------------------------------------
        -- Timestamp comparison interface
        -----------------------------------------------------------------------
        timestamp_valid        :in  std_logic;
        
        -----------------------------------------------------------------------
        -- CAN Core Interface
        -----------------------------------------------------------------------
        -- HW Commands from CAN Core for manipulation with TXT Buffers 
        txt_hw_cmd             :in txt_hw_cmd_type;  
        
        ---------------------------------------------------------------------------
        -- TX Arbitrator FSM outputs
        ---------------------------------------------------------------------------
        -- Load Timestamp lower word to metadata pointer
        load_ts_lw_addr        :out std_logic;
        
        -- Load Timestamp upper word to metadata pointer
        load_ts_uw_addr        :out std_logic;
        
        -- Load Frame format word to metadata pointer
        load_ffmt_w_addr       :out std_logic;
        
        -- Store timestamp lower word
        store_ts_l_w           :out std_logic;
        
        -- Store metadata (Frame format word) on the output of TX Arbitrator
        store_md_w             :out std_logic;
        
        -- Signals that TX Arbitrator is locked (CAN Core is transmitting from TXT
        -- Buffer)
        tx_arb_locked          :out std_logic;
        
        -- Store last locked TXT Buffer index
        store_last_txtb_index  :out std_logic;
        
        -- Set valid selected buffer on TX Arbitrator output.
        frame_valid_com_set    :out std_logic;    
        
        -- Clear valid selected buffer on TX Arbitrator output.
        frame_valid_com_clear  :out std_logic 
  );
end entity;

architecture rtl of tx_arbitrator_fsm is

  -- TX Arbitrator FSM state
  signal tx_arb_fsm               : t_tx_arb_state;  
 
  signal fsm_wait_state           : boolean;

begin
  
  ------------------------------------------------------------------------------
  -- State machine driving selection of highest priority buffer and load of the
  -- metadata and identifier words on parallel outputs.
  ------------------------------------------------------------------------------
  tx_arb_fsm_proc : process(clk_sys, res_n)
  begin
    if (res_n = ACT_RESET) then
        tx_arb_fsm            <= s_arb_sel_low_ts;
      
        fsm_wait_state        <= true;
        
    elsif rising_edge(clk_sys) then
      
        -- Keeping signals values to avoid latch inference
        tx_arb_fsm                <= tx_arb_fsm;
        fsm_wait_state            <= fsm_wait_state;

        ------------------------------------------------------------------------
        -- Finishing the transmission = unlocking the buffer
        ------------------------------------------------------------------------      
        if (tx_arb_fsm = s_arb_locked) then
            if (txt_hw_cmd.unlock = '1') then
                tx_arb_fsm            <= s_arb_sel_low_ts;
                fsm_wait_state        <= true;
            end if;

        ------------------------------------------------------------------------
        -- Locking the buffer
        ------------------------------------------------------------------------
        elsif (txt_hw_cmd.lock     = '1') then
            tx_arb_fsm              <= s_arb_locked;
            
        ------------------------------------------------------------------------
        -- Keep the arbitrator in selection of the lowest word as
        -- long as there is no buffer with valid frame.
        -- If Selected buffer changes, restart the selection.
        ------------------------------------------------------------------------
        elsif ((select_buf_avail = '0')) then
            tx_arb_fsm              <= s_arb_sel_low_ts;
            fsm_wait_state          <= true;
            
        ------------------------------------------------------------------------
        -- Selected buffer index on the output of priority decoder has changed
        -- during selection. Restart the selection!
        ------------------------------------------------------------------------
        elsif (select_index_changed = '1') then
            tx_arb_fsm              <= s_arb_sel_low_ts;
            fsm_wait_state          <= true;

        else
      
            -------------------------------------------------------------------
            -- Clear the wait state if set.
            -------------------------------------------------------------------
            if (fsm_wait_state) then
                fsm_wait_state <= false;
            end if;

            case tx_arb_fsm is   

            --------------------------------------------------------------------
            -- Polling on Low timestamp of the highest prority TXT buffer
            --------------------------------------------------------------------
            when s_arb_sel_low_ts =>
                if (not fsm_wait_state) then
                    tx_arb_fsm         <= s_arb_sel_upp_ts;
                    fsm_wait_state     <= true;
                end if;        

            --------------------------------------------------------------------
            -- Compare the timestamps,
            -- now output of TXT Buffers give the upper timestamp
            -- Lower timestamp is stored from previous state.
            --------------------------------------------------------------------  
            when s_arb_sel_upp_ts =>
                if (not fsm_wait_state) then
                    if (timestamp_valid = '1') then
                        tx_arb_fsm         <= s_arb_sel_ffw;
                        fsm_wait_state     <= true;
                    end if;
                end if;

            --------------------------------------------------------------------
            -- Store the Frame format info to the output. We can do this
            -- directly, since it is all done in one clock cycle!
            -- Buffer input did not change during the whole selection, we 
            -- can store its index to the output and us it for access from 
            -- CAN Core.
            --------------------------------------------------------------------  
            when s_arb_sel_ffw =>
                if (not fsm_wait_state) then
                    tx_arb_fsm               <= s_arb_sel_low_ts;
                    fsm_wait_state           <= true;
                end if;

            --------------------------------------------------------------------
            -- By default only arb_locked is here, but it is checked before
            -- next state decoder! So it is OK that this "others" will never
            -- be covered here!
            --------------------------------------------------------------------  
            when others =>
            end case;
        
      end if;
      
    end if;
  end process;


  ------------------------------------------------------------------------------
  -- TX Arbitrator FSM outputs
  ------------------------------------------------------------------------------
  tx_arb_fsm_out_proc : process(tx_arb_fsm, fsm_wait_state, timestamp_valid,
                                select_index_changed, select_buf_avail,
                                txt_hw_cmd)
  begin
    
    -- By default all outputs are inactive
    load_ts_lw_addr        <= '0';
    load_ts_uw_addr        <= '0';
    load_ffmt_w_addr       <= '0';

    store_ts_l_w           <= '0';
    store_md_w             <= '0';
    tx_arb_locked          <= '0';
    frame_valid_com_set    <= '0';
    frame_valid_com_clear  <= '0';
    store_last_txtb_index  <= '0';

    ------------------------------------------------------------------------
    -- Finishing the transmission = unlocking the Buffer -> Signal
    -- start of new buffer selection, first Timestamp Low word is loaded.
    ------------------------------------------------------------------------      
    if (tx_arb_fsm = s_arb_locked) then
        if (txt_hw_cmd.unlock = '1') then
            load_ts_lw_addr <= '1';
        else
            tx_arb_locked   <= '1';
        end if;

    ------------------------------------------------------------------------
    -- Locking the buffer, only store the last TXT Buffer index!
    ------------------------------------------------------------------------      
    elsif (txt_hw_cmd.lock = '1') then
        store_last_txtb_index   <= '1';

    ------------------------------------------------------------------------
    -- Restart the selection if one of following occurs:
    --  1. Selected Buffer changed.
    --  2. There is not buffer marked as ready -> Hold in not selected!
    ------------------------------------------------------------------------
    elsif (select_buf_avail = '0' or select_index_changed = '1') then
        load_ts_lw_addr         <= '1';
        frame_valid_com_clear   <= '1';

    else
  
        case tx_arb_fsm is   

        --------------------------------------------------------------------
        -- Polling on Low timestamp of the highest prority TXT buffer
        --------------------------------------------------------------------
        when s_arb_sel_low_ts =>
            if (fsm_wait_state = false) then
                load_ts_uw_addr    <= '1';
                store_ts_l_w       <= '1';
            end if;        

        --------------------------------------------------------------------
        -- Compare the timestamps,
        -- now output of TXT Buffers give the upper timestamp
        -- Lower timestamp is stored from previous state.
        --------------------------------------------------------------------  
        when s_arb_sel_upp_ts =>
            if (fsm_wait_state = false) then
                if (timestamp_valid = '1') then
                    load_ffmt_w_addr   <= '1';
                end if;
            end if;

        --------------------------------------------------------------------
        -- Store the Frame format info to the output. We can do this
        -- directly, since it is all done in one clock cycle!
        -- Buffer input did not change during the whole selection, we 
        -- can store its index to the output and us it for access from 
        -- CAN Core.
        --------------------------------------------------------------------  
        when s_arb_sel_ffw =>
            if (fsm_wait_state = false) then
                load_ts_lw_addr        <= '1';
                frame_valid_com_set    <= '1';
                store_md_w             <= '1';
            end if;

        --------------------------------------------------------------------
        -- By default only arb_locked is here, but it is checked before
        -- next state decoder!
        --------------------------------------------------------------------  
        when others =>

        end case;
      end if;
  end process;

  ------------------------------------------------------------------------------
  -- Functional coverage
  ------------------------------------------------------------------------------
  -- psl default clock is rising_edge(clk_sys);
  --
  -- psl txt_buf_wait_till_timestamp_cov : cover
  --    (tx_arb_fsm = arb_sel_upp_ts and fsm_wait_state = false and
  --     timestamp_valid = '0')
  --    report "TXT Buffer waiting for Timestamp to reach TX Time";

end architecture;